#!/usr/bin/env python3
import sys
import os
import threading
import math
import fractions
import tempfile
import re

import collections
import collections.abc
collections.Mapping = collections.abc.Mapping
collections.Set = collections.abc.Set
collections.Iterable = collections.abc.Iterable
fractions.gcd = math.gcd

import numpy as np

setattr(np, 'int', int)
setattr(np, 'float', float)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
import tf2_ros
from ament_index_python.packages import get_package_share_directory


from urdfpy import URDF
import trimesh

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QPushButton, QLabel
)
from PyQt6.QtCore    import pyqtSignal, Qt
from PyQt6.QtGui     import QMatrix4x4, QVector4D, QPalette, QColor, QFont
import pyqtgraph.opengl as gl

from astroviz.utils.window_style import DarkStyle, LightStyle

def quaternion_to_matrix(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
    ], dtype=np.float32)

class RobotStateViewer(QMainWindow):
    update_signal = pyqtSignal()

    def __init__(self, node: Node, root_frame: str = 'pelvis'):
        super().__init__()
        self.node = node
        self.root_frame = root_frame
        self.render_tf = True
        self.render_urdf = True
        self.setWindowTitle('Robot State Viewer')
        self.resize(800, 600)

        self.node.create_subscription(
            String, 
            '/robot_description', 
            self.robot_description_callback, 
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        w = QWidget()
        self.setCentralWidget(w)
        layout = QVBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)

        self.gl_view = gl.GLViewWidget()
        self.gl_view.opts['distance'] = 1.0
        self.gl_view.setBackgroundColor((0.2, 0.2, 0.2, 1))
        layout.addWidget(self.gl_view)


        self.btn_tf = QPushButton('TF', self.gl_view)
        self.btn_tf.setCheckable(True)
        self.btn_tf.setChecked(True)
        self.btn_tf.setFixedWidth(80)
        self.btn_tf.clicked.connect(self.toggle_tf)
        self.btn_urdf = QPushButton('URDF', self.gl_view)
        self.btn_urdf.setCheckable(True)
        self.btn_urdf.setChecked(True)
        self.btn_urdf.setFixedWidth(80)
        self.btn_urdf.clicked.connect(self.toggle_urdf)

        self.loading_label = QLabel("Loading model...", self.gl_view)
        self.loading_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.loading_label.setFont(QFont('Arial', 18, QFont.Weight.Bold))

        palette = self.loading_label.palette()
        palette.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
        self.loading_label.setPalette(palette)
        self.loading_label.setStyleSheet("background-color: rgba(0, 0, 0, 120);")
        self.loading_label.setGeometry(self.gl_view.rect())
        self.loading_label.show()

        self._position_overlays()

        grid = gl.GLGridItem()
        grid.scale(0.1, 0.1, 0.1)
        self.gl_view.addItem(grid)
        self.scatter = gl.GLScatterPlotItem()
        self.x_axes = gl.GLLinePlotItem(color=(1, 0, 0, 1), width=2, mode='lines')
        self.y_axes = gl.GLLinePlotItem(color=(0, 1, 0, 1), width=2, mode='lines')
        self.z_axes = gl.GLLinePlotItem(color=(0, 0, 1, 1), width=2, mode='lines')
        for item in (self.scatter, self.x_axes, self.y_axes, self.z_axes):
            self.gl_view.addItem(item)

        self.update_signal.connect(self._update_view, Qt.ConnectionType.QueuedConnection)

        self.node.create_subscription(TFMessage, '/tf', self._tf_callback, qos_profile=10)
        static_qos = QoSProfile(depth=10)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.node.create_subscription(TFMessage, '/tf_static', self._tf_callback, qos_profile=static_qos)

        self.mesh_items = []
        self.gl_view.installEventFilter(self)

    def eventFilter(self, source, event):
        if source is self.gl_view and event.type() == event.Type.Resize:
            self.loading_label.setGeometry(self.gl_view.rect())
        return super().eventFilter(source, event)


    def fix_urdf_path(self, urdf_string):
            def replace_package(match):
                package_path = get_package_share_directory(match.group(1))
                return package_path + '/' + match.group(2)
            pattern = r'package://([^/]+)/(.+?)(?=["\'])'
            return re.sub(pattern, replace_package, urdf_string)
    
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._position_overlays()
        self.loading_label.setGeometry(self.gl_view.rect())


    def robot_description_callback(self, msg: String):
        xml  = msg.data
        if not xml:
            return
        xml_fixed = self.fix_urdf_path(xml)

        self.loading_label.show()
        threading.Thread(target=self.load_urdf, args=(xml_fixed,), daemon=True).start()

    def load_urdf(self, urdf_string):
        tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
        tmp.write(urdf_string.encode()); tmp.flush(); tmp.close()

        urdf = URDF.load(tmp.name)
        mesh_items_temp = []
        for link in urdf.links:
            for visual in link.visuals:
                fn = visual.geometry.mesh.filename
                mesh_path = fn
                scene_or_mesh = trimesh.load_mesh(mesh_path, process=False)
                tm = scene_or_mesh.dump() if hasattr(scene_or_mesh, 'dump') else scene_or_mesh
                verts = tm.vertices.view(np.ndarray)
                faces = tm.faces.view(np.ndarray)
                normals = tm.vertex_normals.view(np.ndarray) if tm.vertex_normals is not None else None
                if visual.material and visual.material.color is not None:
                    rgba = visual.material.color
                    face_color = (rgba[0], rgba[1], rgba[2], rgba[3])
                else:
                    face_color = (0.7, 0.7, 0.7, 1.0)

                item = gl.GLMeshItem(
                    vertexes=verts,
                    faces=faces,
                    normals=normals,
                    smooth=True,
                    shader='shaded',
                    drawFaces=True,
                    drawEdges=False,
                    faceColor=face_color
                )
                item.setGLOptions('opaque')

                if isinstance(visual.origin, np.ndarray) and visual.origin.shape == (4, 4):
                    T_lv = visual.origin.astype(np.float32)
                else:
                    T = np.eye(4, dtype=np.float32)
                    if hasattr(visual.origin, 'position') and hasattr(visual.origin, 'rotation'):
                        pos = visual.origin.position
                        rot = quaternion_to_matrix(visual.origin.rotation)
                        T[:3, :3] = rot
                        T[:3, 3] = [pos.x, pos.y, pos.z]
                    T_lv = T

                mesh_items_temp.append((item, link.name, T_lv))

        self.mesh_items = mesh_items_temp

        self.update_signal.connect(self.finish_loading, Qt.ConnectionType.QueuedConnection)
        self.update_signal.emit()

    def finish_loading(self):
        for item, _, _ in self.mesh_items:
            self.gl_view.addItem(item)
        self.loading_label.hide()
        self.update_signal.disconnect(self.finish_loading)

    def showEvent(self, event):
        super().showEvent(event)
        self._position_overlays()

    def _position_overlays(self):
        margin = 5
        x = margin
        y = margin
        self.btn_tf.move(x, y)
        x += self.btn_tf.width() + margin
        self.btn_urdf.move(x, y)

    def toggle_tf(self):
        self.render_tf = self.btn_tf.isChecked()
        visible = self.render_tf
        self.scatter.setVisible(visible)
        self.x_axes.setVisible(visible)
        self.y_axes.setVisible(visible)
        self.z_axes.setVisible(visible)

    def toggle_urdf(self):
        self.render_urdf = self.btn_urdf.isChecked()
        for item, _, _ in self.mesh_items:
            item.setVisible(self.render_urdf)

    def _tf_callback(self, msg: TFMessage):
        if msg.transforms:
            self.update_signal.emit()

    def _update_view(self):
        frames = []
        lines = self.tf_buffer.all_frames_as_string().splitlines()
        frames = [L.split()[1] for L in lines if L.startswith('Frame ')]

        for item, link_name, T_lv in self.mesh_items:
            if not self.render_urdf:
                break
            try:
                tf = self.tf_buffer.lookup_transform(self.root_frame, link_name, rclpy.time.Time())
            except Exception:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            T_tf = np.eye(4, dtype=np.float32)
            T_tf[:3, :3] = quaternion_to_matrix(q)
            T_tf[:3, 3] = [t.x, t.y, t.z]
            T = T_tf @ T_lv
            mat = QMatrix4x4()
            for i in range(4):
                mat.setRow(i, QVector4D(*T[i, :]))
            item.setTransform(mat)

        if not self.render_tf:
            return

        mats = []
        for f in frames:
            if f == self.root_frame:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(self.root_frame, f, rclpy.time.Time())
            except Exception:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            M = np.eye(4, dtype=np.float32)
            M[:3, :3] = quaternion_to_matrix(q)
            M[:3, 3] = [t.x, t.y, t.z]
            mats.append(M)
        if not mats:
            return

        pts = np.vstack([M[:3, 3] for M in mats])
        self.scatter.setData(pos=pts, size=5, color=(1, 1, 0, 0.8))

        N, L = len(mats), 0.1
        x = np.zeros((2 * N, 3), dtype=np.float32)
        y = np.zeros_like(x)
        z = np.zeros_like(x)
        for i, M in enumerate(mats):
            o, R = M[:3, 3], M[:3, :3]
            x[2*i] = o;        x[2*i+1] = o + R @ np.array([L, 0, 0], dtype=np.float32)
            y[2*i] = o;        y[2*i+1] = o + R @ np.array([0, L, 0], dtype=np.float32)
            z[2*i] = o;        z[2*i+1] = o + R @ np.array([0, 0, L], dtype=np.float32)
        self.x_axes.setData(pos=x)
        self.y_axes.setData(pos=y)
        self.z_axes.setData(pos=z)


def main():
    rclpy.init()
    node = rclpy.create_node('tf_viewer')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QApplication(sys.argv)
    DarkStyle(app)
    viewer = RobotStateViewer(node)
    viewer.show()
    app.exec()

    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

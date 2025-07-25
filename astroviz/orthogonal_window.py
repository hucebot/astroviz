#!/usr/bin/env python3
import sys
import os
import threading
import math
import fractions
import tempfile
import re
import json

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
    QVBoxLayout, QPushButton, QLabel, QComboBox
)
from PyQt6.QtCore    import pyqtSignal, Qt, QTimer
from PyQt6.QtGui     import QMatrix4x4, QVector4D, QPalette, QColor, QFont, QIcon
import pyqtgraph.opengl as gl

from ament_index_python.packages import get_package_share_directory


from astroviz.utils.window_style import DarkStyle
from astroviz.utils._find import _find_pkg, _find_src_config

_src_config = _find_src_config()
if _src_config:
    _CONFIG_DIR = _src_config
else:
    _CONFIG_DIR = os.path.join(
        get_package_share_directory('astroviz'), 'config'
    )


_pkg = _find_pkg()
if _pkg:
    _PKG_DIR = _pkg
else:
    _PKG_DIR = get_package_share_directory('astroviz')

os.makedirs(_CONFIG_DIR, exist_ok=True)

CONFIG_PATH = os.path.join(_CONFIG_DIR, 'dashboard_config.json')
MESHES_PATH = os.path.join(_CONFIG_DIR, 'world_display.json')
ICONS_DIR  = os.path.join(_PKG_DIR, 'icons')
MESHES_DIR = os.path.join(_PKG_DIR, 'meshes')



def quaternion_to_matrix(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
    ], dtype=np.float32)

class OrthogonalViewer(QMainWindow):
    update_signal = pyqtSignal()

    def __init__(self, node: Node, root_frame: str = 'pelvis'):
        super().__init__()
        self.node = node
        self.root_frame = root_frame
        self.render_tf = True
        self.render_urdf = True
        self.setWindowTitle('Orthogonal Viewer')
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, 'astroviz_icon.png')))
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
        self.btn_world = QPushButton('WORLD', self.gl_view)
        self.btn_world.setCheckable(True)
        self.btn_world.setChecked(True)
        self.btn_world.setFixedWidth(80)
        self.btn_world.clicked.connect(self.toggle_world)
        self.btn_grid = QPushButton('GRID', self.gl_view)
        self.btn_grid.setCheckable(True)
        self.btn_grid.setChecked(True)
        self.btn_grid.setFixedWidth(80)
        self.btn_grid.clicked.connect(self.toggle_grid)


        self.combo = QComboBox(self.gl_view)
        self.combo.setFixedWidth(180)
        self.combo.raise_()
        self.combo.currentTextChanged.connect(self.change_topic)

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

        self.grid_item = gl.GLGridItem()
        self.grid_item.scale(0.1, 0.1, 0.1)
        self.gl_view.addItem(self.grid_item)

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

        self.world_items = []
        self.render_world = True

        self.frame_timer = QTimer(self)
        self.frame_timer.timeout.connect(self._populate_frames)
        self.frame_timer.start(1000)

        self.gl_view.setMouseTracking(True)
        self._mouse_right_pressed = False
        self._last_mouse_pos = None

        self.load_world_objects()

    def load_world_objects(self):
        if not os.path.exists(MESHES_PATH):
            self.node.get_logger().warn(f"No meshes file found at {MESHES_PATH}.")
            return

        with open(MESHES_PATH, 'r') as f:
            try:
                object_list = json.load(f)
            except json.JSONDecodeError:
                self.node.get_logger().error("Error parsing world_display.json")
                return

        for obj in object_list:
            mesh_path = os.path.join(MESHES_DIR, obj['mesh'])
            if not os.path.exists(mesh_path):
                self.node.get_logger().warn(f"Mesh not found: {mesh_path}")
                continue

            try:
                loaded = trimesh.load(mesh_path, process=False)

                if isinstance(loaded, list):
                    mesh = trimesh.util.concatenate(loaded)
                elif hasattr(loaded, 'geometry'):
                    mesh = trimesh.util.concatenate(loaded.geometry.values())
                else:
                    mesh = loaded

            except Exception as e:
                self.node.get_logger().error(f"Error loading mesh {mesh_path}: {str(e)}")
                continue

            scale = obj.get("scale", 1.0)

            verts = mesh.vertices.view(np.ndarray) * scale
            faces = mesh.faces.view(np.ndarray)
            normals = mesh.vertex_normals.view(np.ndarray) if mesh.vertex_normals is not None else None

            face_color = (0.7, 0.7, 0.7, 1.0)
            gl_item = gl.GLMeshItem(
                vertexes=verts,
                faces=faces,
                normals=normals,
                smooth=True,
                shader='shaded',
                drawFaces=True,
                drawEdges=False,
                faceColor=face_color
            )
            gl_item.setGLOptions('opaque')

            pos = np.array(obj["position"], dtype=np.float32)
            q = obj["orientation"]
            T = np.eye(4, dtype=np.float32)
            T[:3, :3] = quaternion_to_matrix(type("Q", (), dict(x=q[0], y=q[1], z=q[2], w=q[3]))())
            T[:3, 3] = pos

            mat = QMatrix4x4()
            for i in range(4):
                mat.setRow(i, QVector4D(*T[i, :]))
            gl_item.setTransform(mat)

            self.gl_view.addItem(gl_item)
            self.world_items.append((gl_item, obj["name"], obj["frame"], T))

    def _populate_frames(self):
        try:
            lines = self.tf_buffer.all_frames_as_string().splitlines()
            frames = [L.split()[1] for L in lines if L.startswith('Frame ')]
        except Exception:
            frames = []
        current = self.combo.currentText()
        items = ['---'] + frames
        if [self.combo.itemText(i) for i in range(self.combo.count())] != items:
            self.combo.blockSignals(True)
            self.combo.clear()
            self.combo.addItems(items)
            if current in items:
                self.combo.setCurrentText(current)
            else:
                self.combo.setCurrentIndex(0)
                self.change_topic('---')
            self.combo.blockSignals(False)

    def change_topic(self, frame_name: str):
        if frame_name == '---':
            return
        self.root_frame = frame_name

    def eventFilter(self, source, event):
        if source is self.gl_view:
            if event.type() == event.Type.Resize:
                self.loading_label.setGeometry(self.gl_view.rect())
            elif event.type() == event.Type.MouseButtonPress and event.button() == Qt.MouseButton.RightButton:
                self._mouse_right_pressed = True
                self._last_mouse_pos = event.position().toPoint()
                return True
            elif event.type() == event.Type.MouseButtonRelease and event.button() == Qt.MouseButton.RightButton:
                self._mouse_right_pressed = False
                self._last_mouse_pos = None
                return True
            elif event.type() == event.Type.MouseMove and self._mouse_right_pressed and self._last_mouse_pos:
                delta = event.position().toPoint() - self._last_mouse_pos
                dz = -0.01 * delta.y()
                center = self.gl_view.opts['center']
                new_center = QVector4D(center.x(), center.y(), center.z() + dz, 1.0)
                self.gl_view.opts['center'] = new_center.toVector3D()
                self._last_mouse_pos = event.position().toPoint()
                return True
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
        x += self.btn_urdf.width() + margin
        self.btn_world.move(x, y)
        x += self.btn_world.width() + margin
        self.btn_grid.move(x, y)
        x += self.btn_grid.width() + margin
        self.combo.move(x, y)



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

    def toggle_world(self):
        self.render_world = self.btn_world.isChecked()
        for item, _, _, _ in self.world_items:
            item.setVisible(self.render_world)

    def toggle_grid(self):
        self.grid_item.setVisible(self.btn_grid.isChecked())

    def _tf_callback(self, msg: TFMessage):
        if msg.transforms:
            self.update_signal.emit()

    def _update_view(self):
        frames = []
        lines = self.tf_buffer.all_frames_as_string().splitlines()
        frames = [L.split()[1] for L in lines if L.startswith('Frame ')]

        if self.render_urdf:
            for item, link_name, T_lv in self.mesh_items:
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

        if self.render_tf:
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
            if mats:
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

        for item, name, frame, T_local in self.world_items:
            try:
                tf = self.tf_buffer.lookup_transform(self.root_frame, frame, rclpy.time.Time())
            except Exception:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            T_tf = np.eye(4, dtype=np.float32)
            T_tf[:3, :3] = quaternion_to_matrix(q)
            T_tf[:3, 3] = [t.x, t.y, t.z]
            T = T_tf @ T_local
            mat = QMatrix4x4()
            for i in range(4):
                mat.setRow(i, QVector4D(*T[i, :]))
            item.setTransform(mat)

def main():
    rclpy.init()
    node = rclpy.create_node('tf_viewer')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QApplication(sys.argv)
    DarkStyle(app)
    viewer = OrthogonalViewer(node)
    viewer.show()
    app.exec()

    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

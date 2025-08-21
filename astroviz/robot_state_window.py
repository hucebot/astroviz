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

import pyqtgraph as pg
import pyqtgraph.opengl as gl

from urdfpy import URDF
import trimesh

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QPushButton, QLabel, QComboBox
)
from PyQt6.QtCore    import pyqtSignal, Qt, QTimer, QTime
from PyQt6.QtGui     import QMatrix4x4, QVector4D, QPalette, QColor, QFont, QIcon

from astroviz.utils.window_style import DarkStyle
from astroviz.common._find import _find_pkg, _find_src_config

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
ICONS_DIR  = os.path.join(_PKG_DIR, 'icons')

def quaternion_to_matrix(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
    ], dtype=np.float32)

class RobotStateViewer(QMainWindow):
    update_signal = pyqtSignal()

    def __init__(self, node: Node, root_frame: str = 'odom'):
        super().__init__()
        self.node = node
        self.root_frame = root_frame
        self.render_tf = True
        self.render_urdf = True
        self._left_pressed = False
        self._left_dragged = False
        self._press_pos = None

        self._cached_frames = []
        self._last_link_tf = {}
        self._axes_last_update_ms = 0
        self._axes_update_interval_ms = 100

        self.setWindowTitle('Robot Viewer')
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, 'astroviz_icon.png')))
        self.resize(800, 600)

        pg.setConfigOptions(antialias=False)
        try:
            gl.setConfigOptions(antialias=False)
        except Exception:
            pass

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
        self.gl_view.setBackgroundColor('#090c28')
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

        self.btn_grid = QPushButton('GRID', self.gl_view)
        self.btn_grid.setCheckable(True)
        self.btn_grid.setChecked(True)
        self.btn_grid.setFixedWidth(80)
        self.btn_grid.clicked.connect(self.toggle_grid)

        self.combo = QComboBox(self.gl_view)
        self.combo.setFixedWidth(180)
        self.combo.raise_()
        self.combo.currentTextChanged.connect(self.change_fixed_frame)

        self.loading_label = QLabel("Loading model...", self.gl_view)
        self.loading_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.loading_label.setFont(QFont('Arial', 18, QFont.Weight.Bold))
        palette = self.loading_label.palette()
        palette.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
        self.loading_label.setPalette(palette)
        self.loading_label.setStyleSheet("background-color: rgba(0, 0, 0, 120);")
        self.loading_label.setGeometry(self.gl_view.rect())
        self.loading_label.hide()

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

        self.frame_timer = QTimer(self)
        self.frame_timer.timeout.connect(self._populate_frames)
        self.frame_timer.start(1000)

        self.render_timer = QTimer(self)
        self.render_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.render_timer.timeout.connect(self._update_view)
        self.render_timer.start(16)

        self.gl_view.setMouseTracking(True)
        self._mouse_right_pressed = False
        self._last_mouse_pos = None

    # ===================== UI HELPERS =====================
    def change_fixed_frame(self, frame_name: str):
        if frame_name == 'Fixed Frame':
            return
        self.root_frame = frame_name

    def eventFilter(self, source, event):
        if source is self.gl_view:
            if event.type() == event.Type.Resize:
                self.loading_label.setGeometry(self.gl_view.rect())
                return False
            
            if event.type() == event.Type.MouseButtonPress and event.button() == Qt.MouseButton.RightButton:
                self._mouse_right_pressed = True
                self._last_mouse_pos = event.position().toPoint()
                return True

            if event.type() == event.Type.MouseButtonRelease and event.button() == Qt.MouseButton.RightButton:
                self._mouse_right_pressed = False
                self._last_mouse_pos = None
                return True

            if event.type() == event.Type.MouseMove and self._mouse_right_pressed and self._last_mouse_pos:
                delta = event.position().toPoint() - self._last_mouse_pos
                dz = -0.01 * delta.y()
                center = self.gl_view.opts['center']
                new_center = QVector4D(center.x(), center.y(), center.z() + dz, 1.0)
                self.gl_view.opts['center'] = new_center.toVector3D()
                self._last_mouse_pos = event.position().toPoint()
                self.gl_view.update()
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
        self.btn_grid.move(x, y)
        x += self.btn_grid.width() + margin
        self.combo.move(x, y)

    # ===================== URDF =====================
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

    # ===================== FRAMES & RENDERING =====================
    def _populate_frames(self):
        try:
            lines = self.tf_buffer.all_frames_as_string().splitlines()
            frames = [L.split()[1] for L in lines if L.startswith('Frame ')]
        except Exception:
            frames = []
        self._cached_frames = frames[:]

        current_frame = self.combo.currentText()
        items = ['Fixed Frame'] + frames
        if [self.combo.itemText(i) for i in range(self.combo.count())] != items:
            self.combo.blockSignals(True)
            self.combo.clear()
            self.combo.addItems(items)
            if current_frame in items:
                self.combo.setCurrentText(current_frame)
            else:
                self.combo.setCurrentIndex(0)
                self.change_fixed_frame('Fixed Frame')
            self.combo.blockSignals(False)

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

    def toggle_grid(self):
        self.grid_item.setVisible(self.btn_grid.isChecked())

    def _tf_callback(self, msg: TFMessage):
        pass

    def _update_view(self):
        frames = self._cached_frames
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

                last = self._last_link_tf.get(link_name)
                if last is not None and np.allclose(T, last, atol=1e-4, rtol=0.0):
                    continue
                self._last_link_tf[link_name] = T

                mat = QMatrix4x4()
                for i in range(4):
                    mat.setRow(i, QVector4D(*T[i, :]))
                item.setTransform(mat)

        if self.render_tf:
            now_ms = QTime.currentTime().msecsSinceStartOfDay()
            if (now_ms - self._axes_last_update_ms) >= self._axes_update_interval_ms:
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
                        x[2*i]   = o; x[2*i+1] = o + R @ np.array([L, 0, 0], dtype=np.float32)
                        y[2*i]   = o; y[2*i+1] = o + R @ np.array([0, L, 0], dtype=np.float32)
                        z[2*i]   = o; z[2*i+1] = o + R @ np.array([0, 0, L], dtype=np.float32)
                    self.x_axes.setData(pos=x)
                    self.y_axes.setData(pos=y)
                    self.z_axes.setData(pos=z)

                self._axes_last_update_ms = now_ms

def main():
    rclpy.init()
    node = rclpy.create_node('RobotStateViewer')
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

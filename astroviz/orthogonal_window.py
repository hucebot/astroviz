#!/usr/bin/env python3
import sys
import os
import threading
import math
import fractions
import tempfile
import re
import json
import cv2

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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from pyqtgraph.opengl import GLScatterPlotItem, GLMeshItem, GLLinePlotItem

from urdfpy import URDF
import trimesh

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QPushButton, QLabel, QComboBox
)
from PyQt6.QtCore    import pyqtSignal, Qt, QTimer, QPoint
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
        self.selected_laser_topic = None
        self.selected_object_name = None
        self.map_item = None
        self.map_data = None
        self._left_pressed = False
        self._left_dragged = False
        self._press_pos = None
        self.world_objects_by_name = {}

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

        self.laser_subscriber = None
        self.laser_points_item = gl.GLScatterPlotItem(color=(1, 0, 0, 1), size=5)
        self.gl_view.addItem(self.laser_points_item)

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
        self.combo.currentTextChanged.connect(self.change_fixed_frame)

        self.laser_combo = QComboBox(self.gl_view)
        self.laser_combo.setFixedWidth(180)
        self.laser_combo.raise_()
        self.laser_combo.currentTextChanged.connect(self.change_laser_topic)

        self.map_combo = QComboBox(self.gl_view)
        self.map_combo.setFixedWidth(180)
        self.map_combo.raise_()
        self.map_combo.currentTextChanged.connect(self.change_map_topic)



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

        self.world_items = []
        self.render_world = True

        self.frame_timer = QTimer(self)
        self.frame_timer.timeout.connect(self._populate_frames)
        self.frame_timer.start(1000)

        self.gl_view.setMouseTracking(True)
        self._mouse_right_pressed = False
        self._last_mouse_pos = None

        self.load_world_objects()


    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin
        map_frame = msg.header.frame_id

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        color_map = np.zeros((height, width, 4), dtype=np.float32)
        color_map[data == -1] = [0.5, 0.5, 0.5, 0.3]
        color_map[data == 0]  = [1.0, 1.0, 1.0, 0.2]
        color_map[data > 50]  = [0.0, 0.0, 0.0, 0.6]

        xs = np.linspace(0, width * resolution, width)
        ys = np.linspace(0, height * resolution, height)
        xv, yv = np.meshgrid(xs, ys)
        zv = np.zeros_like(xv)

        verts = np.stack([xv, yv, zv], axis=-1).reshape(-1, 3)
        colors = color_map.reshape(-1, 4)

        faces = []
        for y in range(height - 1):
            for x in range(width - 1):
                i = y * width + x
                faces.append([i, i + 1, i + width + 1])
                faces.append([i, i + width + 1, i + width])
        faces = np.array(faces, dtype=np.int32)

        if self.map_item:
            self.gl_view.removeItem(self.map_item)

        self.map_item = GLMeshItem(
            vertexes=verts,
            faces=faces,
            faceColors=colors[faces[:, 0]],
            drawEdges=False,
            smooth=False
        )
        self.map_item.setGLOptions('opaque')
        self.gl_view.addItem(self.map_item)

        T_map_local = np.eye(4, dtype=np.float32)
        T_map_local[:3, 3] = [origin.position.x, origin.position.y, origin.position.z]

        try:
            tf = self.tf_buffer.lookup_transform(self.root_frame, map_frame, rclpy.time.Time())
            t, q = tf.transform.translation, tf.transform.rotation

            T_tf = np.eye(4, dtype=np.float32)
            T_tf[:3, :3] = quaternion_to_matrix(q)
            T_tf[:3, 3] = [t.x, t.y, t.z]

            T_global = T_tf @ T_map_local
        except Exception as e:
            self.node.get_logger().warn(f"TF transform failed from {self.root_frame} to {map_frame}: {str(e)}")
            T_global = T_map_local

        mat = QMatrix4x4()
        for i in range(4):
            mat.setRow(i, QVector4D(*T_global[i, :]))
        self.map_item.setTransform(mat)


    def load_world_objects(self):
        if not os.path.exists(MESHES_PATH):
            self.node.get_logger().warn(f"No meshes file found at {MESHES_PATH}.")
            return

        if os.path.getsize(MESHES_PATH) == 0 or open(MESHES_PATH, 'r').read().strip() == "":
            self.node.get_logger().warn("The world_display.json file is empty. No world objects will be loaded.")
            return

        with open(MESHES_PATH, 'r') as f:
            try:
                object_list = json.load(f)
            except json.JSONDecodeError:
                self.node.get_logger().error("Error parsing world_display.json")
                return

        if not object_list:
            self.node.get_logger().warn("World display configuration is an empty list. No world objects will be loaded.")
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
            gl_item._original_vertices = verts 

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
            self.world_items.append((gl_item, obj["name"], obj["frame"], T, verts))
            self.world_objects_by_name[obj["name"]] = (gl_item, verts)


    def highlight_object(self, name):
        if hasattr(self, 'highlight_box'):
            self.gl_view.removeItem(self.highlight_box)
        result = self.world_objects_by_name.get(name)
        if result is None:
            return

        item, verts = result

        transform = item.transform()
        T = np.eye(4, dtype=np.float32)
        for i in range(4):
            row = transform.row(i)
            T[i, :] = [row.x(), row.y(), row.z(), row.w()]
        verts_world = (T @ np.hstack((verts, np.ones((verts.shape[0], 1)))).T).T[:, :3]

        min_pt = np.min(verts_world, axis=0)
        max_pt = np.max(verts_world, axis=0)

        corners = np.array([
            [min_pt[0], min_pt[1], min_pt[2]],
            [max_pt[0], min_pt[1], min_pt[2]],
            [max_pt[0], max_pt[1], min_pt[2]],
            [min_pt[0], max_pt[1], min_pt[2]],
            [min_pt[0], min_pt[1], max_pt[2]],
            [max_pt[0], min_pt[1], max_pt[2]],
            [max_pt[0], max_pt[1], max_pt[2]],
            [min_pt[0], max_pt[1], max_pt[2]],
        ], dtype=np.float32)

        lines = np.array([
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ])

        lines_pts = []
        for start, end in lines:
            lines_pts.extend([corners[start], corners[end]])

        self.highlight_box = GLLinePlotItem(
            pos=np.array(lines_pts),
            color=(1, 1, 0, 1),
            width=2,
            mode='lines'
        )
        self.gl_view.addItem(self.highlight_box)


    def change_laser_topic(self, topic: str):
        if topic == 'Laser Topic':
            self.selected_laser_topic = None
            if self.laser_subscriber:
                self.laser_subscriber.destroy()
                self.laser_subscriber = None
            self.laser_points_item.setData(pos=np.empty((0, 3)))
            return

        self.selected_laser_topic = topic

        if self.laser_subscriber:
            self.laser_subscriber.destroy()
            self.laser_subscriber = None

        self.laser_subscriber = self.node.create_subscription(
            LaserScan,
            topic,
            self.laser_callback,
            qos_profile=10
        )

    def laser_callback(self, msg: LaserScan):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.root_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = quaternion_to_matrix(q)
        T[:3, 3] = [t.x, t.y, t.z]

        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)
        ones = np.ones_like(xs)

        points = np.vstack((xs, ys, zs, ones)).T
        points = points[np.isfinite(points).all(axis=1)]

        points_world = (T @ points.T).T[:, :3]

        self.laser_points_item.setData(pos=points_world)




    def _populate_frames(self):
        try:
            lines = self.tf_buffer.all_frames_as_string().splitlines()
            frames = [L.split()[1] for L in lines if L.startswith('Frame ')]
        except Exception:
            frames = []

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

        laser_topics = []
        try:
            laser_topics = [t for t, ttype in self.node.get_topic_names_and_types()
                            if 'sensor_msgs/msg/LaserScan' in ttype]
        except Exception:
            pass

        current_laser = self.laser_combo.currentText()
        laser_items = ['Laser Topic'] + laser_topics
        if [self.laser_combo.itemText(i) for i in range(self.laser_combo.count())] != laser_items:
            self.laser_combo.blockSignals(True)
            self.laser_combo.clear()
            self.laser_combo.addItems(laser_items)
            if current_laser in laser_items:
                self.laser_combo.setCurrentText(current_laser)
            else:
                self.laser_combo.setCurrentIndex(0)
                self.change_laser_topic('Laser Topic')
            self.laser_combo.blockSignals(False)

        map_topics = []
        try:
            map_topics = [t for t, ttype in self.node.get_topic_names_and_types()
                        if 'nav_msgs/msg/OccupancyGrid' in ttype]
        except Exception:
            pass

        current_map = self.map_combo.currentText()
        map_items = ['Map'] + map_topics
        if [self.map_combo.itemText(i) for i in range(self.map_combo.count())] != map_items:
            self.map_combo.blockSignals(True)
            self.map_combo.clear()
            self.map_combo.addItems(map_items)
            if current_map in map_items:
                self.map_combo.setCurrentText(current_map)
            else:
                self.map_combo.setCurrentIndex(0)
                self.change_map_topic('Map')
            self.map_combo.blockSignals(False)


    def change_map_topic(self, topic: str):
        if topic == 'Map':
            if hasattr(self, 'map_subscriber') and self.map_subscriber:
                self.map_subscriber.destroy()
                self.map_subscriber = None
            return

        if hasattr(self, 'map_subscriber') and self.map_subscriber:
            self.map_subscriber.destroy()
            self.map_subscriber = None

        self.map_subscriber = self.node.create_subscription(
            OccupancyGrid,
            topic,
            self.map_callback,
            qos_profile=10
        )

    def ray_intersect_aabb(self, ray_origin, ray_dir, bbox_min, bbox_max):

        inv_dir = 1.0 / np.where(np.abs(ray_dir) < 1e-6, 1e-6, ray_dir)

        t0s = (bbox_min - ray_origin) * inv_dir
        t1s = (bbox_max - ray_origin) * inv_dir

        t_near = np.max(np.minimum(t0s, t1s), axis=0)
        t_far  = np.min(np.maximum(t0s, t1s), axis=0)

        if t_far < 0 or t_near > t_far:
            return None
        return t_near

    def _select_object_under_mouse(self, pos):
        w, h = self.gl_view.width(), self.gl_view.height()
        x_ndc = (2.0 * pos.x() / w) - 1.0
        y_ndc = 1.0 - (2.0 * pos.y() / h)

        proj_mat = self.gl_view.projectionMatrix()
        view_mat = self.gl_view.viewMatrix()

        mvp = proj_mat * view_mat
        inv_mvp, invertible = mvp.inverted()
        if not invertible:
            return

        near_clip = QVector4D(x_ndc, y_ndc, -1.0, 1.0)
        far_clip  = QVector4D(x_ndc, y_ndc,  1.0, 1.0)

        near_world4 = inv_mvp * near_clip
        far_world4  = inv_mvp * far_clip

        near_world = np.array([
            near_world4.x()/near_world4.w(),
            near_world4.y()/near_world4.w(),
            near_world4.z()/near_world4.w()
        ], dtype=np.float32)
        far_world = np.array([
            far_world4.x()/far_world4.w(),
            far_world4.y()/far_world4.w(),
            far_world4.z()/far_world4.w()
        ], dtype=np.float32)

        ray_origin = near_world
        ray_dir = far_world - near_world
        ray_dir /= np.linalg.norm(ray_dir)

        closest = None
        min_t = np.inf
        for item, name, frame, T_local, verts in self.world_items:
            T = item.transform()
            M = np.eye(4, dtype=np.float32)
            for i in range(4):
                r = T.row(i)
                M[i,:] = [r.x(), r.y(), r.z(), r.w()]
            hw = np.hstack((verts, np.ones((verts.shape[0],1))))
            verts_w = (M @ hw.T).T[:, :3]
            mn, mx = verts_w.min(axis=0), verts_w.max(axis=0)

            t0s = (mn - ray_origin) / ray_dir
            t1s = (mx - ray_origin) / ray_dir
            t_near = np.max(np.minimum(t0s, t1s))
            t_far  = np.min(np.maximum(t0s, t1s))
            if t_near <= t_far and t_far > 0 and t_near < min_t:
                min_t = t_near
                closest = name

        if closest is not None:
            self.highlight_object(closest)
            self.selected_object_name = closest
        else:
            if hasattr(self, 'highlight_box'):
                self.gl_view.removeItem(self.highlight_box)
                del self.highlight_box
            self.selected_object_name = None



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

            if event.type() == event.Type.MouseButtonPress and event.button() == Qt.MouseButton.LeftButton:
                self._left_pressed = True
                self._left_dragged = False
                self._press_pos = event.position().toPoint()
                return False

            if event.type() == event.Type.MouseMove and self._left_pressed:
                if (event.position().toPoint() - self._press_pos).manhattanLength() > 5:
                    self._left_dragged = True
                return False

            if event.type() == event.Type.MouseButtonRelease and event.button() == Qt.MouseButton.LeftButton:
                if self._left_pressed and not self._left_dragged:
                    self._select_object_under_mouse(self._press_pos)
                self._left_pressed = False
                self._press_pos = None
                return False

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
        x += self.combo.width() + margin
        self.laser_combo.move(x, y)
        x += self.laser_combo.width() + margin
        self.map_combo.move(x, y)

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
        for item, _, _, _, _ in self.world_items:
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

        for item, name, frame, T_local, _ in self.world_items:
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

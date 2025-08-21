#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, time, math, os, re, traceback, fractions
import numpy as np
import moderngl

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QSurfaceFormat, QIcon
from PyQt6.QtOpenGL import QOpenGLWindow
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QPushButton

import collections, collections.abc
collections.Mapping = collections.abc.Mapping
collections.Set = collections.abc.Set
collections.Iterable = collections.abc.Iterable
fractions.gcd = math.gcd
setattr(np, 'int', int)
setattr(np, 'float', float)

# ------- ROS 2 -------
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RosTime
from std_msgs.msg import String
import tf2_ros

# ------- URDF + mallas -------
from urdfpy import URDF
import trimesh

from astroviz.common._shape import ShapeGrid, ShapeMesh
from astroviz.common._camera import Camera
from astroviz.utils.window_style import DarkStyle

from ament_index_python.packages import get_package_share_directory
from astroviz.common._find import _find_pkg, _find_src_config

_src_config = _find_src_config()
_CONFIG_DIR = _src_config if _src_config else os.path.join(get_package_share_directory('astroviz'), 'config')
_pkg = _find_pkg()
_PKG_DIR = _pkg if _pkg else get_package_share_directory('astroviz')
ICONS_DIR = os.path.join(_PKG_DIR, 'icons')

# ==================== Utilidades ====================

def _quaternion_to_mat3(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=np.float32)

def _mat4_from_tf(transform):
    t = transform.translation
    q = transform.rotation
    M = np.eye(4, dtype=np.float32)
    M[:3, :3] = _quaternion_to_mat3(q)
    M[:3, 3]  = [t.x, t.y, t.z]
    return M

def _fix_package_urls(urdf_xml: str) -> str:
    def repl(m):
        pkg = m.group(1); rest = m.group(2)
        try:
            base = get_package_share_directory(pkg)
        except Exception:
            return m.group(0)
        return base + "/" + rest
    return re.sub(r'package://([^/]+)/(.+?)(?=["\'])', repl, urdf_xml)

def _load_trimesh_from_path(path: str):
    loaded = trimesh.load(path, process=False)
    if isinstance(loaded, trimesh.Scene):
        geos = list(loaded.geometry.values())
        mesh = trimesh.util.concatenate(geos) if len(geos) > 1 else geos[0]
    else:
        mesh = loaded
    v = np.asarray(mesh.vertices, dtype=np.float32)
    f = np.asarray(mesh.faces,    dtype=np.int32)
    return v, f

# ==================== ROS Node ====================

class ViewerNode(Node):
    def __init__(self):
        super().__init__('moderngl_grid_viewer_cam')

# ==================== GL Window ====================

class GLWindow(QOpenGLWindow):
    def __init__(self, node: ViewerNode | None = None):
        super().__init__()
        self.node = node
        self.ctx: moderngl.Context | None = None
        self.grid: ShapeGrid | None = None

        self.cam = Camera()
        self.cam.setTargetPos([0.0, 0.0, 0.0])
        self.cam.setEyePos([1.8, 0.0, 1.0])

        self._dragging_orbit = False
        self._dragging_dolly = False
        self._last_pos = None
        self._eye_radius, self._eye_az, self._eye_el = self.cam.getEyePolar()
        self._sens_rot = 0.008
        self._sens_zoom = 0.12
        self._sens_dolly = 0.004
        self._min_radius = 0.2
        self._max_radius = 200.0

        self.show_grid = True

        self.root_frame = 'odom'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.robot_visuals = []
        self._last_ok_T = {}

        self.render_delay_s = 0.06
        self.delay_min_s    = 0.02
        self.delay_max_s    = 0.30
        self._adapt_cooldown = 0

        qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/robot_description', self._on_robot_description, qos)

        # Timer (~60 FPS)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(16)

    def set_show_grid(self, flag: bool):
        self.show_grid = bool(flag)
        self.update()
    def initializeGL(self):
        self.ctx = moderngl.create_context()
        self.ctx.enable(moderngl.DEPTH_TEST)

        self.ctx.disable(moderngl.CULL_FACE)

        self.grid = ShapeGrid(self.ctx, length=10.0, segments=20)

        info = self.ctx.info
        print("GL_VENDOR  :", info.get("GL_VENDOR"))
        print("GL_RENDERER:", info.get("GL_RENDERER"))
        print("GL_VERSION :", info.get("GL_VERSION"))

        self.resizeGL(self.width(), self.height())

    def resizeGL(self, w: int, h: int):
        if not self.ctx:
            return
        dpr = self.devicePixelRatio()
        W, H = max(1, int(w * dpr)), max(1, int(h * dpr))
        self.ctx.viewport = (0, 0, W, H)
        self.cam.setParams(self.cam.getFovx(), self.cam.getFovy(), (0, 0, float(W), float(H)))

    def _adaptive_delay_update(self, success_ratio: float):
        if self._adapt_cooldown > 0:
            self._adapt_cooldown -= 1
            return
        if success_ratio < 0.80:
            self.render_delay_s = min(self.render_delay_s + 0.01, self.delay_max_s)
            self._adapt_cooldown = 10
        elif success_ratio > 0.97:
            self.render_delay_s = max(self.render_delay_s - 0.005, self.delay_min_s)
            self._adapt_cooldown = 10

    def paintGL(self):
        if self.ctx is None:
            return

        self.ctx.clear(0.05, 0.06, 0.07, 1.0)

        if self.show_grid and self.grid:
            self.grid.render(self.cam)

        if not self.robot_visuals:
            return

        mat_proj_cm = self.cam.getMatProj()

        now_ros = self.node.get_clock().now()
        render_time = now_ros - Duration(seconds=self.render_delay_s)

        ok = 0
        total = len(self.robot_visuals)

        for it in self.robot_visuals:
            link = it['link_name']
            T_draw = None

            try:
                tf = self.tf_buffer.lookup_transform(self.root_frame, link, render_time, timeout=Duration(seconds=0.0))
                T_tf = _mat4_from_tf(tf.transform)
                T_draw = T_tf @ it['T_local']
                self._last_ok_T[link] = T_draw
                ok += 1
            except Exception:
                try:
                    tf_latest = self.tf_buffer.lookup_transform(self.root_frame, link, RosTime(), timeout=Duration(seconds=0.0))
                    T_tf = _mat4_from_tf(tf_latest.transform)
                    T_draw = T_tf @ it['T_local']
                    self._last_ok_T[link] = T_draw
                except Exception:
                    T_draw = self._last_ok_T.get(link, None)

            if T_draw is not None:
                it['shape'].render(mat_proj_cm, T_draw, color=it['color'])

        if total > 0:
            self._adaptive_delay_update(ok / total)

    def closeEvent(self, e):
        for it in self.robot_visuals:
            try:
                it['shape'].release()
            except Exception:
                pass
        super().closeEvent(e)

    # ---------- input ----------
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging_orbit = True
            self._last_pos = event.position()
            self._eye_radius, self._eye_az, self._eye_el = self.cam.getEyePolar()
        elif event.button() == Qt.MouseButton.MiddleButton:
            self._dragging_dolly = True
            self._last_pos = event.position()
            self._eye_radius, self._eye_az, self._eye_el = self.cam.getEyePolar()

    def mouseMoveEvent(self, event):
        if self._last_pos is None:
            return
        pos = event.position()
        dx = pos.x() - self._last_pos.x()
        dy = pos.y() - self._last_pos.y()
        self._last_pos = pos

        if self._dragging_orbit:
            self._eye_az -= dx * self._sens_rot
            self._eye_el += dy * self._sens_rot
            eps = 1e-3
            self._eye_el = float(np.clip(self._eye_el, eps, math.pi - eps))
            self.cam.setEyePolar(self._eye_radius, self._eye_az, self._eye_el)

        elif self._dragging_dolly:
            factor = math.exp(dy * self._sens_dolly)
            self._eye_radius = float(np.clip(self._eye_radius * factor, self._min_radius, self._max_radius))
            self.cam.setEyePolar(self._eye_radius, self._eye_az, self._eye_el)

        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging_orbit = False
            self._last_pos = None
        elif event.button() == Qt.MouseButton.MiddleButton:
            self._dragging_dolly = False
            self._last_pos = None

    def wheelEvent(self, event):
        num_steps = event.angleDelta().y() / 120.0
        factor = math.exp(-self._sens_zoom * num_steps)
        self._eye_radius = float(np.clip(self._eye_radius * factor, self._min_radius, self._max_radius))
        self.cam.setEyePolar(self._eye_radius, self._eye_az, self._eye_el)
        self.update()

    # ---------- URDF ----------
    def _on_robot_description(self, msg: String):
        try:
            xml = (msg.data or "").strip()
            if not xml:
                return
            xml = _fix_package_urls(xml)

            import tempfile
            with tempfile.NamedTemporaryFile(delete=False, suffix='.urdf') as tmp:
                tmp.write(xml.encode('utf-8'))
                tmp.flush()
                urdf = URDF.load(tmp.name)

            cpu_meshes = []
            for link in urdf.links:
                for visual in link.visuals:
                    geom = visual.geometry
                    mesh_path = None
                    if hasattr(geom, 'filename') and geom.filename:
                        mesh_path = geom.filename
                    elif hasattr(geom, 'mesh') and geom.mesh and hasattr(geom.mesh, 'filename'):
                        mesh_path = geom.mesh.filename
                    if not mesh_path:
                        continue

                    v, f = _load_trimesh_from_path(mesh_path)

                    # scale
                    scale = getattr(geom, 'scale', None)
                    if scale is not None:
                        v = v * np.array(scale, dtype=np.float32).reshape(1, 3)

                    # color
                    color = (0.7, 0.7, 0.7, 1.0)
                    if visual.material is not None and getattr(visual.material, 'color', None) is not None:
                        rgba = visual.material.color
                        if len(rgba) == 4:
                            color = tuple([float(c) for c in rgba])

                    # T_local
                    T_local = np.eye(4, dtype=np.float32)
                    org = getattr(visual, 'origin', None)
                    if org is not None:
                        if isinstance(org, np.ndarray) and org.shape == (4,4):
                            T_local = org.astype(np.float32)
                        else:
                            try:
                                xyz = getattr(org, 'xyz', [0, 0, 0])
                                rpy = getattr(org, 'rpy', [0, 0, 0])
                                cr, sr = np.cos(rpy[0]), np.sin(rpy[0])
                                cp, sp = np.cos(rpy[1]), np.sin(rpy[1])
                                cy, sy = np.cos(rpy[2]), np.sin(rpy[2])
                                R = np.array([
                                    [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                                    [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                                    [-sp,   cp*sr,            cp*cr]
                                ], dtype=np.float32)
                                T_local[:3, :3] = R
                                T_local[:3, 3] = np.array(xyz, dtype=np.float32)
                            except Exception:
                                T_local = np.eye(4, dtype=np.float32)

                    cpu_meshes.append(dict(
                        vertices=v, faces=f, color=color, T_local=T_local, link_name=link.name
                    ))

            for it in self.robot_visuals:
                try: it['shape'].release()
                except Exception: pass
            self.robot_visuals = []
            self._last_ok_T.clear()

            if self.ctx is None:
                return

            for m in cpu_meshes:
                shape = ShapeMesh(self.ctx, m['vertices'], m['faces'], color=m['color'])
                self.robot_visuals.append(dict(
                    shape=shape,
                    T_local=np.ascontiguousarray(m['T_local'], dtype=np.float32),
                    link_name=m['link_name'],
                    color=m['color']
                ))

            self._seed_cache_latest()

        except Exception as e:
            print("[GL] Error parseando URDF:", e)
            traceback.print_exc()

    def _seed_cache_latest(self):
        for it in self.robot_visuals:
            link = it['link_name']
            try:
                tf_latest = self.tf_buffer.lookup_transform(self.root_frame, link, RosTime(), timeout=Duration(seconds=0.0))
                T_tf = _mat4_from_tf(tf_latest.transform)
                self._last_ok_T[link] = T_tf @ it['T_local']
            except Exception:
                pass

# ==================== MainWindow ====================

class MainWindow(QMainWindow):
    def __init__(self, node: ViewerNode, gl_window_cls=GLWindow):
        super().__init__()
        self.setWindowTitle('Orthogonal Viewer')
        icon_path = os.path.join(ICONS_DIR, 'astroviz_icon.png')
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))

        self.glw = gl_window_cls(node)
        self.container = QWidget.createWindowContainer(self.glw, self)
        self.container.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        central = QWidget(self)
        lay = QHBoxLayout(central)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self.container, 1)
        self.setCentralWidget(central)

        self.btn_grid = QPushButton('GRID', self.container)
        self.btn_grid.setCheckable(True)
        self.btn_grid.setChecked(True)
        self.btn_grid.setFixedWidth(80)
        self.btn_grid.clicked.connect(lambda: self.glw.set_show_grid(self.btn_grid.isChecked()))
        self.btn_grid.raise_()

        self._margin = 5
        self._position_overlays()
        self.container.installEventFilter(self)

    def _position_overlays(self):
        self.btn_grid.move(self._margin, self._margin)

    def eventFilter(self, obj, event):
        if obj is self.container and event.type() == event.Type.Resize:
            self._position_overlays()
        return super().eventFilter(obj, event)

# ==================== main() ====================

def main():
    fmt = QSurfaceFormat()
    fmt.setRenderableType(QSurfaceFormat.RenderableType.OpenGL)
    fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CoreProfile)
    fmt.setVersion(3, 3)
    fmt.setSwapInterval(1)
    QSurfaceFormat.setDefaultFormat(fmt)

    rclpy.init()
    node = ViewerNode()

    app = QApplication(sys.argv)
    try:
        DarkStyle(app)
    except Exception:
        pass

    win = MainWindow(node=node, gl_window_cls=GLWindow)
    win.resize(1200, 800)
    win.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)

    def _cleanup():
        spin_timer.stop()
        node.destroy_node()
        rclpy.shutdown()
    app.aboutToQuit.connect(_cleanup)

    sys.exit(app.exec())

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os, math, argparse, threading, time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QComboBox, QPushButton, QDoubleSpinBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QMatrix4x4, QVector3D, QIcon

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from tf2_ros import Buffer, TransformListener

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

try:
    from astroviz.utils.window_style import DarkStyle
    from astroviz.common._find import _find_pkg, _find_src_config
except Exception:
    DarkStyle = lambda app: None
    def _find_pkg(): return None
    def _find_src_config(): return None

# ==== YAML ====
try:
    import yaml
except Exception:
    yaml = None

pg.setConfigOptions(antialias=False)

_src_config = _find_src_config()
if _src_config:
    _CONFIG_DIR = _src_config
else:
    try:
        _CONFIG_DIR = os.path.join(get_package_share_directory('astroviz'), 'config') if get_package_share_directory else os.getcwd()
    except Exception:
        _CONFIG_DIR = os.getcwd()

_pkg = _find_pkg()
if _pkg:
    _PKG_DIR = _pkg
else:
    try:
        _PKG_DIR = get_package_share_directory('astroviz') if get_package_share_directory else os.getcwd()
    except Exception:
        _PKG_DIR = os.getcwd()

os.makedirs(_CONFIG_DIR, exist_ok=True)
ICONS_DIR  = os.path.join(_PKG_DIR, 'icons')

DEFAULT_MAX_POINTS   = int(os.getenv("ASTROVIZ_POINT_BUDGET", "250000"))
DEFAULT_RENDER_HZ    = 60
SCAN_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# ==== Dtype map ====
_PF_TO_DTYPE = {
    PointField.INT8:   np.int8,
    PointField.UINT8:  np.uint8,
    PointField.INT16:  np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32:  np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}

def _mk_dtype(fields_needed, msg):
    fdict = {f.name: f for f in msg.fields}
    names, formats, offsets = [], [], []
    for n in fields_needed:
        f = fdict[n]
        names.append(n); formats.append(_PF_TO_DTYPE[f.datatype]); offsets.append(f.offset)
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                   "itemsize": msg.point_step})
    return dt.newbyteorder(">" if msg.is_bigendian else "<")

def _norm_uint_channel(ch):
    info = np.iinfo(ch.dtype); return (ch.astype(np.float32, copy=False) / float(info.max))

def _norm_float_channel(ch):
    ch = ch.astype(np.float32, copy=False)
    vmax = float(np.nanmax(ch)) if ch.size else 1.0
    if vmax > 1.5: ch = ch / 255.0
    return np.clip(ch, 0.0, 1.0)

class GLViewWidgetFOV(gl.GLViewWidget):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.vfov_deg = 60.0
        self.hfov_deg = None
        dist = float(self.opts.get('distance', 10.0))
        default_near = max(1e-3, dist*0.01)
        default_far  = max(1000.0, dist*100.0)
        self.opts['clip'] = (default_near, default_far)

    def set_fovs(self, vfov=None, hfov=None):
        if vfov is not None: self.vfov_deg = float(vfov)
        if hfov is not None: self.hfov_deg = float(hfov)
        self.opts['fov'] = self.vfov_deg
        self.update()

    def projectionMatrix(self, region=None):
        dist = float(self.opts.get('distance', 10.0))
        default_near = max(1e-3, dist*0.01)
        default_far  = max(1000.0, dist*100.0)
        near, far = self.opts.get('clip', (default_near, default_far))
        if not (isinstance(near,(int,float)) and isinstance(far,(int,float))): near, far = default_near, default_far
        if not (far > near): far = near + max(1.0, abs(far-near) + 1.0)

        m = QMatrix4x4()
        w = max(1, self.width()); h = max(1, self.height())
        aspect = w / h

        if self.hfov_deg is None:
            m.perspective(self.vfov_deg, aspect, near, far)
        else:
            vf = math.radians(self.vfov_deg)
            hf = math.radians(self.hfov_deg)
            top   = math.tan(vf*0.5) * near
            right = math.tan(hf*0.5) * near
            m.frustum(-right, right, -top, top, near, far)
        return m

# ==== Util YAML ====
def getn(d, path, default=None):
    cur = d
    try:
        for k in path.split('.'):
            if k == '': continue
            cur = cur[k]
        return cur
    except Exception:
        return default

def setn(d, path, value):
    cur = d
    keys = path.split('.')
    for k in keys[:-1]:
        if k not in cur or not isinstance(cur[k], dict):
            cur[k] = {}
        cur = cur[k]
    cur[keys[-1]] = value

def ros_vec_to_pg(v):
    x,y,z = float(v[0]), float(v[1]), float(v[2])
    return np.array([-y, x, z], dtype=np.float32)

def pg_vec_to_ros(v):
    x,y,z = float(v[0]), float(v[1]), float(v[2])
    return np.array([y, -x, z], dtype=np.float32)

# ========================= Viewer =========================
class FPVViewer(QMainWindow):
    def __init__(self, node: Node, config: dict | None, config_path: str | None = None):
        super().__init__()
        self.node = node
        self.config = config or {}
        self.config_path = config_path

        self._raw_frame = None
        self._raw_stamp = Time()
        self._frame_sel = "RAW (source)"

        self.max_points = int(getn(self.config, 'point_cloud.max_points', DEFAULT_MAX_POINTS))
        self._lod_scale = 1.0

        self.setWindowTitle(getn(self.config, 'main_window.name', '3D Perspective'))
        try: self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, 'astroviz_icon.png')))
        except Exception: pass
        self._apply_window_geometry_from_config()

        widget = QWidget(); self.setCentralWidget(widget)
        layout = QVBoxLayout(widget); layout.setContentsMargins(0,0,0,0)

        self.gl = GLViewWidgetFOV()
        self.gl.setBackgroundColor('#090c28')
        layout.addWidget(self.gl)
        self.gl.installEventFilter(self)

        self.topic_combo = QComboBox(self.gl); self.topic_combo.setFixedWidth(300); self.topic_combo.raise_()
        self.topic_combo.currentTextChanged.connect(self._on_topic)

        self.btn_reset = QPushButton("Reset Front", self.gl); self.btn_reset.raise_()
        self.btn_reset.clicked.connect(self.reset_front_view)

        self.btn_save = QPushButton("Save view", self.gl); self.btn_save.raise_()
        self.btn_save.clicked.connect(self._save_current_view)

        self.vfov_spin = QDoubleSpinBox(self.gl)
        self.vfov_spin.setRange(5.0, 170.0); self.vfov_spin.setDecimals(1); self.vfov_spin.setSingleStep(1.0)
        self.vfov_spin.setPrefix("vFOV "); self.vfov_spin.setSuffix("°"); self.vfov_spin.setFixedWidth(110); self.vfov_spin.raise_()
        self.vfov_spin.valueChanged.connect(lambda v: (self.gl.set_fovs(vfov=float(v)), self._sync_fov_spins()))

        self.hfov_spin = QDoubleSpinBox(self.gl)
        self.hfov_spin.setRange(5.0, 170.0); self.hfov_spin.setDecimals(1); self.hfov_spin.setSingleStep(1.0)
        self.hfov_spin.setPrefix("hFOV "); self.hfov_spin.setSuffix("°"); self.hfov_spin.setFixedWidth(110); self.hfov_spin.raise_()
        self.hfov_spin.valueChanged.connect(lambda v: (self.gl.set_fovs(hfov=float(v)), self._sync_fov_spins()))

        self._pan_sensitivity = float(getn(self.config, 'main_window.camera_velocity', 0.1))
        self._dolly_sensitivity = 0.01

        ps = float(getn(self.config, 'point_cloud.point_size', 1.0))
        sm = float(getn(self.config, 'point_cloud.size_multiplier', 1.0))
        self._point_size = max(1.0, ps * sm)

        self.scatter = gl.GLScatterPlotItem(size=self._point_size, pxMode=True)
        self.scatter.setGLOptions("opaque")
        self.gl.addItem(self.scatter)

        R = QMatrix4x4(); R.rotate(90.0, -90.0, 0.0, 1.0)
        self.scatter.setTransform(R)

        render_hz = float(getn(self.config, 'main_window.render_hz', DEFAULT_RENDER_HZ))
        self._target_ms = 1000.0 / max(1.0, render_hz)

        self.ros_timer   = QTimer(self); self.ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0)); self.ros_timer.start(2)
        self.update_timer= QTimer(self); self.update_timer.timeout.connect(self._refresh); self.update_timer.start(int(self._target_ms))

        self.topic_timer = QTimer(self); self.topic_timer.timeout.connect(self._populate_topics); self.topic_timer.start(800)

        self._lut = self._make_lut()

        vfov_cfg = getn(self.config, 'point_cloud.vfov', None)
        hfov_cfg = getn(self.config, 'point_cloud.hfov', None)
        self.gl.set_fovs(vfov=vfov_cfg if vfov_cfg is not None else 60.0,
                         hfov=hfov_cfg if hfov_cfg is not None else None)
        self._sync_fov_spins()

        self._apply_front_camera_from_yaml()

        self._msg_lock = threading.Lock()
        self._last_msg = None

        self._dec_lock = threading.Lock()
        self._xyz = np.empty((0,3), np.float32)
        self._rgba = None
        self._decoded_dirty = False

        self._stop = False
        self._decoder = threading.Thread(target=self._decode_worker, daemon=True)
        self._decoder.start()

        self._pending_topic = getn(self.config, 'point_cloud.topic', None)
        self._last_upload_ms = 5.0

    def _apply_window_geometry_from_config(self):
        w = int(getn(self.config, 'main_window.width', 1024))
        h = int(getn(self.config, 'main_window.height', 720))
        x = int(getn(self.config, 'main_window.x', 0))
        y = int(getn(self.config, 'main_window.y', 0))
        self.resize(w, h)
        try: self.move(x, y)
        except Exception: pass

    def _apply_front_camera_from_yaml(self):
        ox = float(getn(self.config, 'viewports.frontal.camera.x_origin',  1.0))
        oy = float(getn(self.config, 'viewports.frontal.camera.y_origin',  0.0))
        oz = float(getn(self.config, 'viewports.frontal.camera.z_origin',  1.0))
        tx = float(getn(self.config, 'viewports.frontal.camera.x_target',  0.0))
        ty = float(getn(self.config, 'viewports.frontal.camera.y_target',  0.0))
        tz = float(getn(self.config, 'viewports.frontal.camera.z_target',  0.0))
        self.FRONT_EYE_ROS    = np.array([ox, oy, oz], dtype=np.float32)
        self.FRONT_TARGET_ROS = np.array([tx, ty, tz], dtype=np.float32)
        self.reset_front_view()

    def reset_front_view(self):
        eye_pg    = ros_vec_to_pg(getattr(self, 'FRONT_EYE_ROS', np.array([1,0,1], np.float32)))
        target_pg = ros_vec_to_pg(getattr(self, 'FRONT_TARGET_ROS', np.array([0,0,0], np.float32)))
        v = eye_pg - target_pg
        dist = float(np.linalg.norm(v)) if np.linalg.norm(v)>1e-9 else 1.0
        elev = math.degrees(math.asin(np.clip(v[2]/dist, -1.0, 1.0)))
        azim = math.degrees(math.atan2(v[1], v[0]))
        self.gl.setCameraPosition(distance=dist, elevation=elev, azimuth=azim,
                                  pos=QVector3D(*target_pg))
        self.gl.opts['clip'] = (max(1e-3, dist*0.01), max(1000.0, dist*100.0))
        self.gl.update()

    def _sync_fov_spins(self):
        self.vfov_spin.blockSignals(True); self.hfov_spin.blockSignals(True)
        try:
            vf = float(getattr(self.gl, "vfov_deg", self.gl.opts.get('fov', 60.0)))
            if getattr(self.gl, "hfov_deg", None) is None:
                w = max(1, self.gl.width()); h = max(1, self.gl.height())
                aspect = w / h
                hf = math.degrees(2.0 * math.atan(aspect * math.tan(math.radians(vf)*0.5)))
            else:
                hf = float(self.gl.hfov_deg)
            self.vfov_spin.setValue(round(vf,1))
            self.hfov_spin.setValue(round(hf,1))
        finally:
            self.vfov_spin.blockSignals(False); self.hfov_spin.blockSignals(False)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        m=5; x=m; y=m
        self.topic_combo.move(x,y); x+= self.topic_combo.width()+m
        self.btn_reset.move(x,y);   x+= self.btn_reset.width()+m
        self.btn_save.move(x,y);    x+= self.btn_save.width()+m
        self.vfov_spin.move(x,y);   x+= self.vfov_spin.width()+m
        self.hfov_spin.move(x,y)

    def eventFilter(self, src, ev):
        if src is self.gl:
            if ev.type() == ev.Type.Resize:
                self._sync_fov_spins()
                return False
            # Pan (MMB)
            if ev.type()==ev.Type.MouseButtonPress and ev.button()==Qt.MouseButton.MiddleButton:
                self._mm_down=True; self._mm_last=ev.position().toPoint(); return True
            if ev.type()==ev.Type.MouseMove and getattr(self,'_mm_down',False):
                p=ev.position().toPoint(); d=p-self._mm_last; self._pan_camera(d.x(), d.y()); self._mm_last=p; return True
            if ev.type()==ev.Type.MouseButtonRelease and ev.button()==Qt.MouseButton.MiddleButton:
                self._mm_down=False; return True
            # Dolly / Pan (Shift+RMB)
            if ev.type()==ev.Type.MouseButtonPress and ev.button()==Qt.MouseButton.RightButton:
                self._rm_down=True; self._rm_last=ev.position().toPoint()
                self._right_mode = 'pan' if (ev.modifiers() & Qt.KeyboardModifier.ShiftModifier) else 'dolly'; return True
            if ev.type()==ev.Type.MouseMove and getattr(self,'_rm_down',False):
                p=ev.position().toPoint(); d=p-self._rm_last
                if self._right_mode=='pan': self._pan_camera(d.x(), d.y())
                else:
                    dz = -self._dolly_sensitivity * float(d.y())
                    c = self.gl.opts['center']
                    self.gl.opts['center'] = QVector3D(c.x(), c.y(), c.z()+dz); self.gl.update()
                self._rm_last=p; return True
            if ev.type()==ev.Type.MouseButtonRelease and ev.button()==Qt.MouseButton.RightButton:
                self._rm_down=False; return True
        return super().eventFilter(src, ev)

    def _pan_camera(self, dx_px: float, dy_px: float):
        view = self.gl.viewMatrix()
        inv, ok = view.inverted()
        if not ok: return
        right = np.array([inv.column(0).x(), inv.column(0).y(), inv.column(0).z()], dtype=np.float32)
        up    = np.array([inv.column(1).x(), inv.column(1).y(), inv.column(1).z()], dtype=np.float32)
        right /= (np.linalg.norm(right)+1e-9); up /= (np.linalg.norm(up)+1e-9)
        dist = float(self.gl.opts.get('distance', 1.0))
        fov_deg = float(getattr(self.gl, "vfov_deg", self.gl.opts.get('fov', 60.0)))
        h = max(1, self.gl.height())
        px2m = (2.0 * dist * math.tan(math.radians(fov_deg)*0.5)) / h
        s = px2m * float(self._pan_sensitivity)
        pan_vec = (-dx_px*s)*right + (dy_px*s)*up
        c = self.gl.opts['center']
        self.gl.opts['center'] = QVector3D(c.x()+pan_vec[0], c.y()+pan_vec[1], c.z()+pan_vec[2])
        self.gl.update()

    # -------------- Topics / Frames --------------
    def _populate_topics(self):
        current = self.topic_combo.currentText()
        topics = [name for name, types in self.node.get_topic_names_and_types() if 'sensor_msgs/msg/PointCloud2' in types]
        items = ['---'] + topics
        if [self.topic_combo.itemText(i) for i in range(self.topic_combo.count())] != items:
            self.topic_combo.blockSignals(True)
            self.topic_combo.clear(); self.topic_combo.addItems(items)
            if hasattr(self, "_pending_topic") and self._pending_topic and (self._pending_topic in items):
                self.topic_combo.setCurrentText(self._pending_topic); self._pending_topic=None
            elif current in items:
                self.topic_combo.setCurrentText(current)
            else:
                self.topic_combo.setCurrentIndex(0); self._on_topic('---')
            self.topic_combo.blockSignals(False)

    def _parse_frames_yaml(self, yaml_txt: str):
        frames = set()
        for line in yaml_txt.splitlines():
            s=line.strip()
            if s.startswith("Frame:"):
                parts=s.split()
                if len(parts)>=2: frames.add(parts[1].strip(','))
            elif s.startswith("Parent:"):
                parts=s.split()
                if len(parts)>=2: frames.add(parts[1])
        return sorted(frames)

    def _on_frame_changed(self, txt: str):
        self._frame_sel = txt

    def _on_topic(self, topic_name: str):
        if hasattr(self, 'cloud_sub') and self.cloud_sub:
            try: self.node.destroy_subscription(self.cloud_sub)
            except Exception: pass
            self.cloud_sub = None
        if topic_name == '---': return
        self.cloud_sub = self.node.create_subscription(PointCloud2, topic_name, self._pc_cb, qos_profile=SCAN_QOS)

    def _pc_cb(self, msg: PointCloud2):
        with self._msg_lock:
            self._last_msg = msg

    @staticmethod
    def _unpack_packed_rgba_from_struct_field(field_array, is_bigendian: bool, with_alpha: bool) -> np.ndarray:
        buf = field_array.tobytes()
        endi = '>' if is_bigendian else '<'
        u32 = np.frombuffer(buf, dtype=np.dtype(endi + 'u4'))
        if is_bigendian:
            r = (u32 >> 24) & 0xFF; g = (u32 >> 16) & 0xFF; b = (u32 >> 8) & 0xFF
            a = (u32 >>  0) & 0xFF if with_alpha else np.full(u32.shape[0], 255, dtype=np.uint32)
        else:
            r = (u32 >> 16) & 0xFF; g = (u32 >> 8) & 0xFF; b = (u32 >> 0) & 0xFF
            a = (u32 >> 24) & 0xFF if with_alpha else np.full(u32.shape[0], 255, dtype=np.uint32)
        out = np.empty((u32.shape[0], 4), dtype=np.float32)
        out[:,0] = r.astype(np.float32)/255.0
        out[:,1] = g.astype(np.float32)/255.0
        out[:,2] = b.astype(np.float32)/255.0
        out[:,3] = a.astype(np.float32)/255.0
        return out

    def _decode_worker(self):
        while not self._stop:
            with self._msg_lock:
                msg = self._last_msg
                self._last_msg = None
            if msg is None:
                time.sleep(0.001); continue

            fdict = {f.name: f for f in msg.fields}
            if not all(k in fdict for k in ("x","y","z")):
                continue

            has_rgba = "rgba" in fdict
            has_rgb  = "rgb"  in fdict
            has_sep  = all(k in fdict for k in ("r","g","b"))
            has_int  = "intensity" in fdict

            needed = ["x","y","z"]
            color_mode = None
            if has_rgba: needed += ["rgba"]; color_mode="rgba"
            elif has_rgb: needed += ["rgb"]; color_mode="rgb"
            elif has_sep:
                needed += ["r","g","b"]
                if "a" in fdict: needed += ["a"]
                color_mode="sep"
            elif has_int:
                needed += ["intensity"]; color_mode="int"

            dtype = _mk_dtype(needed, msg)
            count = msg.width * msg.height
            arr_struct = np.frombuffer(msg.data, dtype=dtype, count=count)

            max_pts = self.max_points
            if count > max_pts:
                if msg.height > 1 and msg.width > 0:
                    stride = int(np.ceil(np.sqrt(count / max_pts)))
                    h, w = msg.height, msg.width
                    arr_struct = arr_struct.reshape(h, -1)[::stride, :].reshape(-1)[::stride]
                else:
                    step = int(np.ceil(count / max_pts))
                    arr_struct = arr_struct[::step]

            xyz = np.column_stack((arr_struct["x"], arr_struct["y"], arr_struct["z"]))
            mask = np.isfinite(xyz).all(axis=1)
            if not mask.all():
                arr_struct = arr_struct[mask]
                xyz = xyz[mask]
            xyz = np.ascontiguousarray(xyz, dtype=np.float32)

            rgba = None
            if color_mode in ("rgb","rgba"):
                fld = "rgba" if color_mode=="rgba" else "rgb"
                rgba = self._unpack_packed_rgba_from_struct_field(arr_struct[fld],
                                                                  msg.is_bigendian,
                                                                  with_alpha=(color_mode=="rgba"))
            elif color_mode == "sep":
                r, g, b = arr_struct["r"], arr_struct["g"], arr_struct["b"]
                a = arr_struct["a"] if "a" in arr_struct.dtype.names else None
                rgba = np.empty((r.shape[0],4), dtype=np.float32)
                rgba[:,0] = _norm_uint_channel(r) if r.dtype.kind in "ui" else _norm_float_channel(r)
                rgba[:,1] = _norm_uint_channel(g) if g.dtype.kind in "ui" else _norm_float_channel(g)
                rgba[:,2] = _norm_uint_channel(b) if b.dtype.kind in "ui" else _norm_float_channel(b)
                rgba[:,3] = (_norm_uint_channel(a) if (a is not None and a.dtype.kind in "ui")
                             else (_norm_float_channel(a) if a is not None else 1.0))
            elif color_mode == "int":
                it = arr_struct["intensity"]
                gray = _norm_uint_channel(it) if it.dtype.kind in "ui" else _norm_float_channel(it)
                rgba = np.empty((gray.shape[0],4), dtype=np.float32)
                rgba[:,0]=gray; rgba[:,1]=gray; rgba[:,2]=gray; rgba[:,3]=1.0

            with self._dec_lock:
                self._xyz = xyz
                self._rgba = rgba
                self._raw_frame = msg.header.frame_id.strip() if msg.header.frame_id else None
                self._raw_stamp = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
                self._decoded_dirty = True

    def _make_lut(self)->np.ndarray:
        n=256; t=np.linspace(0,1,n,dtype=np.float32)
        lut=np.zeros((n,4),dtype=np.float32); lut[:,0]=1.0-t; lut[:,1]=t; lut[:,2]=0.2; lut[:,3]=1.0
        return lut

    def _compute_z_colors(self, z: np.ndarray)->np.ndarray:
        if z.size==0: return np.empty((0,4),np.float32)
        zmin,zmax=float(np.min(z)),float(np.max(z))
        denom=(zmax-zmin) if (zmax>zmin) else 1.0
        idx=np.clip(((z-zmin)/denom*255.0).astype(np.int32),0,255)
        return self._lut[idx]

    def _refresh(self):
        with self._dec_lock:
            if not self._decoded_dirty:
                return
            xyz = self._xyz
            rgba = self._rgba
            src_frame = self._raw_frame
            stamp = self._raw_stamp
            self._decoded_dirty = False

        if xyz.size == 0:
            self.scatter.setData(pos=np.empty((0,3),np.float32))
            return

        colors = rgba if rgba is not None else self._compute_z_colors(xyz[:,2])
        self.scatter.setData(pos=xyz, color=colors, size=self._point_size, pxMode=True)

    def _save_current_view(self):
        center = self.gl.opts['center']
        target_pg = np.array([center.x(), center.y(), center.z()], dtype=np.float32)
        dist  = float(self.gl.opts.get('distance', 1.0))
        elev  = math.radians(float(self.gl.opts.get('elevation', 0.0)))
        azim  = math.radians(float(self.gl.opts.get('azimuth', 0.0)))
        dir_pg = np.array([math.cos(elev)*math.cos(azim),
                           math.cos(elev)*math.sin(azim),
                           math.sin(elev)], dtype=np.float32)
        eye_pg = target_pg + dist * dir_pg

        eye_ros    = pg_vec_to_ros(eye_pg)
        target_ros = pg_vec_to_ros(target_pg)

        vfov = float(getattr(self.gl, "vfov_deg", 60.0))
        hfov = getattr(self.gl, "hfov_deg", None)
        if hfov is not None: hfov = float(hfov)

        setn(self.config, 'viewports.frontal.camera.x_origin', float(eye_ros[0]))
        setn(self.config, 'viewports.frontal.camera.y_origin', float(eye_ros[1]))
        setn(self.config, 'viewports.frontal.camera.z_origin', float(eye_ros[2]))
        setn(self.config, 'viewports.frontal.camera.x_target', float(target_ros[0]))
        setn(self.config, 'viewports.frontal.camera.y_target', float(target_ros[1]))
        setn(self.config, 'viewports.frontal.camera.z_target', float(target_ros[2]))
        setn(self.config, 'point_cloud.vfov', round(vfov, 3))
        if hfov is None:
            try:
                del self.config['point_cloud']['hfov']
            except Exception:
                pass
        else:
            setn(self.config, 'point_cloud.hfov', round(hfov, 3))

        out_path = self.config_path if self.config_path else os.path.join(_CONFIG_DIR, 'saved_view.yaml')
        if yaml is not None:
            try:
                with open(out_path, 'w') as f:
                    yaml.safe_dump(self.config, f, sort_keys=False)
                self.node.get_logger().info(f"Saved current view to: {out_path}")
            except Exception:
                pass

    def closeEvent(self, ev):
        try:
            self._stop = True
        except Exception:
            pass
        return super().closeEvent(ev)

def load_config(path: str | None)->dict | None:
    if path is None or yaml is None: return None
    if not os.path.exists(path): return None
    try:
        with open(path,'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None

def parse_args():
    ap = argparse.ArgumentParser(description="FPV Viewer")
    ap.add_argument("--config", type=str, default=None, help="Path to YAML config file")
    return ap.parse_args()

def main(args=None):
    cli = parse_args()
    cfg = load_config(cli.config)
    rclpy.init(args=args)
    node = rclpy.create_node('lidar_viewer_front_fast')
    app = QApplication(sys.argv)
    try: DarkStyle(app)
    except Exception: pass
    viewer = FPVViewer(node, cfg, cli.config)
    viewer.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

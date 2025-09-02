#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, math, time, argparse, threading
from queue import Queue, Empty, Full
import numpy as np

os.environ.setdefault("OPEN3D_LOG_LEVEL", "ERROR")

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

try:
    import yaml
except Exception:
    yaml = None

import open3d as o3d
from open3d.visualization import gui, rendering
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

DEFAULT_MAX_POINTS = int(os.getenv("ASTROVIZ_POINT_BUDGET", "200000"))
DEFAULT_RENDER_HZ  = 60
SCAN_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

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
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    return dt.newbyteorder(">" if msg.is_bigendian else "<")

def _norm_uint_channel(ch):
    info = np.iinfo(ch.dtype)
    return (ch.astype(np.float32, copy=False) / float(info.max))

def _norm_float_channel(ch):
    ch = ch.astype(np.float32, copy=False)
    vmax = float(np.nanmax(ch)) if ch.size else 1.0
    if vmax > 1.5: ch = ch / 255.0
    return np.clip(ch, 0.0, 1.0)

def pc2_to_xyz_rgba_fast(msg: PointCloud2, max_points: int | None = None, stride_phase: int = 0, remove_nans: bool = False):
    """Decodifica PointCloud2 -> (xyz float32 Nx3, rgba float32 Nx4 ó None) con decimado por stride+fase."""
    fdict = {f.name: f for f in msg.fields}
    if not all(k in fdict for k in ("x","y","z")):
        return (np.empty((0,3), np.float32), None)

    has_rgb  = "rgb"  in fdict
    has_rgba = "rgba" in fdict
    has_sep  = all(k in fdict for k in ("r","g","b"))
    has_int  = "intensity" in fdict

    needed = ["x","y","z"]
    if has_rgba: needed += ["rgba"]
    elif has_rgb: needed += ["rgb"]
    elif has_sep:
        needed += ["r","g","b"]
        if "a" in fdict: needed += ["a"]
    elif has_int: needed += ["intensity"]

    dtype = _mk_dtype(needed, msg)
    count = msg.width * msg.height
    arr_struct = np.frombuffer(msg.data, dtype=dtype, count=count)

    if max_points is not None and count > max_points:
        step = int(np.ceil(count / max_points))
        start = stride_phase % step
        arr_struct = arr_struct[start::step]

    xyz = np.column_stack((arr_struct["x"], arr_struct["y"], arr_struct["z"])).astype(np.float32, copy=False)
    if remove_nans:
        mask = np.isfinite(xyz).all(axis=1)
        if not mask.all():
            arr_struct = arr_struct[mask]
            xyz = xyz[mask]

    rgba = None
    def _unpack_packed_rgba(field_name: str, with_alpha: bool):
        raw = arr_struct[field_name]
        bytes_buf = raw.tobytes()
        endi = '>' if msg.is_bigendian else '<'
        u32 = np.frombuffer(bytes_buf, dtype=np.dtype(endi+'u4'))
        if msg.is_bigendian:
            r = (u32 >> 24) & 0xFF; g = (u32 >> 16) & 0xFF; b = (u32 >> 8) & 0xFF
            a = (u32 >>  0) & 0xFF if with_alpha else np.full(u32.shape[0], 255, dtype=np.uint32)
        else:
            r = (u32 >> 16) & 0xFF; g = (u32 >> 8) & 0xFF; b = (u32 >> 0) & 0xFF
            a = (u32 >> 24) & 0xFF if with_alpha else np.full(u32.shape[0], 255, dtype=np.uint32)
        out = np.empty((u32.shape[0], 4), dtype=np.float32)
        out[:,0] = r.astype(np.float32) / 255.0
        out[:,1] = g.astype(np.float32) / 255.0
        out[:,2] = b.astype(np.float32) / 255.0
        out[:,3] = a.astype(np.float32) / 255.0
        return out

    if has_rgba or has_rgb:
        rgba = _unpack_packed_rgba("rgba" if has_rgba else "rgb", with_alpha=has_rgba)
    elif has_sep:
        r, g, b = arr_struct["r"], arr_struct["g"], arr_struct["b"]
        a = arr_struct["a"] if "a" in arr_struct.dtype.names else None
        rgba = np.empty((r.shape[0], 4), dtype=np.float32)
        rgba[:,0] = _norm_uint_channel(r) if r.dtype.kind in "ui" else _norm_float_channel(r)
        rgba[:,1] = _norm_uint_channel(g) if g.dtype.kind in "ui" else _norm_float_channel(g)
        rgba[:,2] = _norm_uint_channel(b) if b.dtype.kind in "ui" else _norm_float_channel(b)
        rgba[:,3] = (_norm_uint_channel(a) if a is not None and a.dtype.kind in "ui"
                     else (_norm_float_channel(a) if a is not None else 1.0))
    elif has_int:
        it = arr_struct["intensity"]
        gray = _norm_uint_channel(it) if it.dtype.kind in "ui" else _norm_float_channel(it)
        rgba = np.empty((gray.shape[0], 4), dtype=np.float32)
        rgba[:,0] = gray; rgba[:,1] = gray; rgba[:,2] = gray; rgba[:,3] = 1.0

    return xyz, rgba

# ===== TF helpers =====
def quat_to_rot(qx,qy,qz,qw)->np.ndarray:
    n = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    if n == 0.0: return np.eye(3, dtype=np.float32)
    x,y,z,w = qx/n, qy/n, qz/n, qw/n
    xx,yy,zz = x*x,y*y,z*z; xy,xz,yz = x*y,x*z,y*z; wx,wy,wz = w*x,w*y,w*z
    R = np.array([
        [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
        [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
        [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
    ], dtype=np.float32)
    return R

def tf_to_matrix(tf)->np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
    R = quat_to_rot(q.x,q.y,q.z,q.w)
    T = np.eye(4, dtype=np.float32)
    T[:3,:3] = R; T[:3,3] = [t.x,t.y,t.z]
    return T

class DecodeWorker(threading.Thread):
    def __init__(self, in_q: Queue, out_q: Queue, tf_buffer,
                 remove_nans_default=False, max_range=None, min_z=None, max_z=None,
                 use_color=True):
        super().__init__(daemon=True)
        self.in_q = in_q
        self.out_q = out_q
        self.tf_buffer = tf_buffer
        self.remove_nans_default = remove_nans_default
        self._stop = threading.Event()

        self.max_range = float(max_range) if (max_range is not None) else None
        self.min_z     = float(min_z)     if (min_z     is not None) else None
        self.max_z     = float(max_z)     if (max_z     is not None) else None
        self.use_color = bool(use_color)

        self._pos_tmp = None
        self._pos_out = None
        self._rgb_buf = None

        self._stride_phase = 0

    def stop(self): self._stop.set()

    def _ensure_cap(self, n):
        if (self._pos_tmp is None) or (self._pos_tmp.shape[0] < n):
            cap = int(n*1.2) + 1024
            self._pos_tmp = np.empty((cap, 3), dtype=np.float32)
            self._pos_out = np.empty((cap, 3), dtype=np.float32)
        if (self._rgb_buf is None) or (self._rgb_buf.shape[0] < n):
            cap = int(n*1.2) + 1024
            self._rgb_buf = np.empty((cap, 3), dtype=np.float32)

    def _apply_tf(self, xyz, target_frame, source_frame, stamp: Time):
        n = xyz.shape[0]
        self._ensure_cap(n)
        if (source_frame is None) or (target_frame in (None, "RAW (source)")) or (target_frame == source_frame):
            self._pos_tmp[:n, :] = xyz
            return self._pos_tmp[:n, :]
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, stamp, timeout=Duration(seconds=0.001))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=0.001))
            except Exception:
                self._pos_tmp[:n, :] = xyz
                return self._pos_tmp[:n, :]
        T = tf_to_matrix(tf); R = T[:3,:3]; t = T[:3,3]
        self._pos_tmp[:n, 0] = xyz[:,0]*R[0,0] + xyz[:,1]*R[1,0] + xyz[:,2]*R[2,0] + t[0]
        self._pos_tmp[:n, 1] = xyz[:,0]*R[0,1] + xyz[:,1]*R[1,1] + xyz[:,2]*R[2,1] + t[1]
        self._pos_tmp[:n, 2] = xyz[:,0]*R[0,2] + xyz[:,1]*R[1,2] + xyz[:,2]*R[2,2] + t[2]
        return self._pos_tmp[:n, :]

    def _cull(self, pts):
        if pts.size == 0:
            return pts
        mask = np.ones((pts.shape[0],), dtype=bool)
        if self.max_range is not None:
            r2 = np.sum(pts**2, axis=1)
            mask &= (r2 <= float(self.max_range)**2)
        if self.min_z is not None:
            mask &= (pts[:,2] >= self.min_z)
        if self.max_z is not None:
            mask &= (pts[:,2] <= self.max_z)
        if mask.all():
            return pts
        return pts[mask]

    def run(self):
        while not self._stop.is_set():
            try:
                msg, max_points, remove_nans, target_frame = self.in_q.get(timeout=0.1)
            except Empty:
                continue
            remove_nans = self.remove_nans_default if (remove_nans is None) else remove_nans
            self._stride_phase += 1
            try:
                xyz, rgba = pc2_to_xyz_rgba_fast(msg, max_points=max_points,
                                                 stride_phase=self._stride_phase,
                                                 remove_nans=remove_nans)
                if xyz.size == 0:
                    try:
                        while True: self.out_q.get_nowait()
                    except Empty:
                        pass
                    self.out_q.put_nowait((np.empty((0,3), np.float32), None))
                    continue

                frame = msg.header.frame_id.strip() if msg.header.frame_id else None
                stamp = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)

                pts = self._apply_tf(xyz, target_frame, frame, stamp)
                pts = self._cull(pts)
                if pts.size == 0:
                    try:
                        while True: self.out_q.get_nowait()
                    except Empty:
                        pass
                    self.out_q.put_nowait((np.empty((0,3), np.float32), None))
                    continue

                n = pts.shape[0]
                self._ensure_cap(n)

                # ROS->viz: (x,y,z)->(-y, x, z)
                self._pos_out[:n, 0] = -pts[:, 1]
                self._pos_out[:n, 1] =  pts[:, 0]
                self._pos_out[:n, 2] =  pts[:, 2]
                out_xyz = self._pos_out[:n, :]

                if self.use_color and (rgba is not None) and (rgba.shape[0] >= n):
                    self._rgb_buf[:n, :] = rgba[:n, :3]
                    out_rgb = self._rgb_buf[:n, :]
                else:
                    out_rgb = None

                try:
                    while True: self.out_q.get_nowait()
                except Empty:
                    pass
                self.out_q.put_nowait((out_xyz.copy(), None if out_rgb is None else out_rgb.copy()))
            except Exception:
                continue

class O3DViewer:
    def __init__(self, node: Node, config: dict | None):
        self.node = node
        self.cfg  = config or {}

        self.max_points_cap = int(self._get('point_cloud.max_points', DEFAULT_MAX_POINTS))
        self._runtime_max_points = min(self.max_points_cap, 150000)
        self._remove_nans_default = bool(self._get('point_cloud.remove_nans', False))
        self._use_color = bool(self._get('point_cloud.use_color', True))
        self._max_range = self._get('point_cloud.max_range', None)
        self._min_z     = self._get('point_cloud.min_z', None)
        self._max_z     = self._get('point_cloud.max_z', None)

        w  = int(self._get('main_window.width', 1600))
        h  = int(self._get('main_window.height', 800))
        x0 = int(self._get('main_window.x', 0))
        y0 = int(self._get('main_window.y', 0))
        name = str(self._get('main_window.name', '3D Perspective'))

        gui.Application.instance.initialize()
        self.window = gui.Application.instance.create_window(name, w, h, x0, y0)
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene_widget)

        self.scene_widget.scene.set_background([0.035, 0.047, 0.157, 1.0])
        try:
            self.scene_widget.scene.scene.enable_sun_light(False)
        except Exception:
            pass

        self.device = o3d.core.Device('CPU:0')
        self.pcd_t = o3d.t.geometry.PointCloud(self.device)

        self.mat = rendering.MaterialRecord()
        self.mat.shader = "defaultUnlit"
        self.mat.point_size = float(self._get('point_cloud.point_size', 1.0)) * float(self._get('point_cloud.size_multiplier', 2.0))
        self.scene_widget.scene.add_geometry("pcd", self.pcd_t, self.mat)

        ox = float(self._get('viewports.frontal.camera.x_origin',  1.0))
        oy = float(self._get('viewports.frontal.camera.y_origin',  0.0))
        oz = float(self._get('viewports.frontal.camera.z_origin',  1.0))
        tx = float(self._get('viewports.frontal.camera.x_target',  0.0))
        ty = float(self._get('viewports.frontal.camera.y_target',  0.0))
        tz = float(self._get('viewports.frontal.camera.z_target',  0.0))
        eye_v    = np.array([-oy, ox, oz], dtype=np.float32)
        target_v = np.array([-ty, tx, tz], dtype=np.float32)

        self._vfov = self._get('point_cloud.vfov', None)
        self._hfov = self._get('point_cloud.hfov', None)
        self._near = 0.01
        self._far  = 1000.0

        self._target_dt = 1.0 / float(self._get('main_window.render_hz', DEFAULT_RENDER_HZ))
        self._ema = None
        self._running = True

        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=False)

        self._in_q  = Queue(maxsize=1)
        self._out_q = Queue(maxsize=1)
        self.worker = DecodeWorker(self._in_q, self._out_q, self.tf_buffer,
                                   remove_nans_default=self._remove_nans_default,
                                   max_range=self._max_range, min_z=self._min_z, max_z=self._max_z,
                                   use_color=self._use_color)
        self.worker.start()

        self.topic = None
        self.frame_target = "RAW (source)"
        self.pending_topic = self._get('point_cloud.topic', None)
        if self.pending_topic:
            self._subscribe(self.pending_topic)

        self.window.set_on_layout(self._on_layout)
        self.window.set_on_close(self._on_close)

        self._set_camera_view(target_v, eye_v, np.array([0,0,1], dtype=np.float32))
        self._apply_projection()

        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._spin_thread.start()
        self._tick_thread.start()

        try:
            if not o3d.core.cuda.is_available():
                self._runtime_max_points = min(self._runtime_max_points, 80000)
        except Exception:
            pass

    def _get(self, path, default=None):
        d = self.cfg or {}
        try:
            cur = d
            for k in path.split('.'):
                cur = cur[k]
            return cur
        except Exception:
            return default

    def _on_layout(self, layout_ctx):
        r = self.window.content_rect
        self.scene_widget.frame = r
        self._apply_projection()

    def _apply_projection(self):
        cam = self.scene_widget.scene.camera
        w = max(1, self.scene_widget.frame.width)
        h = max(1, self.scene_widget.frame.height)
        aspect = w / h
        near, far = self._near, self._far
        if self._hfov is not None:
            cam.set_projection(float(self._hfov), float(aspect), float(near), float(far),
                               rendering.Camera.FovType.Horizontal)
        else:
            vfov = float(self._vfov if self._vfov is not None else 60.0)
            cam.set_projection(float(vfov), float(aspect), float(near), float(far),
                               rendering.Camera.FovType.Vertical)

    def _set_camera_view(self, center, eye, up):
        cam = self.scene_widget.scene.camera
        if hasattr(cam, "look_at"):
            cam.look_at(center.tolist(), eye.tolist(), up.tolist())
        else:
            minb = np.minimum(center, eye) - 0.5
            maxb = np.maximum(center, eye) + 0.5
            bbox = o3d.geometry.AxisAlignedBoundingBox(minb.astype(np.float64),
                                                       maxb.astype(np.float64))
            self.scene_widget.setup_camera(60.0, bbox, center.astype(np.float32))
        self._apply_projection()

    def _on_close(self):
        self._running = False
        try:
            if self.worker: self.worker.stop()
        except Exception:
            pass
        return True

    def _spin_loop(self):
        while self._running:
            try: rclpy.spin_once(self.node, timeout_sec=0.001)
            except Exception: pass
            time.sleep(0.0005)

    def _tick_loop(self):
        while self._running:
            t0 = time.perf_counter()
            try:
                gui.Application.instance.post_to_main_thread(self.window, self._on_tick)
            except Exception:
                pass
            dt = time.perf_counter() - t0
            time.sleep(max(0.0, self._target_dt - dt))

    def _subscribe(self, topic_name: str):
        if self.topic == topic_name: return
        if hasattr(self, "cloud_sub") and self.cloud_sub:
            try: self.node.destroy_subscription(self.cloud_sub)
            except Exception: pass
        self.cloud_sub = self.node.create_subscription(
            PointCloud2, topic_name, self._pc_cb, qos_profile=SCAN_QOS
        )
        self.topic = topic_name

    def _pc_cb(self, msg: PointCloud2):
        try:
            while True: self._in_q.get_nowait()
        except Empty:
            pass
        try:
            self._in_q.put_nowait((msg, self._runtime_max_points, None, self.frame_target))
        except Full:
            pass

    def _on_tick(self):
        t0 = time.perf_counter()
        updated = False
        try:
            xyz, rgb = self._out_q.get_nowait()

            pos_t = o3d.core.Tensor.from_numpy(xyz)  # float32 Nx3
            self.pcd_t.point["positions"] = pos_t

            if self._use_color and (rgb is not None):
                self.pcd_t.point["colors"] = o3d.core.Tensor.from_numpy(rgb[:, :3].astype(np.float32, copy=False))
            else:
                if "colors" in self.pcd_t.point:
                    del self.pcd_t.point["colors"]

            try:
                self.scene_widget.scene.update_geometry("pcd", self.pcd_t)
            except Exception:
                self.scene_widget.scene.remove_geometry("pcd")
                self.scene_widget.scene.add_geometry("pcd", self.pcd_t, self.mat)
            updated = True
        except Empty:
            pass

        dt = time.perf_counter() - t0
        self._ema = dt if self._ema is None else (0.3*dt + 0.7*self._ema)
        if self._ema > 1.1*self._target_dt and self._runtime_max_points > 20000:
            self._runtime_max_points = max(20000, int(self._runtime_max_points * 0.8))
        elif self._ema < 0.6*self._target_dt and self._runtime_max_points < self.max_points_cap:
            self._runtime_max_points = min(self.max_points_cap, int(self._runtime_max_points * 1.25))

        if updated:
            self.window.post_redraw()

    def close(self):
        self._running = False
        try:
            if self.worker:
                self.worker.stop()
                self.worker.join(timeout=0.2)
        except Exception:
            pass
        try:
            if hasattr(self, "cloud_sub") and self.cloud_sub:
                self.node.destroy_subscription(self.cloud_sub)
        except Exception:
            pass

# ===== YAML / CLI =====
def load_config(path: str | None)->dict | None:
    if path is None: return None
    if yaml is None: return None
    if not os.path.exists(path): return None
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None

def parse_args():
    ap = argparse.ArgumentParser(description="Visor LiDAR (Open3D + GPU)")
    ap.add_argument("--config", type=str, default=None, help="File path to the YAML configuration")
    return ap.parse_args()

# ===== Main =====
def main():
    cli = parse_args()
    cfg = load_config(cli.config)

    rclpy.init()
    node = rclpy.create_node('lidar_viewer_o3d')

    viewer = O3DViewer(node, cfg)
    try:
        gui.Application.instance.run()
    finally:
        viewer.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

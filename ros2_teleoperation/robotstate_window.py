#!/usr/bin/env python3
import sys
import os
import threading
import math
import fractions
import tempfile
# Monkey-patch deprecated imports for urdfpy/networkx compatibility
import collections
import collections.abc
collections.Mapping = collections.abc.Mapping
collections.Set = collections.abc.Set
collections.Iterable = collections.abc.Iterable
fractions.gcd = math.gcd

import numpy as np
# restore deprecated numpy aliases
setattr(np, 'int', int)
setattr(np, 'float', float)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
import tf2_ros

from urdfpy import URDF
import trimesh

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt6.QtCore    import pyqtSignal, Qt
from PyQt6.QtGui     import QMatrix4x4, QVector4D
import pyqtgraph.opengl as gl

ORIGINAL_URDF_PATH = '/ros2_ws/src/g1_description/description_files/urdf/g1_29dof.urdf'
MESH_DIR          = '/ros2_ws/src/g1_description/description_files/meshes/'

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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.mesh_items = []

        w = QWidget()
        self.setCentralWidget(w)
        layout = QVBoxLayout(w)
        layout.setContentsMargins(0,0,0,0)

        self.gl_view = gl.GLViewWidget()
        self.gl_view.opts['distance'] = 1.0
        self.gl_view.setBackgroundColor((0.2, 0.2, 0.2, 1))
        layout.addWidget(self.gl_view)

        grid = gl.GLGridItem()
        grid.scale(0.1,0.1,0.1)
        self.gl_view.addItem(grid)

        self.scatter = gl.GLScatterPlotItem()
        self.x_axes = gl.GLLinePlotItem(color=(1,0,0,1), width=2, mode='lines')
        self.y_axes = gl.GLLinePlotItem(color=(0,1,0,1), width=2, mode='lines')
        self.z_axes = gl.GLLinePlotItem(color=(0,0,1,1), width=2, mode='lines')
        for item in (self.scatter, self.x_axes, self.y_axes, self.z_axes):
            self.gl_view.addItem(item)

        self.update_signal.connect(self._update_view, Qt.ConnectionType.QueuedConnection)

        self.node.create_subscription(TFMessage, '/tf', self._tf_callback, qos_profile=10)
        static_qos = QoSProfile(depth=10)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.node.create_subscription(TFMessage, '/tf_static', self._tf_callback, qos_profile=static_qos)

        self._load_urdf()

    def _load_urdf(self):
        xml = open(ORIGINAL_URDF_PATH, 'r').read()
        xml_fixed = xml.replace(
            'package://g1_description/description_files/meshes/',
            MESH_DIR
        )
        tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
        tmp.write(xml_fixed.encode()); tmp.flush(); tmp.close()

        urdf = URDF.load(tmp.name)

        for link in urdf.links:
            for visual in link.visuals:
                fn = visual.geometry.mesh.filename
                mesh_path = fn if os.path.isabs(fn) else os.path.join(MESH_DIR, fn)

                scene_or_mesh = trimesh.load_mesh(mesh_path, process=False)
                tm = scene_or_mesh.dump() if hasattr(scene_or_mesh, 'dump') else scene_or_mesh
                verts   = tm.vertices.view(np.ndarray)
                faces   = tm.faces.view(np.ndarray)

                if tm.vertex_normals is None or len(tm.vertex_normals)==0:
                    normals = tm.vertex_normals.view(np.ndarray)
                else:
                    normals = tm.vertex_normals.view(np.ndarray)

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
                self.gl_view.addItem(item)

                if isinstance(visual.origin, np.ndarray) and visual.origin.shape == (4,4):
                    T_lv = visual.origin.astype(np.float32)
                else:
                    T = np.eye(4, dtype=np.float32)
                    if hasattr(visual.origin, 'position') and hasattr(visual.origin, 'rotation'):
                        pos = visual.origin.position
                        rot = quaternion_to_matrix(visual.origin.rotation)
                        T[:3,:3] = rot
                        T[:3,3] = [pos.x, pos.y, pos.z]
                    T_lv = T

                self.mesh_items.append((item, link.name, T_lv))

    def _tf_callback(self, msg: TFMessage):
        if msg.transforms:
            self.update_signal.emit()

    def _update_view(self):
        lines = self.tf_buffer.all_frames_as_string().splitlines()
        frames = [L.split()[1] for L in lines if L.startswith('Frame ')]

        for item, link_name, T_lv in self.mesh_items:
            try:
                tf = self.tf_buffer.lookup_transform(self.root_frame, link_name, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            T_tf = np.eye(4, dtype=np.float32)
            T_tf[:3,:3] = quaternion_to_matrix(q)
            T_tf[:3,3]  = [t.x, t.y, t.z]

            T = T_tf @ T_lv
            mat = QMatrix4x4()
            for i in range(4):
                mat.setRow(i, QVector4D(*T[i,:]))
            item.setTransform(mat)

        mats = []
        for f in frames:
            if f == self.root_frame:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(self.root_frame, f, rclpy.time.Time())
            except:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            M = np.eye(4, dtype=np.float32)
            M[:3,:3] = quaternion_to_matrix(q)
            M[:3,3]  = [t.x, t.y, t.z]
            mats.append(M)
        if not mats:
            return

        pts = np.vstack([M[:3,3] for M in mats])
        self.scatter.setData(pos=pts, size=5, color=(1,1,0,0.8))

        N, L = len(mats), 0.1
        x = np.zeros((2*N,3), dtype=np.float32)
        y = np.zeros_like(x)
        z = np.zeros_like(x)
        for i, M in enumerate(mats):
            o, R = M[:3,3], M[:3,:3]
            x[2*i]   = o; x[2*i+1] = o + R @ np.array([L,0,0], dtype=np.float32)
            y[2*i]   = o; y[2*i+1] = o + R @ np.array([0,L,0], dtype=np.float32)
            z[2*i]   = o; z[2*i+1] = o + R @ np.array([0,0,L], dtype=np.float32)
        self.x_axes.setData(pos=x)
        self.y_axes.setData(pos=y)
        self.z_axes.setData(pos=z)


def main():
    rclpy.init()
    node = rclpy.create_node('tf_viewer')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QApplication(sys.argv)
    viewer = RobotStateViewer(node)
    viewer.show()
    app.exec()

    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

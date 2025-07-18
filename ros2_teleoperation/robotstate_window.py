#!/usr/bin/env python3
import sys
import threading
import collections, collections.abc
import math
import fractions
# Monkey‑patch networkx/urdfpy if needed
collections.Mapping   = collections.abc.Mapping
collections.Set       = collections.abc.Set
collections.Iterable  = collections.abc.Iterable
fractions.gcd         = math.gcd

import numpy as np
# restore deprecated numpy aliases
setattr(np, 'int', int)
setattr(np, 'float', float)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
import tf2_ros

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt6.QtCore    import pyqtSignal, Qt
import pyqtgraph.opengl as gl

def quaternion_to_matrix(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
    ], dtype=np.float32)

class TFViewer(QMainWindow):
    update_signal = pyqtSignal()

    def __init__(self, node: Node, root_frame: str = 'pelvis'):
        super().__init__()
        self.node       = node
        self.root_frame = root_frame
        self.setWindowTitle("TF Frames Viewer (event‑driven)")
        self.resize(800, 600)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.latest_transforms = {}

        w = QWidget()
        self.setCentralWidget(w)
        layout = QVBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)

        self.gl_view = gl.GLViewWidget()
        self.gl_view.opts['distance'] = 1.0
        layout.addWidget(self.gl_view)

        grid = gl.GLGridItem()
        grid.scale(0.1, 0.1, 0.1)
        self.gl_view.addItem(grid)

        self.scatter = gl.GLScatterPlotItem()
        self.gl_view.addItem(self.scatter)

        self.x_axes = gl.GLLinePlotItem(color=(1,0,0,1), width=2, mode='lines')
        self.y_axes = gl.GLLinePlotItem(color=(0,1,0,1), width=2, mode='lines')
        self.z_axes = gl.GLLinePlotItem(color=(0,0,1,1), width=2, mode='lines')
        self.gl_view.addItem(self.x_axes)
        self.gl_view.addItem(self.y_axes)
        self.gl_view.addItem(self.z_axes)

        self.update_signal.connect(self._update_view, Qt.ConnectionType.QueuedConnection)

        self.node.create_subscription(
            TFMessage, '/tf', self._tf_callback, qos_profile=10
        )
        static_qos = QoSProfile(depth=10)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.node.create_subscription(
            TFMessage, '/tf_static', self._tf_callback, qos_profile=static_qos
        )

    def _tf_callback(self, msg: TFMessage):
        if msg.transforms:
            self.update_signal.emit()

    def _update_view(self):
        all_lines = self.tf_buffer.all_frames_as_string().splitlines()
        frame_ids = [L.split()[1] for L in all_lines if L.startswith('Frame ')]

        mats = []
        for child in frame_ids:
            if child == self.root_frame:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.root_frame, child, rclpy.time.Time()
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

            t = tf.transform.translation
            q = tf.transform.rotation
            rot3 = quaternion_to_matrix(q)
            M = np.eye(4, dtype=np.float32)
            M[:3, :3] = rot3
            M[:3,  3] = [t.x, t.y, t.z]
            mats.append(M)

        if not mats:
            return

        pts = np.vstack([M[:3,3] for M in mats])
        self.scatter.setData(pos=pts, size=5, color=(1,1,0,0.8))

        N = len(mats)
        axis_len = 0.1
        x_lines = np.zeros((2*N,3), dtype=np.float32)
        y_lines = np.zeros((2*N,3), dtype=np.float32)
        z_lines = np.zeros((2*N,3), dtype=np.float32)

        for i, M in enumerate(mats):
            origin = M[:3,3]
            R      = M[:3,:3]
            x_end = origin + R @ np.array([axis_len,0,0], dtype=np.float32)
            y_end = origin + R @ np.array([0,axis_len,0], dtype=np.float32)
            z_end = origin + R @ np.array([0,0,axis_len], dtype=np.float32)
            x_lines[2*i]   = origin; x_lines[2*i+1]   = x_end
            y_lines[2*i]   = origin; y_lines[2*i+1]   = y_end
            z_lines[2*i]   = origin; z_lines[2*i+1]   = z_end

        self.x_axes.setData(pos=x_lines)
        self.y_axes.setData(pos=y_lines)
        self.z_axes.setData(pos=z_lines)

def main():
    rclpy.init()
    node = rclpy.create_node('tf_viewer')

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    viewer = TFViewer(node, root_frame='pelvis')
    viewer.show()
    app.exec()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

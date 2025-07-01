#!/usr/bin/env python3
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtCore import QTimer
import pyqtgraph.opengl as gl
import pyqtgraph as pg

class LiDARViewer(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("LiDAR Viewer")
        self.resize(800, 600)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_sub = node.create_subscription(
            PointCloud2, '/pandar_points', self.pc_callback, qos_profile=qos
        )

        widget = QWidget()
        self.setCentralWidget(widget)
        layout = QVBoxLayout(widget)
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.opts['distance'] = 30
        layout.addWidget(self.gl_widget)

        grid = gl.GLGridItem()
        grid.scale(1,1,1)
        self.gl_widget.addItem(grid)

        self.scatter = gl.GLScatterPlotItem(size=2.0, pxMode=True)
        self.gl_widget.addItem(self.scatter)

        self._xyz = np.empty((0, 3), dtype=np.float32)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        self.ros_timer.start(10)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._refresh)
        self.update_timer.start(33)

    def pc_callback(self, msg: PointCloud2):
        gen = pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)
        pts = [(p[0], p[1], p[2]) for p in gen]
        if not pts:
            return
        xyz = np.array(pts, dtype=np.float32)
        n = xyz.shape[0]
        if n > 100_000:
            idx = np.linspace(0, n - 1, 100_000, dtype=int)
            xyz = xyz[idx]
        self._xyz = xyz

    def _refresh(self):
        data = self._xyz
        if data.size == 0:
            return
        
        # Color by height (z)
        z = data[:, 2]
        norm = (z - z.min()) / (z.ptp() + 1e-6)

        # RGBA gradient
        colors = np.empty((norm.size, 4), dtype=np.float32)
        colors[:, 0] = 1.0 - norm
        colors[:, 1] = norm
        colors[:, 2] = 0.2
        colors[:, 3] = 1.0

        self.scatter.setData(pos=data, color=colors)

    def closeEvent(self, event):
        self.cloud_sub.destroy()
        return super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('lidar_viewer')
    app = QApplication(sys.argv)
    viewer = LiDARViewer(node)
    viewer.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

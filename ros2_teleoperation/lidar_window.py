#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox
from PyQt6.QtCore import QTimer
import pyqtgraph.opengl as gl

from ros2_teleoperation.utils.window_style import WindowStyle

class LiDARViewer(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("LiDAR Viewer")
        self.resize(800, 600)

        widget = QWidget()
        self.setCentralWidget(widget)
        layout = QVBoxLayout(widget)

        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.opts['distance'] = 30
        layout.addWidget(self.gl_widget)

        self.combo = QComboBox(self.gl_widget)
        self.combo.setFixedWidth(160)
        self.combo.raise_()
        self.combo.move(self.gl_widget.width() - self.combo.width() - 2, 2)
        self.combo.currentTextChanged.connect(self.change_topic)

        self.cloud_sub = None
        self._xyz = np.empty((0, 3), dtype=np.float32)

        self._populate_topics()

        grid = gl.GLGridItem()
        grid.scale(1, 1, 1)
        self.gl_widget.addItem(grid)
        self.scatter = gl.GLScatterPlotItem(size=2.0, pxMode=True)
        self.gl_widget.addItem(self.scatter)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        self.ros_timer.start(10)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._refresh)
        self.update_timer.start(33)

        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self._populate_topics)
        self.topic_timer.start(1000)

    def showEvent(self, event):
        super().showEvent(event)
        self._reposition_combo()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_combo()

    def _reposition_combo(self):
        margin = 2
        x = self.gl_widget.width() - self.combo.width() - margin
        y = margin
        self.combo.move(x, y)

    def _populate_topics(self):
        current = self.combo.currentText()
        all_topics = self.node.get_topic_names_and_types()
        pc2_topics = [name for name, types in all_topics
                      if 'sensor_msgs/msg/PointCloud2' in types]

        new_items = ['---'] + pc2_topics

        old_items = [self.combo.itemText(i) for i in range(self.combo.count())]
        if old_items == new_items:
            return

        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(new_items)

        if current in new_items:
            self.combo.setCurrentText(current)
        else:
            self.combo.setCurrentIndex(0)
            self.change_topic('---')
        self.combo.blockSignals(False)

    def change_topic(self, topic_name: str):
        if self.cloud_sub is not None:
            try:
                self.node.destroy_subscription(self.cloud_sub)
            except Exception:
                pass
            self.cloud_sub = None

        if topic_name == '---':
            self._xyz = np.empty((0, 3), dtype=np.float32)
        else:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cloud_sub = self.node.create_subscription(
                PointCloud2, topic_name, self.pc_callback, qos_profile=qos
            )
            self._xyz = np.empty((0, 3), dtype=np.float32)

    def pc_callback(self, msg: PointCloud2):
        gen = pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)
        pts = [(p[0], p[1], p[2]) for p in gen]
        if not pts:
            return

        xyz = np.array(pts, dtype=np.float32)
        if xyz.shape[0] > 100_000:
            idx = np.linspace(0, xyz.shape[0] - 1, 100_000, dtype=int)
            xyz = xyz[idx]
        self._xyz = xyz

    def _refresh(self):
        data = self._xyz
        if data.size == 0:
            self.scatter.setData(pos=np.empty((0, 3), dtype=np.float32))
            return

        z = data[:, 2]
        norm = (z - z.min()) / (z.ptp() + 1e-6)

        colors = np.empty((norm.size, 4), dtype=np.float32)
        colors[:, 0] = 1.0 - norm
        colors[:, 1] = norm
        colors[:, 2] = 0.2
        colors[:, 3] = 1.0

        self.scatter.setData(pos=data, color=colors)

    def closeEvent(self, event):
        if self.cloud_sub is not None:
            self.node.destroy_subscription(self.cloud_sub)
        return super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('lidar_viewer')
    app = QApplication(sys.argv)
    WindowStyle(app)
    viewer = LiDARViewer(node)
    viewer.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

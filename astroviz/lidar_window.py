#!/usr/bin/env python3
import sys, os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from ament_index_python.packages import get_package_share_directory

import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QComboBox, QPushButton
)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QIcon
import pyqtgraph.opengl as gl
import pyqtgraph as pg

from astroviz.utils.window_style import DarkStyle, LightStyle

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
ICONS_DIR  = os.path.join(_PKG_DIR, 'icons')

class LiDARViewer(QMainWindow):
    def showEvent(self, event):
        super().showEvent(event)

        self._position_overlays()

    
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("LiDAR Viewer")
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, 'astroviz_icon.png')))
        self.resize(800, 600)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_sub = None
        self._xyz = np.empty((0, 3), dtype=np.float32)

        widget = QWidget()
        self.setCentralWidget(widget)
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0,0,0,0)

        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.opts['distance'] = 30
        layout.addWidget(self.gl_widget)

        self._mousePress = self.gl_widget.mousePressEvent
        self._mouseMove = self.gl_widget.mouseMoveEvent
        self._mouseWheel = self.gl_widget.wheelEvent

        self.combo = QComboBox(self.gl_widget)
        self.combo.setFixedWidth(160)
        self.combo.raise_()
        self.combo.currentTextChanged.connect(self.change_topic)

        self.btn_2d = QPushButton("2D", self.gl_widget)
        self.btn_2d.setCheckable(True)
        self.btn_2d.raise_()
        self.btn_2d.clicked.connect(self.toggle_2d_view)

        self._position_overlays()

        self._populate_topics()

        grid = gl.GLGridItem()
        grid.scale(1,1,1)
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

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._position_overlays()

    def _position_overlays(self):
        margin = 5
        x_combo = margin
        y_combo = margin
        self.combo.move(x_combo, y_combo)
        x_btn = x_combo + self.combo.width() + margin
        self.btn_2d.move(x_btn, y_combo)

    def _populate_topics(self):
        current = self.combo.currentText()
        all_topics = self.node.get_topic_names_and_types()
        pc2_topics = [name for name, types in all_topics if 'sensor_msgs/msg/PointCloud2' in types]
        items = ['---'] + pc2_topics
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

    def change_topic(self, topic_name: str):
        if self.cloud_sub:
            try: self.node.destroy_subscription(self.cloud_sub)
            except: pass
            self.cloud_sub = None
        if topic_name == '---':
            self._xyz = np.empty((0,3), dtype=np.float32)
        else:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cloud_sub = self.node.create_subscription(
                PointCloud2, topic_name, self.pc_callback, qos_profile=qos)
            self._xyz = np.empty((0,3), dtype=np.float32)

    def pc_callback(self, msg: PointCloud2):
        xyz = np.fromiter(
            ((p[0], p[1], p[2]) for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)),
            dtype=np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        ).view(np.float32).reshape(-1, 3)
        if xyz.shape[0] > 100_000:
            idx = np.linspace(0, xyz.shape[0]-1, 100_000, dtype=int)
            xyz = xyz[idx]
        self._xyz = xyz

    def _refresh(self):
        if self._xyz.size == 0:
            self.scatter.setData(pos=np.empty((0,3), dtype=np.float32))
            return
        if hasattr(self, '_last_shape') and self._xyz.shape == self._last_shape:
            return
        self._last_shape = self._xyz.shape
        z = self._xyz[:,2]
        norm = (z - z.min())/(z.ptp()+1e-6)
        colors = np.empty((norm.size,4), dtype=np.float32)
        colors[:,0] = 1 - norm
        colors[:,1] = norm
        colors[:,2] = 0.2
        colors[:,3] = 1.0
        self.scatter.setData(pos=self._xyz, color=colors)

    def toggle_2d_view(self):
        if self.btn_2d.isChecked():
            self.btn_2d.setStyleSheet("background-color: green; color: white;")
            self.gl_widget.setCameraPosition(elevation=90, azimuth=0)
            def locked_mouse_move(ev):
                self._mouseMove(ev)
                az = self.gl_widget.opts.get('azimuth', 0)
                self.gl_widget.setCameraPosition(elevation=90, azimuth=az)
            self.gl_widget.mouseMoveEvent = locked_mouse_move
        else:
            self.btn_2d.setStyleSheet("")
            self.gl_widget.setCameraPosition(elevation=30, azimuth=45)
            self.gl_widget.mouseMoveEvent = self._mouseMove
            self.gl_widget.mousePressEvent = self._mousePress
            self.gl_widget.wheelEvent = self._mouseWheel

    def closeEvent(self, event):
        if self.cloud_sub:
            self.node.destroy_subscription(self.cloud_sub)
        return super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('lidar_viewer')
    app = QApplication(sys.argv)
    DarkStyle(app)
    viewer = LiDARViewer(node)
    viewer.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

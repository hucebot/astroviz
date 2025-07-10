#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point

import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QComboBox, QPushButton
)
from PyQt6.QtCore import QTimer, QRectF
from PyQt6.QtGui import QFont

import pyqtgraph as pg
from pyqtgraph import ScatterPlotItem, TextItem, PlotDataItem

from ros2_teleoperation.utils.window_style import DarkStyle

class GridMapViewer(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("GridMap Viewer")
        self.resize(800, 600)

        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.wp_pub = self.node.create_publisher(
            Path, '/gridmap/waypoints', pub_qos)

        self.combo = QComboBox()
        self.combo.setFixedWidth(200)
        self.combo.currentTextChanged.connect(self.change_topic)

        self.btn_waypoints = QPushButton("Add/Remove Waypoints")
        self.btn_waypoints.setCheckable(True)
        self.btn_waypoints.clicked.connect(self.toggle_waypoint_mode)

        self.btn_publish = QPushButton("Publish Waypoints")
        self.btn_publish.clicked.connect(self.publish_waypoints)

        top_bar = QWidget()
        hlay = QHBoxLayout(top_bar)
        hlay.setContentsMargins(5,5,5,5)
        hlay.addWidget(self.combo)
        hlay.addWidget(self.btn_waypoints)
        hlay.addWidget(self.btn_publish)
        hlay.addStretch()

        self.graph_widget = pg.GraphicsLayoutWidget()
        self.view_box = self.graph_widget.addViewBox()
        self.view_box.setAspectLocked(True)

        self.img_item = pg.ImageItem()
        self.view_box.addItem(self.img_item)

        self.wp_scatter = ScatterPlotItem(
            size=10, pen=pg.mkPen('r'), brush=pg.mkBrush('r')
        )
        self.view_box.addItem(self.wp_scatter)

        self.wp_line = PlotDataItem(pen=pg.mkPen('b', width=2))
        self.view_box.addItem(self.wp_line)

        self.wp_labels = []

        central = QWidget()
        self.setCentralWidget(central)
        main_v = QVBoxLayout(central)
        main_v.setContentsMargins(0,0,0,0)
        main_v.addWidget(top_bar)
        main_v.addWidget(self.graph_widget)

        self.sub = None
        self.map_info = None
        self.waypoints = []
        self._first_map = True

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(10)

        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self._populate_topics)
        self.topic_timer.start(1000)

        self._populate_topics()

    def _populate_topics(self):
        topics = self.node.get_topic_names_and_types()
        grids = [n for n, t in topics if 'nav_msgs/msg/OccupancyGrid' in t]
        items = ['---'] + sorted(grids)
        current = [self.combo.itemText(i) for i in range(self.combo.count())]
        if items != current:
            self.combo.blockSignals(True)
            self.combo.clear()
            self.combo.addItems(items)
            self.combo.blockSignals(False)

    def change_topic(self, topic_name: str):
        if self.sub:
            self.node.destroy_subscription(self.sub)
            self.sub = None
        self.img_item.clear()
        for lbl in self.wp_labels:
            self.view_box.removeItem(lbl)
        self.wp_labels.clear()
        self.waypoints.clear()
        self.wp_scatter.setData([])
        self.wp_line.setData([], [])
        self._first_map = True

        if topic_name != '---':
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.sub = self.node.create_subscription(
                OccupancyGrid, topic_name, self.og_callback, qos)

    def og_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        h, w = self.map_info.height, self.map_info.width
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        img = np.empty((h, w), dtype=np.uint8)
        img[data == -1] = 127
        img[data ==  0] = 255
        img[data ==100] =   0

        self.img_item.setImage(img.T, autoLevels=False)
        orig = self.map_info.origin.position
        res = self.map_info.resolution
        rect = QRectF(orig.x, orig.y, w * res, h * res)
        self.img_item.setRect(rect)

        if self._first_map:
            self.view_box.autoRange()
            self._first_map = False

    def toggle_waypoint_mode(self, checked: bool):
        if checked:
            self.btn_waypoints.setStyleSheet("background-color: green; color: white;")
            self.view_box.scene().sigMouseClicked.connect(self.on_click)
        else:
            self.btn_waypoints.setStyleSheet("")
            self.view_box.scene().sigMouseClicked.disconnect(self.on_click)

    def _refresh_waypoints(self):
        pts = [{'x': wp[0], 'y': wp[1]} for wp in self.waypoints]
        self.wp_scatter.setData(pts)
        if len(self.waypoints) >= 2:
            xs, ys = zip(*self.waypoints)
            self.wp_line.setData(xs, ys)
        else:
            self.wp_line.setData([], [])
        for lbl in self.wp_labels:
            self.view_box.removeItem(lbl)
        self.wp_labels.clear()
        for idx, (wx, wy) in enumerate(self.waypoints, start=1):
            label = TextItem(str(idx), anchor=(0.5, 1.0), fill=pg.mkBrush(50,50,50,200))
            label.setColor('w')
            font = QFont(); font.setPointSize(16)
            label.setFont(font)
            label.setPos(wx, wy + self.map_info.resolution * 0.1)
            self.view_box.addItem(label)
            self.wp_labels.append(label)

    def publish_waypoints(self):
        if not self.waypoints or self.map_info is None:
            return
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        path_msg.poses = []
        for x, y in self.waypoints:
            ps = PoseStamped(header=path_msg.header)
            ps.pose.position = Point(x=x, y=y, z=0.0)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.wp_pub.publish(path_msg)

    def on_click(self, event):
        if not self.btn_waypoints.isChecked() or self.map_info is None:
            return
        pos = event.scenePos()
        mp = self.view_box.mapSceneToView(pos)
        orig = self.map_info.origin.position
        res = self.map_info.resolution
        thr = res * 0.5
        for i, (wx, wy) in enumerate(self.waypoints):
            if math.hypot(mp.x() - wx, mp.y() - wy) < thr:
                del self.waypoints[i]
                self._refresh_waypoints()
                return
        ci = int((mp.x() - orig.x) / res)
        cj = int((mp.y() - orig.y) / res)
        if 0 <= ci < self.map_info.width and 0 <= cj < self.map_info.height:
            wx = orig.x + ci * res + res / 2
            wy = orig.y + cj * res + res / 2
            self.waypoints.append((wx, wy))
            self._refresh_waypoints()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('gridmap_waypoint_viewer_ids')
    app = QApplication(sys.argv)
    DarkStyle(app)
    viewer = GridMapViewer(node)
    viewer.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
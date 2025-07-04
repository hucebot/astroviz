#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QVBoxLayout
)
from PyQt6.QtCore import Qt, QTimer

from ros2_teleoperation.gps_map_window import GPSMapWindow
from ros2_teleoperation.camera_window import CameraViewer
from ros2_teleoperation.imu_window import MainWindow as IMUWindow
from ros2_teleoperation.lidar_window import LiDARViewer

from ros2_teleoperation.utils.window_style import WindowStyle


class TeleoperationDashboard(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Teleoperation Dashboard")
        self.setGeometry(100, 100, 1920, 1080)
        self.setMinimumSize(1200, 800)

        self.gps_map     = GPSMapWindow(node)
        self.camera_view = CameraViewer(node)
        self.imu_view    = IMUWindow(node)
        self.lidar_view  = LiDARViewer(node)

        for win in [self.gps_map, self.camera_view, self.imu_view, self.lidar_view]:
            win.hide()

        top_split = QSplitter(Qt.Orientation.Horizontal)
        top_split.addWidget(self.gps_map.centralWidget())
        top_split.addWidget(self.lidar_view.centralWidget())
        top_split.setSizes([960, 960])

        bottom_split = QSplitter(Qt.Orientation.Horizontal)
        bottom_split.addWidget(self.camera_view.centralWidget())
        bottom_split.addWidget(self.imu_view.centralWidget())
        bottom_split.setSizes([960, 960])

        main_split = QSplitter(Qt.Orientation.Vertical)
        main_split.addWidget(top_split)
        main_split.addWidget(bottom_split)
        main_split.setSizes([540, 540])

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(main_split)

        self.setCentralWidget(container)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    WindowStyle(app)

    node = rclpy.create_node('teleoperation_node')
    dashboard = TeleoperationDashboard(node)
    dashboard.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(30)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

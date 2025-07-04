#!/usr/bin/env python3

import sys
import cv2
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QComboBox,
    QHBoxLayout, QPushButton
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QImage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ros2_teleoperation.utils.window_style import WindowStyle


class CameraViewer(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Camera Viewer - ROS2")
        
        self.bridge = CvBridge()
        self.image_sub = None
        self.rotation_angle = 0

        self.central = QWidget()
        self.setCentralWidget(self.central)
        self.layout = QVBoxLayout(self.central)

        self.combo = QComboBox()
        self.combo.setFixedWidth(250)
        self.combo.currentTextChanged.connect(self.change_image_topic)
        self.layout.addWidget(self.combo, alignment=Qt.AlignmentFlag.AlignLeft)

        self.image_label = QLabel("Waiting for image...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(self.image_label)

        btn_layout = QHBoxLayout()
        self.btn_left = QPushButton("⟲ 90° Left")
        self.btn_left.clicked.connect(self.rotate_left)
        btn_layout.addWidget(self.btn_left, alignment=Qt.AlignmentFlag.AlignLeft)

        btn_layout.addStretch()

        self.btn_right = QPushButton("90° Right ⟳")
        self.btn_right.clicked.connect(self.rotate_right)
        btn_layout.addWidget(self.btn_right, alignment=Qt.AlignmentFlag.AlignRight)

        self.layout.addLayout(btn_layout)

        self.topic_timer = QTimer()
        self.topic_timer.timeout.connect(self.update_image_topics)
        self.topic_timer.start(1000)

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(30)

    def rotate_left(self):
        self.rotation_angle = (self.rotation_angle - 90) % 360

    def rotate_right(self):
        self.rotation_angle = (self.rotation_angle + 90) % 360

    def update_image_topics(self):
        current = self.combo.currentText()
        all_topics = self.node.get_topic_names_and_types()
        image_topics = [
            name for name, types in all_topics
            if 'sensor_msgs/msg/Image' in types
        ]
        items = ['---'] + image_topics

        old = [self.combo.itemText(i) for i in range(self.combo.count())]
        if old == items:
            return

        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(items)
        if current in items:
            self.combo.setCurrentText(current)
        else:
            self.combo.setCurrentIndex(0)
            self.change_image_topic('---')
        self.combo.blockSignals(False)

    def change_image_topic(self, topic_name: str):
        if self.image_sub is not None:
            try:
                self.node.destroy_subscription(self.image_sub)
            except Exception:
                pass
            self.image_sub = None

        if topic_name == '---':
            return

        self.image_sub = self.node.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            QoSProfile(depth=10)
        )

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.rotation_angle == 90:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotation_angle == 180:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            elif self.rotation_angle == 270:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_BGR888)
            pixmap = QPixmap.fromImage(q_image)
            self.image_label.setPixmap(pixmap.scaled(
                self.image_label.size(), Qt.AspectRatioMode.KeepAspectRatio
            ))
        except Exception as e:
            self.node.get_logger().warn(f"Image conversion failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    WindowStyle(app)
    
    node = rclpy.create_node('camera_viewer')
    window = CameraViewer(node)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

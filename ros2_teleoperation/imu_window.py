#!/usr/bin/env python3
import sys
import math

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QComboBox
)
from PyQt6.QtGui import QPainter, QColor, QPen, QPainterPath
from PyQt6.QtCore import Qt, QRect, QRectF, QTimer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu

from ros2_teleoperation.utils.window_style import WindowStyle


class ArtificialHorizon(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.roll = 0.0
        self.pitch = 0.0

    def set_orientation(self, roll: float, pitch: float):
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        size = min(w, h)
        size_i = int(size)
        x0 = (w - size_i) // 2
        y0 = (h - size_i) // 2
        rect = QRect(x0, y0, size_i, size_i)

        pitch_offset = (self.pitch / (math.pi / 2)) * (size / 2)

        path = QPainterPath()
        path.addEllipse(QRectF(rect))


        painter.save()
        painter.setClipPath(path)
        painter.translate(w / 2, h / 2 + pitch_offset)
        painter.rotate(-math.degrees(self.roll))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(135, 206, 235))
        painter.drawRect(-size, -size, size * 2, size)
        painter.setBrush(QColor(139, 69, 19))
        painter.drawRect(-size, 0, size * 2, size)
        painter.restore()

        pen = QPen(Qt.GlobalColor.black, 3)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(rect)


class MainWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle('IMU - ROS2 Artificial Horizon')
        self.setGeometry(100, 100, 400, 400)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.horizon = ArtificialHorizon()
        layout.addWidget(self.horizon)

        self.combo = QComboBox(self.centralWidget())
        self.combo.setFixedWidth(150)
        self.combo.raise_()
        self.combo.currentTextChanged.connect(self.change_imu_topic)

        self.imu_sub = None
        self._populate_imu_topics()

        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self._populate_imu_topics)
        self.topic_timer.start(1000)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(50)

    def showEvent(self, event):
        super().showEvent(event)
        self._reposition_combo()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_combo()

    def _reposition_combo(self):
        margin = 5
        cw = self.centralWidget().width()
        x = cw - self.combo.width() - margin
        y = margin
        self.combo.move(x, y)

    def _populate_imu_topics(self):
        current = self.combo.currentText()
        all_topics = self.node.get_topic_names_and_types()
        imu_topics = [
            name for name, types in all_topics
            if 'sensor_msgs/msg/Imu' in types
        ]
        items = ['---'] + imu_topics

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
            self.change_imu_topic('---')
        self.combo.blockSignals(False)

    def change_imu_topic(self, topic_name: str):
        if self.imu_sub is not None:
            try:
                self.node.destroy_subscription(self.imu_sub)
            except Exception:
                pass
            self.imu_sub = None

        if topic_name == '---':
            return

        self.imu_sub = self.node.create_subscription(
            Imu,
            topic_name,
            self.imu_callback,
            QoSProfile(depth=10)
        )

    def imu_callback(self, msg: Imu):
        qx, qy, qz, qw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        self.horizon.set_orientation(roll, pitch)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    WindowStyle(app)
    node = rclpy.create_node('imu_window')
    window = MainWindow(node)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

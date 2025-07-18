#!/usr/bin/env python3

import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout
)
from PyQt6.QtCore import Qt, QTimer, QPointF, QSize
from PyQt6.QtGui import QPainter, QPen, QColor, QMouseEvent

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from astroviz.camera_window import CameraViewer
from astroviz.utils.window_style import DarkStyle


class JoystickWidget(QWidget):
    def __init__(self, parent, radius=80, stick_radius=30, margin=40):
        super().__init__(parent)
        self.radius = radius
        self.stick_radius = stick_radius
        self.margin = margin
        size = 2 * self.radius
        self.setFixedSize(QSize(size, size))

        self.active = False
        self.stick_pos = QPointF(0, 0)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        center = QPointF(w/2, h/2)

        pen = QPen(QColor(255,255,255,180), 4)
        painter.setPen(pen)
        painter.setBrush(QColor(255,255,255,40))
        painter.drawEllipse(center, self.radius, self.radius)

        painter.setBrush(QColor(255,255,255,100))
        painter.drawEllipse(center + self.stick_pos, self.stick_radius, self.stick_radius)

    def mousePressEvent(self, ev: QMouseEvent):
        pos = QPointF(ev.position())
        center = QPointF(self.width()/2, self.height()/2)
        if (pos - center).manhattanLength() <= self.radius:
            self.active = True
            ev.accept()
        else:
            ev.ignore()

    def mouseMoveEvent(self, ev: QMouseEvent):
        if not self.active:
            ev.ignore()
            return
        pos = QPointF(ev.position())
        center = QPointF(self.width()/2, self.height()/2)
        offset = pos - center
        length = offset.manhattanLength()
        max_offset = self.radius - self.stick_radius
        if length > max_offset:
            offset = offset / length * max_offset
        self.stick_pos = offset
        self.update()
        ev.accept()

    def mouseReleaseEvent(self, ev: QMouseEvent):
        if self.active:
            self.active = False
            self.stick_pos = QPointF(0, 0)
            self.update()
            ev.accept()
        else:
            ev.ignore()

    def normalized(self):
        max_offset = self.radius - self.stick_radius
        x = self.stick_pos.x() / max_offset
        y = -self.stick_pos.y() / max_offset
        return float(x), float(y)


class TeleoperationViewer(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Teleoperation Viewer")
        self.setGeometry(100, 100, 900, 720)

        cam_viewer = CameraViewer(node)
        cam_widget = cam_viewer.centralWidget()

        self.container = QWidget()
        layout = QVBoxLayout(self.container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(cam_widget)
        self.setCentralWidget(self.container)

        self.joy_left  = JoystickWidget(self.container)
        self.joy_right = JoystickWidget(self.container)


        self.joy_pub = node.create_publisher(Joy, '/teleop/joy', 10)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(30)

        self.pub_timer = QTimer(self)
        self.pub_timer.timeout.connect(self.publish_joy)
        self.pub_timer.start(50)

    def publish_joy(self):
        left_x, left_y   = self.joy_left.normalized()
        right_x, right_y = self.joy_right.normalized()
        msg = Joy()
        msg.axes = [left_x, left_y, right_x, right_y]
        msg.buttons = []
        self.joy_pub.publish(msg)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        w = self.container.width()
        h = self.container.height()
        m = self.joy_left.margin
        r = self.joy_left.radius
        size = 2 * r

        self.joy_left.move(m, h - m - size)
        self.joy_right.move(w - m - size, h - m - size)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)

    node = rclpy.create_node('teleoperation_viewer')
    win = TeleoperationViewer(node)
    win.show()
    app.exec()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import sys
import os
import json
import threading
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter,
    QVBoxLayout, QHBoxLayout, QComboBox, QLabel, QPushButton
)
from PyQt6.QtCore import Qt, QTimer

from ros2_teleoperation.gps_map_window import GPSMapWindow
from ros2_teleoperation.camera_window import CameraViewer
from ros2_teleoperation.imu_window import MainWindow as IMUWindow
from ros2_teleoperation.lidar_window import LiDARViewer
from ros2_teleoperation.teleoperation_window import TeleoperationViewer
from ros2_teleoperation.utils.window_style import DarkStyle, LightStyle
from ros2_teleoperation.utils.windows_implemented import VIEW_TYPES
from termcolor import colored


def _find_src_config():
    cwd = os.getcwd()
    while True:
        candidate = os.path.join(cwd, 'src', 'ros2_teleoperation', 'config')
        if os.path.isdir(candidate):
            return candidate
        parent = os.path.dirname(cwd)
        if parent == cwd:
            break
        cwd = parent
    return None

_src_config = _find_src_config()
if _src_config:
    _CONFIG_DIR = _src_config
else:
    _CONFIG_DIR = os.path.join(
        get_package_share_directory('ros2_teleoperation'), 'config'
    )
os.makedirs(_CONFIG_DIR, exist_ok=True)
CONFIG_PATH = os.path.join(_CONFIG_DIR, 'dashboard_config.json')


class Panel(QWidget):
    def __init__(self, node: Node, initial_view: str, parent=None):
        super().__init__(parent)
        self.node = node
        self.current_widget = None

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0,0,0,0)
        main_layout.setSpacing(2)

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(2,2,2,2)
        header_layout.setSpacing(5)
        header_layout.addStretch(1)
        header_layout.addWidget(QLabel("View:"))
        self.combo = QComboBox()
        self.combo.addItems(VIEW_TYPES.keys())
        self.combo.setCurrentText(initial_view)
        self.combo.currentTextChanged.connect(self.change_view)
        header_layout.addWidget(self.combo)
        main_layout.addWidget(header)

        self.content_area = QWidget()
        content_layout = QVBoxLayout(self.content_area)
        content_layout.setContentsMargins(0,0,0,0)
        content_layout.setSpacing(0)
        main_layout.addWidget(self.content_area)

        self.change_view(initial_view)

    def change_view(self, view_name: str):
        if self.current_widget:
            self.current_widget.setParent(None)
            self.current_widget.hide()
        view_class = VIEW_TYPES[view_name]
        widget = view_class(self.node)
        widget.setParent(self.content_area)
        widget.setWindowFlags(Qt.WindowType.Widget)
        widget.show()

        layout = self.content_area.layout()
        while layout.count():
            old = layout.takeAt(0).widget()
            if old:
                old.setParent(None)
        layout.addWidget(widget)
        self.current_widget = widget


class TeleoperationDashboard(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self.node = node
        self.setWindowTitle("Teleoperation Dashboard")
        # take dimensions from the screen
        screen = QApplication.primaryScreen()
        if screen:
            geometry = screen.geometry()
            self.setGeometry(geometry)
        else:
            self.setGeometry(100,100,1920,1080)
        self.setMinimumSize(1200,800)

        self.save_btn = QPushButton("Save Config")
        self.save_btn.clicked.connect(self.save_config)
        self.load_btn = QPushButton("Load Config")
        self.load_btn.clicked.connect(self.load_config)

        toolbar = QWidget()
        tlay = QHBoxLayout(toolbar)
        tlay.setContentsMargins(5,5,5,5)
        tlay.setSpacing(10)
        tlay.addWidget(self.save_btn)
        tlay.addWidget(self.load_btn)

        self.close_btn = QPushButton('✕')
        self.close_btn.setStyleSheet(
            'background-color: red; color: white; border: none; font-weight: bold;'
        )
        self.close_btn.clicked.connect(self.close)
        tlay.addStretch(1)
        tlay.addWidget(self.close_btn)

        self.panel_tl = Panel(node, 'GPS Map')
        self.panel_tr = Panel(node, 'LiDAR')
        self.panel_bl = Panel(node, 'Camera')
        self.panel_bm = Panel(node, 'IMU')
        self.panel_br = Panel(node, 'IMU')

        self.top_split = QSplitter(Qt.Orientation.Horizontal)
        self.top_split.addWidget(self.panel_tl)
        self.top_split.addWidget(self.panel_tr)
        self.top_split.setSizes([960,960])

        self.bottom_split = QSplitter(Qt.Orientation.Horizontal)
        self.bottom_split.addWidget(self.panel_bl)
        self.bottom_split.addWidget(self.panel_bm)
        self.bottom_split.addWidget(self.panel_br)
        self.bottom_split.setSizes([640,640,640])

        self.main_split = QSplitter(Qt.Orientation.Vertical)
        self.main_split.addWidget(self.top_split)
        self.main_split.addWidget(self.bottom_split)
        self.main_split.setSizes([540,540])

        central = QWidget()
        clayout = QVBoxLayout(central)
        clayout.setContentsMargins(0,0,0,0)
        clayout.addWidget(toolbar)
        clayout.addWidget(self.main_split)
        self.setCentralWidget(central)

        self.load_config()

    def save_config(self):
        conf = {
            'views': [
                self.panel_tl.combo.currentText(),
                self.panel_tr.combo.currentText(),
                self.panel_bl.combo.currentText(),
                self.panel_bm.combo.currentText(),
                self.panel_br.combo.currentText(),
            ],
            'splitter_sizes': {
                'top': self.top_split.sizes(),
                'bottom': self.bottom_split.sizes(),
                'main': self.main_split.sizes(),
            }
        }
        with open(CONFIG_PATH, 'w') as f:
            json.dump(conf, f, indent=2)
        self.node.get_logger().info(colored("Configuration saved successfully!", 'green'))

    def load_config(self):
        if not os.path.isfile(CONFIG_PATH):
            return
        try:
            with open(CONFIG_PATH,'r') as f:
                conf = json.load(f)
            views = conf.get('views', [])
            panels = [
                self.panel_tl, self.panel_tr,
                self.panel_bl, self.panel_bm, self.panel_br
            ]
            for panel, name in zip(panels, views):
                if name in VIEW_TYPES:
                    panel.combo.setCurrentText(name)
            sizes = conf.get('splitter_sizes', {})
            if 'top' in sizes:
                self.top_split.setSizes(sizes['top'])
            if 'bottom' in sizes:
                self.bottom_split.setSizes(sizes['bottom'])
            if 'main' in sizes:
                self.main_split.setSizes(sizes['main'])
            self.node.get_logger().info(colored("Configuration loaded successfully!", 'green'))
        except Exception as e:
            self.node.get_logger().error(colored(f"Failed to load config: {e}", 'red'))

    def remove_view_option(self, view_name: str):
        for panel in (self.panel_tl, self.panel_tr,
                      self.panel_bl, self.panel_bm, self.panel_br):
            idx = panel.combo.findText(view_name)
            if idx != -1:
                panel.combo.removeItem(idx)

    def add_view_option(self, view_name: str):
        if view_name not in VIEW_TYPES:
            return
        for panel in (self.panel_tl, self.panel_tr,
                      self.panel_bl, self.panel_bm, self.panel_br):
            if panel.combo.findText(view_name) == -1:
                panel.combo.addItem(view_name)

    def hide_panel(self, position: str):
        panel = getattr(self, f'panel_{position}', None)
        if not panel:
            return
        splitter = self.top_split if position in ('tl','tr') else self.bottom_split
        splitter.removeWidget(panel)
        panel.setParent(None)

    def show_panel(self, position: str):
        panel = getattr(self, f'panel_{position}', None)
        if not panel:
            return
        splitter = self.top_split if position in ('tl','tr') else self.bottom_split
        if panel.parent() is None:
            splitter.addWidget(panel)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    LightStyle(app)
    node = rclpy.create_node('teleoperation_node')
    dash = TeleoperationDashboard(node)
    dash.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(30)
    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# main_window.py

import sys
import os
import json
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter,
    QVBoxLayout, QHBoxLayout, QComboBox, QLabel, QPushButton
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QIcon, QPixmap

from astroviz.gps_map_window import GPSMapWindow
from astroviz.camera_window import CameraViewer
from astroviz.imu_window import MainWindow as IMUWindow
from astroviz.lidar_window import LiDARViewer
from astroviz.teleoperation_window import TeleoperationViewer

from astroviz.utils.window_style import DarkStyle, LightStyle
from astroviz.utils.windows_implemented import VIEW_TYPES
from astroviz.utils._find import _find_pkg, _find_src_config
from termcolor import colored


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
        self.actual_style = 'dark'
        self.setWindowTitle("Teleoperation Dashboard")
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, 'astroviz_icon.png')))


        screen = QApplication.primaryScreen()
        if screen:
            self.setGeometry(screen.geometry())
        else:
            self.setGeometry(100,100,1920,1080)
        self.setMinimumSize(1200,800)

        self.icon_sun  = QIcon(QPixmap(os.path.join(ICONS_DIR, 'light_style.png')))
        self.icon_moon = QIcon(QPixmap(os.path.join(ICONS_DIR, 'dark_style.png')))

        self.save_btn         = QPushButton("Save Config")
        self.load_btn         = QPushButton("Load Config")
        self.change_style_btn = QPushButton()
        self.change_style_btn.setIcon(self.icon_sun)
        self.close_btn        = QPushButton('✕')
        self.close_btn.setStyleSheet(
            'background-color: red; color: white; border: none; font-weight: bold;'
        )

        self.save_btn.clicked.connect(self.save_config)
        self.load_btn.clicked.connect(self.load_config)
        self.change_style_btn.clicked.connect(self.toggle_style)
        self.close_btn.clicked.connect(self.close)

        toolbar = QWidget()
        tlay = QHBoxLayout(toolbar)
        tlay.setContentsMargins(5,5,5,5)
        tlay.setSpacing(10)
        tlay.addWidget(self.save_btn)
        tlay.addWidget(self.load_btn)
        tlay.addStretch(1)
        tlay.addWidget(self.change_style_btn)
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

    def toggle_style(self):
        app = QApplication.instance()
        if self.actual_style == 'dark':
            LightStyle(app)
            self.actual_style = 'light'
            self.change_style_btn.setIcon(self.icon_moon)
            self.node.get_logger().info(colored("Switched to Light Style", 'green'))
        else:
            DarkStyle(app)
            self.actual_style = 'dark'
            self.change_style_btn.setIcon(self.icon_sun)
            self.node.get_logger().info(colored("Switched to Dark Style", 'green'))

    def save_config(self):
        conf = {
            'views': [
                self.panel_tl.combo.currentText(),
                self.panel_tr.combo.currentText(),
                self.panel_bl.combo.currentText(),
                self.panel_bm.combo.currentText(),
                self.panel_br.combo.currentText(),
            ],
            'actual_style': self.actual_style,
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
            if 'actual_style' in conf:
                app = QApplication.instance()
                if conf['actual_style'] == 'dark':
                    DarkStyle(app)
                    self.actual_style = 'dark'
                    self.change_style_btn.setIcon(self.icon_sun)
                else:
                    LightStyle(app)
                    self.actual_style = 'light'
                    self.change_style_btn.setIcon(self.icon_moon)

            panels = [
                self.panel_tl, self.panel_tr,
                self.panel_bl, self.panel_bm, self.panel_br
            ]
            for panel, name in zip(panels, conf.get('views', [])):
                if name in VIEW_TYPES:
                    panel.combo.setCurrentText(name)

            sizes = conf.get('splitter_sizes', {})
            if 'top' in sizes:    self.top_split.setSizes(sizes['top'])
            if 'bottom' in sizes: self.bottom_split.setSizes(sizes['bottom'])
            if 'main' in sizes:   self.main_split.setSizes(sizes['main'])

            self.node.get_logger().info(colored("Configuration loaded successfully!", 'green'))
        except Exception as e:
            self.node.get_logger().error(colored(f"Failed to load config: {e}", 'red'))

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

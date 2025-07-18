#!/usr/bin/env python3
import sys
import time
import socket
import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QApplication, QComboBox
)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont, QPalette, QColor, QIcon
import pyqtgraph as pg

from ament_index_python.packages import get_package_share_directory

from ping3 import ping
from ping3.errors import PingError
from collections import deque

from astroviz.utils.window_style import DarkStyle
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

class GraphViewer(QWidget):
    def __init__(self, node: Node, parent=None):
        super().__init__(parent)
        self.node = node
        self.setWindowTitle('System Health')
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, 'astroviz_icon.png')))
        self.max_points = 100

        self.data = {
            'voltage':    deque(maxlen=self.max_points),
            'current':    deque(maxlen=self.max_points),
            'percentage': deque(maxlen=self.max_points),
            'latency':    deque(maxlen=self.max_points),
        }
        self.ping_host = ''

        self._init_ui()
        self.batt_sub = None

        self.batt_topic_timer = QTimer(self)
        self.batt_topic_timer.timeout.connect(self._update_batt_topics)
        self.batt_topic_timer.start(500)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update)
        self.update_timer.start(500)

    def _init_ui(self):
        font = QFont(); font.setPointSize(10)
        layout = QVBoxLayout(self)

        hl = QHBoxLayout()
        hl.addWidget(QLabel('Battery Topic:'))
        self.batt_combo = QComboBox()
        self.batt_combo.addItem('---')
        self.batt_combo.currentTextChanged.connect(self._change_batt_topic)
        hl.addWidget(self.batt_combo)
        layout.addLayout(hl)

        hl = QHBoxLayout()
        hl.addWidget(QLabel('Ping Host/IP:'))
        #en blanco el placeholder
        self.ip_edit = QLineEdit(placeholderText='127.0.0.1')
        pal = self.ip_edit.palette()
        pal.setColor(QPalette.ColorRole.PlaceholderText, QColor('white'))
        self.ip_edit.setPalette(pal)
        hl.addWidget(self.ip_edit)
        btn = QPushButton('Set IP')
        btn.clicked.connect(self._on_ip_entered)
        hl.addWidget(btn)
        layout.addLayout(hl)


        grid = pg.GraphicsLayoutWidget()
        p1 = grid.addPlot(title='Voltage (V)');    self.plots = {'voltage': p1.plot(pen='y')}
        p2 = grid.addPlot(title='Current (A)');    grid.nextRow(); self.plots['current'] = p2.plot(pen='r')
        p3 = grid.addPlot(title='Charge (%)');     self.plots['percentage'] = p3.plot(pen='g')
        p4 = grid.addPlot(title='Latency (ms)');   self.plots['latency'] = p4.plot(pen='c')
        layout.addWidget(grid)

    def _on_ip_entered(self):
        self.ping_host = self.ip_edit.text().strip()
        ms = self._ping_latency(self.ping_host)
        self.ip_edit.setStyleSheet(f"border:2px solid {'green' if ms>0 else 'red'};")
        self._append('latency', ms)

    def _update_batt_topics(self):
        curr = self.batt_combo.currentText()
        tops = [n for n,t in self.node.get_topic_names_and_types()
                if 'sensor_msgs/msg/BatteryState' in t]
        items = ['---'] + tops
        if [self.batt_combo.itemText(i) for i in range(self.batt_combo.count())] != items:
            self.batt_combo.blockSignals(True)
            self.batt_combo.clear()
            self.batt_combo.addItems(items)
            self.batt_combo.setCurrentText(curr if curr in items else '---')
            self.batt_combo.blockSignals(False)

    def _change_batt_topic(self, topic: str):
        if self.batt_sub:
            try: self.node.destroy_subscription(self.batt_sub)
            except: pass
            self.batt_sub = None
        if topic != '---':
            self.batt_sub = self.node.create_subscription(
                BatteryState, topic, self._on_battery, 10)

    def _on_battery(self, msg: BatteryState):
        self._append('voltage',    msg.voltage)
        self._append('current',    msg.current)
        self._append('percentage', msg.percentage * 100)

    def _ping_latency(self, host: str) -> float:
        if not host:
            return 0.0

        try:
            r = ping(host, timeout=1)
            if isinstance(r, (int, float)):
                return r * 1000
        except PingError:
            pass

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            t0 = time.time()
            sock.connect((host, 80))
            sock.close()
            return (time.time() - t0) * 1000
        except:
            return 0.0

    def _append(self, key: str, val: float):
        self.data[key].append(val)

    def _update(self):
        # Latencia
        ms = self._ping_latency(self.ping_host)
        self._append('latency', ms)
        for k, pl in self.plots.items():
            y = list(self.data[k])
            pl.setData(list(range(len(y))), y)

    def closeEvent(self, ev):
        if self.batt_sub:
            try: self.node.destroy_subscription(self.batt_sub)
            except: pass
        super().closeEvent(ev)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)
    node = rclpy.create_node('graph_viewer_node')
    win = GraphViewer(node)
    win.show()
    app.exec()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

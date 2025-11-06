#!/usr/bin/env python3
# motor_table_viewer.py

import sys
import os
import rclpy
from rclpy.node import Node
from astroviz_interfaces.msg import MotorStateList
from astroviz.utils.status_led import StatusLed
from PyQt6.QtWidgets import (
    QWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QHBoxLayout, QApplication, QHeaderView, QLabel, QComboBox
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QColor
from astroviz.utils.window_style import DarkStyle

# Threshold definitions
TEMP_NORMAL_MAX = 45.0
TEMP_ANOMALOUS_MAX = 65.0
VOLTAGE_NORMAL_MIN = 50.0
VOLTAGE_ANOMALOUS_MIN = 48.0

ALPHA = 100 
COLOR_NORMAL = QColor(144, 238, 144, ALPHA)
COLOR_ANOMALOUS = QColor(255, 255, 224, ALPHA)
COLOR_DANGER = QColor(255, 182, 193, ALPHA)

class MotorTableViewer(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Motor State Table')

        main_layout = QVBoxLayout(self)
        header_layout = QHBoxLayout()
        header_layout.addWidget(QLabel('Topic:'))
        self.topic_combo = QComboBox()
        self.topic_combo.addItem('---')
        self.topic_combo.currentTextChanged.connect(self._change_topic)
        self.topic_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.topic_combo.setMinimumContentsLength(20)
        header_layout.addWidget(self.topic_combo)
        header_layout.addStretch(1)
        main_layout.addLayout(header_layout)

        self.table = QTableWidget()
        cols = ['Status', 'Name', 'Temperature (°C)', 'Voltage (V)', 'Position (rad)']
        self.table.setColumnCount(len(cols))
        header = self.table.horizontalHeader()
        self.table.setHorizontalHeaderLabels(cols) 
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)

        self.table.setColumnWidth(0, 60)
        main_layout.addWidget(self.table)

        self.latest_msg = None
        self.subscriber = None

        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self._populate_topics)
        self.topic_timer.start(1000)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_table)
        self.refresh_timer.start(100)

    def _populate_topics(self):
        current = self.topic_combo.currentText()
        available = [name for name, types in self.node.get_topic_names_and_types()
                     if 'astroviz_interfaces/msg/MotorStateList' in types]
        items = ['---'] + available
        if items != [self.topic_combo.itemText(i) for i in range(self.topic_combo.count())]:
            self.topic_combo.blockSignals(True)
            self.topic_combo.clear()
            self.topic_combo.addItems(items)
            self.topic_combo.setCurrentText(current if current in items else '---')
            self.topic_combo.blockSignals(False)

    def _change_topic(self, topic: str):
        if self.subscriber:
            try: self.node.destroy_subscription(self.subscriber)
            except: pass
            self.subscriber = None
            self.latest_msg = None

        if topic != '---':
            self.subscriber = self.node.create_subscription(
                MotorStateList, topic, self._on_msg, 10
            )

    def _on_msg(self, msg: MotorStateList):
        self.latest_msg = msg

    def _refresh_table(self):
        if not self.latest_msg:
            self.table.setRowCount(0)
            return
        motors = list(self.latest_msg.motor_list)
        def severity(m):
            if m.temperature > TEMP_ANOMALOUS_MAX or m.voltage < VOLTAGE_ANOMALOUS_MIN:
                return 2
            if m.temperature > TEMP_NORMAL_MAX or m.voltage < VOLTAGE_NORMAL_MIN:
                return 1
            return 0
        motors.sort(key=severity, reverse=True)

        self.table.setRowCount(len(motors))
        for row, motor in enumerate(motors):
            led = StatusLed("")
            led.set_state(severity(motor))
            
            container = QWidget()
            layout = QHBoxLayout()
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(led)
            container.setLayout(layout)
            self.table.setCellWidget(row, 0, container)

            self._set_cell(row, 1, motor.name, bold=True)
            self._set_cell(row, 2, f"{motor.temperature:.2f}")
            self._set_cell(row, 3, f"{motor.voltage:.2f}")
            self._set_cell(row, 4, f"{motor.position:.2f}")


    def _set_cell(self, row: int, col: int, text: str, bold: bool=False):
        item = QTableWidgetItem(text)
        item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
        
        if col in [2, 3]:
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        
        if bold or col == 1:
            font = item.font()
            font.setBold(True)
            item.setFont(font)

        self.table.setItem(row, col, item)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)
    node = rclpy.create_node('motor_table_viewer_node')

    viewer = MotorTableViewer(node)
    viewer.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    spin_timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

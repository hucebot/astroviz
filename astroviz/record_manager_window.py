#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os, subprocess, datetime, shutil
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QHBoxLayout, QHeaderView, QAbstractItemView, QTextEdit,
    QPushButton, QLineEdit, QLabel, QSizePolicy, QMessageBox
)
from PyQt6.QtCore import QTimer, Qt
from astroviz.utils.status_led import StatusLed
from astroviz.utils.window_style import DarkStyle


class RecordManagerViewer(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Record Manager")

        self.selected_topics = set()
        self.is_recording = False
        self.all_topics = []
        self.record_process = None
        self.current_bag_path = None

        main_layout = QVBoxLayout(self)

        self.text_edit = QTextEdit()
        self.text_edit.setPlaceholderText("Write bag name here (optional)...")
        self.text_edit.setFixedHeight(30)
        self.text_edit.setStyleSheet("QTextEdit { color: gray; }")
        main_layout.addWidget(self.text_edit)

        button_layout = QHBoxLayout()

        self.btn_record = QPushButton("Record")
        self.btn_stop = QPushButton("Stop")
        self.btn_delete = QPushButton("Delete Last")
        self.size_label = QLabel("Size: --")
        self.size_label.setAlignment(Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter)

        for btn in [self.btn_record, self.btn_stop, self.btn_delete]:
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            button_layout.addWidget(btn, 1)

        button_layout.addWidget(self.size_label, 1)

        self.btn_stop.setEnabled(False)
        self.btn_record.clicked.connect(self.start_recording)
        self.btn_stop.clicked.connect(self.stop_recording)
        self.btn_delete.clicked.connect(self.delete_last_record)
        main_layout.addLayout(button_layout)

        search_layout = QHBoxLayout()
        search_label = QLabel("Search Topics:")
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Topic name filter...")
        self.search_input.setStyleSheet("QLineEdit { color: gray; }")
        self.search_input.textChanged.connect(self.filter_topics)
        search_layout.addWidget(search_label)
        search_layout.addWidget(self.search_input)
        main_layout.addLayout(search_layout)

        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Record", "Topic Name"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.table.setColumnWidth(0, 60)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        main_layout.addWidget(self.table)

        self.table.cellClicked.connect(self._on_cell_clicked)

        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self.update_topics)
        self.topic_timer.start(1000)

        self.size_timer = QTimer(self)
        self.size_timer.timeout.connect(self.update_size_label)
        self.size_timer.start(1000)

        self.default_record_style = self.btn_record.styleSheet()
        self.rosbag_dir = "/ros2_ws/src/astroviz/rosbags"
        os.makedirs(self.rosbag_dir, exist_ok=True)

    def start_recording(self):
        if self.is_recording:
            return
        if not self.selected_topics:
            QMessageBox.warning(self, "No Topics", "Select at least one topic to record.")
            return

        name = self.text_edit.toPlainText().strip()
        if not name:
            name = datetime.datetime.now().strftime("bag_%Y%m%d_%H%M%S")

        bag_path = os.path.join(self.rosbag_dir, name)
        if os.path.exists(bag_path):
            QMessageBox.warning(self, "Already Exists", f"The bag '{name}' already exists in rosbag/.")
            return

        os.makedirs(bag_path, exist_ok=False)
        self.current_bag_path = bag_path

        cmd = ["ros2", "bag", "record", "-o", bag_path] + list(self.selected_topics)
        print(f"Recording: {' '.join(cmd)}")

        self.record_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.is_recording = True

        self.btn_record.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                border: 1px solid #27ae60;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
        """)
        self.btn_record.setEnabled(False)
        self.btn_stop.setEnabled(True)

        print(f"Recording topics {', '.join(self.selected_topics)}")
        print(f"Path: {bag_path}")

    def stop_recording(self):
        if not self.is_recording or not self.record_process:
            return

        print("Stopping recording...")

        self.record_process.terminate()
        try:
            self.record_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.record_process.kill()

        self.record_process = None
        self.is_recording = False
        self.btn_record.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_record.setStyleSheet(self.default_record_style)
        self.size_label.setText("Size: --")
        print("Recording stopped.")

    def delete_last_record(self):
        bags = sorted(
            [f for f in os.listdir(self.rosbag_dir) if os.path.isdir(os.path.join(self.rosbag_dir, f))],
            key=lambda f: os.path.getmtime(os.path.join(self.rosbag_dir, f)),
        )
        if not bags:
            QMessageBox.information(self, "Empty", "No recordings to delete.")
            return
        last_bag = bags[-1]
        last_path = os.path.join(self.rosbag_dir, last_bag)
        try:
            shutil.rmtree(last_path)
            QMessageBox.information(self, "Deleted", f"Recording '{last_bag}' deleted successfully.")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Could not delete: {e}")

    def update_size_label(self):
        if not self.is_recording or not self.current_bag_path:
            return

        total_size = 0
        for root, _, files in os.walk(self.current_bag_path):
            for f in files:
                fp = os.path.join(root, f)
                if os.path.isfile(fp):
                    total_size += os.path.getsize(fp)

        if total_size < 1024**2:
            size_str = f"{total_size/1024:.1f} KB"
        elif total_size < 1024**3:
            size_str = f"{total_size/1024**2:.2f} MB"
        else:
            size_str = f"{total_size/1024**3:.2f} GB"

        self.size_label.setText(f"Size: {size_str}")

    def update_topics(self):
        current_topics = [name for name, _ in self.node.get_topic_names_and_types()]
        if set(self.all_topics) == set(current_topics):
            return
        self.all_topics = sorted(current_topics)
        self.populate_table()

    def populate_table(self):
        self.table.setRowCount(len(self.all_topics))
        for row, topic in enumerate(self.all_topics):
            led = StatusLed("")
            state = 0 if topic in self.selected_topics else 2
            led.set_state(state)

            container = QWidget()
            hbox = QHBoxLayout()
            hbox.setContentsMargins(0, 0, 0, 0)
            hbox.setAlignment(Qt.AlignmentFlag.AlignCenter)
            hbox.addWidget(led)
            container.setLayout(hbox)
            self.table.setCellWidget(row, 0, container)

            item = QTableWidgetItem(topic)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 1, item)

        self.filter_topics(self.search_input.text())

    def filter_topics(self, text):
        text = text.strip().lower()
        for row in range(self.table.rowCount()):
            topic = self.table.item(row, 1).text().lower()
            self.table.setRowHidden(row, text not in topic)

    def _on_cell_clicked(self, row, col):
        topic = self.table.item(row, 1).text()
        if topic in self.selected_topics:
            self.selected_topics.remove(topic)
        else:
            self.selected_topics.add(topic)
        self._refresh_led(row, topic)

    def _refresh_led(self, row, topic):
        led_widget = self.table.cellWidget(row, 0).layout().itemAt(0).widget()
        led_widget.set_state(0 if topic in self.selected_topics else 2)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)
    node = rclpy.create_node("record_manager_node")

    viewer = RecordManagerViewer(node)
    viewer.resize(700, 600)
    viewer.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(50)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

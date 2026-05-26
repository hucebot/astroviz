#!/usr/bin/env python3
import sys
import os
from typing import List

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QFrame,
    QSizePolicy,
)
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from astroviz.common._find import _find_pkg, _find_src_config

# --- Paths (keep same structure as your project) ---
_src_config = _find_src_config()
if _src_config:
    _CONFIG_DIR = _src_config
else:
    _CONFIG_DIR = os.path.join(
        get_package_share_directory("astroviz"), "config"
    )

_pkg = _find_pkg()
if _pkg:
    _PKG_DIR = _pkg
else:
    _PKG_DIR = get_package_share_directory("astroviz")

os.makedirs(_CONFIG_DIR, exist_ok=True)

ICONS_DIR = os.path.join(_PKG_DIR, "icons")


# -------------------------------- UI helpers ---------------------------------
class FlashBox(QFrame):
    """A rounded box with centered text that can flash (e.g., turn green briefly)."""

    def __init__(self, text: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._normal_bg = "#2b2b2b"
        self._flash_bg = "#2e7d32"  # green
        self._text = QLabel(text)
        self._text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._text.setStyleSheet("QLabel { color: white; font-weight: 600; }")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.addWidget(self._text)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self.setMinimumHeight(60)
        self._apply_bg(self._normal_bg)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet(
            self.styleSheet() + "\nQFrame { border-radius: 10px; }"
        )

    def _apply_bg(self, color: str):
        self.setStyleSheet(
            f"QFrame {{ background: {color}; border: 1px solid #444; border-radius: 10px; }}\n"
            f"QLabel {{ color: white; font-weight: 600; }}"
        )

    def flash(self, msec: int = 500):
        self._apply_bg(self._flash_bg)
        QTimer.singleShot(msec, lambda: self._apply_bg(self._normal_bg))


# ------------------------------- Main Window ---------------------------------
class MainWindow(QMainWindow):
    START_SRV = "/bag_recorder/start"
    STOP_SRV  = "/bag_recorder/stop"

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Menu Monitor")
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png")))

        # --- central layout (3 rows) ---
        central = QWidget()
        self.setCentralWidget(central)
        vroot = QVBoxLayout(central)
        vroot.setContentsMargins(10, 10, 10, 10)
        vroot.setSpacing(10)

        # NEW row 0: robot status
        row0=QHBoxLayout()
        self.status_label=QLabel("Status: Unknown")
        row0.addWidget(self.status_label)
        vroot.addLayout(row0)

        row01=QHBoxLayout()
        self.order_label=QLabel("Order: None")
        row01.addWidget(self.order_label)
        vroot.addLayout(row01)

        # Row 1: three table boxes + user done box
        row1 = QHBoxLayout()
        row1.setSpacing(10)
        self.box_t1 = FlashBox("Table 1")
        self.box_t2 = FlashBox("Table 2")
        self.box_t3 = FlashBox("Table 3")
        self.box_done = FlashBox("User done")
        row1.addWidget(self.box_t1)
        row1.addWidget(self.box_t2)
        row1.addWidget(self.box_t3)
        row1.addWidget(self.box_done)
        vroot.addLayout(row1)

        # Row 2: queue display
        row2 = QHBoxLayout()
        row2.setSpacing(10)
        self.queue_label = FlashBox("Table call queue: —")
        self.queue_label.setMinimumHeight(50)
        row2.addWidget(self.queue_label)
        vroot.addLayout(row2)

        # Row 3: left Reset button
        row3 = QHBoxLayout()
        row3.setSpacing(10)
        self.btn_reset = QPushButton("Order finished")
        self.btn_reset.setMinimumHeight(50)
        self.btn_reset.setCheckable(False)
        self.btn_reset.clicked.connect(self._on_reset_clicked)
        # Make button visually consistent
        self.btn_reset.setStyleSheet(
            """
            QPushButton { background: #444; color: white; border-radius: 8px; padding: 10px; }
            QPushButton:pressed { background: #666; }
            """
        )
        row3.addWidget(self.btn_reset, 1)


        vroot.addLayout(row3)

        # Row 4: record
        row4 = QHBoxLayout()
        row4.setSpacing(10)
        self.btn_record = QPushButton("Recording: OFF") # TODO: check...
        self.btn_record.setStyleSheet("background-color: gray; color: white;")
        self.btn_record.setMinimumHeight(50)
        self.btn_record.setCheckable(True)
        self.btn_record.setChecked(False)
        self.btn_record.clicked.connect(self._on_record_clicked)

        row4.addWidget(self.btn_record, 1)
        vroot.addLayout(row4)

        # --- ROS pubs/subs ---
        self._start_client = node.create_client(Trigger, self.START_SRV)
        self._stop_client  = node.create_client(Trigger, self.STOP_SRV)
        if not self._start_client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f"Service {self.START_SRV} not available after {timeout_sec}s")
        if not self._stop_client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f"Service {self.STOP_SRV} not available after {timeout_sec}s")

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.sub_t1 = self.node.create_subscription(
            Empty, "/table_1", self._cb_t1, qos
        )
        self.sub_t2 = self.node.create_subscription(
            Empty, "/table_2", self._cb_t2, qos
        )
        self.sub_t3 = self.node.create_subscription(
            Empty, "/table_3", self._cb_t3, qos
        )
        self.sub_done = self.node.create_subscription(
            Empty, "/menu_node/done", self._cb_done, qos
        )
        self.pub_reset = self.node.create_publisher(
            Empty, "/menu_node/reset", 1
        )
        self.sub_order = self.node.create_subscription(
            String, "/order", self._cb_order, qos
        )
        self.sub_status_cafeteria = self.node.create_subscription(
            String, "/status_cafeteria", self._cb_status_cafeteria, qos
        )
        # internal queue as a list of ints, unique membership
        self._queue: List[int] = []
        self._update_queue_label()

        # keep rclpy spinning
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(self.node, timeout_sec=0)
        )
        self.ros_timer.start(30)

        # status bar styling (optional)
        self.statusBar().setStyleSheet(
            "QStatusBar { background: #3a3a3a; color: lightgrey; border-top: 1px solid #444; }"
        )

    # ------------------------- UI + ROS behaviors -------------------------
    def _enqueue_table(self, table_id: int):
        if table_id not in self._queue:
            self._queue.append(table_id)
            self._update_queue_label()

    def _dequeue_head(self):
        if self._queue:
            head = self._queue.pop(0)
            self._update_queue_label()
            return head
        return None

    def _update_queue_label(self):
        if self._queue:
            text = "Table call queue: " + ", ".join(
                str(x) for x in self._queue
            )
        else:
            text = "Table call queue: —"
        self.queue_label._text.setText(text)

    def _cb_t1(self, _msg: Empty):
        self.box_t1.flash(500)
        self._enqueue_table(1)

    def _cb_t2(self, _msg: Empty):
        self.box_t2.flash(500)
        self._enqueue_table(2)

    def _cb_t3(self, _msg: Empty):
        self.box_t3.flash(500)
        self._enqueue_table(3)

    def _cb_done(self, _msg: Empty):
        self.box_done.flash(600)
        self.status_label.setText("Status: fetching order")
        # You can optionally dequeue here if that's your desired behavior later
        # For now, spec says just flash Done; queue management rules can be added later.

    def _cb_order(self, msg):
        # XXX TODO make it better looking :)
        self.order_label.setText(msg.data)

    def _cb_status_cafeteria(self,msg):
        self.status_label.setText("Status: serving")

    def _on_reset_clicked(self):
        self.status_label.setText("Status: awaiting order")
        self.order_label.setText("")
        self.pub_reset.publish(Empty())
        popped = self._dequeue_head()
        # brief visual feedback on the button
        self.btn_reset.setStyleSheet(
            """
            QPushButton { background: #2e7d32; color: white; border-radius: 8px; padding: 10px; }
            """
        )
        QTimer.singleShot(
            300,
            lambda: self.btn_reset.setStyleSheet(
                "QPushButton { background: #444; color: white; border-radius: 8px; padding: 10px; }\n"
                "QPushButton:pressed { background: #666; }"
            ),
        )
        msg = "Published /menu_node/reset"
        if popped is not None:
            msg += f" · dequeued {popped}"
        self.statusBar().showMessage(msg, 1500)

    def recording_service(self,client, msg):
        # Note: assuming recording service is here, but may have disappeared...
        # Check this ! TODO XXX
        future = client.call_async(Trigger.Request())

        # blocking, but expected. For now.
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

        if future.result() is None:
            msg = f"No response from '{msg}' service"
            print(msg)
            # XXX TODO

    def _on_record_clicked(self,checked):
        if checked:
            self.btn_record.setText("Recording: ON")
            self.btn_record.setStyleSheet("background-color: green; color: white;")
            self.recording_service(self._start_client,"start")
        else:
            self.btn_record.setText("Recording: OFF")
            self.btn_record.setStyleSheet("background-color: gray; color: white;")
            self.recording_service(self._stop_client,"stop")


# ---------------------------------- main -------------------------------------

def main(args=None):
    from astroviz.utils.window_style import DarkStyle

    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)
    node = rclpy.create_node("menu_monitor_node")
    window = MainWindow(node)
    window.resize(700, 400)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import sys
import os
import json
import time
import datetime

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
    QGridLayout,
    QStyle,QSpinBox,QStyleOption
)
from PyQt6.QtGui import QIcon, QPainter
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

class GridCell(QWidget):
    def __init__(self, key, text, icon: QIcon = None, parent=None):
        super().__init__(parent)
        self.key=key
        self.init_ui(text, icon)

    def init_ui(self, text, icon):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)
        top_layout = QHBoxLayout()

        self.icon_label = QLabel()
        if icon and not icon.isNull():
            self.icon_label.setPixmap(icon.pixmap(32, 32))
        else:
            # Fallback default icon if none is provided
            default_icon = self.style().standardIcon(QStyle.StandardPixmap.SP_ComputerIcon)
            self.icon_label.setPixmap(default_icon.pixmap(32, 32))
        top_layout.addWidget(self.icon_label)

        self.value_input = QSpinBox()
        self.value_input.setRange(0, 4)
        self.value_input.valueChanged.connect(self._on_user_changed)
        top_layout.addWidget(self.value_input)

        main_layout.addLayout(top_layout)

        self.fixed_label = QLabel(text)
        main_layout.addWidget(self.fixed_label)

        self.setStyleSheet("GridCell { border: 1px solid #333; background-color: transparent; }")

    def paintEvent(self, event):
        opt = QStyleOption()
        opt.initFrom(self)
        p = QPainter(self)
        self.style().drawPrimitive(QStyle.PrimitiveElement.PE_Widget, opt, p, self)

    def _on_user_changed(self, value):
        """Triggered only when the value is manually modified by the user."""
        self.setStyleSheet("GridCell { border: 1px solid #333; background-color: #401010; }")

    def set_value(self, value):
        if 0 <= value <= 4:
            self.value_input.blockSignals(True)
            self.value_input.setValue(value)
            self.value_input.blockSignals(False)
            self.setStyleSheet("GridCell { border: 1px solid #333; background-color: transparent; }")

    def get_order(self):
        return self.key, self.value_input.value()

class StateBox(QFrame):
    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        self._name = name
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._label = QLabel(f"{name}\n—")
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._label)
        self.setMinimumSize(100, 80)
        self._set_active(False)

    def _set_active(self, active: bool):
        color = "#2e7d32" if active else "#444"
        self.setStyleSheet(f"QFrame {{ background: {color}; border-radius: 8px; }}"
                           f"QLabel {{ color: white; font-weight: 500; }}")

    def update_state(self, status: str, count: int):
        self._label.setText(f"{self._name}\n#{count}")
        self._set_active(status == "active")

# ------------------------------- Main Window ---------------------------------
class MainWindow(QMainWindow):
    START_SRV = "/bag_recorder/start"
    STOP_SRV  = "/bag_recorder/stop"
    START_SRV_JETSON = "/jetson_bag_recorder/start"
    STOP_SRV_JETSON = "/jetson_bag_recorder/stop"

    def __init__(self, node, cfg):
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
        row_pipeline = QHBoxLayout()
        row_pipeline.setSpacing(8)
        self.box_streamdeck = StateBox("streamdeck")
        self.box_tablet     = StateBox("tablet")
        self.box_teleop     = StateBox("teleop")
        row_pipeline.addWidget(self.box_streamdeck)
        row_pipeline.addWidget(self.box_tablet)
        row_pipeline.addWidget(self.box_teleop)
        vroot.addLayout(row_pipeline)
        #
        # row0=QHBoxLayout()
        # self.status_label=QLabel("Status: Unknown")
        # row0.addWidget(self.status_label)
        # vroot.addLayout(row0)

        # row01=QHBoxLayout()
        # self.order_label=QLabel("Order: None")
        # row01.addWidget(self.order_label)
        # vroot.addLayout(row01)

        row02=QHBoxLayout()
        self.cells = {}
        grid_layout = QGridLayout()
        initial_data = cfg["products"]
        # ["croissant","espresso","long_coffee","pain_au_chocolat","pain_aux_raisins","tea"]

        for i, text in enumerate(initial_data):
            row = i // 3
            col = i % 3
            icon=QIcon(os.path.join(cfg["config_path"], f"{text}.png"))
            cell = GridCell(text,text,icon)
            grid_layout.addWidget(cell, row, col)
            self.cells[text]=cell
        grid_layout.setColumnStretch(0, 1)
        grid_layout.setColumnStretch(1, 1)
        grid_layout.setColumnStretch(2, 1)

        row02.addLayout(grid_layout)

        self.btn_resend = QPushButton("Re-send order")
        self.btn_resend.setMinimumHeight(50)
        self.btn_resend.setCheckable(False)
        # self.btn_resend.clicked.connect(self._on_resend_clicked) # XXX TODO
        row02.addWidget(self.btn_resend)
        vroot.addLayout(row02)
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
        self.btn_reset.setEnabled(False) # only meaningful when "active"
        self.btn_reset.clicked.connect(self._on_reset_clicked)
        # Make button visually consistent
        self.btn_reset.setStyleSheet(
            """
            QPushButton { background: #444; color: white; border-radius: 8px; padding: 10px; }
            QPushButton:pressed { background: #666; }
            QPushButton:disabled { background: #2a2a2a; color: #666; }
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
        self._start_jetson_client = node.create_client(Trigger, self.START_SRV_JETSON)
        self._stop_jetson_client = node.create_client(Trigger, self.STOP_SRV_JETSON)

        # if not self._start_client.wait_for_service(timeout_sec=5):
        #     raise RuntimeError(f"Service {self.START_SRV} not available after 5s")
        # if not self._stop_client.wait_for_service(timeout_sec=5):
        #     raise RuntimeError(f"Service {self.STOP_SRV} not available after 5s")
        # if not self._start_jetson_client.wait_for_service(timeout_sec=5):
        #     raise RuntimeError(f"Service {self.START_JETSON_SRV} not available after 5s")
        # if not self._stop_jetson_client.wait_for_service(timeout_sec=5):
        #     raise RuntimeError(f"Service {self.STOP_JETSON_SRV} not available after 5s")

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
        self.sub_remote = self.node.create_subscription(String,"/remote_button",
                                                        self.cb_remote,10)
        self.pub_audio = self.node.create_publisher(String, "/teleop/audio_play",10)
        self._status="done"
        self._count=0
        self._order={}
        self.history={}
        self.current_order_count=0 # real one starts at 1
        now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_file_name=f"/data/orders_{now}.log"

        self.current_task={}
        self.current_task["task"]="order"
        self.current_task["count"]=1
        self.current_task["retry"]=0

        self._state_pub = self.node.create_publisher(
            String,
            "teleop_state",
            1,
            )

        # Main state (== current task)
        self._control_state_pub = self.node.create_publisher(
            String,
            "control_state",
            1,
            )

        self._state_sub = self.node.create_subscription(
            String,
            "tablet_state",
            self._tablet_state_cb,
            1,
        )
        self._streamdeck_state_sub = self.node.create_subscription(
            String,
            "streamdeck_state",
            self._streamdeck_state_cb,
            1,
        )
        # self, so not needed but...
        self._teleop_state_sub = self.node.create_subscription(
            String,
            "teleop_state",
            self._teleop_state_cb,
            1,
        )

        self.heartbeat_timer = self.node.create_timer(1.0, self._send_state)

        # self.sub_done = self.node.create_subscription(
        #     Empty, "/menu_node/done", self._cb_done, qos
        # )
        # self.pub_reset = self.node.create_publisher(
        #     Empty, "/menu_node/reset", 1
        # )
        # self.sub_order = self.node.create_subscription(
        #     String, "/order", self._cb_order, qos
        # )
        # self.pub_order = self.node.create_publisher(
        #     String, "/order", 1
        # )
        # self.sub_status_cafeteria = self.node.create_subscription(
        #     String, "/status_cafeteria", self._cb_status_cafeteria, qos
        # )
        # internal queue as a list of ints, unique membership
        self._queue: List[int] = []
        self._update_queue_label()

        # keep rclpy spinning
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.ros_cb)
        self.ros_timer.start(30)

        # status bar styling (optional)
        self.statusBar().setStyleSheet(
            "QStatusBar { background: #3a3a3a; color: lightgrey; border-top: 1px solid #444; }"
        )

    def ros_cb(self):
        for i in range(2):
            rclpy.spin_once(self.node, timeout_sec=0)

    def _send_state(self):
        s=String()
        state={}
        state["status"]=self._status
        state["order"]=self._order
        state["count"]=self._count
        s.data=json.dumps(state)
        self._state_pub.publish(s)
        self._send_control_state()

    def _send_control_state(self):
        s=String()
        s.data=json.dumps(self.current_task)
        self._control_state_pub.publish(s)

    def _tablet_state_cb(self,msg):
        state=json.loads(msg.data)
        self.box_tablet.update_state(state["status"], state["count"])
        # detect end of task
        if (state["status"]=="done") and (self.current_task["task"]=="fetch") and (state["count"]==self.current_task["count"]):
            self.current_task["task"]="serve"
            # shortcut, no need to wait for control, we're in the same process..;
            self._count=state["count"]
            self._status="active"
            self._order = state["order"]
            self.btn_reset.setEnabled(True)

        # if state["status"]=="done":
        #     if state["count"]>self._count:

    def _streamdeck_state_cb(self,msg):
        state=json.loads(msg.data)
        if state["count"]>self.current_order_count:
            self.current_order_count=state["count"] # detect order in progress
            self.current_order_start=time.time()
        self.box_streamdeck.update_state(state["status"], state["count"])
        for k in state["order"]:
             self.cells[k].set_value(state["order"][k])
        # detect end of task
        if (state["status"]=="done") and (self.current_task["task"]=="order") and (state["count"]==self.current_task["count"]):
            self.current_task["task"]="fetch"
            self.current_task["order"]=state["order"]


    def _teleop_state_cb(self,msg):
        state=json.loads(msg.data)
        self.box_teleop.update_state(state["status"], state["count"])

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

    def cb_remote(self,msg):
        audio=String()
        audio.data="Hello"
        self.pub_audio.publish(audio)
        print(msg.data)

    # def _cb_done(self, _msg: Empty):
    #     self.box_done.flash(600)
    #     self.status_label.setText("Status: fetching order")
    #     # You can optionally dequeue here if that's your desired behavior later
    #     # For now, spec says just flash Done; queue management rules can be added later.

    # def _cb_order(self, msg):
    #     # XXX TODO make it better looking :)
    #     self.order_label.setText(msg.data)
    #     order=json.loads(msg.data)
    #     for k in order:
    #         self.cells[k].set_value(order[k])


    # def _cb_status_cafeteria(self,msg):
    #     self.status_label.setText("Status: serving")
    def flatten_history(self,h):
        l=[h["count"], h["time_start"], h["time_end"], h["table"]]
        keys=sorted(h["order"].keys())
        l+=[ h["order"][k] for k in keys]
        return ",".join(map(str,l)) + "\n"

    def _on_reset_clicked(self):
        if self._status=="active":
            self.btn_reset.setEnabled(False)
            # order is now complete.
            self.history[self.current_order_count]= {
                "count": self.current_order_count,
                "order": self._order,
                "time_start": self.current_order_start,
                "time_end": time.time(),
            }
            self._status="done"
            self.current_task["task"]="order"
            self.current_task["count"]+=1 # count++ here: no race condition ( if order and old count, seen as old/dispoable packet)

            # self.status_label.setText("Status: awaiting order")
            # self.order_label.setText("")
            for c in self.cells:
                self.cells[c].set_value(0)
            # self.pub_reset.publish(Empty())
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
            msg = "Done "
            if popped is not None:
                msg += f" · dequeued {popped}"
                self.history[self.current_order_count]["table"]=popped
            else:
                self.history[self.current_order_count]["table"]=0
            self.statusBar().showMessage(msg, 1500)

            with open(self.log_file_name, "a", encoding="utf-8") as f:
                line=self.flatten_history(self.history[self.current_order_count])
                print(line)
                # TODO GUI too
                f.write(line)

    # def _on_resend_clicked(self):
    #     msg=String()
    #     order={}
    #     for c in self.cells:
    #         k,v=self.cells[c].get_order()
    #         order[k]=v
    #     msg.data=json.dumps(order)
    #     self.pub_order.publish(msg)
    #     status_msg=f"re-sent: {msg.data}"
    #     self.statusBar().showMessage(status_msg, 1500)


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
            self.recording_service(self._start_jetson_client,"start jetson")
        else:
            self.btn_record.setText("Recording: OFF")
            self.btn_record.setStyleSheet("background-color: gray; color: white;")
            self.recording_service(self._stop_client,"stop")
            self.recording_service(self._stop_jetson_client,"stop jetson")

# ---------------------------------- main -------------------------------------
def read_json_config_file(node,path):
    try:
        with open(path, "r") as f:
            cfg = json.load(f)
    except Exception as e:
        node.get_logger().error(f"{e}")
        exit(1)
    return cfg

def main(args=None):
    from astroviz.utils.window_style import DarkStyle

    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)
    node = rclpy.create_node("menu_monitor_node")

    config_path = os.environ.get("CONFIG_PATH", ".")
    cfg=read_json_config_file(node,os.path.join(config_path,"config.json")) # main expe config file
    node.get_logger().info(f"config found in {config_path}")
    cfg["config_path"]=config_path
    window = MainWindow(node,cfg)
    window.resize(700, 400)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

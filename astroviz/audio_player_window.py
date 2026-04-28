#!/usr/bin/env python3
import sys
import os
from typing import List, Tuple
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QComboBox,
    QSizePolicy,
    QSpacerItem,
    QGridLayout,
)
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import QTimer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles,QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Joy

import numpy as np
import wave
from audio_common_msgs.msg import AudioStamped, Audio, AudioData, AudioInfo
from std_msgs.msg import Header
from rclpy.qos import QoSPresetProfiles

from ament_index_python.packages import get_package_share_directory
from astroviz.common._find import _find_pkg, _find_src_config

# TODO move this to config file maybe...
# mapping key from gamepad -> sound name
sounds = {
    0: "Whistle_1",
    1: "Whistle_2",
    2: "Hello.wav",
    3: "Goodbye.wav",
    4: "ErrorWindows.wav"
}

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

CONFIG_PATH = os.path.join(_CONFIG_DIR, "dashboard_config.json")
ICONS_DIR = os.path.join(_PKG_DIR, "icons")
AUDIO_DIR = os.path.join(_PKG_DIR, "audio")

PA_INT16 = 8  # PortAudio code for S16LE
PA_INT8 = 16  # PortAudio code for S8
PA_UINT8 = 32  # PortAudio code for U8


def _bytes_to_int16_interleaved(raw: bytes, sampwidth: int) -> np.ndarray:
    """
    Convert raw little-endian PCM bytes (from wave) to int16 interleaved samples.
    Handles 8/16/24/32-bit inputs; 24/32 are downconverted to int16.
    """
    if sampwidth == 2:
        return np.frombuffer(raw, dtype="<i2")  # int16 LE

    arr = np.frombuffer(raw, dtype=np.uint8)

    if sampwidth == 1:
        # WAV 8-bit PCM is unsigned: 0..255. Convert to signed int16 centered at 0.
        s = arr.astype(np.int16)
        s = ((s - 128) << 8).astype(np.int16)  # map to ~[-32768, 32767]
        return s

    if sampwidth == 3:
        # 24-bit little-endian signed → int32 then downshift to int16
        if arr.size % 3 != 0:
            arr = arr[: (arr.size // 3) * 3]
        a = arr.reshape(-1, 3).astype(np.uint32)
        v = (a[:, 0] | (a[:, 1] << 8) | (a[:, 2] << 16)).astype(np.int32)
        # sign-extend from 24 to 32 bits
        neg = (a[:, 2] & 0x80) != 0
        v[neg] |= ~0xFFFFFF
        return (v >> 8).astype(np.int16)  # simple downscale

    if sampwidth == 4:
        # Assume 32-bit signed PCM; downshift to int16
        v = np.frombuffer(raw, dtype="<i4")  # int32 LE
        return (v >> 16).astype(np.int16)

    raise ValueError(f"Unsupported WAV sample width: {sampwidth} bytes")


def load_wav_as_audio_stamped(path: str) -> AudioStamped:
    """Load one WAV and return a full-buffer AudioStamped (S16LE)."""
    with wave.open(path, "rb") as wf:
        channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()  # bytes per sample
        rate = wf.getframerate()
        nframes = wf.getnframes()

        raw = wf.readframes(nframes)  # all frames at once
        pcm_i16 = _bytes_to_int16_interleaved(raw, sampwidth)  # interleaved
        frames_total = pcm_i16.size // channels

    ad = AudioData()
    ad.int16_data = pcm_i16.tolist()  # length = frames_total * channels

    info = AudioInfo()
    info.format = PA_INT16  # publishing as int16
    info.channels = int(channels)
    info.rate = int(rate)
    info.chunk = int(frames_total)  # FULL file (frames per channel)

    a = Audio()
    a.audio_data = ad
    a.info = info

    msg = AudioStamped()
    msg.header = Header()
    msg.header.frame_id = os.path.basename(
        path
    )  # stamp can be set at publish time
    msg.audio = a
    return msg


def build_audio_message_list(dir: str) -> Tuple[List[AudioStamped], List[str]]:
    """Return (messages, labels) for all WAVs in dir (sorted).

    - messages: list[AudioStamped] with full file in int16_data and info.chunk = total frames
    - labels:   list[str] filename without extension, same order as messages
    """
    # Collect and sort .wav files deterministically (case-insensitive)
    wav_paths = sorted(
        (
            os.path.join(dir, f)
            for f in os.listdir(dir)
            if f.lower().endswith(".wav")
        ),
        key=lambda p: os.path.basename(p).lower(),
    )

    messages: List[AudioStamped] = []
    labels: List[str] = []

    for p in wav_paths:
        try:
            msg = load_wav_as_audio_stamped(
                p
            )  # uses the helper defined earlier
            label = os.path.splitext(os.path.basename(p))[0]
            messages.append(msg)
            labels.append(label)
        except Exception as e:
            print(f"[WARN] Skipping {p}: {e}")

    return messages, labels


class MainWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Audio")
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png")))

        self.setGeometry(100, 100, 300, 150)

        central = QWidget()
        self.setCentralWidget(central)
        layout=QGridLayout(central)
        # Topic selector
        self.combo = QComboBox(self.centralWidget())
        self.combo.setFixedWidth(150)
        self.combo.raise_()
        self.combo.currentTextChanged.connect(self.change_topic)
        ncol=4
        layout.addWidget(self.combo,0,0, 1,ncol,Qt.AlignmentFlag.AlignRight)

        # Customize status bar
        sb = self.statusBar()
        sb.setSizeGripEnabled(True)
        sb.setStyleSheet(
            """
            QStatusBar {
                background: #3a3a3a;
                color: lightgrey;
                font-style: italic;
                border-top: 1px solid #444;
            }
        """
        )

        # Load WAV files and create AudioStamped messages
        self.audio_msgs, self.audio_labels = build_audio_message_list(
            AUDIO_DIR
        )

        print(f"Loaded {len(self.audio_labels)} audio messages:")
        for i, label in enumerate(self.audio_labels, 1):
            print(f"[{i:02d}] {label}")

        i=ncol # start at second line, first is combo
        # Buttons for sending audio
        for msg, label in zip(self.audio_msgs, self.audio_labels):
            btn = QPushButton(label)
            btn.clicked.connect(
                lambda _, msg=msg, label=label: self.send_audio(msg, label)
            )
            # layout.addWidget(btn)
            layout.addWidget(btn, i // ncol, i%ncol)
            i+=1
        layout.setRowStretch(i//ncol+1, 1)

        # using gamepad buttons to send audio
        self.gamepad_sub=self.node.create_subscription(
            Joy,
            "/teleop_joy",
            self._gamepad_cb,
            10,
        )
        # publisher for sound bank based sounds
        self.sound_pub = self.node.create_publisher(
            String,
            "audio_play",
            1,
            )
        self.gamepad_buttons = []
        self.audio_pub = None
        self._populate_topics()

        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self._populate_topics)
        self.topic_timer.start(1000)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._main_timer)
        self.ros_timer.start(50)

    def _main_timer(self):
        for i in range(10):
            rclpy.spin_once(self.node, timeout_sec=0)

    def _play_sound(self,name):
        """ send a sound name to the sound_pub topic."""
        s=String()
        s.data=name
        self.sound_pub.publish(s)

    def _gamepad_cb(self, msg: Joy):
        """ callback for joystick/gamepad buttons """
        if not self.gamepad_buttons: #initial state
            self.gamepad_buttons = list(msg.buttons)
            return

        for i in range(len(msg.buttons)):
            if (msg.buttons[i] == 1) and (self.gamepad_buttons[i] == 0):
                if i in sounds.keys():
                    self._play_sound(sounds[i])
                # if i<len(self.audio_msgs):
                #     self.send_audio(self.audio_msgs[i],self.audio_labels[i])

        self.gamepad_buttons = list(msg.buttons)

    def send_audio(self, msg: AudioStamped, label: str):
        if not self.audio_pub:
            self.statusBar().showMessage("No topic selected", 3000)
            return
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.audio_pub.publish(msg)
        self.statusBar().showMessage(f"Sent {label}", 2000)


    def _populate_topics(self):
        current = self.combo.currentText()
        all_topics = self.node.get_topic_names_and_types()
        audio_topics = [
            name
            for name, types in all_topics
            if "audio_common_msgs/msg/AudioStamped" in types
        ]
        items = ["---"] + audio_topics

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
            self.change_topic("---")
        self.combo.blockSignals(False)

    def change_topic(self, topic_name: str):
        if self.audio_pub is not None:
            try:
                self.node.destroy_publisher(self.audio_pub)
            except Exception:
                pass
            self.audio_pub = None

        if topic_name == "---":
            return

        self.audio_pub = self.node.create_publisher(
            AudioStamped, topic_name, QoSPresetProfiles.SENSOR_DATA.value
        )


def main(args=None):
    from astroviz.utils.window_style import DarkStyle

    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)
    node = rclpy.create_node("audio_player_window")
    window = MainWindow(node)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

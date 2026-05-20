#!/usr/bin/env python3
import sys
import os
from typing import Optional, Tuple, List

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QSplitter,
    QSizePolicy,
    QFrame,
)
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import Qt, QSize

import rclpy

# Import your existing windows
from astroviz.audio_player_window import MainWindow as AudioWindow
from astroviz.mobile_base_window import MainWindow as MobileBaseWindow
from astroviz.camera_window import CameraViewer
from astroviz.gstreamer_shelfy_window import (
    GstreamerWindow as ShelfyGstreamerWindow,
)
from astroviz.gstreamer_window import GstreamerWindow
from astroviz.cafeteria_menu_window import MainWindow as CoffeeMenuWindow

from ament_index_python.packages import get_package_share_directory
from astroviz.common._find import _find_pkg, _find_src_config

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


class SplitterGrid(QWidget):
    """
    2x3 grid using nested QSplitters with the following behavior:
    - **Vertical splitters are GLOBAL** across all rows (dragging a column divider
      resizes that column everywhere). Implemented by an outer *horizontal* splitter
      that owns the 3 columns.
    - **Horizontal splitters are INDEPENDENT** per column (dragging the row divider
      affects only that column). Implemented by an inner *vertical* splitter inside
      each column pane.

    All panes are fully collapsible.
    """

    def __init__(self, rows: int, cols: int, parent: Optional[QWidget] = None):
        super().__init__(parent)
        assert rows >= 1 and cols >= 1
        self.rows = rows
        self.cols = cols

        # OUTER: columns (global vertical splitters)
        self._hsplitter = QSplitter(Qt.Orientation.Horizontal, self)
        self._hsplitter.setChildrenCollapsible(True)

        # Each column contains a vertical splitter for the rows (independent)
        self._col_vsplitters: List[QSplitter] = []

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(self._hsplitter)

        for c in range(cols):
            vsplit = QSplitter(Qt.Orientation.Vertical)
            vsplit.setChildrenCollapsible(True)
            self._col_vsplitters.append(vsplit)
            self._hsplitter.addWidget(vsplit)

            # Fill with placeholders for each row
            for r in range(rows):
                ph = self._make_placeholder()
                vsplit.addWidget(ph)
                try:
                    vsplit.setCollapsible(r, True)
                except Exception:
                    pass

            # Allow columns themselves to collapse fully
            try:
                self._hsplitter.setCollapsible(c, True)
            except Exception:
                pass

    def _make_placeholder(self) -> QWidget:
        ph = QFrame()
        ph.setFrameShape(QFrame.Shape.StyledPanel)
        ph.setStyleSheet(
            "QFrame { background: #222; border: 1px dashed #444; }"
        )
        ph.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        ph.setMinimumSize(QSize(0, 0))
        return ph

    def set_widget(self, row: int, col: int, w: QWidget):
        assert 0 <= row < self.rows and 0 <= col < self.cols
        vsplit = self._col_vsplitters[col]
        w.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        w.setMinimumSize(QSize(0, 0))
        # Replace the existing widget at index `row` in this column's vsplitter
        old = vsplit.widget(row)
        if old is not None:
            old.setParent(None)
        vsplit.insertWidget(row, w)
        if vsplit.count() > self.rows:
            extra = vsplit.widget(row + 1)
            if extra is not None:
                extra.setParent(None)
        try:
            vsplit.setCollapsible(row, True)
        except Exception:
            pass

    # --- Sizing helpers ---
    def set_col_sizes(self, sizes: List[int]):
        """Set global column widths (applies to the outer horizontal splitter)."""
        if not sizes:
            return
        total = sum(sizes)
        if total <= 0:
            return
        scaled = [max(0, int(1000 * s / total)) for s in sizes]
        self._hsplitter.setSizes(scaled)

    def set_row_sizes_for_column(self, col: int, sizes: List[int]):
        """Set row heights for a single column (independent)."""
        assert 0 <= col < self.cols
        if not sizes:
            return
        total = sum(sizes)
        if total <= 0:
            return
        scaled = [max(0, int(1000 * s / total)) for s in sizes]
        self._col_vsplitters[col].setSizes(scaled)

    def set_all_row_sizes(self, sizes: List[int]):
        """Optionally apply the same row sizes to every column (still independent)."""
        for c in range(self.cols):
            self.set_row_sizes_for_column(c, sizes)


class MultiWindowHost(QMainWindow):
    """Host window with a 2x3 nested-splitter grid.
    Vertical splitters (between columns) are global; horizontal splitters (between rows)
    are independent per column.
    """

    def __init__(self, grid_shape: Tuple[int, int] = (2, 3)):
        super().__init__()
        self.setWindowTitle("Shelfy Teleoperation Dashboard")
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png")))
        self._owned_widgets: List[QWidget] = []

        self._rows, self._cols = grid_shape

        self._central = QWidget(self)
        self.setCentralWidget(self._central)
        self._root_layout = QVBoxLayout(self._central)
        self._root_layout.setContentsMargins(8, 8, 8, 8)
        self._root_layout.setSpacing(8)

        self._grid = SplitterGrid(self._rows, self._cols, parent=self._central)
        self._root_layout.addWidget(self._grid)

        self.statusBar().setStyleSheet(
            "QStatusBar { background: #3a3a3a; color: #FFFFFF; border-top: 1px solid #444; }"
        )

    def _normalize_child(self, w: QWidget) -> QWidget:
        if isinstance(w, QMainWindow):
            w.setWindowFlags(Qt.WindowType.Widget)
        w.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        w.setMinimumSize(QSize(0, 0))
        return w

    def add_widget(self, w: QWidget, row: int, col: int):
        w = self._normalize_child(w)
        self._owned_widgets.append(w)
        self._grid.set_widget(row, col, w)

    # passthrough helpers
    def set_col_sizes(self, sizes: List[int]):
        self._grid.set_col_sizes(sizes)

    def set_row_sizes_for_column(self, col: int, sizes: List[int]):
        self._grid.set_row_sizes_for_column(col, sizes)

    def set_all_row_sizes(self, sizes: List[int]):
        self._grid.set_all_row_sizes(sizes)

    def closeEvent(self, event):
        for w in self._owned_widgets:
            try:
                w.close()
            except Exception:
                pass
        super().closeEvent(event)


def main():
    from astroviz.utils.window_style import DarkStyle

    rclpy.init()
    app = QApplication(sys.argv)
    DarkStyle(app)

    # Shared node
    node = rclpy.create_node("astroviz_dashboard")

    host = MultiWindowHost(grid_shape=(2, 3))
    host.showMaximized()

    # Build widgets
    gst_screen = GstreamerWindow(port=5004)
    camera_viewer = CameraViewer(node,"imageXXX")
    cmd_vel = MobileBaseWindow(node, "teleop_vel","scan")
    gst_webcam = ShelfyGstreamerWindow(port=5000, width=960, height=540)
    audio = AudioWindow(node,default_topic="/tts/audio")
    cafe_menu = CoffeeMenuWindow(node)

    # Layout:
    # top_row: webcam, camera, cmd_vel
    host.add_widget(gst_webcam, row=0, col=0)
    host.add_widget(camera_viewer, row=0, col=1)
    host.add_widget(cmd_vel, row=0, col=2)
    # bottom_row: coffe menu, screen, audio
    host.add_widget(cafe_menu, row=1, col=0)
    host.add_widget(gst_screen, row=1, col=1)
    host.add_widget(audio, row=1, col=2)

    # # Example: add the TTSWindow
    # from astroviz.tts_window import MainWindow as TTSWindow
    # predef_msgs = [
    #     "Hello, I'm Shelfy!",
    #     "Please, fill your order by pressing the buttons.",
    #     "Please, load the order you see on the screen.",
    # ]
    # host.add_widget(TTSWindow(node, predef_msgs), row=1, col=2)

    # Optional initial sizes:
    host.set_col_sizes([1, 1, 1])  # global columns
    host.set_row_sizes_for_column(0, [1, 1])  # webcam column
    host.set_row_sizes_for_column(1, [1, 1])  # camera/screen column
    host.set_row_sizes_for_column(2, [1, 1])  # cmd_vel/audio column

    host.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

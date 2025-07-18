import sys
from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QGridLayout
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCharts import QChartView, QChart, QLineSeries, QValueAxis
from threading import Thread
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy

class GStreamerPipeline:
    def __init__(self, pipeline_str, callback):
        Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        self.appsink.connect("new-sample", self.on_new_sample)
        self.callback = callback

    def start(self):
        self.loop = GLib.MainLoop()
        self.pipeline.set_state(Gst.State.PLAYING)
        self.thread = Thread(target=self.loop.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value("width")
        height = caps.get_structure(0).get_value("height")
        stride = 4 * width  # BGRx format
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            data = map_info.data
            Thread(target=self.callback, args=(width, height, stride, data), daemon=True).start()
            buf.unmap(map_info)
        return Gst.FlowReturn.OK

class WebcamDisplay(QObject):
    image_updated = pyqtSignal(QImage)
    def __init__(self, label):
        super().__init__()
        self.label = label
        self.image_updated.connect(self._update)

    def update_image(self, w, h, stride, data):
        img = QImage(data, w, h, stride, QImage.Format.Format_RGB32)
        self.image_updated.emit(img)

    def _update(self, img):
        self.label.setPixmap(QPixmap.fromImage(img))

class GstreamerWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Main Camera with QtCharts")
        main_layout = QGridLayout(self)

        # Video display
        self.camera_label = QLabel(self)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(self.camera_label, 0, 0)

        # QtChart for FPS
        self.series = QLineSeries()
        self.chart = QChart()
        self.chart.addSeries(self.series)
        self.chart.createDefaultAxes()
        self.chart.setTitle("FPS over time")
        # Configure axes
        self.axis_x = QValueAxis()
        self.axis_x.setLabelFormat("%i")
        self.axis_x.setTitleText("Frame")
        self.axis_y = QValueAxis()
        self.axis_y.setLabelFormat("%.1f")
        self.axis_y.setTitleText("FPS")
        self.chart.setAxisX(self.axis_x, self.series)
        self.chart.setAxisY(self.axis_y, self.series)

        self.chart_view = QChartView(self.chart, self)
        main_layout.addWidget(self.chart_view, 0, 1)

        # GStreamer pipeline
        pipeline_str = (
            "udpsrc port=5000 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! "
            "videoconvert ! videoscale ! video/x-raw,format=BGRx,width=1510,height=1150 ! appsink name=appsink emit-signals=true async=false"
        )
        self.display = WebcamDisplay(self.camera_label)
        self.pipeline = GStreamerPipeline(pipeline_str, self.display.update_image)
        self.pipeline.start()
        self.frame_count = 0
        self.start_time = None

    def closeEvent(self, event):
        self.pipeline.stop()
        super().closeEvent(event)

    def update_fps(self, fps_value):
        # Call this method each frame with calculated FPS
        self.frame_count += 1
        self.series.append(self.frame_count, fps_value)
        # Adjust axes range
        self.axis_x.setRange(0, self.frame_count)
        self.axis_y.setRange(0, max(30, fps_value))


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    node = rclpy.create_node('gstreamer_viewer_qtcharts')
    window = GstreamerWindow()
    window.show()
    app.exec()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
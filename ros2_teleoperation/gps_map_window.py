#!/usr/bin/env python3

import sys
import os
import json
import threading
import http.server
import socketserver

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, String

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox
)
from PyQt6.QtGui import QFont, QPixmap
from PyQt6.QtCore import Qt, QTimer, QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView

from ros2_teleoperation.utils.window_style import DarkStyle

# Qt rendering flags for Docker
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu --disable-software-rasterizer"
os.environ["QT_QUICK_BACKEND"] = "software"
os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"

# Font for header labels
header_font = QFont()
header_font.setPointSize(12)

class QuietHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP handler that silences logs."""
    def __init__(self, *args, directory=None, **kwargs):
        super().__init__(*args, directory=directory, **kwargs)
    def log_message(self, format, *args):
        pass

class GPSMapWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Map Viewer")

        w, h = self.get_screen_size()
        self.resize(w, h)

        self.wp_pub = node.create_publisher(String, '/gps/waypoints', 10)

        base = "/ros2_ws/src/ros2_teleoperation"
        self.map_dir       = os.path.join(base, "maps")
        os.makedirs(self.map_dir, exist_ok=True)
        self.json_path     = os.path.join(self.map_dir, "gps_data.json")
        self.map_html_path = os.path.join(self.map_dir, "live_map.html")
        self.icons_dir     = os.path.join(base, "icons")

        self.create_map_html()
        threading.Thread(
            target=lambda: self.start_http_server(root=base, directory=base),
            daemon=True
        ).start()

        header = QWidget()
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(5, 5, 5, 5)
        h_layout.setSpacing(20)

        sat_widget = QWidget()
        sat_lay = QHBoxLayout(sat_widget)
        sat_lay.setContentsMargins(0,0,0,0)
        sat_lay.setSpacing(5)
        sat_icon = QLabel()
        pix = QPixmap(os.path.join(self.icons_dir, "satellite.png"))
        sat_icon.setPixmap(
            pix.scaled(32, 32, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        )
        self.sat_label = QLabel("--")
        self.sat_label.setFont(header_font)
        sat_lay.addWidget(sat_icon)
        sat_lay.addWidget(self.sat_label)
        h_layout.addWidget(sat_widget, alignment=Qt.AlignmentFlag.AlignLeft)

        h_layout.addStretch()
        self.pos_label = QLabel("<b>Lat:</b> --, <b>Lon:</b> --")
        self.pos_label.setFont(header_font)
        h_layout.addWidget(self.pos_label, alignment=Qt.AlignmentFlag.AlignCenter)
        h_layout.addStretch()

        btn_container = QWidget()
        btn_vlay = QVBoxLayout(btn_container)
        btn_vlay.setContentsMargins(0,0,0,0)
        btn_vlay.setSpacing(5)

        self.add_wp_button = QPushButton("Add Waypoints")
        self.add_wp_button.setCheckable(True)
        self.add_wp_button.clicked.connect(self.toggle_waypoint_mode)
        self.wp_button = QPushButton("Publish WP")
        self.wp_button.clicked.connect(self.publish_waypoints)
        row1 = QHBoxLayout()
        row1.setSpacing(5)
        row1.addWidget(self.add_wp_button)
        row1.addWidget(self.wp_button)
        btn_vlay.addLayout(row1)

        self.center_button = QPushButton("Center")
        self.center_button.clicked.connect(self.center_map)
        self.is_hybrid = False
        self.hybrid_button = QPushButton("Hybrid")
        self.hybrid_button.clicked.connect(self.toggle_hybrid)
        self.toggle_path_button = QPushButton("Show/Hide Path")
        self.toggle_path_button.clicked.connect(self.toggle_path)
        row2 = QHBoxLayout()
        row2.setSpacing(5)
        row2.addWidget(self.center_button)
        row2.addWidget(self.hybrid_button)
        row2.addWidget(self.toggle_path_button)
        btn_vlay.addLayout(row2)

        h_layout.addWidget(btn_container, alignment=Qt.AlignmentFlag.AlignRight)

        self.view = QWebEngineView()
        self.view.setUrl(QUrl("http://localhost:8080/maps/live_map.html"))

        self.nav_combo = QComboBox(self.view)
        self.nav_combo.setFixedWidth(200)
        self.nav_combo.raise_()
        self.nav_combo.currentTextChanged.connect(self.change_nav_topic)

        self.nav_sub = None
        self.latitude = None
        self.longitude = None
        self.first_gps = False
        self._populate_nav_topics()
        self.nav_timer = QTimer(self)
        self.nav_timer.timeout.connect(self._populate_nav_topics)
        self.nav_timer.start(1000)

        central = QWidget()
        v_layout = QVBoxLayout(central)
        v_layout.setContentsMargins(0,0,0,0)
        v_layout.addWidget(header, stretch=7)
        v_layout.addWidget(self.view, stretch=93)
        self.setCentralWidget(central)

        node.create_subscription(Int32, '/ublox_7/sensor/satellites', self.on_sats, 10)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        self.ros_timer.start(50)

    def get_screen_size(self):
        screen = QApplication.primaryScreen()
        if screen:
            size = screen.size()
            return int(size.width() * 0.4), int(size.height() * 0.9)
        return 800, 600

    def showEvent(self, event):
        super().showEvent(event)
        self._reposition_nav_combo()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_nav_combo()

    def _reposition_nav_combo(self):
        margin = 4
        x = self.view.width() - self.nav_combo.width() - margin
        y = margin
        self.nav_combo.move(x, y)

    def _populate_nav_topics(self):
        current = self.nav_combo.currentText()
        all_topics = self.node.get_topic_names_and_types()
        nav_topics = [name for name, types in all_topics if 'sensor_msgs/msg/NavSatFix' in types]
        items = ['---'] + nav_topics
        old = [self.nav_combo.itemText(i) for i in range(self.nav_combo.count())]
        if old == items:
            return
        self.nav_combo.blockSignals(True)
        self.nav_combo.clear()
        self.nav_combo.addItems(items)
        if current in items:
            self.nav_combo.setCurrentText(current)
        else:
            self.nav_combo.setCurrentIndex(0)
            self.change_nav_topic('---')
        self.nav_combo.blockSignals(False)

    def change_nav_topic(self, topic_name: str):
        if self.nav_sub:
            try:
                self.node.destroy_subscription(self.nav_sub)
            except Exception:
                pass
            self.nav_sub = None
        if topic_name == '---':
            self.latitude = None
            self.longitude = None
            self.pos_label.setText("<b>Lat:</b> --, <b>Lon:</b> --")
            self.first_gps = False
            return
        self.nav_sub = self.node.create_subscription(
            NavSatFix, topic_name, self.on_gps,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.latitude = None
        self.longitude = None
        self.first_gps = False

    def on_sats(self, msg: Int32):
        sats = msg.data
        color = 'red' if sats < 4 else 'orange' if sats < 8 else 'green'
        self.sat_label.setText(f": <b><span style='color:{color}'>{sats}</span></b>")

    def on_gps(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.pos_label.setText(f"<b>Lat:</b> {self.latitude:.6f}, <b>Lon:</b> {self.longitude:.6f}")
        if not self.first_gps:
            self.first_gps = True
            self._write_json()
            QTimer.singleShot(0, self._start_update_timer)

    def _start_update_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_json)
        self.timer.start(1000)

    def _write_json(self):
        with open(self.json_path, 'w') as f:
            json.dump({'lat': self.latitude, 'lon': self.longitude}, f)

    def update_json(self):
        if not self.first_gps:
            return
        self._write_json()
        self.view.page().runJavaScript('updatePosition();')

    def publish_waypoints(self):
        js = 'JSON.stringify(window.wp_list)'
        self.view.page().runJavaScript(js, self._handle_wp_json)

    def _handle_wp_json(self, result):
        msg = String()
        msg.data = result
        self.wp_pub.publish(msg)

    def toggle_waypoint_mode(self):
        if self.add_wp_button.isChecked():
            self.add_wp_button.setText('Done Waypoints')
        else:
            self.add_wp_button.setText('Add Waypoints')
        self.view.page().runJavaScript('toggleAddMode();')

    def center_map(self):
        js = '''
            fetch('/maps/gps_data.json?_='+Date.now())
            .then(r=>r.json())
            .then(o=>{ map.invalidateSize(true); map.setView([o.lat,o.lon], map.getZoom()); });
        '''
        self.view.page().runJavaScript(js)

    def toggle_hybrid(self):
        js = '''
            map.removeLayer(currLayer);
            currLayer = (currLayer===osm ? hybrid : osm);
            currLayer.addTo(map);
        '''
        self.view.page().runJavaScript(js)
        self.is_hybrid = not self.is_hybrid
        self.hybrid_button.setText('OSM' if self.is_hybrid else 'Hybrid')

    def toggle_path(self):
        js = '''
            if (map.hasLayer(poly)) { map.removeLayer(poly); }
            else { map.addLayer(poly); }
        '''
        self.view.page().runJavaScript(js)

    def start_http_server(self, root: str, directory: str):
        os.chdir(root)
        socketserver.TCPServer.allow_reuse_address = True
        handler = lambda *args, **kw: QuietHandler(*args, directory=directory, **kw)

        for port in [8080] + list(range(8081, 8090)):
            try:
                httpd = socketserver.TCPServer(("", port), handler)
                break
            except OSError:
                continue
        else:
            self.node.get_logger().error("Could not bind any port between 8080–8089")
            return

        url = f"http://localhost:{port}/maps/live_map.html"
        QTimer.singleShot(0, lambda: self.view.setUrl(QUrl(url)))

        httpd.serve_forever()


    def create_map_html(self):
        html = """
        <!DOCTYPE html>
        <html>
        <head><meta charset="utf-8"/><title>GPS Map</title>
        <meta name="viewport" content="width=device-width,initial-scale=1.0">
        <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
        <style>
          body,html,#map {margin:0;padding:0;height:100vh;}
          .leaflet-top.leaflet-right .custom-button {display:none;}
        </style>
        </head><body>
        <div id="map"></div>
        <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
        <script>
          let map, homeMarker, robotMarker, poly, path = [], firstFix = false;
          window.wp_list = []; let addMode = false, wp_poly, wp_circles = [];

          const homeIcon  = L.icon({ iconUrl:'/icons/home.png',  iconSize:[32,32], iconAnchor:[16,32] });
          const robotIcon = L.icon({ iconUrl:'/icons/robot.png', iconSize:[32,32], iconAnchor:[16,32] });
          const wpIcon    = L.icon({ iconUrl:'/icons/waypoint.png', iconSize:[24,24], iconAnchor:[12,24] });

          const osm    = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19});
          const hybrid = L.tileLayer('https://{s}.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',{maxZoom:20,subdomains:['mt0','mt1','mt2','mt3']});
          let currLayer = osm;

          function toggleAddMode() { addMode = !addMode; }
          function renderWaypoints() {
            map.eachLayer(layer => {
              if (layer.options && layer.options.icon === wpIcon) {
                map.removeLayer(layer);
              }
            });
            window.wp_list.forEach((coord, idx) => {
              const m = L.marker(coord, {icon: wpIcon}).addTo(map)
                .bindTooltip(`${idx+1}`, {permanent:true, direction:'top'});
              m.on('click', () => {
                window.wp_list.splice(idx, 1);
                renderWaypoints(); updateWaypointPolyline(); checkProximity();
              });
            });
          }
          function updateWaypointPolyline() {
            wp_poly.setLatLngs(window.wp_list);
          }
          function clearWPCircles() {
            wp_circles.forEach(c => map.removeLayer(c));
            wp_circles = [];
          }
          function checkProximity() {
            if (!robotMarker) return;
            clearWPCircles();
            const robotPos = robotMarker.getLatLng();
            window.wp_list.forEach(coord => {
              const d = map.distance(robotPos, L.latLng(coord));
              if (d <= 5) {
                const circle = L.circle(coord, {
                  color: 'green', fillColor: 'green', fillOpacity: 0.2,
                  radius: 2
                }).addTo(map);
                wp_circles.push(circle);
              }
            });
          }
          function onMapClick(e) {
            if (!addMode) return;
            window.wp_list.push([e.latlng.lat, e.latlng.lng]);
            renderWaypoints(); updateWaypointPolyline(); checkProximity();
          }

          window.updatePosition = async function() {
            try {
              const res = await fetch('/maps/gps_data.json?_='+Date.now());
              const {lat,lon} = await res.json();
              const pos = [lat, lon];
              if (!firstFix) {
                firstFix = true;
                map = L.map('map').setView(pos, 18);
                currLayer.addTo(map);
                homeMarker = L.marker(pos,{icon:homeIcon}).addTo(map);
                robotMarker= L.marker(pos,{icon:robotIcon}).addTo(map);
                path.push(pos);
                poly = L.polyline(path,{color:'red'}).addTo(map);
                map.on('click', onMapClick);
                wp_poly = L.polyline([], {color:'blue', weight:3, opacity:0.8}).addTo(map);
                return;
              }
              robotMarker.setLatLng(pos);
              path.push(pos);
              poly.setLatLngs(path);
              updateWaypointPolyline();
              checkProximity();
            } catch {}
          };
          setInterval(window.updatePosition, 1000);
        </script>
        </body>
        </html>
        """
        with open(self.map_html_path, "w") as f:
            f.write(html)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("gps_map_node")

    app = QApplication(sys.argv)
    DarkStyle(app)

    win = GPSMapWindow(node)
    win.show()

    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

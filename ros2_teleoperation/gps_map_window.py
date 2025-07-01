#!/usr/bin/env python3
import sys
import os
import threading
import json
import http.server
import socketserver

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, String

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
    QWidget, QLabel, QPushButton, QFrame
)
from PyQt6.QtGui import QFont, QPixmap
from PyQt6.QtCore import Qt, QTimer, QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView

from ros2_teleoperation.utils.window_style import WindowStyle

# Qt rendering flags for Docker
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu --disable-software-rasterizer"
os.environ["QT_QUICK_BACKEND"] = "software"
os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"

# Font for header labels
header_font = QFont()
header_font.setPointSize(12)


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP handler that silences request logs and serves from a given directory."""
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
        self.setFixedSize(w, h)

        self.latitude = None
        self.longitude = None
        self.first_gps = False

        self.wp_pub = node.create_publisher(String, '/waypoints/json', 10)


        base = "/ros2_ws/src/ros2_teleoperation"
        self.map_dir = os.path.join(base, "maps")
        os.makedirs(self.map_dir, exist_ok=True)
        self.json_path = os.path.join(self.map_dir, "gps_data.json")
        self.map_html_path = os.path.join(self.map_dir, "live_map.html")
        self.icons_dir = os.path.join(base, "icons")

        self.create_map_html()


        threading.Thread(
            target=lambda: self.start_http_server(root=base, directory=base),
            daemon=True
        ).start()

        header = QWidget()
        h_layout = QHBoxLayout(header)

        sat_icon = QLabel()
        pix = QPixmap(os.path.join(self.icons_dir, "satellite.png"))
        sat_icon.setPixmap(pix.scaled(40, 40, Qt.AspectRatioMode.KeepAspectRatio,
                                      Qt.TransformationMode.SmoothTransformation))

        self.sat_label = QLabel("--")
        self.sat_label.setFont(header_font)

        separator1 = QFrame()
        separator1.setFrameShape(QFrame.Shape.VLine)
        separator1.setFrameShadow(QFrame.Shadow.Sunken)
        separator1.setStyleSheet("color:white;")
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.Shape.VLine)
        separator2.setFrameShadow(QFrame.Shadow.Sunken)
        separator2.setStyleSheet("color:white;")

        self.pos_label = QLabel("<b>Lat:</b> --, <b>Lon:</b> --")
        self.pos_label.setTextFormat(Qt.TextFormat.RichText)
        self.pos_label.setFont(header_font)

        self.add_wp_button = QPushButton("Add Waypoints")
        self.add_wp_button.setCheckable(True)
        self.add_wp_button.clicked.connect(self.toggle_waypoint_mode)

        self.wp_button = QPushButton("Publish WP")
        self.wp_button.clicked.connect(self.publish_waypoints)

        for wgt in (sat_icon, self.sat_label, separator1, self.pos_label,
                    separator2, self.add_wp_button, self.wp_button):
            h_layout.addWidget(wgt)
            if wgt is separator1 or wgt is separator2:
                h_layout.addStretch()

        self.view = QWebEngineView()
        self.view.setUrl(QUrl("http://localhost:8080/maps/live_map.html"))

        central = QWidget()
        v_layout = QVBoxLayout(central)
        v_layout.addWidget(header)
        v_layout.addWidget(self.view)
        v_layout.setStretch(0, 7)
        v_layout.setStretch(1, 93)
        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_json)

        node.create_subscription(NavSatFix, '/applanix/lvx_client/gnss/fix', self.on_gps, 10)
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

    def toggle_waypoint_mode(self):
        if self.add_wp_button.isChecked():
            self.add_wp_button.setText("Done Waypoints")
        else:
            self.add_wp_button.setText("Add Waypoints")

        self.view.page().runJavaScript("toggleAddMode();")

    def on_sats(self, msg: Int32):
        sats = msg.data
        color = 'red' if sats < 4 else 'orange' if sats < 8 else 'green'
        self.sat_label.setText(f": <b><span style='color:{color}'>{sats}</span></b>")

    def on_gps(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.pos_label.setText(
            f"<b>Lat:</b> {self.latitude:.6f}, <b>Lon:</b> {self.longitude:.6f}"
        )
        if not self.first_gps:
            self.first_gps = True
            self._write_json()
            QTimer.singleShot(0, self.timer.start)
            self.timer.setInterval(1000)

    def _write_json(self):
        with open(self.json_path, "w") as f:
            json.dump({"lat": self.latitude, "lon": self.longitude}, f)

    def update_json(self):
        if not self.first_gps:
            return
        self._write_json()
        self.view.page().runJavaScript("updatePosition();")

    def publish_waypoints(self):
        # Recupera lista de waypoints desde JS y publica como String
        js = "JSON.stringify(window.wp_list)"
        self.view.page().runJavaScript(js, self._handle_wp_json)

    def _handle_wp_json(self, result):
        msg = String()
        msg.data = result
        self.wp_pub.publish(msg)

    def start_http_server(self, root: str, directory: str):
        """Inicia el servidor HTTP en el puerto 8080 sirviendo desde `directory`."""
        os.chdir(root)
        socketserver.TCPServer.allow_reuse_address = True
        handler = lambda *args, **kwargs: QuietHandler(*args, directory=directory, **kwargs)
        with socketserver.TCPServer(("", 8080), handler) as httpd:
            httpd.serve_forever()

    def create_map_html(self):
        html = """
        <!DOCTYPE html>
        <html>
        <head><meta charset="utf-8"/><title>GPS Map with Waypoints</title>
        <meta name="viewport" content="width=device-width,initial-scale=1.0">
        <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
        <style>
          body,html,#map {margin:0;padding:0;height:100vh;}
          .leaflet-top.leaflet-right .custom-button {
            background:white; border:2px solid gray;
            padding:5px 10px; cursor:pointer;
            border-radius:5px; font:14px sans-serif;
            margin-bottom:5px;
          }
        </style>
        </head><body>
        <div id="map"></div>
        <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
        <script>
          let map, homeMarker, robotMarker, poly, path=[] , firstFix=false;
          window.wp_list = [];
          let addMode = false, wp_poly, wp_circles = [];

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

          const toggleCtrl = L.control({position:'topright'});
          toggleCtrl.onAdd = m => {
            const d=L.DomUtil.create('div','custom-button');
            d.innerHTML='Hide Path';
            return d;
          };

          const centerCtrl = L.control({position:'topright'});
          centerCtrl.onAdd = m => {
            const d=L.DomUtil.create('div','custom-button');
            d.innerHTML='Center';
            d.onclick=async()=>{
              const r=await fetch('/maps/gps_data.json?_='+Date.now());
              const{lat,lon}=await r.json();
              m.setView([lat,lon],m.getZoom());
            };
            return d;
          };

          const styleCtrl = L.control({position:'topright'});
          styleCtrl.onAdd = m => {
            const d=L.DomUtil.create('div','custom-button');
            d.innerHTML='Hybrid';
            d.onclick=()=>{
              m.removeLayer(currLayer);
              currLayer = (currLayer===osm? hybrid:osm);
              currLayer.addTo(m);
              d.innerHTML = (currLayer===osm? 'Hybrid':'OSM');
            };
            return d;
          };

          window.updatePosition = async function() {
            try {
              const res = await fetch('/maps/gps_data.json?_='+Date.now());
              const{lat,lon}=await res.json();
              const pos=[lat,lon];
              if (!firstFix) {
                firstFix=true;
                map = L.map('map').setView(pos,18);
                currLayer.addTo(map);
                homeMarker = L.marker(pos,{icon:homeIcon}).addTo(map);
                robotMarker= L.marker(pos,{icon:robotIcon}).addTo(map);
                path.push(pos);
                poly = L.polyline(path,{color:'red'}).addTo(map);
                toggleCtrl.addTo(map);
                centerCtrl.addTo(map);
                styleCtrl.addTo(map);
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

          setInterval(window.updatePosition,1000);
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
    WindowStyle(app)

    win = GPSMapWindow(node)
    win.show()

    app.exec()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

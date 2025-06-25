#!/usr/bin/env python3
import sys
import os
import threading
import json
import rclpy
import http.server
import socketserver
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QTimer, QUrl

os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu --disable-software-rasterizer"
os.environ["QT_QUICK_BACKEND"] = "software"
os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"

class GPSMapWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("GPS Map Viewer")
        self.setGeometry(100, 100, 800, 600)

        self.latitude = None
        self.longitude = None
        self.first_gps_received = False

        self.map_dir = "/ros2_ws/src/ros2_teleoperation/maps"
        os.makedirs(self.map_dir, exist_ok=True)
        self.json_path = os.path.join(self.map_dir, "gps_data.json")
        self.map_html_path = os.path.join(self.map_dir, "live_map.html")

        self.create_map_html()

        threading.Thread(target=self.start_http_server, daemon=True).start()

        self.view = QWebEngineView()
        self.view.setUrl(QUrl("http://localhost:8080/live_map.html"))

        layout = QVBoxLayout()
        layout.addWidget(self.view)
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_json)

        node.create_subscription(NavSatFix, '/ublox_7/sensor/gps', self.gps_callback, 10)

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        if not self.first_gps_received:
            self.first_gps_received = True
            self._write_json()
            QTimer.singleShot(0, self.timer.start)
            self.timer.setInterval(1000)

    def _write_json(self):
        data = {"lat": self.latitude, "lon": self.longitude}
        with open(self.json_path, "w") as f:
            json.dump(data, f)

    def update_json(self):
        if not self.first_gps_received:
            return
        self._write_json()
        self.view.page().runJavaScript("updatePosition();")

    def start_http_server(self):
        os.chdir(self.map_dir)
        socketserver.TCPServer.allow_reuse_address = True
        handler = http.server.SimpleHTTPRequestHandler
        with socketserver.TCPServer(("", 8080), handler) as httpd:
            httpd.serve_forever()

    def create_map_html(self):
        html = """
        <!DOCTYPE html>
        <html>
            <head>
                <meta charset="utf-8"/>
                <title>GPS Map</title>
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
                <style>
                    body, html, #map { margin:0; padding:0; height:100vh; }
                    .leaflet-top.leaflet-right .custom-button {
                    background: white;
                    border: 2px solid gray;
                    padding: 5px 10px;
                    cursor: pointer;
                    border-radius: 5px;
                    font: 14px sans-serif;
                    margin-bottom: 5px;
                    }
                </style>
            </head>
            <body>
            <div id="map"></div>
            <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
            <script>
                let map, marker, polyline,
                    gpsPath = [],
                    firstFix = false;

                // define both layers
                const osmLayer = L.tileLayer(
                'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
                { maxZoom: 19 }
                );
                const hybridLayer = L.tileLayer(
                'https://{s}.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
                { maxZoom: 20, subdomains:['mt0','mt1','mt2','mt3'] }
                );
                let currentLayer = osmLayer;

                // toggle route control
                const toggleControl = L.control({position:'topright'});
                toggleControl.onAdd = m => {
                const d = L.DomUtil.create('div','custom-button');
                d.innerHTML = 'Hide Path';
                d.onclick = () => {
                    if (gpsPath.length>1 && map.hasLayer(polyline)) {
                    map.removeLayer(polyline);
                    d.innerHTML = 'Show Path';
                    } else {
                    polyline.addTo(map);
                    d.innerHTML = 'Hide Path';
                    }
                };
                return d;
                };

                // center control
                const centerControl = L.control({position:'topright'});
                centerControl.onAdd = m => {
                const d = L.DomUtil.create('div','custom-button');
                d.innerHTML = 'Center';
                d.onclick = async () => {
                    const r = await fetch('gps_data.json?_='+Date.now());
                    const {lat,lon} = await r.json();
                    map.setView([lat,lon], map.getZoom());
                };
                return d;
                };

                // style toggle control
                const styleControl = L.control({position:'topright'});
                styleControl.onAdd = m => {
                const d = L.DomUtil.create('div','custom-button');
                d.innerHTML = 'Hybrid';
                d.onclick = () => {
                    map.removeLayer(currentLayer);
                    currentLayer = (currentLayer === osmLayer) ? hybridLayer : osmLayer;
                    currentLayer.addTo(map);
                    d.innerHTML = (currentLayer === osmLayer) ? 'Hybrid' : 'OSM';
                };
                return d;
                };

                window.updatePosition = async function() {
                try {
                    const r = await fetch('gps_data.json?_='+Date.now());
                    const {lat,lon} = await r.json();
                    if (!firstFix) {
                    firstFix = true;
                    const pos = [lat,lon];
                    map = L.map('map').setView(pos, 18);
                    currentLayer.addTo(map);
                    marker = L.marker(pos).addTo(map);
                    gpsPath.push(pos);
                    polyline = L.polyline(gpsPath,{color:'red'}).addTo(map);
                    toggleControl.addTo(map);
                    centerControl.addTo(map);
                    styleControl.addTo(map);
                    return;
                    }
                    const pos = [lat,lon];
                    marker.setLatLng(pos);
                    gpsPath.push(pos);
                    polyline.setLatLngs(gpsPath);
                } catch {}
                };

                setInterval(updatePosition, 1000);
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
    window = GPSMapWindow(node)
    window.show()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    app.exec()
    rclpy.shutdown()

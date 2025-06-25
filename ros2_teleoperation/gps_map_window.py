#!/usr/bin/env python3
import sys, os, threading, json, http.server, socketserver
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
    QWidget, QLabel
)
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt, QTimer, QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView

from ros2_teleoperation.window_style import WindowStyle

os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu --disable-software-rasterizer"
os.environ["QT_QUICK_BACKEND"] = "software"
os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"

header_font = QFont()
header_font.setPointSize(14)

class GPSMapWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Map Viewer")
        self.resize(800, 800)

        self.setFixedSize(800, 800)


        self.latitude = None
        self.longitude = None
        self.first_gps = False

        base = "/ros2_ws/src/ros2_teleoperation"
        self.map_dir       = os.path.join(base, "maps")
        os.makedirs(self.map_dir, exist_ok=True)
        self.json_path     = os.path.join(self.map_dir, "gps_data.json")
        self.map_html_path = os.path.join(self.map_dir, "live_map.html")
        self.create_map_html()

        threading.Thread(target=lambda: self.start_http_server(base), daemon=True).start()

        header = QWidget()
        h_layout = QHBoxLayout(header)
        self.sat_label = QLabel()
        self.sat_label.setTextFormat(Qt.TextFormat.RichText)
        self.sat_label.setFont(header_font)
        self.sat_label.setText("<b>Satellites:</b> --")
        self.pos_label = QLabel()
        self.pos_label.setTextFormat(Qt.TextFormat.RichText)
        self.pos_label.setFont(header_font)
        self.pos_label.setText("<b>Lat:</b> --, <b>Lon:</b> --")
        h_layout.addWidget(self.sat_label)
        h_layout.addStretch()
        h_layout.addWidget(self.pos_label)

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

        node.create_subscription(NavSatFix, '/ublox_7/sensor/gps', self.on_gps, 10)
        node.create_subscription(Int32, '/ublox_7/sensor/satellites', self.on_sats, 10)

    def on_sats(self, msg: Int32):
        sats = msg.data
        color = 'red' if sats < 4 else 'orange' if sats < 8 else 'green'
        self.sat_label.setText(
            f"<b>Satellites:</b> <span style='color:{color}'>{sats}</span>"
        )

    def on_gps(self, msg: NavSatFix):
        self.latitude, self.longitude = msg.latitude, msg.longitude
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

    def start_http_server(self, root):
        os.chdir(root)
        socketserver.TCPServer.allow_reuse_address = True
        handler = http.server.SimpleHTTPRequestHandler
        with socketserver.TCPServer(("", 8080), handler) as httpd:
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

          const homeIcon  = L.icon({ iconUrl:'/icons/home.png',  iconSize:[32,32], iconAnchor:[16,32] });
          const robotIcon = L.icon({ iconUrl:'/icons/robot.png', iconSize:[32,32], iconAnchor:[16,32] });

          const osm    = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19});
          const hybrid = L.tileLayer('https://{s}.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',{ maxZoom:20, subdomains:['mt0','mt1','mt2','mt3'] });
          let currLayer = osm;

          const toggleCtrl = L.control({position:'topright'});
          toggleCtrl.onAdd = m=>{
            const d=L.DomUtil.create('div','custom-button'); d.innerHTML='Hide Path';
            d.onclick=()=>{
              if(path.length>1 && m.hasLayer(poly)) { m.removeLayer(poly); d.innerHTML='Show Path'; }
              else { poly.addTo(m); d.innerHTML='Hide Path'; }
            };
            return d;
          };

          const centerCtrl = L.control({position:'topright'});
          centerCtrl.onAdd = m=>{
            const d=L.DomUtil.create('div','custom-button'); d.innerHTML='Center';
            d.onclick=async()=>{
              const r=await fetch('/maps/gps_data.json?_='+Date.now());
              const {lat,lon}=await r.json();
              m.setView([lat,lon],m.getZoom());
            };
            return d;
          };

          const styleCtrl = L.control({position:'topright'});
          styleCtrl.onAdd = m=>{
            const d=L.DomUtil.create('div','custom-button'); d.innerHTML='Hybrid';
            d.onclick=()=>{
              m.removeLayer(currLayer);
              currLayer = currLayer===osm? hybrid: osm;
              currLayer.addTo(m);
              d.innerHTML = currLayer===osm? 'Hybrid':'OSM';
            };
            return d;
          };

          window.updatePosition = async function(){
            try {
              const res=await fetch('/maps/gps_data.json?_='+Date.now());
              const {lat,lon}=await res.json(); const pos=[lat,lon];
              if(!firstFix){
                firstFix=true; map=L.map('map').setView(pos,18);
                currLayer.addTo(map);
                homeMarker  = L.marker(pos,{icon:homeIcon}).addTo(map);
                robotMarker = L.marker(pos,{icon:robotIcon}).addTo(map);
                path.push(pos); poly = L.polyline(path,{color:'red'}).addTo(map);
                toggleCtrl.addTo(map); centerCtrl.addTo(map); styleCtrl.addTo(map);
                return;
              }
              robotMarker.setLatLng(pos); path.push(pos); poly.setLatLngs(path);
            } catch {}
          };
          setInterval(updatePosition,1000);
        </script>
        </body>
        </html>
        """
        with open(self.map_html_path, "w") as f:
            f.write(html)

def main(args=None):
    rclpy.init()
    node = rclpy.create_node("gps_map_node")
    app = QApplication(sys.argv)
    WindowStyle(app)
    win = GPSMapWindow(node)
    win.show()
    import threading
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    app.exec()
    rclpy.shutdown()

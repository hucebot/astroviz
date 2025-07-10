#!/usr/bin/env python3
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt6.QtCore import QTimer
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData
from urdf_parser_py.urdf import URDF

class URDFViewer(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("URDF Viewer")
        self.resize(800, 600)

        # Storage for URDF XML and mesh items
        self.robot_xml = ''
        self.mesh_items = []

        # Subscribe to /robot_description topic
        self.sub = self.node.create_subscription(
            String,
            'robot_description',
            self.urdf_callback,
            10
        )

        # Set up the OpenGL view
        widget = QWidget()
        self.setCentralWidget(widget)
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0,0,0,0)

        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.opts['distance'] = 2.0
        layout.addWidget(self.gl_widget)

        # Add a grid for reference
        self.grid = gl.GLGridItem()
        self.grid.scale(0.1, 0.1, 0.1)
        self.gl_widget.addItem(self.grid)

        # Spin ROS periodically
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        self.ros_timer.start(10)

    def urdf_callback(self, msg: String):
        print("Received robot_description message")
        xml = msg.data
        if xml and xml != self.robot_xml:
            self.robot_xml = xml
            self.node.get_logger().info("Received robot_description, rebuilding model...")
            self._rebuild_model()

    def _rebuild_model(self):
        # Clear previous mesh items
        for item in self.mesh_items:
            try:
                self.gl_widget.removeItem(item)
            except Exception:
                pass
        self.mesh_items.clear()

        # Parse URDF
        try:
            robot = URDF.from_xml_string(self.robot_xml)
            self.node.get_logger().info(f"Loaded URDF for robot: {robot.name}")
        except Exception as e:
            self.node.get_logger().error(f"URDF parse error: {e}")
            return

        # Build visuals
        for link in robot.links:
            visual = link.visual
            if not visual or not visual.geometry:
                continue
            meshdata = self._create_mesh(visual.geometry)
            if meshdata is None:
                continue
            mesh_item = gl.GLMeshItem(meshdata=meshdata, smooth=True, drawEdges=False)
            mesh_item.setGLOptions('opaque')
            # Apply origin
            origin = visual.origin
            if origin:
                xyz = origin.xyz or [0,0,0]
                rpy = origin.rpy or [0,0,0]
                mesh_item.translate(*xyz)
                mesh_item.rotate(np.degrees(rpy[0]), 1, 0, 0)
                mesh_item.rotate(np.degrees(rpy[1]), 0, 1, 0)
                mesh_item.rotate(np.degrees(rpy[2]), 0, 0, 1)
            self.gl_widget.addItem(mesh_item)
            self.mesh_items.append(mesh_item)

    def _create_mesh(self, geom):
        if geom.box:
            size = geom.box.size
            return self._create_box_mesh(size)
        if geom.cylinder:
            return self._create_cylinder_mesh(geom.cylinder.radius, geom.cylinder.length)
        if geom.sphere:
            return self._create_sphere_mesh(geom.sphere.radius)
        self.node.get_logger().warn("Unsupported geometry type in URDF visual")
        return None

    def _create_box_mesh(self, size):
        sx, sy, sz = size
        verts = np.array([
            [-sx/2,-sy/2,-sz/2],[sx/2,-sy/2,-sz/2],[sx/2,sy/2,-sz/2],[-sx/2,sy/2,-sz/2],
            [-sx/2,-sy/2,sz/2],[sx/2,-sy/2,sz/2],[sx/2,sy/2,sz/2],[-sx/2,sy/2,sz/2]
        ], dtype=np.float32)
        faces = np.array([
            [0,1,2],[0,2,3],[4,5,6],[4,6,7],
            [0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]
        ], dtype=np.uint32)
        return MeshData(vertexes=verts, faces=faces)

    def _create_cylinder_mesh(self, radius, length, slices=32):
        verts = []
        faces = []
        for i in range(slices):
            theta = 2*np.pi*i/slices
            x,y = radius*np.cos(theta), radius*np.sin(theta)
            verts.append([x,y,-length/2]); verts.append([x,y,length/2])
        for i in range(slices):
            i0=2*i; i1=2*((i+1)%slices)
            faces.append([i0,i1,i0+1]); faces.append([i1,i1+1,i0+1])
        return MeshData(vertexes=np.array(verts,dtype=np.float32), faces=np.array(faces,dtype=np.uint32))

    def _create_sphere_mesh(self, radius, slices=16, stacks=16):
        verts=[]; faces=[]
        for i in range(stacks+1):
            phi=np.pi*i/stacks
            for j in range(slices):
                theta=2*np.pi*j/slices
                verts.append([
                    radius*np.sin(phi)*np.cos(theta),
                    radius*np.sin(phi)*np.sin(theta),
                    radius*np.cos(phi)
                ])
        for i in range(stacks):
            for j in range(slices):
                p0=i*slices+j; p1=p0+slices
                p2=p1+1 if j+1<slices else p1-(slices-1)
                p3=p0+1 if j+1<slices else p0-(slices-1)
                faces.append([p0,p1,p2]); faces.append([p0,p2,p3])
        return MeshData(vertexes=np.array(verts,dtype=np.float32), faces=np.array(faces,dtype=np.uint32))

    def closeEvent(self, event):
        if self.sub:
            self.node.destroy_subscription(self.sub)
        return super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('urdf_viewer')
    app = QApplication(sys.argv)
    viewer = URDFViewer(node)
    viewer.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

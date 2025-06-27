
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QGridLayout, QVBoxLayout, QWidget
import rclpy
from rclpy.node import Node

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Camera - ROS2')
        self.setGeometry(400, 400, 400, 400)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout()
        
        self.central_widget.setLayout(self.layout)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])
    window = MainWindow(rclpy.create_node('camera_window'))
    window.show()
    app.exec()
    rclpy.shutdown()
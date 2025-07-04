from ros2_teleoperation.gps_map_window import GPSMapWindow
from ros2_teleoperation.camera_window import CameraViewer
from ros2_teleoperation.imu_window import MainWindow as IMUWindow
from ros2_teleoperation.lidar_window import LiDARViewer
from ros2_teleoperation.teleoperation_window import TeleoperationViewer
from ros2_teleoperation.plot_window import GraphViewer
from ros2_teleoperation.gridmap_window import GridMapViewer

VIEW_TYPES = {
    'GPS Map': GPSMapWindow,
    'Camera': CameraViewer,
    'IMU': IMUWindow,
    'LiDAR': LiDARViewer,
    'Teleoperation': TeleoperationViewer,
    'System Health': GraphViewer,
    'GridMap': GridMapViewer
}

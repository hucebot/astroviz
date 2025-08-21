from astroviz.gps_map_window import GPSMapWindow
from astroviz.camera_window import CameraViewer
from astroviz.imu_window import MainWindow as IMUWindow
from astroviz.lidar_window import LiDARViewer
from astroviz.teleoperation_window import TeleoperationViewer
from astroviz.plot_window import GraphViewer
from astroviz.grid_map_window import GridMapViewer
from astroviz.orthogonal_window import OrthogonalViewer
from astroviz.robot_state_window import RobotStateViewer
from astroviz.motor_state_viewer import MotorTableViewer      

VIEW_TYPES = {
    'GPS Map': GPSMapWindow,
    'Camera': CameraViewer,
    'IMU': IMUWindow,
    'LiDAR': LiDARViewer,
    'Teleoperation': TeleoperationViewer,
    'System Health': GraphViewer,
    'GridMap': GridMapViewer,
    'Orthogonal': OrthogonalViewer,
    'Motor State': MotorTableViewer,
    'Robot State': RobotStateViewer
}

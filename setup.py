from setuptools import find_packages, setup

package_name = 'astroviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdonoso',
    maintainer_email='clemente.donosok@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_viewer = astroviz.dashboard_window:main',
            'gps_map_viewer = astroviz.gps_map_window:main',
            'camera_viewer = astroviz.camera_window:main',
            'lidar_viewer = astroviz.lidar_window:main',
            'imu_viewer = astroviz.imu_window:main',
            'plot_viewer = astroviz.plot_window:main',
            'teleoperation_viewer = astroviz.teleoperation_window:main',
            'gridmap_viewer = astroviz.grid_map_window:main',
            'orthogonal_viewer = astroviz.orthogonal_window:main',
            'gstreamer_viewer = astroviz.gstreamer_window:main',
            'robot_state_viewer = astroviz.robot_state_window:main',
            'motor_state_viewer = astroviz.motor_state_viewer:main',
            'dummy_map_publisher = astroviz.fake_map:main',
        ],
    },
)

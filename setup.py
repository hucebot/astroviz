from setuptools import find_packages, setup
import os
from glob import glob

package_name = "astroviz"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/dashboard_launcher.launch.py",
            ],
        ),
        (
            os.path.join("share", package_name, "icons"),
            glob("icons/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cdonoso",
    maintainer_email="clemente.donosok@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            #'dashboard_viewer = astroviz.dashboard_window:main', #WORKING BUT NEED TO IMPROVE THE RENDERING
            "gps_map_viewer = astroviz.gps_map_window:main",
            "camera_viewer = astroviz.camera_window:main",
            "lidar_viewer = astroviz.lidar_window:main",
            "imu_viewer = astroviz.imu_window:main",
            "plot_viewer = astroviz.plot_window:main",
            "teleoperation_viewer = astroviz.teleoperation_window:main",
            "gridmap_viewer = astroviz.grid_map_window:main",
            "orthogonal_viewer = astroviz.orthogonal_window:main",
            "gstreamer_viewer = astroviz.gstreamer_window:main",
            "robot_state_viewer = astroviz.robot_state_window:main",
            "motor_state_viewer = astroviz.motor_state_viewer:main",
            "dummy_trajectories_publisher = astroviz.tools.dumy_trajectories:main",
            "gstreamer_shelfy_viewer = astroviz.gstreamer_shelfy_window:main",
            "mobile_base_viewer = astroviz.mobile_base_window:main",
            "tts_viewer = astroviz.tts_window:main",
            "audio_player_viewer = astroviz.audio_player_window:main",
            "shelfy_dashboard_viewer = astroviz.shelfy_dashboard_window:main",
            "cafeteria_menu_viewer = astroviz.cafeteria_menu_window:main",
        ],
    },
)

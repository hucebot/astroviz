#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import (
    Float64MultiArray, MultiArrayLayout, MultiArrayDimension, Header
)

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.get_logger().info('Initializing TrajectoryNode')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('total_trajectories', 20),
                ('update_period', 0.05),
                ('number_of_points', 40),
                ('topic', 'dummy_trajectory'),
                ('frame_id', 'pelvis'),
            ],
        )
        self.total_trajectories = self.get_parameter('total_trajectories').get_parameter_value().integer_value
        self.update_period = self.get_parameter('update_period').get_parameter_value().double_value
        self.number_of_points = self.get_parameter('number_of_points').get_parameter_value().integer_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        base_trajectory = [
            (0.0, 0.5, 0.0),
            (0.5, 1.3, 0.2),
            (1.0, 2.2, 0.5),
            (2.0, 2.5, 0.9),
            (3.0, 2.8, 1.3),
            (4.0, 3.0, 1.8),
            (5.0, 2.5, 2.0),
            (5.5, 2.0, 2.0),
            (6.0, 1.0, 1.7),
            (6.5, 0.0, 1.2),
            (7.0, -1.0, 1.1),
            (7.5, -2.0, 1.0),
            (8.0, -2.5, 0.9),
            (9.0, -2.5, 0.5),
            (10.0, -2.0, 0.2),
            (10.5, -1.0, 0.1),
        ]
        self.base_trajectory = [(x/10.0, y/10.0, z/10.0) for (x, y, z) in base_trajectory]

        self.current_step = 1

        self.trajectories = self.generate_trajectories(self.total_trajectories)

        self.trajectories_publisher = self.create_publisher(Float64MultiArray, self.topic, 10)

        header_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.header_publisher = self.create_publisher(Header, f'{self.topic}/header', header_qos)

        self.timer = self.create_timer(self.update_period, self.publish_incremental_trajectories)

    def generate_trajectories(self, num_trajectories: int):
        trajectories = []
        for _ in range(num_trajectories):
            random_traj = []
            for (bx, by, bz) in self.base_trajectory:
                dx, dy, dz = (random.uniform(-20.0, 20.0) for _ in range(3))
                length = math.sqrt(dx*dx + dy*dy + dz*dz)
                if length > 1e-9:
                    dx, dy, dz = dx/length, dy/length, dz/length

                r = random.uniform(0.0, 0.05)
                nx, ny, nz = bx + dx*r, by + dy*r, bz + dz*r
                random_traj.append((nx, ny, nz))
            trajectories.append(random_traj)
        return trajectories

    def publish_incremental_trajectories(self):
        max_points = min(self.number_of_points, len(self.base_trajectory))
        n_pts = min(self.current_step, max_points)

        data = []
        for trajectory in self.trajectories:
            data.extend(trajectory[:n_pts])

        T = self.total_trajectories
        P = n_pts

        layout = MultiArrayLayout()
        layout.dim = [
            MultiArrayDimension(label='trajectories', size=T, stride=P * 3),
            MultiArrayDimension(label='points',       size=P, stride=3),
            MultiArrayDimension(label='xyz',          size=3, stride=1),
        ]
        layout.data_offset = 0

        msg = Float64MultiArray()
        msg.layout = layout
        msg.data = [coord for point in data for coord in point]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        self.header_publisher.publish(header)
        self.trajectories_publisher.publish(msg)

        self.current_step += 1
        if self.current_step > max_points:
            self.current_step = 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

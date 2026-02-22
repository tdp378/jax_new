import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class InitialPose(Node):
    def __init__(self):
        super().__init__("jax_initial_pose")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/legs_controller/joint_trajectory",
            10
        )

        self.joints = [
            "front_left_hip_joint", "front_left_thigh_joint", "front_left_calf_joint",
            "front_right_hip_joint", "front_right_thigh_joint", "front_right_calf_joint",
            "rear_left_hip_joint", "rear_left_thigh_joint", "rear_left_calf_joint",
            "rear_right_hip_joint", "rear_right_thigh_joint", "rear_right_calf_joint",
        ]

        # Stable crouch that pulls feet back under hips
        self.positions = [
            0.0, 0.6, -0.5,
            0.0, 0.6, -0.5,
            0.0, 0.6, -0.5,
            0.0, 0.6, -0.5,
        ]

    def make_msg(self) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = self.joints

        pt = JointTrajectoryPoint()
        pt.positions = self.positions
        pt.time_from_start = Duration(sec=0, nanosec=300_000_000)  # 0.30s (fast)
        msg.points = [pt]
        return msg


def main():
    rclpy.init()
    node = InitialPose()

    msg = node.make_msg()

    # Publish hard for 2 seconds at 100 Hz using WALL TIME (not sim time)
    start = time.time()
    while rclpy.ok() and (time.time() - start) < 2.0:
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.01)

    node.get_logger().info("Initial pose published for 2s, exiting.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

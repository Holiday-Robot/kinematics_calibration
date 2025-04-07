import time
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from joint_state_recorder import JointStatesRecorder
from rclpy.clock import Clock


class JointStatesRecorderXArm6(JointStatesRecorder):
    def __init__(self, folder_name: str):
        super().__init__(folder_name, 6, "/hday/xarm6/joint_state")
        # self._pykeyboard.stop()

        self.pose_subsciber = self.create_subscription(
            PoseStamped, "/hday/xarm6/ef_pose", self._pose_callback, 10
        )

        self._last_time_pressed = time.time()
        self._pose = np.zeros(7)

    def _pose_callback(self, pose: PoseStamped) -> None:
        self._pose = np.array(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
            ]
        )

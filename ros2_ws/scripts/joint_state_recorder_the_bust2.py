import time
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from joint_state_recorder import JointStatesRecorder
import panda_py
from rclpy.clock import Clock

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


class JointStatesRecorderTheBust2(JointStatesRecorder):
    def __init__(self, folder_name: str):

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        super().__init__(folder_name, 7, "/arm/joint_state", qos=self.qos_profile)
        # self._pykeyboard.stop()

        self.pose_subsciber = self.create_subscription(
            PoseStamped, "/hday/motion_planner/ef_pose", self._pose_callback, 10
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

import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from joint_state_recorder import JointStatesRecorder

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


class JointStatesRecorderFridayLeft(JointStatesRecorder):
    def __init__(self, folder_name: str):

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        super().__init__(folder_name, 7, "/hday/friday/left_arm/joint_state", qos=10)
        self._pykeyboard.stop()

        self._last_time_pressed = time.time()

#!/usr/bin/env python3

import numpy as np
import os
import sys
import threading
import time
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D, Twist
import mujoco
import mujoco.viewer
from ament_index_python.packages import get_package_share_directory

pos = {
    "left_leg": 0,
    "right_leg": 0,
    "left_wheel": 0, 
    "right_wheel": 0,
}

joint = [
    "left_leg_joint",
    "right_leg_joint", 
    "left_wheel_joint",
    "right_wheel_joint",
]

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        pkg_dir_path = get_package_share_directory('mujoco_bootcamp')
        self.paused = False
        self.scene_root = os.path.join(pkg_dir_path, "mjcf/manipulator.xml")
        print("path : " + self.scene_root)
        self.m = mujoco.MjModel.from_xml_path(self.scene_root)
        self.d = mujoco.MjData(self.m)
        self.simulation_thread = threading.Thread(target=self.mujoco)
        self.simulation_thread.start()
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.subscription_joint = self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

    def mujoco(self):
        with mujoco.viewer.launch_passive(self.m, self.d, key_callback=self.key_callback) as viewer:
            start_time = time.time()
            zero_time = self.d.time
            while rclpy.ok():
                step_start = time.time()

                # self.d.ctrl[joint.index("left_leg_joint")] = pos["left_leg"]
                # self.d.ctrl[joint.index("right_leg_joint")] = pos["right_leg"]
                # self.d.ctrl[joint.index("left_wheel_joint")] = self.left_wheel_speed
                # self.d.ctrl[joint.index("right_wheel_joint")] = self.right_wheel_speed

                mujoco.mj_step(self.m, self.d)
                viewer.sync()

                clock_msg = Clock()
                time_sec = self.d.time - zero_time
                clock_msg.clock.sec = int(time_sec)
                clock_msg.clock.nanosec = int((time_sec - int(time_sec)) * 1e9)
                self.clock_pub.publish(clock_msg)

                time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

                rclpy.spin_once(self, timeout_sec=0.0)

    def joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in pos:
                pos[name] = msg.velocity[i]

    def key_callback(self, keycode):
        if chr(keycode) == ' ':
            self.paused = not self.paused

def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()
    rclpy.spin(mujoco_node)
    mujoco_node.simulation_thread.join()

if __name__ == '__main__':
    main()
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
        self.scene_root = os.path.join(pkg_dir_path, "mjcf/2d_manipulator.xml")
        self.cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.cam)
        print("path : " + self.scene_root)
        self.m = mujoco.MjModel.from_xml_path(self.scene_root)
        self.d = mujoco.MjData(self.m)
        self.simulation_thread = threading.Thread(target=self.mujoco)
        self.simulation_thread.start()
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.subscription_joint = self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.cam_pose_x = 0
        self.cam_pose_y = 0
        self.cam_pose_z = 0 
        self.cam_distance = 5
        self.cam_elevation = -89
        self.cam_azimuth = 90
        self.set_cam_pose_flag = True

        N = 500
        q0_start = 0
        q0_end = 1.57
        q1_start = 0
        q1_end = -2*3.14
        self.q0 = np.linspace(q0_start, q0_end, N)
        self.q1 = np.linspace(q1_start, q1_end, N)
        self.i = 0

        # init angle
        self.d.qpos[0] = q0_start
        self.d.qpos[1] = q1_start

        self.get_camera_config = 0

    def mujoco(self):
        with mujoco.viewer.launch_passive(self.m, self.d, key_callback=self.key_callback) as viewer:
            start_time = time.time()
            zero_time = self.d.time
            while rclpy.ok():
                step_start = time.time()

                if (self.i > 500):
                    self.i = 0;

                self.d.qpos[0] = self.q0[self.i];
                self.d.qpos[1] = self.q1[self.i];

                # mujoco.mj_forward(self.m, self.d)
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
                    self.i += 1
                    print(self.d.site_xpos[0])
                    print(self.d.site_xmat[0])

                rclpy.spin_once(self, timeout_sec=0.0)

                if self.set_cam_pose_flag == True:
                    viewer.cam.lookat[0] = self.cam_pose_x
                    viewer.cam.lookat[1] = self.cam_pose_y
                    viewer.cam.lookat[2] = self.cam_pose_z
                    viewer.cam.distance = self.cam_distance
                    viewer.cam.elevation = self.cam_elevation
                    viewer.cam.azimuth = self.cam_azimuth
                    self.set_cam_pose_flag = False

                if (self.get_camera_config == 1):
                    print('cam.azimuth =',viewer.cam.azimuth,';','cam.elevation =',viewer.cam.elevation,';','cam.distance = ',viewer.cam.distance)
                    print('cam.lookat =np.array([',viewer.cam.lookat[0],',',viewer.cam.lookat[1],',',viewer.cam.lookat[2],'])')
                    self.get_camera_config = 0

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
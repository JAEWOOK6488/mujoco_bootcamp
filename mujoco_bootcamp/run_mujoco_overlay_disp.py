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

_overlay = {}
vel_x = 2
vel_z = 4

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        pkg_dir_path = get_package_share_directory('mujoco_bootcamp')
        self.paused = False
        self.scene_root = os.path.join(pkg_dir_path, "mjcf/ball_game.xml")
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
        self.cam_distance = 10
        self.cam_elevation = -40
        self.cam_azimuth = 90
        self.set_cam_pose_flag = True
        self.get_camera_config = 0

        # 초기위치 설정
        self.d.qpos[0] = 0
        self.d.qpos[1] = 0
        self.d.qpos[2] = 0.1 # 구의 반지름과 동일한 높이로 세팅

        #initialize the controller
        self.init_controller(self.m, self.d)

        #set the controller
        mujoco.set_mjcb_control(self.controller)

    def init_controller(self, model,data):
        #initialize the controller here. This function is called once, in the beginning
        pass

    def controller(self, model, data):
        # Drag Force = -c*vx*|v| i + -c*vy*|v| j + -c*vz*|v| k
        vx = self.d.qvel[0]
        vy = self.d.qvel[1]
        vz = self.d.qvel[2]
        v = np.sqrt(vx**2+vy**2+vz**2)
        c = 0.25

        # self.d.qfrc_applied[0] = -c*vx*v
        # self.d.qfrc_applied[1] = -c*vy*v
        # self.d.qfrc_applied[2] = -c*vz*v

        self.d.xfrc_applied[1][0] = -c*vx*v
        self.d.xfrc_applied[1][1] = -c*vy*v
        self.d.xfrc_applied[1][2] = -c*vz*v

    def mujoco(self):
        global vel_x, vel_z
        with mujoco.viewer.launch_passive(self.m, self.d, key_callback=self.key_callback) as viewer:
            start_time = time.time()
            zero_time = self.d.time
            while rclpy.ok():
                step_start = time.time()

                self.add_text(self.d, viewer, "HELLO")

                # mujoco.mj_forward(self.m, self.d)
                if not self.paused:
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
        pass

    def key_callback(self, keycode):
        global vel_x, vel_z

        if chr(keycode) == ' ':
            self.paused = not self.paused
        
        if chr(keycode) == 'R':
            mujoco.mj_resetData(self.m, self.d)
            mujoco.mj_forward(self.m, self.d)

        if chr(keycode) == 'S':
            self.d.qvel[0] = vel_x # m/s
            self.d.qvel[2] = vel_z

        print(f"Pressed keycode: {keycode}, char: {repr(chr(keycode))}")

    def add_text(self, data, viewer, input):
        # create an invisibale geom and add label on it
        geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            type=mujoco.mjtGeom.mjGEOM_LABEL,
            size=np.array([0.2, 0.2, 0.2]),  # label_size
            pos=data.qpos[:3]+np.array([0.0, 0.0, 1.0]),  # lebel position, here is 1 meter above the root joint
            mat=np.eye(3).flatten(),  # label orientation, here is no rotation
            rgba=np.array([0, 0, 0, 0])  # invisible
        )
        geom.label = input  # receive string input only
        viewer.user_scn.ngeom += 1

def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()
    rclpy.spin(mujoco_node)
    mujoco_node.simulation_thread.join()

if __name__ == '__main__':
    main()
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
vel_x = 2
vel_z = 4

_overlay = {}

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        pkg_dir_path = get_package_share_directory('mujoco_bootcamp')
        self.paused = False
        self.scene_root = os.path.join(pkg_dir_path, "mjcf/ball_game.xml")
        print("path : " + self.scene_root)
        self.m = mj.MjModel.from_xml_path(self.scene_root)
        self.d = mj.MjData(self.m)
        self.simulation_thread = threading.Thread(target=self.mujoco, args=(self.m, self.d))
        self.simulation_thread.start()
        self.print_camera_config = 0

        self.d.qpos[0] = 0
        self.d.qpos[1] = 0
        self.d.qpos[2] = 0.1 # 구의 반지름과 동일한 높이로 세팅

    def mujoco(self, model, data):
        global _overlay 
        
        # MuJoCo data structures
        cam = mj.MjvCamera()                        # Abstract camera
        opt = mj.MjvOption()                        # visualization options

        # Init GLFW, create window, make OpenGL context current, request v-sync
        glfw.init()
        window = glfw.create_window(1200, 900, "Mujoco", None, None)
        glfw.make_context_current(window)
        glfw.swap_interval(1)

        # initialize visualization data structures
        mj.mjv_defaultCamera(cam)
        mj.mjv_defaultOption(opt)
        scene = mj.MjvScene(model, maxgeom=10000)
        context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

        # install GLFW mouse and keyboard callbacks
        glfw.set_key_callback(window, self.keyboard)
        glfw.set_cursor_pos_callback(window, self.mouse_move)
        glfw.set_mouse_button_callback(window, self.mouse_button)
        glfw.set_scroll_callback(window, self.scroll)

        # Example on how to set camera configuration
        # cam.azimuth = 90
        # cam.elevation = -45
        # cam.distance = 2
        # cam.lookat = np.array([0.0, 0.0, 0])
        cam.azimuth = 84.69038085937497 ; cam.elevation = -16.31253662109375 ; cam.distance =  7.5876800784056755
        cam.lookat =np.array([ 1 , 0.0 , 1.0 ])

        #initialize the controller
        self.init_controller(model, data)

        #set the controller
        mj.set_mjcb_control(self.controller)
            
        while not glfw.window_should_close(window):
            time_prev = data.time

            while (data.time - time_prev < 1.0/60.0):
                mj.mj_step(model, data)

            # if (data.time>=simend):
            #     break;

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            #create overlay
            self.create_overlay(model,data)

            #print camera configuration (help to initialize the view)
            if (self.print_camera_config==1):
                print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
                print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')


            # Update scene and render
            mj.mjv_updateScene(model, data, opt, None, cam,
                            mj.mjtCatBit.mjCAT_ALL.value, scene)
            mj.mjr_render(viewport, scene, context)

            # overlay items
            for gridpos, [t1, t2] in _overlay.items():

                mj.mjr_overlay(
                    mj.mjtFontScale.mjFONTSCALE_150,
                    gridpos,
                    viewport,
                    t1,
                    t2,
                    context)

            # clear overlay
            _overlay.clear()

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()
        
        glfw.terminate()

    def add_overlay(self, gridpos, text1, text2):
        global _overlay 

        if gridpos not in _overlay:
            _overlay[gridpos] = ["", ""]
        _overlay[gridpos][0] += text1 + "\n"
        _overlay[gridpos][1] += text2 + "\n"

    #HINT1: add the overlay here
    def create_overlay(self, model, data):
        topleft = mj.mjtGridPos.mjGRID_TOPLEFT
        topright = mj.mjtGridPos.mjGRID_TOPRIGHT
        bottomleft = mj.mjtGridPos.mjGRID_BOTTOMLEFT
        bottomright = mj.mjtGridPos.mjGRID_BOTTOMRIGHT

        self.add_overlay(
            bottomleft,
            "Restart",'r' ,
            )

        self.add_overlay(
            bottomleft,
            "Start",'s' ,
            )

        self.add_overlay(
            bottomleft,
            "Start",'time' ,
            )

    #HINT2: add the logics for key press here
    def keyboard(self, window, key, scancode, act, mods):
        global vel_x, vel_z
        if (act == glfw.PRESS and key == glfw.KEY_R):
            mj.mj_resetData(self.m, self.d)
            mj.mj_forward(self.m, self.d)
            print('Reset Ball')

        if act == glfw.PRESS and key == glfw.KEY_S:
            self.d.qvel[0] = vel_x # m/s
            self.d.qvel[2] = vel_z
            print('Jumping ball')

    def init_controller(self, model, data):
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
        
    def mouse_button(self, window, button, act, mods):
        # update button state
        global button_left
        global button_middle
        global button_right

        button_left = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        button_middle = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        button_right = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        # update mouse position
        glfw.get_cursor_pos(window)

    def mouse_move(self, window, xpos, ypos):
        # compute mouse displacement, save
        global lastx
        global lasty
        global button_left
        global button_middle
        global button_right

        dx = xpos - lastx
        dy = ypos - lasty
        lastx = xpos
        lasty = ypos

        # no buttons down: nothing to do
        if (not button_left) and (not button_middle) and (not button_right):
            return

        # get current window size
        width, height = glfw.get_window_size(window)

        # get shift key state
        PRESS_LEFT_SHIFT = glfw.get_key(
            window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        PRESS_RIGHT_SHIFT = glfw.get_key(
            window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

        # determine action based on mouse button
        if button_right:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mj.mjtMouse.mjMOUSE_MOVE_V
        elif button_left:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_ROTATE_H
            else:
                action = mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM

        mj.mjv_moveCamera(self.m, action, dx/height,
                        dy/height, scene, cam)

    def scroll(self, window, xoffset, yoffset):
        action = mj.mjtMouse.mjMOUSE_ZOOM
        mj.mjv_moveCamera(self.m, action, 0.0, -0.05 *
                        yoffset, scene, cam)

def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()
    rclpy.spin(mujoco_node)
    mujoco_node.simulation_thread.join()

if __name__ == '__main__':
    main()
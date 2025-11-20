import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import cv2
import time
import pandas as pd
import os
import rospkg

from aero_force_v2 import aero, nWagner
from Controller_PID_v1 import *
from flapping_controller import FlappingController
from utility_functions.rotation_transformations import *

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
flapping_enabled = False

def keyboard(window, key, scancode, act, mods):
    global flapping_enabled
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
    
    if act == glfw.PRESS and key == glfw.KEY_F:
        flapping_enabled = not flapping_enabled
        print(f"Flapping enabled: {flapping_enabled}")

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    if (not button_left) and (not button_middle) and (not button_right):
        return

    width, height = glfw.get_window_size(window)
    PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    if button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height, dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)

def get_bodyIDs(body_list, model):
    bodyID_dic = {}
    jntID_dic = {}
    posID_dic = {}
    jvelID_dic = {}
    for bodyName in body_list:
        try:
            mjID = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, bodyName)
            jntID = model.body_jntadr[mjID]
            jvelID = model.body_dofadr[mjID]
            posID = model.jnt_qposadr[jntID]
            bodyID_dic[bodyName] = mjID
            jntID_dic[bodyName] = jntID
            posID_dic[bodyName] = posID
            jvelID_dic[bodyName] = jvelID
        except:
            print(f"Warning: Body '{bodyName}' not found")
    return bodyID_dic, jntID_dic, posID_dic, jvelID_dic

def get_jntIDs(jnt_list, model):
    jointID_dic = {}
    for jointName in jnt_list:
        try:
            jointID = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jointName)
            jointID_dic[jointName] = jointID
        except:
            print(f"Warning: Joint '{jointName}' not found")
    return jointID_dic

def original_states(model, data, posID_dic, jvelID_dic):
    xd = np.array([0.0] * 22)
    
    if "L3" in posID_dic and "L7" in posID_dic:
        xd[0] = data.qpos[posID_dic["L3"]] + np.deg2rad(11.345825599281223)
        xd[1] = data.qpos[posID_dic["L7"]] - np.deg2rad(27.45260202) + xd[0]
        xd[5] = data.qvel[jvelID_dic["L3"]]
        xd[6] = data.qvel[jvelID_dic["L7"]]
    
    xd[2:5] = data.sensordata[0:3]
    xd[7:10] = data.sensordata[7:10]
    xd[10:13] = data.sensordata[10:13]

    if np.linalg.norm(data.sensordata[3:7]) == 0:
        data.sensordata[3:7] = [1,0,0,0]
    R_body = quat2rot(data.sensordata[3:7])
    xd[13:23] = list(np.transpose(R_body).flatten())
    return xd, R_body

""" ------------------------MAIN SIMULATION--------------------------------------- """
# Get paths
rospack = rospkg.RosPack()
package_path = rospack.get_path('ros_flappy_sim')
csv_path = os.path.join(package_path, 'src', 'JointAngleData.csv')
# xml_path = os.path.join(package_path, 'worlds', 'Flappy_v10_weld.xml')
xml_path = os.path.join(package_path, 'worlds', 'Cage_scene.xml')


# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# DIAGNOSTIC: Check for issues
print(f"Number of bodies: {model.nbody}")
print(f"Number of geoms: {model.ngeom}")
print(f"Number of DOFs: {model.nv}")
print(f"Body names: {[mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)]}")
print(f"Initial qpos (first 10): {data.qpos[:10]}")
print(f"Gravity: {model.opt.gravity}")
print(f"Total system mass: {sum(model.body_mass):.6f} kg")

# Run one step to see what forces are applied
mj.mj_forward(model, data)
print(f"Initial qfrc_passive (gravity+bias forces, first 10): {data.qfrc_passive[:10]}")

cam = mj.MjvCamera()
opt = mj.MjvOption()

# Init GLFW, create window
glfw.init()
window = glfw.create_window(1200, 900, "Flappy Simulation", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# Initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# Install GLFW callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Set camera configuration
cam.azimuth = -225
cam.elevation = -20
cam.distance = 1
cam.lookat = np.array([0.0, 0.0, 0.5])

# Get body/joint IDs
body_list = ["Guard", "Core", "L3", "L7", "L3R", "L7R"]
joint_list = ['J5', 'J6', 'J5R', 'J6R']
bodyID_dic, jntID_dic, posID_dic, jvelID_dic = get_bodyIDs(body_list, model)
jID_dic = get_jntIDs(joint_list, model)

# Load joint angle data
# TODO: figure this out using a flapping controller instead.
flap_freq = 6
Angle_data = pd.read_csv(csv_path, header=None)
J5_m = Angle_data.loc[:, 0]
J6_m = Angle_data.loc[:, 1]
J5v_m = flap_freq/2 * Angle_data.loc[:, 2]
J6v_m = flap_freq/2 * Angle_data.loc[:, 3]
t_m = np.linspace(0, 1.0/flap_freq, num=len(J5_m))

# Initialize Flapping Controller
# Adjust amplitudes, phases, and offsets as needed to match desired flight behavior
# flapping_ctrl = FlappingController(
#     freq=flap_freq,
#     amp_J5=1,      # Amplitude for proximal joint (rad)
#     amp_J6=1,      # Amplitude for distal joint (rad)
#     phase_J5=2.5,    # Phase for proximal joint
#     phase_J6=-0.8,   # Phase lag for distal joint
#     offset_J5=0.5,   # Offset for proximal joint
#     offset_J6=-1   # Offset for distal joint
# )

# Initialize controller
# controller = Controller(model, data)
# mj.set_mjcb_control(controller.Control)
# controller.x_d = 0.0
# controller.y_d = 0.0
# controller.z_d = 1.0
# controller.yaw_d = 0.0
# Simulation parameters
dt = 1e-3
model.opt.timestep = dt
simend = 60
xa = np.zeros(3 * nWagner)

# Data logging
SimTime = []
JointAng = [[], []]
JointAng_ref = [[], []]

n_steps = int(simend / dt)

# Camera settings for inset view
camera_name = 'onboard_camera'
inset_width = 640
inset_height = 480

t_start = time.time()
i = 0
cage_collision_enabled = False

while not glfw.window_should_close(window):
    time_prev = data.time
    
    while (data.time - time_prev < 1.0/60.0):  # 60 FPS rendering
        # Enable cage collision after 0.5 seconds (robot has settled)
        if not cage_collision_enabled and data.time > 0.5:
            try:
                cage_geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, 'cage_collision')
                model.geom_contype[cage_geom_id] = 1
                model.geom_conaffinity[cage_geom_id] = 1
                cage_collision_enabled = True
                print(f"Cage collision enabled at t={data.time:.2f}s")
            except:
                pass
        
        # Aerodynamics
        xd, R_body = original_states(model, data, posID_dic, jvelID_dic)
        
        # Get flapping kinematics from controller
        # J5_, J6_, J5v_d, J6v_d = flapping_ctrl.update(data.time)
        
        if 1:
            J5v_d = np.interp(data.time, t_m, J5v_m, period=1.0 / flap_freq)
            J6v_d = np.interp(data.time, t_m, J6v_m, period=1.0 / flap_freq)
        else:
            J5v_d = 0.0
            J6v_d = 0.0

        xd[5] = J5v_d
        xd[6] = J6v_d
        
        fa, ua, xd = aero(xd, R_body, xa)
        xa = xa + fa * dt
        
        # Apply aero forces
        if "L3" in jvelID_dic and "L7" in jvelID_dic:
            data.qfrc_applied[jvelID_dic["L3"]] = ua[0]
            data.qfrc_applied[jvelID_dic["L7"]] = ua[1]
        if "Core" in bodyID_dic:
            data.xfrc_applied[bodyID_dic["Core"]] = [*ua[2:5], *ua[5:8]]
        
        # DIAGNOSTIC: Print IMU data every 1 second
        if i % 5 == 0:
            # Core IMU: data.sensordata[16:22]
            core_gyro = data.sensordata[16:19]    # Core gyro (rad/s)
            core_accel = data.sensordata[19:22]   # Core accel (m/s^2)
            
            # Guard IMU: data.sensordata[22:28]
            guard_gyro = data.sensordata[22:25]   # Guard gyro (rad/s)
            guard_accel = data.sensordata[25:28]  # Guard accel (m/s^2)
            
            # Core Ground Truth: data.sensordata[28:41]
            core_gt_pos = data.sensordata[28:31]       # Position (m)
            core_gt_quat = data.sensordata[31:35]      # Quaternion (w,x,y,z)
            core_gt_linvel = data.sensordata[35:38]    # Linear velocity (m/s)
            core_gt_angvel = data.sensordata[38:41]    # Angular velocity (rad/s)
            
            # Guard Ground Truth: data.sensordata[41:54]
            guard_gt_pos = data.sensordata[41:44]      # Position (m)
            guard_gt_quat = data.sensordata[44:48]     # Quaternion (w,x,y,z)
            guard_gt_linvel = data.sensordata[48:51]   # Linear velocity (m/s)
            guard_gt_angvel = data.sensordata[51:54]   # Angular velocity (rad/s)
            
            print(f"\n{'='*80}")
            print(f"Time: {data.time:.2f}s")
            print(f"{'='*80}")
            
            print(f"\n--- CORE BODY ---")
            print(f"  IMU     | Gyro: [{core_gyro[0]:+.4f}, {core_gyro[1]:+.4f}, {core_gyro[2]:+.4f}] rad/s")
            print(f"          | Accel: [{core_accel[0]:+.4f}, {core_accel[1]:+.4f}, {core_accel[2]:+.4f}] m/s²")
            print(f"  GT      | Pos: [{core_gt_pos[0]:+.4f}, {core_gt_pos[1]:+.4f}, {core_gt_pos[2]:+.4f}] m")
            print(f"          | Quat: [{core_gt_quat[0]:+.4f}, {core_gt_quat[1]:+.4f}, {core_gt_quat[2]:+.4f}, {core_gt_quat[3]:+.4f}]")
            print(f"          | LinVel: [{core_gt_linvel[0]:+.4f}, {core_gt_linvel[1]:+.4f}, {core_gt_linvel[2]:+.4f}] m/s")
            print(f"          | AngVel: [{core_gt_angvel[0]:+.4f}, {core_gt_angvel[1]:+.4f}, {core_gt_angvel[2]:+.4f}] rad/s")
            
            print(f"\n--- GUARD BODY ---")
            print(f"  IMU     | Gyro: [{guard_gyro[0]:+.4f}, {guard_gyro[1]:+.4f}, {guard_gyro[2]:+.4f}] rad/s")
            print(f"          | Accel: [{guard_accel[0]:+.4f}, {guard_accel[1]:+.4f}, {guard_accel[2]:+.4f}] m/s²")
            print(f"  GT      | Pos: [{guard_gt_pos[0]:+.4f}, {guard_gt_pos[1]:+.4f}, {guard_gt_pos[2]:+.4f}] m")
            print(f"          | Quat: [{guard_gt_quat[0]:+.4f}, {guard_gt_quat[1]:+.4f}, {guard_gt_quat[2]:+.4f}, {guard_gt_quat[3]:+.4f}]")
            print(f"          | LinVel: [{guard_gt_linvel[0]:+.4f}, {guard_gt_linvel[1]:+.4f}, {guard_gt_linvel[2]:+.4f}] m/s")
            print(f"          | AngVel: [{guard_gt_angvel[0]:+.4f}, {guard_gt_angvel[1]:+.4f}, {guard_gt_angvel[2]:+.4f}] rad/s")
        
        # Wing flapping
        # Using values computed from flapping_ctrl above
        if 1:
            J5_ = np.interp(data.time, t_m, J5_m, period=1.0/flap_freq)
            J6_ = np.interp(data.time, t_m, J6_m, period=1.0/flap_freq)
        else:
            J5_ = J5_m[0]
            J6_ = J6_m[0]
            
        J5_d = J5_ - np.deg2rad(11.345825599281223)
        J6_d = J6_ + np.deg2rad(27.45260202) - J5_
        
        try:
            data.actuator("J5_angle").ctrl[0] = J5_d
            data.actuator("J6_angle").ctrl[0] = J6_d
        except:
            pass
        
        # Simulation step
        mj.mj_step(model, data)
        i += 1
    
    if (data.time >= simend):
        break
    
    # Get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    
    # Update main scene and render
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    
    # ******** INSET VIEW (onboard camera) *********
    loc_x = int(viewport_width - inset_width)
    loc_y = int(viewport_height - inset_height)
    offscreen_viewport = mj.MjrRect(loc_x, loc_y, inset_width, inset_height)
    
    # Set camera to onboard_camera
    try:
        camera_id = model.camera(camera_name).id
        offscreen_cam = mj.MjvCamera()
        offscreen_cam.type = mj.mjtCamera.mjCAMERA_FIXED
        offscreen_cam.fixedcamid = camera_id
        
        # Update scene for offscreen camera
        mj.mjv_updateScene(model, data, opt, None, offscreen_cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        
        # Render the inset view
        mj.mjr_render(offscreen_viewport, scene, context)
        
        # Optional: Read pixels and show in OpenCV window
        rgb_pixels = np.zeros((inset_height, inset_width, 3), dtype=np.uint8)
        depth_pixels = np.zeros((inset_height, inset_width), dtype=np.float32)
        mj.mjr_readPixels(rgb_pixels, depth_pixels, offscreen_viewport, context)
        
        # Flip vertically for OpenCV
        rgb_flipped = cv2.flip(rgb_pixels, 0)
        rgb_bgr = cv2.cvtColor(rgb_flipped, cv2.COLOR_RGB2BGR)
        
        cv2.imshow("Onboard Camera", rgb_bgr)
        cv2.waitKey(1)
    except:
        print(f"Warning: Camera '{camera_name}' not found")
    
    # Swap buffers
    glfw.swap_buffers(window)
    glfw.poll_events()

# Cleanup
glfw.terminate()
cv2.destroyAllWindows()

total_time = time.time() - t_start
print(f"--- Total Time: {total_time:.2f} seconds ---")
print(f"--- Total realtime factor: {simend/total_time:.2f}x ---")
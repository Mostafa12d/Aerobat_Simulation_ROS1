import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import cv2
import time
import pandas as pd
import os
import rospkg
from datetime import datetime
import matplotlib.pyplot as plt

from aero_force_v2 import aero, nWagner
from Controller_PID_v1 import *
from flapping_controller import FlappingController
from utility_functions.rotation_transformations import *

# Global variables for control
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
flapping_enabled = False
save_image_request = False
control_speed = 0.1  # Step size for position control
yaw_speed = np.deg2rad(5) # Step size for yaw control (5 degrees)

# Create directory for SLAM data
slam_data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "slam_data")
if not os.path.exists(slam_data_dir):
    os.makedirs(slam_data_dir)
    print(f"Created directory for SLAM data: {slam_data_dir}")

def keyboard(window, key, scancode, act, mods):
    global flapping_enabled, save_image_request, control_speed, controller
    
    if act == glfw.PRESS or act == glfw.REPEAT:
        # Reset simulation
        if key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(model, data)
            mj.mj_forward(model, data)
            controller.x_d = 0
            controller.y_d = 0
            controller.z_d = 1.0
            controller.yaw_d = 0
            print("Simulation Reset")

        # Toggle Flapping
        elif key == glfw.KEY_F and act == glfw.PRESS:
            flapping_enabled = not flapping_enabled
            print(f"Flapping enabled: {flapping_enabled}")
            
        # Capture Image
        elif key == glfw.KEY_C and act == glfw.PRESS:
            save_image_request = True
            print("Image capture requested")

        # Control Speed Adjustment
        elif key == glfw.KEY_EQUAL: # '+' key
            control_speed *= 1.2
            print(f"Control speed increased to: {control_speed:.3f} m/step")
        elif key == glfw.KEY_MINUS: # '-' key
            control_speed /= 1.2
            print(f"Control speed decreased to: {control_speed:.3f} m/step")

        # Robot Control
        # X/Y Movement (Arrow Keys)
        elif key == glfw.KEY_UP:
            controller.x_d += control_speed
            print(f"Target X: {controller.x_d:.2f}")
        elif key == glfw.KEY_DOWN:
            controller.x_d -= control_speed
            print(f"Target X: {controller.x_d:.2f}")
        elif key == glfw.KEY_LEFT:
            controller.y_d += control_speed
            print(f"Target Y: {controller.y_d:.2f}")
        elif key == glfw.KEY_RIGHT:
            controller.y_d -= control_speed
            print(f"Target Y: {controller.y_d:.2f}")
            
        # Z Movement (W/S)
        elif key == glfw.KEY_W:
            controller.z_d += control_speed
            print(f"Target Z: {controller.z_d:.2f}")
        elif key == glfw.KEY_S:
            controller.z_d -= control_speed
            print(f"Target Z: {controller.z_d:.2f}")
            
        # Yaw Rotation (A/D)
        elif key == glfw.KEY_A:
            controller.yaw_d += yaw_speed
            print(f"Target Yaw: {np.rad2deg(controller.yaw_d):.1f} deg")
        elif key == glfw.KEY_D:
            controller.yaw_d -= yaw_speed
            print(f"Target Yaw: {np.rad2deg(controller.yaw_d):.1f} deg")

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
xml_path = os.path.join(package_path, 'worlds', 'Cage_scene.xml')

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# DIAGNOSTIC: Check for issues
print(f"Number of bodies: {model.nbody}")
print(f"Number of geoms: {model.ngeom}")
print(f"Number of DOFs: {model.nv}")
print(f"Total system mass: {sum(model.body_mass):.6f} kg")

cam = mj.MjvCamera()
opt = mj.MjvOption()

# Init GLFW, create window
glfw.init()
window = glfw.create_window(1200, 900, "Flappy Teleop Simulation", None, None)
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
flap_freq = 5
Angle_data = pd.read_csv(csv_path, header=None)
J5_m = Angle_data.loc[:, 0]
J6_m = Angle_data.loc[:, 1]
J5v_m = flap_freq/2 * Angle_data.loc[:, 2]
J6v_m = flap_freq/2 * Angle_data.loc[:, 3]
t_m = np.linspace(0, 1.0/flap_freq, num=len(J5_m))

# Initialize controller
controller = Controller(model, data)
mj.set_mjcb_control(controller.Control)
controller.x_d = 0.0
controller.y_d = 0.0
controller.z_d = 1.0
controller.yaw_d = 0.0

# Simulation parameters
dt = 1e-3
model.opt.timestep = dt
simend = 100 # Longer simulation time for teleop
xa = np.zeros(3 * nWagner)

# Camera settings for inset view
camera_name = 'onboard_camera'
inset_width = 640
inset_height = 480
last_camera_time = 0
camera_rate = 25.0 # Hz - How often to capture/process images

i = 0
cage_collision_enabled = False

print("\n" + "="*60)
print("           KEYBOARD TELEOP INSTRUCTIONS")
print("="*60)
print("  [Arrow Keys] : Move X/Y (Forward/Back/Left/Right)")
print("  [W] / [S]    : Move Z (Up/Down)")
print("  [A] / [D]    : Rotate Yaw (Left/Right)")
print("  [+] / [-]    : Increase/Decrease Control Speed")
print("  [F]          : Toggle Flapping")
print("  [C]          : Capture Image (saved to slam_data/)")
print("  [Backspace]  : Reset Simulation")
print("="*60 + "\n")

# Data logging
aero_forces_log = []
time_log = []

t_start = time.time() # Start timer exactly when loop begins

while not glfw.window_should_close(window):
    time_prev = data.time
    
    while (data.time - time_prev < 1.0/30.0):  # 30 FPS rendering for better real-time performance
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
        
        if flapping_enabled:
            J5v_d = np.interp(data.time, t_m, J5v_m, period=1.0 / flap_freq)
            J6v_d = np.interp(data.time, t_m, J6v_m, period=1.0 / flap_freq)
        else:
            J5v_d = 0.0
            J6v_d = 0.0

        xd[5] = J5v_d
        xd[6] = J6v_d
        
        fa, ua, xd = aero(xd, R_body, xa)
        xa = xa + fa * dt
        
        # Log forces
        aero_forces_log.append(ua[2:5])
        time_log.append(data.time)

        # Apply aero forces
        if "L3" in jvelID_dic and "L7" in jvelID_dic:
            data.qfrc_applied[jvelID_dic["L3"]] = ua[0]
            data.qfrc_applied[jvelID_dic["L7"]] = ua[1]
        if "Core" in bodyID_dic:
            data.xfrc_applied[bodyID_dic["Core"]] = [*ua[2:5], *ua[5:8]]
        
        # Wing flapping
        if flapping_enabled:
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
    
    # ******** ONBOARD CAMERA (Hidden from main window) *********
    # Only process camera frames at a specific rate (e.g., 10Hz) to save performance
    if data.time - last_camera_time >= 1.0/camera_rate:
        last_camera_time = data.time
        
        # We render to the bottom-left corner (0,0), but it will be overwritten later
        offscreen_viewport = mj.MjrRect(0, 0, inset_width, inset_height)
        
        # Set camera to onboard_camera
        try:
            camera_id = model.camera(camera_name).id
            offscreen_cam = mj.MjvCamera()
            offscreen_cam.type = mj.mjtCamera.mjCAMERA_FIXED
            offscreen_cam.fixedcamid = camera_id
            
            # Update scene for offscreen camera
            mj.mjv_updateScene(model, data, opt, None, offscreen_cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
            
            # Render the camera view
            mj.mjr_render(offscreen_viewport, scene, context)
            
            # Read pixels for display and saving
            rgb_pixels = np.zeros((inset_height, inset_width, 3), dtype=np.uint8)
            mj.mjr_readPixels(rgb_pixels, None, offscreen_viewport, context)
            
            # Flip vertically for OpenCV
            rgb_flipped = cv2.flip(rgb_pixels, 0)
            rgb_bgr = cv2.cvtColor(rgb_flipped, cv2.COLOR_RGB2BGR)
            
            # Save image if requested
            if save_image_request:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = os.path.join(slam_data_dir, f"img_{timestamp}.png")
                cv2.imwrite(filename, rgb_bgr)
                print(f"Saved image: {filename}")
                save_image_request = False
            
            cv2.imshow("Onboard Camera", rgb_bgr)
            cv2.waitKey(1)
        except:
            pass

    # Update main scene and render (This overwrites the camera view in the buffer)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    
    # Swap buffers
    glfw.swap_buffers(window)
    glfw.poll_events()

# Cleanup
glfw.terminate()
cv2.destroyAllWindows()

total_time = time.time() - t_start
print(f"--- Total Time: {total_time:.2f} seconds ---")
print(f"--- Total realtime factor: {simend/total_time:.2f}x ---")

# Plotting
if len(time_log) > 0:
    aero_forces_log = np.array(aero_forces_log)
    time_log = np.array(time_log)

    plt.figure(figsize=(12, 8))
    
    # Plot X, Y, Z components of Aerodynamic Force
    plt.subplot(2, 1, 1)
    plt.plot(time_log, aero_forces_log[:, 0], label='Force X (Drag/Thrust)')
    plt.plot(time_log, aero_forces_log[:, 1], label='Force Y (Side)')
    plt.plot(time_log, aero_forces_log[:, 2], label='Force Z (Lift)')
    plt.title('Aerodynamic Forces on Body')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.legend()
    plt.grid(True)

    # Calculate total force magnitude
    total_aero_force = np.linalg.norm(aero_forces_log, axis=1)

    # Plot Total Aerodynamic Force Magnitude
    plt.subplot(2, 1, 2)
    plt.plot(time_log, total_aero_force, label='Total Aero Force Magnitude', color='black')
    plt.title('Total Aerodynamic Force Magnitude')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
else:
    print("No data logged for plotting.")

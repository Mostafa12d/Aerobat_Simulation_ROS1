import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import cv2
import time
import pandas as pd
import os
import rospkg
import threading

from aero_force_v2 import aero, nWagner
from Controller_PID_v1 import *
from utility_functions.rotation_transformations import *

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

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

#### Controller helper functions ####
# Global control variables
control_commands = {
    'mode': 'pid',  # 'pid' or 'manual'
    'x_setpoint': 0.0,
    'y_setpoint': 0.0,
    'z_setpoint': 0.5,
    'yaw_setpoint': 0.0,
    'motor1': 0.0,
    'motor2': 0.0,
    'motor3': 0.0,
    'motor4': 0.0,
    'motor5': 0.0,
    'motor6': 0.0
}

accel_bias = np.random.normal(0, 0.05, 3)  # Random bias for x, y, z
gyro_bias = np.random.normal(0, 0.002, 3)  # Random bias for roll, pitch, yaw

def terminal_input_thread():
    """Handle terminal input in a separate thread"""
    global control_commands
    
    print("\n=== FLAPPY CONTROL INTERFACE ===")
    print("Commands:")
    print("  mode <pid|manual>     - Switch control mode")
    print("  pid <x> <y> <z> <yaw> - Set PID setpoints (x,y,z in meters, yaw in degrees)")
    print("  motor <1-6> <value>   - Set individual motor (0.0-1.0) in manual mode")
    print("  motors <m1> <m2> <m3> <m4> <m5> <m6> - Set all motors at once")
    print("  status                - Show current status")
    print("  help                  - Show this help")
    print("  quit                  - Exit simulation")
    print("\nExamples:")
    print("  mode pid")
    print("  pid 1.0 0.5 1.0 45")
    print("  mode manual")
    print("  motor 1 0.3")
    print("  motors 0.3 0.3 0.3 0.3 0.1 0.1")
    print("\nEnter commands:")
    
    while True:
        try:
            cmd = input("> ").strip().lower().split()
            if not cmd:
                continue
                
            if cmd[0] == 'quit' or cmd[0] == 'exit':
                print("Exiting simulation...")
                break
                
            elif cmd[0] == 'mode':
                if len(cmd) > 1 and cmd[1] in ['pid', 'manual']:
                    control_commands['mode'] = cmd[1]
                    print(f"Mode set to: {cmd[1]}")
                else:
                    print("Usage: mode <pid|manual>")
                    
            elif cmd[0] == 'pid':
                if len(cmd) == 5:
                    try:
                        control_commands['x_setpoint'] = float(cmd[1])
                        control_commands['y_setpoint'] = float(cmd[2])
                        control_commands['z_setpoint'] = float(cmd[3])
                        control_commands['yaw_setpoint'] = np.deg2rad(float(cmd[4]))
                        print(f"PID setpoints: x={cmd[1]}, y={cmd[2]}, z={cmd[3]}, yaw={cmd[4]}°")
                    except ValueError:
                        print("Invalid numbers. Usage: pid <x> <y> <z> <yaw_deg>")
                else:
                    print("Usage: pid <x> <y> <z> <yaw_deg>")
                    
            elif cmd[0] == 'motor':
                if len(cmd) == 3:
                    try:
                        motor_num = int(cmd[1])
                        motor_val = float(cmd[2])
                        if 1 <= motor_num <= 6 and 0.0 <= motor_val <= 1.0:
                            control_commands[f'motor{motor_num}'] = motor_val
                            print(f"Motor {motor_num} set to: {motor_val}")
                        else:
                            print("Motor number must be 1-6, value must be 0.0-1.0")
                    except ValueError:
                        print("Invalid numbers. Usage: motor <1-6> <0.0-1.0>")
                else:
                    print("Usage: motor <1-6> <0.0-1.0>")
                    
            elif cmd[0] == 'motors':
                if len(cmd) == 7:
                    try:
                        values = [float(x) for x in cmd[1:]]
                        if all(0.0 <= v <= 1.0 for v in values):
                            for i, val in enumerate(values, 1):
                                control_commands[f'motor{i}'] = val
                            print(f"All motors set: {values}")
                        else:
                            print("All values must be between 0.0 and 1.0")
                    except ValueError:
                        print("Invalid numbers. Usage: motors <m1> <m2> <m3> <m4> <m5> <m6>")
                else:
                    print("Usage: motors <m1> <m2> <m3> <m4> <m5> <m6>")
                    
            elif cmd[0] == 'status':
                print(f"\nCurrent Status:")
                print(f"  Mode: {control_commands['mode']}")
                if control_commands['mode'] == 'pid':
                    print(f"  PID Setpoints: x={control_commands['x_setpoint']:.2f}, y={control_commands['y_setpoint']:.2f}, z={control_commands['z_setpoint']:.2f}, yaw={np.rad2deg(control_commands['yaw_setpoint']):.1f}°")
                else:
                    print(f"  Motor Values: {[control_commands[f'motor{i}'] for i in range(1,7)]}")
                    
            elif cmd[0] == 'help':
                print("\nCommands:")
                print("  mode <pid|manual>     - Switch control mode")
                print("  pid <x> <y> <z> <yaw> - Set PID setpoints")
                print("  motor <1-6> <value>   - Set individual motor")
                print("  motors <m1> <m2> <m3> <m4> <m5> <m6> - Set all motors")
                print("  status                - Show current status")
                print("  quit                  - Exit simulation")
                
            else:
                print(f"Unknown command: {cmd[0]}. Type 'help' for commands.")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

def apply_flight_control(data, controller):
    """Apply control commands from terminal input"""
    global control_commands
    
    if control_commands['mode'] == 'manual':
        # Manual mode - apply motor commands directly
        data.actuator("Motor1").ctrl[0] = control_commands['motor1']
        data.actuator("Motor2").ctrl[0] = control_commands['motor2']
        data.actuator("Motor3").ctrl[0] = control_commands['motor3']
        data.actuator("Motor4").ctrl[0] = control_commands['motor4']
        data.actuator("Motor5").ctrl[0] = control_commands['motor5']
        data.actuator("Motor6").ctrl[0] = control_commands['motor6']
    else:
        # PID mode - update controller setpoints
        controller.x_d = control_commands['x_setpoint']
        controller.y_d = control_commands['y_setpoint']
        controller.z_d = control_commands['z_setpoint']
        controller.yaw_d = control_commands['yaw_setpoint']
        # PID controller handles motor commands automatically



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
flap_freq = 6
Angle_data = pd.read_csv(csv_path, header=None)
J5_m = Angle_data.loc[:, 0]
J6_m = Angle_data.loc[:, 1]
J5v_m = flap_freq/2 * Angle_data.loc[:, 2]
J6v_m = flap_freq/2 * Angle_data.loc[:, 3]
t_m = np.linspace(0, 1.0/flap_freq, num=len(J5_m))

# Initialize controller
controller = Controller(model, data)
mj.set_mjcb_control(controller.Control)

# Simulation parameters
dt = 1e-3
model.opt.timestep = dt
simend = 60
xa = np.zeros(3 * nWagner)

# Start terminal input thread
input_thread = threading.Thread(target=terminal_input_thread, daemon=True)
input_thread.start()

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
        
        J5v_d = np.interp(data.time, t_m, J5v_m, period=1.0 / flap_freq)
        J6v_d = np.interp(data.time, t_m, J6v_m, period=1.0 / flap_freq)
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
        
        # Wing flapping
        J5_ = np.interp(data.time, t_m, J5_m, period=1.0/flap_freq)
        J6_ = np.interp(data.time, t_m, J6_m, period=1.0/flap_freq)
        J5_d = J5_ - np.deg2rad(11.345825599281223)
        J6_d = J6_ + np.deg2rad(27.45260202) - J5_
        
        try:
            data.actuator("J5_angle").ctrl[0] = J5_d
            data.actuator("J6_angle").ctrl[0] = J6_d
        except:
            pass
        
        # Apply flight control
        apply_flight_control(data, controller)
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
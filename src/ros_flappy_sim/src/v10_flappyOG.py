import mujoco as mj
import mujoco_viewer
import rospy
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import tf 

import time
import cv2

import numpy as np
import matplotlib.pyplot as plt

from aero_force_v2 import aero, nWagner
from Controller_PID_v1 import *
from utility_functions.rotation_transformations import *
import pandas as pd
import os
import rospkg



''' 
 This version includes uses one period of recorded joint angle generate Flapping Wing Motion.
 => This script publishes camera and IMU data to ROS topics
 => The IMU data includes position, orientation (quaternion), linear velocity, and angular velocity
 => Require JointAngleData.csv to get correct wing joint angles
 => Require v8_JointInput.xml (open-loop kinematic chain)
 => Allow big dt !!! (currently dt =  5e-4 and can be much larger without issue of instability)
 NOTE: this version of xml uses two joint motor to control the two joint angle (Shoulder and Elbow)
 Don't use data.ctrl[0] and data.ctrl[0] in control! (would not make sense to do so)
 Note: instead of use "data.ctrl[2:8]", we can use "data.actuator("Motor1/2/3/4/5/6").ctrl[0]" 
 Recommend to use mujoco build-in aerodynamics for now; 
 The aero_force_v2 function is runnable but seem to be very sensitive to joint angle velocity 
 Use pre-recorded joint velocity for aero function computation => ua Consistent with MATLAB now
 May use aero_function for aero-dynamics now
'''


def publish_imu(pub, data):
    imu_msg = Imu()
    # Orientation (quaternion)
    imu_msg.orientation.x = data.sensordata[4]
    imu_msg.orientation.y = data.sensordata[5]
    imu_msg.orientation.z = data.sensordata[6]
    imu_msg.orientation.w = data.sensordata[3]
    # Angular velocity (body frame)
    imu_msg.angular_velocity.x = data.sensordata[10]
    imu_msg.angular_velocity.y = data.sensordata[11]
    imu_msg.angular_velocity.z = data.sensordata[12]
    # Linear acceleration (approximate with finite difference or use MuJoCo sensor if available)
    # Here, we use linear velocity difference as a placeholder (not accurate!)
    imu_msg.linear_acceleration.x = 0.0
    imu_msg.linear_acceleration.y = 0.0
    imu_msg.linear_acceleration.z = 0.0
    pub.publish(imu_msg)
    # print("IMU data published")

def publish_camera(pub, bridge, viewer):
    img = viewer.read_pixels()
    img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    ros_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(ros_img)
    # print("Camera image published")

def publish_tf(tf_broadcaster, data):
    # Get position from MuJoCo sensor data
    position = data.sensordata[0:3]  # x, y, z in inertial/world frame

    # Orientation (quaternion)
    orientation = data.sensordata[3:7]  # [w, x, y, z] format in MuJoCo

    # Convert to [x, y, z, w] as expected by ROS
    qx = orientation[1]
    qy = orientation[2]
    qz = orientation[3]
    qw = orientation[0]

    tf_broadcaster.sendTransform(
        (position[0], position[1], position[2]),  # translation
        (qx, qy, qz, qw),                         # rotation quaternion
        rospy.Time.now(),
        "base_link",   # child frame (robot base)
        "odom"         # parent frame (world)
    )

    # print("TF transform published")


def get_bodyIDs(body_list):
    bodyID_dic = {}
    jntID_dic = {}
    posID_dic = {}
    jvelID_dic = {}
    for bodyName in body_list:
        mjID = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, bodyName)
        jntID = model.body_jntadr[mjID]  # joint ID
        jvelID = model.body_dofadr[mjID]  # joint velocity
        posID = model.jnt_qposadr[jntID]  # joint position
        bodyID_dic[bodyName] = mjID
        jntID_dic[bodyName] = jntID
        posID_dic[bodyName] = posID
        jvelID_dic[bodyName] = jvelID
    return bodyID_dic, jntID_dic, posID_dic, jvelID_dic


def get_jntIDs(jnt_list):
    jointID_dic = {}
    for jointName in jnt_list:
        jointID = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jointName)
        jointID_dic[jointName] = jointID
    return jointID_dic


def original_states(model, data):  # improved original states function (use sensor instead of qpos/qvel)
    # takes mujoco states vectors and converts to MATLAB states vectors defined in func_eom
    xd = np.array([0.0] * 22)
    xd[0] = data.qpos[posID_dic["L3"]] + np.deg2rad(11.345825599281223)  # xd[0:1] left wing angles [shoulder, elbow]
    xd[1] = data.qpos[posID_dic["L7"]] - np.deg2rad(27.45260202) + xd[0]
    xd[2:5] = data.sensordata[0:3]  # inertial position (x, y, z)
    xd[5] = data.qvel[jvelID_dic["L3"]]  # Joint 5 velocity
    xd[6] = data.qvel[jvelID_dic["L7"]]  # Joint 6 velocity
    xd[7:10] = data.sensordata[7:10]  # Inertial Frame Linear Velocity
    xd[10:13] = data.sensordata[10:13]  # Body Frame Angular velocity

    if np.linalg.norm(data.sensordata[3:7]) == 0:
        data.sensordata[3:7] = [1,0,0,0]
    R_body = quat2rot(data.sensordata[3:7])
    xd[13:23] = list(np.transpose(R_body).flatten())
    return xd, R_body


""" ------------------------SIMULATION--------------------------------------- """
# Create ROS node and publishers
rospy.init_node('flappy_sim_publisher')
# cam_pub = rospy.Publisher('/flappy/camera/image_raw', Image, queue_size=10)
imu_pub = rospy.Publisher('/flappy/imu', Imu, queue_size=10)
tf_broadcaster = tf.TransformBroadcaster()
bridge = CvBridge()


rospack = rospkg.RosPack()
package_path = rospack.get_path('ros_flappy_sim')
csv_path = os.path.join(package_path, 'src', 'JointAngleData.csv')
xml_path = os.path.join(package_path, 'worlds', 'Flappy_v10_weld.xml')

# xml_path = '/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/src/ros_flappy_sim/worlds/Flappy_v9_RB.xml'
# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)  # MuJoCo data

# Create the viewer object
record_enable = True  # <============== Set to True to enable video recording
video_show_enable = False  # <============== Set to True to show video when recording
width = 1200  # None
height = 800 # None (default)
if record_enable:
    viewer = mujoco_viewer.MujocoViewer(model, data, 'offscreen', width=width, height=height, hide_menus=True)  # To record video
else:
    viewer = mujoco_viewer.MujocoViewer(model, data, width=width, height=height, hide_menus=True)

viewer._render_every_frame = False  # To run faster (same as hitting key "D")
viewer._image_path = "frame_%07d.png"  # Enable screenshot

# Camera Setup
viewer.cam.azimuth = -225
viewer.cam.elevation = -20
viewer.cam.distance = 1
viewer.cam.lookat = np.array([0, 0.0, 0.5])

# Video Recording
# if record_enable:
#     VideoName = "Flappy_video_JointInput.mp4"
#     fps = 60
#     # Define the codec and create VideoWriter object
#     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#     out = cv2.VideoWriter(VideoName, fourcc, fps, (width,height))
#     n_frame = 1  # initialize number of frame
#     if video_show_enable:
#         cv2.namedWindow("video", cv2.WINDOW_NORMAL)

# -------------------------------------------- #

# Get id of bodies and joints
body_list = ["Base", "L1", "L2", "L3", "L4", "L5", "L6", "L7",
             "L1R", "L2R", "L3R", "L4R", "L5R", "L6R", "L7R"]
joint_list = ['J1', 'J2', 'J3', 'J5', 'J6', 'J7', 'J10',
              'J1R', 'J2R', 'J3R', 'J5R', 'J6R', 'J7R', 'J10R']
bodyID_dic, jntID_dic, posID_dic, jvelID_dic = get_bodyIDs(body_list)
jID_dic = get_jntIDs(joint_list)

flap_freq = 6 # Hz <===== Change here to change flapping frequency
# Load pre-recorded joint angle from MATLAB (1 period; 1 second)
Angle_data = pd.read_csv(csv_path, header=None)
J5_m = Angle_data.loc[:, 0]  # Mujoco reference joint angle (θ_5)
J6_m = Angle_data.loc[:, 1]  # Mujoco reference joint angle (θ_6)
J5v_m = flap_freq/2 * Angle_data.loc[:, 2]  # Mujoco reference joint angle velocity (θ_5_dot)
J6v_m = flap_freq/2 * Angle_data.loc[:, 3]  # Mujoco reference joint angle velocity (θ_6_dot)
t_m = np.linspace(0, 1.0/flap_freq, num=len(J5_m))



# initialize the controller
controller = Controller(model,data)
# set the controller
mj.set_mjcb_control(controller.Control)

# simulation parameters
dt = 1e-3
model.opt.timestep = dt
simend = 30

# Initialize aerodynamic states
xa = np.zeros(3 * nWagner)  # xa_0

# Initialize data logging
SimTime = []
ua_ = []
JointAng = [[],[]]
JointAng_ref = [[],[]]
JointVel = [[],[]]
JointVel_ref = [[],[]]

n_steps = int(simend / dt)
rate = rospy.Rate(int(1/dt))
# To Record Computation Time

comp_time = np.zeros((3, n_steps))
factor = 1
publish_interval = 10 
camera_rate = 25 # Hz
camera_interval = int((1/camera_rate) / dt)  # publish camera every 40 steps
imu_rate = 400 # Hz
imu_interval = int((1/imu_rate) / dt)  # publish IMU every 2.5 steps
try:
    t_start = time.time()
    for i in range(n_steps):
        start_time_loop = time.time()

        if i%factor == 0:
            start_time_aero = time.time()
            ########## External Forces ##########
            # Get aero states and force
            xd, R_body = original_states(model, data)

            # Use pre-recorded joint velocity for aero_function => consistent to MATLAB result
            J5v_d = np.interp(data.time, t_m, J5v_m, period=1.0 / flap_freq)
            J6v_d = np.interp(data.time, t_m, J6v_m, period=1.0 / flap_freq)
            xd[5] = J5v_d  # Joint 5 velocity
            xd[6] = J6v_d  # Joint 6 velocity  (Only this one would also work)

            fa, ua, xd = aero(xd, R_body, xa)

            # # publish ros topics
            # if not rospy.is_shutdown() and (i % publish_interval == 0):
            #     publish_camera(cam_pub, bridge, viewer)
            # if not rospy.is_shutdown():
            #     publish_imu(imu_pub, data)
            #     publish_tf(tf_broadcaster, data)
            # if not rospy.is_shutdown() and (i % camera_interval == 0):
            #     publish_camera(cam_pub, bridge, viewer)
            
            if not rospy.is_shutdown() and (i % imu_interval == 0):
                publish_imu(imu_pub, data)
                publish_tf(tf_broadcaster, data)

            # Integrate Aero States
            xa = xa + fa*factor*dt  # 1 step

            comp_time[0, i] = time.time() - start_time_aero
            # print("--- Aero Time: %s seconds ---" % (time.time() - start_time_aero))
            ########## ^^^^^^^^^^^^^^^ ##########

        # Apply Aero forces
        data.qfrc_applied[jvelID_dic["L3"]] = ua[0]
        data.qfrc_applied[jvelID_dic["L7"]] = ua[1]
        data.xfrc_applied[bodyID_dic["Base"]] = [*ua[2:5], *ua[5:8]]
        #  Record Aero data
        ua_.append(ua)

        ## Move Forward ##

        # Apply wing joint angles
        J5_ = np.interp(data.time, t_m, J5_m, period=1.0/flap_freq)
        J6_ = np.interp(data.time, t_m, J6_m, period=1.0/flap_freq)
        J5_d = J5_ - np.deg2rad(11.345825599281223)  # convert to Mujoco Reference
        J6_d = J6_ + np.deg2rad(27.45260202) - J5_
        # Apply angles to Joints
        data.actuator("J5_angle").ctrl[0] = J5_d
        data.actuator("J6_angle").ctrl[0] = J6_d

        # Integrate Kinematics & Dynamics in MuJoCo
        mj.mj_step(model, data)

        # Record joint angles
        J5 = data.qpos[posID_dic["L3"]] + np.deg2rad(11.345825599281223)  # Get angle from mujoco
        J6 = data.qpos[posID_dic["L7"]] - np.deg2rad(27.45260202) + J5    #
        SimTime.append(data.time)
        JointAng_ref[0].append(J5_)
        JointAng_ref[1].append(J6_)
        J5v_d = np.interp(data.time, t_m, J5v_m, period=1.0 / flap_freq) # Get velocity just for comparison
        J6v_d = np.interp(data.time, t_m, J6v_m, period=1.0 / flap_freq)
        JointVel_ref[0].append(J5v_d)
        JointVel_ref[1].append(J6v_d)
        JointAng[0].append(J5)
        JointAng[1].append(J6)
        JointVel[0].append(data.qvel[jvelID_dic["L3"]])
        JointVel[1].append(data.qvel[jvelID_dic["L7"]])


        ## Record Video ##
        # if record_enable:
        #     if n_frame < data.time * fps:
        #         start_time_record = time.time()
        #         img = viewer.read_pixels()
        #         img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        #         if video_show_enable:
        #             cv2.imshow('video', img)
        #             # Press Q on keyboard to  exit
        #             if cv2.waitKey(25) & 0xFF == ord('q'):
        #                 break

        #         out.write(img)
        #         n_frame += 1
        #         comp_time[1, i] = (time.time() - start_time_record)
        #         print("--- Record Time: %s seconds ---" % (time.time() - start_time_record))

        ## Rendering ##
        if not record_enable and (i % camera_interval == 0):
            start_time_render = time.time()
            viewer.render()
            comp_time[1, i] = (time.time() - start_time_render)

        comp_time[2, i] = (time.time() - start_time_loop)

        # Add a small delay to prevent overwhelming the system
        if i % 100 == 0:  # Every 100 steps (0.1 seconds)
            time.sleep(0.001)  # 1ms delay
        # if not record_enable:
        #     start_time_render = time.time()
        #     viewer.render()
        #     comp_time[1, i] = (time.time() - start_time_render)
        #     # print("--- Render Time: %s seconds ---" % (time.time() - start_time_render))

        # comp_time[2, i] = (time.time() - start_time_loop)
        # print("--- Loop Time: %s seconds ---\n" % (time.time() - start_time_loop))

        # Remove real-time synchronization to run as fast as possible
        # elapsed = time.time() - start_time_loop
        # sleep_time = dt - elapsed
        # if sleep_time > 0:
        #     time.sleep(sleep_time)

        if not viewer.is_alive or rospy.is_shutdown():
            break
except KeyboardInterrupt:
    print("Simulation interrupted by user.")
# close
finally:
    viewer.close()
    # if record_enable:
    #     out.release()  # done recording
    if video_show_enable or record_enable:
        cv2.destroyWindow("video")
    print("Simulation ended and resources released.")

total_time = time.time() - t_start
print("--- Total Time: %s seconds ---" % total_time)
print("Avg Aero Comp Time:", np.mean(comp_time[0, :]))
print("Avg Record Comp Time:" if record_enable else "Avg Viewer Comp Time:", np.mean(comp_time[1, :]))
print("Avg Loop Comp Time:", np.mean(comp_time[2, :]))
print("--- Total realtime factor: %f x ---" % (simend/total_time))

# Plot Results

plt.figure()
plt.subplot(2,1,1)
plt.title("Joint Angle (MuJoCo vs. Matlab)")
plt.plot(SimTime, np.rad2deg(JointAng_ref[0]), 'b--', label='J5 ref')
plt.plot(SimTime, np.rad2deg(JointAng[0]), 'b', label='J5 real')
plt.xlabel('Time, t (s)')
plt.ylabel('J5 Angle, θ_5 (deg)')
plt.legend()

plt.subplot(2,1,2)
plt.plot(SimTime, np.rad2deg(JointAng_ref[1]), 'g--', label='J6 ref')
plt.plot(SimTime, np.rad2deg(JointAng[1]), 'g', label='J6 real')
plt.xlabel('Time, t (s)')
plt.ylabel('J6 Angle, θ_6 (deg)')
plt.legend()

plt.figure()
plt.subplot(2,1,1)
plt.title("Joint Angle Velocity (MuJoCo vs. Matlab)")
plt.plot(SimTime, np.rad2deg(JointVel_ref[0]), 'b--', label='J5 ref')
plt.plot(SimTime, np.rad2deg(JointVel[0]), 'b', label='J5 real')
plt.xlabel('Time, t (s)')
plt.ylabel('J5 Speed, θ_5 (deg/s)')
# plt.ylim([-2000,2000])
plt.legend()

plt.subplot(2,1,2)
plt.plot(SimTime, np.rad2deg(JointVel_ref[1]), 'g--', label='J6 ref')
plt.plot(SimTime, np.rad2deg(JointVel[1]), 'g', label='J6 real')
plt.xlabel('Time, t (s)')
plt.ylabel('J6 Speed, θ_6 (deg/s)')
# plt.ylim([-3000,3000])
plt.legend()

plt.show()

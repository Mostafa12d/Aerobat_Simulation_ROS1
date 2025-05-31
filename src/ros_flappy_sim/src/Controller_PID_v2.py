import numpy as np
from utility_functions.rotation_transformations import *

''' PID Linear Trajectory Tracking control; Track by translation only'''

class Controller:
    def __init__(self,model,data, points, tspans):
        # initialize states
        self.x_d = 0
        self.y_d = 0
        self.z_d = 0.5
        self.vx_d = 0
        self.vy_d = 0
        self.roll_d = 0
        self.pitch_d = 0
        self.yaw_d = 0  # np.pi/18
        self.roll_rate_d = 0
        self.pitch_rate_d = 0
        self.yaw_rate_d = 0

        self.pitch_I = 0
        self.roll_I = 0
        self.yaw_I = 0
        self.vx_I = 0
        self.vy_I = 0
        self.z_I = 0

        self.pitch_err_prv = 0
        self.roll_err_prv = 0
        self.yaw_err_prv = 0
        self.vx_err_prv = 0
        self.vy_err_prv = 0
        self.z_err_prv = 0

        self.time_ctrlPos_prv = data.time
        self.time_ctrlAtt_prv = data.time

        # Position Control
        self.Kp_Xh = 1
        self.Kp_vh = 1

        self.points = points
        self.tspans = tspans

    def Control(self,model,data):
        t = data.time
        pos_d = trajectory_generator(self.points, self.tspans, t)
        self.x_d = pos_d[0]
        self.y_d = pos_d[1]
        self.z_d = pos_d[2]

        # PID parameters
        Ki_vh = 0.1
        Kd_vh = 0.0
        MAX_CONTROL_Velocity_x = 3
        MAX_CONTROL_Velocity_y = 3
        MAX_CONTROL_ANGLE_ROLL = np.deg2rad(30)
        MAX_CONTROL_ANGLE_PITCH = np.deg2rad(30)

        Kp_RP = 5
        Kp_RP_rate = 0.5
        Ki_RP_rate = 0.01
        Kd_RP_rate = 0.0  # 0.001
        Kp_Y = 5
        Kp_Y_rate = 1
        Ki_Y_rate = 0.8
        Kd_Y_rate = 0
        I_clamp_att = 0.5

        Kp_th = 4
        Ki_th = 0.15
        Kd_th = 3

        dt_pos = 1 / 100
        dt_att = 1 / 500

        body_pos_mes = data.sensordata[0:3]  # should be same as pos
        euler_mes = quat2euler_raw(data.sensordata[3:7])  # roll, pitch, yaw
        body_vel_mes = data.sensordata[7:10]  # should be same as qvel
        rot_rate_mes = data.sensordata[10:13]  # p, q, r

        # print(rot_rate_mes)
        # print(euler_mes)

        dt_pos_ctrl = data.time - self.time_ctrlPos_prv
        dt_att_ctrl = data.time - self.time_ctrlAtt_prv

        ############# Position Control #############
        if dt_pos_ctrl >= dt_pos:
            self.time_ctrlPos_prv = data.time

            self.vx_d = clamp(MAX_CONTROL_Velocity_x, self.Kp_Xh * (self.x_d - data.qpos[0]))
            vx_err = (self.vx_d - data.qvel[0])
            self.vx_I += vx_err * dt_pos
            self.vx_I = clamp(I_clamp_att, self.vx_I)
            pitch_d_raw = (self.Kp_vh * vx_err + Ki_vh * self.vx_I + Kd_vh * (vx_err - self.vx_err_prv) / dt_pos)
            self.pitch_d = clamp(MAX_CONTROL_ANGLE_PITCH, pitch_d_raw)  #
            self.vx_err_prv = vx_err

            self.vy_d = clamp(MAX_CONTROL_Velocity_y, self.Kp_Xh * (self.y_d - data.qpos[1]))
            vy_err = (self.vy_d - data.qvel[1])
            self.vy_I += vy_err * dt_pos
            self.vy_I = clamp(I_clamp_att, self.vy_I)
            roll_d_raw = -(self.Kp_vh * vy_err + Ki_vh * self.vy_I + Kd_vh * (vy_err - self.vy_err_prv) / dt_pos)
            self.roll_d = clamp(MAX_CONTROL_ANGLE_ROLL, roll_d_raw)  #
            self.vy_err_prv = vy_err

        # self.pitch_d = 0 # np.deg2rad(10) # np.deg2rad(30)
        # self.roll_d = 0 # np.deg2rad(10)
        ############# Attitude Control #############
        if dt_att_ctrl >= dt_att:
            self.time_ctrlAtt_prv = data.time
            # Roll
            self.roll_rate_d = Kp_RP * (self.roll_d - euler_mes[0])
            roll_rate_err = (self.roll_rate_d - rot_rate_mes[0])
            self.roll_I += roll_rate_err * dt_att
            self.roll_I = clamp(I_clamp_att, self.roll_I)
            Roll_raw = Kp_RP_rate * roll_rate_err + Ki_RP_rate * self.roll_I + Kd_RP_rate * (
                        roll_rate_err - self.roll_err_prv) / dt_att
            Roll_command = clamp(1, Roll_raw)
            self.roll_err_prv = roll_rate_err
            # print(Roll_command)
            # Pitch
            self.pitch_rate_d = Kp_RP * (self.pitch_d - euler_mes[1])
            pitch_rate_err = (self.pitch_rate_d - rot_rate_mes[1])
            self.pitch_I += pitch_rate_err * dt_att
            self.pitch_I = clamp(I_clamp_att, self.pitch_I)
            Pitch_raw = Kp_RP_rate * pitch_rate_err + Ki_RP_rate * self.pitch_I + Kd_RP_rate * (
                        pitch_rate_err - self.pitch_err_prv) / dt_att
            Pitch_command = clamp(1, Pitch_raw)
            self.pitch_err_prv = pitch_rate_err
            # print(Pitch_command)
            # Yaw
            self.yaw_rate_d = Kp_Y * (self.yaw_d - euler_mes[2])
            yaw_rate_err = (self.yaw_rate_d - rot_rate_mes[2])
            self.yaw_I += yaw_rate_err * dt_att
            self.yaw_I = clamp(I_clamp_att, self.yaw_I)
            Yaw_raw = Kp_Y_rate * yaw_rate_err + Ki_Y_rate * self.yaw_I + Kd_Y_rate * (yaw_rate_err - self.yaw_err_prv) / dt_att
            Yaw_command = clamp(1, Yaw_raw)
            self.yaw_err_prv = yaw_rate_err
            # print(Yaw_command)

            # Thrust Control
            z_err = (self.z_d - data.qpos[2])
            self.z_I += z_err * dt_att
            self.z_I = clamp(I_clamp_att, self.z_I)
            Thrust_raw = Kp_th * z_err + Ki_th * self.z_I + Kd_th * (z_err - self.z_err_prv) / dt_att + 0.5  # 0.609
            Thrust_command = clamp(1, Thrust_raw)
            self.z_err_prv = z_err

            ControlInput0 = Thrust_command - Pitch_command  # front
            ControlInput1 = Thrust_command + Pitch_command  # back
            ControlInput2 = Thrust_command - Roll_command  # right
            ControlInput3 = Thrust_command + Roll_command  # left

            Thrust0 = ControlInput0
            Thrust1 = ControlInput1
            Thrust2 = ControlInput2
            Thrust3 = ControlInput3
            Thrust4 = Yaw_command
            Thrust5 = Yaw_command

            # put the controller here. This function is called inside the simulation.
            try:
                data.actuator("velocity_servo").ctrl[
                    0] = -np.pi * 2 * 8  # set to -29.8451302 => Drive Flapping Kinematics # data.ctrl[0]
            except:
                # print("Actuator name: 'velocity_servo' not exist")
                pass
            data.actuator("Motor1").ctrl[0] = Thrust0  # data.ctrl[1] # front
            data.actuator("Motor2").ctrl[0] = Thrust1  # data.ctrl[2] # back
            data.actuator("Motor3").ctrl[0] = Thrust2  # data.ctrl[3] # left
            data.actuator("Motor4").ctrl[0] = Thrust3  # data.ctrl[4] # right
            data.actuator("Motor5").ctrl[0] = Thrust4  # data.ctrl[5] # left H
            data.actuator("Motor6").ctrl[0] = Thrust5  # data.ctrl[6] # right H
            # print(data.ctrl)

def clamp(limit, ref):
    if ref < -limit:
        output = -limit
    elif ref > limit:
        output = limit
    else:
        output = ref
    return output


def trajectory_generator(points, tspans, t):
    trajectory_type = "Linear"
    # trajectory_type = "Spiral"
    if trajectory_type == "Linear":
        # Inputs: points - 3xn list = [(0,0,0),...]; tspans - 1xn list
        # Output: pos_d
        if len(points) != len(tspans) + 1:
            print("length not compatible: len(points) not equal len(tspans)+1")

        for i in range(len(tspans)):
            if t <= sum(tspans[:i + 1]):
                # print(sum(tspans[:i+1]))
                pos_d = [points[i, j] + (points[i + 1, j] - points[i, j]) * (t - sum(tspans[:i])) / tspans[i] for j in
                         range(3)]
                return pos_d
        if t > sum(tspans):
            return points[-1]
    elif trajectory_type == "Spiral":
        x = math.sin(2 * math.pi * t / 10)
        y = math.cos(2 * math.pi * t / 10) - 1
        z = 0.3 * t + 0.5
        pos_d = [x,y,z]
        return pos_d

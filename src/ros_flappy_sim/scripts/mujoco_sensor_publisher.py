#!/usr/bin/env python3

import rospy
import mujoco
import numpy as np
from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge
import cv2
import os

MODEL_PATH = os.path.join(os.path.dirname(__file__), '../src/Flappy_v9_RB.xml')


def main():
    rospy.init_node('mujoco_sensor_publisher')
    imu_pub = rospy.Publisher('/flappy/imu', Imu, queue_size=10)
    cam_pub = rospy.Publisher('/flappy/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    width, height = 640, 480
    rgb = np.zeros((height, width, 3), dtype=np.uint8)

    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown():
        mujoco.mj_step(model, data)

        # IMU: orientation (quaternion), angular velocity, linear velocity
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'body_frame'
        # sensordata: [pos(3), quat(4), linvel(3), gyro(3)]
        imu_msg.orientation.x = data.sensordata[3]
        imu_msg.orientation.y = data.sensordata[4]
        imu_msg.orientation.z = data.sensordata[5]
        imu_msg.orientation.w = data.sensordata[6]
        imu_msg.angular_velocity.x = data.sensordata[10]
        imu_msg.angular_velocity.y = data.sensordata[11]
        imu_msg.angular_velocity.z = data.sensordata[12]
        # Linear acceleration is not directly available; set to zero or compute if needed
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        imu_pub.publish(imu_msg)

        # Camera: render and publish image
        mujoco.mj_render(model, data, rgb, camera="onboard_camera")
        img_msg = bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = 'onboard_camera'
        cam_pub.publish(img_msg)

        rate.sleep()

if __name__ == '__main__':
    main()

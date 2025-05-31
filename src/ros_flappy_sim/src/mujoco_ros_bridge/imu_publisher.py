import rospy
from sensor_msgs.msg import Imu
import numpy as np

class IMUPublisher:
    def __init__(self, topic_name='/imu/data', rate=10):
        self.publisher = rospy.Publisher(topic_name, Imu, queue_size=10)
        self.rate = rospy.Rate(rate)
        self.imu_msg = Imu()

    def publish_data(self, orientation, angular_velocity, linear_acceleration):
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = 'imu_link'
        
        # Set orientation (quaternion)
        self.imu_msg.orientation.x = orientation[0]
        self.imu_msg.orientation.y = orientation[1]
        self.imu_msg.orientation.z = orientation[2]
        self.imu_msg.orientation.w = orientation[3]

        # Set angular velocity
        self.imu_msg.angular_velocity.x = angular_velocity[0]
        self.imu_msg.angular_velocity.y = angular_velocity[1]
        self.imu_msg.angular_velocity.z = angular_velocity[2]

        # Set linear acceleration
        self.imu_msg.linear_acceleration.x = linear_acceleration[0]
        self.imu_msg.linear_acceleration.y = linear_acceleration[1]
        self.imu_msg.linear_acceleration.z = linear_acceleration[2]

        self.publisher.publish(self.imu_msg)
        self.rate.sleep()
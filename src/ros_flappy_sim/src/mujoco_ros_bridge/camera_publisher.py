import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPublisher:
    def __init__(self, topic_name='/camera/image_raw', frame_rate=30):
        self.publisher = rospy.Publisher(topic_name, Image, queue_size=10)
        self.bridge = CvBridge()
        self.frame_rate = frame_rate
        self.cap = cv2.VideoCapture(0)  # Change the index if you have multiple cameras

    def publish_camera_frames(self):
        rate = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                # Convert the frame to a ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher.publish(ros_image)
            rate.sleep()

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
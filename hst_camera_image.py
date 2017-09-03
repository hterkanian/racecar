#!/usr/bin/python

# Author: Harry Terkanian
# July 25, 2017

import rospy
from sensor_msgs.msg import Image

class HSTImageViewerNode:


    def __init__(self):
        # ROS topic subscriptions & publications
        # subscribe to /zed/rgb/image_rect_color
        rospy.Subscriber("/camera/rgb/image_rect_color", 
            Image, self.camera_callback)
        # publish image
        self.pub_cmd = rospy.Publisher("/hst/image_pub", 
            Image, queue_size = 10)
        print("=====starting====")


    def camera_callback(self, msg):
        """receives and republishes an image."""
        print("image republished")
        self.msg = msg
        self.pub_cmd.publish(self.msg)


if __name__ == "__main__":
    rospy.init_node("hst_image_viewer", anonymous = True)
    node = HSTImageViewerNode()
    rospy.spin()

#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

class DisparityVisualizer:
    def __init__(self):
        rospy.init_node('disparity_visualizer')
        self.bridge = CvBridge()
        
        # Subscribe to disparity image
        self.sub = rospy.Subscriber('/vk180/disparity/image', Image, self.image_callback)
        
        print("Disparity Visualizer started. Press Ctrl+C to exit.")
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            if msg.encoding == "mono8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            elif msg.encoding == "mono16":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono16")
                # Normalize 16-bit to 8-bit for display
                cv_image = (cv_image / 256).astype(np.uint8)
            elif msg.encoding == "32FC1":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
                # Normalize float to 8-bit
                cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Apply colormap for better visualization
            colored_disparity = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)
            
            # Display
            cv2.imshow('Disparity Image', cv_image)
            cv2.imshow('Colored Disparity', colored_disparity)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        visualizer = DisparityVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo

def image_callback(msg):
    print(f"Disparity Image Encoding: {msg.encoding}")
    print(f"Dimensions: {msg.width}x{msg.height}")
    print(f"Step: {msg.step} bytes per row")
    
    # Sample pixel values
    if len(msg.data) > 0:
        if msg.encoding == "mono8":
            pixel_val = msg.data[0]
            print(f"Sample pixel value: {pixel_val}")
        elif msg.encoding == "mono16":
            if len(msg.data) >= 2:
                pixel_val = int.from_bytes(msg.data[0:2], byteorder='little')
                print(f"Sample pixel value: {pixel_val}")
        elif msg.encoding == "32FC1":
            if len(msg.data) >= 4:
                import struct
                pixel_val = struct.unpack('<f', msg.data[0:4])[0]
                print(f"Sample pixel value: {pixel_val:.3f}")

def camera_info_callback(msg):
    print(f"Camera Info:")
    print(f"  fx: {msg.K[0]:.2f}")
    print(f"  fy: {msg.K[4]:.2f}")  
    print(f"  cx: {msg.K[2]:.2f}")
    print(f"  cy: {msg.K[5]:.2f}")
    if len(msg.P) >= 4:
        baseline = -msg.P[3] / msg.K[0] if msg.K[0] != 0 else 0
        print(f"  baseline: {baseline:.4f}m")

if __name__ == '__main__':
    rospy.init_node('disparity_analyzer')
    
    rospy.Subscriber('/vk180/disparity/image', Image, image_callback)
    rospy.Subscriber('/vk180/disparity/camera_info', CameraInfo, camera_info_callback)
    
    print("Analyzing disparity data... Press Ctrl+C to exit")
    rospy.spin()
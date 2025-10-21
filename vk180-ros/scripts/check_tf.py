#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def check_tf_publishing():
    rospy.init_node('tf_checker')
    
    # Create TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(1.0)  # 1 Hz
    
    while not rospy.is_shutdown():
        try:
            # Try to get transform from odom to base_link
            transform = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time())
            
            print("✅ TF Transform Found:")
            print(f"  From: {transform.header.frame_id}")
            print(f"  To: {transform.child_frame_id}")
            print(f"  Time: {transform.header.stamp}")
            print(f"  Translation: [{transform.transform.translation.x:.3f}, {transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f}]")
            print(f"  Rotation: [{transform.transform.rotation.x:.3f}, {transform.transform.rotation.y:.3f}, {transform.transform.rotation.z:.3f}, {transform.transform.rotation.w:.3f}]")
            print("-" * 50)
            
        except tf2_ros.LookupException as e:
            print(f"❌ TF Lookup failed: {e}")
        except tf2_ros.ConnectivityException as e:
            print(f"❌ TF Connectivity issue: {e}")
        except tf2_ros.ExtrapolationException as e:
            print(f"❌ TF Extrapolation issue: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        check_tf_publishing()
    except rospy.ROSInterruptException:
        pass
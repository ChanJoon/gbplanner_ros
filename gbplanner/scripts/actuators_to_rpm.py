#!/usr/bin/env python
import rospy
from mav_msgs.msg import Actuators
from std_msgs.msg import Float32MultiArray

def actuator_callback(msg):
    rpm_array = Float32MultiArray()
    
    for v in msg.angular_velocities:
        rpm_array.data.append(v)

    rpm_pub.publish(rpm_array)
    rospy.loginfo_throttle(1.0, "Published RPMs: " + ", ".join(["%.1f" % v for v in rpm_array.data]))

def main():
    global rpm_pub

    rospy.init_node('actuator_to_rpm_node')
    rospy.Subscriber('/command/motor_speed', Actuators, actuator_callback)
    rpm_pub = rospy.Publisher('/quad_0/cmdRPM', Float32MultiArray, queue_size=10)

    rospy.loginfo("actuator_to_rpm_node started.")
    rospy.spin()

if __name__ == '__main__':
    main()

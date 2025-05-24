#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Point, Vector3
from quadrotor_msgs.msg import PositionCommand
import tf.transformations as transformations

latest_cmd = None

def trajectory_callback(msg):
    global latest_cmd
    if not msg.points:
        rospy.logwarn("Received empty trajectory!")
        return

    point = msg.points[0]
    tf = point.transforms[0]
    vel = point.velocities[0]
    acc = point.accelerations[0]

    cmd = PositionCommand()
    cmd.header.stamp = rospy.Time.now()
    cmd.position = Point(tf.translation.x, tf.translation.y, tf.translation.z)
    cmd.velocity = Vector3(vel.linear.x, vel.linear.y, vel.linear.z)
    cmd.acceleration = Vector3(acc.linear.x, acc.linear.y, acc.linear.z)

    quat = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
    euler = transformations.euler_from_quaternion(quat)
    cmd.yaw = euler[2]

    latest_cmd = cmd

def timer_callback(event):
    global latest_cmd
    if latest_cmd is not None:
        latest_cmd.header.stamp = rospy.Time.now()
        position_pub.publish(latest_cmd)
        rospy.loginfo_throttle(1.0, "Published cmd at: %.2f, %.2f, %.2f", latest_cmd.position.x, latest_cmd.position.y, latest_cmd.position.z)

def main():
    global position_pub
    rospy.init_node('traj_to_position_cmd_node')

    position_pub = rospy.Publisher('/quad_0/planning/pos_cmd', PositionCommand, queue_size=10)
    rospy.Subscriber('/command/trajectory', MultiDOFJointTrajectory, trajectory_callback)

    rospy.Timer(rospy.Duration(0.005), timer_callback)

    rospy.loginfo("traj_to_position_cmd_node with Timer started.")
    rospy.spin()

if __name__ == '__main__':
    main()

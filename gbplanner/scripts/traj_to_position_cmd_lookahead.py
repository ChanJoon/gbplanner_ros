#!/usr/bin/env python
import rospy
import math
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import tf.transformations as transformations

trajectory_points = []
target_idx = 0
DIST_THRESHOLD = 1.0

def trajectory_callback(msg):
    global trajectory_points, target_idx
    trajectory_points = msg.points
    target_idx = 0

def odom_callback(msg):
    global target_idx, trajectory_points

    if not trajectory_points or target_idx >= len(trajectory_points):
        return

    pos = msg.pose.pose.position
    current_pos = [pos.x, pos.y, pos.z]

    tf = trajectory_points[target_idx].transforms[0]
    goal_pos = [tf.translation.x, tf.translation.y, tf.translation.z]

    dist = euclidean_distance(current_pos, goal_pos)

    if dist < DIST_THRESHOLD and target_idx < len(trajectory_points) - 1:
        target_idx += 1

    point = trajectory_points[target_idx]
    publish_position_command(point)

def publish_position_command(point):
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

    position_pub.publish(cmd)

def euclidean_distance(p1, p2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

def main():
    global position_pub
    rospy.init_node('traj_to_position_cmd_lookahead_node')

    rospy.Subscriber('/command/trajectory', MultiDOFJointTrajectory, trajectory_callback)
    rospy.Subscriber('/quad_0/odom', Odometry, odom_callback)
    position_pub = rospy.Publisher('/quad_0/planning/pos_cmd', PositionCommand, queue_size=10)

    rospy.loginfo("traj_to_position_cmd_lookahead_node started.")
    rospy.spin()

if __name__ == '__main__':
    main()

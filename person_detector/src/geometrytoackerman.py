#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
# Modifier: Donghee Han, hdh7485@kaist.ac.kr

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    # Avoid division by zero
    if omega == 0:
        omega = 0.0001
    if v == 0:
        v = 0.0001

    # Calculate the steering angle
    radius = v / omega
    angle = math.atan(wheelbase / radius)

    # Limit the steering angle
    max_steering_angle = 0.35
    if angle > max_steering_angle:
        angle = max_steering_angle
    elif angle < -max_steering_angle:
        angle = -max_steering_angle

    return angle

def cmd_callback(data):
    global wheelbase, ackermann_cmd_topic, frame_id, pub

    v = data.linear.x
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

    # Create and publish the AckermannDriveStamped message
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.drive.steering_angle = steering
    msg.drive.speed = v
    pub.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('cmd_vel_to_ackermann_drive')

        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd_mux/input/teleop')
        wheelbase = rospy.get_param('~wheelbase', 1.0)
        frame_id = rospy.get_param('~frame_id', 'base_link')

        rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=5)
        pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=5)

        rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started. Listening to %s, publishing to %s. Frame id: %s, wheelbase: %f",
                      twist_cmd_topic, ackermann_cmd_topic, frame_id, wheelbase)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

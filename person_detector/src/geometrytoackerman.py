#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

global old_msg

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0:
    omega = 0.00001
  if v == 0:
    v = 0.00001

  radius = v / omega
  angle = math.atan(wheelbase / radius)
  if angle > 0.34:
    angle = 0.34
  elif angle < -0.34:
    angle = -0.34
  return angle

def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub

  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

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
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/car/mux/ackermann_cmd_mux/input/teleop')
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=10)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

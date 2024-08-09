#!/usr/bin/env python

import Jetson.GPIO as GPIO
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Float64

class PushButton:

    def __init__(self):
        self.gpio_pin = rospy.get_param("push_button/gpio_pin", 33)
        self.pub_topic = rospy.get_param("push_button/pub_topic", "push_button_state")
        self.pub_rate = rospy.get_param("push_button/pub_rate", 100)

        self.state_pub = rospy.Publisher(self.pub_topic, Bool, queue_size=10)

    def start(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpio_pin, GPIO.IN)

        loop_rate = rospy.Rate(self.pub_rate)
        bool_msg = Bool()

	speedpub = rospy.Publisher('/car/vesc/commands/motor/speed', Float64, queue_size=10)
	speed_msg = Float64()

        try:
            while True:
                val = GPIO.input(self.gpio_pin)

                bool_msg.data = (val < 1)
                self.state_pub.publish(bool_msg)
		if bool_msg.data == True:
			speed_msg.data = 0.0
			speedpub.publish(speed_msg)

                loop_rate.sleep()

        finally:
            GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node("push_button_node", anonymous=True)

    pb = PushButton()
    pb.start()        

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class TwistRepeater:
    def __init__(self):
        rospy.init_node('twist_repeater', anonymous=True)
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.publisher = rospy.Publisher('/terrasentia/cmd_vel', TwistStamped, queue_size=10)

    def callback(self, data):
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = data.linear.x
        twist_msg.twist.linear.y = data.linear.y
        twist_msg.twist.linear.z = data.linear.z
        twist_msg.twist.angular.x = data.angular.x
        twist_msg.twist.angular.y = data.angular.y
        twist_msg.twist.angular.z = data.angular.z
        twist_msg.header.stamp = rospy.Time.now()

        self.publisher.publish(twist_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    repeater = TwistRepeater()
    repeater.run()
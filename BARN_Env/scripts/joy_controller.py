#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToCmdVel:
    def __init__(self):
        rospy.init_node("joy_to_cmdvel")

        self.max_linear = rospy.get_param("~max_linear", 1.0)
        self.max_angular = rospy.get_param("~max_angular", 1)
        self.deadzone = rospy.get_param("~deadzone", 0.05)

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_cb)

        rospy.loginfo("Joy to cmd_vel started (dual-stick, no trigger).")

    def dz(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def joy_cb(self, msg: Joy):
        twist = Twist()

        # 左摇杆：前后
        forward = self.dz(msg.axes[1])

        # 右摇杆：左右
        steer = self.dz(msg.axes[3])

        twist.linear.x  = forward * self.max_linear
        twist.angular.z = steer   * self.max_angular

        self.pub.publish(twist)


if __name__ == "__main__":
    JoyToCmdVel()
    rospy.spin()

#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8


class JoyToCmdVel:
    def __init__(self):
        rospy.init_node("joy_to_cmdvel")

        # ===== 参数 =====
        self.max_linear = rospy.get_param("~max_linear", 1.0)
        self.max_angular = rospy.get_param("~max_angular", 1.0)
        self.deadzone = rospy.get_param("~deadzone", 0.05)

        # RB 键
        self.mode_button = rospy.get_param("~mode_button", 5)

        # ===== 发布器 =====
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.mode_pub = rospy.Publisher("mode_select", UInt8, queue_size=10)

        rospy.Subscriber("joy", Joy, self.joy_cb)

        # ===== 状态 =====
        self.mode = 0
        self.last_button_state = 0

        rospy.loginfo("Joy to cmd_vel started with mode switching (0-3).")

    def dz(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def joy_cb(self, msg: Joy):
        twist = Twist()

        # ======================
        # 1. 模态相关控制
        # ======================
        if self.mode == 2:
            # ===== 自转模态 ：360° 速度向量控制 =====

            # 左摇杆 Y：速度大小
            v = self.dz(msg.axes[1]) * self.max_linear

            # 右摇杆 X,Y：方向
            rx = self.dz(msg.axes[3])
            ry = self.dz(msg.axes[4])   # 关键：Y 取反

            if abs(rx) > 1e-6 or abs(ry) > 1e-6:
                theta = math.atan2(rx, ry)   # 0 在正上方
            else:
                theta = 0.0

            twist.angular.z = theta   # 这是方向角（rad），不是角速度
            twist.linear.x = v

        else:
            # ===== 其他模态：传统差速 / Ackermann =====
            forward = self.dz(msg.axes[1])
            steer = self.dz(msg.axes[3])

            twist.linear.x = forward * self.max_linear
            twist.angular.z = steer * self.max_angular

        self.cmd_pub.publish(twist)

        # ======================
        # 2. 模态切换（RB/R1）
        # ======================
        if self.mode_button < len(msg.buttons):
            button_state = msg.buttons[self.mode_button]

            if button_state == 1 and self.last_button_state == 0:
                self.mode = (self.mode + 1) % 4
                self.mode_pub.publish(UInt8(data=self.mode))
                rospy.loginfo(f"[Joy] Switch mode -> {self.mode}")

            self.last_button_state = button_state


if __name__ == "__main__":
    JoyToCmdVel()
    rospy.spin()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates


class GazeboWorldTF:
    def __init__(self):
        rospy.init_node("gazebo_world_tf")

        # =========================
        # 参数
        # =========================
        self.robot_name = rospy.get_param("~robot_name", "jackal")

        self.world_frame = rospy.get_param("~world_frame", "world")
        self.base_frame  = rospy.get_param("~base_frame", "base_link")

        self.publish_rate = rospy.get_param("~publish_rate", 100.0)
        self.min_dt = rospy.Duration(1.0 / self.publish_rate)

        # =========================
        # TF Broadcaster
        # =========================
        self.tf_br = tf2_ros.TransformBroadcaster()
        self.last_pub_time = rospy.Time(0)

        # =========================
        # 订阅 Gazebo 真值
        # =========================
        rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.model_states_callback,
            queue_size=1
        )

        rospy.loginfo("===================================")
        rospy.loginfo("Gazebo World TF started")
        rospy.loginfo(" Publishing: %s -> %s", self.world_frame, self.base_frame)
        rospy.loginfo("===================================")

    # ======================================================
    def model_states_callback(self, msg: ModelStates):
        if self.robot_name not in msg.name:
            return

        idx = msg.name.index(self.robot_name)
        pose = msg.pose[idx]

        now = rospy.Time.now()
        if abs(now - self.last_pub_time) < self.min_dt:
            return

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_frame

        # Gazebo 真值（不加任何偏移）
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_br.sendTransform(t)
        self.last_pub_time = now


if __name__ == "__main__":
    GazeboWorldTF()
    rospy.spin()

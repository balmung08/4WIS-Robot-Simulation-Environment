#!/usr/bin/env python3
"""
Collision / SAFE 状态可视化(修复时间倒退问题 - 使用Rate循环)
"""

import rospy
import time
from gazebo_msgs.msg import ContactsState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class CollisionVisualizer:
    def __init__(self):
        rospy.init_node("collision_visualizer", anonymous=True)

        self.robot_name = rospy.get_param("~robot_name", "robot_0")
        self.frame_id = rospy.get_param("~frame_id", "robot_0/base_link")
        self.text_height = rospy.get_param("~text_height", 0.8)
        self.update_rate = rospy.get_param("~update_rate", 20.0)  # Hz

        self.marker_pub = rospy.Publisher(
            "/collision_marker", Marker, queue_size=5
        )

        self.sub = rospy.Subscriber(
            f"/{self.robot_name}/collision_states",
            ContactsState,
            self.collision_callback,
        )

        # TF2 buffer 用于坐标转换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.is_colliding = False
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.cooldown = 0.5

        self.last_contacts = []
        self.last_msg_stamp = rospy.Time.now()

        # 使用Rate控制循环频率
        self.rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo("Collision visualizer started.")
        rospy.loginfo(f"Update rate: {self.update_rate} Hz")

    # =====================================================
    def collision_callback(self, msg):
        now = time.time()
        self.is_colliding = len(msg.states) > 0
        
        # 保存消息时间戳
        self.last_msg_stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        if self.is_colliding:
            if now - self.last_collision_time > self.cooldown:
                self.collision_count += 1
                self.last_collision_time = now

            # 转换碰撞点坐标
            self.last_contacts = self.transform_contact_points(
                msg.states[0].contact_positions,
                self.last_msg_stamp
            )
        else:
            self.last_contacts = []

    # =====================================================
    def transform_contact_points(self, world_points, stamp):
        """将world坐标系的点转换到robot坐标系"""
        transformed_points = []
        
        try:
            # 获取从world到robot_frame的变换
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                "world",
                stamp,
                rospy.Duration(0.02)
            )
            
            for point in world_points:
                point.z = 1
                # 创建PointStamped消息
                point_stamped = PointStamped()
                point_stamped.header.frame_id = "world"
                point_stamped.header.stamp = stamp
                point_stamped.point = point
                # 执行转换
                transformed = do_transform_point(point_stamped, transform)
                transformed_points.append(transformed.point)
                
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"TF transform failed: {e}")
            return world_points
            
        return transformed_points

    # =====================================================
    def publish_text_marker(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "collision_status"
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = self.text_height
        m.pose.orientation.w = 1.0

        m.scale.z = 0.25
        m.lifetime = rospy.Duration(0.2)

        if self.is_colliding:
            m.text = f"COLLISION"
            m.color.r = 0.95
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 1.0
        else:
            m.text = "SAFE"
            m.color.r = 0.1
            m.color.g = 0.9
            m.color.b = 0.1
            m.color.a = 0.9
        
        self.marker_pub.publish(m)

    # =====================================================
    def publish_contact_points(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "collision_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        marker.color.r = 0.95
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0.2)

        if self.last_contacts:
            marker.points = self.last_contacts
            self.marker_pub.publish(marker)
        else:
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)

    # =====================================================
    def run(self):
        while not rospy.is_shutdown():
            try:
                self.publish_text_marker()
                self.publish_contact_points()
                self.rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("Time jumped backwards (Gazebo reset?), reinitializing rate...")
                # 重新创建Rate对象
                self.rate = rospy.Rate(self.update_rate)
            except rospy.ROSInterruptException:
                break


if __name__ == "__main__":
    try:
        CollisionVisualizer().run()
    except rospy.ROSInterruptException as e:
        pass
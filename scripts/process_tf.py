#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker


def create_marker(position, scale, angle):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "vector_info_markers"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.orientation.w = np.cos(np.rad2deg(angle) / 2)
    marker.scale = scale
    marker.color.a = 1.0  # Fully opaque
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 0.0  # Blue
    return marker


def transform_listener():
    rospy.init_node("transform_listener", anonymous=True)
    vector_info_pub = rospy.Publisher("vector_info", String, queue_size=10)
    marker_pub = rospy.Publisher("vector_info_marker", Marker, queue_size=10)
    rate = rospy.Rate(10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(2.0)

    update_interval = rospy.Duration(5.0)
    previous_pos = None
    current_pos = None

    while not rospy.is_shutdown():
        try:
            trans_odom_to_base_footprint = tfBuffer.lookup_transform(
                "odom", "base_footprint", rospy.Time()
            )

            trans_base_footprint_to_base_link = tfBuffer.lookup_transform(
                "base_footprint", "base_link", rospy.Time()
            )

            trans_base_link_to_human = tfBuffer.lookup_transform(
                "base_link", "human1", rospy.Time()
            )

            trans_odom_to_human = do_transform_pose(
                geometry_msgs.msg.PoseStamped(), trans_base_link_to_human
            )
            trans_odom_to_human = do_transform_pose(
                trans_odom_to_human, trans_base_footprint_to_base_link
            )
            trans_odom_to_human = do_transform_pose(
                trans_odom_to_human, trans_odom_to_base_footprint
            )

            if previous_pos is None:
                previous_pos = (
                    trans_odom_to_human.pose.position.x,
                    trans_odom_to_human.pose.position.y,
                )
                rate.sleep()
                continue

            current_pos = (
                trans_odom_to_human.pose.position.x,
                trans_odom_to_human.pose.position.y,
            )

            if rospy.Time.now() - trans_odom_to_human.header.stamp > update_interval:
                vector = (
                    current_pos[0] - previous_pos[0],
                    current_pos[1] - previous_pos[1],
                )
                vector_length = np.sqrt(vector[0] ** 2 + vector[1] ** 2)
                angle_radians = np.arctan2(vector[1], vector[0])
                angle_degrees = np.rad2deg(angle_radians)

                previous_pos = current_pos
                vector_info = f"Position_x: {current_pos[0]}, Position_y: {current_pos[1]}, Vector_Length: {vector_length}, Angle: {angle_degrees}"
                vector_info_pub.publish(vector_info)
                marker = create_marker(
                    current_pos, scale=(vector_length, 0.1, 0.1), angle=angle_radians
                )
                marker_pub.publish(marker)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr("Transform lookup failed!")

        rate.sleep()


if __name__ == "__main__":
    transform_listener()

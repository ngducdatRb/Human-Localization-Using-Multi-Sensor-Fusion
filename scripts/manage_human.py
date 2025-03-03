#!/usr/bin/env python3

import rospy
from project2.msg import human_msg
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry


def create_topic(human):
    human_topic_sub_camera = "/" + human + "_position_camera"

    human_sub_camera = rospy.Subscriber(
        human_topic_sub_camera,
        human_msg,
        callback=lambda msg, human=human: human_camera_callback(msg, human),
    )

    human_topic_sub_lidar = "/" + human + "_position_lidar"

    human_sub_lidar = rospy.Subscriber(
        human_topic_sub_lidar,
        human_msg,
        callback=lambda msg, human=human: human_lidar_callback(msg, human),
    )


def human_lidar_callback(msg, human):
    human_topic_pub_odom_lidar = "/" + human + "_lidar_data"
    odom_lidar = Odometry()

    odom_lidar.header.frame_id = "base_link"
    odom_lidar.child_frame_id = human
    odom_lidar.header.stamp = rospy.Time.now()

    odom_lidar.pose.pose.position.x = msg.lidar_location.position.x
    odom_lidar.pose.pose.position.y = msg.lidar_location.position.y
    odom_lidar.pose.pose.position.z = msg.lidar_location.position.z

    odom_lidar.pose.pose.orientation.x = msg.lidar_location.orientation.x
    odom_lidar.pose.pose.orientation.y = msg.lidar_location.orientation.y
    odom_lidar.pose.pose.orientation.z = msg.lidar_location.orientation.z
    odom_lidar.pose.pose.orientation.w = msg.lidar_location.orientation.w

    odom_lidar.pose.covariance[0] = 0.001
    odom_lidar.pose.covariance[7] = 0.001
    odom_lidar.pose.covariance[14] = 0.001
    odom_lidar.pose.covariance[21] = 0.001
    odom_lidar.pose.covariance[28] = 0.001
    odom_lidar.pose.covariance[35] = 0.003

    human_pub_odom_lidar = rospy.Publisher(
        human_topic_pub_odom_lidar, Odometry, queue_size=10
    )

    human_pub_odom_lidar.publish(odom_lidar)


def human_camera_callback(msg, human):
    human_topic_pub_odom_camera = "/" + human + "_camera_data"
    odom_camera = Odometry()

    odom_camera.header.frame_id = "base_link"
    odom_camera.child_frame_id = human
    odom_camera.header.stamp = rospy.Time.now()

    odom_camera.pose.pose.position.x = msg.camera_location.position.x
    odom_camera.pose.pose.position.y = msg.camera_location.position.y
    odom_camera.pose.pose.position.z = msg.camera_location.position.z

    odom_camera.pose.pose.orientation.x = msg.camera_location.orientation.x
    odom_camera.pose.pose.orientation.y = msg.camera_location.orientation.y
    odom_camera.pose.pose.orientation.z = msg.camera_location.orientation.z
    odom_camera.pose.pose.orientation.w = msg.camera_location.orientation.w

    odom_camera.pose.covariance[0] = 0.001
    odom_camera.pose.covariance[7] = 0.001
    odom_camera.pose.covariance[14] = 0.001
    odom_camera.pose.covariance[21] = 0.001
    odom_camera.pose.covariance[28] = 0.001
    odom_camera.pose.covariance[35] = 0.003

    human_pub_odom_camera = rospy.Publisher(
        human_topic_pub_odom_camera, Odometry, queue_size=10
    )

    human_pub_odom_camera.publish(odom_camera)


def main():
    rospy.init_node("manage_human_node", anonymous=True)
    rate = rospy.Rate(10)

    if rospy.has_param("human_list"):
        retrieved_list = rospy.get_param("human_list")
        if retrieved_list is not None:
            for human in retrieved_list:
                create_topic(human)

        tf_broadcaster = TransformBroadcaster()

        while not rospy.is_shutdown():
            for human in retrieved_list:
                human_msgs = rospy.wait_for_message(
                    "/" + human + "_position_lidar", human_msg
                )
                t = TransformStamped()
                t.header.frame_id = "base_link"
                t.child_frame_id = human
                t.header.stamp = rospy.Time.now()
                t.transform.translation.x = human_msgs.human_location.position.x
                t.transform.translation.y = human_msgs.human_location.position.y
                t.transform.translation.z = human_msgs.human_location.position.z
                t.transform.rotation.x = human_msgs.human_location.orientation.x
                t.transform.rotation.y = human_msgs.human_location.orientation.y
                t.transform.rotation.z = human_msgs.human_location.orientation.z
                t.transform.rotation.w = human_msgs.human_location.orientation.w
                tf_broadcaster.sendTransform(t)
            rate.sleep()
    else:
        rospy.loginfo("No parameter 'human_list' found.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

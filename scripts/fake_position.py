#!/usr/bin/env python3

import rospy
from project2.msg import human_msg
import numpy as np


def add_gaussian_noise(value, mean, std_dev):
    noise = np.random.normal(mean, std_dev)
    return value + noise


def main():
    rospy.init_node("fake_position", anonymous=True)

    human_pub_1 = rospy.Publisher("human1_position_camera", human_msg, queue_size=10)
    human_pub_2 = rospy.Publisher("human2_position_camera", human_msg, queue_size=10)
    human_pub_3 = rospy.Publisher("human3_position_camera", human_msg, queue_size=10)
    human_list = ["human1", "human2", "human3"]
    rospy.set_param("human_list", human_list)

    rate = rospy.Rate(10)

    msg_1 = human_msg()
    msg_1.human_header.frame_id = "human1"
    msg_2 = human_msg()
    msg_2.human_header.frame_id = "human2"
    msg_3 = human_msg()
    msg_3.human_header.frame_id = "human3"

    while not rospy.is_shutdown():
        msg_1.human_header.stamp = rospy.Time.now()

        msg_1.human_location.position.x = add_gaussian_noise(2.0, 0.0, 0.1)
        msg_1.human_location.position.y = add_gaussian_noise(2.0, 0.0, 0.1)
        msg_1.human_location.position.z = 0.0

        msg_1.human_location.orientation.x = 0.0
        msg_1.human_location.orientation.y = 0.0
        msg_1.human_location.orientation.z = 0.0
        msg_1.human_location.orientation.w = 1.0

        msg_1.camera_location.position.x = add_gaussian_noise(1.8, 0.0, 0.1)
        msg_1.camera_location.position.y = add_gaussian_noise(1.8, 0.0, 0.1)
        msg_1.camera_location.position.z = 0.0

        msg_1.camera_location.orientation.x = 0.0
        msg_1.camera_location.orientation.y = 0.0
        msg_1.camera_location.orientation.z = 0.0
        msg_1.camera_location.orientation.w = 1.0

        msg_1.lidar_location.position.x = add_gaussian_noise(2.2, 0.0, 0.1)
        msg_1.lidar_location.position.y = add_gaussian_noise(2.2, 0.0, 0.1)
        msg_1.lidar_location.position.z = 0.0

        msg_1.lidar_location.orientation.x = 0.0
        msg_1.lidar_location.orientation.y = 0.0
        msg_1.lidar_location.orientation.z = 0.0
        msg_1.lidar_location.orientation.w = 1.0

        msg_2.human_header.stamp = rospy.Time.now()

        msg_2.human_location.position.x = add_gaussian_noise(2.0, 0.0, 0.1)
        msg_2.human_location.position.y = add_gaussian_noise(2.0, 0.0, 0.1)
        msg_2.human_location.position.z = 0.0

        msg_2.human_location.orientation.x = 0.0
        msg_2.human_location.orientation.y = 0.0
        msg_2.human_location.orientation.z = 0.0
        msg_2.human_location.orientation.w = 1.0

        msg_2.camera_location.position.x = add_gaussian_noise(1.8, 0.0, 0.1)
        msg_2.camera_location.position.y = add_gaussian_noise(1.8, 0.0, 0.1)
        msg_2.camera_location.position.z = 0.0

        msg_2.camera_location.orientation.x = 0.0
        msg_2.camera_location.orientation.y = 0.0
        msg_2.camera_location.orientation.z = 0.0
        msg_2.camera_location.orientation.w = 1.0

        msg_2.lidar_location.position.x = add_gaussian_noise(2.2, 0.0, 0.1)
        msg_2.lidar_location.position.y = add_gaussian_noise(2.2, 0.0, 0.1)
        msg_2.lidar_location.position.z = 0.0

        msg_2.lidar_location.orientation.x = 0.0
        msg_2.lidar_location.orientation.y = 0.0
        msg_2.lidar_location.orientation.z = 0.0
        msg_2.lidar_location.orientation.w = 1.0

        msg_3.human_header.stamp = rospy.Time.now()

        msg_3.human_location.position.x = add_gaussian_noise(2.0, 0.0, 0.1)
        msg_3.human_location.position.y = add_gaussian_noise(2.0, 0.0, 0.1)
        msg_3.human_location.position.z = 0.0

        msg_3.human_location.orientation.x = 0.0
        msg_3.human_location.orientation.y = 0.0
        msg_3.human_location.orientation.z = 0.0
        msg_3.human_location.orientation.w = 1.0

        msg_3.camera_location.position.x = add_gaussian_noise(1.8, 0.0, 0.1)
        msg_3.camera_location.position.y = add_gaussian_noise(1.8, 0.0, 0.1)
        msg_3.camera_location.position.z = 0.0

        msg_3.camera_location.orientation.x = 0.0
        msg_3.camera_location.orientation.y = 0.0
        msg_3.camera_location.orientation.z = 0.0
        msg_3.camera_location.orientation.w = 1.0

        msg_3.lidar_location.position.x = add_gaussian_noise(2.2, 0.0, 0.1)
        msg_3.lidar_location.position.y = add_gaussian_noise(2.2, 0.0, 0.1)
        msg_3.lidar_location.position.z = 0.0

        msg_3.lidar_location.orientation.x = 0.0
        msg_3.lidar_location.orientation.y = 0.0
        msg_3.lidar_location.orientation.z = 0.0
        msg_3.lidar_location.orientation.w = 1.0

        human_pub_1.publish(msg_1)
        human_pub_2.publish(msg_2)
        human_pub_3.publish(msg_3)

        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

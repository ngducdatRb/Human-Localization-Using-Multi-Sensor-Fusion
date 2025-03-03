#!/usr/bin/env python3

import rospy
import numpy as np
import json
from std_msgs.msg import String
from project2.msg import human_msg

g_distance = 0


def bbox_lidar_callback(data, processed_bbox_publisher_lidar):
    global g_distance
    bbox_data = json.loads(data.data)
    msg = human_msg()

    lidar2baselink_pos_x = 0.315
    lidar2baselink_pos_y = 0.084

    for obj in bbox_data:
        distance = obj.get("distance", 0)
        angle = obj.get("angle", 0)

        if g_distance == 0 and distance != 0:
            g_distance = distance

        if g_distance != 0 and distance != 0 and abs(g_distance - distance) < 0.5:
            g_distance = distance

        leg2lidar_pos_x = distance * np.cos(angle)
        leg2lidar_pos_y = distance * np.sin(angle)

        leg2baselink_pos_x = leg2lidar_pos_x + lidar2baselink_pos_x
        leg2baselink_pos_y = leg2lidar_pos_y + lidar2baselink_pos_y

        if distance is not None:
            msg.human_header.frame_id = "human1"
            msg.human_header.stamp = rospy.Time.now()
            msg.human_location.position.x = leg2baselink_pos_x
            msg.human_location.position.y = leg2baselink_pos_y

            msg.human_location.orientation.w = 1.0

            msg.lidar_location.position.x = leg2baselink_pos_x
            msg.lidar_location.position.y = leg2baselink_pos_y

            processed_bbox_publisher_lidar.publish(msg)


def main():
    rospy.init_node("lidar_processor_node", anonymous=True)

    human_list = ["human1"]
    rospy.set_param("human_list", human_list)

    processed_bbox_publisher_lidar = rospy.Publisher(
        "/human1_position_lidar", human_msg, queue_size=10
    )

    bbox_subcriber = rospy.Subscriber(
        "/laser_detector",
        String,
        callback=lambda data: bbox_lidar_callback(data, processed_bbox_publisher_lidar),
    )

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

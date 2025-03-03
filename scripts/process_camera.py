#!/usr/bin/env python3

import rospy
import numpy as np
import json
from std_msgs.msg import String
from project2.msg import human_msg

g_distance = 0


def bbox_camera_callback(data, processed_bbox_publisher_camera):
    global g_distance
    bbox_data = json.loads(data.data)
    msg = human_msg()
    camera2baselink_pos_x = 0.035
    camera2baselink_pos_y = 0.075
    camera2baselink_pos_z = 0.96

    for obj in bbox_data:
        distance = obj.get("distance", 0)
        angle_bearing = np.deg2rad(obj.get("angle", [0, 0])[0])
        angle_elevation = np.deg2rad(obj.get("angle", [0, 0])[1])

        if g_distance == 0 and distance != 0:
            g_distance = distance

        if g_distance != 0 and distance != 0 and abs(g_distance - distance) < 0.5:
            g_distance = distance

        face2camera_pos_x = g_distance * np.cos(angle_elevation) * np.cos(angle_bearing)
        face2camera_pos_y = g_distance * np.cos(angle_elevation) * np.sin(angle_bearing)

        face2baselink_pos_x = face2camera_pos_x + camera2baselink_pos_x
        face2baselink_pos_y = face2camera_pos_y + camera2baselink_pos_y

        if distance is not None:
            msg.human_header.frame_id = "human1"
            msg.human_header.stamp = rospy.Time.now()
            msg.human_location.position.x = face2camera_pos_x
            msg.human_location.position.y = face2baselink_pos_y

            msg.human_location.orientation.w = 1.0

            msg.camera_location.position.x = face2camera_pos_x
            msg.camera_location.position.y = face2baselink_pos_y

            processed_bbox_publisher_camera.publish(msg)


def main():
    rospy.init_node("camera_processor_node", anonymous=True)

    human_list = ["human1"]
    rospy.set_param("human_list", human_list)

    processed_bbox_publisher_camera = rospy.Publisher(
        "/human1_position_camera", human_msg, queue_size=10
    )

    bbox_subcriber = rospy.Subscriber(
        "/bbox_topic",
        String,
        callback=lambda data: bbox_camera_callback(
            data, processed_bbox_publisher_camera
        ),
    )

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

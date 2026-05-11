#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler


def covariance():
    cov = [0.0] * 36
    cov[0] = rospy.get_param("~cov_x", 0.25)
    cov[7] = rospy.get_param("~cov_y", 0.25)
    cov[35] = rospy.get_param("~cov_yaw", 0.0685)
    return cov


def make_pose():
    frame_id = rospy.get_param("~frame_id", "map")
    x = float(rospy.get_param("~x", -2.0))
    y = float(rospy.get_param("~y", -0.5))
    yaw = float(rospy.get_param("~yaw", 0.0))
    q = quaternion_from_euler(0.0, 0.0, yaw)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    msg.pose.covariance = covariance()
    return msg


def main():
    rospy.init_node("initial_pose_pub")
    delay = float(rospy.get_param("~delay", 3.0))
    repeat = int(rospy.get_param("~repeat", 5))
    rate_hz = float(rospy.get_param("~rate", 2.0))
    topic = rospy.get_param("~topic", "/initialpose")

    pub = rospy.Publisher(topic, PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(delay)
    rate = rospy.Rate(rate_hz)
    for _ in range(max(1, repeat)):
        if rospy.is_shutdown():
            break
        msg = make_pose()
        pub.publish(msg)
        rospy.loginfo("Published initial pose: x=%.3f y=%.3f yaw=%.3f",
                      msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      float(rospy.get_param("~yaw", 0.0)))
        rate.sleep()


if __name__ == "__main__":
    main()

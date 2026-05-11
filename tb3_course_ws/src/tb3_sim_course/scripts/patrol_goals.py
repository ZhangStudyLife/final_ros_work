#!/usr/bin/env python3

import math

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


class PatrolGoals:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.goals = rospy.get_param("~goals", [])
        if not self.goals:
            raise rospy.ROSException("No patrol goals configured")

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            raise rospy.ROSException("move_base action server is not available")

    def run(self):
        index = 0
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            goal_data = self.goals[index]
            goal = self.make_goal(goal_data)
            rospy.loginfo("Sending patrol goal %d: x=%.2f y=%.2f yaw=%.2f",
                          index, goal_data["x"], goal_data["y"], goal_data.get("yaw", 0.0))
            self.client.send_goal(goal)
            finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout))
            if not finished:
                rospy.logwarn("Goal %d timed out; canceling and continuing", index)
                self.client.cancel_goal()
            else:
                rospy.loginfo("Goal %d finished with state %s", index, self.client.get_state())

            index = (index + 1) % len(self.goals)
            rate.sleep()

    def make_goal(self, data):
        yaw = float(data.get("yaw", 0.0))
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(data["x"])
        goal.target_pose.pose.position.y = float(data["y"])
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        return goal


def main():
    rospy.init_node("patrol_goals")
    PatrolGoals().run()


if __name__ == "__main__":
    main()

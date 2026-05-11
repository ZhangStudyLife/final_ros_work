#!/usr/bin/env python3

import csv
import math
import os
import threading
from datetime import datetime

import rospy
from actionlib_msgs.msg import GoalStatusArray
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionFeedback
from std_msgs.msg import String


class RedBallNavMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.target = None
        self.target_recv_time = rospy.Time(0)
        self.cmd = None
        self.det_status = ""
        self.nav_status = ""
        self.move_base_status = ""
        self.move_base_state = ""
        self.robot_x = math.nan
        self.robot_y = math.nan
        self.robot_yaw = math.nan
        self.csv_file = None
        self.csv_writer = None
        self.get_model_state = None

        self.log_dir = os.path.expanduser(rospy.get_param("~log_dir", "~/final_work/tb3_course_ws/logs"))
        self.robot_model_name = rospy.get_param("~robot_model_name", "turtlebot3_waffle_pi")
        self.ball_model_name = rospy.get_param("~ball_model_name", "red_ball")
        self.csv_path = self.make_csv_path()
        self.open_csv()
        self.connect_gazebo_service()

        rospy.Subscriber("/red_ball/target_base", PoseStamped, self.target_cb, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=1)
        rospy.Subscriber("/red_ball/status", String, self.det_status_cb, queue_size=1)
        rospy.Subscriber("/red_ball/nav_status", String, self.nav_status_cb, queue_size=1)
        rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.move_base_feedback_cb, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_cb, queue_size=1)

    def target_cb(self, msg):
        with self.lock:
            self.target = msg
            self.target_recv_time = rospy.Time.now()

    def cmd_cb(self, msg):
        with self.lock:
            self.cmd = msg

    def det_status_cb(self, msg):
        with self.lock:
            self.det_status = msg.data

    def nav_status_cb(self, msg):
        with self.lock:
            self.nav_status = msg.data

    def move_base_feedback_cb(self, msg):
        pose = msg.feedback.base_position.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        with self.lock:
            self.move_base_status = "base=({:.2f},{:.2f})".format(pose.position.x, pose.position.y)
            self.robot_x = pose.position.x
            self.robot_y = pose.position.y
            self.robot_yaw = yaw

    def move_base_status_cb(self, msg):
        status = "none"
        if msg.status_list:
            latest = msg.status_list[-1]
            status = "{}:{}".format(latest.status, latest.text)
        with self.lock:
            self.move_base_state = status

    def run(self):
        rate = rospy.Rate(float(rospy.get_param("~rate", 1.0)))
        while not rospy.is_shutdown():
            with self.lock:
                target = self.target
                target_recv_time = self.target_recv_time
                cmd = self.cmd
                det_status = self.det_status
                nav_status = self.nav_status
                move_base_status = self.move_base_status
                move_base_state = self.move_base_state
                robot_x = self.robot_x
                robot_y = self.robot_y
                robot_yaw = self.robot_yaw

            target_text = "target=none"
            target_x = math.nan
            target_y = math.nan
            target_bearing = math.nan
            target_age = math.nan
            target_frame = ""
            if target is not None:
                target_x = target.pose.position.x
                target_y = target.pose.position.y
                target_bearing = math.atan2(target_y, max(target_x, 1e-3))
                target_age = (rospy.Time.now() - target_recv_time).to_sec()
                target_frame = target.header.frame_id
                target_text = "target_base=({:.2f},{:.2f}) bearing={:.2f}".format(target_x, target_y, target_bearing)

            cmd_text = "cmd=none"
            cmd_linear = math.nan
            cmd_angular = math.nan
            if cmd is not None:
                cmd_linear = cmd.linear.x
                cmd_angular = cmd.angular.z
                cmd_text = "cmd=({:.2f},{:.2f})".format(cmd_linear, cmd_angular)

            robot_gt = self.get_model_pose(self.robot_model_name)
            ball_gt = self.get_model_pose(self.ball_model_name)
            gt_distance = distance_xy(robot_gt, ball_gt)
            gt_bearing = bearing_from_robot_to_ball(robot_gt, ball_gt)
            self.write_csv(
                robot_x,
                robot_y,
                robot_yaw,
                robot_gt,
                ball_gt,
                gt_distance,
                gt_bearing,
                target_x,
                target_y,
                target_bearing,
                target_age,
                target_frame,
                cmd_linear,
                cmd_angular,
                det_status,
                nav_status,
                move_base_state,
            )

            rospy.loginfo("%s | %s | %s | move_base=[%s] | det=[%s] | nav=[%s]",
                          target_text, cmd_text, move_base_status, move_base_state, det_status, nav_status)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        self.close_csv()

    def make_csv_path(self):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.log_dir, "red_ball_nav_{}.csv".format(stamp))

    def open_csv(self):
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "wall_time",
            "ros_time",
            "robot_x",
            "robot_y",
            "robot_yaw",
            "robot_gt_x",
            "robot_gt_y",
            "robot_gt_yaw",
            "ball_gt_x",
            "ball_gt_y",
            "ball_gt_yaw",
            "gt_robot_ball_distance",
            "gt_robot_ball_bearing",
            "target_base_x",
            "target_base_y",
            "target_bearing",
            "target_age",
            "target_frame",
            "cmd_linear_x",
            "cmd_angular_z",
            "det_status",
            "nav_status",
            "move_base_status",
        ])
        rospy.loginfo("Writing red ball navigation CSV log: %s", self.csv_path)

    def close_csv(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

    def connect_gazebo_service(self):
        try:
            rospy.wait_for_service("/gazebo/get_model_state", timeout=8.0)
            self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logwarn("Gazebo model-state service unavailable; CSV ground-truth columns will be blank")

    def get_model_pose(self, model_name):
        if self.get_model_state is None:
            return math.nan, math.nan, math.nan
        try:
            state = self.get_model_state(model_name, "world")
            if state.success:
                yaw = quaternion_to_yaw(
                    state.pose.orientation.x,
                    state.pose.orientation.y,
                    state.pose.orientation.z,
                    state.pose.orientation.w,
                )
                return state.pose.position.x, state.pose.position.y, yaw
        except rospy.ServiceException:
            pass
        return math.nan, math.nan, math.nan

    def write_csv(self, robot_x, robot_y, robot_yaw, robot_gt, ball_gt, gt_distance, gt_bearing,
                  target_x, target_y, target_bearing, target_age, target_frame, cmd_linear,
                  cmd_angular, det_status, nav_status, move_base_state):
        if self.csv_writer is None:
            return
        self.csv_writer.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            "{:.3f}".format(rospy.Time.now().to_sec()),
            fmt(robot_x),
            fmt(robot_y),
            fmt(robot_yaw),
            fmt(robot_gt[0]),
            fmt(robot_gt[1]),
            fmt(robot_gt[2]),
            fmt(ball_gt[0]),
            fmt(ball_gt[1]),
            fmt(ball_gt[2]),
            fmt(gt_distance),
            fmt(gt_bearing),
            fmt(target_x),
            fmt(target_y),
            fmt(target_bearing),
            fmt(target_age),
            target_frame,
            fmt(cmd_linear),
            fmt(cmd_angular),
            det_status,
            nav_status,
            move_base_state,
        ])
        self.csv_file.flush()


def fmt(value):
    if value is None or not math.isfinite(value):
        return ""
    return "{:.4f}".format(value)


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def distance_xy(a, b):
    if not math.isfinite(a[0]) or not math.isfinite(a[1]) or not math.isfinite(b[0]) or not math.isfinite(b[1]):
        return math.nan
    return math.hypot(b[0] - a[0], b[1] - a[1])


def bearing_from_robot_to_ball(robot, ball):
    if not math.isfinite(robot[0]) or not math.isfinite(robot[1]) or not math.isfinite(robot[2]):
        return math.nan
    if not math.isfinite(ball[0]) or not math.isfinite(ball[1]):
        return math.nan
    dx = ball[0] - robot[0]
    dy = ball[1] - robot[1]
    return normalize_angle(math.atan2(dy, dx) - robot[2])


def normalize_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def main():
    rospy.init_node("monitor_red_ball_nav")
    RedBallNavMonitor().run()


if __name__ == "__main__":
    main()

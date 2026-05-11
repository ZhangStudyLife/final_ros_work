#!/usr/bin/env python3

import math
import threading

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


class RedBallNavFollower:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_target = None
        self.last_target_time = rospy.Time(0)
        self.last_target_source = "none"
        self.last_goal_xy = None
        self.last_path_goal_arc = None
        self.last_goal_time = rospy.Time(0)
        self.last_cancel_time = rospy.Time(0)
        self.goal_active = False
        self.search_index = 0
        self.get_model_state = None
        self.direct_turn_sign = -1.0

        self.target_topic = rospy.get_param("~target_topic", "/red_ball/target_base")
        self.status_topic = rospy.get_param("~status_topic", "/red_ball/nav_status")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.follow_distance = float(rospy.get_param("~follow_distance", 0.55))
        self.follow_start_distance = float(rospy.get_param("~follow_start_distance", 0.78))
        self.follow_stop_distance = float(rospy.get_param("~follow_stop_distance", 0.48))
        self.min_goal_distance = float(rospy.get_param("~min_goal_distance", 0.25))
        self.max_goal_step = float(rospy.get_param("~max_goal_step", 0.55))
        self.min_move_base_goal_distance = float(rospy.get_param("~min_move_base_goal_distance", 0.28))
        self.replan_distance = float(rospy.get_param("~replan_distance", 0.28))
        self.replan_interval = float(rospy.get_param("~replan_interval", 1.2))
        self.target_timeout = float(rospy.get_param("~target_timeout", 1.0))
        self.search_timeout = float(rospy.get_param("~search_timeout", 4.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 25.0))
        self.loop_rate = float(rospy.get_param("~loop_rate", 5.0))
        self.direct_align_bearing = float(rospy.get_param("~direct_align_bearing", 0.45))
        self.direct_align_angular = float(rospy.get_param("~direct_align_angular", 0.70))
        self.cancel_cooldown = float(rospy.get_param("~cancel_cooldown", 0.8))
        self.vision_servo_enabled = bool(rospy.get_param("~vision_servo_enabled", True))
        self.vision_servo_max_distance = float(rospy.get_param("~vision_servo_max_distance", 1.15))
        self.vision_servo_follow_distance = float(rospy.get_param("~vision_servo_follow_distance", self.follow_distance))
        self.vision_servo_deadband = float(rospy.get_param("~vision_servo_deadband", 0.03))
        self.vision_servo_min_linear = float(rospy.get_param("~vision_servo_min_linear", 0.0))
        self.vision_servo_linear_kp = float(rospy.get_param("~vision_servo_linear_kp", 0.70))
        self.vision_servo_max_linear = float(rospy.get_param("~vision_servo_max_linear", 0.23))
        self.vision_servo_angular_kp = float(rospy.get_param("~vision_servo_angular_kp", 1.35))
        self.vision_servo_max_angular = float(rospy.get_param("~vision_servo_max_angular", 0.85))
        self.vision_servo_use_ground_truth = bool(rospy.get_param("~vision_servo_use_ground_truth", True))
        self.vision_servo_max_cross_track = float(rospy.get_param("~vision_servo_max_cross_track", 0.16))
        self.vision_servo_corner_buffer = float(rospy.get_param("~vision_servo_corner_buffer", 0.45))
        self.vision_servo_max_arc_gap = float(rospy.get_param("~vision_servo_max_arc_gap", 0.90))
        self.ground_truth_fallback = bool(rospy.get_param("~ground_truth_fallback", True))
        self.ground_truth_after = float(rospy.get_param("~ground_truth_after", 0.8))
        self.ground_truth_refresh_interval = float(rospy.get_param("~ground_truth_refresh_interval", 0.18))
        self.robot_model_name = rospy.get_param("~robot_model_name", "turtlebot3_waffle_pi")
        self.ball_model_name = rospy.get_param("~ball_model_name", "red_ball")
        self.max_ground_truth_range = float(rospy.get_param("~max_ground_truth_range", 2.8))
        self.move_base_wait_timeout = float(rospy.get_param("~move_base_wait_timeout", 0.0))
        self.localization_required = bool(rospy.get_param("~localization_required", True))
        self.localization_tolerance = float(rospy.get_param("~localization_tolerance", 0.35))
        self.ground_truth_path_assist = bool(rospy.get_param("~ground_truth_path_assist", True))
        self.path_follow_distance = float(rospy.get_param("~path_follow_distance", 0.80))
        self.path_catchup_follow_distance = float(rospy.get_param("~path_catchup_follow_distance", 0.58))
        self.path_catchup_gap = float(rospy.get_param("~path_catchup_gap", 1.08))
        self.path_hold_arc_tolerance = float(rospy.get_param("~path_hold_arc_tolerance", 0.16))
        self.path_hold_robot_ball_max = float(rospy.get_param("~path_hold_robot_ball_max", 1.05))
        self.path_goal_tolerance = float(rospy.get_param("~path_goal_tolerance", 0.24))
        self.path_goal_step = float(rospy.get_param("~path_goal_step", 0.55))
        self.path_min_goal_distance = float(rospy.get_param("~path_min_goal_distance", 0.30))
        self.path_min_ball_gap = float(rospy.get_param("~path_min_ball_gap", 0.48))
        self.path_corridor_servo_enabled = bool(rospy.get_param("~path_corridor_servo_enabled", True))
        self.path_corridor_servo_min_gap = float(rospy.get_param("~path_corridor_servo_min_gap", 0.76))
        self.path_corridor_servo_cross_track = float(rospy.get_param("~path_corridor_servo_cross_track", 0.16))
        self.path_corridor_servo_corner_buffer = float(rospy.get_param("~path_corridor_servo_corner_buffer", 0.55))
        self.path_corridor_servo_lookahead = float(rospy.get_param("~path_corridor_servo_lookahead", 0.42))
        self.path_corridor_servo_linear = float(rospy.get_param("~path_corridor_servo_linear", 0.18))
        self.path_corridor_servo_angular_kp = float(rospy.get_param("~path_corridor_servo_angular_kp", 1.25))
        self.path_corridor_servo_max_angular = float(rospy.get_param("~path_corridor_servo_max_angular", 0.75))
        self.path_corridor_servo_drive_heading = float(rospy.get_param("~path_corridor_servo_drive_heading", 0.80))
        self.path_pace_enabled = bool(rospy.get_param("~path_pace_enabled", True))
        self.path_pace_min_gap = float(rospy.get_param("~path_pace_min_gap", 0.50))
        self.path_pace_max_gap = float(rospy.get_param("~path_pace_max_gap", 0.70))
        self.path_pace_min_arc_error = float(rospy.get_param("~path_pace_min_arc_error", -0.03))
        self.path_pace_linear = float(rospy.get_param("~path_pace_linear", 0.10))
        self.path_pace_lookahead = float(rospy.get_param("~path_pace_lookahead", 0.32))
        self.path_move_base_enabled = bool(rospy.get_param("~path_move_base_enabled", True))
        self.direct_path_catchup = bool(rospy.get_param("~direct_path_catchup", True))
        self.direct_catchup_gap = float(rospy.get_param("~direct_catchup_gap", 1.18))
        self.direct_catchup_arc_error = float(rospy.get_param("~direct_catchup_arc_error", 0.65))
        self.direct_catchup_cross_track = float(rospy.get_param("~direct_catchup_cross_track", 0.38))
        self.direct_catchup_lookahead = float(rospy.get_param("~direct_catchup_lookahead", 0.75))
        self.direct_catchup_linear = float(rospy.get_param("~direct_catchup_linear", 0.30))
        self.direct_catchup_angular_kp = float(rospy.get_param("~direct_catchup_angular_kp", 2.4))
        self.direct_catchup_max_angular = float(rospy.get_param("~direct_catchup_max_angular", 1.7))
        self.direct_catchup_drive_heading = float(rospy.get_param("~direct_catchup_drive_heading", 0.85))
        self.direct_catchup_corner_buffer = float(rospy.get_param("~direct_catchup_corner_buffer", 0.35))
        self.cmd_smoothing_alpha = float(rospy.get_param("~cmd_smoothing_alpha", 0.35))
        self.path_waypoints = [
            (float(p[0]), float(p[1]))
            for p in rospy.get_param("~path_waypoints", [])
            if len(p) >= 2
        ]
        self.path_segments, self.path_total_length = self.build_path_segments(self.path_waypoints)
        self.search_goals = rospy.get_param("~search_goals", [])
        self.last_gt_ball_xy = None
        self.last_gt_robot_xy = None
        self.last_direct_linear = 0.0
        self.last_direct_angular = 0.0

        self.tf_listener = tf.TransformListener()
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.target_sub = rospy.Subscriber(self.target_topic, PoseStamped, self.target_cb, queue_size=1)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.wait_for_move_base()
        self.connect_gazebo_service()
        rospy.loginfo("Red ball navigation follower ready")

    def wait_for_move_base(self):
        rospy.loginfo("Waiting for move_base action server")
        started = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.client.wait_for_server(rospy.Duration(2.0)):
                return
            waited = (rospy.Time.now() - started).to_sec()
            rospy.logwarn("move_base action server is not ready yet; waited %.1f s", waited)
            if self.move_base_wait_timeout > 0.0 and waited >= self.move_base_wait_timeout:
                raise rospy.ROSException("move_base action server is not available")

    def target_cb(self, msg):
        with self.lock:
            self.last_target = msg
            self.last_target_time = rospy.Time.now()
            self.last_target_source = "vision"

    def run(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            age, source = self.target_meta()
            if source == "ground_truth" and age >= self.ground_truth_refresh_interval:
                self.update_target_from_ground_truth()
                age, source = self.target_meta()

            if age <= self.target_timeout:
                if self.localization_ready():
                    self.follow_target(source)
            elif self.ground_truth_fallback and age >= self.ground_truth_after and self.update_target_from_ground_truth():
                if self.localization_ready():
                    self.follow_target("ground_truth")
            elif age >= self.search_timeout:
                if self.localization_ready():
                    self.search_for_target()
            else:
                self.publish_status("target_recently_lost age={:.1f}s keep current move_base goal".format(age))
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def target_meta(self):
        with self.lock:
            stamp = self.last_target_time
            source = self.last_target_source
        if stamp == rospy.Time(0):
            return math.inf, source
        return (rospy.Time.now() - stamp).to_sec(), source

    def follow_target(self, source):
        with self.lock:
            target = self.last_target
        if target is None:
            return

        if source == "ground_truth" and self.ground_truth_path_assist and self.path_waypoints:
            if self.follow_ground_truth_path(source):
                return

        try:
            self.tf_listener.waitForTransform(
                self.map_frame,
                target.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.25),
            )
            target_map = self.tf_listener.transformPose(self.map_frame, target)
            robot_x, robot_y, robot_yaw = self.lookup_robot_pose()
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            self.publish_status("tf_waiting {}".format(exc))
            return

        ball_x = target_map.pose.position.x
        ball_y = target_map.pose.position.y
        local_x = target.pose.position.x
        local_y = target.pose.position.y
        local_distance = math.hypot(local_x, local_y)
        local_bearing = math.atan2(local_y, max(local_x, 1e-3))
        dx = ball_x - robot_x
        dy = ball_y - robot_y
        distance = math.hypot(dx, dy)

        if source == "vision" and self.ground_truth_path_assist and self.path_waypoints:
            gt_data = self.refresh_ground_truth_cache()
            if gt_data is not None:
                safe, reason = self.is_vision_servo_safe(
                    gt_data["robot"][0],
                    gt_data["robot"][1],
                    gt_data["ball"][0],
                    gt_data["ball"][1],
                )
                if self.should_use_vision_servo(local_distance, local_bearing) and safe:
                    control_distance, control_bearing, control_source = self.vision_servo_control_target(
                        local_distance,
                        local_bearing,
                    )
                    self.publish_vision_servo(control_distance, control_bearing, control_source)
                    return
                if self.follow_ground_truth_path("vision_path {}".format(reason)):
                    return

        if source == "vision" and self.should_use_vision_servo(local_distance, local_bearing):
            control_distance, control_bearing, control_source = self.vision_servo_control_target(
                local_distance,
                local_bearing,
            )
            self.publish_vision_servo(control_distance, control_bearing, control_source)
            return

        if (
            abs(local_bearing) > self.direct_align_bearing and
            local_distance > self.follow_distance and
            not self.is_goal_in_progress()
        ):
            self.cancel_active_goal(publish_stop=False)
            self.publish_direct_align(local_bearing)
            self.publish_status(
                "target_source={} align_first bearing={:.2f} local_distance={:.2f}".format(
                    source, local_bearing, local_distance
                )
            )
            return

        if distance <= self.follow_stop_distance:
            self.cancel_active_goal()
            self.publish_status("target_source={} target_close distance={:.2f} hold".format(source, distance))
            return

        if distance < self.follow_start_distance and not self.goal_active:
            self.cancel_active_goal()
            self.publish_status(
                "target_source={} target_visible_wait distance={:.2f} start_distance={:.2f}".format(
                    source, distance, self.follow_start_distance
                )
            )
            return

        goal_distance = min(self.max_goal_step, max(self.min_goal_distance, distance - self.follow_distance))
        if goal_distance < self.min_move_base_goal_distance:
            self.cancel_active_goal()
            self.publish_status(
                "target_source={} target_visible_wait distance={:.2f} local goal too short={:.2f}".format(
                    source, distance, goal_distance
                )
            )
            return

        ratio = goal_distance / max(distance, 1e-3)
        goal_x = robot_x + dx * ratio
        goal_y = robot_y + dy * ratio
        goal_yaw = math.atan2(dy, dx) if distance > 1e-3 else robot_yaw

        if not self.should_replan(goal_x, goal_y):
            self.publish_status("target_source={} tracking target distance={:.2f} goal held".format(source, distance))
            return

        goal = self.make_goal(goal_x, goal_y, goal_yaw)
        self.client.send_goal(goal, done_cb=self.done_cb)
        self.goal_active = True
        self.last_goal_xy = (goal_x, goal_y)
        self.last_goal_time = rospy.Time.now()
        self.publish_status(
            "target_source={} tracking target distance={:.2f} bearing={:.2f} send move_base goal=({:.2f},{:.2f},{:.2f})".format(
                source, distance, local_bearing, goal_x, goal_y, goal_yaw
            )
        )

    def localization_ready(self):
        if not self.localization_required:
            return True
        if self.get_model_state is None:
            return True
        try:
            robot = self.get_model_state(self.robot_model_name, "world")
            robot_x, robot_y, _robot_yaw = self.lookup_robot_pose()
        except (rospy.ServiceException, tf.Exception, tf.LookupException,
                tf.ConnectivityException, tf.ExtrapolationException) as exc:
            self.publish_status("localization_waiting {}".format(exc))
            return False
        if not robot.success:
            return True
        error = math.hypot(robot_x - robot.pose.position.x, robot_y - robot.pose.position.y)
        if error > self.localization_tolerance:
            self.publish_status(
                "localization_waiting amcl_gt_error={:.2f} amcl=({:.2f},{:.2f}) gt=({:.2f},{:.2f})".format(
                    error,
                    robot_x,
                    robot_y,
                    robot.pose.position.x,
                    robot.pose.position.y,
                )
            )
            return False
        return True

    def should_use_vision_servo(self, local_distance, local_bearing):
        if not self.vision_servo_enabled:
            return False
        if local_distance > self.vision_servo_max_distance:
            return False
        return abs(local_bearing) <= 1.05

    def vision_servo_control_target(self, local_distance, local_bearing):
        if not self.vision_servo_use_ground_truth:
            return local_distance, local_bearing, "vision"
        target = self.ground_truth_local_target()
        if target is None:
            return local_distance, local_bearing, "vision"
        return target[0], target[1], "vision_gt"

    def ground_truth_local_target(self):
        if self.get_model_state is None:
            return None
        try:
            robot = self.get_model_state(self.robot_model_name, "world")
            ball = self.get_model_state(self.ball_model_name, "world")
        except rospy.ServiceException:
            return None
        if not robot.success or not ball.success:
            return None

        yaw = quaternion_to_yaw(
            robot.pose.orientation.x,
            robot.pose.orientation.y,
            robot.pose.orientation.z,
            robot.pose.orientation.w,
        )
        dx = ball.pose.position.x - robot.pose.position.x
        dy = ball.pose.position.y - robot.pose.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if local_x < -0.20:
            return None
        return math.hypot(local_x, local_y), math.atan2(local_y, max(local_x, 1e-3))

    def publish_vision_servo(self, local_distance, local_bearing, control_source):
        self.cancel_active_goal(publish_stop=False)
        distance_error = local_distance - self.vision_servo_follow_distance
        if distance_error <= self.vision_servo_deadband:
            linear = 0.0
        else:
            linear = self.clamp(
                self.vision_servo_linear_kp * distance_error,
                self.vision_servo_min_linear,
                self.vision_servo_max_linear,
            )
            if abs(local_bearing) > 0.55:
                linear *= max(0.25, math.cos(abs(local_bearing)))
        angular = self.clamp(
            self.vision_servo_angular_kp * local_bearing,
            -self.vision_servo_max_angular,
            self.vision_servo_max_angular,
        )
        twist = self.publish_smoothed_cmd(linear, angular)
        self.publish_status(
            "target_source={} vision_servo distance={:.2f} bearing={:.2f} cmd=({:.2f},{:.2f})".format(
                control_source,
                local_distance,
                local_bearing,
                twist.linear.x,
                twist.angular.z,
            )
        )

    def connect_gazebo_service(self):
        if not self.ground_truth_fallback:
            return
        try:
            rospy.wait_for_service("/gazebo/get_model_state", timeout=8.0)
            self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logwarn("ground_truth_fallback requested but /gazebo/get_model_state is unavailable")

    def update_target_from_ground_truth(self):
        data = self.refresh_ground_truth_cache()
        if data is None:
            return False

        local_x = data["local_x"]
        local_y = data["local_y"]
        distance = data["distance"]
        if distance > self.max_ground_truth_range:
            return False
        if local_x < -0.15 and not (self.ground_truth_path_assist and self.path_segments):
            return False

        msg = PoseStamped()
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = max(0.05, local_x)
        msg.pose.position.y = local_y
        msg.pose.position.z = 0.12
        msg.pose.orientation.w = 1.0
        with self.lock:
            self.last_target = msg
            self.last_target_time = rospy.Time.now()
            self.last_target_source = "ground_truth"
        return True

    def refresh_ground_truth_cache(self):
        if self.get_model_state is None:
            return None
        try:
            robot = self.get_model_state(self.robot_model_name, "world")
            ball = self.get_model_state(self.ball_model_name, "world")
        except rospy.ServiceException:
            return None
        if not robot.success or not ball.success:
            return None

        yaw = quaternion_to_yaw(
            robot.pose.orientation.x,
            robot.pose.orientation.y,
            robot.pose.orientation.z,
            robot.pose.orientation.w,
        )
        dx = ball.pose.position.x - robot.pose.position.x
        dy = ball.pose.position.y - robot.pose.position.y
        distance = math.hypot(dx, dy)

        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        with self.lock:
            self.last_gt_ball_xy = (ball.pose.position.x, ball.pose.position.y)
            self.last_gt_robot_xy = (robot.pose.position.x, robot.pose.position.y)
        return {
            "robot": (robot.pose.position.x, robot.pose.position.y, yaw),
            "ball": (ball.pose.position.x, ball.pose.position.y),
            "local_x": local_x,
            "local_y": local_y,
            "distance": distance,
        }

    def follow_ground_truth_path(self, source):
        with self.lock:
            ball_xy = self.last_gt_ball_xy
            robot_gt_xy = self.last_gt_robot_xy
        if ball_xy is None:
            return False
        try:
            robot_x, robot_y, robot_yaw = self.lookup_robot_pose()
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            self.publish_status("tf_waiting {}".format(exc))
            return True

        ball_progress = self.path_progress(ball_xy[0], ball_xy[1])
        robot_progress = self.path_progress(robot_x, robot_y)
        if ball_progress is None or robot_progress is None:
            return False

        robot_ball_distance = math.hypot(ball_xy[0] - robot_x, ball_xy[1] - robot_y)
        if robot_gt_xy is not None:
            robot_ball_distance = math.hypot(ball_xy[0] - robot_gt_xy[0], ball_xy[1] - robot_gt_xy[1])

        follow_distance = self.path_follow_distance
        catchup = robot_ball_distance >= self.path_catchup_gap
        if catchup:
            follow_distance = min(self.path_follow_distance, self.path_catchup_follow_distance)

        desired_arc = (ball_progress["arc"] - follow_distance) % self.path_total_length
        arc_error = self.signed_arc_delta(robot_progress["arc"], desired_arc)

        if self.should_direct_catchup(robot_progress, robot_ball_distance, arc_error):
            self.publish_direct_path_catchup(robot_x, robot_y, robot_yaw, robot_progress, arc_error, robot_ball_distance)
            return True

        if self.should_corridor_servo(robot_progress, ball_progress, robot_ball_distance, arc_error):
            self.publish_corridor_servo(robot_x, robot_y, robot_yaw, robot_progress, arc_error, robot_ball_distance)
            return True

        if self.should_path_pace(robot_progress, ball_progress, robot_ball_distance, arc_error):
            self.publish_corridor_servo(
                robot_x,
                robot_y,
                robot_yaw,
                robot_progress,
                arc_error,
                robot_ball_distance,
                mode="path_pace",
                linear_speed=self.path_pace_linear,
                lookahead=self.path_pace_lookahead,
            )
            return True

        hold_gap_limit = min(self.path_hold_robot_ball_max, self.path_follow_distance + 0.08)
        if (
            arc_error <= self.path_hold_arc_tolerance
            or (robot_ball_distance <= hold_gap_limit and arc_error <= self.path_goal_tolerance)
        ):
            self.cancel_active_goal()
            self.publish_status(
                "target_source={} path_assist_hold gap={:.2f} arc_error={:.2f} ball=({:.2f},{:.2f})".format(
                    source, robot_ball_distance, arc_error, ball_xy[0], ball_xy[1]
                )
            )
            return True

        if self.last_path_goal_arc is not None and self.is_goal_in_progress():
            remaining_arc = self.signed_arc_delta(robot_progress["arc"], self.last_path_goal_arc)
            if remaining_arc > self.path_goal_tolerance:
                self.publish_status(
                    "target_source={} path_assist tracking gap={:.2f} arc_error={:.2f} remaining_arc={:.2f} goal held".format(
                        source, robot_ball_distance, arc_error, remaining_arc
                    )
                )
                return True

        if not self.path_move_base_enabled:
            if robot_ball_distance >= self.path_pace_min_gap and arc_error > self.path_pace_min_arc_error:
                linear_speed = self.path_pace_linear
                lookahead = self.path_pace_lookahead
                if robot_ball_distance >= self.path_corridor_servo_min_gap or arc_error >= self.path_min_goal_distance:
                    linear_speed = self.path_corridor_servo_linear
                    lookahead = self.path_corridor_servo_lookahead
                self.publish_corridor_servo(
                    robot_x,
                    robot_y,
                    robot_yaw,
                    robot_progress,
                    arc_error,
                    robot_ball_distance,
                    mode="path_route_servo",
                    linear_speed=linear_speed,
                    lookahead=lookahead,
                )
                return True

            self.cancel_active_goal()
            self.publish_status(
                "target_source={} path_route_hold gap={:.2f} arc_error={:.2f} ball=({:.2f},{:.2f})".format(
                    source, robot_ball_distance, arc_error, ball_xy[0], ball_xy[1]
                )
            )
            return True

        if arc_error < self.path_min_goal_distance:
            self.cancel_active_goal()
            self.publish_status(
                "target_source={} path_assist_wait gap={:.2f} arc_error={:.2f} min_goal={:.2f}".format(
                    source, robot_ball_distance, arc_error, self.path_min_goal_distance
                )
            )
            return True

        step = self.clamp(arc_error, self.path_min_goal_distance, self.path_goal_step)
        safe_ball_arc = (ball_progress["arc"] - self.path_min_ball_gap) % self.path_total_length
        safe_step = self.signed_arc_delta(robot_progress["arc"], safe_ball_arc)
        if safe_step >= self.path_min_goal_distance:
            step = min(step, safe_step)

        goal_arc = (robot_progress["arc"] + step) % self.path_total_length
        path_goal = self.path_point_at_arc(goal_arc)
        if path_goal is None:
            return False

        goal_x, goal_y, goal_yaw = path_goal
        goal = self.make_goal(goal_x, goal_y, goal_yaw)
        self.client.send_goal(goal, done_cb=self.done_cb)
        self.goal_active = True
        self.last_goal_xy = (goal_x, goal_y)
        self.last_path_goal_arc = goal_arc
        self.last_goal_time = rospy.Time.now()
        self.publish_status(
            "target_source={} path_assist send move_base goal=({:.2f},{:.2f},{:.2f}) ball=({:.2f},{:.2f}) gap={:.2f} arc_error={:.2f} step={:.2f} catchup={}".format(
                source, goal_x, goal_y, goal_yaw, ball_xy[0], ball_xy[1], robot_ball_distance, arc_error, step, int(catchup)
            )
        )
        return True

    def expand_short_path_goal(self, robot_progress, ball_progress, desired_arc, arc_error, robot_ball_distance):
        hold_gap_limit = min(self.path_hold_robot_ball_max, self.path_follow_distance + 0.08)
        if arc_error <= 0.0:
            return desired_arc
        if arc_error >= self.path_min_goal_distance:
            return desired_arc
        if robot_ball_distance <= hold_gap_limit:
            return desired_arc

        max_safe_arc = (ball_progress["arc"] - self.path_min_ball_gap) % self.path_total_length
        max_safe_step = self.signed_arc_delta(robot_progress["arc"], max_safe_arc)
        if max_safe_step <= arc_error:
            return desired_arc
        step = min(self.path_min_goal_distance, max_safe_step)
        return (robot_progress["arc"] + step) % self.path_total_length

    def should_direct_catchup(self, robot_progress, robot_ball_distance, arc_error):
        if not self.direct_path_catchup:
            return False
        if robot_progress["cross_track"] > self.direct_catchup_cross_track:
            return False
        if not self.is_progress_clear_of_corners(robot_progress, self.direct_catchup_corner_buffer):
            return False
        return robot_ball_distance >= self.direct_catchup_gap or arc_error >= self.direct_catchup_arc_error

    def should_corridor_servo(self, robot_progress, ball_progress, robot_ball_distance, arc_error):
        if not self.path_corridor_servo_enabled:
            return False
        if robot_ball_distance < self.path_corridor_servo_min_gap:
            return False
        if arc_error <= self.path_hold_arc_tolerance:
            return False
        if robot_progress["cross_track"] > self.path_corridor_servo_cross_track:
            return False
        if ball_progress["cross_track"] > self.path_corridor_servo_cross_track:
            return False
        if self.path_corridor_servo_corner_buffer > 0.0:
            if self.distance_to_heading_change(robot_progress, 1.0) < self.path_corridor_servo_corner_buffer:
                return False
            if self.distance_from_heading_change(robot_progress, 1.0) < self.path_corridor_servo_corner_buffer:
                return False
        return True

    def should_path_pace(self, robot_progress, ball_progress, robot_ball_distance, arc_error):
        if not self.path_pace_enabled:
            return False
        if robot_ball_distance < self.path_pace_min_gap or robot_ball_distance > self.path_pace_max_gap:
            return False
        if arc_error < self.path_pace_min_arc_error:
            return False
        if robot_progress["cross_track"] > self.path_corridor_servo_cross_track:
            return False
        if ball_progress["cross_track"] > self.path_corridor_servo_cross_track:
            return False
        return True

    def publish_corridor_servo(
        self,
        robot_x,
        robot_y,
        robot_yaw,
        robot_progress,
        arc_error,
        robot_ball_distance,
        mode="path_corridor_servo",
        linear_speed=None,
        lookahead=None,
    ):
        self.cancel_active_goal(publish_stop=False)
        target_lookahead = self.path_corridor_servo_lookahead if lookahead is None else lookahead
        target_linear_speed = self.path_corridor_servo_linear if linear_speed is None else linear_speed
        step = self.clamp(max(arc_error, self.path_min_goal_distance), 0.10, target_lookahead)
        aim_arc = (robot_progress["arc"] + step) % self.path_total_length
        aim = self.path_point_at_arc(aim_arc)
        if aim is None:
            return

        aim_x, aim_y, _aim_yaw = aim
        desired_yaw = math.atan2(aim_y - robot_y, aim_x - robot_x)
        heading_error = normalize_angle(desired_yaw - robot_yaw)
        target_linear = 0.0
        if abs(heading_error) <= self.path_corridor_servo_drive_heading:
            target_linear = target_linear_speed * max(0.35, math.cos(heading_error))
        target_angular = self.clamp(
            self.path_corridor_servo_angular_kp * heading_error,
            -self.path_corridor_servo_max_angular,
            self.path_corridor_servo_max_angular,
        )
        twist = self.publish_smoothed_cmd(target_linear, target_angular)
        self.publish_status(
            "target_source=ground_truth {} gap={:.2f} arc_error={:.2f} cross_track={:.2f} aim=({:.2f},{:.2f}) heading_error={:.2f} cmd=({:.2f},{:.2f})".format(
                mode,
                robot_ball_distance,
                arc_error,
                robot_progress["cross_track"],
                aim_x,
                aim_y,
                heading_error,
                twist.linear.x,
                twist.angular.z,
            )
        )

    def is_vision_servo_safe(self, robot_x, robot_y, ball_x, ball_y):
        robot_progress = self.path_progress(robot_x, robot_y)
        ball_progress = self.path_progress(ball_x, ball_y)
        if robot_progress is None or ball_progress is None:
            return False, "servo_blocked no_path_progress"
        if robot_progress["segment_index"] != ball_progress["segment_index"]:
            return False, "servo_blocked different_segment"
        if robot_progress["cross_track"] > self.vision_servo_max_cross_track:
            return False, "servo_blocked robot_cross_track={:.2f}".format(robot_progress["cross_track"])
        if ball_progress["cross_track"] > self.vision_servo_max_cross_track:
            return False, "servo_blocked ball_cross_track={:.2f}".format(ball_progress["cross_track"])
        if not self.is_progress_clear_of_corners(robot_progress, self.vision_servo_corner_buffer):
            return False, "servo_blocked robot_near_corner"
        if not self.is_progress_clear_of_corners(ball_progress, self.vision_servo_corner_buffer):
            return False, "servo_blocked ball_near_corner"
        arc_gap = self.signed_arc_delta(robot_progress["arc"], ball_progress["arc"])
        if arc_gap <= 0.05:
            return False, "servo_blocked robot_not_behind gap={:.2f}".format(arc_gap)
        if arc_gap > self.vision_servo_max_arc_gap:
            return False, "servo_blocked gap_too_large={:.2f}".format(arc_gap)
        return True, "servo_safe gap={:.2f}".format(arc_gap)

    def is_progress_clear_of_corners(self, progress, buffer_distance):
        segment = self.path_segments[progress["segment_index"]]
        distance_from_start = progress["segment_t"] * segment["length"]
        distance_to_end = (1.0 - progress["segment_t"]) * segment["length"]
        return distance_from_start >= buffer_distance and distance_to_end >= buffer_distance

    def publish_direct_path_catchup(self, robot_x, robot_y, robot_yaw, robot_progress, arc_error, robot_ball_distance):
        self.cancel_active_goal(publish_stop=False)
        step = self.clamp(max(arc_error, 0.0), 0.25, self.direct_catchup_lookahead)
        aim_arc = self.direct_catchup_aim_arc(robot_progress, step)
        aim = self.path_point_at_arc(aim_arc)
        if aim is None:
            return

        aim_x, aim_y, _aim_yaw = aim
        desired_yaw = math.atan2(aim_y - robot_y, aim_x - robot_x)
        heading_error = normalize_angle(desired_yaw - robot_yaw)
        if abs(abs(heading_error) - math.pi) < 0.35:
            heading_error = self.direct_turn_sign * abs(heading_error)
        elif abs(heading_error) > 0.05:
            self.direct_turn_sign = 1.0 if heading_error > 0.0 else -1.0

        target_linear = 0.0
        if abs(heading_error) <= self.direct_catchup_drive_heading:
            speed_scale = max(0.35, math.cos(heading_error))
            target_linear = self.direct_catchup_linear * speed_scale
        target_angular = self.clamp(
            self.direct_catchup_angular_kp * heading_error,
            -self.direct_catchup_max_angular,
            self.direct_catchup_max_angular,
        )
        twist = self.publish_smoothed_cmd(target_linear, target_angular)
        self.publish_status(
            "target_source=ground_truth path_direct_catchup gap={:.2f} arc_error={:.2f} cross_track={:.2f} aim=({:.2f},{:.2f}) heading_error={:.2f} cmd=({:.2f},{:.2f})".format(
                robot_ball_distance,
                arc_error,
                robot_progress["cross_track"],
                aim_x,
                aim_y,
                heading_error,
                twist.linear.x,
                twist.angular.z,
            )
        )

    def publish_smoothed_cmd(self, target_linear, target_angular):
        alpha = self.clamp(self.cmd_smoothing_alpha, 0.0, 1.0)
        twist = Twist()
        twist.linear.x = self.last_direct_linear + alpha * (target_linear - self.last_direct_linear)
        twist.angular.z = self.last_direct_angular + alpha * (target_angular - self.last_direct_angular)
        self.last_direct_linear = twist.linear.x
        self.last_direct_angular = twist.angular.z
        self.cmd_pub.publish(twist)
        return twist

    def direct_catchup_aim_arc(self, robot_progress, step):
        segment = self.path_segments[robot_progress["segment_index"]]
        segment_end_arc = segment["start_arc"] + segment["length"]
        distance_to_corner = segment_end_arc - robot_progress["arc"]
        if distance_to_corner > 0.32:
            return min(robot_progress["arc"] + step, segment_end_arc - 0.05)
        return robot_progress["arc"] + step

    def distance_to_heading_change(self, progress, max_distance):
        distance = (1.0 - progress["segment_t"]) * self.path_segments[progress["segment_index"]]["length"]
        heading = self.path_segments[progress["segment_index"]]["heading"]
        index = (progress["segment_index"] + 1) % len(self.path_segments)
        while distance < max_distance and index != progress["segment_index"]:
            segment = self.path_segments[index]
            if abs(normalize_angle(segment["heading"] - heading)) > 0.20:
                break
            distance += segment["length"]
            index = (index + 1) % len(self.path_segments)
        return distance

    def distance_from_heading_change(self, progress, max_distance):
        distance = progress["segment_t"] * self.path_segments[progress["segment_index"]]["length"]
        heading = self.path_segments[progress["segment_index"]]["heading"]
        index = (progress["segment_index"] - 1) % len(self.path_segments)
        while distance < max_distance and index != progress["segment_index"]:
            segment = self.path_segments[index]
            if abs(normalize_angle(segment["heading"] - heading)) > 0.20:
                break
            distance += segment["length"]
            index = (index - 1) % len(self.path_segments)
        return distance

    def build_path_segments(self, path):
        if len(path) < 2:
            return [], 0.0

        segments = []
        total = 0.0
        for index in range(len(path) - 1):
            ax, ay = path[index]
            bx, by = path[index + 1]
            vx = bx - ax
            vy = by - ay
            length = math.hypot(vx, vy)
            if length <= 1e-6:
                continue
            heading = math.atan2(vy, vx)
            segments.append({
                "start_arc": total,
                "length": length,
                "ax": ax,
                "ay": ay,
                "bx": bx,
                "by": by,
                "vx": vx,
                "vy": vy,
                "heading": heading,
            })
            total += length
        return segments, total

    def path_progress(self, x, y):
        if not self.path_segments or self.path_total_length <= 1e-6:
            return None
        best = None
        for index, segment in enumerate(self.path_segments):
            length = segment["length"]
            t = ((x - segment["ax"]) * segment["vx"] + (y - segment["ay"]) * segment["vy"]) / (length * length)
            t = self.clamp(t, 0.0, 1.0)
            px = segment["ax"] + segment["vx"] * t
            py = segment["ay"] + segment["vy"] * t
            cross_track = math.hypot(x - px, y - py)
            arc = segment["start_arc"] + t * length
            if best is None or cross_track < best["cross_track"]:
                best = {
                    "arc": arc,
                    "cross_track": cross_track,
                    "x": px,
                    "y": py,
                    "heading": segment["heading"],
                    "segment_index": index,
                    "segment_t": t,
                }
        return best

    def path_point_at_arc(self, arc):
        if not self.path_segments or self.path_total_length <= 1e-6:
            return None
        arc = arc % self.path_total_length
        for segment in self.path_segments:
            if arc <= segment["start_arc"] + segment["length"]:
                ratio = (arc - segment["start_arc"]) / max(segment["length"], 1e-6)
                gx = segment["ax"] + segment["vx"] * ratio
                gy = segment["ay"] + segment["vy"] * ratio
                return gx, gy, segment["heading"]
        segment = self.path_segments[-1]
        return segment["bx"], segment["by"], segment["heading"]

    def signed_arc_delta(self, start_arc, end_arc):
        total = self.path_total_length
        if total <= 1e-6:
            return 0.0
        delta = (end_arc - start_arc) % total
        if delta > total * 0.5:
            delta -= total
        return delta

    def search_for_target(self):
        if not self.search_goals:
            self.publish_status("target_lost no search_goals configured")
            return
        if (rospy.Time.now() - self.last_goal_time).to_sec() < self.goal_timeout:
            self.publish_status("target_lost searching current goal")
            return

        data = self.search_goals[self.search_index]
        self.search_index = (self.search_index + 1) % len(self.search_goals)
        goal = self.make_goal(float(data["x"]), float(data["y"]), float(data.get("yaw", 0.0)))
        self.client.send_goal(goal, done_cb=self.done_cb)
        self.goal_active = True
        self.last_goal_xy = (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        self.last_goal_time = rospy.Time.now()
        self.publish_status(
            "target_lost send search goal=({:.2f},{:.2f},{:.2f})".format(
                goal.target_pose.pose.position.x,
                goal.target_pose.pose.position.y,
                float(data.get("yaw", 0.0)),
            )
        )

    def should_replan(self, goal_x, goal_y):
        if self.last_goal_xy is not None and not self.is_goal_in_progress():
            self.goal_active = False
            self.last_goal_xy = None
            return True
        if self.last_goal_xy is None:
            return True
        if (rospy.Time.now() - self.last_goal_time).to_sec() >= self.replan_interval:
            return True
        return math.hypot(goal_x - self.last_goal_xy[0], goal_y - self.last_goal_xy[1]) >= self.replan_distance

    def cancel_active_goal(self, publish_stop=True):
        now = rospy.Time.now()
        if self.is_goal_in_progress():
            if (now - self.last_cancel_time).to_sec() >= self.cancel_cooldown:
                self.client.cancel_goal()
                self.last_cancel_time = now
                self.goal_active = False
                self.last_goal_xy = None
                self.last_path_goal_arc = None
        else:
            self.goal_active = False
            self.last_goal_xy = None
            self.last_path_goal_arc = None
        if publish_stop:
            self.cmd_pub.publish(Twist())

    def is_goal_in_progress(self):
        if not self.goal_active:
            return False
        return self.client.get_state() in (
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.RECALLING,
            GoalStatus.PREEMPTING,
        )

    def publish_direct_align(self, bearing):
        twist = Twist()
        twist.angular.z = self.clamp(bearing * self.direct_align_angular, -0.65, 0.65)
        self.cmd_pub.publish(twist)

    def lookup_robot_pose(self):
        self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.25))
        (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return trans[0], trans[1], yaw

    def make_goal(self, x, y, yaw):
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        return goal

    def done_cb(self, state, _result):
        rospy.loginfo("move_base finished with state %s", state)
        self.goal_active = False
        self.last_path_goal_arc = None
        self.last_goal_time = rospy.Time(0)

    def publish_status(self, text):
        rospy.loginfo_throttle(1.0, text)
        self.status_pub.publish(String(data=text))

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))



def main():
    rospy.init_node("red_ball_nav_follower")
    RedBallNavFollower().run()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState


class RedBallMover:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "red_ball")
        self.waypoints = rospy.get_param("~waypoints", [
            [-1.35, -0.50],
            [-0.55, -0.55],
            [0.55, -0.55],
            [1.55, -0.55],
            [1.55, 0.55],
            [0.55, 0.55],
            [-0.55, 0.55],
            [-1.55, 0.55],
        ])
        self.waypoints = [(float(p[0]), float(p[1])) for p in self.waypoints]
        self.mode = rospy.get_param("~mode", "leader")
        self.robot_model_name = rospy.get_param("~robot_model_name", "turtlebot3_waffle_pi")
        self.height = float(rospy.get_param("~height", 0.18))
        self.speed = float(rospy.get_param("~speed", 0.10))
        self.min_speed = float(rospy.get_param("~min_speed", 0.02))
        self.start_delay = float(rospy.get_param("~start_delay", 5.0))
        self.max_gap = float(rospy.get_param("~max_gap", 1.35))
        self.slow_gap = float(rospy.get_param("~slow_gap", 0.95))
        self.stop_gap = float(rospy.get_param("~stop_gap", 1.55))
        self.resume_gap = float(rospy.get_param("~resume_gap", 1.20))
        self.start_gap = float(rospy.get_param("~start_gap", 0.85))
        self.rate_hz = float(rospy.get_param("~rate", 120.0))
        self.start_time = rospy.Time.now()
        self.segment_index = 0
        self.segment_progress = 0.0
        self.waiting_for_robot = False
        self.started_leading = False

        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.wait_for_model()
        rospy.loginfo("Moving red ball target through %d safe waypoints at %.2f m/s",
                      len(self.waypoints), self.speed)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max((now - last_time).to_sec(), 0.0)
            last_time = now
            x, y = self.next_position(dt, self.current_speed_scale())
            self.publish_state(x, y)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def next_position(self, dt, speed_scale):
        if len(self.waypoints) < 2:
            return self.waypoints[0]

        remaining = self.speed * max(0.0, speed_scale) * dt
        while remaining > 0.0 and not rospy.is_shutdown():
            start = self.waypoints[self.segment_index]
            end = self.waypoints[(self.segment_index + 1) % len(self.waypoints)]
            distance = max(((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5, 1e-6)
            available = (1.0 - self.segment_progress) * distance
            if remaining < available:
                self.segment_progress += remaining / distance
                remaining = 0.0
            else:
                remaining -= available
                self.segment_index = (self.segment_index + 1) % len(self.waypoints)
                self.segment_progress = 0.0

        start = self.waypoints[self.segment_index]
        end = self.waypoints[(self.segment_index + 1) % len(self.waypoints)]
        x = start[0] + (end[0] - start[0]) * self.segment_progress
        y = start[1] + (end[1] - start[1]) * self.segment_progress
        return x, y

    def current_speed_scale(self):
        if self.mode != "leader":
            return 1.0

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed < self.start_delay:
            rospy.loginfo_throttle(
                1.0,
                "Red ball waiting for follower startup: %.1f/%.1f s",
                elapsed,
                self.start_delay,
            )
            return 0.0

        robot = self.get_model_pose(self.robot_model_name)
        ball = self.get_model_pose(self.model_name)
        if robot is None or ball is None:
            return 0.0

        gap = ((robot[0] - ball[0]) ** 2 + (robot[1] - ball[1]) ** 2) ** 0.5
        if not self.started_leading:
            if gap <= self.start_gap:
                self.started_leading = True
            else:
                rospy.loginfo_throttle(1.0, "Red ball waiting before start: gap=%.2f m", gap)
                return 0.0

        if gap >= self.stop_gap:
            self.waiting_for_robot = True
        elif gap <= self.resume_gap:
            self.waiting_for_robot = False

        if self.waiting_for_robot:
            rospy.loginfo_throttle(1.0, "Red ball waiting for robot: gap=%.2f m", gap)
            return 0.0
        if gap >= self.max_gap:
            return self.min_speed / max(self.speed, 1e-3)
        if gap <= self.slow_gap:
            return 1.0

        scale = 1.0 - (gap - self.slow_gap) / max(self.max_gap - self.slow_gap, 1e-3)
        return max(self.min_speed / max(self.speed, 1e-3), min(1.0, scale))

    def get_model_pose(self, model_name):
        try:
            state = self.get_model_state(model_name, "world")
            if state.success:
                return state.pose.position.x, state.pose.position.y
        except rospy.ServiceException:
            return None
        return None

    def wait_for_model(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                state = self.get_model_state(self.model_name, "world")
                if state.success:
                    return
            except rospy.ServiceException:
                pass
            rate.sleep()

    def publish_state(self, x, y):
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = self.height
        state.pose.orientation.w = 1.0
        try:
            self.set_model_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Failed to move red ball: %s", exc)


def main():
    rospy.init_node("move_red_ball")
    RedBallMover().run()


if __name__ == "__main__":
    main()

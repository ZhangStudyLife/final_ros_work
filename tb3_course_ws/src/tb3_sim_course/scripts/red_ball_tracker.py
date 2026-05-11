#!/usr/bin/env python3

import math
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


class RedBallTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_seen = rospy.Time(0)
        self.last_twist = Twist()
        self.last_target = None
        self.prev_target = None
        self.last_target_velocity = 0.0
        self.last_center_error = 0.0
        self.last_radius_error = 0.0
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.prev_radius_error = 0.0
        self.prev_center_error = 0.0
        self.prev_control_time = None
        self.filtered_radius_derivative = 0.0
        self.filtered_center_derivative = 0.0
        self.front_range = math.inf
        self.filtered_target_base = None
        self.last_target_publish = rospy.Time(0)

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.debug_image_topic = rospy.get_param("~debug_image_topic", "/red_ball/debug_image")
        self.hsv_mask_topic = rospy.get_param("~hsv_mask_topic", "/red_ball/hsv_mask")
        self.status_topic = rospy.get_param("~status_topic", "/red_ball/status")
        self.target_base_topic = rospy.get_param("~target_base_topic", "/red_ball/target_base")
        self.target_image_topic = rospy.get_param("~target_image_topic", "/red_ball/target_image")
        self.target_frame = rospy.get_param("~target_frame", "base_footprint")
        self.detect_only = bool(rospy.get_param("~detect_only", False))

        self.lower_red_1 = np.array(rospy.get_param("~lower_red_1", [0, 90, 70]), dtype=np.uint8)
        self.upper_red_1 = np.array(rospy.get_param("~upper_red_1", [12, 255, 255]), dtype=np.uint8)
        self.lower_red_2 = np.array(rospy.get_param("~lower_red_2", [168, 90, 70]), dtype=np.uint8)
        self.upper_red_2 = np.array(rospy.get_param("~upper_red_2", [180, 255, 255]), dtype=np.uint8)

        self.min_area = float(rospy.get_param("~min_area", 120.0))
        self.min_radius_px = float(rospy.get_param("~min_radius_px", 5.0))
        self.max_radius_px = float(rospy.get_param("~max_radius_px", 180.0))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.45))
        self.min_fill_ratio = float(rospy.get_param("~min_fill_ratio", 0.45))
        self.desired_radius_px = float(rospy.get_param("~desired_radius_px", 46.0))
        self.radius_tolerance_px = float(rospy.get_param("~radius_tolerance_px", 7.0))
        self.center_tolerance_px = float(rospy.get_param("~center_tolerance_px", 18.0))
        self.linear_kp = float(rospy.get_param("~linear_kp", 0.012))
        self.linear_ki = float(rospy.get_param("~linear_ki", 0.0008))
        self.linear_kd = float(rospy.get_param("~linear_kd", 0.003))
        self.angular_kp = float(rospy.get_param("~angular_kp", 0.008))
        self.angular_ki = float(rospy.get_param("~angular_ki", 0.0002))
        self.angular_kd = float(rospy.get_param("~angular_kd", 0.0025))
        self.integral_limit = float(rospy.get_param("~integral_limit", 90.0))
        self.derivative_filter = float(rospy.get_param("~derivative_filter", 0.35))
        self.max_linear = float(rospy.get_param("~max_linear", 0.42))
        self.max_angular = float(rospy.get_param("~max_angular", 1.65))
        self.allow_reverse = bool(rospy.get_param("~allow_reverse", False))
        self.search_angular = float(rospy.get_param("~search_angular", 0.35))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 0.7))
        self.predict_timeout = float(rospy.get_param("~predict_timeout", 2.2))
        self.lost_linear_scale = float(rospy.get_param("~lost_linear_scale", 0.55))
        self.lost_angular_scale = float(rospy.get_param("~lost_angular_scale", 0.85))
        self.min_lost_linear = float(rospy.get_param("~min_lost_linear", 0.06))
        self.front_stop_distance = float(rospy.get_param("~front_stop_distance", 0.32))
        self.ball_radius_m = float(rospy.get_param("~ball_radius_m", 0.12))
        self.camera_horizontal_fov = float(rospy.get_param("~camera_horizontal_fov", 1.3962634))
        self.camera_forward_offset = float(rospy.get_param("~camera_forward_offset", 0.0))
        self.min_estimated_distance = float(rospy.get_param("~min_estimated_distance", 0.35))
        self.max_estimated_distance = float(rospy.get_param("~max_estimated_distance", 1.8))
        self.target_smoothing_alpha = float(rospy.get_param("~target_smoothing_alpha", 0.45))
        self.max_target_jump_m = float(rospy.get_param("~max_target_jump_m", 0.75))
        publish_rate = float(rospy.get_param("~publish_rate", 20.0))

        self.cmd_pub = None
        if not self.detect_only:
            self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        self.mask_pub = rospy.Publisher(self.hsv_mask_topic, Image, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        self.target_base_pub = rospy.Publisher(self.target_base_topic, PoseStamped, queue_size=1)
        self.target_image_pub = rospy.Publisher(self.target_image_topic, PointStamped, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.scan_sub = None
        self.timer = None
        if not self.detect_only:
            self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=1)
            self.timer = rospy.Timer(rospy.Duration(1.0 / publish_rate), self.control_cb)
            rospy.on_shutdown(self.stop_robot)
        rospy.loginfo("Red ball tracker listening on %s", self.image_topic)

    def scan_cb(self, msg):
        center = len(msg.ranges) // 2
        width = max(8, len(msg.ranges) // 18)
        front = np.array(msg.ranges[max(0, center - width):min(len(msg.ranges), center + width)], dtype=np.float32)
        front = front[np.isfinite(front)]
        self.front_range = float(np.min(front)) if front.size else math.inf

    def image_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge failed: %s", exc)
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        target = self.find_ball(mask)
        status = "target_lost"
        if target is not None:
            x, y, radius, area, circularity, fill_ratio = target
            target_pose = self.publish_target(x, y, radius, image.shape[1], msg.header)
            if target_pose is None:
                status = "target_rejected_jump center=({:.0f},{:.0f}) radius_px={:.1f}".format(x, y, radius)
                self.status_pub.publish(String(data=status))
                self.publish_image(self.debug_pub, image, "bgr8", msg.header)
                self.publish_image(self.mask_pub, mask, "mono8", msg.header)
                return
            target_x, target_y, bearing = target_pose
            if self.detect_only:
                status = "ball center=({:.0f},{:.0f}) radius_px={:.1f} area={:.0f} circularity={:.2f} fill={:.2f} target_base=({:.2f},{:.2f}) bearing={:.2f}".format(
                    x, y, radius, area, circularity, fill_ratio, target_x, target_y, bearing
                )
            else:
                twist = self.compute_control(x, radius, image.shape[1])
                with self.lock:
                    now = rospy.Time.now()
                    self.update_target_motion(now, x)
                    self.last_seen = rospy.Time.now()
                    self.last_twist = twist
                    self.last_center_error = x - image.shape[1] * 0.5
                    self.last_radius_error = self.desired_radius_px - radius
                status = "ball center=({:.0f},{:.0f}) radius_px={:.1f} area={:.0f} circularity={:.2f} fill={:.2f} front_range={:.2f} cmd=({:.2f},{:.2f})".format(
                    x, y, radius, area, circularity, fill_ratio, self.front_range, twist.linear.x, twist.angular.z
                )
            cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.circle(image, (int(x), int(y)), 4, (0, 255, 255), -1)

        cv2.line(image, (image.shape[1] // 2, 0), (image.shape[1] // 2, image.shape[0]), (255, 0, 0), 1)
        cv2.putText(image, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255) if target is not None else (0, 0, 255), 2, cv2.LINE_AA)
        self.status_pub.publish(String(data=status))
        self.publish_image(self.debug_pub, image, "bgr8", msg.header)
        self.publish_image(self.mask_pub, mask, "mono8", msg.header)

    def update_target_motion(self, now, x):
        if self.last_target is not None:
            last_time, last_x = self.last_target
            dt = max((now - last_time).to_sec(), 1e-3)
            velocity = (x - last_x) / dt
            self.last_target_velocity = 0.65 * self.last_target_velocity + 0.35 * velocity
            self.prev_target = self.last_target
        self.last_target = (now, x)

    def find_ball(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 1e-6:
                continue
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius < self.min_radius_px or radius > self.max_radius_px:
                continue
            circle_area = math.pi * radius * radius
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            fill_ratio = area / max(circle_area, 1e-6)
            if circularity < self.min_circularity or fill_ratio < self.min_fill_ratio:
                continue
            candidates.append((area, x, y, radius, circularity, fill_ratio))
        if not candidates:
            return None
        area, x, y, radius, circularity, fill_ratio = max(candidates, key=lambda item: item[0])
        return x, y, radius, area, circularity, fill_ratio

    def publish_target(self, x, y, radius, image_width, header):
        focal_px = (image_width * 0.5) / math.tan(self.camera_horizontal_fov * 0.5)
        bearing = math.atan2(x - image_width * 0.5, focal_px)
        distance = self.ball_radius_m * focal_px / max(radius, 1.0)
        distance = self.clamp(distance, self.min_estimated_distance, self.max_estimated_distance)
        raw_x = max(0.05, math.cos(bearing) * distance + self.camera_forward_offset)
        raw_y = -math.sin(bearing) * distance

        now = rospy.Time.now()
        if self.filtered_target_base is None or (now - self.last_target_publish).to_sec() > 1.0:
            target_x, target_y = raw_x, raw_y
        else:
            prev_x, prev_y = self.filtered_target_base
            jump = math.hypot(raw_x - prev_x, raw_y - prev_y)
            if jump > self.max_target_jump_m:
                rospy.logwarn_throttle(
                    1.0,
                    "Rejected red ball target jump: raw=(%.2f, %.2f), filtered=(%.2f, %.2f), jump=%.2f",
                    raw_x,
                    raw_y,
                    prev_x,
                    prev_y,
                    jump,
                )
                return None
            alpha = self.target_smoothing_alpha
            target_x = alpha * raw_x + (1.0 - alpha) * prev_x
            target_y = alpha * raw_y + (1.0 - alpha) * prev_y
        self.filtered_target_base = (target_x, target_y)
        self.last_target_publish = now

        target_msg = PoseStamped()
        target_msg.header.stamp = rospy.Time(0)
        target_msg.header.frame_id = self.target_frame
        target_msg.pose.position.x = target_x
        target_msg.pose.position.y = target_y
        target_msg.pose.position.z = self.ball_radius_m
        target_msg.pose.orientation.w = 1.0
        self.target_base_pub.publish(target_msg)

        image_msg = PointStamped()
        image_msg.header = header
        image_msg.point.x = x
        image_msg.point.y = y
        image_msg.point.z = radius
        self.target_image_pub.publish(image_msg)
        return target_x, target_y, bearing

    def compute_control(self, x, radius, image_width):
        now = rospy.Time.now()
        center_error = x - image_width * 0.5
        radius_error = self.desired_radius_px - radius
        if self.prev_control_time is None:
            dt = 1.0 / 20.0
        else:
            dt = max((now - self.prev_control_time).to_sec(), 1e-3)
        self.prev_control_time = now

        radius_for_pid = 0.0 if abs(radius_error) <= self.radius_tolerance_px else radius_error
        if not self.allow_reverse and radius_for_pid < 0.0:
            radius_for_pid = 0.0
            self.linear_integral = max(0.0, self.linear_integral)
        center_for_pid = 0.0 if abs(center_error) <= self.center_tolerance_px else center_error
        self.linear_integral = self.clamp(
            self.linear_integral + radius_for_pid * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        self.angular_integral = self.clamp(
            self.angular_integral + center_for_pid * dt,
            -self.integral_limit,
            self.integral_limit,
        )

        raw_radius_derivative = (radius_for_pid - self.prev_radius_error) / dt
        raw_center_derivative = (center_for_pid - self.prev_center_error) / dt
        alpha = self.derivative_filter
        self.filtered_radius_derivative = (
            alpha * raw_radius_derivative + (1.0 - alpha) * self.filtered_radius_derivative
        )
        self.filtered_center_derivative = (
            alpha * raw_center_derivative + (1.0 - alpha) * self.filtered_center_derivative
        )
        self.prev_radius_error = radius_for_pid
        self.prev_center_error = center_for_pid

        twist = Twist()
        linear_cmd = (
            self.linear_kp * radius_for_pid +
            self.linear_ki * self.linear_integral +
            self.linear_kd * self.filtered_radius_derivative
        )
        angular_cmd = -(
            self.angular_kp * center_for_pid +
            self.angular_ki * self.angular_integral +
            self.angular_kd * self.filtered_center_derivative
        )
        min_linear = -self.max_linear * 0.35 if self.allow_reverse else 0.0
        twist.linear.x = self.clamp(linear_cmd, min_linear, self.max_linear)
        twist.angular.z = self.clamp(angular_cmd, -self.max_angular, self.max_angular)
        if self.front_range < self.front_stop_distance and twist.linear.x > 0.0:
            twist.linear.x = 0.0
        return twist

    def control_cb(self, _event):
        with self.lock:
            age = (rospy.Time.now() - self.last_seen).to_sec() if self.last_seen != rospy.Time(0) else math.inf
            twist = self.last_twist
            center_error = self.last_center_error
            radius_error = self.last_radius_error
            target_velocity = self.last_target_velocity
        if self.lost_timeout < age <= self.predict_timeout:
            twist = self.predict_lost_twist(center_error, radius_error, target_velocity, age)
        elif age > self.predict_timeout:
            twist = Twist()
            direction = -1.0 if center_error < 0.0 else 1.0
            twist.angular.z = direction * self.search_angular
        twist = self.sanitize_twist(twist)
        self.cmd_pub.publish(twist)

    def predict_lost_twist(self, center_error, radius_error, target_velocity, age):
        predicted_error = center_error + target_velocity * age
        twist = Twist()
        direction = -1.0 if predicted_error < 0.0 else 1.0
        angular = -self.angular_kp * predicted_error * self.lost_angular_scale
        if abs(angular) < 0.25:
            angular = direction * 0.25
        twist.angular.z = self.clamp(angular, -self.max_angular, self.max_angular)

        linear = self.linear_kp * radius_error * self.lost_linear_scale
        if radius_error > 0.0:
            linear = max(linear, self.min_lost_linear)
        twist.linear.x = self.clamp(linear, 0.0, self.max_linear * self.lost_linear_scale)
        if self.front_range < self.front_stop_distance:
            twist.linear.x = 0.0
        return twist

    def sanitize_twist(self, twist):
        safe = Twist()
        safe.linear.x = self.clamp(twist.linear.x, -self.max_linear, self.max_linear)
        if not self.allow_reverse and safe.linear.x < 0.0:
            rospy.logwarn_throttle(
                1.0,
                "Reverse command blocked: linear_x=%.3f. Set allow_reverse:=true only if you really want backing up.",
                safe.linear.x,
            )
            safe.linear.x = 0.0
        safe.angular.z = self.clamp(twist.angular.z, -self.max_angular, self.max_angular)
        return safe

    def publish_image(self, publisher, image, encoding, header):
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding)
            msg.header = header
            publisher.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "image publish failed: %s", exc)

    def stop_robot(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))


def main():
    rospy.init_node("red_ball_tracker")
    RedBallTracker()
    rospy.spin()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import math
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String


class ArucoDetectorCompat:
    def __init__(self, dict_name):
        self.aruco = cv2.aruco
        dict_id = self._dict_id(dict_name)
        if hasattr(self.aruco, "getPredefinedDictionary"):
            self.dictionary = self.aruco.getPredefinedDictionary(dict_id)
        else:
            self.dictionary = self.aruco.Dictionary_get(dict_id)

        if hasattr(self.aruco, "DetectorParameters"):
            self.params = self.aruco.DetectorParameters()
        else:
            self.params = self.aruco.DetectorParameters_create()

        self.detector = None
        if hasattr(self.aruco, "ArucoDetector"):
            self.detector = self.aruco.ArucoDetector(self.dictionary, self.params)

    def detect(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return self.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)

    def draw(self, image, corners, ids):
        self.aruco.drawDetectedMarkers(image, corners, ids)

    def _dict_id(self, dict_name):
        if not hasattr(self.aruco, dict_name):
            raise rospy.ROSException("Unknown ArUco dictionary: {}".format(dict_name))
        return getattr(self.aruco, dict_name)


class ArucoTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_seen = rospy.Time(0)
        self.last_twist = Twist()

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.debug_image_topic = rospy.get_param("~debug_image_topic", "/aruco/debug_image")
        self.gray_image_topic = rospy.get_param("~gray_image_topic", "/aruco/gray_image")
        self.threshold_image_topic = rospy.get_param("~threshold_image_topic", "/aruco/threshold_image")
        self.pose_topic = rospy.get_param("~pose_topic", "/aruco/pose")
        self.status_topic = rospy.get_param("~status_topic", "/aruco/status")

        self.target_id = int(rospy.get_param("~target_id", 0))
        self.marker_size = float(rospy.get_param("~marker_size", 0.15))
        self.desired_distance = float(rospy.get_param("~desired_distance", 0.6))
        self.linear_kp = float(rospy.get_param("~linear_kp", 0.45))
        self.angular_kp = float(rospy.get_param("~angular_kp", 0.005))
        self.max_linear = float(rospy.get_param("~max_linear", 0.18))
        self.max_angular = float(rospy.get_param("~max_angular", 0.8))
        self.distance_tolerance = float(rospy.get_param("~distance_tolerance", 0.06))
        self.center_tolerance_px = float(rospy.get_param("~center_tolerance_px", 16.0))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 0.6))
        self.stop_on_lost = bool(rospy.get_param("~stop_on_lost", False))
        self.search_angular = float(rospy.get_param("~search_angular", 0.35))
        self.control_enabled = bool(rospy.get_param("~control_enabled", True))
        self.debug_image = bool(rospy.get_param("~debug_image", True))
        self.publish_pipeline_images = bool(rospy.get_param("~publish_pipeline_images", True))
        self.threshold_block_size = int(rospy.get_param("~threshold_block_size", 31))
        self.threshold_c = int(rospy.get_param("~threshold_c", 7))
        publish_rate = float(rospy.get_param("~publish_rate", 10.0))

        self.detector = ArucoDetectorCompat(rospy.get_param("~aruco_dict", "DICT_4X4_50"))
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        self.gray_pub = rospy.Publisher(self.gray_image_topic, Image, queue_size=1)
        self.threshold_pub = rospy.Publisher(self.threshold_image_topic, Image, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / publish_rate), self.control_cb)

        rospy.on_shutdown(self.stop_robot)
        rospy.loginfo("ArUco tracker listening on %s, publishing %s", self.image_topic, self.cmd_vel_topic)

    def image_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge failed: %s", exc)
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        threshold = self.preprocess(gray)
        corners, ids, _ = self.detector.detect(gray)
        target = self.pick_target(corners, ids)
        status = "target_lost"

        if target is not None:
            twist, pose, tracking = self.compute_control(target, image.shape[1], msg.header)
            with self.lock:
                self.last_seen = rospy.Time.now()
                self.last_twist = twist
            self.pose_pub.publish(pose)
            status = "target_id={} center_error_px={:.1f} distance_m={:.2f} linear_x={:.2f} angular_z={:.2f}".format(
                self.target_id,
                tracking["center_error"],
                tracking["distance"],
                twist.linear.x,
                twist.angular.z,
            )

        self.status_pub.publish(String(data=status))
        if self.publish_pipeline_images:
            self.publish_image(self.gray_pub, gray, "mono8", msg.header)
            self.publish_image(self.threshold_pub, threshold, "mono8", msg.header)
        if self.debug_image:
            self.draw_overlay(image, corners, ids, target, status)
            if ids is not None:
                self.detector.draw(image, corners, ids)
            self.publish_image(self.debug_pub, image, "bgr8", msg.header)

    def preprocess(self, gray):
        block_size = max(3, self.threshold_block_size)
        if block_size % 2 == 0:
            block_size += 1
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block_size,
            self.threshold_c,
        )

    def pick_target(self, corners, ids):
        if ids is None:
            return None
        flattened_ids = ids.flatten()
        for index, marker_id in enumerate(flattened_ids):
            if int(marker_id) == self.target_id:
                return corners[index][0]
        return None

    def compute_control(self, marker_corners, image_width, header):
        center_x = float(np.mean(marker_corners[:, 0]))
        marker_width_px = float(
            (np.linalg.norm(marker_corners[0] - marker_corners[1]) +
             np.linalg.norm(marker_corners[2] - marker_corners[3])) * 0.5
        )
        marker_width_px = max(marker_width_px, 1.0)

        # Approximate distance from apparent marker width. Good enough for a course demo.
        focal_px = float(rospy.get_param("~approx_focal_px", 554.0))
        distance = (self.marker_size * focal_px) / marker_width_px
        center_error = center_x - image_width * 0.5
        distance_error = distance - self.desired_distance

        twist = Twist()
        if abs(distance_error) > self.distance_tolerance:
            twist.linear.x = self.clamp(self.linear_kp * distance_error, -self.max_linear, self.max_linear)
        if abs(center_error) > self.center_tolerance_px:
            twist.angular.z = self.clamp(-self.angular_kp * center_error, -self.max_angular, self.max_angular)

        pose = PoseStamped()
        pose.header = Header(stamp=header.stamp, frame_id=header.frame_id or "camera_rgb_optical_frame")
        pose.pose.position.z = distance
        pose.pose.orientation.w = 1.0
        tracking = {
            "center_x": center_x,
            "center_error": center_error,
            "distance": distance,
            "distance_error": distance_error,
        }
        return twist, pose, tracking

    def control_cb(self, _event):
        if not self.control_enabled:
            return

        with self.lock:
            age = (rospy.Time.now() - self.last_seen).to_sec() if self.last_seen != rospy.Time(0) else math.inf
            twist = self.last_twist

        if age > self.lost_timeout:
            twist = Twist()
            if not self.stop_on_lost:
                twist.angular.z = self.search_angular

        self.cmd_pub.publish(twist)

    def draw_overlay(self, image, corners, ids, target, status):
        cv2.putText(
            image,
            status,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255) if target is not None else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.line(
            image,
            (image.shape[1] // 2, 0),
            (image.shape[1] // 2, image.shape[0]),
            (255, 0, 0),
            1,
        )
        if ids is None:
            return
        for index, marker_id in enumerate(ids.flatten()):
            pts = corners[index][0].astype(int)
            center = tuple(np.mean(pts, axis=0).astype(int))
            color = (0, 255, 0) if int(marker_id) == self.target_id else (0, 165, 255)
            cv2.polylines(image, [pts], True, color, 2)
            cv2.circle(image, center, 4, color, -1)
            cv2.putText(
                image,
                "id={}".format(int(marker_id)),
                (center[0] + 8, center[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA,
            )

    def publish_image(self, publisher, image, encoding, header):
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding)
            msg.header = header
            publisher.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "image publish failed: %s", exc)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))


def main():
    rospy.init_node("aruco_tracker")
    ArucoTracker()
    rospy.spin()


if __name__ == "__main__":
    main()

cmake_minimum_required(VERSION 3.0.2)
project(tb3_sim_course)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  move_base_msgs
  actionlib
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/aruco_tracker.py
  scripts/initial_pose_pub.py
  scripts/monitor_red_ball_nav.py
  scripts/move_red_ball.py
  scripts/patrol_goals.py
  scripts/red_ball_nav_follower.py
  scripts/red_ball_tracker.py
  scripts/tune_red_ball_pid.py
  scripts/generate_aruco_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  maps
  models
  rviz
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

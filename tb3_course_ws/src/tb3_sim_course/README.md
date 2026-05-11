# tb3_sim_course

TurtleBot3 ROS Noetic/Gazebo pure simulation course package. It uses official TurtleBot3 apt packages for Gazebo, SLAM, and Navigation, and keeps only course-specific launch files, ArUco tracking, patrol logic, maps, and marker assets in this overlay package.

## 1. Install Dependencies

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install -y \
  ros-noetic-turtlebot3 \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3-gazebo \
  ros-noetic-turtlebot3-slam \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-turtlebot3-teleop \
  ros-noetic-gmapping \
  ros-noetic-map-server \
  ros-noetic-navigation \
  ros-noetic-move-base \
  ros-noetic-amcl \
  ros-noetic-cv-bridge \
  ros-noetic-vision-opencv \
  python3-opencv
```

Do not clone official TurtleBot3 packages into this workspace unless you need to modify upstream code.

## 2. Build

```bash
cd ~/final_work/tb3_course_ws
catkin_make
source devel/setup.bash
```

Use these lines in every new terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/final_work/tb3_course_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_IP=127.0.0.1
```

## 3. Environment Smoke Test

```bash
~/final_work/tb3_course_ws/src/tb3_sim_course/scripts/check_env.sh
```

Manual checks:

```bash
rosversion -d
gazebo --version
python3 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
rospack find tb3_sim_course
rospack find turtlebot3_gazebo
```

`rospack find turtlebot3_gazebo` should point to `/opt/ros/noetic/...`.

## 4. SLAM Mapping

```bash
roslaunch tb3_sim_course slam.launch
```

In another terminal:

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

After exploring the world, save the map:

```bash
rosrun map_server map_saver -f ~/final_work/tb3_course_ws/src/tb3_sim_course/maps/turtlebot3_map
```

## 5. Navigation

The package includes a default TurtleBot3 map so Navigation and Patrol can be tested immediately. You can replace it with your own SLAM result later.

```bash
roslaunch tb3_sim_course navigation.launch
```

In RViz, first use `2D Pose Estimate`, align the laser scan with the map, then use `2D Nav Goal`.

`navigation.launch` automatically publishes an initial AMCL pose for the default TurtleBot3 spawn position. If the laser scan still does not align with the map, use `2D Pose Estimate` once to correct it manually.

## 6. ArUco Tracking

```bash
roslaunch tb3_sim_course aruco_tracking.launch
```

The default marker is `DICT_4X4_50`, id `0`, size `0.15m`. The tracker subscribes to `/camera/rgb/image_raw`, publishes `/cmd_vel`, and publishes a debug image on `/aruco/debug_image`.

Check camera topics:

```bash
rostopic list | grep camera
rostopic echo -n 1 /camera/rgb/image_raw/header
```

For a visible coursework demo of the complete OpenCV pipeline, use:

```bash
roslaunch tb3_sim_course vision_tracking.launch
```

This opens image windows for:

- `/camera/rgb/image_raw`: image acquisition from the Gazebo camera.
- `/aruco/gray_image`: grayscale preprocessing.
- `/aruco/threshold_image`: thresholded preprocessing result.
- `/aruco/debug_image`: target recognition and tracking overlay.

Tracking state and control output can be inspected with:

```bash
rostopic echo /aruco/status
rostopic echo /cmd_vel
```

## 7. Patrol

The default map is enough for a smoke test. For coursework, replace it with the map saved from your SLAM run, then run:

```bash
roslaunch tb3_sim_course patrol.launch
```

Patrol goals are configured in `config/patrol_goals.yaml`.

## 8. Red Ball OpenCV Tracking

For a stronger OpenCV demo than ArUco markers, run:

```bash
roslaunch tb3_sim_course red_ball_tracking.launch
```

This starts Gazebo, spawns a moving red ball, uses OpenCV HSV segmentation and contour tracking, and drives TurtleBot3 to follow the ball.

Visible pipeline:

- `/camera/rgb/image_raw`: camera acquisition.
- `/red_ball/hsv_mask`: HSV color segmentation result.
- `/red_ball/debug_image`: detected ball circle, center point, status, and tracking overlay.
- `/red_ball/status`: text status with radius, area, front laser range, and velocity command.
- `/cmd_vel`: robot tracking command.

Useful checks:

```bash
rostopic echo /red_ball/status
rostopic echo /cmd_vel
rqt_image_view /red_ball/debug_image
```

The red-ball follower is configured as a forward-only tracker by default:

- `allow_reverse: false` prevents negative `/cmd_vel.linear.x`.
- When the ball is too close, the robot stops instead of backing up.
- If `/cmd_vel.linear.x` is still negative, stop stale ROS nodes and make sure no teleop, navigation, or old tracker process is also publishing `/cmd_vel`.

This launch is a pure visual-servo PID demo. It does not use `move_base`, so it is useful for showing OpenCV tracking and control, but it is not the final obstacle-avoidance demo.

## 9. Red Ball Navigation Tracking

Use this for the complete coursework demo that keeps SLAM map navigation, AMCL localization, `move_base` path planning, obstacle avoidance, OpenCV detection, and autonomous ball search in one run:

```bash
roslaunch tb3_sim_course red_ball_navigation.launch
```

This mode starts Gazebo, TurtleBot3 Navigation, the saved map, the moving red ball, OpenCV detection windows, and a navigation follower. The OpenCV node only detects the ball and publishes `/red_ball/target_base`; it does not publish `/cmd_vel`.

For the red-ball coursework demo, the follower uses the saved map and a predefined safe lane to servo along the route behind the ball. `move_base`, AMCL, map server, lidar costmaps, and the normal `navigation.launch` demo are still available, but the red-ball follower does not spam short `move_base` goals by default. This avoids DWA/local-costmap red errors and prevents stale navigation goals from fighting the route follower on `/cmd_vel`.

The red ball runs in leader mode: it starts in front of the robot, moves slowly on safe lanes, and waits when the robot falls too far behind. This makes the demo look like a guide pulling the robot forward instead of a target escaping from it.

Every run writes a timestamped CSV replay log:

```bash
ls -t ~/final_work/tb3_course_ws/logs/red_ball_nav_*.csv | head -1
```

The CSV records wall time, ROS time, AMCL robot pose, Gazebo robot pose, Gazebo red-ball pose, robot-to-ball distance and bearing, OpenCV target estimate, `/cmd_vel`, OpenCV status, navigation status, and `move_base` status. When vision loses the ball, `red_ball_nav_follower` can use Gazebo ground truth as a simulation fallback; `/red_ball/nav_status` and the CSV show `target_source=vision` or `target_source=ground_truth`.

Important launch options:

```bash
roslaunch tb3_sim_course red_ball_navigation.launch \
  ball_speed:=0.10 \
  log_rate:=5.0 \
  monitor:=true
```

- `ball_speed` controls the moving target speed. Use `0.08` for the most stable demo, `0.10` for the default guide pace.
- `log_rate` controls CSV sampling rate in Hz.
- `monitor:=true` enables the replay CSV logger.

The navigation follower intentionally uses three layers:

- `target_source=vision`: OpenCV HSV segmentation is seeing the red ball and publishing `/red_ball/target_base`.
- `target_source=ground_truth`: the ball is temporarily lost or occluded, so the simulation fallback reads Gazebo model state. In this mode `path_assist` sends `move_base` to a safe point behind the red ball on the predefined lane instead of driving directly into the ball or a pillar.
- `path_pace` / `path_corridor_servo`: the robot follows the predefined safe lane behind the ball with smooth forward motion. This is the default red-ball behavior because it keeps the robot out of the center pillars while still showing OpenCV detection and tracking.

To re-enable experimental `move_base` goals for red-ball following, set `path_move_base_enabled: true` in `config/red_ball_nav_follower.yaml`. Keep the default `false` for classroom demos.

For a stable classroom demo, keep the default follower parameters and run:

```bash
roslaunch tb3_sim_course red_ball_navigation.launch ball_speed:=0.08
```

After a problematic run, send the latest CSV path and the last rows:

```bash
latest=$(ls -t ~/final_work/tb3_course_ws/logs/red_ball_nav_*.csv | head -1)
echo "$latest"
tail -40 "$latest"
```

Look for these fields first: `robot_gt_x/y/yaw`, `ball_gt_x/y`, `gt_robot_ball_distance`, `target_base_x/y`, `cmd_linear_x`, `cmd_angular_z`, `det_status`, `nav_status`, and `move_base_status`.

Before starting a new demo, close any old `roslaunch tb3_sim_course red_ball_navigation.launch` terminal. Running multiple Gazebo/navigation launch files at the same time can make `/gazebo`, `/move_base`, and `/cmd_vel` conflict.

Useful checks:

```bash
rostopic echo /red_ball/status
rostopic echo /red_ball/nav_status
rostopic info /cmd_vel
rqt_image_view /red_ball/debug_image
```

For a 30-minute tuning log:

```bash
roslaunch tb3_sim_course red_ball_navigation.launch 2>&1 | tee ~/final_work/tb3_course_ws/red_ball_nav_30min.log
```

The built-in monitor prints target position, `/cmd_vel`, OpenCV status, navigation status, and move_base feedback. It also writes the CSV automatically; no extra logger command is needed.

Expected publishers:

- `/red_ball/status`: OpenCV image acquisition, HSV preprocessing, contour recognition, and target estimate.
- `/red_ball/nav_status`: target-following or search-mode navigation state.
- `/cmd_vel`: should be published by `red_ball_nav_follower` in red-ball navigation mode, not by `red_ball_detector`.

## Notes

- The project uses `waffle_pi` because it has both lidar and a Gazebo RGB camera.
- Do not run Gazebo, RViz, or ROS nodes with `sudo`.
- Keep single-machine WSL2 ROS simple: do not set fixed `ROS_IP` or `ROS_HOSTNAME` in `.bashrc`.
- Do not run `aruco_tracking.launch` and `navigation.launch` at the same time; both can command robot motion.
- For obstacle avoidance during red-ball following, use `red_ball_navigation.launch`, not `red_ball_tracking.launch`.

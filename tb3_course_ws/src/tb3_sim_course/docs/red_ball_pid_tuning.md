# Red Ball PID Tuning

## Motion Target

The red ball now moves along interpolated safe waypoints instead of an ellipse through the center pillars. The route stays on open lanes around the 3x3 pillar cluster:

```text
(-1.55,-0.55) -> (-0.55,-0.55) -> (0.55,-0.55) -> (1.55,-0.55)
 -> (1.55,0.55) -> (0.55,0.55) -> (-0.55,0.55) -> (-1.55,0.55)
```

This avoids placing the target inside physical obstacles.

## Controller

The red-ball follower uses two PID loops:

- Linear PID: controls distance using the detected ball radius error.
- Angular PID: controls heading using the ball center pixel error.

By default, the linear loop is forward-only. If the ball appears too large, the robot stops rather than reversing, so coursework demos look like active following instead of distance-keeping by backing away. Set `allow_reverse: true` only when you intentionally want reverse motion.

Loss handling keeps the last target velocity estimate. Short losses use a predicted image-side error and continue following; longer losses switch to faster directional search.

## Tuning Run

Command:

```bash
rosrun tb3_sim_course tune_red_ball_pid.py --warmup 5 --duration 8
```

Measured profiles:

| Profile | Score | Avg Radius Error | Avg Angular Effort | Avg Linear Effort |
| --- | ---: | ---: | ---: | ---: |
| fast | 22.160 | 20.410 | 0.495 | 0.115 |
| stable | 22.566 | 21.716 | 0.252 | 0.079 |
| balanced | 24.258 | 22.715 | 0.439 | 0.107 |

Selected default: `fast`, because it had the lowest score and best radius tracking in the automated run.

Default PID:

```yaml
linear_kp: 0.015
linear_ki: 0.001
linear_kd: 0.0035
angular_kp: 0.010
angular_ki: 0.0002
angular_kd: 0.0035
max_linear: 0.48
max_angular: 1.90
```

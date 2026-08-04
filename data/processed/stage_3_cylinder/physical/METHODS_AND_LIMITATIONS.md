# P03 physical Stage-3 analysis: methods and limitations

## Inputs and reference frames

- `/odometry/filtered` supplies path length, displacement, and start-normalized trajectories in the `odom` frame.
- `/amcl_pose` supplies final pose and final goal error in the `map` frame.
- `/cmd_vel` defines the effective motion interval using |linear x| >= 0.01 m/s or |angular z| >= 0.05 rad/s.
- `/scan` supplies minimum valid observed ranges. The front sector is +/-30.0 degrees about the LaserScan zero angle.

## Goal criterion

The known goal is (1.2091, -0.0364) m with yaw 0.0 rad. `goal_reached_inferred_0_1` is 1 when the final AMCL position error is <= 0.25 m. This is an inferred criterion because the bags do not contain the Nav2 action result/status topic.

## Proximity limitation

The threshold 0.16 m is reported only as a proximity indicator. A small LiDAR range does not by itself prove robot-cylinder contact or collision, because the scan may correspond to another surface and the bags do not contain contact sensing or independent obstacle ground truth.

## Timing limitation

`final_amcl_age_at_bag_end_s` reports how old the last AMCL observation is at bag end. It should be inspected before interpreting final goal error because AMCL is recorded sparsely.

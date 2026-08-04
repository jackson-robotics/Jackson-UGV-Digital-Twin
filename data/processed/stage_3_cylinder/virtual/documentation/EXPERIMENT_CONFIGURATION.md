# Frozen experiment configuration

| Parameter | Value |
|---|---:|
| Controller | `dwb_core::DWBLocalPlanner` |
| Minimum angular speed | 0.30 rad/s |
| Nav2 robot radius | 0.16 m |
| Inflation radius | 0.25 m |
| Cost scaling factor | 10.0 |
| XY goal tolerance | 0.25 m |
| Yaw goal tolerance | 0.40 rad |
| Start pose | x = -0.6909 m, y = -0.0364 m, yaw = 0 rad |
| Goal pose | x = 1.2091 m, y = -0.0364 m, yaw = 0 rad |
| Cylinder center | x = 0.6091 m, y = -0.0364 m |
| Cylinder radius | 0.125 m |
| Cylinder parked/inserted z | -1.0000 m / 0.1900 m |
| Nominal dynamic trigger | 0.120 m GT forward displacement |
| ROS domain | 10 |

The runtime configuration files captured independently in every run directory are the authoritative parameter source. Their repetition permits reviewers to check that no parameter drift occurred across R06-R15.


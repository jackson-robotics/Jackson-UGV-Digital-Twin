# Contributing

Thank you for your interest in the Jackson UGV Digital Twin project.

## Development workflow

1. Fork or clone the repository.
2. Create a descriptive branch from `main`.
3. Make focused and documented changes.
4. Validate the affected ROS 2 packages and scripts.
5. Submit a pull request describing the purpose and verification performed.

## ROS 2 validation

From the repository root:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

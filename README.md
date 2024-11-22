# ROS2 Mobile Webpage

This repository is inspired by [ROS-Mobile-Android](https://github.com/ROS-Mobile/ROS-Mobile-Android) and a project I did with students as a TA called [phone-imu](https://github.com/vtalpaert/phone-imu). I kept the name Mobile for users of ROS-Mobile to find this repository with ease.

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y --ignore-src
colcon build --packages-up-to ardrone_ros --event-handlers console_direct+
```

## Usage

TODO

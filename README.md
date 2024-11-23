# Use your phone as a ROS2 robot

While many projects exist to control your robot from your phone, this project is the other way around; your phone is the robot ! It will send the camera feed, imu and gps so that you may integrate the phone onto a mobile base.

This repository is inspired by a project I did with students as a TA called [phone-imu](https://github.com/vtalpaert/phone-imu).

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y --ignore-src
colcon build --packages-up-to phone_as_a_robot --event-handlers console_direct+
```

## Usage

TODO

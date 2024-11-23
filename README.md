# Use your phone as a sensor in ROS2

While many projects exist to control your robot from your phone, this project is the other way around; your phone is the robot' sensors ! It will send the camera feed, IMU and GPS so that you may integrate the phone onto a mobile base.

The particularity of this project is that it relies on the mobile browser instead of a specific app. A webpage is served from the ROS2 node, opening the page from a mobile client will ask for permissions in the browser. The data is transmitted between the server and client using websockets for a modern and fast communication, as opposed to making HTTP requests or streaming UDP.

This repository is inspired by a project I did with students as a TA called [phone-imu](https://github.com/vtalpaert/phone-imu).

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y --ignore-src
colcon build --packages-up-to phone_sensors
# colcon build --symlink-install  --packages-up-to phone_sensors --event-handlers console_direct+
```

## Usage

### Quickstart

```bash
source install/setup.bash
ros2 run phone_sensors server
```

### Parameters

| Name | Type | Unit | Description |
|------|------|------|-------------|
| `host` | string | IP | Use `0.0.0.0` to accept connections outside of locahost |
| `port` | int | | The port where the server listens on |
| `debug` | bool | | Use Flask in debug mode |
| `time_reference_interval` | int | ms | Rate to emit TimeReference data on the client side |

## TODO

- [ ] Explain usage
- [ ] Document server files
- [x] Publish [TimeReference](https://docs.ros2.org/foxy/api/sensor_msgs/msg/TimeReference.html)
- [x] Publish [IMU](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html)
- [ ] Publish [GPS (NavSatFix)](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html)
- [ ] Publish [CameraInfo](https://docs.ros2.org/foxy/api/sensor_msgs/msg/CameraInfo.html)
- [ ] Publish [video stream (Image)](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)
- [ ] Publish orientation as a [Quaternion](http://docs.ros.org/en/api/geometry_msgs/html/msg/Quaternion.html) or [Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)
- [ ] Add [Serial](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API) or Bluetooth, as a possible way for the phone to send data over USB to a microcontroller
- [ ] Launch file example to set parameters
- [ ] robot_localization example for IMU, GPS and Orientation fusion into a complete odometry

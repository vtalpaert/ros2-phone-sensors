# Server configuration guide

## Parameters

| Name                           | Type   | Default          | Unit   | Description                                                                  |
| ------------------------------ | ------ | ---------------- | ------ | ---------------------------------------------------------------------------- |
| `host`                         | string | "0.0.0.0"        | IP     | Use `0.0.0.0` to accept connections outside of localhost                     |
| `port`                         | int    | 2000             |        | The port where the server listens on                                         |
| `debug`                        | bool   | True             |        | Use Flask in debug mode                                                      |
| `secret_key`                   | string | "secret!"        |        | Flask SECRET_KEY                                                             |
| `ssl_certificate`              | string | "certs/certificate.crt" | path | Path to public SSL certificate                                          |
| `ssl_private_key`              | string | "certs/private.key" | path | Path to private SSL key                                                     |
| `use_ros_time`                 | bool   | False            |        | Use ROS time instead of device time for message timestamps                   |
| `time_reference_source_device` | string | "ros_to_device"  |        | Source identifier for device TimeReference messages                          |
| `time_reference_source_gnss`   | string | "device_to_gnss" |        | Source identifier for GNSS TimeReference messages                            |
| `time_reference_frequency`     | float  | -1.0             | Hz     | Rate to emit TimeReference data                                              |
| `imu_frequency`                | float  | 50.0             | Hz     | Rate to emit IMU data                                                        |
| `gnss_watch_position`          | bool   | True             |        | Enable GNSS using `watchPosition`; set to `False` to disable GNSS            |
| `frame_id_imu`                 | string | package_name_imu |        | Frame ID for IMU messages                                                    |
| `frame_id_gnss`                | string | package_name_gnss |        | Frame ID for GNSS messages                                                   |
| `frame_id_image_camera1`       | string | package_name_camera1 |    | Frame ID for camera1 image messages                                          |
| `frame_id_image_camera2`       | string | package_name_camera2 |    | Frame ID for camera2 image messages                                          |
| `camera1_device_label`         | string | "Facing front:3" |        | Label to identify which camera to use for camera1                            |
| `camera1_video_fps`            | float  | 20.0             | Hz     | Video frame rate for camera1                                                 |
| `camera1_video_width`          | int    | 720              | pixels | Video frame width for camera1                                                |
| `camera1_video_height`         | int    | 720              | pixels | Video frame height for camera1                                               |
| `camera1_video_compression`    | float  | 0.3              | 0-1    | JPEG compression quality for camera1 (0=max compression, 1=best quality)     |
| `camera1_calibration_file`     | string | ""               | path   | Path to camera1 calibration YAML file (output from camera_calibration)       |
| `camera2_device_label`         | string | "Facing back:0"  |        | Label to identify which camera to use for camera2                            |
| `camera2_video_fps`            | float  | 20.0             | Hz     | Video frame rate for camera2                                                 |
| `camera2_video_width`          | int    | 720              | pixels | Video frame width for camera2                                                |
| `camera2_video_height`         | int    | 720              | pixels | Video frame height for camera2                                               |
| `camera2_video_compression`    | float  | 0.3              | 0-1    | JPEG compression quality for camera2 (0=max compression, 1=best quality)     |
| `camera2_calibration_file`     | string | ""               | path   | Path to camera2 calibration YAML file (output from camera_calibration)       |
| `usb_enabled`                  | bool   | False            |        | Show the USB section on the main page and activate `usb/rx` / `usb/tx`       |
| `usb_device_type`              | string | "cdc"            |        | USB protocol: `"cdc"` for native USB boards (Teensy, Leonardo), `"cp2102"` for USB-UART converter |
| `usb_baud`                     | int    | 115200           | baud   | Serial baud rate sent to the USB device during connection setup               |

A negative value for the time reference or IMU frequency, or a negative video FPS will disable sending the corresponding data from the client device. This allows conserving bandwidth and processing power when certain sensors are not needed.

To find out the available `camera1_device_label` and `camera2_device_label`, open the video test page
<p align="center">
    <img src="docs/webpage_firefox_test_video.jpg" alt="Video test page" width="50%" height="auto">
</p>

## Coordinates

### IMU frame (`frame_id_imu`)

The device rotation rate and linear acceleration are expressed in the **device coordinate frame**:
<p align="center">
    <img src="https://developer.mozilla.org/en-US/docs/Web/API/Device_orientation_events/Orientation_and_motion_data_explained/axes.png" alt="Device coordinate frame" width="50%" height="auto">
</p>

The device orientation is defined in **ENU (East-North-Up)**. The angles alpha, beta, gamma from the web API are rotations around Z, X, Y respectively.
They are converted to a quaternion using beta as roll, gamma as pitch, alpha as yaw. The behaviour of a device with its Y-axis pointing up is unstable since the API is only defined between -90° to +90°. See the [web API documentation](https://developer.mozilla.org/en-US/docs/Web/API/Window/deviceorientationabsolute_event) for more details.

### GNSS frame (`frame_id_gnss`)

The `gnss` (NavSatFix) and `gnss/odometry` messages share the same frame ID set by `frame_id_gnss`.

This frame is **ENU-aligned**: X points East, Y points North, Z points Up. The `gnss/odometry` message uses this frame for both `header.frame_id` and `child_frame_id`, meaning the twist is expressed in ENU coordinates:

- `twist.linear.x` — East velocity (m/s)
- `twist.linear.y` — North velocity (m/s)

The browser [GeolocationCoordinates](https://developer.mozilla.org/en-US/docs/Web/API/GeolocationCoordinates) API reports heading in degrees clockwise from true North (0 = North, 90 = East). This is converted to ENU velocity components as:

```txt
vx (East)  = speed × sin(heading_rad)
vy (North) = speed × cos(heading_rad)
```

The message is published according to the following cases:

| Condition | Twist | Twist covariance (vx, vy) |
| --------- | ----- | ------------------------- |
| `heading` available, `speed` > 0 | ENU velocity from heading + speed | 0.01 (m/s)² |
| `heading` is null, `speed` ≈ 0 | Zero (stationary) | 0.001 (m/s)² |
| `heading` is null, `speed` > 0 | Zero (direction unknown) | 1e9 (ignore) |
| `speed` is null | Not published | — |

The pose covariance is set to 1e9 on all diagonal elements. When using `robot_localization`, configure it to fuse the **twist** from `gnss/odometry` and the **position** from `gnss` (via `navsat_transform_node`), not the pose of the odometry message.

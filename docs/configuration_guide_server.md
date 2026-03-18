# Server configuration guide

## Parameters

| Name                           | Type   | Default                  | Unit          | Description                                                                  |
| ------------------------------ | ------ | ------------------------ | ------------- | ---------------------------------------------------------------------------- |
| `name`                         | string | "phone_sensors_bridge"   |               | Prefix used to build default frame IDs                                       |
| `host`                         | string | "0.0.0.0"                | IP            | Use `0.0.0.0` to accept connections outside of localhost                     |
| `port`                         | int    | 2000                     |               | The port where the server listens on                                         |
| `debug`                        | bool   | False                    |               | Use Flask in debug mode                                                      |
| `secret_key`                   | string | "secret!"                |               | Flask SECRET_KEY                                                             |
| `ssl_certificate`              | string | "certs/certificate.crt"  | path          | Path to public SSL certificate                                               |
| `ssl_private_key`              | string | "certs/private.key"      | path          | Path to private SSL key                                                      |
| `use_ros_time`                 | bool   | False                    |               | Use ROS time instead of device time for message timestamps                   |
| `time_reference_source_device` | string | "ros_to_device"          |               | Source identifier for device TimeReference messages                          |
| `time_reference_source_gnss`   | string | "device_to_gnss"         |               | Source identifier for GNSS TimeReference messages                            |
| `time_reference_frequency`     | float  | -1.0                     | Hz            | Rate to emit TimeReference data                                              |
| `imu_frequency`                | float  | 50.0                     | Hz            | Rate to emit IMU data                                                        |
| `gnss_watch_position`          | bool   | True                     |               | Enable GNSS using `watchPosition`; set to `False` to disable GNSS            |
| `frame_id_imu`                 | string | name + "_imu"            |               | Frame ID for IMU messages                                                    |
| `frame_id_gnss`                | string | name + "_gnss"           |               | Frame ID for NavSatFix messages                                              |
| `frame_id_gnss_velocity`       | string | name + "_odom"           |               | Frame ID for `gnss/odometry` twist (should match `child_frame_id`)           |
| `frame_id_image_camera1`       | string | name + "_camera1"        |               | Frame ID for camera1 image messages                                          |
| `frame_id_image_camera2`       | string | name + "_camera2"        |               | Frame ID for camera2 image messages                                          |
| `camera1_device_label`         | string | "camera 1, facing front" |               | Label to identify which camera to use for camera1                            |
| `camera1_video_fps`            | float  | 20.0                     | Hz            | Video frame rate for camera1                                                 |
| `camera1_video_width`          | int    | 720                      | pixels        | Video frame width for camera1                                                |
| `camera1_video_height`         | int    | 720                      | pixels        | Video frame height for camera1                                               |
| `camera1_video_compression`    | float  | 0.3                      | 0-1           | JPEG compression quality for camera1 (0=max compression, 1=best quality)     |
| `camera1_calibration_file`     | string | ""                       | path          | Path to camera1 calibration YAML file (output from camera_calibration)       |
| `camera2_device_label`         | string | "camera 0, facing back"  |               | Label to identify which camera to use for camera2                            |
| `camera2_video_fps`            | float  | 20.0                     | Hz            | Video frame rate for camera2                                                 |
| `camera2_video_width`          | int    | 720                      | pixels        | Video frame width for camera2                                                |
| `camera2_video_height`         | int    | 720                      | pixels        | Video frame height for camera2                                               |
| `camera2_video_compression`    | float  | 0.3                      | 0-1           | JPEG compression quality for camera2 (0=max compression, 1=best quality)     |
| `camera2_calibration_file`     | string | ""                       | path          | Path to camera2 calibration YAML file (output from camera_calibration)       |
| `usb_enabled`                  | bool   | False                    |               | Show the USB section on the main page and activate `usb/rx` / `usb/tx`       |
| `usb_device_type`              | string | "cdc"                    |               | USB protocol: `"cdc"` for native USB boards (Teensy, Leonardo), `"cp2102"` for USB-UART converter |
| `usb_baud`                     | int    | 115200                   | baud          | Serial baud rate sent to the USB device during connection setup               |

### IMU covariance parameters

These parameters control the covariance injected into IMU messages. The values for gyroscope and accelerometer can be obtained from the calibration script `imu_gaussian_noise.py`. The formula used is:

```txt
gyro_var   = imu_k_inflation × imu_frequency × imu_gyro_noise_density²   + imu_gyro_quantization_variance
accel_var  = imu_k_inflation × imu_frequency × imu_accel_noise_density²  + imu_accel_quantization_variance
orient_var = imu_orient_inflation × imu_orient_variance
```

| Name                              | Type  | Default | Unit            | Description                                              |
| --------------------------------- | ----- | ------- | --------------- | -------------------------------------------------------- |
| `imu_k_inflation`                 | float | 10.0    |                 | Safety inflation factor applied to gyro and accel variance |
| `imu_gyro_noise_density`          | float | 7e-05   | rad/s/√Hz       | Gyroscope noise density from calibration                 |
| `imu_gyro_quantization_variance`  | float | 3e-07   | rad²            | Gyroscope quantization variance (q²/12)                  |
| `imu_accel_noise_density`         | float | 2e-04   | m/s²/√Hz        | Accelerometer noise density from calibration             |
| `imu_accel_quantization_variance` | float | 9e-04   | (m/s²)²         | Accelerometer quantization variance (q²/12)              |
| `imu_orient_variance`             | float | 2e-06   | rad²            | Orientation variance (measured + quantization)           |
| `imu_orient_inflation`            | float | 10.0    |                 | Safety inflation factor applied to orientation variance  |

### GNSS covariance parameters

| Name                       | Type  | Default | Unit    | Description                                                              |
| -------------------------- | ----- | ------- | ------- | ------------------------------------------------------------------------ |
| `gnss_position_inflation`  | float | 10.0    |         | Safety factor multiplied onto GPS position and altitude variance         |
| `gnss_speed_variance`      | float | 0.01    | (m/s)²  | Twist covariance for case 1 (heading available, speed > 0)               |
| `gnss_zero_velocity_variance` | float | 0.001 | (m/s)²  | Twist covariance for case 2 (heading null, speed ≈ 0, zero-velocity update) |

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

### GNSS frames

The server publishes two GNSS-related messages with separate frame IDs:

- **`gnss` (NavSatFix)** — uses `frame_id_gnss` (default `name_gnss`). This is the frame attached to the GPS sensor on the device, typically set to `base_link` when the device is the robot.
- **`gnss/odometry` (Odometry)** — uses `frame_id_gnss_velocity` (default `name_odom`) for both `header.frame_id` and `child_frame_id`.

#### Odometry message frames

The ROS `Odometry` message has two frame fields with distinct meanings:

- `header.frame_id` — the reference frame in which the **pose** is expressed (i.e. the world frame)
- `child_frame_id` — the frame in which the **twist** is expressed (i.e. the moving body frame)

The pose describes the position and orientation of `child_frame_id` relative to `header.frame_id`. The twist describes the velocity of `child_frame_id` expressed in `child_frame_id` coordinates.

For `gnss/odometry`, both fields are set to `frame_id_gnss_velocity` (typically `"odom"`). This reflects that the GPS chip reports velocity already in ENU coordinates — the same frame as `odom`. Setting both fields to the same ENU frame means:

- The twist components are directly the ENU velocity: `vx` = East, `vy` = North.
- `robot_localization` will look up the `odom → base_link` transform to rotate the twist into the robot body frame before fusing.
- The pose is **not used** — all diagonal elements of the pose covariance are set to `1e9`. Only the twist is intended to be fused.

> If you want to express the velocity for `base_link`, you would need the change the pose.orientation according to the IMU

#### GPS velocity cases

The browser [GeolocationCoordinates](https://developer.mozilla.org/en-US/docs/Web/API/GeolocationCoordinates) API reports heading in degrees clockwise from true North (0 = North, 90 = East). This is converted to ENU velocity components as:

```txt
vx (East)  = speed × sin(heading_rad)
vy (North) = speed × cos(heading_rad)
```

The message is published according to the following cases:

| Condition | Twist | Twist covariance (vx, vy) |
| --------- | ----- | ------------------------- |
| `heading` available, `speed` > 0 | ENU velocity from heading + speed | `gnss_speed_variance` |
| `heading` is null, `speed` ≈ 0 | Zero (stationary) | `gnss_zero_velocity_variance` |
| `heading` is null, `speed` > 0 | Zero (direction unknown) | 1e9 (ignore) |
| `speed` is null | Not published | — |

#### GPS position covariance

The NavSatFix position covariance is computed from the accuracy values reported by the browser:

```txt
horizontal_var = gnss_position_inflation × accuracy²
alt_var        = gnss_position_inflation × altitudeAccuracy²
```

`accuracy` and `altitudeAccuracy` are 1σ radii in meters as defined by the [GeolocationCoordinates API](https://developer.mozilla.org/en-US/docs/Web/API/GeolocationCoordinates). If `altitudeAccuracy` is unavailable, `alt_var` is set to `1e9`.

When using `robot_localization`, configure it to fuse the **twist** from `gnss/odometry` and the **position** from `gnss` (via `navsat_transform_node`), not the pose of the odometry message.

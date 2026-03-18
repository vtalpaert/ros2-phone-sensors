# IMU calibration

We use the [IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) as defined in [Kalibr](https://github.com/ethz-asl/kalibr). This defines an additive white noise (noise density) and a slow drift (bias instability).

The noise density drives the IMU covariance in the EKF; the bias is used by sensor fusion algorithms.

## Covariances

Using a discretized model, the per-sample covariance for gyro and accel is:

```math
cov_{imu} = \texttt{imu\_k\_inflation} \times f_{IMU} \times \sigma_{noise-density}^2 + \frac{q^2}{12}
```

- `imu_k_inflation`, safety factor recommended for low-cost sensors such as phone IMUs ([Kalibr wiki](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model#kalibr-imu-noise-parameters-in-practice)), default 10
- `f_IMU`, sampling frequency (`imu_frequency`), set at runtime
- `sigma_noise_density`, sensor constant measured once (`imu_gyro_noise_density` / `imu_accel_noise_density`)
- `q²/12`, quantization noise, fixed regardless of frequency (`imu_gyro_quantization_variance` / `imu_accel_quantization_variance`)

Orientation (already fused by the phone) uses a single fixed variance scaled by an inflation factor:

```math
cov_{orient} = \texttt{imu\_orient\_inflation} \times \texttt{imu\_orient\_variance}
```

- `imu_orient_variance`, total orientation variance (measured variance + quantization variance), measured once
- `imu_orient_inflation`, safety inflation factor, default 10

## Why not Allan variance

Allan variance is the standard method for IMU characterisation but it **requires raw, unfiltered sensor data**. Phone IMUs pass their output through on-device sensor fusion and low-pass filters before exposing it to the browser. This makes the Allan deviation curves unreliable: the filters suppress noise at short averaging times and can hide rate random walk entirely. The Gaussian method below works on filtered data because it only needs the variance at a single time scale (one sample).

The Allan variance results are still included at the end for comparison. The order of magnitude is consistent, which confirms the measurement is plausible.

## Recording the two bags

Two recordings are needed.

**Static bag**: place the phone on a flat, vibration-free surface for several hours (3h, the longer the better). This is used to estimate noise density and orientation variance. Do not touch or bump the surface. This same file is used for Allan variance.

**Dynamic bag**: hold the phone and move it in a figure-eight pattern at varying speeds for 2 minutes, covering all orientations (exactly like the compass calibration gesture). Speed variation matters: slow passes expose small quantization steps, fast passes cover the full measurement range. This is used only for quantization detection, not for noise estimation.

To record a bag

```bash
# If you are running an old ROS2 version such as Humble
sudo apt install ros-$ROS_DISTRO-rosbag2-storage-mcap

ros2 run phone_sensors_bridge server --ros-args \
  -p gnss_watch_position:=False \
  -p imu_frequency:=80.0 \
  -p camera1_video_fps:=-1.0 \
  -p camera2_video_fps:=-1.0

ros2 bag record -s mcap /imu
```

Verify the actual output frequency before long recordings, and lower `imu_frequency` if the topic lags:

```bash
ros2 topic hz /imu
```

## Evaluate IMU parameters

### Using a Gaussian model

```bash
python3 src/phone_sensors_bridge_examples/scripts/imu_gaussian_noise.py rosbag2_static/ rosbag2_dynamic/

# Skip the first 60 s if there were disturbances at the start of the static bag:
python3 src/phone_sensors_bridge_examples/scripts/imu_gaussian_noise.py rosbag2_static/ rosbag2_dynamic/ --start 60
```

Example output:

```text
Static bag : rosbag2_2026_03_10-18_09_34/
Dynamic bag: rosbag2_2026_03_11-17_09_12/

############################################################
  NOISE ESTIMATION  (static bag)
############################################################

============================================================
  ORIENTATION  [rad]
============================================================
  Samples: 860719  |  Duration: 10311.9 s  |  fs = 83.47 Hz

  roll (x):
    mean = +58.1272°   std = 0.0595° (0.001038 rad)   variance = 1.076871e-06 rad²
    quantization step: 0.001745 rad  (0.1000°)   var_q = 2.538478e-07 rad²
  pitch (y):
    mean = -1.0680°   std = 0.0485° (0.000847 rad)   variance = 7.168635e-07 rad²
    quantization step: 0.001745 rad  (0.1000°)   var_q = 2.538478e-07 rad²
  yaw (z):
    mean = +78.3136°   std = 0.0597° (0.001041 rad)   variance = 1.084527e-06 rad²
    quantization step: 0.001745 rad  (0.1000°)   var_q = 2.538478e-07 rad²

  Mean variance (all axes): 9.594204e-07 rad²

============================================================
  GYROSCOPE  [rad/s]
============================================================
  Samples: 860719  |  Duration: 10311.9 s  |  fs = 83.47 Hz

  gx:  mean = +0.000003   std = 0.000534   noise_density = 5.847292e-05   discrete_var = 2.853865e-07
  gy:  mean = -0.000025   std = 0.000283   noise_density = 3.093233e-05   discrete_var = 7.986366e-08
  gz:  mean = +0.000008   std = 0.001044   noise_density = 1.142509e-04   discrete_var = 1.089540e-06

  Mean noise density (all axes): 6.788539e-05
  Mean discrete_var  (all axes): 3.846595e-07  (= fs × sigma²)

============================================================
  ACCELEROMETER  [m/s2]
============================================================
  Samples: 860719  |  Duration: 10311.9 s  |  fs = 83.47 Hz

  ax:  mean = -0.000000   std = 0.001000   noise_density = 1.094099e-04   discrete_var = 9.991656e-07
  ay:  mean = +0.000001   std = 0.001376   noise_density = 1.506265e-04   discrete_var = 1.893767e-06
  az:  mean = -0.000003   std = 0.002733   noise_density = 2.991664e-04   discrete_var = 7.470496e-06

  Mean noise density (all axes): 1.864009e-04
  Mean discrete_var  (all axes): 2.900147e-06  (= fs × sigma²)

############################################################
  QUANTIZATION DETECTION  (dynamic bag)
############################################################

============================================================
  GYROSCOPE  [rad/s]
============================================================

  gx: quantization step = 1.744987e-03   var_q = 2.537484e-07
  gy: quantization step = 1.744804e-03   var_q = 2.536952e-07
  gz: quantization step = 1.744162e-03   var_q = 2.535084e-07

============================================================
  ACCELEROMETER  [m/s2]
============================================================

  ax: quantization step = 9.999990e-02   var_q = 8.333317e-04
  ay: quantization step = 9.999553e-02   var_q = 8.332588e-04
  az: quantization step = 9.999801e-02   var_q = 8.333002e-04

============================================================
  VARIANCE COMPARISON  (before inflation, at measured fs)
============================================================
                          Gaussian (fs×σ²)  Quantization (q²/12)           Sum
  ----------------------------------------------------------------------------
  gyro   [rad/s]²             3.846595e-07          2.536506e-07  6.383101e-07
  accel  [m/s²]²              2.900147e-06          8.332969e-04  8.361970e-04
  orient [rad²]               9.594204e-07          2.538478e-07  1.213268e-06

  Both sources are independent → variances sum (do not take max).

============================================================
  Suggested server parameter values (see configuration_guide_server.md)
============================================================

  GYROSCOPE  [rad/s]
    noise_density  = 6.7885e-05  # rad/s/sqrt(Hz)  → dynamic part (computed from fs)
    var_quant      = 2.5365e-07  # rad²            → fixed part (q²/12)
    k_inflation    = 10                       # suggested safety factor
    total at fs    = k × fs × sigma² + var_quant  =  4.1002e-06  rad²

  ACCELEROMETER  [m/s²]
    noise_density  = 1.8640e-04  # m/s²/sqrt(Hz)  → dynamic part (computed from fs)
    var_quant      = 8.3330e-04  # (m/s²)²        → fixed part (q²/12)
    k_inflation    = 10                       # suggested safety factor
    total at fs    = k × fs × sigma² + var_quant  =  8.6230e-04  (m/s²)²

  ORIENTATION  [rad]
    var_measured   = 9.5942e-07  # rad²  (std² from static recording)
    var_quant      = 2.5385e-07  # rad²  (q²/12)
    total (fixed)  = var_measured + var_quant  =  1.2133e-06  rad²

  NOTE: gyro/accel have a fixed (var_quant) + dynamic (k × fs × sigma²) part.
        orientation uses a single fixed variance (no fs scaling).
```

### Comparison with Allan variance

[`allan_variance_ros2`](https://github.com/Autoliv-Research/allan_variance_ros2) (check the Pull Requests for bug fixes) can also be run on the static bag. The results are **not reliable for filtered phone data** (see above), but the order of magnitude is consistent with the Gaussian method, which is a useful sanity check.

```bash
python3 ./src/allan_variance_ros2/scripts/analysis.py --data output/allan_variance.csv
ACCELEROMETER:
X Velocity Random Walk:  0.00006 m/s/sqrt(s)  0.00354 m/s/sqrt(hr)
Y Velocity Random Walk:  0.00011 m/s/sqrt(s)  0.00686 m/s/sqrt(hr)
Z Velocity Random Walk:  0.00016 m/s/sqrt(s)  0.00977 m/s/sqrt(hr)
X Bias Instability:  0.00000 m/s^2  24.88652 m/hr^2
Y Bias Instability:  0.00000 m/s^2  36.50673 m/hr^2
Z Bias Instability:  0.00001 m/s^2  153.40441 m/hr^2
X Accel Random Walk:  0.00000 m/s^2/sqrt(s)
Y Accel Random Walk:  0.00000 m/s^2/sqrt(s)
Z Accel Random Walk:  0.00000 m/s^2/sqrt(s)
GYROSCOPE:
X Angle Random Walk:  0.00461 deg/sqrt(s)  0.27680 deg/sqrt(hr)
Y Angle Random Walk:  0.00259 deg/sqrt(s)  0.15565 deg/sqrt(hr)
Z Angle Random Walk:  0.01882 deg/sqrt(s)  1.12942 deg/sqrt(hr)
X Bias Instability:  0.00028 deg/s  1.02039 deg/hr
Y Bias Instability:  0.00049 deg/s  1.74850 deg/hr
Z Bias Instability:  0.00516 deg/s  18.55872 deg/hr
X Rate Random Walk:  0.00004 deg/s/sqrt(s)
Y Rate Random Walk:  0.00007 deg/s/sqrt(s)
Z Rate Random Walk:  0.00067 deg/s/sqrt(s)
Writing Kalibr imu.yaml file.
Make sure to update rostopic and rate.
```

The gyro angle random walk (approx 0.005 deg/sqrt(s) approx 8.7×10e-5 rad/sqrt(s)) and accel velocity random walk (approx 0.0001 m/s/sqrt(s)) are in the same order of magnitude as the noise densities from the Gaussian method (6.8×10e-5 rad/s/sqrt(Hz) and 1.9×10e-4 m/s²/sqrt(Hz) respectively), confirming both approaches are consistent despite the filtered signal.

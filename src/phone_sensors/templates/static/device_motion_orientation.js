function registerDeviceMotionOrientationPublisher(socket, window) {
    var include_gravity = false;
    window.motion_permission_granted = false;
    window.motion_init = false;
    window.orientation_init = false;

    var accel_x = 0;
    var accel_y = 0;
    var accel_z = 0;
    var rotation_beta = 0;
    var rotation_gamma = 0;
    var rotation_alpha = 0;
    var orientation_beta = 0;
    var orientation_gamma = 0;
    var orientation_alpha = 0;
    var motion_interval_ms = 0;
    var absolute = true;
    var compass_heading = 0;
    var compass_accuracy = 0;

    function handleMotionEvent(event) {
        motion_init = true;
        if (include_gravity) {
            accel_x = event.accelerationIncludingGravity.x;
            accel_y = event.accelerationIncludingGravity.y;
            accel_z = event.accelerationIncludingGravity.z;
        } else {
            accel_x = event.acceleration.x;
            accel_y = event.acceleration.y;
            accel_z = event.acceleration.z;
        }
        rotation_alpha = event.rotationRate.alpha
        rotation_beta = event.rotationRate.beta
        rotation_gamma = event.rotationRate.gamma 
        motion_interval_ms = event.interval
    }

    function handleOrientationEvent(event) {
        orientation_init = true;
        absolute = event.absolute;
        orientation_alpha = event.alpha;
        orientation_beta = event.beta;
        orientation_gamma = event.gamma;
        compass_heading = event.webkitCompassHeading;
        compass_accuracy = event.webkitCompassAccuracy;
    }

    window.addEventListener("devicemotion", handleMotionEvent, true);
    window.addEventListener("deviceorientationabsolute", handleOrientationEvent, true);

    function deviceMotionOrientationSendData() {
        if (motion_init && orientation_init) {
            socket.emit("data", {
                date_ms: Date.now(),
                motion: {
                    ax: accel_x,
                    ay: accel_y,
                    az: accel_z,
                    rb: rotation_beta,
                    rg: rotation_gamma,
                    ra: rotation_alpha,
                    ob: orientation_beta,
                    og: orientation_gamma,
                    oa: orientation_alpha,
                    im: motion_interval_ms,
                    abs: absolute,
                    ch: compass_heading,
                    ca: compass_accuracy
                }
            });
        }
    };

    var deviceMotionOrientationLoop = 0;
    function deviceMotionOrientationStartSending(sendInterval) {
        if (window.motion_permission_granted) {
            socket.emit("info", "Sending deviceMotionOrientation with interval " + sendInterval + " ms");
            deviceMotionOrientationLoop = setInterval(deviceMotionOrientationSendData, sendInterval);
        } else {
            socket.emit("error", "Cannot start IMU: motion permission not granted. Retrying in 1 second...");
            setTimeout(() => deviceMotionOrientationStartSending(sendInterval), 1000);
        }
    };
    function deviceMotionOrientationStopSending() {
        if (deviceMotionOrientationLoop != 0) {
            clearInterval(deviceMotionOrientationLoop);
        }
        socket.sendBuffer = [];  // empty buffer to stop sending
    };

    socket.on("imu_frequency", function(value, cb) {
        deviceMotionOrientationStopSending();
        if (value > 0) {
            const interval = Math.floor(1000 / value);  // convert Hz to ms
            deviceMotionOrientationStartSending(interval);
        }
    });
}

function registerImuPublisher(socket) {
    var include_gravity = false;

    var motion_init = false;
    var orientation_init = false

    var ax = 0;
    var ay = 0;
    var az = 0;
    var gx = 0;
    var gy = 0;
    var gz = 0;
    var ox = 0;
    var oy = 0;
    var oz = 0;
    var motion_interval_ms = 0;
    var absolute = true;
    var compass_heading = 0;
    var compass_accuracy = 0;

    function handleMotionEvent(event) {
        motion_init = true;
        if (include_gravity) {
            ax = event.accelerationIncludingGravity.x;
            ay = event.accelerationIncludingGravity.y;
            az = event.accelerationIncludingGravity.z;
        } else {
            ax = event.acceleration.x;
            ay = event.acceleration.y;
            az = event.acceleration.z;
        }
        gz = event.rotationRate.alpha
        gx = event.rotationRate.beta
        gy = event.rotationRate.gamma 
        motion_interval_ms = event.interval
    }

    function handleOrientationEvent(event) {
        orientation_init = true;
        absolute = event.absolute;
        oz = event.alpha;
        ox = event.beta;
        oy = event.gamma;
        compass_heading = event.webkitCompassHeading;
        compass_accuracy = event.webkitCompassAccuracy;
    }

    window.addEventListener("devicemotion", handleMotionEvent, true);
    window.addEventListener("deviceorientationabsolute", handleOrientationEvent, true);

    function imuSendData() {
        if (motion_init && orientation_init) {
            socket.emit("data", {
                date_ms: Date.now(),
                imu: {
                    ax: ax,
                    ay: ay,
                    az: az,
                    gx: gx,
                    gy: gy,
                    gz: gz,
                    ox: ox,
                    oy: oy,
                    oz: oz,
                    motion_interval_ms: motion_interval_ms,
                    absolute: absolute,
                    compass_heading: compass_heading,
                    compass_accuracy: compass_accuracy
                }
            });
        }
    };

    var imuLoop = 0;
    function imuStartSending(sendInterval) {
        socket.emit("log", "Sending imu with interval " + sendInterval + " ms");
        imuLoop = setInterval(imuSendData, sendInterval);
    };
    function imuStopSending() {
        if (imuLoop != 0) {
            clearInterval(imuLoop);
        }
        socket.sendBuffer = [];  // empty buffer to stop sending
    };

    socket.on("imu_set_interval", function(interval, cb) {
        imuStopSending();
        if (interval >= 0) {
            imuStartSending(interval);
        }
    });
}
function registerGeolocationPublisher(socket, window) {
    window.geolocation_permission_granted = false;
    const useWatchPosition = true;

    function success(geolocation) {
        socket.emit("data", {
            date_ms: Date.now(),
            loc: geolocation.toJSON()
        });
    }

    function error(error) {
        socket.emit("error", "Geolocation: " + error.message);
    }

    var watchId = 0;
    function geolocationStartSending(sendInterval) {
        if (window.geolocation_permission_granted) {
            socket.emit("debug", "Starting GNSS");
            if (useWatchPosition) {
                const options = {
                    enableHighAccuracy: true,
                    maximumAge: sendInterval,
                    timeout: sendInterval,
                };
                socket.emit("info", "Starting GNSS watch");
                watchId = navigator.geolocation.watchPosition(success, error, options);
            } else {
                const options = {
                    enableHighAccuracy: true,
                    maximumAge: 500,
                    timeout: 1000,
                };
                socket.emit("info", "Starting GNSS get position at " + sendInterval + " ms");
                watchId = setInterval(
                    navigator.geolocation.getCurrentPosition(success, error, options),
                    sendInterval
                );
            }
        } else {
            socket.emit("error", "Cannot start GNSS: geolocation permission not granted. Retrying in 1 second...");
            setTimeout(() => geolocationStartSending(sendInterval), 1000);
        }
    };
    function geolocationStopSending() {
        if (watchId != 0) {
            socket.emit("debug", "Stopping GNSS");
            if (useWatchPosition) {
                navigator.geolocation.clearWatch(watchId);
            } else {
                clearInterval(watchId);
            }
            watchId = 0;
        }
        socket.sendBuffer = [];  // empty buffer to stop sending
    };

    socket.on("gnss_frequency", function (value, cb) {
        geolocationStopSending();
        if (value > 0) {
            const interval = Math.floor(1000 / value);  // convert Hz to ms
            geolocationStartSending(interval);
        }
    });
}

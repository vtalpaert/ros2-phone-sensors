function requestGeolocationPermission(logCallback) {
    if ("geolocation" in navigator) {
        navigator.geolocation.getCurrentPosition(
            () => {
                window.geolocation_permission_granted = true;
                logCallback("Geolocation permission granted");
            },
            error => {
                window.geolocation_permission_granted = false;
                logCallback("Geolocation permission denied: " + error.message);
            }
        );
    } else {
        window.geolocation_permission_granted = false;
        logCallback("Geolocation not available");
    }
}

function registerGeolocationPublisher(socket, window) {
    let useWatchPosition = true;

    // Must be received before "gnss_frequency" fires (guaranteed by server emission order
    // in client_params, where gnss_use_watch_position is declared before gnss_frequency).
    socket.on("gnss_use_watch_position", function (value) {
        useWatchPosition = value;
    });

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
    // sendInterval is only used in polling mode (useWatchPosition=false).
    // In watch mode the browser controls the update rate and sendInterval is ignored.
    function geolocationStartSending(sendInterval) {
        if (!window.geolocation_permission_granted) {
            socket.emit("error", "Cannot start GNSS: geolocation permission not granted. Retrying in 1 second...");
            setTimeout(() => geolocationStartSending(sendInterval), 1000);
            return;
        }

        socket.emit("debug", "Starting GNSS");
        if (useWatchPosition) {
            const options = {
                enableHighAccuracy: true,
                maximumAge: 0,
                timeout: 5000,
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

    // "gnss_frequency" is always the start/stop trigger regardless of mode (value <= 0 disables GNSS).
    // The computed interval is only meaningful in polling mode; in watch mode it is passed through
    // but ignored inside geolocationStartSending.
    socket.on("gnss_frequency", function (value, cb) {
        geolocationStopSending();
        if (value > 0) {
            const interval = Math.floor(1000 / value);  // convert Hz to ms, used in polling mode only
            geolocationStartSending(interval);
        }
    });
}

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
    function geolocationStartSending() {
        if (!window.geolocation_permission_granted) {
            socket.emit("error", "Cannot start GNSS: geolocation permission not granted. Retrying in 1 second...");
            setTimeout(() => geolocationStartSending(), 1000);
            return;
        }
        const options = {
            enableHighAccuracy: true,
            maximumAge: 0,
            timeout: 5000,
        };
        socket.emit("info", "Starting GNSS watch");
        watchId = navigator.geolocation.watchPosition(success, error, options);
    };
    function geolocationStopSending() {
        if (watchId != 0) {
            socket.emit("debug", "Stopping GNSS");
            navigator.geolocation.clearWatch(watchId);
            watchId = 0;
        }
        socket.sendBuffer = [];  // empty buffer to stop sending
    };

    // gnss_watch_position: true starts the watchPosition loop (enables GNSS), false stops it.
    socket.on("gnss_watch_position", function (value) {
        geolocationStopSending();
        if (value) {
            geolocationStartSending();
        }
    });
}

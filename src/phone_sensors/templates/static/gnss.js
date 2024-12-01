function registerGpsPublisher(socket, window) {
    window.geolocation_permission_granted = false;
    const options = {
        enableHighAccuracy: true,
        maximumAge: 1000,
        timeout: 1000,
    };

    function success(position) {
        socket.emit("data", {
            date_ms: Date.now(),
            gnss: position.toJSON()
        });
    }

    function error(error) {
        socket.emit("error", error.message);
    }

    function gnssSendData(position) {
        navigator.geolocation.getCurrentPosition(success, error, options);
    }

    var gnssLoop = 0;
    function gnssStartSending(sendInterval) {
        if (window.geolocation_permission_granted) {
            socket.emit("info", "Sending gnss with interval " + sendInterval + " ms");
            gnssLoop = setInterval(gnssSendData, sendInterval);
        } else {
            socket.emit("error", "Cannot start GNSS: geolocation permission not granted. Retrying in 1 second...");
            setTimeout(() => gnssStartSending(sendInterval), 1000);
        }
    };
    function gnssStopSending() {
        if (gnssLoop != 0) {
            clearInterval(gnssLoop);
        }
        socket.sendBuffer = [];  // empty buffer to stop sending
    };

    socket.on("gnss_frequency", function(value, cb) {
        gnssStopSending();
        if (value > 0) {
            const interval = Math.floor(1000 / value);  // convert Hz to ms
            gnssStartSending(interval);
        }
    });
}

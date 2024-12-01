function registerGpsPublisher(socket) {
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
        socket.emit("info", "Sending gnss with interval " + sendInterval + " ms");
        gnssLoop = setInterval(gnssSendData, sendInterval);
    };
    function gnssStopSending() {
        if (gnssLoop != 0) {
            clearInterval(gnssLoop);
        }
        socket.sendBuffer = [];  // empty buffer to stop sending
    };

    if ("geolocation" in navigator) {
        /* geolocation is available */
        socket.on("gnss_frequency", function(value, cb) {
            gnssStopSending();
            if (value > 0) {
                const interval = Math.floor(1000 / value);  // convert Hz to ms
                gnssStartSending(interval);
            }
        });
    } else {
        /* geolocation IS NOT available */
        socket.emit("error", "Geolocation not available");
    }
}

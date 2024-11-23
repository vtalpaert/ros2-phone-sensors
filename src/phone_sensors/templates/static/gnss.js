function registerGpsPublisher(socket) {
    const options = {
        enableHighAccuracy: false,
        maximumAge: 300000,
        timeout: 270000,
    };

    function success(position) {
        socket.emit("data", {
            date_ms: Date.now(),
            gnss: position.toJSON()
        });
    }

    function error(error) {
        socket.emit("log", error.message);
    }

    function gnssSendData(position) {
        navigator.geolocation.getCurrentPosition(success, error);
    }

    var gnssLoop = 0;
    function gnssStartSending(sendInterval) {
        socket.emit("log", "Sending gnss with interval " + sendInterval + " ms");
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
        socket.on("gnss_set_interval", function(interval, cb) {
            gnssStopSending();
            if (interval >= 0) {
                gnssStartSending(interval);
            }
        });
    } else {
        /* geolocation IS NOT available */
        socket.emit("log", "Geolocation not available");
    }

}

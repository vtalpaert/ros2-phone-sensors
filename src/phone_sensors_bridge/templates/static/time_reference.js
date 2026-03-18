function registerTimeReferencePublisher(socket) {
    function timeReferenceSendData() {
        socket.emit("data", {date_us: Math.round((performance.timeOrigin + performance.now()) * 1000)});
    };

    var timeReferenceLoop = 0;
    function timeReferenceStartSending(sendInterval) {
        socket.emit("info", "Sending timeReference with interval " + sendInterval + " ms");
        timeReferenceLoop = setInterval(timeReferenceSendData, sendInterval);
    };
    function timeReferenceStopSending() {
        if (timeReferenceLoop != 0) {
            clearInterval(timeReferenceLoop);
            timeReferenceLoop = 0;
        }
        socket.sendBuffer = [];  // empty buffer to stop sending
    };

    var pendingTimeReferenceInterval = null;

    socket.on("time_reference_frequency", function(value) {
        pendingTimeReferenceInterval = value > 0 ? Math.floor(1000 / value) : null;
    });

    socket.on("stream_stop", function () {
        timeReferenceStopSending();
    });

    socket.on("stream_start", function () {
        timeReferenceStopSending();
        if (pendingTimeReferenceInterval !== null) {
            timeReferenceStartSending(pendingTimeReferenceInterval);
        }
    });
}

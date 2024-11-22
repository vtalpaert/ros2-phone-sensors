function registerTimeReferencePublisher(socket) {
    function timeReferenceSendData() {
        socket.emit("data", {date_ms: Date.now()});
    };

    var timeReferenceLoop = 0;
    function timeReferenceStartSending(sendInterval) {
        socket.emit("info", "Sending timeReference with interval " + sendInterval + " ms");
        timeReferenceLoop = setInterval(timeReferenceSendData, sendInterval);
    };
    function timeReferenceStopSending() {
        if (timeReferenceLoop != 0) {
            clearInterval(timeReferenceLoop);
        }
        socket.sendBuffer = [];  // empty buffer to stop sending
    };

    socket.on("time_reference_set_interval", function(interval, cb) {
        timeReferenceStopSending();
        if (interval >= 0) {
            timeReferenceStartSending(interval);
        }
    });
}

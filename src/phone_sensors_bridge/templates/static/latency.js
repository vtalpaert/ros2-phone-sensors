function registerLatencyMeasurement(socket, logMessage) {
    const N = 100;
    var latencySamples = [];
    var offsetSamples = [];
    var pendingMeasurement = false;

    socket.on("latency_pong", function(data) {
        if (!pendingMeasurement) return;
        var t2 = Date.now();
        var rtt = t2 - data.t1;
        latencySamples.push(rtt / 2);
        offsetSamples.push(data.t_server_ms - (data.t1 + t2) / 2);

        if (latencySamples.length < N) {
            socket.emit("latency_ping", { t1: Date.now() });
        } else {
            pendingMeasurement = false;

            function stats(arr) {
                var mean = arr.reduce(function(a, b) { return a + b; }, 0) / arr.length;
                return { mean: mean, min: Math.min.apply(null, arr), max: Math.max.apply(null, arr) };
            }

            var lat = stats(latencySamples);
            var off = stats(offsetSamples);

            var msg = "Latency (N=" + N + "): mean=" + lat.mean.toFixed(2) + " ms, min=" + lat.min.toFixed(2) + " ms, max=" + lat.max.toFixed(2) + " ms"
                    + " | Clock offset (ROS-client): mean=" + off.mean.toFixed(2) + " ms, min=" + off.min.toFixed(2) + " ms, max=" + off.max.toFixed(2) + " ms";
            logMessage(msg);
            socket.emit("info", msg);
        }
    });

    return function triggerLatencyMeasurement() {
        if (pendingMeasurement) {
            logMessage("Latency measurement already in progress");
            return;
        }
        latencySamples = [];
        offsetSamples = [];
        pendingMeasurement = true;
        logMessage("Starting latency measurement (" + N + " samples)...");
        socket.emit("latency_ping", { t1: Date.now() });
    };
}

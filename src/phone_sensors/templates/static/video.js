function registerVideoFunctions(socket, window, videoElement) {
    var videoSource;
    var videoLabel;
    
    function getDevices() {
        return navigator.mediaDevices.enumerateDevices();
    }

    function gotDevices(deviceInfos) {
        window.deviceInfos = deviceInfos; // make available to console
        for (const deviceInfo of deviceInfos) {
            if (deviceInfo.kind === 'videoinput') {
                socket.emit("data", {
                    date_ms: Date.now(),
                    device_info: deviceInfo.toJSON()
                });
                if (deviceInfo.label === videoLabel) {
                    videoSource = deviceInfo.deviceId;
                    socket.emit("info", "Found matching label " + videoLabel + " for deviceId " + videoSource);
                }
            }
        }
    }

    function getStream() {
        if (window.stream) {
            window.stream.getTracks().forEach(track => {
                socket.emit("info", "Stopping track " + track.label);
                track.stop();
            });
        }
        // width: { ideal: 4096 },
        // height: { ideal: 2160 } 
        const constraints = {
            video: {deviceId: videoSource ? {exact: videoSource} : undefined}
        };
        socket.emit("info", "Opening stream with constraints " + JSON.stringify(constraints));
        return navigator.mediaDevices.getUserMedia(constraints).then(gotStream).catch(handleError);
    }

    function gotStream(stream) {
        window.stream = stream;
        socket.emit("info", "media gotStream");
        videoElement.srcObject = stream;
    }

    function handleError(error) {
        socket.emit("error", "getUserMedia " + error.toString());
    }

    function triggerStreamToDeviceInfos() {
        socket.emit("info", "trigger device infos");
        getStream().then(getDevices).then(gotDevices);
    }

    socket.on("camera_device_label", function(label, cb) {
        socket.emit("info", "You selected device with label " + label)
        videoLabel = label;
        getStream().then(getDevices).then(gotDevices);
        let waitTime = 10000;
        socket.emit("info", "Waiting " + waitTime + "ms to restart video with your label");
        setTimeout(getStream, waitTime);
    });
}

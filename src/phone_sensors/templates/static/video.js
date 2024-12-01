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
        return new Promise((resolve, reject) => {
            if (window.stream) {
                window.stream.getTracks().forEach(track => {
                    socket.emit("info", "Stopping track " + track.label);
                    track.stop();
                });
                // Add delay after stopping tracks
                setTimeout(() => {
                    // width: { ideal: 4096 },
                    // height: { ideal: 2160 } 
                    const constraints = {
                        video: {deviceId: videoSource ? {exact: videoSource} : undefined}
                    };
                    socket.emit("info", "Opening stream with constraints " + JSON.stringify(constraints));
                    navigator.mediaDevices.getUserMedia(constraints)
                        .then(gotStream)
                        .then(resolve)
                        .catch(error => {
                            handleError(error);
                            reject(error);
                        });
                }, 500); // 500ms delay before starting new stream
            } else {
                const constraints = {
                    video: {deviceId: videoSource ? {exact: videoSource} : undefined}
                };
                socket.emit("info", "Opening initial stream with constraints " + JSON.stringify(constraints));
                navigator.mediaDevices.getUserMedia(constraints)
                    .then(gotStream)
                    .then(resolve)
                    .catch(error => {
                        handleError(error);
                        reject(error);
                    });
            }
        });
    }

    function gotStream(stream) {
        window.stream = stream;
        socket.emit("info", "media gotStream");
        videoElement.srcObject = stream;
        
        // Create canvas for frame capture
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        
        // Set capture interval (e.g., 30fps)
        setInterval(() => {
            canvas.width = videoElement.videoWidth;
            canvas.height = videoElement.videoHeight;
            context.drawImage(videoElement, 0, 0, canvas.width, canvas.height);
            
            // Convert to base64 and send through websocket
            const frame = canvas.toDataURL('image/jpeg', 0.7);
            socket.emit("data", {
                date_ms: Date.now(),
                video_frame: frame
            });
        }, 33); // ~30fps
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
        getStream()
            .then(getDevices)
            .then(gotDevices)
            .then(() => {
                let waitTime = 1000;
                socket.emit("info", "Waiting " + waitTime + "ms to restart video with your label");
                return new Promise(resolve => setTimeout(resolve, waitTime));
            })
            .then(() => getStream())
            .catch(error => {
                socket.emit("error", "Error in camera setup sequence: " + error.toString());
            });
    });
}

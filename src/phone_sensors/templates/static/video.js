function registerVideoFunctions(socket, window, videoElement) {
    var videoSource;
    var videoLabel;
    var captureLoop = 0;

    socket.on("video_fps", function (value) {
        window.videoFps = value;
        socket.emit("info", "Video FPS setting: " + value);
    });

    socket.on("video_width", function (value) {
        window.videoWidth = value;
        socket.emit("info", "Video width setting: " + value);
    });

    socket.on("video_height", function (value) {
        window.videoHeight = value;
        socket.emit("info", "Video height setting: " + value);
    });

    socket.on("video_compression", function (value) {
        window.videoCompression = value;
        socket.emit("info", "Video compression setting: " + value);
    });

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
                    if (!videoSource) {
                        socket.emit("error", "No available label to match the required " + videoLabel);
                    } else {
                        const constraints = {
                            video: {
                                deviceId: { exact: videoSource },
                                // Note: width/height are in landscape orientation
                                width: { ideal: window.videoWidth || 1280 },  // horizontal resolution
                                height: { ideal: window.videoHeight || 720 }  // vertical resolution
                            }
                        };
                        socket.emit("info", "Opening stream " + videoLabel + " with constraints " + JSON.stringify(constraints));
                        navigator.mediaDevices.getUserMedia(constraints)
                            .then(gotStream)
                            .then(resolve)
                            .catch(error => {
                                handleError(error);
                                reject(error);
                            });
                    }
                }, 500); // 500ms delay before starting new stream
            } else {
                const constraints = {
                    video: { deviceId: videoSource ? { exact: videoSource } : undefined }
                };
                socket.emit("info", "Opening initial stream to detect all existing labels with constraints " + JSON.stringify(constraints));
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
        // First time we open the stream, hide preview
        if (!window.firstStreamOpened) {
            window.firstStreamOpened = true;
            videoElement.style.display = 'none';
        } else {
            if (window.videoFps > 0) {
                videoElement.style.display = 'block';
                videoElement.srcObject = stream;

                // Create canvas for frame capture
                const canvas = document.createElement('canvas');
                const context = canvas.getContext('2d');

                // Set capture interval based on FPS parameter
                const interval = Math.floor(1000 / (window.videoFps || 30));
                if (captureLoop !== 0) {
                    clearInterval(captureLoop);
                    captureLoop = 0;
                }
                captureLoop = setInterval(() => {
                    canvas.width = videoElement.videoWidth;
                    canvas.height = videoElement.videoHeight;
                    context.drawImage(videoElement, 0, 0, canvas.width, canvas.height);

                    // Only send if frame is not empty
                    if (canvas.width > 0 && canvas.height > 0) {
                        // Convert to base64 and send through websocket
                        const frame = canvas.toDataURL('image/jpeg', window.videoCompression || 0.7);
                        socket.emit("data", {
                            date_ms: Date.now(),
                            video_frame: frame
                        });
                    }
                }, interval);
                socket.emit("info", "Video effective interval (ms): " + interval);
            } else {
                videoElement.style.display = 'none';
            }
        }


    }

    function handleError(error) {
        socket.emit("error", "getUserMedia " + error.toString());
    }

    function triggerStreamToDeviceInfos() {
        socket.emit("info", "trigger device infos");
        getStream().then(getDevices).then(gotDevices);
    }

    socket.on("camera_device_label", function (label, cb) {
        socket.emit("info", "You selected device with label " + label)
        videoLabel = label;
        // Initial delay to ensure we receive parameters
        setTimeout(() => {
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
        }, 1000);
    });
}

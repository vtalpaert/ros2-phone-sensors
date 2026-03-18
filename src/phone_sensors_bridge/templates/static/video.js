function registerVideoFunctions(socket, videoElement, cameraId) {
    var videoSource;
    var videoLabel;
    var captureLoop = 0;
    var localVideoFps, localVideoWidth, localVideoHeight, localVideoCompression;
    var localStream = null;

    socket.on(cameraId + "_video_fps", function (value) {
        localVideoFps = value;
        socket.emit("info", cameraId + " video FPS setting: " + value);
    });

    socket.on(cameraId + "_video_width", function (value) {
        localVideoWidth = value;
        socket.emit("info", cameraId + " video width setting: " + value);
    });

    socket.on(cameraId + "_video_height", function (value) {
        localVideoHeight = value;
        socket.emit("info", cameraId + " video height setting: " + value);
    });

    socket.on(cameraId + "_video_compression", function (value) {
        localVideoCompression = value;
        socket.emit("info", cameraId + " video compression setting: " + value);
    });

    function getDevices() {
        return navigator.mediaDevices.enumerateDevices();
    }

    function gotDevices(deviceInfos) {
        socket.emit("debug", cameraId + " gotDevices called with " + deviceInfos.length + " devices");
        let videoInputCount = 0;
        for (const deviceInfo of deviceInfos) {
            if (deviceInfo.kind === 'videoinput') {
                videoInputCount++;
                socket.emit("debug", cameraId + " video device found: label='" + deviceInfo.label + "' deviceId=" + deviceInfo.deviceId);
                socket.emit("data", {
                    date_us: Math.round((performance.timeOrigin + performance.now()) * 1000),
                    camera_id: cameraId,
                    device_info: deviceInfo.toJSON()
                });
                if (deviceInfo.label === videoLabel) {
                    videoSource = deviceInfo.deviceId;
                    socket.emit("info", cameraId + " found matching label '" + videoLabel + "' for deviceId " + videoSource);
                }
            }
        }
        socket.emit("debug", cameraId + " total video inputs found: " + videoInputCount + ", looking for: '" + videoLabel + "', videoSource=" + videoSource);
    }

    function getStream() {
        socket.emit("debug", cameraId + " getStream called, localStream exists: " + !!localStream + ", videoSource=" + videoSource + ", videoLabel='" + videoLabel + "'");
        return new Promise((resolve, reject) => {
            if (localStream) {
                socket.emit("debug", cameraId + " stopping existing stream with " + localStream.getTracks().length + " tracks");
                localStream.getTracks().forEach(track => {
                    socket.emit("info", cameraId + " stopping track " + track.label);
                    track.stop();
                });
                // Add delay after stopping tracks
                setTimeout(() => {
                    socket.emit("debug", cameraId + " after 500ms delay, localVideoFps=" + localVideoFps + ", videoSource=" + videoSource);
                    if (localVideoFps <= 0) {
                        socket.emit("info", cameraId + " video stream disabled due to negative video FPS " + localVideoFps);
                        resolve();
                    } else if (!videoSource) {
                        socket.emit("error", cameraId + " no available label to match the required '" + videoLabel + "'");
                        resolve();
                    } else {
                        const constraints = {
                            video: {
                                deviceId: { exact: videoSource },
                                // Note: width/height are in landscape orientation
                                width: { ideal: localVideoWidth || 1280 },  // horizontal resolution
                                height: { ideal: localVideoHeight || 720 }  // vertical resolution
                            }
                        };
                        socket.emit("info", cameraId + " opening stream '" + videoLabel + "' with constraints " + JSON.stringify(constraints));
                        Promise.race([
                            navigator.mediaDevices.getUserMedia(constraints),
                            new Promise((_, reject) =>
                                setTimeout(() => reject(new Error('getUserMedia timeout after 5s')), 5000)
                            )
                        ])
                            .then(stream => {
                                socket.emit("debug", cameraId + " getUserMedia succeeded for '" + videoLabel + "'");
                                return gotStream(stream);
                            })
                            .then(resolve)
                            .catch(error => {
                                socket.emit("error", cameraId + " getUserMedia failed or timed out for '" + videoLabel + "': " + error.message);
                                handleError(error);
                                reject(error);
                            });
                    }
                }, 500); // 500ms delay before starting new stream
            } else {
                const constraints = {
                    video: { deviceId: videoSource ? { exact: videoSource } : undefined }
                };
                socket.emit("info", cameraId + " opening initial stream to detect all existing labels with constraints " + JSON.stringify(constraints));
                Promise.race([
                    navigator.mediaDevices.getUserMedia(constraints),
                    new Promise((_, reject) =>
                        setTimeout(() => reject(new Error('getUserMedia timeout after 5s')), 5000)
                    )
                ])
                    .then(stream => {
                        socket.emit("debug", cameraId + " initial getUserMedia succeeded");
                        return gotStream(stream);
                    })
                    .then(resolve)
                    .catch(error => {
                        socket.emit("error", cameraId + " initial getUserMedia failed or timed out: " + error.message);
                        handleError(error);
                        reject(error);
                    });
            }
        });
    }

    function gotStream(stream) {
        socket.emit("debug", cameraId + " gotStream called, stream has " + stream.getTracks().length + " tracks, localVideoFps=" + localVideoFps);
        var firstStreamOpened = videoElement.dataset.firstStreamOpened === 'true';
        // First time we open the stream, hide preview
        localStream = stream;
        if (!firstStreamOpened) {
            socket.emit("debug", cameraId + " first stream opened, hiding video element");
            videoElement.dataset.firstStreamOpened = 'true';
            videoElement.style.display = 'none';
        } else {
            socket.emit("debug", cameraId + " subsequent stream opened, localVideoFps=" + localVideoFps);
            if (localVideoFps > 0) {
                socket.emit("debug", cameraId + " starting video capture, showing video element");
                videoElement.style.display = 'block';
                videoElement.srcObject = stream;

                // Create canvas for frame capture
                const canvas = document.createElement('canvas');
                const context = canvas.getContext('2d');

                // Set capture interval based on FPS parameter
                const interval = Math.floor(1000 / (localVideoFps || 30));
                if (captureLoop !== 0) {
                    socket.emit("debug", cameraId + " clearing existing capture loop");
                    clearInterval(captureLoop);
                    captureLoop = 0;
                }
                socket.emit("debug", cameraId + " starting capture loop with interval " + interval + "ms");
                captureLoop = setInterval(() => {
                    canvas.width = videoElement.videoWidth;
                    canvas.height = videoElement.videoHeight;
                    context.drawImage(videoElement, 0, 0, canvas.width, canvas.height);

                    // Only send if frame is not empty
                    if (canvas.width > 0 && canvas.height > 0) {
                        const dateMs = BigInt(Math.round((performance.timeOrigin + performance.now()) * 1000));
                        canvas.toBlob((blob) => {
                            blob.arrayBuffer().then(jpegBuffer => {
                                // Prepend 8-byte little-endian uint64 timestamp header
                                const buf = new Uint8Array(8 + jpegBuffer.byteLength);
                                new DataView(buf.buffer).setBigUint64(0, dateMs, true);
                                buf.set(new Uint8Array(jpegBuffer), 8);
                                socket.emit(cameraId + "_frame", buf.buffer);
                            });
                        }, 'image/jpeg', localVideoCompression || 0.7);
                    }
                }, interval);
                socket.emit("info", cameraId + " video effective interval (ms): " + interval);
            } else {
                socket.emit("debug", cameraId + " localVideoFps <= 0, hiding video element");
                videoElement.style.display = 'none';
            }
        }
    }

    function handleError(error) {
        socket.emit("error", cameraId + " getUserMedia error: " + error.name + " - " + error.message + " (" + error.toString() + ")");
    }

    socket.on(cameraId + "_device_label", function (label) {
        socket.emit("info", cameraId + " device label received: " + label);
        videoLabel = label;
    });

    socket.on("stream_stop", function () {
        socket.emit("debug", cameraId + " stream_stop received");
        if (captureLoop !== 0) {
            clearInterval(captureLoop);
            captureLoop = 0;
        }
        if (localStream) {
            localStream.getTracks().forEach(track => track.stop());
            localStream = null;
        }
        videoElement.style.display = 'none';
        videoElement.dataset.firstStreamOpened = 'false';
    });

    socket.on("stream_start", function () {
        if (!videoLabel || localVideoFps <= 0) return;
        socket.emit("debug", cameraId + " starting camera setup (stream_start)");
        getStream()
            .then(() => {
                socket.emit("debug", cameraId + " first getStream completed, calling getDevices");
                return getDevices();
            })
            .then(devices => {
                socket.emit("debug", cameraId + " getDevices completed, calling gotDevices");
                return gotDevices(devices);
            })
            .then(() => {
                let waitTime = 1000;
                socket.emit("info", cameraId + " waiting " + waitTime + "ms to restart video with label");
                return new Promise(resolve => setTimeout(resolve, waitTime));
            })
            .then(() => {
                socket.emit("debug", cameraId + " after wait, calling getStream again with videoSource=" + videoSource);
                return getStream();
            })
            .catch(error => {
                socket.emit("error", cameraId + " error in camera setup: " + error.toString());
            });
    });
}

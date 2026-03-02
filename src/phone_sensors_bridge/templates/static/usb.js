function registerUSBFunctions(socket) {
    var usbEnabled = false;
    var usbDeviceType = "cdc";  // "cdc" or "cp2102"
    var usbBaud = 115200;
    var usbDevice = null;
    var usbEpIn = null;
    var usbEpOut = null;
    var usbReading = false;

    // Returns a 4-byte little-endian ArrayBuffer for a baud rate (CP2102 SET_BAUDRATE)
    function baudToBytes(baud) {
        var buf = new ArrayBuffer(4);
        new DataView(buf).setUint32(0, baud, true);
        return buf;
    }

    function updateUI() {
        var section = document.getElementById("usb-section");
        if (section) {
            section.style.display = usbEnabled ? "block" : "none";
        }
    }

    async function usbReadLoop() {
        usbReading = true;
        while (usbReading) {
            try {
                var result = await usbDevice.transferIn(usbEpIn, 64);
                if (result.data && result.data.byteLength > 0) {
                    socket.emit("usb_rx_data", result.data.buffer);
                }
            } catch (e) {
                if (usbReading) socket.emit("warn", "USB RX error: " + e.toString());
                usbReading = false;
            }
        }
    }

    async function openDevice(device) {
        usbDevice = device;
        try {
            await device.open();
            await device.selectConfiguration(1);

            if (usbDeviceType === "cdc") {
                await device.claimInterface(0);  // CDC communication
                await device.claimInterface(1);  // CDC data

                // SET_LINE_CODING: baud rate + 8N1
                var coding = new ArrayBuffer(7);
                var view = new DataView(coding);
                view.setUint32(0, usbBaud, true);
                view.setUint8(4, 0);  // 1 stop bit
                view.setUint8(5, 0);  // no parity
                view.setUint8(6, 8);  // 8 data bits
                await device.controlTransferOut({
                    requestType: "class", recipient: "interface",
                    request: 0x20, value: 0, index: 0
                }, coding);

                // SET_CONTROL_LINE_STATE: raise DTR so the board's while(!Serial) unblocks
                await device.controlTransferOut({
                    requestType: "class", recipient: "interface",
                    request: 0x22, value: 0x01, index: 0
                }, new ArrayBuffer(0));

                var altCdc = device.configuration.interfaces[1].alternates[0];
                for (var epCdc of altCdc.endpoints) {
                    if (epCdc.type === "bulk" && epCdc.direction === "in")  usbEpIn  = epCdc.endpointNumber;
                    if (epCdc.type === "bulk" && epCdc.direction === "out") usbEpOut = epCdc.endpointNumber;
                }

            } else if (usbDeviceType === "cp2102") {
                await device.claimInterface(0);

                // IFC_ENABLE
                await device.controlTransferOut({
                    requestType: "vendor", recipient: "interface",
                    request: 0x00, value: 0x0001, index: 0
                }, new ArrayBuffer(0));

                // SET_BAUDRATE: 4-byte LE
                await device.controlTransferOut({
                    requestType: "vendor", recipient: "interface",
                    request: 0x1E, value: 0, index: 0
                }, baudToBytes(usbBaud));

                // SET_LINE_CTL: 8N1
                await device.controlTransferOut({
                    requestType: "vendor", recipient: "interface",
                    request: 0x03, value: 0x0800, index: 0
                }, new ArrayBuffer(0));

                var altCp2 = device.configuration.interfaces[0].alternates[0];
                for (var epCp2 of altCp2.endpoints) {
                    if (epCp2.type === "bulk" && epCp2.direction === "in")  usbEpIn  = epCp2.endpointNumber;
                    if (epCp2.type === "bulk" && epCp2.direction === "out") usbEpOut = epCp2.endpointNumber;
                }

            } else {
                socket.emit("error", "Unknown usb_device_type: " + usbDeviceType);
                usbDevice = null;
                return;
            }

            socket.emit("info", "USB opened: type=" + usbDeviceType + " baud=" + usbBaud + " epIn=" + usbEpIn + " epOut=" + usbEpOut);
            usbReadLoop();
        } catch (e) {
            socket.emit("error", "USB open error: " + e.toString());
            usbDevice = null;
        }
    }

    async function closeDevice() {
        usbReading = false;
        if (!usbDevice) return;
        try {
            if (usbDeviceType === "cdc") {
                // DTR off
                await usbDevice.controlTransferOut({
                    requestType: "class", recipient: "interface",
                    request: 0x22, value: 0x00, index: 0
                }, new ArrayBuffer(0));
                await usbDevice.releaseInterface(1);
                await usbDevice.releaseInterface(0);
            } else if (usbDeviceType === "cp2102") {
                // IFC_DISABLE
                await usbDevice.controlTransferOut({
                    requestType: "vendor", recipient: "interface",
                    request: 0x00, value: 0x0000, index: 0
                }, new ArrayBuffer(0));
                await usbDevice.releaseInterface(0);
            }
            await usbDevice.close();
            socket.emit("info", "USB disconnected");
        } catch (e) {
            socket.emit("warn", "USB close error: " + e.toString());
        }
        usbDevice = null;
        usbEpIn = null;
        usbEpOut = null;
    }

    socket.on("usb_enabled", function(value) {
        usbEnabled = value;
        updateUI();
    });

    socket.on("usb_device_type", function(value) {
        usbDeviceType = value;
    });

    socket.on("usb_baud", function(value) {
        usbBaud = value;
    });

    // usb_tx_data: server → browser → USB device
    socket.on("usb_tx_data", async function(data) {
        if (usbEpOut === null) return;
        try {
            await usbDevice.transferOut(usbEpOut, new Uint8Array(data));
        } catch (e) {
            socket.emit("warn", "USB TX error: " + e.toString());
        }
    });

    if (!navigator.usb) {
        socket.emit("warn", "WebUSB is not supported in this browser.");
        return;
    }

    navigator.usb.addEventListener("disconnect", function(event) {
        if (usbDevice && event.device === usbDevice) {
            usbReading = false;
            usbDevice = null;
            usbEpIn = null;
            usbEpOut = null;
            socket.emit("warn", "USB device disconnected unexpectedly");
        }
    });

    document.getElementById("usb-connect").addEventListener("click", function() {
        navigator.usb.requestDevice({ filters: [] })
            .then(function(device) { return openDevice(device); })
            .catch(function(e) { socket.emit("warn", "USB request cancelled or failed: " + e.toString()); });
    });

    document.getElementById("usb-disconnect").addEventListener("click", function() {
        closeDevice();
    });
}

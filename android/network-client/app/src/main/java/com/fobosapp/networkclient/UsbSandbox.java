package com.fobosapp.networkclient;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Build;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;

final class UsbSandbox {
    interface Listener {
        void onUsbLog(String message);
    }

    static final int FOBOS_VENDOR_ID = 0x16d0;
    static final int FOBOS_PRODUCT_ID = 0x132e;
    static final int FOBOS_STANDARD_BCD_DEVICE = 0x0000;
    static final int FOBOS_AGILE_BCD_DEVICE = 0x0101;

    private static final String ACTION_USB_PERMISSION =
            "com.fobosapp.networkclient.USB_PERMISSION";

    private final Activity activity;
    private final UsbManager usbManager;
    private final Listener listener;
    private final PendingIntent permissionIntent;
    private final BroadcastReceiver usbReceiver;
    private boolean receiverRegistered;

    UsbSandbox(Activity activity, Listener listener) {
        this.activity = activity;
        this.listener = listener;
        this.usbManager = (UsbManager) activity.getSystemService(Context.USB_SERVICE);
        int flags = PendingIntent.FLAG_UPDATE_CURRENT;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            flags |= PendingIntent.FLAG_MUTABLE;
        }
        this.permissionIntent = PendingIntent.getBroadcast(
                activity,
                0,
                new Intent(ACTION_USB_PERMISSION).setPackage(activity.getPackageName()),
                flags);
        this.usbReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                handleUsbBroadcast(intent);
            }
        };
    }

    void start() {
        if (receiverRegistered) {
            return;
        }
        IntentFilter filter = new IntentFilter();
        filter.addAction(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            activity.registerReceiver(usbReceiver, filter, Context.RECEIVER_NOT_EXPORTED);
        } else {
            activity.registerReceiver(usbReceiver, filter);
        }
        receiverRegistered = true;
    }

    void stop() {
        if (!receiverRegistered) {
            return;
        }
        try {
            activity.unregisterReceiver(usbReceiver);
        } catch (IllegalArgumentException ignored) {
            // Receiver was already gone during Activity teardown.
        }
        receiverRegistered = false;
    }

    String scanReport() {
        List<UsbDevice> devices = sortedDevices();
        if (devices.isEmpty()) {
            return "[USB] no devices visible to Android USB host API";
        }
        StringBuilder report = new StringBuilder();
        report.append("[USB] visible devices: ").append(devices.size());
        for (UsbDevice device : devices) {
            appendDeviceReport(report, device);
        }
        return report.toString();
    }

    String requestPermissionForBestDevice() {
        UsbDevice device = bestDevice();
        if (device == null) {
            return "[USB] no USB device available for permission request";
        }
        if (usbManager.hasPermission(device)) {
            return probeOpenDevice(device);
        }
        usbManager.requestPermission(device, permissionIntent);
        return "[USB] permission requested for " + shortDeviceName(device);
    }

    private void handleUsbBroadcast(Intent intent) {
        if (intent == null) {
            return;
        }
        String action = intent.getAction();
        UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
        if (ACTION_USB_PERMISSION.equals(action)) {
            boolean granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
            if (device == null) {
                log("[USB] permission result without device");
                return;
            }
            if (!granted) {
                log("[USB] permission denied for " + shortDeviceName(device));
                return;
            }
            log(probeOpenDevice(device));
            return;
        }
        if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
            log("[USB] attached: " + (device != null ? shortDeviceName(device) : "unknown device"));
            log(scanReport());
            return;
        }
        if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
            log("[USB] detached: " + (device != null ? shortDeviceName(device) : "unknown device"));
        }
    }

    private UsbDevice bestDevice() {
        List<UsbDevice> devices = sortedDevices();
        return devices.isEmpty() ? null : devices.get(0);
    }

    private List<UsbDevice> sortedDevices() {
        HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
        List<UsbDevice> devices = new ArrayList<>(deviceList.values());
        devices.sort(Comparator
                .comparing((UsbDevice device) -> !isFobosCandidate(device))
                .thenComparing(UsbDevice::getDeviceName));
        return devices;
    }

    private void appendDeviceReport(StringBuilder report, UsbDevice device) {
        report.append('\n')
                .append(isFobosCandidate(device) ? "  * " : "  - ")
                .append(shortDeviceName(device))
                .append('\n')
                .append("    class ").append(formatByte(device.getDeviceClass()))
                .append(" subclass ").append(formatByte(device.getDeviceSubclass()))
                .append(" protocol ").append(formatByte(device.getDeviceProtocol()))
                .append('\n')
                .append("    manufacturer: ").append(safeManufacturer(device))
                .append(" product: ").append(safeProduct(device))
                .append(" serial: ").append(safeSerial(device))
                .append('\n')
                .append("    permission: ").append(usbManager.hasPermission(device) ? "granted" : "not granted")
                .append(" interfaces: ").append(device.getInterfaceCount());
        for (int i = 0; i < device.getInterfaceCount(); ++i) {
            UsbInterface usbInterface = device.getInterface(i);
            report.append('\n')
                    .append("      if").append(i)
                    .append(" id ").append(usbInterface.getId())
                    .append(" class ").append(formatByte(usbInterface.getInterfaceClass()))
                    .append(" subclass ").append(formatByte(usbInterface.getInterfaceSubclass()))
                    .append(" protocol ").append(formatByte(usbInterface.getInterfaceProtocol()))
                    .append(" endpoints ").append(usbInterface.getEndpointCount());
            for (int j = 0; j < usbInterface.getEndpointCount(); ++j) {
                UsbEndpoint endpoint = usbInterface.getEndpoint(j);
                report.append('\n')
                        .append("        ep").append(j)
                        .append(' ').append(endpointDirection(endpoint))
                        .append(' ').append(endpointType(endpoint))
                        .append(" addr ").append(formatByte(endpoint.getAddress()))
                        .append(" maxPacket ").append(endpoint.getMaxPacketSize())
                        .append(" interval ").append(endpoint.getInterval());
            }
        }
    }

    private String probeOpenDevice(UsbDevice device) {
        StringBuilder report = new StringBuilder();
        report.append("[USB] permission granted for ").append(shortDeviceName(device));
        UsbDeviceConnection connection = null;
        try {
            connection = usbManager.openDevice(device);
            if (connection == null) {
                report.append("\n[USB] openDevice returned null");
                return report.toString();
            }
            byte[] rawDescriptors = connection.getRawDescriptors();
            report.append("\n[USB] openDevice OK");
            if (rawDescriptors != null) {
                report.append(", raw descriptors ").append(rawDescriptors.length).append(" bytes");
                int bcdDevice = bcdDeviceFromRawDescriptors(rawDescriptors);
                if (bcdDevice >= 0) {
                    report.append(", bcdDevice ").append(formatWord(bcdDevice));
                }
            }
            if (isFobosCandidate(device)) {
                report.append("\n[USB] Fobos candidate detected: ")
                        .append(fobosApiHint(rawDescriptors));
            }
        } catch (SecurityException e) {
            report.append("\n[USB] open blocked by Android security: ").append(e.getMessage());
        } catch (RuntimeException e) {
            report.append("\n[USB] open failed: ").append(e.getMessage());
        } finally {
            if (connection != null) {
                connection.close();
            }
        }
        return report.toString();
    }

    private boolean isFobosCandidate(UsbDevice device) {
        return device.getVendorId() == FOBOS_VENDOR_ID &&
                device.getProductId() == FOBOS_PRODUCT_ID;
    }

    private String fobosApiHint(byte[] rawDescriptors) {
        int bcdDevice = bcdDeviceFromRawDescriptors(rawDescriptors);
        if (bcdDevice == FOBOS_AGILE_BCD_DEVICE) {
            return "agile API expected";
        }
        if (bcdDevice == FOBOS_STANDARD_BCD_DEVICE) {
            return "standard API expected";
        }
        if (bcdDevice >= 0) {
            return "unknown bcdDevice " + formatWord(bcdDevice);
        }
        return "bcdDevice unavailable";
    }

    private int bcdDeviceFromRawDescriptors(byte[] rawDescriptors) {
        if (rawDescriptors == null || rawDescriptors.length < 14) {
            return -1;
        }
        return (rawDescriptors[12] & 0xff) | ((rawDescriptors[13] & 0xff) << 8);
    }

    private String shortDeviceName(UsbDevice device) {
        return String.format(Locale.US,
                "%s vid:pid %s:%s version %s%s",
                device.getDeviceName(),
                formatWord(device.getVendorId()),
                formatWord(device.getProductId()),
                safeVersion(device),
                isFobosCandidate(device) ? " (Fobos)" : "");
    }

    private String safeVersion(UsbDevice device) {
        try {
            String value = device.getVersion();
            return value != null ? value : "?";
        } catch (RuntimeException e) {
            return "?";
        }
    }

    private String safeManufacturer(UsbDevice device) {
        try {
            String value = device.getManufacturerName();
            return value != null ? value : "?";
        } catch (SecurityException e) {
            return "<permission needed>";
        }
    }

    private String safeProduct(UsbDevice device) {
        try {
            String value = device.getProductName();
            return value != null ? value : "?";
        } catch (SecurityException e) {
            return "<permission needed>";
        }
    }

    private String safeSerial(UsbDevice device) {
        try {
            String value = device.getSerialNumber();
            return value != null ? value : "?";
        } catch (SecurityException e) {
            return "<permission needed>";
        }
    }

    private String endpointDirection(UsbEndpoint endpoint) {
        return endpoint.getDirection() == UsbConstants.USB_DIR_IN ? "IN" : "OUT";
    }

    private String endpointType(UsbEndpoint endpoint) {
        switch (endpoint.getType()) {
            case UsbConstants.USB_ENDPOINT_XFER_CONTROL:
                return "control";
            case UsbConstants.USB_ENDPOINT_XFER_ISOC:
                return "iso";
            case UsbConstants.USB_ENDPOINT_XFER_BULK:
                return "bulk";
            case UsbConstants.USB_ENDPOINT_XFER_INT:
                return "interrupt";
            default:
                return "type " + endpoint.getType();
        }
    }

    private String formatByte(int value) {
        return String.format(Locale.US, "0x%02x", value & 0xff);
    }

    private String formatWord(int value) {
        return String.format(Locale.US, "0x%04x", value & 0xffff);
    }

    private void log(String message) {
        if (listener != null) {
            listener.onUsbLog(message);
        }
    }
}

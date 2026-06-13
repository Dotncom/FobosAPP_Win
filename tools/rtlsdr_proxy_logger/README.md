# RTL-SDR proxy logger

This tool builds a diagnostic `rtlsdr.dll` proxy for comparing how another SDR application
talks to the RTL-SDR runtime.

Usage with uSDR:

1. Close uSDR.
2. Open the folder that contains `uSDR.exe`.
3. Rename the real `rtlsdr.dll` in that folder to `rtlsdr_real.dll`.
4. Copy the proxy `rtlsdr.dll` built from this folder into the same folder.
5. Start uSDR and use the RTL-SDR device normally.
6. Read `rtlsdr_proxy.log` in the same folder.

The proxy logs high-level API calls such as open, sample rate, center frequency, gain,
buffer reset, read_async begin/end, cancel and close. It intentionally does not log raw IQ
buffers because that would create huge logs and slow the receiver path.

To restore uSDR, close it, delete the proxy `rtlsdr.dll`, and rename `rtlsdr_real.dll`
back to `rtlsdr.dll`.

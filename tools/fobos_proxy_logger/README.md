# Fobos proxy logger

This tool builds diagnostic proxy DLLs for comparing how another SDR application talks
to the Fobos runtime.

It produces two DLLs:

- `fobos.dll`, which forwards to `fobos_real.dll`
- `fobos_sdr.dll`, which forwards to `fobos_sdr_real.dll`

Usage with uSDR:

1. Close uSDR.
2. Open the folder that contains `uSDR.exe`.
3. Rename the real `fobos.dll` to `fobos_real.dll`.
4. Rename the real `fobos_sdr.dll` to `fobos_sdr_real.dll`.
5. Copy the proxy `fobos.dll` and `fobos_sdr.dll` from this folder into the same folder.
6. Start uSDR and use the Fobos receiver normally.
7. Read `fobos_proxy.log` and `fobos_sdr_proxy.log` in the same folder.

The proxies log high-level API calls such as open, sample rate, center frequency,
gain, scanning, read_async begin/end, cancel and close. They intentionally do not log
raw IQ buffers because that would create huge logs and slow the receiver path.

To restore uSDR, close it, delete the proxy DLLs, and rename the real DLLs back to
their original names.

# bladeRF native RX beta

FobosAPP 4.3.2-beta adds an experimental native bladeRF receive backend.
It does not use SoapySDR. The app dynamically loads Nuand `libbladeRF`
at runtime and converts bladeRF `SC16_Q11` IQ samples into the existing
FobosAPP float-IQ pipeline.

## Runtime

The Windows beta package includes:

- `bladerf/bladeRF.dll`
- `bladerf/libusb-1.0.dll`
- bladeRF and libusb license notice files

These files come from the official Nuand Windows installer
`bladeRF-win-installer-2025.10.exe`, published with the Nuand bladeRF
2025.10 release.

Upstream:

- https://github.com/Nuand/bladeRF
- https://github.com/Nuand/bladeRF/releases/tag/2025.10
- https://www.nuand.com/win_installers/

## Licensing

Nuand `COPYING` states that `libbladeRF` is LGPLv2.1. The bundled
`libusb-1.0.dll` is also LGPLv2.1. FobosAPP loads both dynamically and
ships license notices in the beta package.

## Current scope

Implemented:

- native bladeRF device enumeration;
- runtime `bladeRF.dll` loading;
- RX frequency, sample rate, bandwidth, and gain-mode setup;
- synchronous RX streaming through `bladerf_sync_rx`;
- conversion from interleaved `int16 I/Q` SC16_Q11 to float IQ;
- live center retune;
- Standard scan retune compatibility;
- optional detailed RX statistics when verbose logging is enabled.

Not implemented yet:

- transmit/PTT;
- MIMO channel selection;
- full gain-stage UI;
- external clock/reference controls;
- FPGA image management;
- bladeRF-specific calibration UI.

TX must be added only through the future separate `TransmitterBackend`
architecture with explicit safety interlocks and test coverage.

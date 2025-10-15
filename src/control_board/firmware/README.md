# Scout - Central Control Board

USB Operated control board for the Scout Capstone Project.

- Control interfaces are exposed over a USB CDC interface build with the IAD mechanism.
- Virtual COM port implemented for error feedback through USB CDC (IAD)

## USB Device Specifications
(exposed from the viewpoint of the host as per USB IF specifications)
- Vendor ID: 0xFFFE
- Product ID: 0x0A4D
- Virtual COM port (CDC)
    - NTF Interface Number: 0x00
    - Data Interface Number: 0x01
    - Data RX EP: 0x01
    - Data TX EP: 0x81
    - Maximum Data Packet Size: 0x40
    - NTF EP: 0x82
    - Maximum NTF Packet Size: 0x08
- Control Endpoint (CDC)
    - NTF Interface Number: 0x02
    - Data Interface Number: 0x03
    - Data RX EP: 0x03
    - Data TX EP: 0x83
    - Maximum Data Packet Size: 0x40
    - NTF EP: 0x84
    - Maximum NTF Packet Size: 0x08

## Firmware
The application firmware is located under `core`.
Documentation for working within the core is in [`core/README.md`](./core/README.md).

### Dependencies
- FreeRTOS (`FreeRTOS/`)
- CMSIS (`vendor/`)
- GNU Make
- CMake
- ARM GCC Cross-Compiler

> [!WARNING]
> You must install the `arm-none-eabi` tool-chain before attempting to build.
> This tool-chain can be found on the [ARM Developer Website](https://developer.arm.com/downloads/-/gnu-rm)

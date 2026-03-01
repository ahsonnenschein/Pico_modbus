# Pico Modbus TCP Server

A Modbus TCP server running on the [W6100-EVB-Pico2](https://docs.wiznet.io/Product/iEthernet/W6100/w6100-evb-pico2) board, which combines the Raspberry Pi RP2350 microcontroller with a WIZnet W6100 Ethernet chip.

The server exposes the board's GPIO and ADC pins over Modbus TCP on port 502, making it easy to integrate with industrial control systems such as [LinuxCNC](https://linuxcnc.org/) via `mb2hal`.

## Features

- Modbus TCP server on port 502
- DHCP for automatic IP address assignment
- MAC address derived from the Pico's unique board ID
- 8 digital inputs (GP0–GP7)
- 8 digital outputs (GP8–GP15)
- 4 ADC inputs (GP26–GP29, 12-bit resolution)

## Modbus Register Map

| Type | Address | Description |
|---|---|---|
| Coils (read/write) | 0–7 | Digital outputs GP8–GP15 |
| Discrete Inputs (read) | 0–7 | Digital inputs GP0–GP7 |
| Holding Registers (read) | 0–3 | ADC channels GP26–GP29 (0–4095) |

### Supported Function Codes

| Code | Name |
|---|---|
| 0x01 | Read Coils |
| 0x02 | Read Discrete Inputs |
| 0x03 | Read Holding Registers |
| 0x05 | Write Single Coil |
| 0x0F | Write Multiple Coils |

## Hardware

- **Board:** W6100-EVB-Pico2
- **MCU:** Raspberry Pi RP2350 (Pico2)
- **Ethernet:** WIZnet W6100

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk). The SDK is fetched automatically by CMake during the build.

```bash
mkdir build && cd build
cmake ..
make
```

This produces `pico_modbus.uf2` in the `build/` directory.

## Flashing

1. Hold the BOOTSEL button on the board while plugging in USB
2. The board appears as a USB mass storage device
3. Copy `pico_modbus.uf2` to the drive — the board reboots automatically

## Serial Output

The server prints status messages over USB serial (115200 baud). On startup it displays the assigned IP address once DHCP completes:

```
========================================
Pico Modbus TCP Server
W6100-EVB-Pico2
========================================
Waiting for DHCP...

========================================
DHCP IP Assigned!
IP Address: 192.168.1.x
Subnet:     255.255.255.0
Gateway:    192.168.1.1
Modbus TCP Server ready on port 502
========================================
```

## LinuxCNC Integration

A sample `mb2hal.ini` configuration for LinuxCNC is provided in `config/mb2hal.ini`.

## Dependencies

- [WIZnet ioLibrary_Driver](https://github.com/Wiznet/ioLibrary_Driver) (included as a git submodule)
- Raspberry Pi Pico SDK 2.1.0 (fetched automatically)

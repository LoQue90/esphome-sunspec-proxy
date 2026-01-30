# ESPHome SunSpec Proxy — Hoymiles DTU-Pro Bridge

An ESPHome custom component that reads Hoymiles microinverter data via the DTU-Pro RS-485 Modbus RTU interface, and serves it as a SunSpec-compliant Modbus TCP device for Victron GX integration.

## Architecture

```
Hoymiles µ-inverters ◄── 2.4GHz RF ──► DTU-Pro ◄── RS-485 ──► ESP32 ◄── LAN ──► Victron GX
                                                                  │
                                                           ESPHome API
                                                                  │
                                                          Home Assistant
```

## Features

- Reads per-inverter data from DTU-Pro via Hoymiles Modbus RTU protocol
- Presents aggregated data as a SunSpec Model 101/103 inverter over Modbus TCP
- Auto-detected by Victron GX (`dbus-fronius`) as a Fronius-compatible PV inverter
- All sensors auto-generated in Home Assistant via ESPHome API
- DTU diagnostic polling (serial number read) to verify RS-485 link health
- Power limiting support from Victron GX (Model 123)

## Hardware Required

- ESP32 with RS-485 transceiver (e.g. Waveshare ESP32-S3 RS485 board)
- Hoymiles DTU-Pro with RS-485 port enabled for "Remote Control/Modbus Protocol"
- RS-485 wiring: A→A, B→B, GND→C(common)

## DTU-Pro Setup

1. Open Hoymiles installer app
2. Navigate: Me → Local Install Assistant → Home → DTU Information → RS485 port setting
3. Set mode to **Remote Control/Modbus Protocol**
4. Set Modbus address (101-254, e.g. 126)
5. Baud rate is fixed at **9600 bps, 8N1**

## Installation

### Option 1: ESPHome Dashboard / HA Add-on

Add as external component in your YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Geoffn-Hub/esphome-sunspec-proxy
      ref: main
    components: [sunspec_proxy]
```

### Option 2: Local

Clone this repo into your ESPHome `components/` directory.

## Configuration

See [example-config.yaml](example-config.yaml) for a full example.

### Minimal config:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Geoffn-Hub/esphome-sunspec-proxy
      ref: main
    components: [sunspec_proxy]

uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO18
  baud_rate: 9600

sunspec_proxy:
  uart_id: uart_bus
  dtu_address: 126
  rtu_sources:
    - port: 0
      inverter_model: "HMS-2000-4T"
      name: "My Inverter"
      connected_phase: 1
```

### Supported Inverter Models

| Model | Phases | MPPT | Power |
|-------|--------|------|-------|
| HMS-300-1T to HMS-500-1T | 1 | 1 | 300-500W |
| HMS-600-2T to HMS-1000-2T | 1 | 2 | 600-1000W |
| HMS-1600-4T to HMS-2000-4T | 1 | 4 | 1600-2000W |
| HMT-1600-4T to HMT-2000-4T | 3 | 4 | 1600-2000W |
| MIT-1300-2T to MIT-6000-8T | 3 | 2-8 | 1300-6000W |

## Hoymiles Modbus Register Map

Based on the official Hoymiles Technical Note V1.2 (December 2020).

Register addresses are **byte addresses** — each Modbus register spans 2 byte addresses.
Read 20 Modbus registers (40 bytes) per port from base address `0x1000 + (port × 0x28)`.

| Register | Byte Addr | Field | Scaling | Unit |
|----------|-----------|-------|---------|------|
| 0 | 0x00-01 | Data Type | - | - |
| 0-3 | 0x01-06 | Serial Number | hex | 12-char |
| 3 | 0x07 | Port Number | - | - |
| 4 | 0x08-09 | PV Voltage | ÷10 | V |
| 5 | 0x0A-0B | PV Current | ÷100 (HM/HMS), ÷10 (MI) | A |
| 6 | 0x0C-0D | Grid Voltage | ÷10 | V |
| 7 | 0x0E-0F | Grid Frequency | ÷100 | Hz |
| 8 | 0x10-11 | PV Power | ÷10 | W |
| 9 | 0x12-13 | Today Production | raw | Wh |
| 10-11 | 0x14-17 | Total Production | raw, uint32 | Wh |
| 12 | 0x18-19 | Temperature | ÷10, signed | °C |
| 13 | 0x1A-1B | Operating Status | - | - |
| 14 | 0x1C-1D | Alarm Code | - | - |
| 15 | 0x1E-1F | Alarm Count | - | - |
| 16 | 0x20 | Link Status | high byte | - |

### DTU Registers (FC 0x03)

| Address | Field |
|---------|-------|
| 0x2000-0x2005 | DTU Serial Number (6 regs) |
| 0x2503 | RS485 Function Mode (0=Export Mgmt, 1=Modbus) |

### Status Registers (FC 0x01/0x02 for read, FC 0x05 for write)

| Address | Field | Values |
|---------|-------|--------|
| 0xC000 | ON/OFF All | 0=OFF, 1=ON |
| 0xC001 | Power Limit All | 2-100% (HM), 10-100% (MI) |
| 0xC006 + port×6 | ON/OFF per port | 0=OFF, 1=ON |
| 0xC007 + port×6 | Power Limit per port | 2-100% |

## License

MIT

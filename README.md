# SunSpec Proxy for Hoymiles DTU-Pro

An ESPHome component that bridges Hoymiles microinverters to Victron energy systems via SunSpec over Modbus TCP.

## Overview

```
┌──────────────┐                    ┌─────────────┐
│  Hoymiles    │◄──── 2.4GHz RF ───►│  DTU-Pro    │
│  µ-inverters │                    │ Modbus TCP  │
└──────────────┘                    └──────┬──────┘
                                           │ WiFi / LAN
                                    ┌──────▼──────┐
                                    │  ESP32-S3   │
                                    │  SunSpec    │
                                    │  Proxy      │
                                    │             │
                              LAN ──┤ SunSpec TCP ├── LAN
                                    │ Port 502    │
                                    └──────┬──────┘
                                           │
                              ┌────────────┼────────────┐
                              ▼                         ▼
                       ┌─────────────┐           ┌─────────────┐
                       │  Victron GX │           │    Home      │
                       │ dbus-fronius│           │  Assistant   │
                       └─────────────┘           └─────────────┘
```

The ESP32 connects to the DTU-Pro via Modbus TCP to read per-MPPT-channel data, then:
- Serves aggregated SunSpec registers to Victron on port 502
- Exposes per-MPPT and per-inverter sensors to Home Assistant via ESPHome API

## Features

- **Modbus TCP client** — polls DTU-Pro over WiFi (no RS-485 wiring needed)
- **Per-MPPT sensors** — DC voltage, current, power for each panel input
- **Per-inverter aggregates** — AC power, voltage, current, energy, temperature
- **SunSpec compliant** — Models 1, 101/103, 120, 123 for Victron compatibility
- **Auto-discovery** — sensors auto-register in Home Assistant
- **Power limit forwarding** — Victron power curtailment passed to inverters (experimental)

## Supported Hardware

- **ESP32-S3** (tested on Waveshare ESP32-S3-Relay-6CH)
- **Hoymiles DTU-Pro** with Modbus TCP enabled
- **Hoymiles inverters**: HMS, HMT, MIT series (1-8 MPPT inputs)

## Quick Start

1. Copy `example-config.yaml` and customise with your DTU IP, inverter serials, etc.
2. Create `secrets.yaml` with WiFi credentials
3. Flash with ESPHome: `esphome run your-config.yaml`
4. Victron GX auto-discovers on port 502
5. Home Assistant discovers via ESPHome integration

## Configuration

See `example-config.yaml` for a complete annotated example.

Key settings:

```yaml
sunspec_proxy:
  dtu_host: "192.168.1.100"    # DTU-Pro IP
  dtu_port: 502                # DTU Modbus TCP port
  tcp_port: 502                # SunSpec server port for Victron

  rtu_sources:
    - inverter_model: "HMS-2000-4T"
      inverter_serial: "1520a025566b"  # Hex from DTU registers
      name: "My Inverter"
      sensors:
        mppt_sensors: true     # Enable per-MPPT channel sensors
```

## DTU Register Map

Data is read from register `0x4000` with a stride of 25 registers per MPPT channel:

| Offset | Field | Scaling |
|--------|-------|---------|
| 0 | Marker (12) | — |
| 1-3 | Inverter SN | 6-byte hex |
| 4 | MPPT channel | 1-based |
| 5 | DC voltage | ÷10 → V |
| 6 | DC current | ÷100 → A |
| 7 | AC voltage | ÷10 → V |
| 8 | Frequency | ÷100 → Hz |
| 9 | Power | ÷10 → W |
| 10 | Today energy | Wh |
| 11-12 | Total energy | hi:lo Wh |
| 13 | Temperature | ÷10 → °C |
| 14 | Status | 3=producing |

## License

MIT

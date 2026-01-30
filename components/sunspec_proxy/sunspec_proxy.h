#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>
#include <cstring>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

namespace esphome {
namespace sunspec_proxy {

// SunSpec register base (0-based Modbus address) - for serving to Victron
static const uint16_t SUNSPEC_BASE = 40000;

// Hoymiles Modbus RTU register map (for polling DTU-Pro)
// IMPORTANT: Hoymiles spec uses BYTE addresses, but Modbus FC 0x03 uses REGISTER addresses
// Conversion: Modbus_Register_Address = Byte_Address (since spec starts at 0x1000 bytes = 0x1000 registers)
// Port stride: 0x28 bytes = 40 decimal = 20 Modbus registers
static const uint16_t HM_DATA_BASE = 0x1000;       // Port 0 data start (Modbus register address)
static const uint16_t HM_PORT_STRIDE = 0x28;       // 40 bytes = 20 registers per port
static const uint16_t HM_STATUS_BASE = 0xC000;     // Status registers (use FC 0x01/0x02, not FC 0x03!)
static const uint16_t HM_DEVICE_SN_BASE = 0x2000;  // Device serial number base (FC 0x03)

// Hoymiles data register offsets (relative to port base)
// These are REGISTER offsets, derived from byte addresses in spec:
// Byte 0x1008 → Register (0x1008-0x1000)/2 = 4, etc.
// 
// Spec's "decimal" column: 1 = ×0.1 (divide by 10), 2 = ×0.01 (divide by 100)
// Layout per port (20 registers):
// Reg 0: [data_type(u8)][serial_byte_0(u8)]
// Reg 1-2: serial bytes 1-4
// Reg 3: [serial_byte_5(u8)][port_number(u8)]
// Reg 4-16: live data (voltage, current, power, energy, temp, status, alarms)
// Reg 17-19: reserved
static const uint16_t HM_DATA_TYPE_SN = 0x00;      // Reg 0: [data_type(u8)][serial_byte_0]
static const uint16_t HM_SN_REG1 = 0x01;           // Reg 1-2: serial bytes 1-4
static const uint16_t HM_SN_PORT = 0x03;           // Reg 3: [serial_byte_5][port_number(u8)]
static const uint16_t HM_PV_VOLTAGE = 0x04;        // PV voltage (×0.1 → V)
static const uint16_t HM_PV_CURRENT = 0x05;        // PV current (×0.01 → A for HM/HMS)
static const uint16_t HM_GRID_VOLTAGE = 0x06;      // Grid voltage (×0.1 → V)
static const uint16_t HM_GRID_FREQ = 0x07;         // Grid frequency (×0.01 → Hz)
static const uint16_t HM_PV_POWER = 0x08;          // PV power (×0.1 → W)
static const uint16_t HM_TODAY_PROD = 0x09;        // Today production (raw Wh, uint16)
static const uint16_t HM_TOTAL_PROD_H = 0x0A;      // Total production high word (uint32 Wh, no scaling)
static const uint16_t HM_TOTAL_PROD_L = 0x0B;      // Total production low word
static const uint16_t HM_TEMPERATURE = 0x0C;       // Temperature (×0.1 → °C, int16)
static const uint16_t HM_OPERATING_STATUS = 0x0D;  // Operating status (uint16)
static const uint16_t HM_ALARM_CODE = 0x0E;        // Alarm code (uint16)
static const uint16_t HM_ALARM_COUNT = 0x0F;       // Alarm count (uint16)
static const uint16_t HM_LINK_STATUS = 0x10;       // Link status in high byte (uint8)
static const uint16_t HM_PORT_REGS = 20;           // Only 20 registers contain actual data
static const uint16_t HM_DTU_SN_BASE = 0x2000;     // DTU serial number base address (3 regs = 6 bytes)
static const uint16_t HM_DTU_SN_REGS = 3;          // Number of registers for DTU serial

// Model sizes (register count)
static const uint16_t MODEL_1_SIZE = 66;    // Common
static const uint16_t MODEL_101_SIZE = 50;  // Single-phase inverter (int)
static const uint16_t MODEL_103_SIZE = 50;  // Three-phase inverter (int)
static const uint16_t MODEL_120_SIZE = 26;  // Nameplate ratings
static const uint16_t MODEL_123_SIZE = 24;  // Immediate controls
static const uint16_t MODEL_END_SIZE = 0;   // End marker

// Offsets within Model 101/103 for key data (relative to model data start)
static const uint16_t INV_A = 0;       // AC Total Current
static const uint16_t INV_AphA = 1;    // Phase A current
static const uint16_t INV_AphB = 2;    // Phase B current
static const uint16_t INV_AphC = 3;    // Phase C current
static const uint16_t INV_A_SF = 4;    // Current scale factor
static const uint16_t INV_PPVphAB = 5; // Phase AB voltage
static const uint16_t INV_PPVphBC = 6; // Phase BC voltage
static const uint16_t INV_PPVphCA = 7; // Phase CA voltage
static const uint16_t INV_PhVphA = 8;  // Phase A voltage
static const uint16_t INV_PhVphB = 9;  // Phase B voltage
static const uint16_t INV_PhVphC = 10; // Phase C voltage
static const uint16_t INV_V_SF = 11;   // Voltage scale factor
static const uint16_t INV_W = 12;      // AC Power
static const uint16_t INV_W_SF = 13;   // Power scale factor
static const uint16_t INV_Hz = 14;     // Frequency
static const uint16_t INV_Hz_SF = 15;  // Frequency scale factor
static const uint16_t INV_VA = 16;     // Apparent power
static const uint16_t INV_VA_SF = 17;  // Apparent power SF
static const uint16_t INV_VAr = 18;    // Reactive power
static const uint16_t INV_VAr_SF = 19; // Reactive power SF
static const uint16_t INV_PF = 20;     // Power factor
static const uint16_t INV_PF_SF = 21;  // Power factor SF
static const uint16_t INV_WH = 22;     // Lifetime energy (acc32, 2 regs)
static const uint16_t INV_WH_SF = 24;  // Energy SF
static const uint16_t INV_DCA = 25;    // DC current
static const uint16_t INV_DCA_SF = 26; // DC current SF
static const uint16_t INV_DCV = 27;    // DC voltage
static const uint16_t INV_DCV_SF = 28; // DC voltage SF
static const uint16_t INV_DCW = 29;    // DC power
static const uint16_t INV_DCW_SF = 30; // DC power SF
static const uint16_t INV_TmpCab = 31; // Cabinet temp
static const uint16_t INV_TmpSnk = 32; // Heatsink temp
static const uint16_t INV_TmpTrns = 33;// Transformer temp
static const uint16_t INV_TmpOt = 34;  // Other temp
static const uint16_t INV_Tmp_SF = 35; // Temperature SF
static const uint16_t INV_St = 36;     // Operating state
static const uint16_t INV_StVnd = 37;  // Vendor state
static const uint16_t INV_Evt1 = 38;   // Event bitfield 1 (32-bit)
static const uint16_t INV_Evt2 = 40;   // Event bitfield 2 (32-bit)
static const uint16_t INV_EvtVnd1 = 42;// Vendor event 1
static const uint16_t INV_EvtVnd2 = 44;// Vendor event 2
static const uint16_t INV_EvtVnd3 = 46;// Vendor event 3
static const uint16_t INV_EvtVnd4 = 48;// Vendor event 4

// Max TCP clients
static const int MAX_TCP_CLIENTS = 4;
// Max RTU sources (physical inverters polled via RS-485)
static const int MAX_RTU_SOURCES = 8;

// An RTU source is a physical Hoymiles inverter connected to a DTU port
// Data is read from the DTU using Hoymiles Modbus registers (0x1000 + port*40)
struct RtuSource {
  uint8_t port_number;       // DTU port number (0, 1, 2... for each inverter)
  uint8_t phases;            // 1 or 3
  uint8_t connected_phase;   // For single-phase: which grid phase (1=L1, 2=L2, 3=L3)
  uint16_t rated_power_w;    // Rated output power in watts
  uint8_t mppt_inputs;       // Number of MPPT inputs (DC strings)
  char name[32];             // Friendly name for logging/sensors
  char model[24];            // Inverter model (e.g., "HMS-2000-4T")
  char serial_number[33];    // Inverter serial (configured or auto-read)
  char serial_from_dtu[33];  // Serial read from DTU (auto-populated)

  // Raw register block from last poll (50 regs for Model 101/103 data area)
  uint16_t raw_regs[MODEL_103_SIZE];
  bool data_valid;
  uint32_t last_poll_ms;
  bool initial_model1_read;  // Have we read Model 1 (serial) yet?

  // Statistics
  uint32_t poll_success_count;
  uint32_t poll_fail_count;
  uint32_t poll_timeout_count;
  uint32_t crc_error_count;

  // Decoded values in real-world units (for sensors)
  float power_w;
  float current_a;
  float voltage_v;
  float frequency_hz;
  float energy_kwh;
  float today_energy_wh;
  float temperature_c;
  float pv_voltage_v;
  float pv_current_a;
  float pv_power_w;
  uint16_t alarm_code;
  uint16_t alarm_count;
  uint8_t link_status;
  uint16_t operating_status;
  bool producing;
};

// The aggregated SunSpec device presented to Victron
struct AggregatedConfig {
  uint8_t unit_id;          // Modbus TCP unit ID (126)
  uint8_t phases;           // 1 or 3 (of the combined output)
  uint16_t rated_power_w;   // Sum of all sources
  uint16_t rated_voltage_v; // Nominal voltage
  float rated_current_a;    // Sum of rated currents
  char manufacturer[32];    // e.g. "Fronius" for best Victron compat
  char model_name[32];      // e.g. "Hoymiles Aggregate"
  char serial_number[32];   // e.g. "HM-BRIDGE-001"
};

class SunSpecProxy : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_tcp_port(uint16_t port) { tcp_port_ = port; }
  void set_poll_interval_ms(uint32_t ms) { poll_interval_ms_ = ms; }
  void set_rtu_timeout_ms(uint32_t ms) { rtu_timeout_ms_ = ms; }
  void set_dtu_address(uint8_t addr) { dtu_address_ = addr; }

  // Aggregated device identity
  void set_unit_id(uint8_t id) { agg_config_.unit_id = id; }
  void set_phases(uint8_t p) { agg_config_.phases = p; }
  void set_rated_voltage(uint16_t v) { agg_config_.rated_voltage_v = v; }
  void set_manufacturer(const std::string &s) { strncpy(agg_config_.manufacturer, s.c_str(), 31); agg_config_.manufacturer[31] = 0; }
  void set_model_name(const std::string &s) { strncpy(agg_config_.model_name, s.c_str(), 31); agg_config_.model_name[31] = 0; }
  void set_serial_number(const std::string &s) { strncpy(agg_config_.serial_number, s.c_str(), 31); agg_config_.serial_number[31] = 0; }

  // Add an RTU source (physical inverter to poll)
  void add_rtu_source(uint8_t rtu_address, uint8_t phases, uint16_t rated_power_w,
                      uint8_t connected_phase, uint8_t mppt_inputs,
                      const std::string &name, const std::string &model,
                      const std::string &serial);

  // --- Sensor setters (per-source, indexed 0..N-1) ---
  void set_source_power_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_power_sensors_[idx] = s; }
  void set_source_voltage_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_voltage_sensors_[idx] = s; }
  void set_source_current_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_current_sensors_[idx] = s; }
  void set_source_energy_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_energy_sensors_[idx] = s; }
  void set_source_today_energy_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_today_energy_sensors_[idx] = s; }
  void set_source_frequency_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_frequency_sensors_[idx] = s; }
  void set_source_temperature_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_temperature_sensors_[idx] = s; }
  void set_source_pv_voltage_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_pv_voltage_sensors_[idx] = s; }
  void set_source_pv_current_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_pv_current_sensors_[idx] = s; }
  void set_source_pv_power_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_pv_power_sensors_[idx] = s; }
  void set_source_alarm_code_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_alarm_code_sensors_[idx] = s; }
  void set_source_alarm_count_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_alarm_count_sensors_[idx] = s; }
  void set_source_link_status_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_link_status_sensors_[idx] = s; }
  void set_source_poll_success_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_poll_ok_sensors_[idx] = s; }
  void set_source_poll_fail_sensor(int idx, sensor::Sensor *s) { if (idx < MAX_RTU_SOURCES) src_poll_fail_sensors_[idx] = s; }
  void set_source_online_sensor(int idx, binary_sensor::BinarySensor *s) { if (idx < MAX_RTU_SOURCES) src_online_sensors_[idx] = s; }
  void set_source_status_sensor(int idx, text_sensor::TextSensor *s) { if (idx < MAX_RTU_SOURCES) src_status_sensors_[idx] = s; }

  // --- Aggregate sensors ---
  void set_agg_power_sensor(sensor::Sensor *s) { agg_power_sensor_ = s; }
  void set_agg_voltage_sensor(sensor::Sensor *s) { agg_voltage_sensor_ = s; }
  void set_agg_current_sensor(sensor::Sensor *s) { agg_current_sensor_ = s; }
  void set_agg_energy_sensor(sensor::Sensor *s) { agg_energy_sensor_ = s; }
  void set_agg_frequency_sensor(sensor::Sensor *s) { agg_frequency_sensor_ = s; }

  // --- Victron/TCP sensors ---
  void set_tcp_clients_sensor(sensor::Sensor *s) { tcp_clients_sensor_ = s; }
  void set_tcp_requests_sensor(sensor::Sensor *s) { tcp_requests_sensor_ = s; }
  void set_tcp_errors_sensor(sensor::Sensor *s) { tcp_errors_sensor_ = s; }
  void set_victron_connected_sensor(binary_sensor::BinarySensor *s) { victron_connected_sensor_ = s; }
  void set_victron_status_sensor(text_sensor::TextSensor *s) { victron_status_sensor_ = s; }
  void set_power_limit_sensor(sensor::Sensor *s) { power_limit_sensor_ = s; }

  // --- DTU diagnostic sensors ---
  void set_dtu_serial_sensor(text_sensor::TextSensor *s) { dtu_serial_sensor_ = s; }
  void set_dtu_poll_ok_sensor(sensor::Sensor *s) { dtu_poll_ok_sensor_ = s; }
  void set_dtu_poll_fail_sensor(sensor::Sensor *s) { dtu_poll_fail_sensor_ = s; }
  void set_dtu_online_sensor(binary_sensor::BinarySensor *s) { dtu_online_sensor_ = s; }

 protected:
  // TCP server
  void setup_tcp_server_();
  void handle_tcp_clients_();
  void process_tcp_request_(int client_fd, uint8_t *buf, int len);
  void send_tcp_response_(int client_fd, uint16_t transaction_id, uint8_t unit_id,
                          uint8_t function_code, const uint8_t *data, uint16_t data_len);
  void send_tcp_error_(int client_fd, uint16_t transaction_id, uint8_t unit_id,
                       uint8_t function_code, uint8_t error_code);

  // RTU master
  void poll_rtu_sources_();
  bool send_rtu_request_(uint8_t address, uint8_t function, uint16_t reg_start, uint16_t reg_count);
  int read_rtu_response_(uint8_t *buf, int max_len);
  uint16_t calc_crc16_(const uint8_t *data, uint16_t len);

  // SunSpec register handling
  bool read_sunspec_registers_(uint16_t start_reg, uint16_t count, uint16_t *out);
  bool write_sunspec_registers_(uint16_t start_reg, uint16_t count, const uint16_t *values);
  void build_static_registers_();
  void aggregate_and_update_registers_();

  // Forward power limit to all RTU sources
  void forward_power_limit_(uint16_t pct_raw, bool enabled);

  // Sensor publishing
  void publish_source_sensors_(int idx);
  void publish_aggregate_sensors_();
  void publish_tcp_sensors_();
  void update_source_status_(int idx);
  uint32_t last_sensor_publish_ms_{0};
  static const uint32_t SENSOR_PUBLISH_INTERVAL_MS = 5000;

  // Config
  uint16_t tcp_port_{502};
  uint32_t poll_interval_ms_{5000};
  uint32_t rtu_timeout_ms_{3000};
  uint8_t dtu_address_{126};  // Modbus address of DTU-Pro

  // Aggregated device config
  AggregatedConfig agg_config_{};

  // RTU sources
  int num_sources_{0};
  RtuSource sources_[MAX_RTU_SOURCES];

  // TCP server state
  int server_fd_{-1};
  int client_fds_[MAX_TCP_CLIENTS];
  uint32_t tcp_request_count_{0};
  uint32_t tcp_error_count_{0};
  uint32_t last_tcp_activity_ms_{0};

  // RTU polling state
  int current_poll_source_{0};
  int current_poll_phase_{0}; // -1=DTU serial read, 0=inverter data
  uint32_t last_poll_time_{0};
  bool rtu_busy_{false};
  uint32_t rtu_request_time_{0};

  // Single register map for the aggregated device
  static const uint16_t OFF_SUNS = 0;
  static const uint16_t OFF_MODEL1 = 2;
  static const uint16_t OFF_INV = 70;
  static const uint16_t OFF_M120 = 122;
  static const uint16_t OFF_M123 = 150;
  static const uint16_t OFF_END = 176;
  static const uint16_t TOTAL_REGS = 178;

  uint16_t register_map_[TOTAL_REGS];

  // Aggregated decoded values (for sensors)
  float agg_power_w_{0};
  float agg_current_a_{0};
  float agg_voltage_v_{0};
  float agg_frequency_hz_{0};
  float agg_energy_kwh_{0};

  // --- Sensor pointers ---
  // Per-source
  sensor::Sensor *src_power_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_voltage_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_current_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_energy_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_today_energy_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_frequency_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_temperature_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_pv_voltage_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_pv_current_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_pv_power_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_alarm_code_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_alarm_count_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_link_status_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_poll_ok_sensors_[MAX_RTU_SOURCES]{};
  sensor::Sensor *src_poll_fail_sensors_[MAX_RTU_SOURCES]{};
  binary_sensor::BinarySensor *src_online_sensors_[MAX_RTU_SOURCES]{};
  text_sensor::TextSensor *src_status_sensors_[MAX_RTU_SOURCES]{};

  // Aggregate
  sensor::Sensor *agg_power_sensor_{nullptr};
  sensor::Sensor *agg_voltage_sensor_{nullptr};
  sensor::Sensor *agg_current_sensor_{nullptr};
  sensor::Sensor *agg_energy_sensor_{nullptr};
  sensor::Sensor *agg_frequency_sensor_{nullptr};

  // TCP / Victron
  sensor::Sensor *tcp_clients_sensor_{nullptr};
  sensor::Sensor *tcp_requests_sensor_{nullptr};
  sensor::Sensor *tcp_errors_sensor_{nullptr};
  binary_sensor::BinarySensor *victron_connected_sensor_{nullptr};
  text_sensor::TextSensor *victron_status_sensor_{nullptr};
  sensor::Sensor *power_limit_sensor_{nullptr};

  // DTU diagnostics
  char dtu_serial_[13]{};                // DTU serial number (hex string)
  bool dtu_serial_read_{false};          // Have we read the DTU serial?
  uint32_t dtu_poll_count_{0};           // Successful DTU polls
  uint32_t dtu_poll_fail_count_{0};      // Failed DTU polls
  uint32_t last_dtu_poll_ok_ms_{0};      // Timestamp of last successful DTU poll
  text_sensor::TextSensor *dtu_serial_sensor_{nullptr};
  sensor::Sensor *dtu_poll_ok_sensor_{nullptr};
  sensor::Sensor *dtu_poll_fail_sensor_{nullptr};
  binary_sensor::BinarySensor *dtu_online_sensor_{nullptr};
};

}  // namespace sunspec_proxy
}  // namespace esphome

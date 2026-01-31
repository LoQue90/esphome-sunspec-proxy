#pragma once

#include "esphome/core/component.h"
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

// Hoymiles DTU-Pro Modbus TCP register map
// NEW LAYOUT: Per-MPPT-channel data at 0x4000, 25 registers per channel
// 8 channels total (2 inverters × 4 MPPT each) = 200 registers
static const uint16_t HM_DATA_BASE = 0x4000;       // MPPT channel data start
static const uint16_t HM_MPPT_STRIDE = 25;         // 25 registers per MPPT channel
static const uint16_t HM_MAX_CHANNELS = 8;         // Max MPPT channels to read
static const uint16_t HM_TOTAL_REGS = 200;         // Total registers to read (8 × 25)

// Per-MPPT register offsets (relative to channel base at 0x4000 + channel×25)
// Verified register layout from live DTU-Pro testing:
// Each MPPT channel uses 25 registers, indexed [0..24]:
static const uint16_t HM_MARKER = 0;               // [0] = 12 (marker/type)
static const uint16_t HM_INV_SN_1 = 1;             // [1:3] = Inverter SN (3 regs, 6 bytes hex)
static const uint16_t HM_INV_SN_2 = 2;
static const uint16_t HM_INV_SN_3 = 3;
static const uint16_t HM_MPPT_NUM = 4;             // [4] = MPPT channel number (1-based)
static const uint16_t HM_DC_VOLTAGE = 5;           // [5] = DC voltage × 10 (÷10 for volts)
static const uint16_t HM_DC_CURRENT = 6;           // [6] = DC current × 10 (÷100 for amps: raw/100)
static const uint16_t HM_AC_VOLTAGE = 7;           // [7] = AC voltage × 10 (÷10 for volts)
static const uint16_t HM_FREQUENCY = 8;            // [8] = Frequency × 100 (÷100 for Hz)
static const uint16_t HM_POWER = 9;                // [9] = Power × 10 (÷10 for watts)
static const uint16_t HM_TODAY_ENERGY = 10;        // [10] = Energy today (Wh, raw)
static const uint16_t HM_TOTAL_ENERGY_H = 11;      // [11] = Total energy high word
static const uint16_t HM_TOTAL_ENERGY_L = 12;      // [12] = Total energy low word
static const uint16_t HM_TEMPERATURE = 13;         // [13] = Temperature × 10 (÷10 for °C)
static const uint16_t HM_STATUS = 14;              // [14] = Status (3 = producing)
// [15:24] = Reserved/unknown

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
// Max RTU sources (physical inverters)
static const int MAX_RTU_SOURCES = 8;
// Max MPPT channels per inverter
static const int MAX_MPPT_PER_INVERTER = 8;

// Per-MPPT channel data (from DTU-Pro register 0x4000 + channel×25)
struct MpptData {
  uint8_t mppt_num;         // MPPT number (1-based) from register
  bool data_valid;          // Is this data populated?
  
  // Decoded values from registers (real-world units)
  float dc_voltage_v;       // [5] ÷ 10
  float dc_current_a;       // [6] ÷ 100
  float ac_voltage_v;       // [7] ÷ 10
  float frequency_hz;       // [8] ÷ 100
  float power_w;            // [9] ÷ 10
  float today_energy_wh;    // [10] raw
  float total_energy_kwh;   // [11:12] as uint32 Wh ÷ 1000
  float temperature_c;      // [13] ÷ 10
  uint16_t status;          // [14] (3 = producing)
};

// An RTU source is a physical Hoymiles inverter
// Data is read from the DTU-Pro via Modbus TCP at 0x4000 (per-MPPT layout)
struct RtuSource {
  uint8_t port_number;       // Legacy field (not used in TCP mode)
  uint8_t phases;            // 1 or 3
  uint8_t connected_phase;   // For single-phase: which grid phase (1=L1, 2=L2, 3=L3)
  uint16_t rated_power_w;    // Rated output power in watts
  uint8_t mppt_inputs;       // Number of MPPT inputs (DC strings)
  char name[32];             // Friendly name for logging/sensors
  char model[24];            // Inverter model (e.g., "HMS-2000-4T")
  char serial_number[33];    // Inverter serial (hex string, e.g., "1520a025566b")
  
  // Per-MPPT data (populated by matching SN from register 0x4000+ data)
  MpptData mppt[MAX_MPPT_PER_INVERTER];
  uint8_t mppt_count;        // How many MPPT channels are populated
  
  bool data_valid;
  uint32_t last_poll_ms;

  // Statistics
  uint32_t poll_success_count;
  uint32_t poll_fail_count;

  // Aggregated values for this inverter (sum/avg of all MPPTs)
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

class SunSpecProxy : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_dtu_host(const std::string &host) { dtu_host_ = host; }
  void set_dtu_port(uint16_t port) { dtu_port_ = port; }
  void set_dtu_address(uint8_t addr) { dtu_address_ = addr; }
  void set_tcp_port(uint16_t port) { tcp_port_ = port; }
  void set_poll_interval_ms(uint32_t ms) { poll_interval_ms_ = ms; }
  void set_tcp_timeout_ms(uint32_t ms) { tcp_timeout_ms_ = ms; }

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

  // --- Per-MPPT sensor setters ---
  void set_mppt_dc_voltage_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_dc_voltage_sensors_[inv_idx][mppt_idx] = s; 
  }
  void set_mppt_dc_current_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_dc_current_sensors_[inv_idx][mppt_idx] = s; 
  }
  void set_mppt_dc_power_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_dc_power_sensors_[inv_idx][mppt_idx] = s; 
  }
  void set_mppt_ac_voltage_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_ac_voltage_sensors_[inv_idx][mppt_idx] = s; 
  }
  void set_mppt_frequency_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_frequency_sensors_[inv_idx][mppt_idx] = s; 
  }
  void set_mppt_power_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_power_sensors_[inv_idx][mppt_idx] = s; 
  }
  void set_mppt_today_energy_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_today_energy_sensors_[inv_idx][mppt_idx] = s; 
  }
  void set_mppt_total_energy_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_total_energy_sensors_[inv_idx][mppt_idx] = s; 
  }
  void set_mppt_temperature_sensor(int inv_idx, int mppt_idx, sensor::Sensor *s) { 
    if (inv_idx < MAX_RTU_SOURCES && mppt_idx < MAX_MPPT_PER_INVERTER) 
      mppt_temperature_sensors_[inv_idx][mppt_idx] = s; 
  }

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
  // TCP server (for Victron)
  void setup_tcp_server_();
  void handle_tcp_clients_();
  void process_tcp_request_(int client_fd, uint8_t *buf, int len);
  void send_tcp_response_(int client_fd, uint16_t transaction_id, uint8_t unit_id,
                          uint8_t function_code, const uint8_t *data, uint16_t data_len);
  void send_tcp_error_(int client_fd, uint16_t transaction_id, uint8_t unit_id,
                       uint8_t function_code, uint8_t error_code);

  // Modbus TCP client (to DTU-Pro)
  void setup_dtu_client_();
  void poll_dtu_data_();
  bool connect_to_dtu_();
  void close_dtu_connection_();
  bool send_modbus_tcp_request_(uint8_t function, uint16_t reg_start, uint16_t reg_count);
  int read_modbus_tcp_response_(uint8_t *buf, int max_len);

  // SunSpec register handling
  bool read_sunspec_registers_(uint16_t start_reg, uint16_t count, uint16_t *out);
  bool write_sunspec_registers_(uint16_t start_reg, uint16_t count, const uint16_t *values);
  void build_static_registers_();
  void aggregate_and_update_registers_();

  // Forward power limit to all RTU sources
  void forward_power_limit_(uint16_t pct_raw, bool enabled);

  // Data parsing and mapping
  void parse_dtu_registers_(const uint16_t *regs, int reg_count);
  void map_mppt_to_inverters_();
  void aggregate_inverter_data_(int inv_idx);
  
  // Sensor publishing
  void publish_source_sensors_(int idx);
  void publish_mppt_sensors_(int inv_idx, int mppt_idx);
  void publish_aggregate_sensors_();
  void publish_tcp_sensors_();
  void update_source_status_(int idx);
  uint32_t last_sensor_publish_ms_{0};
  static const uint32_t SENSOR_PUBLISH_INTERVAL_MS = 5000;

  // Config
  std::string dtu_host_;       // DTU-Pro IP/hostname
  uint16_t dtu_port_{502};     // DTU-Pro Modbus TCP port
  uint8_t dtu_address_{101};   // DTU Modbus unit ID
  uint16_t tcp_port_{502};     // SunSpec server port (for Victron)
  uint32_t poll_interval_ms_{5000};
  uint32_t tcp_timeout_ms_{3000};

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

  // DTU client state
  int dtu_fd_{-1};             // DTU connection socket
  uint32_t last_poll_time_{0};
  uint32_t last_dtu_connect_attempt_{0};
  uint16_t modbus_transaction_id_{1};
  bool dtu_connected_{false};
  
  // Raw register buffer from DTU (200 registers = 8 channels × 25)
  uint16_t dtu_regs_[HM_TOTAL_REGS];
  bool dtu_data_valid_{false};

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
  uint32_t dtu_poll_count_{0};           // Successful DTU polls
  uint32_t dtu_poll_fail_count_{0};      // Failed DTU polls
  uint32_t last_dtu_poll_ok_ms_{0};      // Timestamp of last successful DTU poll
  text_sensor::TextSensor *dtu_serial_sensor_{nullptr};
  sensor::Sensor *dtu_poll_ok_sensor_{nullptr};
  sensor::Sensor *dtu_poll_fail_sensor_{nullptr};
  binary_sensor::BinarySensor *dtu_online_sensor_{nullptr};

  // Per-MPPT sensor arrays [inverter_idx][mppt_idx]
  sensor::Sensor *mppt_dc_voltage_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
  sensor::Sensor *mppt_dc_current_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
  sensor::Sensor *mppt_dc_power_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
  sensor::Sensor *mppt_ac_voltage_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
  sensor::Sensor *mppt_frequency_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
  sensor::Sensor *mppt_power_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
  sensor::Sensor *mppt_today_energy_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
  sensor::Sensor *mppt_total_energy_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
  sensor::Sensor *mppt_temperature_sensors_[MAX_RTU_SOURCES][MAX_MPPT_PER_INVERTER]{};
};

}  // namespace sunspec_proxy
}  // namespace esphome

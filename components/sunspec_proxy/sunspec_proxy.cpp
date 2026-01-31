#include "sunspec_proxy.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cerrno>
#include <fcntl.h>
#include <cmath>

namespace esphome {
namespace sunspec_proxy {

static const char *const TAG = "sunspec_proxy";

// Helper: write string into uint16 register array (SunSpec string encoding: big-endian char pairs)
static void write_string_regs(uint16_t *regs, const char *str, int max_regs) {
  memset(regs, 0, max_regs * 2);
  int len = strlen(str);
  if (len > max_regs * 2) len = max_regs * 2;
  for (int i = 0; i < len; i++) {
    if (i % 2 == 0)
      regs[i / 2] = ((uint16_t)(uint8_t)str[i]) << 8;
    else
      regs[i / 2] |= (uint8_t)str[i];
  }
}

static uint16_t be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static void put_be16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v & 0xFF; }

// Apply SunSpec scale factor to a raw int16 value
static float apply_sf(int16_t raw, int16_t sf) {
  if (raw == (int16_t)0x8000) return NAN; // not implemented
  return (float)raw * powf(10.0f, (float)sf);
}
static float apply_sf_u16(uint16_t raw, int16_t sf) {
  if (raw == 0xFFFF) return NAN; // not implemented
  return (float)raw * powf(10.0f, (float)sf);
}

// SunSpec operating state to human string
static const char *sunspec_state_str(uint16_t st) {
  switch (st) {
    case 1: return "Off";
    case 2: return "Sleeping";
    case 3: return "Starting";
    case 4: return "MPPT";
    case 5: return "Throttled";
    case 6: return "Shutting Down";
    case 7: return "Fault";
    case 8: return "Standby";
    default: return "Unknown";
  }
}

// ============================================================
// Configuration
// ============================================================

void SunSpecProxy::add_rtu_source(uint8_t port_number, uint8_t phases, uint16_t rated_power_w,
                                   uint8_t connected_phase, uint8_t mppt_inputs,
                                   const std::string &name, const std::string &model,
                                   const std::string &serial) {
  if (num_sources_ >= MAX_RTU_SOURCES) return;
  auto &s = sources_[num_sources_];
  memset(&s, 0, sizeof(RtuSource));
  s.port_number = port_number;
  s.phases = phases;
  s.connected_phase = (phases == 1) ? connected_phase : 0; // 0 = all phases (3-phase)
  s.rated_power_w = rated_power_w;
  s.mppt_inputs = mppt_inputs;
  strncpy(s.name, name.c_str(), 31); s.name[31] = 0;
  strncpy(s.model, model.c_str(), 23); s.model[23] = 0;
  strncpy(s.serial_number, serial.c_str(), 32); s.serial_number[32] = 0;
  s.data_valid = false;
  s.mppt_count = 0;
  for (int m = 0; m < MAX_MPPT_PER_INVERTER; m++) {
    s.mppt[m].data_valid = false;
  }
  num_sources_++;
  if (phases == 1) {
    ESP_LOGI(TAG, "Added RTU source #%d: '%s' (%s) port=%d, 1-phase on L%d, %dW, %d MPPT",
             num_sources_ - 1, s.name, s.model, port_number, connected_phase, rated_power_w, mppt_inputs);
  } else {
    ESP_LOGI(TAG, "Added RTU source #%d: '%s' (%s) port=%d, 3-phase, %dW, %d MPPT",
             num_sources_ - 1, s.name, s.model, port_number, rated_power_w, mppt_inputs);
  }
  if (serial.length() > 0) {
    ESP_LOGI(TAG, "  Serial: %s", s.serial_number);
  }
}

// ============================================================
// Setup
// ============================================================

void SunSpecProxy::setup() {
  ESP_LOGI(TAG, "============================================");
  ESP_LOGI(TAG, "  SunSpec Proxy v2.0 — Hoymiles TCP Mode");
  ESP_LOGI(TAG, "  DTU: %s:%d (unit_id=%d)", dtu_host_.c_str(), dtu_port_, dtu_address_);
  ESP_LOGI(TAG, "  Serving as unit_id %d on TCP :%d",
           agg_config_.unit_id, tcp_port_);
  ESP_LOGI(TAG, "  Manufacturer: %s", agg_config_.manufacturer);
  ESP_LOGI(TAG, "  Model: %s", agg_config_.model_name);
  ESP_LOGI(TAG, "  Serial: %s", agg_config_.serial_number);
  ESP_LOGI(TAG, "============================================");

  for (int i = 0; i < MAX_TCP_CLIENTS; i++) client_fds_[i] = -1;

  // Compute aggregated rated power/current from sources
  agg_config_.rated_power_w = 0;
  agg_config_.rated_current_a = 0;
  for (int i = 0; i < num_sources_; i++) {
    agg_config_.rated_power_w += sources_[i].rated_power_w;
    if (agg_config_.rated_voltage_v > 0) {
      agg_config_.rated_current_a += (float)sources_[i].rated_power_w / agg_config_.rated_voltage_v;
    }
    ESP_LOGI(TAG, "  Source #%d: '%s' port=%d, %dW",
             i, sources_[i].name, sources_[i].port_number, sources_[i].rated_power_w);
  }
  ESP_LOGI(TAG, "  Total rated: %dW, %.1fA @ %dV",
           agg_config_.rated_power_w, agg_config_.rated_current_a, agg_config_.rated_voltage_v);

  build_static_registers_();
  setup_tcp_server_();
}

void SunSpecProxy::loop() {
  handle_tcp_clients_();
  poll_dtu_data_();

  // Periodic sensor publishing
  uint32_t now = millis();
  if (now - last_sensor_publish_ms_ >= SENSOR_PUBLISH_INTERVAL_MS) {
    last_sensor_publish_ms_ = now;
    for (int i = 0; i < num_sources_; i++) {
      publish_source_sensors_(i);
      // Publish per-MPPT sensors if they exist
      for (int m = 0; m < sources_[i].mppt_count; m++) {
        publish_mppt_sensors_(i, m);
      }
    }
    publish_aggregate_sensors_();
    publish_tcp_sensors_();
  }
}

// ============================================================
// Static Register Map Construction
// ============================================================

void SunSpecProxy::build_static_registers_() {
  for (int i = 0; i < TOTAL_REGS; i++) register_map_[i] = 0xFFFF;

  // --- SunS header ---
  register_map_[OFF_SUNS] = 0x5375;
  register_map_[OFF_SUNS + 1] = 0x6e53;

  // --- Model 1: Common Block ---
  register_map_[OFF_MODEL1] = 1;
  register_map_[OFF_MODEL1 + 1] = MODEL_1_SIZE;

  uint16_t *m1 = &register_map_[OFF_MODEL1 + 2];
  for (int i = 0; i < MODEL_1_SIZE; i++) m1[i] = 0x0000;
  write_string_regs(&m1[0], agg_config_.manufacturer, 16);
  write_string_regs(&m1[16], agg_config_.model_name, 16);
  write_string_regs(&m1[40], "1.1.0", 8);
  write_string_regs(&m1[48], agg_config_.serial_number, 16);
  m1[64] = agg_config_.unit_id;
  m1[65] = 0x8000;

  // --- Model 101/103: Inverter ---
  uint16_t model_id = (agg_config_.phases == 3) ? 103 : 101;
  register_map_[OFF_INV] = model_id;
  register_map_[OFF_INV + 1] = MODEL_103_SIZE;

  uint16_t *inv = &register_map_[OFF_INV + 2];
  for (int i = 0; i < MODEL_103_SIZE; i++) inv[i] = 0xFFFF;

  inv[INV_A_SF]   = (uint16_t)(int16_t)-2;
  inv[INV_V_SF]   = (uint16_t)(int16_t)-1;
  inv[INV_W_SF]   = (uint16_t)(int16_t) 0;
  inv[INV_Hz_SF]  = (uint16_t)(int16_t)-2;
  inv[INV_VA_SF]  = (uint16_t)(int16_t) 0;
  inv[INV_VAr_SF] = (uint16_t)(int16_t) 0;
  inv[INV_PF_SF]  = (uint16_t)(int16_t)-2;
  inv[INV_WH_SF]  = (uint16_t)(int16_t) 0;
  inv[INV_DCA_SF] = (uint16_t)(int16_t)-2;
  inv[INV_DCV_SF] = (uint16_t)(int16_t)-1;
  inv[INV_DCW_SF] = (uint16_t)(int16_t) 0;
  inv[INV_Tmp_SF] = (uint16_t)(int16_t)-1;

  inv[INV_St] = 2; // OFF
  inv[INV_Evt1] = 0; inv[INV_Evt1 + 1] = 0;
  inv[INV_Evt2] = 0; inv[INV_Evt2 + 1] = 0;
  inv[INV_EvtVnd1] = 0; inv[INV_EvtVnd1 + 1] = 0;
  inv[INV_EvtVnd2] = 0; inv[INV_EvtVnd2 + 1] = 0;
  inv[INV_EvtVnd3] = 0; inv[INV_EvtVnd3 + 1] = 0;
  inv[INV_EvtVnd4] = 0; inv[INV_EvtVnd4 + 1] = 0;

  // --- Model 120: Nameplate Ratings ---
  register_map_[OFF_M120] = 120;
  register_map_[OFF_M120 + 1] = MODEL_120_SIZE;
  uint16_t *m120 = &register_map_[OFF_M120 + 2];
  for (int i = 0; i < MODEL_120_SIZE; i++) m120[i] = 0xFFFF;
  m120[0] = 4;
  m120[1] = agg_config_.rated_power_w;
  m120[2] = 0;
  m120[3] = agg_config_.rated_power_w;
  m120[4] = 0;
  m120[10] = (uint16_t)(agg_config_.rated_current_a * 10);
  m120[11] = (uint16_t)(int16_t)-1;

  // --- Model 123: Immediate Controls ---
  register_map_[OFF_M123] = 123;
  register_map_[OFF_M123 + 1] = MODEL_123_SIZE;
  uint16_t *m123 = &register_map_[OFF_M123 + 2];
  for (int i = 0; i < MODEL_123_SIZE; i++) m123[i] = 0xFFFF;
  m123[2] = 1;                          // Conn = connected
  m123[3] = (uint16_t)(int16_t)-1;      // WMaxLimPct_SF
  m123[5] = 1000;                       // WMaxLimPct = 100.0%
  m123[8] = 0;                          // WMaxLim_Ena = disabled

  // --- End marker ---
  register_map_[OFF_END] = 0xFFFF;
  register_map_[OFF_END + 1] = 0;

  ESP_LOGI(TAG, "Register map built: %d registers, Model %d", TOTAL_REGS, model_id);
}

// ============================================================
// Aggregation
// ============================================================

void SunSpecProxy::aggregate_and_update_registers_() {
  uint16_t *inv = &register_map_[OFF_INV + 2];

  // Per-phase accumulators (real-world units)
  float phase_power[3] = {0, 0, 0};       // W per phase
  float phase_current[3] = {0, 0, 0};     // A per phase
  float phase_voltage_sum[3] = {0, 0, 0}; // V sum for averaging
  int phase_voltage_count[3] = {0, 0, 0};

  float total_power = 0, total_current = 0;
  float sum_freq = 0, total_va = 0, total_var = 0;
  uint32_t total_energy_wh = 0;
  float max_temp = NAN;
  float total_dc_power = 0;
  int valid_count = 0;
  bool any_producing = false;

  // FIXED: Use decoded float values from sources instead of reading raw_regs as SunSpec
  for (int i = 0; i < num_sources_; i++) {
    auto &s = sources_[i];
    if (!s.data_valid) continue;
    valid_count++;

    // Use the already-decoded values from the Hoymiles data
    total_power += s.power_w;
    total_current += s.current_a;
    if (s.power_w > 0) any_producing = true;
    sum_freq += s.frequency_hz;

    // Total energy in Wh from kWh
    total_energy_wh += (uint32_t)(s.energy_kwh * 1000.0f);

    // Temperature
    if (std::isnan(max_temp) || s.temperature_c > max_temp)
      max_temp = s.temperature_c;

    // Phase distribution
    if (s.phases == 3) {
      // 3-phase: distribute evenly (we only have total values from Hoymiles)
      for (int p = 0; p < 3; p++) {
        phase_current[p] += s.current_a / 3.0f;
        phase_voltage_sum[p] += s.voltage_v;
        phase_voltage_count[p]++;
      }
    } else {
      // Single-phase: add to connected_phase
      int ph = s.connected_phase - 1;  // 0-indexed (0=L1, 1=L2, 2=L3)
      if (ph < 0 || ph > 2) ph = 0;
      phase_current[ph] += s.current_a;
      phase_voltage_sum[ph] += s.voltage_v;
      phase_voltage_count[ph]++;
    }

    // DC power (approximate from PV power)
    total_dc_power += s.pv_power_w;

    s.producing = (s.power_w > 0);
    update_source_status_(i);
  }

  if (valid_count == 0) {
    inv[INV_St] = 2;
    agg_power_w_ = 0; agg_current_a_ = 0; agg_voltage_v_ = 0; agg_frequency_hz_ = 0;
    ESP_LOGW(TAG, "Aggregation: no valid sources");
    return;
  }

  // Compute averaged voltages per phase
  float avg_v[3];
  for (int p = 0; p < 3; p++) {
    avg_v[p] = phase_voltage_count[p] > 0 ? phase_voltage_sum[p] / phase_voltage_count[p] : 0;
  }

  // Store aggregate decoded values
  agg_power_w_ = total_power;
  agg_current_a_ = total_current;
  agg_voltage_v_ = avg_v[0]; // report L1 as primary
  agg_frequency_hz_ = sum_freq / valid_count;
  agg_energy_kwh_ = (float)total_energy_wh / 1000.0f;

  // Write to register map
  // Our SFs: A=-2, V=-1, W=0, Hz=-2, VA=0, VAr=0, PF=-2, WH=0, Tmp=-1

  // Total AC power
  inv[INV_W] = (uint16_t)(int16_t)(int)total_power;

  // Total and per-phase current (SF=-2 → register = A * 100)
  inv[INV_A]    = (uint16_t)(total_current * 100.0f);
  inv[INV_AphA] = (uint16_t)(phase_current[0] * 100.0f);
  inv[INV_AphB] = (uint16_t)(phase_current[1] * 100.0f);
  inv[INV_AphC] = (uint16_t)(phase_current[2] * 100.0f);

  // Per-phase voltage (SF=-1 → register = V * 10)
  inv[INV_PhVphA] = (uint16_t)(avg_v[0] * 10.0f);
  inv[INV_PhVphB] = (uint16_t)(avg_v[1] * 10.0f);
  inv[INV_PhVphC] = (uint16_t)(avg_v[2] * 10.0f);

  // Line-to-line voltages (SF=-1)
  if (agg_config_.phases == 3) {
    // Proper L-L from L-N: Vab = sqrt(Va² + Vb² - 2*Va*Vb*cos(120°))
    // For balanced system: Vll ≈ Vln * sqrt(3)
    // Use actual phase voltages for better accuracy
    float vab = sqrtf(avg_v[0]*avg_v[0] + avg_v[1]*avg_v[1] + avg_v[0]*avg_v[1]); // cos(120°) = -0.5
    float vbc = sqrtf(avg_v[1]*avg_v[1] + avg_v[2]*avg_v[2] + avg_v[1]*avg_v[2]);
    float vca = sqrtf(avg_v[2]*avg_v[2] + avg_v[0]*avg_v[0] + avg_v[2]*avg_v[0]);
    inv[INV_PPVphAB] = (uint16_t)(vab * 10.0f);
    inv[INV_PPVphBC] = (uint16_t)(vbc * 10.0f);
    inv[INV_PPVphCA] = (uint16_t)(vca * 10.0f);
  }

  // Frequency (SF=-2 → Hz * 100)
  inv[INV_Hz] = (uint16_t)((sum_freq / valid_count) * 100.0f);

  // VA / VAr (SF=0)
  inv[INV_VA] = (uint16_t)(int16_t)(int)total_va;
  inv[INV_VAr] = (uint16_t)(int16_t)(int)total_var;

  // Power factor (SF=-2 → PF * 100)
  if (total_va > 0) {
    float pf = total_power / total_va;
    if (pf > 1.0f) pf = 1.0f;
    inv[INV_PF] = (uint16_t)(int16_t)(int)(pf * 100.0f);
  }

  // Energy (SF=0, acc32 Wh)
  inv[INV_WH]     = (uint16_t)(total_energy_wh >> 16);
  inv[INV_WH + 1] = (uint16_t)(total_energy_wh & 0xFFFF);

  // Temperature (SF=-1 → °C * 10)
  if (!std::isnan(max_temp)) {
    inv[INV_TmpCab] = (uint16_t)(int16_t)(int)(max_temp * 10.0f);
  }

  // DC power (SF=0)
  if (total_dc_power > 0) {
    inv[INV_DCW] = (uint16_t)(int16_t)(int)total_dc_power;
  }

  // Operating state
  inv[INV_St] = any_producing ? 4 : 2;

  ESP_LOGI(TAG, "AGG: P=%.0fW (L1:%.0f L2:%.0f L3:%.0f) I=%.2fA V=%.1f/%.1f/%.1fV f=%.2fHz E=%.1fkWh [%d/%d, %s]",
           total_power, phase_power[0], phase_power[1], phase_power[2],
           total_current, avg_v[0], avg_v[1], avg_v[2],
           sum_freq / valid_count,
           (float)total_energy_wh / 1000.0f,
           valid_count, num_sources_,
           any_producing ? "MPPT" : "Sleep");
}

// ============================================================
// Sensor Publishing
// ============================================================

void SunSpecProxy::update_source_status_(int idx) {
  auto &s = sources_[idx];
  if (!s.data_valid) return;

  uint32_t age_s = (millis() - s.last_poll_ms) / 1000;
  bool stale = age_s > (poll_interval_ms_ / 1000) * 3;

  if (src_status_sensors_[idx]) {
    char buf[64];
    if (stale) {
      snprintf(buf, sizeof(buf), "Stale (%lus)", age_s);
    } else if (s.producing) {
      snprintf(buf, sizeof(buf), "Producing %.0fW", s.power_w);
    } else {
      snprintf(buf, sizeof(buf), "Idle");
    }
    src_status_sensors_[idx]->publish_state(buf);
  }
}

void SunSpecProxy::publish_source_sensors_(int idx) {
  auto &s = sources_[idx];

  uint32_t age_s = s.data_valid ? (millis() - s.last_poll_ms) / 1000 : 999;
  bool online = s.data_valid && age_s < (poll_interval_ms_ / 1000) * 3;

  // Core electrical sensors
  if (src_power_sensors_[idx]) src_power_sensors_[idx]->publish_state(s.data_valid ? s.power_w : NAN);
  if (src_voltage_sensors_[idx]) src_voltage_sensors_[idx]->publish_state(s.data_valid ? s.voltage_v : NAN);
  if (src_current_sensors_[idx]) src_current_sensors_[idx]->publish_state(s.data_valid ? s.current_a : NAN);
  if (src_energy_sensors_[idx]) src_energy_sensors_[idx]->publish_state(s.data_valid ? s.energy_kwh : NAN);
  if (src_today_energy_sensors_[idx]) src_today_energy_sensors_[idx]->publish_state(s.data_valid ? s.today_energy_wh : NAN);
  if (src_frequency_sensors_[idx]) src_frequency_sensors_[idx]->publish_state(s.data_valid ? s.frequency_hz : NAN);
  if (src_temperature_sensors_[idx]) src_temperature_sensors_[idx]->publish_state(s.data_valid ? s.temperature_c : NAN);

  // DC (PV) side sensors
  if (src_pv_voltage_sensors_[idx]) src_pv_voltage_sensors_[idx]->publish_state(s.data_valid ? s.pv_voltage_v : NAN);
  if (src_pv_current_sensors_[idx]) src_pv_current_sensors_[idx]->publish_state(s.data_valid ? s.pv_current_a : NAN);
  if (src_pv_power_sensors_[idx]) src_pv_power_sensors_[idx]->publish_state(s.data_valid ? s.pv_power_w : NAN);

  // Status and diagnostics
  if (src_alarm_code_sensors_[idx]) src_alarm_code_sensors_[idx]->publish_state(s.data_valid ? s.alarm_code : 0);
  if (src_alarm_count_sensors_[idx]) src_alarm_count_sensors_[idx]->publish_state(s.data_valid ? s.alarm_count : 0);
  if (src_link_status_sensors_[idx]) src_link_status_sensors_[idx]->publish_state(s.data_valid ? s.link_status : 0);

  // Statistics
  if (src_poll_ok_sensors_[idx]) src_poll_ok_sensors_[idx]->publish_state(s.poll_success_count);
  if (src_poll_fail_sensors_[idx]) src_poll_fail_sensors_[idx]->publish_state(s.poll_fail_count);
  if (src_online_sensors_[idx]) src_online_sensors_[idx]->publish_state(online);

  update_source_status_(idx);
}

void SunSpecProxy::publish_aggregate_sensors_() {
  if (agg_power_sensor_) agg_power_sensor_->publish_state(agg_power_w_);
  if (agg_voltage_sensor_) agg_voltage_sensor_->publish_state(agg_voltage_v_);
  if (agg_current_sensor_) agg_current_sensor_->publish_state(agg_current_a_);
  if (agg_energy_sensor_) agg_energy_sensor_->publish_state(agg_energy_kwh_);
  if (agg_frequency_sensor_) agg_frequency_sensor_->publish_state(agg_frequency_hz_);
}

void SunSpecProxy::publish_tcp_sensors_() {
  // Count active TCP clients
  int active = 0;
  for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
    if (client_fds_[i] >= 0) active++;
  }

  bool victron_active = active > 0 && (millis() - last_tcp_activity_ms_ < 30000);

  if (tcp_clients_sensor_) tcp_clients_sensor_->publish_state(active);
  if (tcp_requests_sensor_) tcp_requests_sensor_->publish_state(tcp_request_count_);
  if (tcp_errors_sensor_) tcp_errors_sensor_->publish_state(tcp_error_count_);
  if (victron_connected_sensor_) victron_connected_sensor_->publish_state(victron_active);

  if (victron_status_sensor_) {
    if (!victron_active && active == 0) {
      victron_status_sensor_->publish_state("No connection");
    } else if (!victron_active) {
      victron_status_sensor_->publish_state("Connected, idle");
    } else {
      char buf[64];
      snprintf(buf, sizeof(buf), "Active (%lu reqs)", tcp_request_count_);
      victron_status_sensor_->publish_state(buf);
    }
  }

  // Power limit
  if (power_limit_sensor_) {
    uint16_t pct = register_map_[OFF_M123 + 2 + 5]; // WMaxLimPct
    uint16_t ena = register_map_[OFF_M123 + 2 + 8]; // WMaxLim_Ena
    power_limit_sensor_->publish_state(ena == 1 ? pct / 10.0f : 100.0f);
  }

  // DTU diagnostics
  if (dtu_poll_ok_sensor_) {
    dtu_poll_ok_sensor_->publish_state(dtu_poll_count_);
  }
  if (dtu_poll_fail_sensor_) {
    dtu_poll_fail_sensor_->publish_state(dtu_poll_fail_count_);
  }
  if (dtu_online_sensor_) {
    bool dtu_online = dtu_poll_count_ > 0 && (millis() - last_dtu_poll_ok_ms_) < 30000;
    dtu_online_sensor_->publish_state(dtu_online);
  }
}

// ============================================================
// TCP Server
// ============================================================

void SunSpecProxy::setup_tcp_server_() {
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    ESP_LOGE(TAG, "TCP socket create failed: errno=%d", errno);
    return;
  }

  int opt = 1;
  setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  fcntl(server_fd_, F_SETFL, fcntl(server_fd_, F_GETFL, 0) | O_NONBLOCK);

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(tcp_port_);

  if (bind(server_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "TCP bind port %d failed: errno=%d", tcp_port_, errno);
    close(server_fd_); server_fd_ = -1; return;
  }
  if (listen(server_fd_, 4) < 0) {
    ESP_LOGE(TAG, "TCP listen failed: errno=%d", errno);
    close(server_fd_); server_fd_ = -1; return;
  }

  ESP_LOGI(TAG, "Modbus TCP listening on port %d (unit_id=%d)", tcp_port_, agg_config_.unit_id);
}

void SunSpecProxy::handle_tcp_clients_() {
  if (server_fd_ < 0) return;

  struct sockaddr_in ca;
  socklen_t al = sizeof(ca);
  int nfd = accept(server_fd_, (struct sockaddr *)&ca, &al);
  if (nfd >= 0) {
    fcntl(nfd, F_SETFL, fcntl(nfd, F_GETFL, 0) | O_NONBLOCK);
    bool placed = false;
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
      if (client_fds_[i] < 0) {
        client_fds_[i] = nfd;
        placed = true;
        char ip[16];
        inet_ntoa_r(ca.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "TCP: Client connected from %s (slot %d)", ip, i);
        break;
      }
    }
    if (!placed) {
      ESP_LOGW(TAG, "TCP: No slot available, rejecting connection");
      close(nfd);
    }
  }

  uint8_t buf[260];
  for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
    if (client_fds_[i] < 0) continue;
    int n = recv(client_fds_[i], buf, sizeof(buf), 0);
    if (n > 0) {
      process_tcp_request_(client_fds_[i], buf, n);
    } else if (n == 0) {
      ESP_LOGI(TAG, "TCP: Client slot %d disconnected", i);
      close(client_fds_[i]); client_fds_[i] = -1;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
      ESP_LOGW(TAG, "TCP: Client slot %d error: errno=%d", i, errno);
      close(client_fds_[i]); client_fds_[i] = -1;
    }
  }
}

void SunSpecProxy::process_tcp_request_(int client_fd, uint8_t *buf, int len) {
  if (len < 8) return;

  uint16_t txn_id = be16(&buf[0]);
  uint16_t proto = be16(&buf[2]);
  uint8_t unit_id = buf[6];
  uint8_t fc = buf[7];

  if (proto != 0) return;

  last_tcp_activity_ms_ = millis();
  tcp_request_count_++;

  if (unit_id != agg_config_.unit_id) {
    ESP_LOGD(TAG, "TCP: Request for unit_id %d (we are %d), ignoring", unit_id, agg_config_.unit_id);
    return;
  }

  switch (fc) {
    case 0x03: { // Read Holding Registers
      if (len < 12) return;
      uint16_t start = be16(&buf[8]);
      uint16_t count = be16(&buf[10]);

      ESP_LOGD(TAG, "TCP RX: ReadHolding txn=%d unit=%d reg=%d count=%d",
               txn_id, unit_id, start, count);

      if (count > 125) {
        ESP_LOGW(TAG, "TCP: Read count %d exceeds max 125", count);
        send_tcp_error_(client_fd, txn_id, unit_id, fc, 0x03);
        tcp_error_count_++;
        return;
      }

      uint16_t values[125];
      if (!read_sunspec_registers_(start, count, values)) {
        ESP_LOGW(TAG, "TCP: Read failed for reg %d count %d (out of range)", start, count);
        send_tcp_error_(client_fd, txn_id, unit_id, fc, 0x02);
        tcp_error_count_++;
        return;
      }

      uint8_t resp[1 + 250];
      resp[0] = count * 2;
      for (int i = 0; i < count; i++) put_be16(&resp[1 + i * 2], values[i]);
      send_tcp_response_(client_fd, txn_id, unit_id, fc, resp, 1 + count * 2);

      ESP_LOGV(TAG, "TCP TX: ReadHolding response %d regs", count);
      break;
    }
    case 0x06: { // Write Single Register
      if (len < 12) return;
      uint16_t reg = be16(&buf[8]);
      uint16_t val = be16(&buf[10]);

      ESP_LOGI(TAG, "TCP RX: WriteSingle txn=%d reg=%d val=%d", txn_id, reg, val);

      if (!write_sunspec_registers_(reg, 1, &val)) {
        send_tcp_error_(client_fd, txn_id, unit_id, fc, 0x02);
        tcp_error_count_++;
        return;
      }
      uint8_t resp[4]; put_be16(&resp[0], reg); put_be16(&resp[2], val);
      send_tcp_response_(client_fd, txn_id, unit_id, fc, resp, 4);
      break;
    }
    case 0x10: { // Write Multiple Registers
      if (len < 13) return;
      uint16_t reg = be16(&buf[8]);
      uint16_t cnt = be16(&buf[10]);

      ESP_LOGI(TAG, "TCP RX: WriteMultiple txn=%d reg=%d count=%d", txn_id, reg, cnt);

      if (len < 13 + cnt * 2 || cnt > 100) {
        send_tcp_error_(client_fd, txn_id, unit_id, fc, 0x03);
        tcp_error_count_++;
        return;
      }
      uint16_t vals[100];
      for (int i = 0; i < cnt; i++) vals[i] = be16(&buf[13 + i * 2]);
      if (!write_sunspec_registers_(reg, cnt, vals)) {
        send_tcp_error_(client_fd, txn_id, unit_id, fc, 0x02);
        tcp_error_count_++;
        return;
      }
      uint8_t resp[4]; put_be16(&resp[0], reg); put_be16(&resp[2], cnt);
      send_tcp_response_(client_fd, txn_id, unit_id, fc, resp, 4);
      break;
    }
    default:
      ESP_LOGW(TAG, "TCP: Unsupported function code 0x%02X", fc);
      send_tcp_error_(client_fd, txn_id, unit_id, fc, 0x01);
      tcp_error_count_++;
  }
}

void SunSpecProxy::send_tcp_response_(int client_fd, uint16_t txn_id, uint8_t unit_id,
                                       uint8_t fc, const uint8_t *data, uint16_t data_len) {
  uint8_t frame[260];
  put_be16(&frame[0], txn_id);
  put_be16(&frame[2], 0);
  put_be16(&frame[4], 1 + 1 + data_len);
  frame[6] = unit_id;
  frame[7] = fc;
  memcpy(&frame[8], data, data_len);
  send(client_fd, frame, 8 + data_len, 0);
}

void SunSpecProxy::send_tcp_error_(int client_fd, uint16_t txn_id, uint8_t unit_id,
                                    uint8_t fc, uint8_t err) {
  uint8_t data[1] = {err};
  send_tcp_response_(client_fd, txn_id, unit_id, fc | 0x80, data, 1);
}

// ============================================================
// SunSpec Register Access
// ============================================================

bool SunSpecProxy::read_sunspec_registers_(uint16_t start_reg, uint16_t count, uint16_t *out) {
  if (start_reg < SUNSPEC_BASE) return false;
  uint16_t off = start_reg - SUNSPEC_BASE;
  if (off + count > TOTAL_REGS) return false;
  memcpy(out, &register_map_[off], count * 2);
  return true;
}

bool SunSpecProxy::write_sunspec_registers_(uint16_t start_reg, uint16_t count, const uint16_t *values) {
  if (start_reg < SUNSPEC_BASE) return false;
  uint16_t off = start_reg - SUNSPEC_BASE;

  if (off < OFF_M123 + 2 || off + count > OFF_END) {
    ESP_LOGW(TAG, "TCP: Write rejected — register %d not in Model 123 writable area", start_reg);
    return false;
  }

  for (uint16_t i = 0; i < count; i++) register_map_[off + i] = values[i];

  uint16_t lim_off = OFF_M123 + 2 + 5;
  uint16_t ena_off = OFF_M123 + 2 + 8;
  bool changed = false;
  for (uint16_t r = off; r < off + count; r++) {
    if (r == lim_off || r == ena_off) { changed = true; break; }
  }

  if (changed) {
    uint16_t pct = register_map_[lim_off];
    uint16_t ena = register_map_[ena_off];
    ESP_LOGI(TAG, "VICTRON: Power limit command — %.1f%%, enabled=%d", pct / 10.0f, ena);
    forward_power_limit_(pct, ena == 1);
  }

  return true;
}

// ============================================================
// Power Limit Forwarding
// ============================================================

void SunSpecProxy::forward_power_limit_(uint16_t pct_raw, bool enabled) {
  // TODO: Implement power limit forwarding via Modbus TCP
  // DTU-Pro control register mapping for TCP mode needs to be tested
  // For now, just log the command
  
  float pct = pct_raw / 10.0f;
  if (enabled) {
    ESP_LOGI(TAG, "VICTRON: Power limit command received: %.1f%% (forwarding not yet implemented in TCP mode)", pct);
  } else {
    ESP_LOGI(TAG, "VICTRON: Power limit removed (forwarding not yet implemented in TCP mode)");
  }
  
  // Note: The DTU-Pro control registers (0xC000 series) may work differently over TCP
  // than they did over RTU. This needs testing with the actual DTU before implementing.
}

// ============================================================
// Modbus TCP Client (DTU-Pro Polling)
// ============================================================

bool SunSpecProxy::connect_to_dtu_() {
  if (dtu_fd_ >= 0) return true;  // Already connected
  
  uint32_t now = millis();
  if (now - last_dtu_connect_attempt_ < 5000) return false;  // Throttle reconnects
  last_dtu_connect_attempt_ = now;
  
  ESP_LOGI(TAG, "DTU: Connecting to %s:%d...", dtu_host_.c_str(), dtu_port_);
  
  // Resolve hostname
  struct hostent *host = gethostbyname(dtu_host_.c_str());
  if (!host) {
    ESP_LOGW(TAG, "DTU: DNS lookup failed for %s", dtu_host_.c_str());
    dtu_poll_fail_count_++;
    return false;
  }
  
  // Create socket
  dtu_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (dtu_fd_ < 0) {
    ESP_LOGE(TAG, "DTU: Socket create failed: errno=%d", errno);
    dtu_poll_fail_count_++;
    return false;
  }
  
  // Set non-blocking
  fcntl(dtu_fd_, F_SETFL, fcntl(dtu_fd_, F_GETFL, 0) | O_NONBLOCK);
  
  // Connect
  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(dtu_port_);
  memcpy(&addr.sin_addr, host->h_addr, host->h_length);
  
  int res = connect(dtu_fd_, (struct sockaddr *)&addr, sizeof(addr));
  if (res < 0 && errno != EINPROGRESS) {
    ESP_LOGW(TAG, "DTU: Connect failed: errno=%d", errno);
    close(dtu_fd_);
    dtu_fd_ = -1;
    dtu_poll_fail_count_++;
    return false;
  }
  
  // Wait for connection (with timeout)
  fd_set wfds;
  struct timeval tv;
  FD_ZERO(&wfds);
  FD_SET(dtu_fd_, &wfds);
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  
  res = select(dtu_fd_ + 1, nullptr, &wfds, nullptr, &tv);
  if (res <= 0) {
    ESP_LOGW(TAG, "DTU: Connect timeout");
    close(dtu_fd_);
    dtu_fd_ = -1;
    dtu_poll_fail_count_++;
    return false;
  }
  
  // Check socket error
  int err = 0;
  socklen_t len = sizeof(err);
  getsockopt(dtu_fd_, SOL_SOCKET, SO_ERROR, &err, &len);
  if (err) {
    ESP_LOGW(TAG, "DTU: Connect failed: err=%d", err);
    close(dtu_fd_);
    dtu_fd_ = -1;
    dtu_poll_fail_count_++;
    return false;
  }
  
  dtu_connected_ = true;
  ESP_LOGI(TAG, "DTU: Connected successfully");
  return true;
}

void SunSpecProxy::close_dtu_connection_() {
  if (dtu_fd_ >= 0) {
    close(dtu_fd_);
    dtu_fd_ = -1;
    dtu_connected_ = false;
  }
}

bool SunSpecProxy::send_modbus_tcp_request_(uint8_t function, uint16_t reg_start, uint16_t reg_count) {
  if (!dtu_connected_) return false;
  
  uint8_t frame[12];
  put_be16(&frame[0], modbus_transaction_id_);
  put_be16(&frame[2], 0);  // Protocol ID = 0
  put_be16(&frame[4], 6);  // Length = 6 (unit_id + function + data)
  frame[6] = dtu_address_;
  frame[7] = function;
  put_be16(&frame[8], reg_start);
  put_be16(&frame[10], reg_count);
  
  int sent = send(dtu_fd_, frame, 12, 0);
  if (sent != 12) {
    ESP_LOGW(TAG, "DTU: Send failed (%d bytes, errno=%d)", sent, errno);
    close_dtu_connection_();
    return false;
  }
  
  modbus_transaction_id_++;
  return true;
}

int SunSpecProxy::read_modbus_tcp_response_(uint8_t *buf, int max_len) {
  if (!dtu_connected_) return 0;
  
  // Read with timeout
  fd_set rfds;
  struct timeval tv;
  FD_ZERO(&rfds);
  FD_SET(dtu_fd_, &rfds);
  tv.tv_sec = tcp_timeout_ms_ / 1000;
  tv.tv_usec = (tcp_timeout_ms_ % 1000) * 1000;
  
  int res = select(dtu_fd_ + 1, &rfds, nullptr, nullptr, &tv);
  if (res <= 0) {
    if (res == 0) ESP_LOGW(TAG, "DTU: Read timeout");
    else ESP_LOGW(TAG, "DTU: Select error: errno=%d", errno);
    close_dtu_connection_();
    return -1;
  }
  
  int n = recv(dtu_fd_, buf, max_len, 0);
  if (n <= 0) {
    ESP_LOGW(TAG, "DTU: Connection closed (recv=%d, errno=%d)", n, errno);
    close_dtu_connection_();
    return -1;
  }
  
  // Validate MBAP header
  if (n < 8) {
    ESP_LOGW(TAG, "DTU: Short response (%d bytes)", n);
    return -1;
  }
  
  uint16_t trans_id = be16(&buf[0]);
  uint16_t proto_id = be16(&buf[2]);
  uint16_t length = be16(&buf[4]);
  
  if (proto_id != 0) {
    ESP_LOGW(TAG, "DTU: Invalid protocol ID: %d", proto_id);
    return -1;
  }
  
  // Check for exception
  if (buf[7] & 0x80) {
    uint8_t exc = n >= 9 ? buf[8] : 0;
    ESP_LOGW(TAG, "DTU: Modbus exception: func=0x%02X, exc=%d", buf[7], exc);
    return -1;
  }
  
  return n;
}

// NEW poll_dtu_data_ implementation
void SunSpecProxy::poll_dtu_data_() {
  uint32_t now = millis();
  
  // Time for next poll?
  if (now - last_poll_time_ < poll_interval_ms_) return;
  last_poll_time_ = now;
  
  // Ensure connection
  if (!connect_to_dtu_()) {
    ESP_LOGW(TAG, "DTU: Not connected, skipping poll");
    return;
  }
  
  // Read all 200 registers from 0x4000 (8 MPPT channels × 25 regs)
  // Split into two reads if needed (max 125 regs per Modbus read)
  ESP_LOGD(TAG, "DTU: Reading %d registers from 0x%04X", HM_TOTAL_REGS, HM_DATA_BASE);
  
  // Read first 125 registers
  if (!send_modbus_tcp_request_(0x03, HM_DATA_BASE, 125)) {
    ESP_LOGW(TAG, "DTU: Failed to send request (chunk 1)");
    dtu_poll_fail_count_++;
    return;
  }
  
  uint8_t resp[512];
  int n = read_modbus_tcp_response_(resp, sizeof(resp));
  if (n < 0) {
    ESP_LOGW(TAG, "DTU: Failed to read response (chunk 1)");
    dtu_poll_fail_count_++;
    return;
  }
  
  // Parse first chunk
  if (n >= 9 && resp[7] == 0x03) {
    uint8_t byte_count = resp[8];
    int reg_count = byte_count / 2;
    if (reg_count >= 125) {
      for (int i = 0; i < 125; i++) {
        dtu_regs_[i] = be16(&resp[9 + i * 2]);
      }
    } else {
      ESP_LOGW(TAG, "DTU: Short response chunk 1: %d regs", reg_count);
      dtu_poll_fail_count_++;
      return;
    }
  } else {
    ESP_LOGW(TAG, "DTU: Invalid response chunk 1");
    dtu_poll_fail_count_++;
    return;
  }
  
  // Read remaining 75 registers
  if (!send_modbus_tcp_request_(0x03, HM_DATA_BASE + 125, 75)) {
    ESP_LOGW(TAG, "DTU: Failed to send request (chunk 2)");
    dtu_poll_fail_count_++;
    return;
  }
  
  n = read_modbus_tcp_response_(resp, sizeof(resp));
  if (n < 0) {
    ESP_LOGW(TAG, "DTU: Failed to read response (chunk 2)");
    dtu_poll_fail_count_++;
    return;
  }
  
  // Parse second chunk
  if (n >= 9 && resp[7] == 0x03) {
    uint8_t byte_count = resp[8];
    int reg_count = byte_count / 2;
    if (reg_count >= 75) {
      for (int i = 0; i < 75; i++) {
        dtu_regs_[125 + i] = be16(&resp[9 + i * 2]);
      }
      dtu_data_valid_ = true;
      dtu_poll_count_++;
      last_dtu_poll_ok_ms_ = now;
    } else {
      ESP_LOGW(TAG, "DTU: Short response chunk 2: %d regs", reg_count);
      dtu_poll_fail_count_++;
      return;
    }
  } else {
    ESP_LOGW(TAG, "DTU: Invalid response chunk 2");
    dtu_poll_fail_count_++;
    return;
  }
  
  ESP_LOGI(TAG, "DTU: Successfully read 200 registers (poll count: %lu)", dtu_poll_count_);
  
  // Parse register data
  parse_dtu_registers_(dtu_regs_, HM_TOTAL_REGS);
  
  // Map MPPT channels to inverters
  map_mppt_to_inverters_();
  
  // Aggregate per-inverter data
  for (int i = 0; i < num_sources_; i++) {
    aggregate_inverter_data_(i);
  }
  
  // Update SunSpec registers
  aggregate_and_update_registers_();
}

void SunSpecProxy::parse_dtu_registers_(const uint16_t *regs, int reg_count) {
  ESP_LOGD(TAG, "DTU: Parsing %d registers into MPPT channel data", reg_count);
  
  // Clear all inverter MPPT data first
  for (int i = 0; i < num_sources_; i++) {
    sources_[i].mppt_count = 0;
    for (int m = 0; m < MAX_MPPT_PER_INVERTER; m++) {
      sources_[i].mppt[m].data_valid = false;
    }
  }
  
  // Parse each 25-register block
  int channels_found = 0;
  for (int ch = 0; ch < HM_MAX_CHANNELS && ch * HM_MPPT_STRIDE < reg_count; ch++) {
    const uint16_t *ch_regs = &regs[ch * HM_MPPT_STRIDE];
    
    // Check marker
    uint16_t marker = ch_regs[HM_MARKER];
    if (marker != 12) {
      ESP_LOGV(TAG, "DTU: Channel %d marker invalid (%d), skipping", ch, marker);
      continue;
    }
    
    // Extract inverter serial number (3 regs = 6 bytes)
    char inv_sn[13];
    uint8_t sn_bytes[6];
    sn_bytes[0] = (ch_regs[HM_INV_SN_1] >> 8) & 0xFF;
    sn_bytes[1] = ch_regs[HM_INV_SN_1] & 0xFF;
    sn_bytes[2] = (ch_regs[HM_INV_SN_2] >> 8) & 0xFF;
    sn_bytes[3] = ch_regs[HM_INV_SN_2] & 0xFF;
    sn_bytes[4] = (ch_regs[HM_INV_SN_3] >> 8) & 0xFF;
    sn_bytes[5] = ch_regs[HM_INV_SN_3] & 0xFF;
    
    for (int j = 0; j < 6; j++) {
      snprintf(&inv_sn[j*2], 3, "%02x", sn_bytes[j]);
    }
    inv_sn[12] = 0;
    
    // Extract MPPT number
    uint16_t mppt_num = ch_regs[HM_MPPT_NUM];
    
    // Find matching inverter by serial number
    int inv_idx = -1;
    for (int i = 0; i < num_sources_; i++) {
      if (strlen(sources_[i].serial_number) > 0 && strcmp(sources_[i].serial_number, inv_sn) == 0) {
        inv_idx = i;
        break;
      }
    }
    
    if (inv_idx < 0) {
      ESP_LOGD(TAG, "DTU: Channel %d: SN=%s MPPT=%d (no matching inverter config)", ch, inv_sn, mppt_num);
      continue;
    }
    
    auto &inv = sources_[inv_idx];
    
    // Find or create MPPT slot
    int mppt_idx = -1;
    for (int m = 0; m < inv.mppt_count; m++) {
      if (inv.mppt[m].mppt_num == mppt_num) {
        mppt_idx = m;
        break;
      }
    }
    if (mppt_idx < 0 && inv.mppt_count < MAX_MPPT_PER_INVERTER) {
      mppt_idx = inv.mppt_count++;
    }
    if (mppt_idx < 0) {
      ESP_LOGW(TAG, "DTU: No MPPT slot available for %s MPPT%d", inv.name, mppt_num);
      continue;
    }
    
    auto &mppt = inv.mppt[mppt_idx];
    mppt.mppt_num = mppt_num;
    mppt.data_valid = true;
    
    // Decode values with correct scaling
    mppt.dc_voltage_v = (float)ch_regs[HM_DC_VOLTAGE] / 10.0f;
    mppt.dc_current_a = (float)ch_regs[HM_DC_CURRENT] / 100.0f;  // ÷100, not ÷10!
    mppt.ac_voltage_v = (float)ch_regs[HM_AC_VOLTAGE] / 10.0f;
    mppt.frequency_hz = (float)ch_regs[HM_FREQUENCY] / 100.0f;
    mppt.power_w = (float)ch_regs[HM_POWER] / 10.0f;
    mppt.today_energy_wh = (float)ch_regs[HM_TODAY_ENERGY];
    
    uint32_t total_wh = ((uint32_t)ch_regs[HM_TOTAL_ENERGY_H] << 16) | ch_regs[HM_TOTAL_ENERGY_L];
    mppt.total_energy_kwh = (float)total_wh / 1000.0f;
    
    mppt.temperature_c = (float)(int16_t)ch_regs[HM_TEMPERATURE] / 10.0f;
    mppt.status = ch_regs[HM_STATUS];
    
    channels_found++;
    
    ESP_LOGD(TAG, "DTU: Ch%d → %s MPPT%d: %.0fW (DC: %.1fV/%.2fA), AC: %.0fV/%.2fHz, Today: %.0fWh, Total: %.1fkWh, T: %.1f°C, Status: %d",
             ch, inv.name, mppt_num, mppt.power_w, 
             mppt.dc_voltage_v, mppt.dc_current_a,
             mppt.ac_voltage_v, mppt.frequency_hz,
             mppt.today_energy_wh, mppt.total_energy_kwh,
             mppt.temperature_c, mppt.status);
  }
  
  ESP_LOGI(TAG, "DTU: Parsed %d MPPT channels from register data", channels_found);
}

void SunSpecProxy::map_mppt_to_inverters_() {
  // Already done in parse_dtu_registers_ via serial number matching
}

void SunSpecProxy::aggregate_inverter_data_(int inv_idx) {
  auto &inv = sources_[inv_idx];
  
  // Reset aggregates
  inv.power_w = 0;
  inv.voltage_v = 0;
  inv.current_a = 0;
  inv.frequency_hz = 0;
  inv.energy_kwh = 0;
  inv.today_energy_wh = 0;
  inv.temperature_c = NAN;
  inv.pv_voltage_v = 0;
  inv.pv_current_a = 0;
  inv.pv_power_w = 0;
  inv.producing = false;
  inv.data_valid = false;
  
  if (inv.mppt_count == 0) return;
  
  int valid_count = 0;
  float max_temp = NAN;
  
  for (int m = 0; m < inv.mppt_count; m++) {
    auto &mppt = inv.mppt[m];
    if (!mppt.data_valid) continue;
    
    valid_count++;
    inv.power_w += mppt.power_w;
    inv.pv_power_w += mppt.power_w;  // For microinverters, AC ≈ DC
    inv.voltage_v += mppt.ac_voltage_v;
    inv.frequency_hz += mppt.frequency_hz;
    inv.today_energy_wh += mppt.today_energy_wh;
    inv.energy_kwh += mppt.total_energy_kwh;
    
    // DC side
    inv.pv_voltage_v += mppt.dc_voltage_v;
    inv.pv_current_a += mppt.dc_current_a;
    
    // Max temperature
    if (std::isnan(max_temp) || mppt.temperature_c > max_temp) {
      max_temp = mppt.temperature_c;
    }
    
    if (mppt.power_w > 0) inv.producing = true;
  }
  
  if (valid_count > 0) {
    inv.voltage_v /= valid_count;
    inv.frequency_hz /= valid_count;
    inv.pv_voltage_v /= valid_count;
    inv.temperature_c = max_temp;
    inv.data_valid = true;
    inv.last_poll_ms = millis();
    
    // Estimate current from power/voltage
    if (inv.voltage_v > 0) {
      inv.current_a = inv.power_w / inv.voltage_v;
    }
    
    inv.poll_success_count++;
    
    ESP_LOGI(TAG, "INV: %s — P=%.0fW (DC: %.1fV/%.2fA=%.0fW), AC: %.1fV/%.2fHz, Today=%.0fWh, Total=%.1fkWh, T=%.1f°C (%d MPPTs)",
             inv.name, inv.power_w,
             inv.pv_voltage_v, inv.pv_current_a, inv.pv_power_w,
             inv.voltage_v, inv.frequency_hz,
             inv.today_energy_wh, inv.energy_kwh,
             inv.temperature_c, valid_count);
  }
}

void SunSpecProxy::publish_mppt_sensors_(int inv_idx, int mppt_idx) {
  auto &inv = sources_[inv_idx];
  if (mppt_idx >= inv.mppt_count) return;
  
  auto &mppt = inv.mppt[mppt_idx];
  if (!mppt.data_valid) return;
  
  // Publish per-MPPT sensors if configured
  if (mppt_dc_voltage_sensors_[inv_idx][mppt_idx]) 
    mppt_dc_voltage_sensors_[inv_idx][mppt_idx]->publish_state(mppt.dc_voltage_v);
  if (mppt_dc_current_sensors_[inv_idx][mppt_idx]) 
    mppt_dc_current_sensors_[inv_idx][mppt_idx]->publish_state(mppt.dc_current_a);
  if (mppt_dc_power_sensors_[inv_idx][mppt_idx]) 
    mppt_dc_power_sensors_[inv_idx][mppt_idx]->publish_state(mppt.dc_voltage_v * mppt.dc_current_a);
  if (mppt_ac_voltage_sensors_[inv_idx][mppt_idx]) 
    mppt_ac_voltage_sensors_[inv_idx][mppt_idx]->publish_state(mppt.ac_voltage_v);
  if (mppt_frequency_sensors_[inv_idx][mppt_idx]) 
    mppt_frequency_sensors_[inv_idx][mppt_idx]->publish_state(mppt.frequency_hz);
  if (mppt_power_sensors_[inv_idx][mppt_idx]) 
    mppt_power_sensors_[inv_idx][mppt_idx]->publish_state(mppt.power_w);
  if (mppt_today_energy_sensors_[inv_idx][mppt_idx]) 
    mppt_today_energy_sensors_[inv_idx][mppt_idx]->publish_state(mppt.today_energy_wh);
  if (mppt_total_energy_sensors_[inv_idx][mppt_idx]) 
    mppt_total_energy_sensors_[inv_idx][mppt_idx]->publish_state(mppt.total_energy_kwh);
  if (mppt_temperature_sensors_[inv_idx][mppt_idx]) 
    mppt_temperature_sensors_[inv_idx][mppt_idx]->publish_state(mppt.temperature_c);
}

}  // namespace sunspec_proxy
}  // namespace esphome

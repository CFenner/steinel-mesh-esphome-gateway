// Generic multi-node engine.
//
// A small scheduler that reads and controls every node stored from the imported
// backup (lamps, sensors) through standard SIG models. It uses the single
// acknowledged-access slot, so at most one request is ever in flight.
#include "steinel_mesh.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_idf_version.h"

namespace esphome {
namespace steinel_mesh {

static const char *const NODE_TAG = "steinel_nodes";
// Idle time after each request, so the radio stays free for replies.
static constexpr uint32_t NODE_REQUEST_GAP_MS = 350;
static constexpr uint32_t NODE_COMMAND_GAP_MS = 150;
static constexpr uint32_t NODE_PASS_INTERVAL_MS = 20000;
// The first pass waits so the Mesh can settle after start-up.
static constexpr uint32_t NODE_FIRST_PASS_DELAY_MS = 8000;
// Light Control "ambient lux on" property, the twilight threshold. Values are
// 24-bit illuminance in 0.01 lx; the API accepts whole lux from 1 to 1500.
static constexpr uint16_t LC_LIGHT_ON_THRESHOLD_PROPERTY = 0x002B;
static constexpr uint32_t THRESHOLD_MIN_CENTILUX = 100;
static constexpr uint32_t THRESHOLD_MAX_CENTILUX = 150000;
// The composition reply is segmented and can take several seconds.
static constexpr uint32_t NODE_COMPOSITION_TIMEOUT_MS = 4000;
// Ask a device for its version this many times at most before giving up.
static constexpr uint8_t NODE_VERSION_MAX_ATTEMPTS = 5;
static constexpr uint8_t NODE_TTL = 7;
// Present Ambient Light Level, asked for explicitly on sensor elements that never
// answered the plain Sensor Get. Tried a limited number of times per device.
static constexpr uint16_t PROPERTY_AMBIENT_LIGHT_LEVEL = 0x004E;
static constexpr uint8_t NODE_SENSOR_PROBE_MAX_ATTEMPTS = 5;
// Light Control "Time Run On", shown as "Run time" (the Steinel app calls it
// "regular time" and the main light time): how long the light stays on after
// the last motion.
// It is a 24-bit time in milliseconds; 0xFFFFFF means "unknown".
static constexpr uint16_t LC_TIME_RUN_ON_PROPERTY = 0x003C;
// The read is repeated for a device that has not answered yet: the first few
// passes every time, then only now and then, so a device that never answers
// does not cost traffic forever.
static constexpr uint8_t NODE_RUN_TIME_QUICK_ATTEMPTS = 6;
static constexpr uint8_t NODE_RUN_TIME_SLOW_EVERY = 10;

namespace {

// Returns the element that carries the light output: the one with a Light
// Lightness server, otherwise the first with a Generic OnOff server.
int light_element(const std::array<uint16_t, 6> &caps, size_t count, uint16_t cap_lightness,
                  uint16_t cap_onoff) {
  const size_t limit = std::min(count, caps.size());
  for (size_t i = 0; i < limit; i++)
    if ((caps[i] & cap_lightness) != 0) return static_cast<int>(i);
  for (size_t i = 0; i < limit; i++)
    if ((caps[i] & cap_onoff) != 0) return static_cast<int>(i);
  return -1;
}

int first_element_with(const std::array<uint16_t, 6> &caps, size_t count, uint16_t cap) {
  const size_t limit = std::min(count, caps.size());
  for (size_t i = 0; i < limit; i++)
    if ((caps[i] & cap) != 0) return static_cast<int>(i);
  return -1;
}

bool parse_switch(const std::string &text, int &value) {
  if (text == "1" || text == "true" || text == "on") { value = 1; return true; }
  if (text == "0" || text == "false" || text == "off") { value = 0; return true; }
  return false;
}

}  // namespace

bool NightmatiqMesh::find_node_index_(uint16_t address, uint8_t &index) const {
  for (uint16_t i = 0; i < this->node_table_.count && i < this->node_table_.nodes.size(); i++) {
    const StoredNode &node = this->node_table_.nodes[i];
    if (address >= node.address && address < node.address + node.element_count) {
      index = static_cast<uint8_t>(i);
      return true;
    }
  }
  return false;
}

void NightmatiqMesh::build_node_poll_plan_(int only_node) {
  // Reads for one node: light output state, LC mode, then every sensor element.
  const auto add_node = [this](uint8_t index, std::vector<NodeRequest> &plan, bool probe) {
    const StoredNode &node = this->node_table_.nodes[index];
    {
      // The firmware version is read live from the device's composition data.
      // Ask until it is known, but not endlessly for a device that never answers.
      std::lock_guard<std::mutex> lock(this->node_mutex_);
      const NodeRuntime &state = this->node_runtime_[index];
      if (state.version_id == 0 && state.version_attempts < NODE_VERSION_MAX_ATTEMPTS)
        plan.push_back({NodeRequestKind::COMPOSITION_GET, index, 0, 0, false});
    }
    const int light = light_element(node.element_caps, node.element_count, CAP_LIGHTNESS, CAP_ONOFF);
    if (light >= 0) {
      plan.push_back({NodeRequestKind::ONOFF_GET, index, static_cast<uint8_t>(light), 0, false});
      if ((node.element_caps[light] & CAP_LIGHTNESS) != 0)
        plan.push_back({NodeRequestKind::LIGHTNESS_GET, index, static_cast<uint8_t>(light), 0, false});
    }
    const int lc = first_element_with(node.element_caps, node.element_count, CAP_LC);
    if (lc >= 0) plan.push_back({NodeRequestKind::LC_MODE_GET, index, static_cast<uint8_t>(lc), 0, false});
    // Only nodes with a sensor have a meaningful twilight threshold.
    const bool has_sensor = std::any_of(node.element_caps.begin(), node.element_caps.end(),
                                        [](uint16_t caps) { return (caps & CAP_SENSOR) != 0; });
    if (lc >= 0 && has_sensor)
      plan.push_back({NodeRequestKind::THRESHOLD_GET, index, static_cast<uint8_t>(lc), 0, false});
    const size_t limit = std::min<size_t>(node.element_count, node.element_caps.size());
    for (size_t element = 0; element < limit; element++)
      if ((node.element_caps[element] & CAP_SENSOR) != 0)
        plan.push_back({NodeRequestKind::SENSOR_GET, index, static_cast<uint8_t>(element), 0, false});
    if (lc >= 0) {
      // Read the run time while it is unknown, and again after a command to
      // this node so a change is confirmed.
      std::lock_guard<std::mutex> lock(this->node_mutex_);
      NodeRuntime &state = this->node_runtime_[index];
      bool ask = !probe;
      if (probe && state.run_time_ms < 0) {
        state.run_time_attempts++;
        ask = state.run_time_attempts <= NODE_RUN_TIME_QUICK_ATTEMPTS ||
              state.run_time_attempts % NODE_RUN_TIME_SLOW_EVERY == 0;
      }
      if (ask) plan.push_back({NodeRequestKind::RUN_TIME_GET, index, static_cast<uint8_t>(lc), 0, false});
    }
    if (probe) {
      // A sensor element that has never produced a reading may need to be asked for
      // the ambient light level explicitly, and its descriptor tells which properties
      // it offers. Do that a few times only, so a sensor that has nothing to report
      // does not cost traffic forever.
      std::lock_guard<std::mutex> lock(this->node_mutex_);
      NodeRuntime &state = this->node_runtime_[index];
      if (state.sensor_probe_attempts < NODE_SENSOR_PROBE_MAX_ATTEMPTS) {
        bool probed = false;
        for (size_t element = 0; element < limit; element++) {
          if ((node.element_caps[element] & CAP_SENSOR) == 0) continue;
          bool has_reading = false;
          for (uint8_t i = 0; i < state.sensor_count; i++)
            if (state.sensors[i].element == element) has_reading = true;
          if (has_reading) continue;
          plan.push_back({NodeRequestKind::SENSOR_PROPERTY_GET, index, static_cast<uint8_t>(element),
                          PROPERTY_AMBIENT_LIGHT_LEVEL, false});
          if (state.sensor_probe_attempts == 0)
            plan.push_back({NodeRequestKind::SENSOR_DESCRIPTOR_GET, index, static_cast<uint8_t>(element), 0, false});
          probed = true;
        }
        if (probed) state.sensor_probe_attempts++;
      }
    }
  };

  if (only_node >= 0) {
    // A refresh after a command runs before the remainder of the current pass.
    std::vector<NodeRequest> extra;
    add_node(static_cast<uint8_t>(only_node), extra, false);
    this->node_poll_plan_.insert(this->node_poll_plan_.begin() + this->node_poll_pos_, extra.begin(),
                                 extra.end());
    return;
  }
  this->node_poll_plan_.clear();
  this->node_poll_pos_ = 0;
  for (uint16_t i = 0; i < this->node_table_.count && i < this->node_table_.nodes.size(); i++)
    add_node(static_cast<uint8_t>(i), this->node_poll_plan_, true);
}

bool NightmatiqMesh::send_node_request_(const NodeRequest &request) {
  if (request.node >= this->node_table_.count) return false;
  const StoredNode &node = this->node_table_.nodes[request.node];
  const uint16_t destination = static_cast<uint16_t>(node.address + request.element);
  esp_ble_mesh_client_common_param_t common{};

  this->node_inflight_ = request;
  this->node_inflight_valid_ = true;

  switch (request.kind) {
    case NodeRequestKind::ONOFF_GET: {
      esp_ble_mesh_generic_client_get_state_t get{};
      if (!this->set_common_(common, onoff_model_(), ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_ONOFF_GET, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET))
        break;
      return this->record_access_send_result_(AccessOperation::NODE_ONOFF_GET, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET,
                                              esp_ble_mesh_generic_client_get_state(&common, &get));
    }
    case NodeRequestKind::LIGHTNESS_GET: {
      esp_ble_mesh_light_client_get_state_t get{};
      if (!this->set_common_(common, light_lightness_model_(), ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_GET,
                             destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_LIGHTNESS_GET,
                                         ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_GET))
        break;
      return this->record_access_send_result_(AccessOperation::NODE_LIGHTNESS_GET,
                                              ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_GET,
                                              esp_ble_mesh_light_client_get_state(&common, &get));
    }
    case NodeRequestKind::LC_MODE_GET: {
      esp_ble_mesh_light_client_get_state_t get{};
      if (!this->set_common_(common, light_lc_model_(), ESP_BLE_MESH_MODEL_OP_LIGHT_LC_MODE_GET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_LC_MODE_GET,
                                         ESP_BLE_MESH_MODEL_OP_LIGHT_LC_MODE_GET))
        break;
      return this->record_access_send_result_(AccessOperation::NODE_LC_MODE_GET,
                                              ESP_BLE_MESH_MODEL_OP_LIGHT_LC_MODE_GET,
                                              esp_ble_mesh_light_client_get_state(&common, &get));
    }
    case NodeRequestKind::SENSOR_GET: {
      esp_ble_mesh_sensor_client_get_state_t get{};
      if (!this->set_common_(common, sensor_model_(), ESP_BLE_MESH_MODEL_OP_SENSOR_GET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_SENSOR_GET, ESP_BLE_MESH_MODEL_OP_SENSOR_GET))
        break;
      // Without a Property ID the server returns every sensor value it has, so
      // the response also tells us which properties these devices expose.
      get.sensor_get.op_en = false;
      return this->record_access_send_result_(AccessOperation::NODE_SENSOR_GET, ESP_BLE_MESH_MODEL_OP_SENSOR_GET,
                                              esp_ble_mesh_sensor_client_get_state(&common, &get));
    }
    case NodeRequestKind::SENSOR_PROPERTY_GET: {
      esp_ble_mesh_sensor_client_get_state_t get{};
      if (!this->set_common_(common, sensor_model_(), ESP_BLE_MESH_MODEL_OP_SENSOR_GET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_SENSOR_GET, ESP_BLE_MESH_MODEL_OP_SENSOR_GET))
        break;
      get.sensor_get.op_en = true;
      get.sensor_get.property_id = request.value;
      return this->record_access_send_result_(AccessOperation::NODE_SENSOR_GET, ESP_BLE_MESH_MODEL_OP_SENSOR_GET,
                                              esp_ble_mesh_sensor_client_get_state(&common, &get));
    }
    case NodeRequestKind::SENSOR_DESCRIPTOR_GET: {
      esp_ble_mesh_sensor_client_get_state_t get{};
      if (!this->set_common_(common, sensor_model_(), ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_SENSOR_GET,
                                         ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET))
        break;
      get.descriptor_get.op_en = false;
      return this->record_access_send_result_(AccessOperation::NODE_SENSOR_GET,
                                              ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET,
                                              esp_ble_mesh_sensor_client_get_state(&common, &get));
    }
    case NodeRequestKind::COMPOSITION_GET: {
      // Configuration messages are encrypted with the node's device key.
      esp_ble_mesh_cfg_client_get_state_t get{};
      esp_ble_mesh_model_t *model = config_model_();
      if (!this->mesh_ready_.load() || model == nullptr) break;
      model->keys[0] = ESP_BLE_MESH_KEY_DEV;
      common.opcode = ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET;
      common.model = model;
      common.ctx.net_idx = this->config_.net_key_index;
      common.ctx.app_idx = ESP_BLE_MESH_KEY_DEV;
      common.ctx.addr = destination;
      common.ctx.send_ttl = NODE_TTL;
      common.msg_timeout = NODE_COMPOSITION_TIMEOUT_MS;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
      common.msg_role = ROLE_PROVISIONER;
#endif
      get.comp_data_get.page = 0;
      if (!this->begin_access_operation_(AccessOperation::NODE_COMPOSITION_GET,
                                         ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET))
        break;
      // Keep the access slot until the client itself times out. Otherwise the
      // watchdog could release it first and a late reply would be taken for the
      // primary device's own composition query.
      this->access_deadline_.store(millis() + NODE_COMPOSITION_TIMEOUT_MS + 750);
      {
        std::lock_guard<std::mutex> lock(this->node_mutex_);
        if (this->node_runtime_[request.node].version_attempts < 255) this->node_runtime_[request.node].version_attempts++;
      }
      return this->record_access_send_result_(AccessOperation::NODE_COMPOSITION_GET,
                                              ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET,
                                              esp_ble_mesh_config_client_get_state(&common, &get));
    }
    case NodeRequestKind::ONOFF_SET: {
      esp_ble_mesh_generic_client_set_state_t set{};
      if (!this->set_common_(common, onoff_model_(), ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_ONOFF_SET, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET))
        break;
      set.onoff_set.op_en = false;
      set.onoff_set.onoff = request.value != 0 ? 1 : 0;
      set.onoff_set.tid = this->next_tid_();
      return this->record_access_send_result_(AccessOperation::NODE_ONOFF_SET, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET,
                                              esp_ble_mesh_generic_client_set_state(&common, &set));
    }
    case NodeRequestKind::LIGHTNESS_SET: {
      esp_ble_mesh_light_client_set_state_t set{};
      if (!this->set_common_(common, light_lightness_model_(), ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET,
                             destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_LIGHTNESS_SET,
                                         ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET))
        break;
      set.lightness_set.op_en = false;
      set.lightness_set.lightness = request.value;
      set.lightness_set.tid = this->next_tid_();
      return this->record_access_send_result_(AccessOperation::NODE_LIGHTNESS_SET,
                                              ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET,
                                              esp_ble_mesh_light_client_set_state(&common, &set));
    }
    case NodeRequestKind::THRESHOLD_GET: {
      esp_ble_mesh_light_client_get_state_t get{};
      if (!this->set_common_(common, light_lc_model_(), ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_GET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_THRESHOLD_GET,
                                         ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_GET))
        break;
      get.lc_property_get.property_id = LC_LIGHT_ON_THRESHOLD_PROPERTY;
      return this->record_access_send_result_(AccessOperation::NODE_THRESHOLD_GET,
                                              ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_GET,
                                              esp_ble_mesh_light_client_get_state(&common, &get));
    }
    case NodeRequestKind::RUN_TIME_GET: {
      esp_ble_mesh_light_client_get_state_t get{};
      if (!this->set_common_(common, light_lc_model_(), ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_GET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_THRESHOLD_GET,
                                         ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_GET))
        break;
      get.lc_property_get.property_id = LC_TIME_RUN_ON_PROPERTY;
      return this->record_access_send_result_(AccessOperation::NODE_THRESHOLD_GET,
                                              ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_GET,
                                              esp_ble_mesh_light_client_get_state(&common, &get));
    }
    case NodeRequestKind::RUN_TIME_SET: {
      esp_ble_mesh_light_client_set_state_t set{};
      if (!this->set_common_(common, light_lc_model_(), ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_SET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_THRESHOLD_SET,
                                         ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_SET))
        break;
      const uint32_t milliseconds = static_cast<uint32_t>(request.value) * 1000U;
      this->node_threshold_storage_[0] = milliseconds & 0xFF;
      this->node_threshold_storage_[1] = (milliseconds >> 8) & 0xFF;
      this->node_threshold_storage_[2] = (milliseconds >> 16) & 0xFF;
      this->node_threshold_buffer_.data = this->node_threshold_storage_.data();
      this->node_threshold_buffer_.len = this->node_threshold_storage_.size();
      this->node_threshold_buffer_.size = this->node_threshold_storage_.size();
      this->node_threshold_buffer_.__buf = this->node_threshold_storage_.data();
      set.lc_property_set.property_id = LC_TIME_RUN_ON_PROPERTY;
      set.lc_property_set.property_value = &this->node_threshold_buffer_;
      return this->record_access_send_result_(AccessOperation::NODE_THRESHOLD_SET,
                                              ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_SET,
                                              esp_ble_mesh_light_client_set_state(&common, &set));
    }
    case NodeRequestKind::THRESHOLD_SET: {
      esp_ble_mesh_light_client_set_state_t set{};
      if (!this->set_common_(common, light_lc_model_(), ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_SET, destination))
        break;
      if (!this->begin_access_operation_(AccessOperation::NODE_THRESHOLD_SET,
                                         ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_SET))
        break;
      const uint32_t centilux = static_cast<uint32_t>(request.value) * 100U;
      this->node_threshold_storage_[0] = centilux & 0xFF;
      this->node_threshold_storage_[1] = (centilux >> 8) & 0xFF;
      this->node_threshold_storage_[2] = (centilux >> 16) & 0xFF;
      this->node_threshold_buffer_.data = this->node_threshold_storage_.data();
      this->node_threshold_buffer_.len = this->node_threshold_storage_.size();
      this->node_threshold_buffer_.size = this->node_threshold_storage_.size();
      this->node_threshold_buffer_.__buf = this->node_threshold_storage_.data();
      set.lc_property_set.property_id = LC_LIGHT_ON_THRESHOLD_PROPERTY;
      set.lc_property_set.property_value = &this->node_threshold_buffer_;
      return this->record_access_send_result_(AccessOperation::NODE_THRESHOLD_SET,
                                              ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_SET,
                                              esp_ble_mesh_light_client_set_state(&common, &set));
    }
    case NodeRequestKind::LC_MODE_SET: {
      // Unacknowledged: the lamp applies it
      // immediately and the follow-up LC Mode Get confirms the result. It does
      // not use the access slot.
      esp_ble_mesh_light_client_set_state_t set{};
      if (!this->set_common_(common, light_lc_model_(), ESP_BLE_MESH_MODEL_OP_LIGHT_LC_MODE_SET_UNACK,
                             destination))
        break;
      set.lc_mode_set.mode = request.value != 0 ? 1 : 0;
      this->node_inflight_valid_ = false;
      return this->record_send_result_(esp_ble_mesh_light_client_set_state(&common, &set));
    }
  }
  this->node_inflight_valid_ = false;
  return false;
}

void NightmatiqMesh::node_request_failed_() {
  // The IV-index and address-recovery checks count unanswered requests.
  this->mesh_timeouts_.fetch_add(1);
  if (!this->node_inflight_valid_ || this->node_inflight_.node >= this->node_runtime_.size()) return;
  std::lock_guard<std::mutex> lock(this->node_mutex_);
  this->node_runtime_[this->node_inflight_.node].consecutive_failures++;
}

// An authenticated response proves the keys and the IV index are usable and
// feeds the signal-strength entity. The per-model counters of the old poll are
// left alone: its retry logic compares them.
void NightmatiqMesh::note_node_response_(const esp_ble_mesh_msg_ctx_t &context) {
  this->mesh_rx_messages_.fetch_add(1);
  this->record_mesh_rssi_(context);
}

bool NightmatiqMesh::queue_node_command_(const NodeRequest &request) {
  if (this->node_command_count_ >= this->node_commands_.size()) return false;
  this->node_commands_[this->node_command_count_++] = request;
  return true;
}

bool NightmatiqMesh::submit_node_command_(uint8_t node_index, int on, int brightness_percent, int auto_mode,
                                          int threshold_lux, int run_time_seconds, std::string &error) {
  if (!this->node_table_valid_ || node_index >= this->node_table_.count) {
    error = "Unknown node";
    return false;
  }
  const StoredNode &node = this->node_table_.nodes[node_index];
  const int light = light_element(node.element_caps, node.element_count, CAP_LIGHTNESS, CAP_ONOFF);
  const int lc = first_element_with(node.element_caps, node.element_count, CAP_LC);
  const bool manual = on >= 0 || brightness_percent >= 0;
  if (!manual && auto_mode < 0 && threshold_lux < 0 && run_time_seconds < 0) {
    error = "Nothing to do; provide on, brightness, auto, threshold or run_time";
    return false;
  }
  if (manual && light < 0) {
    error = "This node has no controllable light output";
    return false;
  }
  if (auto_mode >= 0 && lc < 0) {
    error = "This node has no automatic (light control) mode";
    return false;
  }
  if (threshold_lux >= 0 && lc < 0) {
    error = "This node has no twilight threshold";
    return false;
  }
  if (run_time_seconds >= 0 && lc < 0) {
    error = "This node has no run time";
    return false;
  }
  if (auto_mode == 1 && manual) {
    error = "Automatic mode cannot be combined with a manual on/off or brightness value";
    return false;
  }

  std::vector<NodeRequest> list;
  if (threshold_lux >= 0)
    list.push_back({NodeRequestKind::THRESHOLD_SET, node_index, static_cast<uint8_t>(lc),
                    static_cast<uint16_t>(threshold_lux), false});
  if (run_time_seconds >= 0)
    list.push_back({NodeRequestKind::RUN_TIME_SET, node_index, static_cast<uint8_t>(lc),
                    static_cast<uint16_t>(run_time_seconds), false});
  if (auto_mode >= 0) {
    // Sent twice: unacknowledged messages can be lost, and the lamp ignores
    // the duplicate.
    for (int i = 0; i < 2; i++)
      list.push_back({NodeRequestKind::LC_MODE_SET, node_index, static_cast<uint8_t>(lc),
                      static_cast<uint16_t>(auto_mode), false});
  } else if (manual && lc >= 0) {
    // A manual command only sticks once automatic control is switched off.
    for (int i = 0; i < 2; i++)
      list.push_back({NodeRequestKind::LC_MODE_SET, node_index, static_cast<uint8_t>(lc), 0, false});
  }
  if (brightness_percent >= 0) {
    const uint16_t level = static_cast<uint16_t>(std::lround(brightness_percent * 65535.0 / 100.0));
    list.push_back({NodeRequestKind::LIGHTNESS_SET, node_index, static_cast<uint8_t>(light), level, false});
  } else if (on >= 0) {
    list.push_back({NodeRequestKind::ONOFF_SET, node_index, static_cast<uint8_t>(light),
                    static_cast<uint16_t>(on), false});
  }
  list.back().refresh_after = true;

  std::lock_guard<std::mutex> lock(this->node_mutex_);
  if (this->node_command_count_ + list.size() > this->node_commands_.size()) {
    error = "Command queue is full; try again shortly";
    return false;
  }
  for (const NodeRequest &request : list) this->queue_node_command_(request);
  return true;
}

void NightmatiqMesh::advance_node_engine_(uint32_t now) {
  if (!this->mesh_ready_.load()) return;

  if (!this->node_table_valid_) return;
  if (this->node_next_pass_at_ == 0) this->node_next_pass_at_ = now + NODE_FIRST_PASS_DELAY_MS;
  if (static_cast<int32_t>(now - this->node_next_action_at_) < 0) return;
  if (this->access_operation_.load() != AccessOperation::NONE || this->import_busy_.load() ||
      this->auto_update_running_.load() || this->reboot_pending_.load())
    return;
  this->node_inflight_valid_ = false;

  NodeRequest command{};
  bool have_command = false;
  {
    std::lock_guard<std::mutex> lock(this->node_mutex_);
    if (this->node_command_count_ > 0) {
      command = this->node_commands_[0];
      for (size_t i = 1; i < this->node_command_count_; i++) this->node_commands_[i - 1] = this->node_commands_[i];
      this->node_command_count_--;
      have_command = true;
    }
  }
  if (have_command) {
    const bool sent = this->send_node_request_(command);
    if (!sent) ESP_LOGW(NODE_TAG, "Node command %u for node %u was not sent", static_cast<unsigned>(command.kind),
                        static_cast<unsigned>(command.node));
    if (command.refresh_after) this->build_node_poll_plan_(command.node);
    this->node_next_action_at_ = now + NODE_COMMAND_GAP_MS;
    return;
  }

  if (this->node_poll_pos_ >= this->node_poll_plan_.size()) {
    this->node_poll_plan_.clear();
    this->node_poll_pos_ = 0;
    if (static_cast<int32_t>(now - this->node_next_pass_at_) < 0) return;
    this->build_node_poll_plan_();
    if (this->node_poll_plan_.empty()) {
      this->node_next_pass_at_ = now + NODE_PASS_INTERVAL_MS;
      return;
    }
  }
  const NodeRequest request = this->node_poll_plan_[this->node_poll_pos_++];
  this->send_node_request_(request);
  this->node_next_action_at_ = now + NODE_REQUEST_GAP_MS;
  if (this->node_poll_pos_ >= this->node_poll_plan_.size())
    this->node_next_pass_at_ = now + NODE_PASS_INTERVAL_MS;
}

void NightmatiqMesh::handle_node_generic_(esp_ble_mesh_generic_client_cb_event_t event,
                                          esp_ble_mesh_generic_client_cb_param_t *param) {
  const uint32_t opcode = param->params->opcode;
  if (event == ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT || param->error_code != 0 ||
      param->params->ctx.recv_op != ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS ||
      !this->node_inflight_valid_ || this->node_inflight_.node >= this->node_runtime_.size()) {
    this->node_request_failed_();
    this->complete_access_operation_(opcode, false);
    return;
  }
  this->note_node_response_(param->params->ctx);
  {
    std::lock_guard<std::mutex> lock(this->node_mutex_);
    NodeRuntime &state = this->node_runtime_[this->node_inflight_.node];
    state.onoff = param->status_cb.onoff_status.present_onoff != 0 ? 1 : 0;
    state.responded = true;
    state.last_response_at = millis();
    state.consecutive_failures = 0;
  }
  this->complete_access_operation_(opcode, true);
}

void NightmatiqMesh::handle_node_light_(esp_ble_mesh_light_client_cb_event_t event,
                                        esp_ble_mesh_light_client_cb_param_t *param) {
  const uint32_t opcode = param->params->opcode;
  const uint32_t received = param->params->ctx.recv_op;
  const bool lightness = received == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_STATUS;
  const bool lc_mode = received == ESP_BLE_MESH_MODEL_OP_LIGHT_LC_MODE_STATUS;
  const bool property = received == ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_STATUS;
  if (this->node_inflight_valid_ && this->node_inflight_.node < this->node_runtime_.size() &&
      (this->node_inflight_.kind == NodeRequestKind::RUN_TIME_GET ||
       this->node_inflight_.kind == NodeRequestKind::RUN_TIME_SET)) {
    const bool is_read = this->node_inflight_.kind == NodeRequestKind::RUN_TIME_GET;
    const auto &status = param->status_cb.lc_property_status;
    const bool valid = event != ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT && param->error_code == 0 && property &&
                       status.property_id == LC_TIME_RUN_ON_PROPERTY && status.property_value != nullptr &&
                       status.property_value->len >= 3;
    uint32_t milliseconds = 0;
    if (valid) {
      const uint8_t *value = status.property_value->data;
      milliseconds = static_cast<uint32_t>(value[0]) | (static_cast<uint32_t>(value[1]) << 8) |
                     (static_cast<uint32_t>(value[2]) << 16);
    }
    if (!valid || milliseconds == 0xFFFFFF) {
      // A read that goes unanswered is not held against the device; a lost
      // write is.
      if (!is_read) this->node_request_failed_();
      this->complete_access_operation_(opcode, false);
      return;
    }
    this->note_node_response_(param->params->ctx);
    {
      std::lock_guard<std::mutex> lock(this->node_mutex_);
      NodeRuntime &state = this->node_runtime_[this->node_inflight_.node];
      if (state.run_time_ms < 0)
        ESP_LOGI(NODE_TAG, "Node '%s': run time %u s", this->node_table_.nodes[this->node_inflight_.node].name,
                 static_cast<unsigned>(milliseconds / 1000U));
      state.run_time_ms = static_cast<int32_t>(milliseconds);
      state.responded = true;
      state.last_response_at = millis();
      state.consecutive_failures = 0;
    }
    this->complete_access_operation_(opcode, true);
    return;
  }
  if (event == ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT || param->error_code != 0 || (!lightness && !lc_mode && !property) ||
      !this->node_inflight_valid_ || this->node_inflight_.node >= this->node_runtime_.size()) {
    this->node_request_failed_();
    this->complete_access_operation_(opcode, false);
    return;
  }
  int32_t threshold_centilux = -1;
  if (property) {
    const auto &status = param->status_cb.lc_property_status;
    if (status.property_id == LC_LIGHT_ON_THRESHOLD_PROPERTY && status.property_value != nullptr &&
        status.property_value->len >= 3) {
      const uint8_t *value = status.property_value->data;
      const uint32_t parsed = static_cast<uint32_t>(value[0]) | (static_cast<uint32_t>(value[1]) << 8) |
                              (static_cast<uint32_t>(value[2]) << 16);
      // Anything outside the range the API accepts is not trusted.
      if (parsed >= THRESHOLD_MIN_CENTILUX && parsed <= THRESHOLD_MAX_CENTILUX)
        threshold_centilux = static_cast<int32_t>(parsed);
    }
    if (threshold_centilux < 0) {
      this->node_request_failed_();
      this->complete_access_operation_(opcode, false);
      return;
    }
  }
  this->note_node_response_(param->params->ctx);
  {
    std::lock_guard<std::mutex> lock(this->node_mutex_);
    NodeRuntime &state = this->node_runtime_[this->node_inflight_.node];
    if (lightness)
      state.lightness = param->status_cb.lightness_status.present_lightness;
    else if (property)
      state.threshold_centilux = threshold_centilux;
    else
      state.lc_mode = param->status_cb.lc_mode_status.mode != 0 ? 1 : 0;
    state.responded = true;
    state.last_response_at = millis();
    state.consecutive_failures = 0;
  }
  this->complete_access_operation_(opcode, true);
}

void NightmatiqMesh::handle_node_composition_(esp_ble_mesh_cfg_client_cb_event_t event,
                                              esp_ble_mesh_cfg_client_cb_param_t *param) {
  const uint32_t opcode = param->params->opcode;
  const bool replied = event == ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT && param->error_code == 0;
  const net_buf_simple *data = replied ? param->status_cb.comp_data_status.composition_data : nullptr;
  if (data == nullptr || param->status_cb.comp_data_status.page != 0 || data->len < 10 ||
      !this->node_inflight_valid_ || this->node_inflight_.node >= this->node_runtime_.size()) {
    this->node_request_failed_();
    this->complete_access_operation_(opcode, false);
    return;
  }
  const auto read_le16 = [](const uint8_t *value) -> uint16_t {
    return static_cast<uint16_t>(value[0]) | (static_cast<uint16_t>(value[1]) << 8);
  };
  const uint16_t company_id = read_le16(data->data);
  const uint16_t product_id = read_le16(data->data + 2);
  const uint16_t version_id = read_le16(data->data + 4);
  const StoredNode &node = this->node_table_.nodes[this->node_inflight_.node];
  if (company_id != node.company_id || product_id != node.product_id) {
    // Not the device we asked: do not attribute its version to this node.
    ESP_LOGW(NODE_TAG, "Composition reply for 0x%04X has product 0x%04X, expected 0x%04X", node.address,
             product_id, node.product_id);
    this->node_request_failed_();
    this->complete_access_operation_(opcode, false);
    return;
  }
  this->note_node_response_(param->params->ctx);
  {
    std::lock_guard<std::mutex> lock(this->node_mutex_);
    NodeRuntime &state = this->node_runtime_[this->node_inflight_.node];
    state.version_id = version_id;
    state.responded = true;
    state.last_response_at = millis();
    state.consecutive_failures = 0;
  }
  ESP_LOGI(NODE_TAG, "Node '%s' (0x%04X): version ID 0x%04X = firmware %u.%u.%u", node.name, node.address,
           version_id, static_cast<unsigned>(version_id >> 11), static_cast<unsigned>((version_id >> 6) & 0x1F),
           static_cast<unsigned>(version_id & 0x3F));
  this->complete_access_operation_(opcode, true);
}

void NightmatiqMesh::store_sensor_data_(uint8_t node, uint8_t element, const uint8_t *data, size_t length) {
  if (node >= this->node_runtime_.size()) return;
  {
    std::lock_guard<std::mutex> lock(this->node_mutex_);
    NodeRuntime &state = this->node_runtime_[node];
    const uint8_t *cursor = data;
    size_t remaining = length;
    while (remaining > 0) {
      const uint8_t format = ESP_BLE_MESH_GET_SENSOR_DATA_FORMAT(cursor);
      const size_t mpid_length = format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A
                                     ? ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN
                                     : ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
      if (remaining < mpid_length) break;
      const uint8_t encoded_length = ESP_BLE_MESH_GET_SENSOR_DATA_LENGTH(cursor, format);
      const uint16_t property_id = ESP_BLE_MESH_GET_SENSOR_DATA_PROPERTY_ID(cursor, format);
      // A zero-length marker means the property exists but has no value.
      const size_t value_length =
          encoded_length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN ? 0 : static_cast<size_t>(encoded_length) + 1;
      if (remaining < mpid_length + value_length) break;

      NodeSensorValue *slot = nullptr;
      for (uint8_t i = 0; i < state.sensor_count; i++)
        if (state.sensors[i].element == element && state.sensors[i].property == property_id) {
          slot = &state.sensors[i];
          break;
        }
      if (slot == nullptr && state.sensor_count < state.sensors.size()) slot = &state.sensors[state.sensor_count++];
      if (slot != nullptr) {
        if (slot->length == 0 && slot->element == 0 && slot->property == 0)
          ESP_LOGI(NODE_TAG, "Node '%s' element %u: first reading of sensor property 0x%04X (%u bytes)",
                   this->node_table_.nodes[node].name, static_cast<unsigned>(element),
                   static_cast<unsigned>(property_id), static_cast<unsigned>(value_length));
        slot->element = element;
        slot->property = property_id;
        slot->length = static_cast<uint8_t>(std::min(value_length, slot->raw.size()));
        std::memcpy(slot->raw.data(), cursor + mpid_length, slot->length);
      }
      cursor += mpid_length + value_length;
      remaining -= mpid_length + value_length;
    }
  }
}

void NightmatiqMesh::handle_node_sensor_publish_(esp_ble_mesh_sensor_client_cb_param_t *param) {
  const esp_ble_mesh_msg_ctx_t &context = param->params->ctx;
  if (context.recv_op != ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS) return;
  const net_buf_simple *buffer = param->status_cb.sensor_status.marshalled_sensor_data;
  uint8_t node = 0;
  if (buffer == nullptr || buffer->len == 0 || !this->find_node_index_(context.addr, node)) return;
  const uint8_t element = static_cast<uint8_t>(context.addr - this->node_table_.nodes[node].address);
  this->note_node_response_(context);
  this->store_sensor_data_(node, element, buffer->data, buffer->len);
  std::lock_guard<std::mutex> lock(this->node_mutex_);
  NodeRuntime &state = this->node_runtime_[node];
  state.responded = true;
  state.last_response_at = millis();
  state.consecutive_failures = 0;
}

void NightmatiqMesh::add_sensor_group_(StoredSensorGroups &groups, uint16_t address) {
  // Only group addresses (0xC000-0xFEFF) can be subscribed to.
  if (address < 0xC000 || address > 0xFEFF) return;
  for (uint16_t i = 0; i < groups.count; i++)
    if (groups.groups[i] == address) return;
  if (groups.count < groups.groups.size()) groups.groups[groups.count++] = address;
}

bool NightmatiqMesh::load_sensor_groups_() {
  StoredSensorGroups loaded{};
  if (!this->sensor_groups_preference_.load(&loaded) || loaded.magic != SENSOR_GROUPS_MAGIC ||
      loaded.version != SENSOR_GROUPS_VERSION || loaded.count > loaded.groups.size()) {
    this->sensor_groups_ = StoredSensorGroups{};
    return false;
  }
  this->sensor_groups_ = loaded;
  return true;
}

bool NightmatiqMesh::save_sensor_groups_(const StoredSensorGroups &groups) {
  if (!this->sensor_groups_preference_.save(&groups)) return false;
  this->sensor_groups_ = groups;
  return true;
}

void NightmatiqMesh::subscribe_sensor_groups_() {
  // The sensors publish to these groups on their own. Subscribing the local
  // Sensor client lets the gateway receive those readings without asking.
  if (this->sensor_groups_.count == 0) {
    ESP_LOGI(NODE_TAG, "No sensor groups stored; import the backup again to receive sensor readings without polling");
    return;
  }
  for (uint16_t i = 0; i < this->sensor_groups_.count; i++) {
    const uint16_t group = this->sensor_groups_.groups[i];
    const esp_err_t error = esp_ble_mesh_model_subscribe_group_addr(
        this->config_.local_address, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_SENSOR_CLI, group);
    if (error == ESP_OK)
      ESP_LOGI(NODE_TAG, "Subscribed to sensor group 0x%04X", group);
    else
      ESP_LOGW(NODE_TAG, "Could not subscribe to sensor group 0x%04X: %s", group, esp_err_to_name(error));
  }
}

void NightmatiqMesh::handle_node_sensor_(esp_ble_mesh_sensor_client_cb_event_t event,
                                         esp_ble_mesh_sensor_client_cb_param_t *param) {
  const uint32_t opcode = param->params->opcode;
  const bool descriptor_reply = event != ESP_BLE_MESH_SENSOR_CLIENT_TIMEOUT_EVT && param->error_code == 0 &&
                                param->params->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS;
  if (descriptor_reply && this->node_inflight_valid_ && this->node_inflight_.node < this->node_runtime_.size()) {
    // Lists the sensor properties an element offers, 8 bytes per sensor.
    const net_buf_simple *list = param->status_cb.descriptor_status.descriptor;
    const StoredNode &described = this->node_table_.nodes[this->node_inflight_.node];
    this->note_node_response_(param->params->ctx);
    if (list == nullptr || list->len < 8) {
      ESP_LOGW(NODE_TAG, "Node '%s' element %u: sensor descriptor reply is empty", described.name,
               static_cast<unsigned>(this->node_inflight_.element));
    } else {
      for (size_t offset = 0; offset + 8 <= list->len; offset += 8)
        ESP_LOGI(NODE_TAG, "Node '%s' element %u offers sensor property 0x%04X", described.name,
                 static_cast<unsigned>(this->node_inflight_.element),
                 static_cast<unsigned>(list->data[offset] | (list->data[offset + 1] << 8)));
    }
    this->complete_access_operation_(opcode, true);
    return;
  }
  net_buf_simple *buffer =
      event == ESP_BLE_MESH_SENSOR_CLIENT_TIMEOUT_EVT || param->error_code != 0 ||
              param->params->ctx.recv_op != ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS
          ? nullptr
          : param->status_cb.sensor_status.marshalled_sensor_data;
  if (buffer == nullptr || !this->node_inflight_valid_ || this->node_inflight_.node >= this->node_runtime_.size()) {
    if (this->node_inflight_valid_ && this->node_inflight_.node < this->node_table_.count)
      ESP_LOGW(NODE_TAG, "Sensor request to '%s' element %u (kind %u) failed: event=%d error=%d recv_op=0x%X",
               this->node_table_.nodes[this->node_inflight_.node].name,
               static_cast<unsigned>(this->node_inflight_.element), static_cast<unsigned>(this->node_inflight_.kind),
               static_cast<int>(event), static_cast<int>(param->error_code),
               static_cast<unsigned>(param->params->ctx.recv_op));
    this->node_request_failed_();
    this->complete_access_operation_(opcode, false);
    return;
  }
  if (buffer->len == 0)
    ESP_LOGW(NODE_TAG, "Sensor status from '%s' element %u carries no sensor data",
             this->node_table_.nodes[this->node_inflight_.node].name,
             static_cast<unsigned>(this->node_inflight_.element));

  this->note_node_response_(param->params->ctx);
  this->store_sensor_data_(this->node_inflight_.node, this->node_inflight_.element, buffer->data, buffer->len);
  {
    std::lock_guard<std::mutex> lock(this->node_mutex_);
    NodeRuntime &state = this->node_runtime_[this->node_inflight_.node];
    state.responded = true;
    state.last_response_at = millis();
    state.consecutive_failures = 0;
  }
  this->complete_access_operation_(opcode, true);
}

// POST /api/nodes/<address>?on=1&brightness=40&auto=0
void NightmatiqMesh::handle_api_node_(AsyncWebServerRequest *request) {
  char url_buffer[AsyncWebServerRequest::URL_BUF_SIZE];
  const StringRef url = request->url_to(url_buffer);
  static constexpr char PREFIX[] = "/api/nodes/";
  std::string address_text(url.c_str() + sizeof(PREFIX) - 1);
  uint16_t address = 0;
  uint8_t index = 0;
  if (address_text.empty() || !parse_hex_u16_(address_text, address) || !this->find_node_index_(address, index) ||
      this->node_table_.nodes[index].address != address)
    return send_json_(request, 404, "{\"message\":\"Unknown node address\"}");
  if (!this->mesh_ready_.load())
    return send_json_(request, 409, "{\"message\":\"Bluetooth Mesh is not ready\"}");

  int on = -1;
  int auto_mode = -1;
  int brightness = -1;
  int threshold = -1;
  int run_time = -1;
  const std::string on_arg = request->arg("on");
  const std::string auto_arg = request->arg("auto");
  const std::string brightness_arg = request->arg("brightness");
  const std::string threshold_arg = request->arg("threshold");
  const std::string run_time_arg = request->arg("run_time");
  if (!on_arg.empty() && !parse_switch(on_arg, on))
    return send_json_(request, 400, "{\"message\":\"on must be 0/1, true/false or on/off\"}");
  if (!auto_arg.empty() && !parse_switch(auto_arg, auto_mode))
    return send_json_(request, 400, "{\"message\":\"auto must be 0/1, true/false or on/off\"}");
  if (!brightness_arg.empty()) {
    uint32_t parsed = 0;
    if (!parse_u32_(brightness_arg, 0, 100, parsed))
      return send_json_(request, 400, "{\"message\":\"brightness must be 0-100\"}");
    brightness = static_cast<int>(parsed);
  }
  if (!threshold_arg.empty()) {
    uint32_t parsed = 0;
    if (!parse_u32_(threshold_arg, 1, 1500, parsed))
      return send_json_(request, 400, "{\"message\":\"threshold must be 1-1500 (lux)\"}");
    threshold = static_cast<int>(parsed);
  }
  if (!run_time_arg.empty()) {
    uint32_t parsed = 0;
    if (!parse_u32_(run_time_arg, 5, 3600, parsed))
      return send_json_(request, 400, "{\"message\":\"run_time must be 5-3600 (seconds)\"}");
    run_time = static_cast<int>(parsed);
  }

  std::string error;
  if (!this->submit_node_command_(index, on, brightness, auto_mode, threshold, run_time, error)) {
    std::string escaped;
    for (char c : error)
      if (c != '"' && c != '\\' && static_cast<unsigned char>(c) >= 0x20) escaped += c;
    return send_json_(request, error.find("queue") != std::string::npos ? 409 : 400,
                      "{\"message\":\"" + escaped + "\"}");
  }
  // ESPHome's web server can only send 200, 204, 400, 401, 404, 409 and 422;
  // any other code, such as 202, is turned into a 500.
  send_json_(request, 200, "{\"message\":\"Command queued\"}");
}

}  // namespace steinel_mesh
}  // namespace esphome

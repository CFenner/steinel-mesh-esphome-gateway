// Generic multi-node engine.
//
// The NightmatIQ logic in nightmatiq_mesh.cpp drives exactly one node. This
// file adds a small scheduler that reads and controls every node stored from the
// imported backup (lamps, sensors) through standard SIG models. It shares the
// single acknowledged-access slot with the NightmatIQ code, so at most one
// request is ever in flight and neither side can disturb the other.
#include "nightmatiq_mesh.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"

namespace esphome {
namespace nightmatiq_mesh {

static const char *const NODE_TAG = "nightmatiq_nodes";
// Idle time after each request. It keeps the radio free for the NightmatIQ
// control path and for Composition Data reads.
static constexpr uint32_t NODE_REQUEST_GAP_MS = 350;
static constexpr uint32_t NODE_COMMAND_GAP_MS = 150;
static constexpr uint32_t NODE_PASS_INTERVAL_MS = 20000;
// The first pass waits so the NightmatIQ identity and initial poll go first.
static constexpr uint32_t NODE_FIRST_PASS_DELAY_MS = 8000;
// Light Control "ambient lux on" property, the twilight threshold. Values are
// 24-bit illuminance in 0.01 lx; the API accepts whole lux from 1 to 1500.
static constexpr uint16_t LC_LIGHT_ON_THRESHOLD_PROPERTY = 0x002B;
static constexpr uint32_t THRESHOLD_MIN_CENTILUX = 100;
static constexpr uint32_t THRESHOLD_MAX_CENTILUX = 150000;

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
  const auto add_node = [this](uint8_t index, std::vector<NodeRequest> &plan) {
    const StoredNode &node = this->node_table_.nodes[index];
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
  };

  if (only_node >= 0) {
    // A refresh after a command runs before the remainder of the current pass.
    std::vector<NodeRequest> extra;
    add_node(static_cast<uint8_t>(only_node), extra);
    this->node_poll_plan_.insert(this->node_poll_plan_.begin() + this->node_poll_pos_, extra.begin(),
                                 extra.end());
    return;
  }
  this->node_poll_plan_.clear();
  this->node_poll_pos_ = 0;
  for (uint16_t i = 0; i < this->node_table_.count && i < this->node_table_.nodes.size(); i++)
    add_node(static_cast<uint8_t>(i), this->node_poll_plan_);
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
      // Unacknowledged, like the NightmatIQ mode change: the lamp applies it
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
  if (!this->node_inflight_valid_ || this->node_inflight_.node >= this->node_runtime_.size()) return;
  std::lock_guard<std::mutex> lock(this->node_mutex_);
  this->node_runtime_[this->node_inflight_.node].consecutive_failures++;
}

bool NightmatiqMesh::queue_node_command_(const NodeRequest &request) {
  if (this->node_command_count_ >= this->node_commands_.size()) return false;
  this->node_commands_[this->node_command_count_++] = request;
  return true;
}

bool NightmatiqMesh::submit_node_command_(uint8_t node_index, int on, int brightness_percent, int auto_mode,
                                          int threshold_lux, std::string &error) {
  if (!this->node_table_valid_ || node_index >= this->node_table_.count) {
    error = "Unknown node";
    return false;
  }
  const StoredNode &node = this->node_table_.nodes[node_index];
  const int light = light_element(node.element_caps, node.element_count, CAP_LIGHTNESS, CAP_ONOFF);
  const int lc = first_element_with(node.element_caps, node.element_count, CAP_LC);
  const bool manual = on >= 0 || brightness_percent >= 0;
  if (!manual && auto_mode < 0 && threshold_lux < 0) {
    error = "Nothing to do; provide on, brightness, auto or threshold";
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
  if (auto_mode == 1 && manual) {
    error = "Automatic mode cannot be combined with a manual on/off or brightness value";
    return false;
  }

  std::vector<NodeRequest> list;
  if (threshold_lux >= 0)
    list.push_back({NodeRequestKind::THRESHOLD_SET, node_index, static_cast<uint8_t>(lc),
                    static_cast<uint16_t>(threshold_lux), false});
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

  // The periodic NightmatIQ poll could not start because a node request held
  // the access slot. Start it as soon as the slot is free, ahead of node work.
  if (this->nightmatiq_poll_deferred_) {
    if (this->access_operation_.load() == AccessOperation::NONE && this->poll_stage_ == 0 &&
        this->control_kind_ == ControlKind::NONE && !this->control_request_pending_()) {
      this->nightmatiq_poll_deferred_ = false;
      this->poll_sensor_rx_start_ = this->mesh_sensor_rx_.load();
      this->poll_generic_rx_start_ = this->mesh_generic_rx_.load();
      this->poll_stage_ = 1;
      this->poll_stage_at_ = now;
    }
    return;
  }

  if (!this->node_table_valid_) return;
  if (this->node_next_pass_at_ == 0) this->node_next_pass_at_ = now + NODE_FIRST_PASS_DELAY_MS;
  if (static_cast<int32_t>(now - this->node_next_action_at_) < 0) return;
  if (this->access_operation_.load() != AccessOperation::NONE || this->poll_stage_ != 0 ||
      this->control_kind_ != ControlKind::NONE || this->control_request_pending_() ||
      this->composition_query_in_flight_.load() || this->import_busy_.load() ||
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

void NightmatiqMesh::handle_node_sensor_(esp_ble_mesh_sensor_client_cb_event_t event,
                                         esp_ble_mesh_sensor_client_cb_param_t *param) {
  const uint32_t opcode = param->params->opcode;
  net_buf_simple *buffer =
      event == ESP_BLE_MESH_SENSOR_CLIENT_TIMEOUT_EVT || param->error_code != 0 ||
              param->params->ctx.recv_op != ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS
          ? nullptr
          : param->status_cb.sensor_status.marshalled_sensor_data;
  if (buffer == nullptr || !this->node_inflight_valid_ || this->node_inflight_.node >= this->node_runtime_.size()) {
    this->node_request_failed_();
    this->complete_access_operation_(opcode, false);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->node_mutex_);
    NodeRuntime &state = this->node_runtime_[this->node_inflight_.node];
    const uint8_t element = this->node_inflight_.element;
    const uint8_t *data = buffer->data;
    size_t remaining = buffer->len;
    while (remaining > 0) {
      const uint8_t format = ESP_BLE_MESH_GET_SENSOR_DATA_FORMAT(data);
      const size_t mpid_length = format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A
                                     ? ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN
                                     : ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
      if (remaining < mpid_length) break;
      const uint8_t encoded_length = ESP_BLE_MESH_GET_SENSOR_DATA_LENGTH(data, format);
      const uint16_t property_id = ESP_BLE_MESH_GET_SENSOR_DATA_PROPERTY_ID(data, format);
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
        slot->element = element;
        slot->property = property_id;
        slot->length = static_cast<uint8_t>(std::min(value_length, slot->raw.size()));
        std::memcpy(slot->raw.data(), data + mpid_length, slot->length);
      }
      data += mpid_length + value_length;
      remaining -= mpid_length + value_length;
    }
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
  const std::string on_arg = request->arg("on");
  const std::string auto_arg = request->arg("auto");
  const std::string brightness_arg = request->arg("brightness");
  const std::string threshold_arg = request->arg("threshold");
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

  std::string error;
  if (!this->submit_node_command_(index, on, brightness, auto_mode, threshold, error)) {
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

}  // namespace nightmatiq_mesh
}  // namespace esphome

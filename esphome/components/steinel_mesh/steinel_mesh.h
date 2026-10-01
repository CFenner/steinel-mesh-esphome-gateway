#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/esp32_ble/ble.h"
#include "esphome/components/esphome/ota/ota_esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "esp_ble_mesh_time_scene_model_api.h"
#include "esp_http_client.h"

namespace esphome {
namespace steinel_mesh {

class NightmatiqMesh final : public Component, public AsyncWebHandler {
 public:
  NightmatiqMesh(web_server_base::WebServerBase *base, ESPHomeOTAComponent *ota)
      : base_(base), ota_(ota) {}

  void set_rssi_sensor(sensor::Sensor *value) { this->rssi_sensor_ = value; }
  void set_ready_binary_sensor(binary_sensor::BinarySensor *value) { this->ready_binary_sensor_ = value; }
  void set_status_text_sensor(text_sensor::TextSensor *value) { this->status_text_sensor_ = value; }
  void set_web_credentials(const std::string &username, const std::string &password) {
    this->web_username_ = username;
    this->web_password_ = password;
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  void handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index,
                  size_t total) override;
  bool isRequestHandlerTrivial() const override { return false; }

  void request_refresh();
  bool mesh_mode_enabled() const { return this->mesh_mode_enabled_; }
  // Traffic counters, published as diagnostic entities.
  uint32_t mesh_tx_attempts() const { return this->mesh_tx_attempts_.load(); }
  uint32_t mesh_tx_errors() const { return this->mesh_tx_errors_.load(); }
  int32_t mesh_last_tx_error() const { return this->mesh_last_tx_error_.load(); }
  uint32_t mesh_rx_messages() const { return this->mesh_rx_messages_.load(); }
  uint32_t mesh_timeouts() const { return this->mesh_timeouts_.load(); }

  static void provisioning_callback(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param);
  static void config_callback(esp_ble_mesh_cfg_client_cb_event_t event,
                              esp_ble_mesh_cfg_client_cb_param_t *param);
  static void generic_callback(esp_ble_mesh_generic_client_cb_event_t event,
                               esp_ble_mesh_generic_client_cb_param_t *param);
  static void sensor_callback(esp_ble_mesh_sensor_client_cb_event_t event,
                              esp_ble_mesh_sensor_client_cb_param_t *param);
  static void light_callback(esp_ble_mesh_light_client_cb_event_t event,
                             esp_ble_mesh_light_client_cb_param_t *param);
  static void scene_callback(esp_ble_mesh_time_scene_client_cb_event_t event,
                             esp_ble_mesh_time_scene_client_cb_param_t *param);
 protected:
  static constexpr uint32_t CONFIG_MAGIC = 0x4E4D5131U;  // "NMQ1"
  static constexpr uint16_t CONFIG_VERSION = 2;
  static constexpr uint32_t DEVICE_KEY_MAGIC = 0x4E4D514BU;  // "NMQK"
  static constexpr uint16_t DEVICE_KEY_VERSION = 1;
  static constexpr uint32_t RETIRED_ADDRESS_MAGIC = 0x4E4D5141U;  // "NMQA"
  static constexpr uint16_t RETIRED_ADDRESS_VERSION = 1;
  static constexpr uint32_t ADDRESS_POLICY_MAGIC = 0x4E4D5150U;  // "NMQP"
  static constexpr uint16_t ADDRESS_POLICY_VERSION = 1;
  static constexpr uint16_t ADDRESS_POOL_TARGET_SIZE = 2048;
  static constexpr uint16_t AUTO_ADDRESS_ROTATION_LIMIT = 16;
  static constexpr uint32_t AUTO_ADDRESS_RECOVERY_DELAY_MS = 60000;
  static constexpr uint32_t AUTO_ADDRESS_MIN_ACCEPTED_TX = 10;
  static constexpr uint32_t AUTO_ADDRESS_MIN_TIMEOUTS = 10;
  static constexpr uint32_t ADDRESS_CONFIRMATION_MAGIC = 0x4E4D5143U;  // "NMQC"
  static constexpr uint16_t ADDRESS_CONFIRMATION_VERSION = 1;
  static constexpr uint32_t IV_CACHE_MAGIC = 0x4E4D5149U;  // "NMQI"
  static constexpr uint16_t IV_CACHE_VERSION = 1;
  static constexpr uint32_t ADVERTISED_IDENTITY_MAGIC = 0x4E4D5156U;  // "NMQV"
  static constexpr uint16_t ADVERTISED_IDENTITY_VERSION = 2;
  static constexpr uint32_t ADMIN_CREDENTIALS_MAGIC = 0x4E4D5157U;  // "NMQW"
  static constexpr uint16_t ADMIN_CREDENTIALS_VERSION = 1;
  static constexpr size_t ADMIN_PASSWORD_MIN_LENGTH = 8;
  static constexpr size_t ADMIN_PASSWORD_MAX_LENGTH = 63;
  static constexpr uint32_t NODE_TABLE_MAGIC = 0x4E4D514EU;  // "NMQN"
  static constexpr uint16_t NODE_TABLE_VERSION = 1;
  static constexpr size_t NODE_TABLE_MAX_NODES = 12;
  static constexpr size_t NODE_MAX_ELEMENTS = 6;
  static constexpr size_t NODE_NAME_MAX_LENGTH = 31;
  static constexpr uint32_t SENSOR_GROUPS_MAGIC = 0x4E4D510BU;  // "NMQ\v"
  static constexpr uint16_t SENSOR_GROUPS_VERSION = 1;
  static constexpr size_t SENSOR_GROUPS_MAX = 6;
  static constexpr uint32_t AUTO_UPDATE_MAGIC = 0x4E4D5155U;  // "NMQU"
  static constexpr uint16_t AUTO_UPDATE_VERSION = 1;
  static constexpr size_t AUTO_UPDATE_VERSION_MAX_LENGTH = 23;
  static constexpr size_t AUTO_UPDATE_URL_MAX_LENGTH = 255;
  static constexpr uint16_t STEINEL_COMPANY_ID = 0x0563;
  static constexpr uint16_t NIGHTMATIQ_PRODUCT_ID = 0x1DCE;
  static constexpr uint16_t FLAG_ENABLED = 0x0001;
  static constexpr uint16_t FLAG_REMOVE_PENDING = 0x0002;
  // Set only after a Secure Network Beacon changes the IV Index or an
  // authenticated Access response proves that the stored value is usable.
  static constexpr uint16_t FLAG_IV_INDEX_CONFIRMED = 0x0004;
  // A mode SET is intentionally unacknowledged for immediate lamp control.
  // Confirm its physical result several times before returning to the normal
  // Retry within the 30-second polling interval after an unanswered GET.

  struct StoredConfig {
    uint32_t magic{CONFIG_MAGIC};
    uint16_t version{CONFIG_VERSION};
    uint16_t net_key_index{0};
    uint16_t app_key_index{0};
    uint16_t local_address{0};
    uint16_t onoff_address{0};
    uint16_t lc_address{0};
    uint16_t sensor_address{0};
    uint16_t scene_number{0};
    uint16_t flags{0};
    uint32_t iv_index{0};
    std::array<uint8_t, 16> net_key{};
    std::array<uint8_t, 16> app_key{};
    std::array<uint8_t, 16> mesh_uuid{};
    char network_name[48]{};
    char node_name[48]{};
  };

  struct StoredDeviceKey {
    uint32_t magic{DEVICE_KEY_MAGIC};
    uint16_t version{DEVICE_KEY_VERSION};
    std::array<uint8_t, 16> key{};
  };

  struct StoredRetiredAddress {
    uint32_t magic{RETIRED_ADDRESS_MAGIC};
    uint16_t version{RETIRED_ADDRESS_VERSION};
    uint16_t address{0};
  };

  struct StoredAddressPolicy {
    uint32_t magic{ADDRESS_POLICY_MAGIC};
    uint16_t version{ADDRESS_POLICY_VERSION};
    uint16_t pool_low{0};
    uint16_t pool_high{0};
    uint16_t initial_address{0};
    uint16_t current_address{0};
    uint16_t automatic_rotations{0};
    // Retained to preserve the version-1 NVS record layout. Manual rotation
    // is no longer exposed by the standalone gateway.
    uint16_t reserved{0};
    uint32_t installation_nonce{0};
    std::array<uint8_t, 16> mesh_uuid{};
  };

  struct StoredAddressConfirmation {
    uint32_t magic{ADDRESS_CONFIRMATION_MAGIC};
    uint16_t version{ADDRESS_CONFIRMATION_VERSION};
    uint16_t address{0};
    uint32_t installation_nonce{0};
    std::array<uint8_t, 16> mesh_uuid{};
  };

  struct StoredIvCache {
    uint32_t magic{IV_CACHE_MAGIC};
    uint16_t version{IV_CACHE_VERSION};
    uint16_t reserved{0};
    uint32_t iv_index{0};
    std::array<uint8_t, 16> mesh_uuid{};
  };

  struct StoredAdminCredentials {
    uint32_t magic{ADMIN_CREDENTIALS_MAGIC};
    uint16_t version{ADMIN_CREDENTIALS_VERSION};
    char password[ADMIN_PASSWORD_MAX_LENGTH + 1]{};
  };

  struct StoredAutoUpdate {
    uint32_t magic{AUTO_UPDATE_MAGIC};
    uint16_t version{AUTO_UPDATE_VERSION};
    uint16_t reserved{0};
    uint32_t image_size{0};
    char target_version[AUTO_UPDATE_VERSION_MAX_LENGTH + 1]{};
    char url[AUTO_UPDATE_URL_MAX_LENGTH + 1]{};
    std::array<uint8_t, 32> sha256{};
  };

  // Bit flags describing which Bluetooth Mesh SIG server models an element
  // exposes. They are derived from the backup and are what a generic client
  // needs to decide how a node can be controlled.
  static constexpr uint16_t CAP_ONOFF = 0x0001;      // Generic OnOff Server 0x1000
  static constexpr uint16_t CAP_LEVEL = 0x0002;      // Generic Level Server 0x1002
  static constexpr uint16_t CAP_LIGHTNESS = 0x0004;  // Light Lightness Server 0x1300
  static constexpr uint16_t CAP_LC = 0x0008;         // Light LC Server 0x130F
  static constexpr uint16_t CAP_SENSOR = 0x0010;     // Sensor Server 0x1100
  static constexpr uint16_t CAP_SCENE = 0x0020;      // Scene Server 0x1203
  static constexpr uint16_t CAP_SCHEDULER = 0x0040;  // Scheduler Server 0x1206

  struct StoredNode {
    uint16_t address{0};
    uint16_t company_id{0};
    uint16_t product_id{0};
    uint16_t bound_app_key{0};
    uint8_t element_count{0};
    uint8_t reserved{0};
    std::array<uint16_t, NODE_MAX_ELEMENTS> element_caps{};
    std::array<uint8_t, 16> device_key{};
    char name[NODE_NAME_MAX_LENGTH + 1]{};
  };

  struct StoredNodeTable {
    uint32_t magic{NODE_TABLE_MAGIC};
    uint16_t version{NODE_TABLE_VERSION};
    uint16_t count{0};
    std::array<uint8_t, 16> mesh_uuid{};
    std::array<StoredNode, NODE_TABLE_MAX_NODES> nodes{};
  };

  // Group addresses the sensors in the backup publish their readings to. The
  // gateway subscribes to them and receives those readings without asking.
  struct StoredSensorGroups {
    uint32_t magic{SENSOR_GROUPS_MAGIC};
    uint16_t version{SENSOR_GROUPS_VERSION};
    uint16_t count{0};
    std::array<uint16_t, SENSOR_GROUPS_MAX> groups{};
  };

  enum class AccessOperation : uint8_t {
    NONE,
    // Requests issued by the node engine. They share the single access slot, so
    // only one request is ever active.
    NODE_ONOFF_GET,
    NODE_LIGHTNESS_GET,
    NODE_LC_MODE_GET,
    NODE_SENSOR_GET,
    NODE_ONOFF_SET,
    NODE_LIGHTNESS_SET,
    NODE_THRESHOLD_GET,
    NODE_THRESHOLD_SET,
    NODE_COMPOSITION_GET,
  };
  static bool is_node_operation_(AccessOperation operation) {
    return operation >= AccessOperation::NODE_ONOFF_GET;
  }

  // Requests the generic engine can issue to one element of one stored node.
  enum class NodeRequestKind : uint8_t {
    ONOFF_GET,
    LIGHTNESS_GET,
    LC_MODE_GET,
    SENSOR_GET,
    ONOFF_SET,
    LIGHTNESS_SET,
    LC_MODE_SET,
    THRESHOLD_GET,
    THRESHOLD_SET,
    // Configuration Composition Data Get (page 0), read with the node's device key.
    // Its version ID is the device's firmware version.
    COMPOSITION_GET,
    // Sensor Get for one property id (value), and Sensor Descriptor Get. Used on
    // sensor elements that never produced a reading with the plain Sensor Get.
    SENSOR_PROPERTY_GET,
    SENSOR_DESCRIPTOR_GET,
    // Steinel's "regular time" (the app's main light time): how long the light
    // stays on after the last motion. It is the Light Control property Time Run On. A missing answer to the read is not a failure of
    // the node: not every device answers every time.
    REGULAR_TIME_GET,
    REGULAR_TIME_SET,
  };
  struct NodeRequest {
    NodeRequestKind kind{NodeRequestKind::ONOFF_GET};
    uint8_t node{0};
    uint8_t element{0};
    uint16_t value{0};
    bool refresh_after{false};
  };
  struct NodeSensorValue {
    uint16_t element{0};
    uint16_t property{0};
    uint8_t length{0};
    std::array<uint8_t, 6> raw{};
  };
  static constexpr size_t NODE_MAX_SENSOR_VALUES = 8;
  static constexpr size_t NODE_COMMAND_QUEUE_SIZE = 12;
  struct NodeRuntime {
    int8_t onoff{-1};
    int32_t lightness{-1};
    int8_t lc_mode{-1};
    // Light Control "ambient lux on" threshold in 0.01 lx; -1 while unknown.
    int32_t threshold_centilux{-1};
    // Composition version ID read from the device (0 while unknown). Steinel packs
    // the firmware version into it: 5 bits major, 5 bits minor, 6 bits patch.
    uint16_t version_id{0};
    uint8_t version_attempts{0};
    uint8_t sensor_probe_attempts{0};
    // Regular time in ms (Light Control Time Run On); -1 while unknown. The attempts count how
    // often it was asked for while unknown.
    int32_t regular_time_ms{-1};
    uint8_t regular_time_attempts{0};
    bool responded{false};
    uint32_t last_response_at{0};
    uint32_t consecutive_failures{0};
    uint8_t sensor_count{0};
    std::array<NodeSensorValue, NODE_MAX_SENSOR_VALUES> sensors{};
  };
  struct BackupBody;
  struct ImportTaskArgs;
  struct AutoUpdateContext;

  bool initialize_bluetooth_();
  bool initialize_mesh_();
  bool deinitialize_mesh_(bool erase_flash);
  bool restore_target_node_();
  void advance_mesh_start_();
  void advance_mesh_remove_();
  void advance_factory_reset_();
  void monitor_iv_index_();
  bool load_sensor_groups_();
  bool save_sensor_groups_(const StoredSensorGroups &groups);
  void subscribe_sensor_groups_();
  static void add_sensor_group_(StoredSensorGroups &groups, uint16_t address);
  void handle_node_sensor_publish_(esp_ble_mesh_sensor_client_cb_param_t *param);
  void store_sensor_data_(uint8_t node, uint8_t element, const uint8_t *data, size_t length);
  bool load_node_table_();
  bool save_node_table_(const StoredNodeTable &table);
  void clear_node_table_();
  bool restore_primary_node_();
  // Generic multi-node engine (steinel_nodes.cpp).
  void advance_node_engine_(uint32_t now);
  void build_node_poll_plan_(int only_node = -1);
  bool send_node_request_(const NodeRequest &request);
  bool find_node_index_(uint16_t address, uint8_t &index) const;
  bool queue_node_command_(const NodeRequest &request);
  bool submit_node_command_(uint8_t node, int on, int brightness_percent, int auto_mode, int threshold_lux,
                            int regular_time_seconds,
                            std::string &error);
  void handle_node_generic_(esp_ble_mesh_generic_client_cb_event_t event,
                            esp_ble_mesh_generic_client_cb_param_t *param);
  void handle_node_light_(esp_ble_mesh_light_client_cb_event_t event,
                          esp_ble_mesh_light_client_cb_param_t *param);
  void handle_node_composition_(esp_ble_mesh_cfg_client_cb_event_t event,
                                esp_ble_mesh_cfg_client_cb_param_t *param);
  void handle_node_sensor_(esp_ble_mesh_sensor_client_cb_event_t event,
                           esp_ble_mesh_sensor_client_cb_param_t *param);
  void node_request_failed_();
  void note_node_response_(const esp_ble_mesh_msg_ctx_t &context);
  void handle_api_node_(AsyncWebServerRequest *request);
  static esp_ble_mesh_model_t *config_model_();
  static esp_ble_mesh_model_t *onoff_model_();
  static esp_ble_mesh_model_t *sensor_model_();
  static esp_ble_mesh_model_t *light_lc_model_();
  static esp_ble_mesh_model_t *light_lightness_model_();
  bool restore_node_table_();
  static const char *product_name_(uint16_t company_id, uint16_t product_id);
  static const char *manufacturer_name_(uint16_t company_id);
  bool install_backup_(BackupBody &body, uint32_t iv_index, uint16_t node_address, std::string &error);
  void handle_nodes_(AsyncWebServerRequest *request);
  void handle_import_(AsyncWebServerRequest *request);
  bool load_config_();
  bool load_device_key_();
  bool load_admin_credentials_();
  bool save_admin_password_(const std::string &password);
  void apply_admin_credentials_();
  static bool valid_admin_password_(const std::string &password);
  bool load_auto_update_();
  bool save_auto_update_(const StoredAutoUpdate &update);
  bool clear_auto_update_();
  void advance_auto_update_();
  void fail_auto_update_(const std::string &error);
  static void auto_update_task_(void *parameter);
  static esp_err_t auto_update_http_event_(esp_http_client_event_t *event);
  void load_retired_address_();
  void retire_local_address_();
  bool load_address_policy_();
  bool save_address_policy_(const StoredAddressPolicy &policy);
  bool select_next_local_address_(uint16_t &address) const;
  bool rotate_local_address_(std::string &error);
  void advance_address_recovery_(uint32_t now);
  bool load_address_confirmation_();
  bool current_address_confirmed_() const;
  void persist_address_confirmation_();
  bool load_cached_iv_index_(const std::array<uint8_t, 16> &mesh_uuid, uint32_t &iv_index);
  void remember_iv_index_(const StoredConfig &config);
  bool save_config_(const StoredConfig &config);
  bool save_device_key_(const std::array<uint8_t, 16> &device_key);
  bool save_enabled_(bool enabled);
  void clear_config_();
  bool parse_backup_(const BackupBody &body, uint32_t requested_iv_index,
                     uint16_t requested_node_address, StoredConfig &config,
                     std::array<uint8_t, 16> &device_key,
                     StoredAddressPolicy &address_policy, StoredNodeTable &node_table,
                     std::string &error);
  void resume_ble_after_mesh_();
  static void import_task_(void *parameter);

  bool authenticate_(AsyncWebServerRequest *request) const;
  void handle_index_(AsyncWebServerRequest *request);
  void handle_status_(AsyncWebServerRequest *request);
  void handle_enable_(AsyncWebServerRequest *request);
  void handle_disable_(AsyncWebServerRequest *request);
  void handle_remove_(AsyncWebServerRequest *request);
  void handle_factory_reset_(AsyncWebServerRequest *request);
  void handle_refresh_(AsyncWebServerRequest *request);
  void handle_password_(AsyncWebServerRequest *request);
  void handle_wifi_(AsyncWebServerRequest *request);
  void handle_auto_update_(AsyncWebServerRequest *request);
  static void send_json_(AsyncWebServerRequest *request, int code, const std::string &body);
  static bool parse_u32_(const std::string &value, uint32_t minimum, uint32_t maximum, uint32_t &output);
  static bool parse_hex_u16_(const std::string &value, uint16_t &output);
  static bool parse_version_(const std::string &value, std::array<uint16_t, 3> &version);
  static bool parse_sha256_(const std::string &value, std::array<uint8_t, 32> &digest);
  void set_status_(const std::string &status, bool publish = true);
  bool set_common_(esp_ble_mesh_client_common_param_t &common, esp_ble_mesh_model_t *model,
                   uint32_t opcode, uint16_t destination);
  bool record_send_result_(esp_err_t result);
  bool begin_access_operation_(AccessOperation operation, uint32_t opcode);
  bool record_access_send_result_(AccessOperation operation, uint32_t opcode, esp_err_t result);
  bool complete_access_operation_(uint32_t opcode, bool success);
  void expire_access_operation_(uint32_t now);
  void record_mesh_rssi_(const esp_ble_mesh_msg_ctx_t &context);
  void bind_model_(uint16_t model_id);
  void keys_bound_();
  void mark_ready_();
  void publish_pending_();

  uint8_t next_tid_();

  static NightmatiqMesh *instance_;

  web_server_base::WebServerBase *base_;
  ESPHomeOTAComponent *ota_;
  ESPPreferenceObject config_preference_;
  ESPPreferenceObject device_key_preference_;
  ESPPreferenceObject retired_address_preference_;
  ESPPreferenceObject address_policy_preference_;
  ESPPreferenceObject address_confirmation_preference_;
  ESPPreferenceObject iv_cache_preference_;
  ESPPreferenceObject admin_credentials_preference_;
  ESPPreferenceObject auto_update_preference_;
  ESPPreferenceObject node_table_preference_;
  ESPPreferenceObject sensor_groups_preference_;
  StoredSensorGroups sensor_groups_{};
  StoredSensorGroups import_sensor_groups_{};
  StoredNodeTable node_table_{};
  bool node_table_valid_{false};
  // Local backup upload. The body is streamed into the inactive OTA partition
  // so the 200+ KB backup never sits in RAM.
  // Generic engine state. node_mutex_ guards node_runtime_ and the queues; the
  // in-flight index is only touched by the main loop and the Mesh callbacks
  // that complete its request, which never overlap because of the access slot.
  std::mutex node_mutex_;
  std::array<NodeRuntime, NODE_TABLE_MAX_NODES> node_runtime_{};
  std::array<NodeRequest, NODE_COMMAND_QUEUE_SIZE> node_commands_{};
  size_t node_command_count_{0};
  // ESP-IDF does not copy the property bytes of a Light LC Property Set, so the
  // buffer must outlive the acknowledged request. One request is in flight at a
  // time, so a single buffer is enough.
  std::array<uint8_t, 3> node_threshold_storage_{};
  net_buf_simple node_threshold_buffer_{};
  std::vector<NodeRequest> node_poll_plan_;
  size_t node_poll_pos_{0};
  NodeRequest node_inflight_{};
  bool node_inflight_valid_{false};
  uint32_t node_next_action_at_{0};
  uint32_t node_next_pass_at_{0};
  BackupBody *import_body_{nullptr};
  std::string import_error_;
  int import_error_status_{0};
  StoredConfig config_{};
  std::array<uint8_t, 16> device_key_{};
  bool device_key_valid_{false};
  uint16_t retired_local_address_{0};
  StoredAddressPolicy address_policy_{};
  bool address_policy_valid_{false};
  StoredAddressConfirmation address_confirmation_{};
  bool address_confirmation_valid_{false};
  bool address_confirmation_save_attempted_this_boot_{false};
  bool configured_{false};
  bool mesh_mode_enabled_{false};
  bool mesh_started_{false};
  bool mesh_start_pending_{false};
  std::atomic<bool> mesh_remove_pending_{false};
  uint32_t mesh_remove_not_before_{0};
  std::atomic<bool> factory_reset_pending_{false};
  uint32_t factory_reset_at_{0};
  uint32_t mesh_start_not_before_{0};
  uint32_t mesh_start_deadline_{0};
  uint32_t iv_index_check_at_{0};
  bool keys_bound_pending_{false};
  uint32_t keys_bound_at_{0};
  uint8_t tid_{0};

  sensor::Sensor *rssi_sensor_{nullptr};
  binary_sensor::BinarySensor *ready_binary_sensor_{nullptr};
  text_sensor::TextSensor *status_text_sensor_{nullptr};

  std::string web_username_;
  std::string web_password_;
  bool using_factory_admin_password_{true};
  StoredAutoUpdate auto_update_{};
  bool auto_update_mode_{false};
  bool auto_update_api_shutdown_started_{false};
  bool auto_update_ble_disable_started_{false};
  uint32_t auto_update_stage_deadline_{0};
  std::atomic<bool> auto_update_running_{false};
  std::atomic<uint8_t> auto_update_progress_{0};
  std::mutex state_mutex_;
  std::string status_{"Configuration required"};
  std::atomic<bool> import_busy_{false};
  std::atomic<bool> ble_resume_pending_{false};
  std::atomic<bool> reboot_pending_{false};
  uint32_t reboot_at_{0};

  std::atomic<bool> mesh_ready_{false};
  uint32_t mesh_ready_at_{0};
  bool address_recovery_attempted_this_boot_{false};
  std::atomic<uint32_t> live_iv_index_{0};
  std::atomic<bool> live_iv_index_confirmed_{false};
  std::atomic<bool> ready_publish_pending_{false};
  std::atomic<bool> web_refresh_pending_{false};
  std::atomic<bool> status_publish_pending_{false};
  std::atomic<uint32_t> mesh_tx_attempts_{0};
  std::atomic<uint32_t> mesh_tx_accepted_{0};
  std::atomic<uint32_t> mesh_tx_errors_{0};
  std::atomic<int32_t> mesh_last_tx_error_{0};
  std::atomic<uint32_t> mesh_rx_messages_{0};
  std::atomic<uint32_t> mesh_timeouts_{0};
  std::atomic<int16_t> last_mesh_rssi_dbm_{0};
  std::atomic<uint32_t> last_mesh_rssi_at_{0};
  std::atomic<bool> mesh_rssi_received_{false};
  std::atomic<bool> mesh_rssi_publish_pending_{false};

  std::atomic<AccessOperation> access_operation_{AccessOperation::NONE};
  std::atomic<uint32_t> access_opcode_{0};
  std::atomic<uint32_t> access_deadline_{0};
  std::atomic<AccessOperation> access_last_completed_{AccessOperation::NONE};
  std::atomic<bool> access_last_success_{false};


  // ESP-IDF deep-copies the Light LC SET structure but not the property-value
  // bytes referenced by it. Keep both objects alive until the acknowledged
  // transaction completes; a stack buffer here corrupts the written lux value.

};

}  // namespace steinel_mesh
}  // namespace esphome

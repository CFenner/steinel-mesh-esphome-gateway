#include "steinel_mesh.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <new>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/esp32_ble/ble.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_ota_ops.h"

// ESP-IDF exposes read/delete operations for provisioner nodes publicly, but
// its settings loader uses this internal restore entry point to rebuild the
// same public esp_ble_mesh_node_t layout. We use it only to reconstruct the
// already-provisioned address range of the primary device from our authenticated backup.
extern "C" {
struct bt_mesh_node;
int bt_mesh_provisioner_restore_node_info(struct bt_mesh_node *node);

}

namespace esphome {
namespace steinel_mesh {

static const char *const TAG = "steinel_mesh";
static constexpr uint8_t MESSAGE_TTL = 7;
// Normal state reads are local, single-hop Mesh traffic. A four-second client
// timeout made a user command wait behind a missed background response. Keep
// routine Access requests short; Composition Data has its own longer timeout.
static constexpr uint32_t MESSAGE_TIMEOUT_MS = 1200;

static void reboot_after_confirming_firmware() {
  const esp_err_t result = esp_ota_mark_app_valid_cancel_rollback();
  if (result != ESP_OK)
    ESP_LOGW(TAG, "Could not confirm the running firmware before restart: %s",
             esp_err_to_name(result));
  App.safe_reboot();
}

NightmatiqMesh *NightmatiqMesh::instance_ = nullptr;

static uint8_t device_uuid[16]{};
static esp_ble_mesh_cfg_srv_t config_server{};
static esp_ble_mesh_client_t config_client{};
static esp_ble_mesh_client_t onoff_client{};
static esp_ble_mesh_client_t sensor_client{};
static esp_ble_mesh_client_t scene_client{};
static esp_ble_mesh_client_t light_lc_client{};
static esp_ble_mesh_client_t light_lightness_client{};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(nullptr, &onoff_client),
    ESP_BLE_MESH_MODEL_SENSOR_CLI(nullptr, &sensor_client),
    ESP_BLE_MESH_MODEL_SCENE_CLI(nullptr, &scene_client),
    ESP_BLE_MESH_MODEL_LIGHT_LC_CLI(nullptr, &light_lc_client),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_CLI(nullptr, &light_lightness_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition{};
static esp_ble_mesh_prov_t *provision = nullptr;

esp_ble_mesh_model_t *NightmatiqMesh::config_model_() { return config_client.model; }
esp_ble_mesh_model_t *NightmatiqMesh::onoff_model_() { return onoff_client.model; }
esp_ble_mesh_model_t *NightmatiqMesh::sensor_model_() { return sensor_client.model; }
esp_ble_mesh_model_t *NightmatiqMesh::light_lc_model_() { return light_lc_client.model; }
esp_ble_mesh_model_t *NightmatiqMesh::light_lightness_model_() { return light_lightness_client.model; }

float NightmatiqMesh::get_setup_priority() const { return setup_priority::AFTER_BLUETOOTH - 1.0f; }
bool NightmatiqMesh::initialize_bluetooth_() {
  esp_err_t error;
  esp_bt_controller_status_t controller_status = esp_bt_controller_get_status();

  if (controller_status == ESP_BT_CONTROLLER_STATUS_IDLE) {
    // Classic Bluetooth is unused. Ignore "already released" on installations
    // where ESPHome initialized the shared BLE host before this component.
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t controller_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    error = esp_bt_controller_init(&controller_config);
    if (error != ESP_OK) {
      ESP_LOGE(TAG, "Bluetooth controller initialization failed: %s", esp_err_to_name(error));
      this->set_status_(std::string("Bluetooth controller initialization failed: ") + esp_err_to_name(error));
      return false;
    }
    controller_status = esp_bt_controller_get_status();
  }

  if (controller_status == ESP_BT_CONTROLLER_STATUS_INITED) {
    error = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (error != ESP_OK) {
      ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(error));
      this->set_status_(std::string("Bluetooth controller enable failed: ") + esp_err_to_name(error));
      return false;
    }
  }

  esp_bluedroid_status_t host_status = esp_bluedroid_get_status();
  if (host_status == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
    error = esp_bluedroid_init();
    if (error != ESP_OK) {
      ESP_LOGE(TAG, "Bluedroid initialization failed: %s", esp_err_to_name(error));
      this->set_status_(std::string("Bluedroid initialization failed: ") + esp_err_to_name(error));
      return false;
    }
    host_status = esp_bluedroid_get_status();
  }
  if (host_status == ESP_BLUEDROID_STATUS_INITIALIZED) {
    error = esp_bluedroid_enable();
    if (error != ESP_OK) {
      ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(error));
      this->set_status_(std::string("Bluedroid enable failed: ") + esp_err_to_name(error));
      return false;
    }
  }

  return esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED &&
         esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED;
}

bool NightmatiqMesh::initialize_mesh_() {
  const uint8_t *address = esp_bt_dev_get_address();
  if (address == nullptr) {
    ESP_LOGE(TAG, "Bluetooth device address is unavailable");
    this->set_status_("Bluetooth device address is unavailable");
    return false;
  }
  device_uuid[0] = 0x4E;  // "NM", only a local provisioner identity marker.
  device_uuid[1] = 0x4D;
  std::memcpy(device_uuid + 2, address, 6);

  config_server.net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20);
  config_server.relay = ESP_BLE_MESH_RELAY_DISABLED;
  config_server.relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20);
  config_server.beacon = ESP_BLE_MESH_BEACON_ENABLED;
  config_server.gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED;
  config_server.friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED;
  config_server.default_ttl = MESSAGE_TTL;

  composition.cid = 0x02E5;  // Espressif company identifier used by their examples.
  composition.element_count = sizeof(elements) / sizeof(elements[0]);
  composition.elements = elements;

  // prov_unicast_addr is const in ESP-IDF. Build the provisioning context only
  // after the saved network has been loaded, so the runtime-selected free
  // address is initialized legally instead of assigned after construction.
  if (provision != nullptr) {
    ESP_LOGE(TAG, "Bluetooth Mesh provisioning context is already initialized");
    this->set_status_("Bluetooth Mesh provisioning context is already initialized");
    return false;
  }
  provision = new (std::nothrow) esp_ble_mesh_prov_t{
      .prov_uuid = device_uuid,
      .prov_unicast_addr = this->config_.local_address,
      .prov_start_address = static_cast<uint16_t>(this->config_.local_address + 1),
      .prov_attention = 0,
      .prov_algorithm = 0,
      .prov_pub_key_oob = 0,
      .prov_static_oob_val = nullptr,
      .prov_static_oob_len = 0,
      .flags = 0,
      .iv_index = this->config_.iv_index,
  };
  if (provision == nullptr) {
    ESP_LOGE(TAG, "Could not allocate Bluetooth Mesh provisioning context");
    this->set_status_("Could not allocate Bluetooth Mesh provisioning context");
    return false;
  }

  esp_ble_mesh_register_prov_callback(NightmatiqMesh::provisioning_callback);
  esp_ble_mesh_register_config_client_callback(NightmatiqMesh::config_callback);
  esp_ble_mesh_register_generic_client_callback(NightmatiqMesh::generic_callback);
  esp_ble_mesh_register_sensor_client_callback(NightmatiqMesh::sensor_callback);
  esp_ble_mesh_register_light_client_callback(NightmatiqMesh::light_callback);
  esp_ble_mesh_register_time_scene_client_callback(NightmatiqMesh::scene_callback);

  this->set_status_("Initializing ESP-BLE-MESH core");
  esp_err_t error = esp_ble_mesh_init(provision, &composition);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "ESP-BLE-MESH initialization failed: %s", esp_err_to_name(error));
    this->set_status_(std::string("ESP-BLE-MESH initialization failed: ") + esp_err_to_name(error));
    delete provision;
    provision = nullptr;
    return false;
  }
  // From this point the Mesh core must be deinitialized explicitly even if a
  // later bearer or key-import step fails.
  this->mesh_started_ = true;
  // ESP-IDF restores persisted model bindings after initializing the Config
  // Client and can overwrite its mandatory DeviceKey binding with an unused
  // value. Configuration messages are DeviceKey-only; restore the SIG-defined
  // binding explicitly after the settings load has completed.
  if (config_client.model != nullptr)
    config_client.model->keys[0] = ESP_BLE_MESH_KEY_DEV;
  this->live_iv_index_.store(this->config_.iv_index);
  this->live_iv_index_confirmed_.store(false);
  this->iv_index_check_at_ = millis() + 1000;

  // ESP-IDF persists the provisioner's primary address independently of the
  // application configuration.  A value restored from Mesh NVS takes
  // precedence over prov_unicast_addr, so explicitly synchronize it before
  // enabling the provisioner bearer.  The bearer is enabled from the
  // completion callback to preserve the required asynchronous ordering.
  this->set_status_("Synchronizing Bluetooth Mesh provisioner address");
  error = esp_ble_mesh_provisioner_set_primary_elem_addr(this->config_.local_address);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Mesh provisioner address request failed: %s", esp_err_to_name(error));
    this->set_status_(std::string("Mesh provisioner address request failed: ") + esp_err_to_name(error));
    return false;
  }
  return true;
}

bool NightmatiqMesh::deinitialize_mesh_(bool erase_flash) {
  if (!this->mesh_started_)
    return true;

  this->mesh_ready_.store(false);
  this->ready_publish_pending_.store(true);
  this->keys_bound_pending_ = false;
  this->access_operation_.store(AccessOperation::NONE);
  this->access_opcode_.store(0);

  // ESP-IDF requires every client model to be deinitialized before the Mesh
  // core. The configuration server at index 0 is owned by the core itself.
  esp_err_t first_error = ESP_OK;
  for (size_t index = 1; index < sizeof(root_models) / sizeof(root_models[0]); index++) {
    const esp_err_t error = esp_ble_mesh_client_model_deinit(&root_models[index]);
    if (error != ESP_OK) {
      ESP_LOGE(TAG, "Mesh client model %u deinit failed: %s",
               static_cast<unsigned>(index), esp_err_to_name(error));
      if (first_error == ESP_OK)
        first_error = error;
    }
  }

  esp_ble_mesh_deinit_param_t parameters{.erase_flash = erase_flash};
  const esp_err_t deinit_error = esp_ble_mesh_deinit(&parameters);
  if (deinit_error != ESP_OK) {
    ESP_LOGE(TAG, "ESP-BLE-MESH deinit failed: %s", esp_err_to_name(deinit_error));
    this->set_status_(std::string("Bluetooth Mesh cleanup failed: ") + esp_err_to_name(deinit_error));
    return false;
  }

  delete provision;
  provision = nullptr;
  this->mesh_started_ = false;
  this->mesh_start_pending_ = false;
  this->live_iv_index_confirmed_.store(false);
  if (first_error != ESP_OK)
    ESP_LOGW(TAG, "Mesh core cleanup completed after a client model deinit error");
  return true;
}

bool NightmatiqMesh::restore_target_node_() {
  // The primary node is restored first; every other node from the
  // imported backup follows so the whole network is known to the provisioner.
  if (!this->restore_primary_node_())
    return false;
  return this->restore_node_table_();
}

bool NightmatiqMesh::restore_primary_node_() {
  if (esp_ble_mesh_provisioner_get_node_with_addr(this->config_.onoff_address) != nullptr)
    return true;

  esp_ble_mesh_node_t node{};
  node.unicast_addr = this->config_.onoff_address;
  // The provisioner checks this range before it permits any unicast Access
  // message or accepts a response from the device. The imported node table knows
  // the real element count (the IS 180 has four, where the NightmatIQ Plus has
  // three, and the sensor is on the last one); without it assume three.
  node.element_num = 3;
  if (this->node_table_valid_) {
    for (uint16_t index = 0; index < this->node_table_.count; index++) {
      const StoredNode &stored = this->node_table_.nodes[index];
      if (stored.address == this->config_.onoff_address && stored.element_count > node.element_num)
        node.element_num = stored.element_count;
    }
  }
  node.net_idx = this->config_.net_key_index;
  node.flags = 0;
  node.iv_index = this->config_.iv_index;
  if (this->device_key_valid_)
    std::memcpy(node.dev_key, this->device_key_.data(), sizeof(node.dev_key));
  std::memcpy(node.dev_uuid, this->config_.mesh_uuid.data(), sizeof(node.dev_uuid));
  node.dev_uuid[14] ^= static_cast<uint8_t>(this->config_.onoff_address >> 8);
  node.dev_uuid[15] ^= static_cast<uint8_t>(this->config_.onoff_address & 0xFF);
  std::strncpy(node.name, this->config_.node_name, sizeof(node.name) - 1);

  const int error = bt_mesh_provisioner_restore_node_info(reinterpret_cast<bt_mesh_node *>(&node));
  if (error != 0) {
    ESP_LOGE(TAG, "Could not restore the primary node 0x%04X: %d", this->config_.onoff_address, error);
    this->set_status_("Could not restore the primary device: " + std::to_string(error));
    return false;
  }
  ESP_LOGI(TAG, "Restored the primary node address range 0x%04X-0x%04X",
           this->config_.onoff_address, this->config_.onoff_address + node.element_num - 1);
  return true;
}

bool NightmatiqMesh::restore_node_table_() {
  if (!this->node_table_valid_)
    return true;
  for (uint16_t index = 0; index < this->node_table_.count; index++) {
    const StoredNode &stored = this->node_table_.nodes[index];
    if (stored.address == 0 || stored.address == this->config_.onoff_address ||
        stored.element_count == 0)
      continue;
    if (esp_ble_mesh_provisioner_get_node_with_addr(stored.address) != nullptr)
      continue;

    esp_ble_mesh_node_t node{};
    node.unicast_addr = stored.address;
    node.element_num = stored.element_count;
    node.net_idx = this->config_.net_key_index;
    node.flags = 0;
    node.iv_index = this->config_.iv_index;
    std::memcpy(node.dev_key, stored.device_key.data(), sizeof(node.dev_key));
    std::memcpy(node.dev_uuid, this->node_table_.mesh_uuid.data(), sizeof(node.dev_uuid));
    node.dev_uuid[14] ^= static_cast<uint8_t>(stored.address >> 8);
    node.dev_uuid[15] ^= static_cast<uint8_t>(stored.address & 0xFF);
    std::strncpy(node.name, stored.name, sizeof(node.name) - 1);

    const int error = bt_mesh_provisioner_restore_node_info(reinterpret_cast<bt_mesh_node *>(&node));
    if (error != 0) {
      // One unusable entry must not prevent the primary node or the remaining
      // nodes from working.
      ESP_LOGW(TAG, "Could not restore node '%s' at 0x%04X: %d", stored.name, stored.address, error);
      continue;
    }
    ESP_LOGI(TAG, "Restored node '%s' address range 0x%04X-0x%04X", stored.name, stored.address,
             stored.address + stored.element_count - 1);
  }
  return true;
}

void NightmatiqMesh::advance_mesh_start_() {
  if (!this->mesh_start_pending_)
    return;

  const uint32_t now = millis();
  if (static_cast<int32_t>(now - this->mesh_start_not_before_) < 0)
    return;

  if (esp32_ble::global_ble == nullptr || !esp32_ble::global_ble->is_active() ||
      esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
    if (static_cast<int32_t>(now - this->mesh_start_deadline_) < 0)
      return;
    this->mesh_start_pending_ = false;
    ESP_LOGE(TAG, "Bluetooth did not become ready for Mesh startup");
    this->set_status_("Bluetooth did not become ready for Mesh startup");
    return;
  }

  this->mesh_start_pending_ = false;
  ESP_LOGI(TAG, "Starting Mesh: free heap=%u, largest block=%u",
           static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
           static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
  this->set_status_("Starting Bluetooth Mesh");
  if (!this->initialize_bluetooth_() || !this->initialize_mesh_())
    return;

  if ((this->config_.flags & FLAG_REMOVE_PENDING) != 0) {
    this->set_status_("Removing Bluetooth Mesh configuration");
    this->mesh_remove_pending_.store(true);
  }
}

void NightmatiqMesh::advance_mesh_remove_() {
  if (!this->mesh_remove_pending_.load())
    return;
  if (static_cast<int32_t>(millis() - this->mesh_remove_not_before_) < 0)
    return;
  if (!this->mesh_remove_pending_.exchange(false))
    return;

  this->set_status_("Stopping Bluetooth Mesh");
  if (!this->deinitialize_mesh_(true))
    return;

  this->clear_config_();
  this->ble_resume_pending_.store(true);
  this->set_status_("Configuration removed; gateway ready for setup");
}

void NightmatiqMesh::advance_factory_reset_() {
  if (!this->factory_reset_pending_.load() ||
      static_cast<int32_t>(millis() - this->factory_reset_at_) < 0)
    return;
  if (!this->factory_reset_pending_.exchange(false))
    return;

  ESP_LOGW(TAG, "Erasing all gateway settings and restoring factory defaults");
  if (!global_preferences->reset()) {
    this->set_status_("Factory reset failed");
    return;
  }
  delay(100);
  reboot_after_confirming_firmware();
}

void NightmatiqMesh::monitor_iv_index_() {
  if (!this->mesh_started_)
    return;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - this->iv_index_check_at_) < 0)
    return;
  this->iv_index_check_at_ = now + 1000;

  const uint32_t live_iv_index = bt_mesh.iv_index;
  this->live_iv_index_.store(live_iv_index);
  // A changed value has passed Secure Network Beacon authentication inside
  // ESP-IDF. A successfully decoded Access response independently proves that
  // the current value is usable even when it equals the imported starting
  // value.
  const bool authenticated = live_iv_index != this->config_.iv_index ||
                             this->mesh_rx_messages_.load() > 0;
  if (!authenticated)
    return;
  this->live_iv_index_confirmed_.store(true);
  if (live_iv_index == this->config_.iv_index &&
      (this->config_.flags & FLAG_IV_INDEX_CONFIRMED) != 0)
    return;

  StoredConfig updated = this->config_;
  const uint32_t previous_iv_index = updated.iv_index;
  updated.iv_index = live_iv_index;
  updated.flags |= FLAG_IV_INDEX_CONFIRMED;
  if (this->save_config_(updated)) {
    ESP_LOGI(TAG, "Persisted authenticated IV Index: %" PRIu32 " -> %" PRIu32 "%s",
             previous_iv_index, live_iv_index,
             previous_iv_index == live_iv_index ? " (confirmed)" : "");
  } else {
    ESP_LOGW(TAG, "Could not persist authenticated IV Index %" PRIu32, live_iv_index);
  }
}

bool NightmatiqMesh::valid_admin_password_(const std::string &password) {
  if (password.size() < ADMIN_PASSWORD_MIN_LENGTH ||
      password.size() > ADMIN_PASSWORD_MAX_LENGTH)
    return false;
  return std::all_of(password.begin(), password.end(), [](unsigned char value) {
    return value >= 0x21 && value <= 0x7E;
  });
}

bool NightmatiqMesh::load_admin_credentials_() {
  StoredAdminCredentials stored{};
  if (!this->admin_credentials_preference_.load(&stored) ||
      stored.magic != ADMIN_CREDENTIALS_MAGIC ||
      stored.version != ADMIN_CREDENTIALS_VERSION ||
      stored.password[ADMIN_PASSWORD_MAX_LENGTH] != '\0') {
    this->using_factory_admin_password_ = true;
    return false;
  }

  const std::string password(stored.password);
  if (!valid_admin_password_(password)) {
    this->using_factory_admin_password_ = true;
    return false;
  }

  this->web_password_ = password;
  this->using_factory_admin_password_ = false;
  return true;
}

bool NightmatiqMesh::save_admin_password_(const std::string &password) {
  if (!valid_admin_password_(password))
    return false;

  StoredAdminCredentials stored{};
  std::memcpy(stored.password, password.data(), password.size());
  stored.password[password.size()] = '\0';
  if (!this->admin_credentials_preference_.save(&stored))
    return false;

  std::fill(this->web_password_.begin(), this->web_password_.end(), '\0');
  this->web_password_ = password;
  this->using_factory_admin_password_ = false;
  this->apply_admin_credentials_();
  return true;
}

void NightmatiqMesh::apply_admin_credentials_() {
  this->base_->set_auth_username(this->web_username_);
  this->base_->set_auth_password(this->web_password_);
  if (this->ota_ != nullptr)
    this->ota_->set_auth_password(this->web_password_);
}

void NightmatiqMesh::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Bluetooth Mesh gateway");
  this->instance_ = this;
  this->config_preference_ = global_preferences->make_preference<StoredConfig>(0x4E4D5101U);
  this->device_key_preference_ = global_preferences->make_preference<StoredDeviceKey>(0x4E4D5102U);
  this->retired_address_preference_ =
      global_preferences->make_preference<StoredRetiredAddress>(0x4E4D5103U);
  this->iv_cache_preference_ = global_preferences->make_preference<StoredIvCache>(0x4E4D5104U);
  this->address_policy_preference_ =
      global_preferences->make_preference<StoredAddressPolicy>(0x4E4D5106U);
  this->address_confirmation_preference_ =
      global_preferences->make_preference<StoredAddressConfirmation>(0x4E4D5107U);
  this->admin_credentials_preference_ =
      global_preferences->make_preference<StoredAdminCredentials>(0x4E4D5108U);
  this->auto_update_preference_ =
      global_preferences->make_preference<StoredAutoUpdate>(0x4E4D5109U);
  this->node_table_preference_ =
      global_preferences->make_preference<StoredNodeTable>(0x4E4D510AU);
  this->load_admin_credentials_();
  this->apply_admin_credentials_();
  this->base_->add_handler(this);
  const bool auto_update_pending = this->load_auto_update_();
  this->load_retired_address_();
  this->load_address_policy_();
  this->load_address_confirmation_();
  if (this->ready_binary_sensor_ != nullptr)
    this->ready_binary_sensor_->publish_state(false);
  const bool has_config = this->load_config_();
  if (has_config) {
    this->load_device_key_();
    this->load_node_table_();
    this->mesh_mode_enabled_ =
        (this->config_.flags & (FLAG_ENABLED | FLAG_REMOVE_PENDING)) != 0;
  }
  if (auto_update_pending) {
    this->set_status_("Firmware update pending; waiting for network");
    return;
  }
  if (!has_config) {
    this->set_status_("Gateway ready; import a network backup on this page");
    return;
  }
  if (!this->mesh_mode_enabled_) {
    this->set_status_("Bluetooth Mesh disabled; gateway in setup mode");
    return;
  }
  // Start the Mesh as soon as the Bluetooth stack is ready.
  this->mesh_start_pending_ = true;
  this->mesh_start_not_before_ = millis() + 250;
  this->mesh_start_deadline_ = millis() + 30000;
  this->set_status_("Preparing Bluetooth Mesh");
}

void NightmatiqMesh::dump_config() {
  ESP_LOGCONFIG(TAG, "Steinel Bluetooth Mesh gateway:");
  ESP_LOGCONFIG(TAG, "  Configuration: %s", YESNO(this->configured_));
  if (!this->configured_)
    return;
  ESP_LOGCONFIG(TAG, "  Integration enabled: %s", YESNO(this->mesh_mode_enabled_));
  ESP_LOGCONFIG(TAG, "  BLE runtime mode: %s", this->mesh_mode_enabled_ ? "Bluetooth Mesh" : "Setup");
  ESP_LOGCONFIG(TAG, "  Network: %s", this->config_.network_name);
  ESP_LOGCONFIG(TAG, "  Node: %s", this->config_.node_name);
  ESP_LOGCONFIG(TAG, "  Initial IV Index: %" PRIu32, this->config_.iv_index);
  ESP_LOGCONFIG(TAG, "  Local address: 0x%04X", this->config_.local_address);
  ESP_LOGCONFIG(TAG, "  Primary device address: 0x%04X", this->config_.onoff_address);
  ESP_LOGCONFIG(TAG, "  Mesh models ready: %s", YESNO(this->mesh_ready_.load()));
}

bool NightmatiqMesh::set_common_(esp_ble_mesh_client_common_param_t &common, esp_ble_mesh_model_t *model,
                                 uint32_t opcode, uint16_t destination) {
  if (!this->mesh_ready_.load() || model == nullptr)
    return false;
  std::memset(&common, 0, sizeof(common));
  common.opcode = opcode;
  common.model = model;
  common.ctx.net_idx = this->config_.net_key_index;
  common.ctx.app_idx = this->config_.app_key_index;
  common.ctx.addr = destination;
  common.ctx.send_ttl = MESSAGE_TTL;
  common.msg_timeout = MESSAGE_TIMEOUT_MS;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
  common.msg_role = ROLE_PROVISIONER;
#endif
  return true;
}

bool NightmatiqMesh::record_send_result_(esp_err_t result) {
  this->mesh_tx_attempts_.fetch_add(1);
  this->mesh_last_tx_error_.store(result);
  if (result == ESP_OK) {
    this->mesh_tx_accepted_.fetch_add(1);
    return true;
  }
  this->mesh_tx_errors_.fetch_add(1);
  return false;
}

bool NightmatiqMesh::begin_access_operation_(AccessOperation operation, uint32_t opcode) {
  AccessOperation expected = AccessOperation::NONE;
  if (!this->access_operation_.compare_exchange_strong(expected, operation)) {
    ESP_LOGW(TAG, "Access request 0x%08" PRIX32 " deferred; another acknowledged request is active",
             opcode);
    return false;
  }
  this->access_opcode_.store(opcode);
  this->access_deadline_.store(millis() + MESSAGE_TIMEOUT_MS + 750);
  return true;
}

bool NightmatiqMesh::record_access_send_result_(AccessOperation operation, uint32_t opcode,
                                                esp_err_t result) {
  const bool accepted = this->record_send_result_(result);
  if (!accepted)
    this->complete_access_operation_(opcode, false);
  return accepted;
}

bool NightmatiqMesh::complete_access_operation_(uint32_t opcode, bool success) {
  if (this->access_opcode_.load() != opcode)
    return false;
  const AccessOperation operation = this->access_operation_.load();
  if (operation == AccessOperation::NONE)
    return false;

  // Publish the completion before releasing the global access slot. The main
  // loop can then safely advance a control transaction as soon as it observes
  // NONE, without racing a stale result from the callback task.
  this->access_last_completed_.store(operation);
  this->access_last_success_.store(success);
  this->access_opcode_.store(0);
  this->access_operation_.store(AccessOperation::NONE);
  return true;
}

void NightmatiqMesh::expire_access_operation_(uint32_t now) {
  const AccessOperation operation = this->access_operation_.load();
  if (operation == AccessOperation::NONE ||
      static_cast<int32_t>(now - this->access_deadline_.load()) < 0)
    return;
  const uint32_t opcode = this->access_opcode_.load();
  if (this->complete_access_operation_(opcode, false)) {
    this->mesh_timeouts_.fetch_add(1);
    ESP_LOGW(TAG, "Access request watchdog expired for opcode 0x%08" PRIX32, opcode);
  }
}

void NightmatiqMesh::bind_model_(uint16_t model_id) {
  const uint16_t local_address = esp_ble_mesh_get_primary_element_address();
  if (local_address == ESP_BLE_MESH_ADDR_UNASSIGNED) {
    ESP_LOGE(TAG, "Cannot bind AppKey to model 0x%04X: local primary element is unassigned", model_id);
    return;
  }
  if (local_address != this->config_.local_address) {
    ESP_LOGW(TAG, "Local primary element is 0x%04X, expected 0x%04X", local_address,
             this->config_.local_address);
  }
  const esp_err_t error = esp_ble_mesh_provisioner_bind_app_key_to_local_model(
      local_address, this->config_.app_key_index, model_id, ESP_BLE_MESH_CID_NVAL);
  if (error != ESP_OK)
    ESP_LOGE(TAG, "Local AppKey bind request for model 0x%04X failed: %s", model_id, esp_err_to_name(error));
}

void NightmatiqMesh::mark_ready_() {
  this->mesh_ready_.store(true);
  this->mesh_ready_at_ = millis();
  this->address_recovery_attempted_this_boot_ = false;
  this->ready_publish_pending_.store(true);
  this->set_status_("Mesh client ready");
  ESP_LOGI(TAG, "Mesh keys imported and all client models bound");
}

void NightmatiqMesh::advance_address_recovery_(uint32_t now) {
  if (this->address_recovery_attempted_this_boot_ || !this->mesh_ready_.load() ||
      !this->configured_ || !this->mesh_mode_enabled_ ||
      !this->address_policy_valid_ || this->current_address_confirmed_() ||
      this->mesh_ready_at_ == 0 ||
      static_cast<uint32_t>(now - this->mesh_ready_at_) < AUTO_ADDRESS_RECOVERY_DELAY_MS ||
      this->mesh_rx_messages_.load() != 0 ||
      this->mesh_tx_accepted_.load() < AUTO_ADDRESS_MIN_ACCEPTED_TX ||
      this->mesh_timeouts_.load() < AUTO_ADDRESS_MIN_TIMEOUTS ||
      this->import_busy_.load() || this->reboot_pending_.load() ||
      this->access_operation_.load() != AccessOperation::NONE)
    return;

  this->address_recovery_attempted_this_boot_ = true;
  std::string error;
  if (!this->rotate_local_address_(error)) {
    ESP_LOGE(TAG, "Automatic local Mesh address recovery stopped: %s", error.c_str());
    this->set_status_(error);
  }
}

void NightmatiqMesh::keys_bound_() {
  // A newly imported value has not been authenticated yet, so give the stack
  // time to recover a newer IV Index from a Secure Network Beacon. Once an
  // authenticated Access response (or beacon update) has confirmed it, retain
  // that fact across gateway restarts and start normal polling promptly.
  const bool previously_confirmed =
      (this->config_.flags & FLAG_IV_INDEX_CONFIRMED) != 0;
  this->keys_bound_pending_ = true;
  this->keys_bound_at_ = millis() + (previously_confirmed ? 1000 : 15000);
  this->set_status_(previously_confirmed
                        ? "Mesh keys loaded; restoring confirmed IV Index"
                        : "Mesh keys loaded; synchronizing IV Index");
}

void NightmatiqMesh::provisioning_callback(esp_ble_mesh_prov_cb_event_t event,
                                           esp_ble_mesh_prov_cb_param_t *param) {
  NightmatiqMesh *self = NightmatiqMesh::instance_;
  if (self == nullptr || param == nullptr)
    return;

  switch (event) {
    case ESP_BLE_MESH_PROVISIONER_SET_PRIMARY_ELEM_ADDR_COMP_EVT: {
      const int address_error = param->provisioner_set_primary_elem_addr_comp.err_code;
      if (address_error != 0) {
        ESP_LOGE(TAG, "Bluetooth Mesh provisioner address synchronization failed: %d", address_error);
        self->set_status_("Bluetooth Mesh provisioner address synchronization failed: " +
                          std::to_string(address_error));
        return;
      }

      const uint16_t actual_address = esp_ble_mesh_get_primary_element_address();
      ESP_LOGI(TAG, "Bluetooth Mesh local primary element: 0x%04X", actual_address);
      if (actual_address != self->config_.local_address) {
        ESP_LOGE(TAG, "Bluetooth Mesh local address mismatch: expected 0x%04X", self->config_.local_address);
        self->set_status_("Bluetooth Mesh local address mismatch");
        return;
      }

      self->set_status_("Enabling Bluetooth Mesh bearer");
      const esp_err_t result = esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV);
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "Mesh advertising bearer enable failed: %s", esp_err_to_name(result));
        self->set_status_(std::string("Mesh advertising bearer enable failed: ") + esp_err_to_name(result));
        return;
      }
      // The provisioner creates its reserved primary NetKey asynchronously.
      // Import the Steinel key only after PROV_ENABLE_COMP_EVT because
      // add_local_net_key explicitly rejects the reserved primary index (0).
      self->set_status_("Waiting for Bluetooth Mesh provisioner");
      break;
    }
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT: {
      const int enable_error = param->provisioner_prov_enable_comp.err_code;
      if (enable_error != 0) {
        ESP_LOGE(TAG, "Bluetooth Mesh provisioner enable failed: %d", enable_error);
        self->set_status_("Bluetooth Mesh provisioner enable failed: " + std::to_string(enable_error));
        return;
      }

      self->set_status_("Importing primary Mesh key");
      const uint16_t net_idx = self->config_.net_key_index;
      const uint8_t *existing_net_key = esp_ble_mesh_provisioner_get_local_net_key(net_idx);
      // ESP-IDF reserves index 0 as ESP_BLE_MESH_KEY_PRIMARY and forbids
      // add_local_net_key() for it. The primary key always exists after the
      // provisioner-enable completion event, so replace it with the Steinel
      // key. Non-primary networks retain normal add/update semantics.
      const esp_err_t result =
          net_idx == ESP_BLE_MESH_KEY_PRIMARY || existing_net_key != nullptr
              ? esp_ble_mesh_provisioner_update_local_net_key(self->config_.net_key.data(), net_idx)
              : esp_ble_mesh_provisioner_add_local_net_key(self->config_.net_key.data(), net_idx);
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "Primary NetKey import request failed: %s", esp_err_to_name(result));
        self->set_status_(std::string("Primary NetKey import request failed: ") + esp_err_to_name(result));
      }
      break;
    }
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_NET_KEY_COMP_EVT:
    case ESP_BLE_MESH_PROVISIONER_UPDATE_LOCAL_NET_KEY_COMP_EVT: {
      const int error = event == ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_NET_KEY_COMP_EVT
                            ? param->provisioner_add_net_key_comp.err_code
                            : param->provisioner_update_net_key_comp.err_code;
      if (error != 0) {
        ESP_LOGE(TAG, "Primary NetKey import failed: %d", error);
        return;
      }
      // Provisioner callbacks execute in the ESP-BLE-MESH task. Restore the
      // target entry here, after the NetKey exists, rather than touching the
      // private provisioner table from ESPHome's main loop.
      if (!self->restore_target_node_())
        return;
      const uint8_t *existing_app_key =
          esp_ble_mesh_provisioner_get_local_app_key(self->config_.net_key_index, self->config_.app_key_index);
      const esp_err_t result = existing_app_key == nullptr
                                   ? esp_ble_mesh_provisioner_add_local_app_key(
                                         self->config_.app_key.data(), self->config_.net_key_index,
                                         self->config_.app_key_index)
                                   : esp_ble_mesh_provisioner_update_local_app_key(
                                         self->config_.app_key.data(), self->config_.net_key_index,
                                         self->config_.app_key_index);
      if (result != ESP_OK)
        ESP_LOGE(TAG, "AppKey import request failed: %s", esp_err_to_name(result));
      break;
    }
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
      if (param->provisioner_add_app_key_comp.err_code != 0) {
        ESP_LOGE(TAG, "AppKey import failed: %d", param->provisioner_add_app_key_comp.err_code);
        return;
      }
      self->bind_model_(ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI);
      break;
    case ESP_BLE_MESH_PROVISIONER_UPDATE_LOCAL_APP_KEY_COMP_EVT:
      if (param->provisioner_update_app_key_comp.err_code != 0) {
        ESP_LOGE(TAG, "AppKey update failed: %d", param->provisioner_update_app_key_comp.err_code);
        return;
      }
      self->bind_model_(ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI);
      break;
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT: {
      const auto &binding = param->provisioner_bind_app_key_to_model_comp;
      if (binding.err_code != 0) {
        ESP_LOGE(TAG, "AppKey bind failed for model 0x%04X: %d", binding.model_id, binding.err_code);
        return;
      }
      if (binding.model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI)
        self->bind_model_(ESP_BLE_MESH_MODEL_ID_SENSOR_CLI);
      else if (binding.model_id == ESP_BLE_MESH_MODEL_ID_SENSOR_CLI)
        self->bind_model_(ESP_BLE_MESH_MODEL_ID_SCENE_CLI);
      else if (binding.model_id == ESP_BLE_MESH_MODEL_ID_SCENE_CLI)
        self->bind_model_(ESP_BLE_MESH_MODEL_ID_LIGHT_LC_CLI);
      else if (binding.model_id == ESP_BLE_MESH_MODEL_ID_LIGHT_LC_CLI)
        self->bind_model_(ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_CLI);
      else if (binding.model_id == ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_CLI)
        self->keys_bound_();
      break;
    }
    default:
      break;
  }
}
void NightmatiqMesh::config_callback(esp_ble_mesh_cfg_client_cb_event_t event,
                                     esp_ble_mesh_cfg_client_cb_param_t *param) {
  NightmatiqMesh *self = NightmatiqMesh::instance_;
  if (self == nullptr || param == nullptr)
    return;
  // A composition read issued by the node engine for one device.
  if (param->params != nullptr && is_node_operation_(self->access_operation_.load()) &&
      param->params->opcode == self->access_opcode_.load())
    self->handle_node_composition_(event, param);
}

uint8_t NightmatiqMesh::next_tid_() { return ++this->tid_; }
void NightmatiqMesh::record_mesh_rssi_(const esp_ble_mesh_msg_ctx_t &context) {
  this->last_mesh_rssi_dbm_.store(context.recv_rssi);
  this->last_mesh_rssi_at_.store(millis());
  this->mesh_rssi_received_.store(true);
  this->mesh_rssi_publish_pending_.store(true);
}

void NightmatiqMesh::generic_callback(esp_ble_mesh_generic_client_cb_event_t event,
                                      esp_ble_mesh_generic_client_cb_param_t *param) {
  NightmatiqMesh *self = NightmatiqMesh::instance_;
  if (self == nullptr || param == nullptr || param->params == nullptr)
    return;
  if (is_node_operation_(self->access_operation_.load()) &&
      param->params->opcode == self->access_opcode_.load())
    self->handle_node_generic_(event, param);
}

void NightmatiqMesh::sensor_callback(esp_ble_mesh_sensor_client_cb_event_t event,
                                     esp_ble_mesh_sensor_client_cb_param_t *param) {
  NightmatiqMesh *self = NightmatiqMesh::instance_;
  if (self == nullptr || param == nullptr || param->params == nullptr)
    return;
  if (is_node_operation_(self->access_operation_.load()) &&
      param->params->opcode == self->access_opcode_.load())
    self->handle_node_sensor_(event, param);
}

void NightmatiqMesh::light_callback(esp_ble_mesh_light_client_cb_event_t event,
                                    esp_ble_mesh_light_client_cb_param_t *param) {
  NightmatiqMesh *self = NightmatiqMesh::instance_;
  if (self == nullptr || param == nullptr || param->params == nullptr)
    return;
  if (is_node_operation_(self->access_operation_.load()) &&
      param->params->opcode == self->access_opcode_.load())
    self->handle_node_light_(event, param);
}
void NightmatiqMesh::scene_callback(esp_ble_mesh_time_scene_client_cb_event_t event,
                                    esp_ble_mesh_time_scene_client_cb_param_t *param) {
  // The gateway sends no Scene requests any more; nothing to handle.
  (void) event;
  (void) param;
}
void NightmatiqMesh::publish_pending_() {
  if (this->status_publish_pending_.exchange(false) && this->status_text_sensor_ != nullptr) {
    std::string status;
    {
      std::lock_guard<std::mutex> lock(this->state_mutex_);
      status = this->status_;
    }
    this->status_text_sensor_->publish_state(status);
  }
  if (this->ready_publish_pending_.exchange(false)) {
    if (this->ready_binary_sensor_ != nullptr)
      this->ready_binary_sensor_->publish_state(this->mesh_ready_.load());
  }
  if (this->mesh_rssi_publish_pending_.exchange(false)) {
    if (this->rssi_sensor_ != nullptr)
      this->rssi_sensor_->publish_state(this->last_mesh_rssi_dbm_.load());
  }
}
void NightmatiqMesh::request_refresh() {
  if (!this->mesh_ready_.load()) {
    if (!this->configured_)
      this->set_status_("Configuration required");
    else if (!this->mesh_mode_enabled_)
      this->set_status_("Bluetooth Mesh disabled; gateway in setup mode");
    else
      this->set_status_("Mesh is still synchronizing");
    return;
  }
  // Let the node engine read every device again right away.
  this->node_poll_plan_.clear();
  this->node_poll_pos_ = 0;
  this->node_next_pass_at_ = millis();
}

void NightmatiqMesh::loop() {
  this->advance_factory_reset_();
  this->publish_pending_();
  if (this->auto_update_mode_) {
    this->advance_auto_update_();
    const uint32_t update_now = millis();
    if (this->reboot_pending_.load() &&
        static_cast<int32_t>(update_now - this->reboot_at_) >= 0) {
      this->reboot_pending_.store(false);
      reboot_after_confirming_firmware();
    }
    return;
  }
  this->persist_address_confirmation_();
  this->advance_mesh_start_();
  this->advance_mesh_remove_();
  this->monitor_iv_index_();
  this->resume_ble_after_mesh_();

  const uint32_t now = millis();
  if (this->reboot_pending_.load() && static_cast<int32_t>(now - this->reboot_at_) >= 0) {
    this->reboot_pending_.store(false);
    reboot_after_confirming_firmware();
    return;
  }
  if (this->keys_bound_pending_ && static_cast<int32_t>(now - this->keys_bound_at_) >= 0) {
    this->keys_bound_pending_ = false;
    this->mark_ready_();
  }
  if (!this->mesh_ready_.load())
    return;
  if (this->web_refresh_pending_.load() &&
      this->access_operation_.load() == AccessOperation::NONE) {
    this->web_refresh_pending_.store(false);
    this->request_refresh();
  }
  this->expire_access_operation_(now);
  this->advance_address_recovery_(now);
  if (this->reboot_pending_.load())
    return;
  this->advance_node_engine_(now);
}

void NightmatiqMesh::resume_ble_after_mesh_() {
  if (!this->ble_resume_pending_.load() || this->mesh_mode_enabled_)
    return;

  if (esp32_ble::global_ble != nullptr && !esp32_ble::global_ble->is_active()) {
    esp32_ble::global_ble->enable();
    return;
  }

  // This standalone gateway has no Bluetooth Proxy role. Leave the setup
  // scanner idle until the next explicit identity scan or Mesh-mode reboot.
  this->ble_resume_pending_.store(false);
}

}  // namespace steinel_mesh
}  // namespace esphome

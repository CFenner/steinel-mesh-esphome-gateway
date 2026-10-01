#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="$ROOT_DIR/esphome/steinel-c3.yaml"
FAIL=0
IS_GIT_WORKTREE=0
if git -C "$ROOT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  IS_GIT_WORKTREE=1
fi

ok() { printf 'OK: %s\n' "$1"; }
fail() { printf 'ERROR: %s\n' "$1" >&2; FAIL=$((FAIL + 1)); }

printf '%s\n' '=== Steinel NightmatIQ ESP32-C3 Gateway self-test ==='

for script in "$ROOT_DIR"/scripts/*.sh; do
  if bash -n "$script"; then
    ok "shell syntax: $(basename "$script")"
  else
    fail "invalid shell syntax: $(basename "$script")"
  fi
done

if grep -Fq 'python3 -m venv --clear "$VENV"' "$ROOT_DIR/scripts/01_install_esphome.sh"; then
  ok 'ESPHome installer recreates a clean virtual environment'
else
  fail 'ESPHome installer does not recreate a clean virtual environment'
fi

for path in \
  "$ROOT_DIR/README.md" \
  "$ROOT_DIR/CHANGELOG.md" \
  "$CONFIG" \
  "$ROOT_DIR/esphome/components/steinel_mesh/__init__.py" \
  "$ROOT_DIR/esphome/components/steinel_mesh/steinel_mesh.h" \
  "$ROOT_DIR/esphome/components/steinel_mesh/steinel_mesh.cpp" \
  "$ROOT_DIR/esphome/components/steinel_mesh/steinel_web.cpp" \
  "$ROOT_DIR/esphome/components/steinel_mesh/steinel_page.html" \
  "$ROOT_DIR/esphome/components/steinel_mesh/steinel_page.h" \
  "$ROOT_DIR/scripts/10_prepare_release.sh" \
  "$ROOT_DIR/docs/images/esp32-c3-super-mini.jpg" \
  "$ROOT_DIR/docs/images/steinel-web-interface.png" \
  "$ROOT_DIR/docs/images/home-assistant-device.png"; do
  if [[ -f "$path" ]]; then
    ok "required file: ${path#"$ROOT_DIR/"}"
  else
    fail "missing file: ${path#"$ROOT_DIR/"}"
  fi
done

for marker in \
  'project_version: "0.1.0"' \
  '## Unreleased'; do
  if grep -Fq "$marker" "$CONFIG" "$ROOT_DIR/CHANGELOG.md"; then
    ok "release marker: $marker"
  else
    fail "missing release marker: $marker"
  fi
done

for private_path in \
  "$ROOT_DIR/esphome/secrets.yaml"; do
  if (( IS_GIT_WORKTREE )) && git -C "$ROOT_DIR" ls-files --error-unmatch \
    "${private_path#"$ROOT_DIR/"}" >/dev/null 2>&1; then
    fail "private file tracked by repository: ${private_path#"$ROOT_DIR/"}"
  fi
done

if (( IS_GIT_WORKTREE )) && git -C "$ROOT_DIR" ls-files | grep -Eiq \
  '(^|/)(\.DS_Store|\._[^/]*|Thumbs\.db|Desktop\.ini)$'; then
  fail 'operating-system or generated work file present in repository'
else
  ok 'no operating-system or generated work files'
fi

if grep -REiq --exclude-dir=.git --exclude-dir=.esphome --exclude-dir=output \
  --exclude='*.png' --exclude='*.jpg' --exclude='*.jpeg' \
  '[0-9]+\.[0-9]+\.[0-9]+-dev|10\.1\.0\.[0-9]+' "$ROOT_DIR"; then
  fail 'private or development marker present in public repository text'
else
  ok 'no private or development markers in public repository text'
fi

grep -Fq 'web.esphome.io' "$ROOT_DIR/README.md" || fail 'missing ready-made installation instructions: README.md'

for marker in \
  'variant: esp32c3' \
  'CONFIG_BLE_MESH_PROVISIONER: y' \
  'CONFIG_BT_BLE_50_FEATURES_SUPPORTED: n' \
  'CONFIG_ESP_WIFI_SOFTAP_SUPPORT: y' \
  'factory_username: "admin"' \
  'factory_password: "12345678"' \
  'id: nightmatiq_ota' \
  'password: ""' \
  'password: "${factory_password}"' \
  'channel: 6' \
  'ap_timeout: 60s' \
  'captive_portal:' \
  'mdns:' \
  'disabled: false' \
  'username: "${factory_username}"'; do
  grep -Fq "$marker" "$CONFIG" || fail "missing configuration marker: $marker"
done

for forbidden in api_encryption_key manual_ip '!secret' wifi_ssid wifi_password; do
  if grep -Rq --exclude-dir=.esphome --exclude=secrets.yaml "$forbidden" "$ROOT_DIR/esphome"; then
    fail "forbidden standalone configuration marker: $forbidden"
  fi
done

if grep -REq --exclude-dir=.esphome --exclude=secrets.yaml '10\.1\.0\.[0-9]+' "$ROOT_DIR/esphome" "$ROOT_DIR/scripts"; then
  fail 'a deployment IP address is hardcoded in firmware or scripts'
else
  ok 'no deployment IP address is hardcoded in firmware or scripts'
fi

if python3 - "$ROOT_DIR" <<'PY'
from __future__ import annotations

import ast
import gzip
import re
import sys
from pathlib import Path

root = Path(sys.argv[1])
errors: list[str] = []

for path in (root / "scripts").glob("*.py"):
    try:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except SyntaxError as error:
        errors.append(f"invalid Python syntax in {path.name}: {error}")

html = (root / "esphome/components/steinel_mesh/steinel_page.html").read_bytes()
header = (root / "esphome/components/steinel_mesh/steinel_page.h").read_text(encoding="utf-8")
size_match = re.search(r"NIGHTMATIQ_PAGE_RAW_SIZE = (\d+);", header)
array_match = re.search(r"NIGHTMATIQ_PAGE_GZ\[\d+\] = \{(.*?)\};", header, re.DOTALL)
if size_match is None or array_match is None:
    errors.append("generated NightmatIQ page header has an invalid structure")
else:
    compressed = bytes(int(value, 16) for value in re.findall(r"0x([0-9a-fA-F]{2})", array_match.group(1)))
    try:
        embedded = gzip.decompress(compressed)
    except gzip.BadGzipFile as error:
        errors.append(f"generated NightmatIQ page is not valid gzip: {error}")
    else:
        if embedded != html:
            errors.append("generated NightmatIQ page header is stale")
        if int(size_match.group(1)) != len(html):
            errors.append("generated NightmatIQ raw page size is stale")

mesh = (root / "esphome/components/steinel_mesh/steinel_mesh.cpp").read_text(encoding="utf-8")
for marker in (
    "esp_ble_gap_set_rand_addr(random_address)",
    "esp_ble_gap_set_scan_params(&this->identity_scan_params_)",
    "esp_ble_gap_start_scanning(IDENTITY_SCAN_WINDOW_MS / 1000U)",
    "ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET",
    "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET",
):
    if marker not in mesh:
        errors.append(f"missing NightmatIQ source marker: {marker}")

header_source = (root / "esphome/components/steinel_mesh/steinel_mesh.h").read_text(encoding="utf-8")
web_source = (root / "esphome/components/steinel_mesh/steinel_web.cpp").read_text(encoding="utf-8")
component_source = (root / "esphome/components/steinel_mesh/__init__.py").read_text(encoding="utf-8")
config_source = (root / "esphome/steinel-c3.yaml").read_text(encoding="utf-8")
page_source = html.decode("utf-8")
for source, marker in (
    (header_source, "ADDRESS_POOL_TARGET_SIZE = 2048"),
    (header_source, "AUTO_ADDRESS_ROTATION_LIMIT = 16"),
    (header_source, "struct StoredAddressConfirmation"),
    (header_source, "struct StoredAdminCredentials"),
    (header_source, "ADMIN_PASSWORD_MIN_LENGTH = 8"),
    (header_source, "struct StoredAutoUpdate"),
    (header_source, "AUTO_UPDATE_URL_MAX_LENGTH = 255"),
    (component_source, "esp32_ble.register_gap_event_handler(ble, var)"),
    (component_source, "esp32_ble.register_gap_scan_event_handler(ble, var)"),
    (component_source, "ESPHomeOTAComponent"),
    (mesh, "advance_address_recovery_(now)"),
    (mesh, "reboot_after_confirming_firmware()"),
    (mesh, "esp_ota_mark_app_valid_cancel_rollback()"),
    (mesh, "current_address_confirmed_()"),
    (mesh, "!this->identity_found_this_boot_.load()"),
    (web_source, "esp_read_mac(mac, ESP_MAC_WIFI_STA)"),
    (web_source, "address_policy.installation_nonce = esp_random()"),
    (web_source, "mesh_rx_messages_.load() == 0"),
    (web_source, "Confirmed local Mesh address"),
    (mesh, "context.recv_rssi"),
    (header_source, "mesh_rssi_received_"),
    (header_source, "mesh_rssi_publish_pending_"),
    (header_source, "set_rssi_sensor"),
    (mesh, "mesh_rssi_publish_pending_.store(true)"),
    (mesh, "rssi_sensor_->publish_state"),
    (component_source, 'CONF_RSSI_SENSOR_ID = "rssi_sensor_id"'),
    (component_source, "var.set_rssi_sensor(rssi)"),
    (config_source, 'friendly_name: "Steinel Mesh Gateway"'),
    (config_source, 'main_light_internal: "true"'),
    (config_source, "platform: debug"),
    (config_source, "id(nightmatiq_gateway).mesh_tx_attempts()"),
    (config_source, "rssi_sensor_id: nightmatiq_rssi"),
    (config_source, 'name: "Signal Strength"'),
    (config_source, "device_class: signal_strength"),
    (config_source, 'name: "Refresh"'),
    (config_source, "id(nightmatiq_gateway).request_refresh();"),
    (config_source, "optimistic: true"),
    (config_source, "restore_value: true"),
    (config_source, 'initial_option: "Auto"'),
    (web_source, "mesh_last_rssi_dbm"),
    (web_source, "mesh_last_rssi_age_seconds"),
    (page_source, "Last Mesh RSSI"),
    (page_source, "meshRssi"),
    (web_source, "api::global_api_server->on_shutdown()"),
    (web_source, "api::global_api_server->teardown()"),
    (header_source, "web_refresh_pending_"),
    (mesh, "this->request_refresh()"),
    (web_source, 'url == "/steinel/mode"'),
    (web_source, 'url == "/steinel/threshold"'),
    (web_source, 'url == "/steinel/refresh"'),
    (web_source, 'url == "/steinel/password"'),
    (web_source, 'url == "/steinel/wifi"'),
    (web_source, "handle_mode_"),
    (web_source, "handle_threshold_"),
    (web_source, "handle_refresh_"),
    (web_source, "handle_password_"),
    (web_source, "handle_wifi_"),
    (web_source, "global_wifi_component->save_wifi_sta"),
    (web_source, r'\"connected_ssid\"'),
    (web_source, "save_admin_password_"),
    (web_source, "RELEASE_DOWNLOAD_PREFIX"),
    (web_source, "auto_update_task_"),
    (web_source, "config.timeout_ms = 120000"),
    (web_source, "AUTO_UPDATE_MAX_REDIRECT_URL_LENGTH = 4096"),
    (web_source, "std::max<size_t>("),
    (web_source, "context.redirect_url.size() + AUTO_UPDATE_REQUEST_OVERHEAD_BYTES"),
    (web_source, "RELEASE_ASSET_REDIRECT_PREFIX"),
    (web_source, "config.disable_auto_redirect = true"),
    (web_source, 'strcasecmp(event->header_key, "Location") == 0'),
    (web_source, "context.accept_firmware_data = true"),
    (web_source, 'esp_http_client_set_header(client, "Accept", "application/octet-stream")'),
    (web_source, 'esp_http_client_set_header(client, "Connection", "close")'),
    (web_source, "esp_http_client_get_and_clear_last_tls_error"),
    (web_source, "esp_http_client_get_errno"),
    (web_source, '"download was incomplete: "'),
    (web_source, "esp_ota_set_boot_partition"),
    (web_source, "firmware_pending_validation"),
    (web_source, "ESP_OTA_IMG_PENDING_VERIFY"),
    (web_source, "mbedtls_sha256_update"),
    (web_source, 'url == "/steinel/update"'),
    (web_source, 'url == "/steinel/factory-reset"'),
    (web_source, "handle_factory_reset_"),
    (mesh, "global_preferences->reset()"),
    (header_source, "factory_reset_pending_"),
    (web_source, r'\"factory_password\"'),
    (web_source, r'\"mode_known\"'),
    (web_source, 'send_json_(request, 200, "{\\\"message\\\":\\\"Changing NightmatIQ mode\\\"}")'),
    (web_source, 'send_json_(request, 200, "{\\\"message\\\":\\\"Changing twilight threshold\\\"}")'),
    (web_source, 'send_json_(request, 200, "{\\\"message\\\":\\\"Refreshing NightmatIQ state\\\"}")'),
    (page_source, "Administrator access"),
    (page_source, "<h2>Administration"),
    (page_source, "Factory reset"),
    (page_source, '<div class="maintenance-section"><h3>Administrator access</h3>'),
    (page_source, '<div class="maintenance-section"><h3>Wi-Fi network</h3>'),
    (page_source, '<div class="maintenance-section"><h3>Factory reset</h3>'),
    (page_source, "SAVE WI-FI AND RESTART"),
    (page_source, 'placeholder="********"'),
    (page_source, "/steinel/wifi"),
    (page_source, r'!/^[\x20-\x7E]+$/.test(password)'),
    (page_source, "FACTORY RESET GATEWAY"),
    (page_source, "Type RESET to confirm"),
    (page_source, "/steinel/factory-reset"),
    (page_source, "changePassword"),
    (page_source, "/steinel/password"),
    (page_source, "CHECK FOR UPDATES"),
    (page_source, "DOWNLOAD AND INSTALL"),
    (page_source, "RELEASE_API"),
    (page_source, "poll().then(checkForUpdates)"),
    (page_source, "/steinel/update"),
    (page_source, "Factory images are only for USB installation"),
    (page_source, "watchManualUpdate"),
    (page_source, "Do not disconnect power"),
):
    if marker not in source:
        errors.append(f"missing required source marker: {marker}")

# The administration area is always visible, never folded. A factory password
# must stay obvious, so the heading carries a warning chip while it is active.
admin_start = page_source.index("<h2>Administration")
admin_after_heading = page_source[page_source.index("</h2>", admin_start) + len("</h2>"):]
if admin_after_heading.startswith("<details") or "<details id=\"adminSection\"" in page_source:
    errors.append("the administration section must not be foldable")
if 'id="adminWarn"' not in page_source:
    errors.append("the administration heading is missing the factory password warning")
if 'class="maintenance-section danger-zone"' in page_source:
    errors.append("factory reset must use the standard administration section separator")

for obsolete in ("esp32_ble_tracker", "set_scan_own_address_type"):
    if obsolete in header_source + mesh + component_source + config_source:
        errors.append(f"obsolete patched BLE tracker dependency remains: {obsolete}")

for obsolete_path in (
    root / "scripts/patch_esphome_api.py",
    root / "scripts/patch_esphome_ble_tracker.py",
):
    if obsolete_path.exists():
        errors.append(f"obsolete ESPHome patch script remains: {obsolete_path.name}")

for obsolete_path in (
    root / "scripts/02_configure_secrets.sh",
    root / "esphome/secrets.example.yaml",
):
    if obsolete_path.exists():
        errors.append(f"obsolete per-device configuration file remains: {obsolete_path.name}")

release_script = (root / "scripts/10_prepare_release.sh").read_text(encoding="utf-8")
for marker in (
    '"$BASE_NAME-factory.bin"',
    '"$BASE_NAME-ota.bin"',
    "SHA256SUMS",
):
    if marker not in release_script:
        errors.append(f"release packaging contract is missing: {marker}")

for script_path in (root / "scripts").glob("*.sh"):
    if script_path.name == "00_self_test.sh":
        continue
    if "patch_esphome" in script_path.read_text(encoding="utf-8"):
        errors.append(f"ESPHome patch hook remains in {script_path.name}")

if "\n  devices:\n" in config_source:
    errors.append("standalone C3 must expose NightmatIQ as its primary Home Assistant device")

for forbidden_gui_marker in (
    "Automatic address recovery",
    "Address pool:",
    "Initial Mesh address:",
    "Automatic / manual address changes:",
    "TRY ANOTHER MESH ADDRESS",
):
    if forbidden_gui_marker in page_source:
        errors.append(f"internal Mesh address detail leaked into GUI: {forbidden_gui_marker}")

for forbidden_source_marker in (
    'url == "/steinel/rotate-address"',
    r'body.append(",\"address_policy_valid\":")',
    "local_address_confirmed",
    "address_pool_low",
    "automatic_address_rotations",
    "manual_address_rotations",
):
    if forbidden_source_marker in web_source:
        errors.append(f"obsolete Mesh address interface remains public: {forbidden_source_marker}")

control_handlers = web_source.split("void NightmatiqMesh::handle_mode_", 1)[1].split(
    "bool NightmatiqMesh::parse_backup_", 1
)[0]
if "send_json_(request, 202" in control_handlers:
    errors.append("NightmatIQ web controls must use an HTTP status supported by web_server_idf")

nodes_source = (root / "esphome/components/steinel_mesh/steinel_nodes.cpp").read_text(encoding="utf-8")
if "send_json_(request, 202" in nodes_source:
    errors.append("node API must use an HTTP status supported by web_server_idf (200, 204, 400, 401, 404, 409, 422)")

# Multi-node contract: local backup import and the node API.
for source, marker in (
    (web_source, 'url == "/steinel/import"'),
    (web_source, 'url == "/api/nodes"'),
    (web_source, 'url.find("/api/nodes/") == 0'),
    (web_source, "handleBody"),
    (header_source, "StoredNodeTable"),
    (nodes_source, "advance_node_engine_"),
    (nodes_source, "note_node_response_"),
    (nodes_source, "NodeRequestKind::COMPOSITION_GET"),
    (nodes_source, "handle_node_composition_"),
    (nodes_source, "NodeRequestKind::SENSOR_DESCRIPTOR_GET"),
    (web_source, "firmware_version"),
    (web_source, "manufacturer_name_"),
    (mesh, "main_light_entities_published_"),
    (web_source, "mac_text"),
    (nodes_source, "NodeRequestKind::THRESHOLD_SET"),
    (web_source, r'\"threshold\":'),
    (page_source, "IMPORT BACKUP FILE"),
    (page_source, "/steinel/import"),
    (page_source, "/api/nodes"),
    (page_source, "nodeCommand(a,{threshold"),
    (page_source, "n.company_id"),
    (page_source, "st.firmware_version"),
):
    if marker not in source:
        errors.append(f"missing multi-node source marker: {marker}")

# The Steinel Cloud download was removed on purpose: setup is a local file import.
for forbidden_cloud_marker in (
    "connectapp.steinel.de",
    "discover_networks_",
    "cloud_get_",
    "start_cloud_job_",
    "CloudJob",
    'url == "/steinel/discover"',
    'url == "/steinel/install"',
):
    for label, source in (("firmware", web_source), ("header", header_source), ("mesh", mesh)):
        if forbidden_cloud_marker in source:
            errors.append(f"removed Steinel Cloud feature remains in {label}: {forbidden_cloud_marker}")
for forbidden_page_marker in ("/steinel/discover", "/steinel/install", "DOWNLOAD NETWORK LIST"):
    if forbidden_page_marker in page_source:
        errors.append(f"removed Steinel Cloud feature remains in GUI: {forbidden_page_marker}")

if errors:
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)
    raise SystemExit(1)
print("OK: Python syntax and embedded NightmatIQ page")
PY
then
  ok 'NightmatIQ source checks'
else
  fail 'NightmatIQ source checks failed'
fi

if (( FAIL > 0 )); then
  echo "Self-test failed: $FAIL error(s)." >&2
  exit 1
fi

echo 'All standalone project self-tests passed.'

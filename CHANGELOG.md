# Changelog

All notable changes to this project are documented in this file.

## Unreleased

Generic multi-node support (experimental). Verified on a real network with an IS 180, two L 810 SC and an L 810 C; motion detection values are still unconfirmed.

- Network setup is now a local `.json` backup import. The Steinel Cloud download and its login form were removed.
- All controllable nodes of the backup are stored and restored, not only the NightmatIQ Plus.
- `/api/nodes` also reports the gateway's MAC address so Home Assistant can link the mesh devices to the gateway's ESPHome device.
- `GET /steinel/nodes` lists the stored nodes; `GET /api/nodes` adds live state (on/off, brightness, automatic mode, sensor readings).
- `POST /api/nodes/<address>` with `on`, `brightness` (0-100), `auto` and `threshold` (twilight threshold, 1-1500 lx) controls a lamp.
- New Home Assistant integration `steinel_mesh` (maintained in [HomeAssistant-Steinel-Mesh](https://github.com/CFenner/HomeAssistant-Steinel-Mesh)) creates lights, automatic-mode switches, illuminance and presence sensors for every node.
- Bluetooth Mesh limits raised (12 nodes, replay protection list of 16) and a Light Lightness client added.

## 1.1.1 — 2026-08-27

- Wi-Fi credentials can now be changed from the local gateway administration page.
- The configured SSID is displayed without exposing the saved Wi-Fi password.
- The standard ESPHome captive portal starts after 60 seconds when the configured Wi-Fi network is unavailable.
- The fallback access point uses channel 6 for predictable discovery and connection.

## 1.1.0 — 2026-08-26

- One universal firmware image for every supported ESP32-C3 board.
- First-run Wi-Fi provisioning through a password-protected access point and captive portal.
- Unique device and access-point names derived from the ESP32-C3 MAC address.
- Persistent administrator password managed from the local page and shared with firmware updates.
- Build, validation and upload scripts no longer require `secrets.yaml`.
- Clean, pinned ESPHome 2026.7.3 build environment without modifications to the installed toolchain.
- Automatic update checks against the latest stable GitHub release when the gateway page opens.
- Dedicated Mesh-free HTTPS update mode with image-size and SHA-256 verification.
- Reproducible release packaging for factory, OTA and checksum files.
- Unified gateway administration panel for firmware updates, directly visible administrator access and factory reset.
- Confirmed factory reset that clears Wi-Fi, administrator and Bluetooth Mesh settings without changing firmware.
- Automatic Mesh address recovery now requires a NightmatIQ advertisement detected during the current boot, preventing address rotation and restart while the sensor is offline or out of range.
- Ready-made factory-image installation instructions and complete English, Polish and German documentation.

## 1.0.0 — 2026-08-24

- Initial stable standalone release for ESP32-C3 Super Mini.
- Local Steinel NightmatIQ Plus control through Bluetooth Mesh.
- Browser-assisted import of the Steinel network configuration.
- Automatic gateway Mesh address selection, recovery and confirmation.
- Local bilingual web interface with control, diagnostics and firmware updates.
- Native Home Assistant integration through ESPHome.
- Optional bilingual Home Assistant area tile and compact control dialog.
- USB, OTA, validation and secrets-configuration scripts.
- Password-protected fallback access point and captive portal.

# Changelog

All notable changes to this project are documented in this file.

## Unreleased

Generic multi-node support (experimental). Verified on a real network with an IS 180, two L 810 SC and an L 810 C; motion detection values are still unconfirmed.

- Removed the optional Home Assistant area tile and compact control dialog (`home-assistant/`); the Home Assistant integration replaces them.
- The gateway's diagnostics from its web page are also ESPHome entities: uptime, reset reason, free heap, largest free block and the Bluetooth Mesh traffic counters.
- The device name no longer gets a MAC suffix: the hostname and setup access point are `steinel-mesh-gateway`, and the Home Assistant device is "Steinel Mesh Gateway". With several gateways, set a unique `name` for each.
- The ESPHome device is now called "Steinel Mesh Gateway" (project `cfenner.steinel_mesh_gateway`) and its entities have short names such as "Mesh Ready" and "Status"; the entities describing the primary device start with "Primary Device". Because ESPHome derives an entity's ID from its name, Home Assistant creates new entities and leaves the old ones orphaned: delete them after updating. The hostname is unchanged.
- Network setup is now a local `.json` backup import. The Steinel Cloud download and its login form were removed.
- All controllable nodes of the backup are stored and restored, not only the NightmatIQ Plus.
- A brightness reading of 0xFFFFFF, which Bluetooth Mesh uses for "not known", is reported as `"lux":null` instead of 167772.15 lx.
- The time a light stays on after motion (Light Control property Time Run On, `0x003C`) is read from every lamp and shown on the page as "On-time after motion" (seconds); `GET /api/nodes` reports it as `run_on` (`null` while unknown) and `POST /api/nodes/<address>` accepts `run_on` (1-3600 s). A lamp that has not answered is asked a few times, then only now and then, and a missing answer is not held against it.
- The gateway subscribes to the groups the sensors in the backup publish their readings to, and receives those readings (brightness every 10 s, and motion of sensors that publish to a group) without asking. The groups are read from the backup, so import it again once after updating; until then the gateway logs that no sensor groups are stored and keeps polling as before.
- The device the gateway was set up with (the IS 180 "Laterne") is restored with its real element count, from the imported backup, instead of a fixed three. Its brightness sensor on the fourth element was out of the range the Bluetooth Mesh stack allows to send to, so it never answered.
- A sensor element that never answers the plain Sensor Get (the IS 180's ambient light sensor on element 3) is now also asked for the ambient light level explicitly and for its Sensor Descriptor, a few times after start-up; the results are logged.
- `/api/nodes` reports each device's manufacturer, company ID and product ID, and its firmware version, read live from the device's composition data. Steinel packs the version into the composition version ID (5 bits major, 5 bits minor, 6 bits patch, e.g. `0x0883` is 1.2.3); this matches the versions shown in the Steinel app for the IS 180 (1.2.3), L 810 SC (1.1.1) and L 810 C (1.1.1).
- `/api/nodes` also reports the gateway's MAC address so Home Assistant can link the mesh devices to the gateway's ESPHome device.
- `GET /api/nodes` lists the stored nodes with their live state (on/off, brightness, automatic mode, sensor readings, firmware version).
- `POST /api/nodes/<address>` with `on`, `brightness` (0-100), `auto` and `threshold` (twilight threshold, 1-1500 lx) controls a lamp.
- New Home Assistant integration `steinel_mesh` (maintained in [HomeAssistant-Steinel-Mesh](https://github.com/CFenner/HomeAssistant-Steinel-Mesh)) creates lights, automatic-mode switches, illuminance and presence sensors for every node.
- The single-device NightmatIQ code is removed: the gateway no longer has the primary device's light output, illuminance, twilight threshold and mode entities, their polling and control, the Bluetooth identity scan, and the `/steinel/mode` and `/steinel/threshold` requests. The node engine handles every device, including the one the gateway was set up with. The component options `lux_sensor_id`, `threshold_number_id`, `mode_select_id`, `actual_output_binary_sensor_id`, the optional identity text sensors and the `update_interval` are gone, and the status JSON no longer carries the primary device's state fields. The Mesh now starts as soon as Bluetooth is ready.
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

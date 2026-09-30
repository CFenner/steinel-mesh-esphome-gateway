# Steinel Mesh Gateway (Home Assistant custom integration)

Exposes every node of the Steinel Bluetooth Mesh network that the ESP32-C3
gateway has imported: lamps, sensor lamps and motion sensors.

**Status:** experimental. Verified on a real network (IS 180, two L 810 SC and
one L 810 C): setup, reading state, switching lamps, automatic mode and
illuminance work. A motion detection (non-zero motion value) has not been
observed yet, so the motion decoding is unconfirmed.

## Install

1. Copy the `steinel_mesh_gateway` folder into `<config>/custom_components/`.
2. Restart Home Assistant.
3. Add the integration under *Settings → Devices & services*. Enter the
   gateway's address and its web login (username `admin`, and the administrator
   password from the gateway page).

## Entities

| Entity | For nodes with | Notes |
|---|---|---|
| Light | a light output | on/off and brightness |
| Automatic mode (switch) | Light Control | on: the lamp follows its own sensors; off: manual. A manual on/off or brightness command switches automatic mode off first. |
| Illuminance | an illuminance sensor | in lx |
| Presence / Motion | a presence or motion property | decoding of these properties is unverified until seen on real devices |
| Mesh connection | every node | diagnostic; off when the gateway gets no answers |
| Sensor `0x....` | any other sensor property | diagnostic, disabled by default; shows the raw bytes |

## Gateway API

Both endpoints use the gateway's HTTP Digest login.

- `GET /api/nodes` returns the nodes with their models and live state.
- `POST /api/nodes/<address>?on=1&brightness=40&auto=0` queues a command (HTTP 200).
  `on` and `auto` accept `0`/`1`; `brightness` is 0–100. The request needs a
  `Content-Length` header, so `curl -X POST --digest -u admin ... -d ''` works.
  Automatic mode (`auto=1`) cannot be combined with `on` or `brightness`.

Nodes are read every ~20 seconds, one request at a time, so state can lag a
command by a few seconds.

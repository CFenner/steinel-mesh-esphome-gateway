import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    binary_sensor,
    esp32_ble,
    sensor,
    text_sensor,
    web_server_base,
)
from esphome.components.esphome import ota as esphome_ota
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
from esphome.const import CONF_ID

CODEOWNERS = []
DEPENDENCIES = ["esp32", "web_server"]
AUTO_LOAD = ["binary_sensor", "esp32_ble", "sensor", "text_sensor", "web_server_base"]

CONF_RSSI_SENSOR_ID = "rssi_sensor_id"
CONF_READY_BINARY_SENSOR_ID = "ready_binary_sensor_id"
CONF_STATUS_TEXT_SENSOR_ID = "status_text_sensor_id"
CONF_EXTENDED_DIAGNOSTICS = "extended_diagnostics"
CONF_WEB_USERNAME = "web_username"
CONF_WEB_PASSWORD = "web_password"
CONF_OTA_ID = "ota_id"

steinel_ns = cg.esphome_ns.namespace("steinel_mesh")
NightmatiqMesh = steinel_ns.class_(
    "NightmatiqMesh", cg.Component
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NightmatiqMesh),
        cv.GenerateID(esp32_ble.CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
        cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(web_server_base.WebServerBase),
        cv.Required(CONF_OTA_ID): cv.use_id(esphome_ota.ESPHomeOTAComponent),
        cv.Required(CONF_RSSI_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Required(CONF_READY_BINARY_SENSOR_ID): cv.use_id(binary_sensor.BinarySensor),
        cv.Required(CONF_STATUS_TEXT_SENSOR_ID): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_EXTENDED_DIAGNOSTICS, default=True): cv.boolean,
        cv.Required(CONF_WEB_USERNAME): cv.string_strict,
        cv.Required(CONF_WEB_PASSWORD): cv.sensitive(cv.string_strict),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    if config[CONF_EXTENDED_DIAGNOSTICS]:
        cg.add_define("USE_NIGHTMATIQ_EXTENDED_DIAGNOSTICS")

    base = await cg.get_variable(config[CONF_WEB_SERVER_BASE_ID])
    ota = await cg.get_variable(config[CONF_OTA_ID])
    var = cg.new_Pvariable(config[CONF_ID], base, ota)
    await cg.register_component(var, config)

    rssi = await cg.get_variable(config[CONF_RSSI_SENSOR_ID])
    ready = await cg.get_variable(config[CONF_READY_BINARY_SENSOR_ID])
    status = await cg.get_variable(config[CONF_STATUS_TEXT_SENSOR_ID])
    cg.add(var.set_rssi_sensor(rssi))
    cg.add(var.set_ready_binary_sensor(ready))
    cg.add(var.set_status_text_sensor(status))
    cg.add(var.set_web_credentials(config[CONF_WEB_USERNAME], config[CONF_WEB_PASSWORD]))

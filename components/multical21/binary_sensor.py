import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_MOISTURE, DEVICE_CLASS_PROBLEM

from . import CONF_MULTICAL21_ID, Multical21Component

DEPENDENCIES = ["multical21"]

CONF_LEAK = "leak"
CONF_BURST = "burst"
CONF_DRY = "dry"
CONF_REVERSE = "reverse"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MULTICAL21_ID): cv.use_id(Multical21Component),
        cv.Optional(CONF_LEAK): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOISTURE,
            icon="mdi:water-alert",
        ),
        cv.Optional(CONF_BURST): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:pipe-leak",
        ),
        cv.Optional(CONF_DRY): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:water-off",
        ),
        cv.Optional(CONF_REVERSE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:swap-horizontal",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MULTICAL21_ID])
    for key in (CONF_LEAK, CONF_BURST, CONF_DRY, CONF_REVERSE):
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(parent, f"set_{key}_binary_sensor")(sens))

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_STATUS, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_MULTICAL21_ID, Multical21Component

DEPENDENCIES = ["multical21"]

CONF_ALARM_HISTORY = "alarm_history"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MULTICAL21_ID): cv.use_id(Multical21Component),
        cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
            icon="mdi:water-check",
        ),
        cv.Optional(CONF_ALARM_HISTORY): text_sensor.text_sensor_schema(
            icon="mdi:history",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MULTICAL21_ID])
    for key in (CONF_STATUS, CONF_ALARM_HISTORY):
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(parent, f"set_{key}_text_sensor")(sens))

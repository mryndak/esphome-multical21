import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WATER,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_DECIBEL_MILLIWATT,
)

from . import CONF_MULTICAL21_ID, Multical21Component

DEPENDENCIES = ["multical21"]

UNIT_CUBIC_METER = "m³"

CONF_TOTAL_VOLUME = "total_volume"
CONF_TARGET_VOLUME = "target_volume"
CONF_FLOW_TEMPERATURE = "flow_temperature"
CONF_AMBIENT_TEMPERATURE = "ambient_temperature"
CONF_RSSI = "rssi"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MULTICAL21_ID): cv.use_id(Multical21Component),
        cv.Optional(CONF_TOTAL_VOLUME): sensor.sensor_schema(
            unit_of_measurement=UNIT_CUBIC_METER,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_WATER,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:water",
        ),
        cv.Optional(CONF_TARGET_VOLUME): sensor.sensor_schema(
            unit_of_measurement=UNIT_CUBIC_METER,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_WATER,
            icon="mdi:counter",
        ),
        cv.Optional(CONF_FLOW_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_AMBIENT_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RSSI): sensor.sensor_schema(
            unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MULTICAL21_ID])
    for key in (
        CONF_TOTAL_VOLUME,
        CONF_TARGET_VOLUME,
        CONF_FLOW_TEMPERATURE,
        CONF_AMBIENT_TEMPERATURE,
        CONF_RSSI,
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(parent, f"set_{key}_sensor")(sens))

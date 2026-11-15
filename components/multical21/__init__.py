import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import CONF_FREQUENCY, CONF_ID

CODEOWNERS = ["@mryndak"]
DEPENDENCIES = ["spi"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

CONF_MULTICAL21_ID = "multical21_id"
CONF_METER_ID = "meter_id"
CONF_KEY = "key"

multical21_ns = cg.esphome_ns.namespace("multical21")
Multical21Component = multical21_ns.class_(
    "Multical21Component", cg.Component, spi.SPIDevice
)


def validate_meter_id(value):
    """Meter id as printed on the meter, e.g. "12345678" or 0x12345678.

    The wM-Bus address is BCD, so the printed serial number is read as hex.
    """
    if isinstance(value, int):
        if not 0 <= value <= 0xFFFFFFFF:
            raise cv.Invalid("meter_id must fit in 32 bits")
        return value
    text = cv.string(value).strip().lower()
    if text.startswith("0x"):
        text = text[2:]
    if not re.fullmatch(r"[0-9a-f]{1,8}", text):
        raise cv.Invalid(
            "meter_id must be the 8 digit serial number of the meter, e.g. \"12345678\""
        )
    return int(text, 16)


def validate_key(value):
    text = re.sub(r"[\s:-]", "", cv.string(value)).upper()
    if not re.fullmatch(r"[0-9A-F]{32}", text):
        raise cv.Invalid("key must be 32 hex characters (16 bytes AES key)")
    return text


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Multical21Component),
            cv.Required(CONF_METER_ID): validate_meter_id,
            cv.Required(CONF_KEY): validate_key,
            cv.Optional(CONF_FREQUENCY, default="868.95MHz"): cv.All(
                cv.frequency, cv.Range(min=860.0e6, max=870.0e6)
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True)),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    cg.add(var.set_meter_id(config[CONF_METER_ID]))
    cg.add(var.set_key(config[CONF_KEY]))
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))

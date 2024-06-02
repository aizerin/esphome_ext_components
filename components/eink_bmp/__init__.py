import logging
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
)

_LOGGER = logging.getLogger(__name__)

MULTI_CONF = False

eink_bmp_ns = cg.esphome_ns.namespace('eink_bmp')
EinkBmp = eink_bmp_ns.class_('EinkBmp',cg.PollingComponent)
# EinkBmp = eink_bmp_ns.class_('EinkBmp', cg.Component)


DEPENDENCIES = ["network"]

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(EinkBmp),
    })
    .extend(cv.polling_component_schema("60s"))
    .extend(cv.COMPONENT_SCHEMA)
)


# CONFIG_SCHEMA = (
#     cv.COMPONENT_SCHEMA
# )

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add_library("WiFi", None)
    cg.add_library("Wire", None)
    cg.add_library("SPI", None)
    cg.add_library("GxEPD2", None)
    cg.add_library("Adafruit BusIO", None)
    cg.add_library("Adafruit GFX Library", None)

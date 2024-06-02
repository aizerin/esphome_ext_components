import logging
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import (
    CONF_ID,
)

_LOGGER = logging.getLogger(__name__)

MULTI_CONF = False

spi_test_ns = cg.esphome_ns.namespace('spi_test')
SpiTest = spi_test_ns.class_('SpiTest', cg.PollingComponent, spi.SPIDevice)

DEPENDENCIES = ["network", "spi"]

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(SpiTest),
    })
    .extend(cv.polling_component_schema("60s"))
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(False, "4MHz"))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    
    cg.add_library("WiFi", None)
    cg.add_library("Wire", None)
    cg.add_library("SPI", None)
    cg.add_library("GxEPD2", None)
    cg.add_library("Adafruit BusIO", None)
    cg.add_library("Adafruit GFX Library", None)

import logging
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
)

MULTI_CONF = False

dummy_ns = cg.esphome_ns.namespace('dummy')
Dummy = dummy_ns.class_('Dummy', cg.Component)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(Dummy),
    })
    # .extend(cv.polling_component_schema("60s"))
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
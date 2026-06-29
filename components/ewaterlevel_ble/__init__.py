import esphome.config_validation as cv

from ._shared import (
    EWaterlevel_schema,
    EWaterlevel_to_code,
    validate_value_range,
)

CODEOWNERS = ["@Fabian-Schmidt"]

AUTO_LOAD = ["sensor"]
DEPENDENCIES = ["esp32_ble_tracker"]

CONFIG_SCHEMA = cv.All(EWaterlevel_schema, validate_value_range)


async def to_code(config):
    await EWaterlevel_to_code(config)

"""Module containing the configuration files of the interactive scenarios"""
from enum import Enum

from .cr_sumo_config_base import CRSumoConfigBase

from .cr_sumo_config_1 import CRSumoConfig_1
# TODO: 1. Import more configurations here


class CONFIG_TYPE(Enum):
    """Definition of the configuration types"""
    SUMO_CONFIG_1 = 1
    # TODO: 2. Define more config types here


__CONFIG_MAP = {
    CONFIG_TYPE.SUMO_CONFIG_1: CRSumoConfig_1
    # TODO: 3. Define more config types here
}


def get_interactive_scenario_configuration(config_type: CONFIG_TYPE, scenario_name: str):
    """
    Gets the configuration of the interactive scenario given the config type and the name of the scenario
    :param config_type: The type of the condiguration
    :param scenario_name: The name of the scenario
    :return: The confiruation instance
    """
    return __CONFIG_MAP[config_type].from_scenario_name(scenario_name)

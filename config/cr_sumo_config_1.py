"""
Default configuration for CommonRoad to SUMO map converter
"""
from commonroad.scenario.scenario import ScenarioID
from sumocr.sumo_config.default import DefaultConfig

from .cr_sumo_config_base import CRSumoConfigBase


class CRSumoConfig_1(CRSumoConfigBase):

    def __init__(self):
        # logging level for logging module
        super().__init__()
        self.__class__ = DefaultConfig
        self.config_id = 1

        self.logging_level = 'INFO'  # select DEBUG, INFO, WARNING, ERROR, CRITICAL

        self.scenario_name: str = ''

        # simulation
        self.dt = 0.1  # length of simulation step of the interface
        self.delta_steps = 1  # number of sub-steps simulated in SUMO during every dt
        self.presimulation_steps = 2  # number of time steps before simulation with ego vehicle starts
        self.simulation_steps = 200  # number of simulated (and synchronized) time steps
        self.with_sumo_gui = False
        # lateral resolution > 0 enables SUMO'S sublane model, see https://sumo.dlr.de/docs/Simulation/SublaneModel.html
        self.lateral_resolution = 0
        # re-compute orientation when fetching vehicles from SUMO.
        # Avoids lateral "sliding" at lane changes at computational costs
        self.compute_orientation = True

        # [m/s] if not None: use this speed limit instead of speed limit from CommonRoad files
        self.overwrite_speed_limit = 130 / 3.6
        # [m/s] default max. speed for SUMO for unrestricted sped limits
        self.unrestricted_max_speed_default = 120 / 3.6
        # [m] shifted waiting position at junction (equivalent to SUMO's contPos parameter)
        self.wait_pos_internal_junctions = -4.0
        # [m/s] default speed limit when no speed_limit is given
        self.unrestricted_speed_limit_default = 130 / 3.6

        # ego vehicle
        self.ego_start_time: int = 0

        # ego vehicle sync parameters
        #
        # Time window to detect the lanelet change in seconds
        self.lanelet_check_time_window = int(2 / self.dt)
        # The absolute margin allowed between the planner position and ego position in SUMO
        self.protection_margin = 2.0
        # Variable can be used  to force the consistency to certain number of steps
        self.consistency_window = 4
        # Used to limit the sync mechanism only to move xy
        self.lane_change_sync = False
        # tolerance for detecting start of lane change
        self.lane_change_tol = 0.00
        self.ego_veh_width = 1.6
        self.ego_veh_length = 4.3

        # TRAFFIC GENERATION
        #
        # max. number of vehicles per km
        self.max_veh_per_km: int = 70
        # random seed for deterministic sumo traffic generation
        self.random_seed: int = 1234

        # vehicle attributes
        self.veh_params = {
            # length
            'length': {
                'passenger': 5.0,
                'truck': 7.5,
                'bus': 12.4,
                'motorcycle': 2.5,
                'bicycle': 2.,
                'pedestrian': 0.415
            },
            # width
            'width': {
                'passenger': 2.0,
                'truck': 2.6,
                'bus': 2.7,
                'motorcycle': 0.8,
                'bicycle': 0.68,
                'pedestrian': 0.678
            },
            'minGap': {
                'passenger': 1.0,
                'truck': 2.5,
                'bus': 2.5,
                'motorcycle': 2.5,
                # default 0.5
                'bicycle': 1.,
                'pedestrian': 0.25
            },
            'accel': {
                # default 2.9 m/s²
                'passenger': 2.9,
                # default 1.3
                'truck': 1.3,
                # default 1.2
                'bus': 1.2,
                # default 2.5
                'motorcycle': 2.5,
                # default 1.2
                'bicycle': 1.2,
                # default 1.5
                'pedestrian': 1.5,
            },
            'decel': {
                # default 7.5 m/s²
                'passenger': 7.5,
                # default 4
                'truck': 4,
                # default 4
                'bus': 4,
                # default 6
                'motorcycle': 6,
                # default 3
                'bicycle': 3,
                # default 2
                'pedestrian': 2,
            },
            'maxSpeed': {
                # default 180/3.6 m/s
                'passenger': 180 / 3.6,
                # default 130/3.6
                'truck': 130 / 3.6,
                # default 85/3.6
                'bus': 85 / 3.6,
                # default 130/3.6
                'motorcycle': 130 / 3.6,
                # default 85/3.6
                'bicycle': 25 / 3.6,
                # default 5.4/3.6
                'pedestrian': 5.4 / 3.6,
            }
        }

        # vehicle behavior
        """
        'minGap': minimum gap between vehicles
        'accel': maximum acceleration allowed
        'decel': maximum deceleration allowed (absolute value)
        'maxSpeed': maximum speed. sumo_default 55.55 m/s (200 km/h)
        'lcStrategic': eagerness for performing strategic lane changing. Higher values result in earlier lane-changing. sumo_default: 1.0
        'lcSpeedGain': eagerness for performing lane changing to gain speed. Higher values result in more lane-changing. sumo_default: 1.0
        'lcCooperative': willingness for performing cooperative lane changing. Lower values result in reduced cooperation. sumo_default: 1.0
        'sigma': [0-1] driver imperfection (0 denotes perfect driving. sumo_default: 0.5
        'speedDev': [0-1] deviation of the speedFactor. sumo_default 0.1
        'speedFactor': [0-1] The vehicles expected multiplicator for lane speed limits. sumo_default 1.0
        """
        self.driving_params = {
            'lcStrategic': 10,
            'lcSpeedGain': 3,
            'lcCooperative': 1,
            'sigma': 0.5,
            'speedDev': 0.1,
            'speedFactor': 0.9,
            'lcImpatience': 0,
            'impatience': 0
        }

    @classmethod
    def from_scenario_name(cls, scenario_name: str):
        """Initialize the config with a scenario name"""
        obj = cls()
        obj.scenario_name = scenario_name + f"-{obj.config_id}"
        return obj

    @classmethod
    def from_dict(cls, param_dict: dict):
        """Initialize config from dictionary"""
        obj = cls()
        for param, value in param_dict.items():
            if hasattr(obj, param):
                setattr(obj, param, value)
        return obj

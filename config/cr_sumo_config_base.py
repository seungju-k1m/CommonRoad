"""
Default configuration for CommonRoad to SUMO map converter
"""
from sumocr.sumo_config.default import DefaultConfig


class CRSumoConfigBase(DefaultConfig):

    def __init__(self):
        # logging level for logging module
        self.logging_level = None  # select DEBUG, INFO, WARNING, ERROR, CRITICAL

        self.scenario_name = None

        self.country_id = None

        # simulation
        self.dt = None  # length of simulation step of the interface
        self.delta_steps = None  # number of sub-steps simulated in SUMO during every dt
        self.presimulation_steps = None  # number of time steps before simulation with ego vehicle starts
        self.simulation_steps = None  # number of simulated (and synchronized) time steps
        self.with_sumo_gui = None
        # lateral resolution > 0 enables SUMO'S sublane model, see https://sumo.dlr.de/docs/Simulation/SublaneModel.html
        self.lateral_resolution = None
        # re-compute orientation when fetching vehicles from SUMO.
        # Avoids lateral "sliding" at lane changes at computational costs
        self.compute_orientation = None

        # [m/s] if not None: use this speed limit instead of speed limit from CommonRoad files
        self.overwrite_speed_limit = None
        # [m/s] default max. speed for SUMO for unrestricted sped limits
        self.unrestricted_max_speed_default = None
        # [m] shifted waiting position at junction (equivalent to SUMO's contPos parameter)
        self.wait_pos_internal_junctions = None
        # [m/s] default speed limit when no speed_limit is given
        self.unrestricted_speed_limit_default = None

        # ego vehicle
        self.ego_start_time = None

        ##
        # Time window to detect the lanelet change in seconds
        self.lanelet_check_time_window = None
        # The absolute margin allowed between the planner position and ego position in SUMO
        self.protection_margin = None
        # Variable can be used  to force the consistency to certain number of steps
        self.consistency_window = None
        # Used to limit the sync mechanism only to move xy
        self.lane_change_sync = None
        # tolerance for detecting start of lane change
        self.lane_change_tol = None

        ##
        ## TRAFFIC GENERATION
        ##
        # max. number of vehicles per km
        self.max_veh_per_km = None
        # random seed for deterministic sumo traffic generation
        self.random_seed = None

        # vehicle attributes
        self.veh_params = None

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
        self.driving_params = None

    @classmethod
    def from_scenario_name(cls, scenario_name: str):
        """Initialize the config with a scenario name"""
        obj = cls()
        obj.scenario_name = scenario_name
        return obj

    @classmethod
    def from_dict(cls, param_dict: dict):
        """Initialize config from dictionary"""
        obj = cls()
        for param, value in param_dict.items():
            if hasattr(obj, param):
                setattr(obj, param, value)
        return obj

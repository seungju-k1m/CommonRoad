from commonroad.common.util import Interval
from sumocr.sumo_config.default import DefaultConfig
from typing import List


class SumoConf(DefaultConfig):
    logging_level = 'INFO'  # select DEBUG, INFO, WARNING, ERROR, CRITICAL

    # simulation
    dt = 0.1  # length of simulation step
    delta_steps = 1  # number of substeps simulated in SUMO during every dt
    presimulation_steps = 30
    simulation_steps = 200
    # lateral resolution > 0 enables SUMO'S sublane model, see https://sumo.dlr.de/docs/Simulation/SublaneModel.html
    # example: lateral_resolution=0.5 divides a 3m wide lane into 6 lateral segments
    lateral_resolution = 0.5

    # plotting
    video_start = 1
    video_end = simulation_steps
    scenario_name = ''

    # adjust speed limits
    overwrite_speed_limit = 120  # [m/s]

    # ego vehicle
    ego_id = 4444
    n_ego_vehicles: int = 0
    ego_ids: List[int] = []
    ego_start_time: int = 10
    departure_time_ego = 3

    # sumo traffic generation
    fringe_factor: int = 1000000000
    veh_per_second = 50
    departure_interval_vehicles = Interval(0, 30)
    n_vehicles_max: int = 30
    max_veh_per_km: int = 50

    # other vehicles size bound (values are sampled from normal distribution within bounds)
    vehicle_length_relative_variance = 0.15
    vehicle_width_relative_variance = 0.2

    # probability distribution of different vehicle classes. Do not need to sum up to 1.
    veh_distribution = {
        'passenger': 4,
        'truck': 0.4,
        'bus': 0.15,
        'bicycle': 0.15,
        'pedestrian': 0,
        'motorcycle': 0.2
    }

    # vehicle attributes
    veh_params = {
        # maximum length
        'length': {
            'passenger': 5.0,
            'truck': 7.5,
            'bus': 12.4,
            'bicycle': 2.,
            'pedestrian': 0.415,
            'motorcycle': 2.5,
        },
        # maximum width
        'width': {
            'passenger': 2.0,
            'truck': 2.6,
            'bus': 2.7,
            'bicycle': 0.68,
            'pedestrian': 0.678,
            'motorcycle': 0.8,
        },
        'minGap': {
            'passenger': 2.5,
            'truck': 2.5,
            'bus': 2.5,
            'bicycle': 1.,  # default 0.5
            'pedestrian': 0.25,
            'motorcycle': 2.5,
        },
        'accel': {
            'passenger': Interval(2, 2.9),  # default 2.9 m/s²
            'truck': Interval(1, 1.5),  # default 1.3
            'bus': Interval(1, 1.4),  # default 1.2
            'bicycle': Interval(1, 1.4),  # default 1.2
            'pedestrian': Interval(1.3, 1.7),  # default 1.5
            'motorcycle': Interval(2, 3.5),  # default 1.5
        },
        'decel': {
            'passenger': Interval(4, 6.5),  # default 7.5 m/s²
            'truck': Interval(3, 4.5),  # default 4
            'bus': Interval(3, 4.5),  # default 4
            'bicycle': Interval(2.5, 3.5),  # default 3
            'pedestrian': Interval(1.5, 2.5),  # default 2
            'motorcycle': Interval(4, 6.5),  # default 2
        },
        'maxSpeed': {
            'passenger': 180 / 3.6,  # default 180/3.6 m/s
            'truck': 130 / 3.6,  # default 130/3.6
            'bus': 85 / 3.6,  # default 85/3.6
            'bicycle': 25 / 3.6,  # default 85/3.6
            'pedestrian': 5.4 / 3.6,  # default 5.4/3.6
            'motorcycle': 80 / 3.6,  # default 80/3.6
        },
        'latAlignment': {
            'passenger': 'center',
            'truck': 'center',
            'bus': 'center',
            'bicycle': 'right',
            'pedestrian': 'right',
            'motorcycle': 'center',
        },
        'lcAccelLat': {
            'passenger': 1.5,
            'truck': 0.8,
            'bus': 0.8,
            'bicycle': 1.5,
            'pedestrian': 0.5,
            'motorcycle': 0.5,
        }
    }

    # VEHICLE BEHAVIOR
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
    driving_params = {'lcStrategic': Interval(2, 10),
                      'lcSpeedGain': Interval(3, 20),
                      'lcCooperative': Interval(0.8, 1),
                      'sigma': Interval(0.55, 0.7),
                      'speedDev': Interval(0.1, 0.2),
                      'speedFactor': Interval(0.9, 1.1)}
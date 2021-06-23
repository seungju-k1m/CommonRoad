"""
SUMO simulation
"""

from sumocr.maps.scenario_wrapper import AbstractScenarioWrapper
from sumocr.interface.sumo_simulation import SumoSimulation
from sumocr.interface.ego_vehicle import EgoVehicle
from sumocr.sumo_config import EGO_ID_START

from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet, GoalRegion
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.util import Interval, AngleInterval
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.geometry.shape import Rectangle

from sumoSimulation.cfg import simulationCfg
from config.sumo_config import SumoConf
from typing import List

import numpy as np

import pickle
import time
import copy
import os


class vehicleSpec:
    def __init__(self, cfg: dict):
        self.wid: float = cfg['wid']
        self.length: float = cfg['length']
        self.departSpeed: str = str(cfg['departSpeed'])
        self.pos: list = cfg['pos']


class Simulator:
    def __init__(
        self,
        cfg: simulationCfg
    ):
        # custom configuration for our case.
        self._cfg = cfg
        path = os.path.join(self._cfg.base_path, self._cfg.scenario_name)

        # sumocr.conf.
        self.conf = self.load_sumo_configuration(path)
        self.conf.with_sumo_gui = self._cfg.gui
        self.conf.ego_veh_width = self._cfg.wid
        self.conf.ego_veh_length = self._cfg.length

        # scenario is a kind of data format in CommonRoad.
        # only use lanelet_network data.
        scenario_file = os.path.join(
            path, self._cfg.scenario_name+".cr.xml"
        )
        scenario, planningP = CommonRoadFileReader(scenario_file).open()
        self.problem = planningP
        # scenario_wrapper consists of two elements: sumo_cfg_file and lanelet network
        scenario_wrapper_path = os.path.join(
            path, self._cfg.scenario_name+".sumo.cfg"
        )
        lanelet_network = scenario.lanelet_network
        self.scenario_wrapper = self.load_scenario_wrapper(
            scenario_wrapper_path, lanelet_network)

        self.vehicleSpecs: List[vehicleSpec] = [vehicleSpec(
            self._cfg.ego_vehs[key]) for key in self._cfg.ego_vehs.keys()]

    @staticmethod
    def load_sumo_configuration(path: str) -> SumoConf:
        with open(os.path.join(path, "simulation_config.p"), "rb") as file:
            conf = pickle.load(file)
        return conf

    @staticmethod
    def load_scenario_wrapper(path, lanelet_network: LaneletNetwork) -> AbstractScenarioWrapper:
        scenario_wrapper = AbstractScenarioWrapper()
        scenario_wrapper.sumo_cfg_file = path
        scenario_wrapper.lanelet_network = lanelet_network
        return scenario_wrapper

    def init(self):
        # There are two ways to configure ego vehicles.
        # First, choose already defined vehicle from *.rou.xml.
        # Second, configure new ego vehicle.
        # We use First method.

        # TODO: how to configure ego vehicels in SumoSimulation ?
        # Ego vehicles can be defined in .rou file
        # Domain can indicate its type instances in sumo.
        """
        ---------------------SUMO-------------------------
            1. specify the route id in .rou.xml file.
            2. Add sumo_id and generic_route_id in vehicledomain
            3. moveToXY vehicle.
            4. specify the indicators
        ----------------------COMMONROAD-----------------------------
            5. add ego vehicle
        """
        sumo_sim = SumoSimulation()
        problems = []
        # goalState = self.problem.planning_problem_dict[125].goal.state_list[0]
        _goalState = State(
            position=Rectangle(1, 1),
            orientation=AngleInterval(-1, 1),
            time_step=Interval(1, 10000))
        for i in range(len(self._cfg.ego_vehs.keys())):
            spec = self.vehicleSpecs[i]
            initState = State(
                position=np.array(spec.pos[:-1]),
                orientation=spec.pos[-1],
                velocity=spec.departSpeed,
                time_step=0,
                yaw_rate=0,
                slip_angle=0
            )
            problems.append(
                PlanningProblem(i+1, initState, GoalRegion([_goalState]))
            )
        problemSet = PlanningProblemSet(problems)
        sumo_sim.initialize(self.conf, self.scenario_wrapper, problemSet)
        # sumo_sim.initialize(self.conf, self.scenario_wrapper, self.problem)
        # ego_vehicles = sumo_sim.ego_vehicles
        # for step in range(self._cfg.step):
        #     scenario = sumo_sim.commonroad_scenario_at_time_step(
        #         sumo_sim.current_time_step)
        #     info = self.wrap_scenario(scenario)
        #     ego_info = self.wrap_scenario(ego_vehicles)

        #     sumo_sim.simulate_step()
        #     time.sleep(0.01)
        return sumo_sim

    @staticmethod
    def wrap_scenario(data):
        info = {}
        states = []
        attributes = ['position', "orientation", "velocity"]
        for key in attributes:
            info[key] = []
        if type(data) is Scenario:
            obs = data.obstacles
            for ob in obs:
                states.append(ob.initial_state)
        elif type(data) is dict:
            for vehicle in data.values():
                vehicle: EgoVehicle
                states.append(
                    vehicle.current_state
                )

        for state in states:
            for key in attributes:
                info[key].append(getattr(state, key))

        return info

    def step(self, action: np.array = None):
        if action is None:
            for ego_vehicle in self.env.ego_vehicles.values():
                ego_vehicle: EgoVehicle
                state_current_ego = ego_vehicle.current_state
                state_current = copy.deepcopy(state_current_ego)
                state_current.time_step = 1
                ego_vehicle.set_planned_trajectory([state_current])

        self.env.simulate_step()

    def get_state(self):
        scenario = self.env.commonroad_scenario_at_time_step(
            self.env.current_time_step
        )
        egoVehicles = self.env.ego_vehicles
        info = self.wrap_scenario(scenario)
        ego_info = self.wrap_scenario(egoVehicles)

        return ego_info, info

    def run(self):
        self.env = self.init()

        ego_info, info = self.get_state()
        for _ in range(100):
            self.step()

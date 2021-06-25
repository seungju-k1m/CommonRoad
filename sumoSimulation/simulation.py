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
import math
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
        if self.egoMode:
            sumo_sim.initialize(self.conf, self.scenario_wrapper, problemSet)
            ids = sumo_sim.ids_cr2sumo['egoVehicle'].values()
            for id in ids:
                sumo_sim.vehicledomain.setColor(id, (255, 0, 0))
        else:
            sumo_sim.initialize(self.conf, self.scenario_wrapper, None)

        roadNetworks = sumo_sim.commonroad_scenario_at_time_step(
            sumo_sim.current_time_step).lanelet_network
        lanelets = roadNetworks.lanelets
        self.lane_id2seq = {}
        for i, lane in enumerate(lanelets):
            self.lane_id2seq[lane.lanelet_id] = i+1
        return sumo_sim, roadNetworks

    def wrap_scenario(self, data):
        info = {}
        states = []
        attributes = ['position', "orientation", "velocity"]
        list_id = []
        if type(data) is Scenario:
            obs = data.obstacles
            for ob in obs:
                states.append(ob.initial_state)
                list_id.append(ob.obstacle_id)
        elif type(data) is dict:
            for vehicle in data.values():
                vehicle: EgoVehicle
                states.append(
                    vehicle.current_state
                )
                list_id.append(vehicle.id)

        for i, id in enumerate(list_id):
            info[id] = {}
            for key in attributes:
                if key == 'orientation':
                    pos = info[id]['position']
                    local_ori = self.find_local_orientation(pos)
                    info[id][key] = getattr(states[i], key)
                    info[id][key] = info[id][key] - local_ori
                    info[id]['lane_ori'] = local_ori
                else:
                    info[id][key] = getattr(states[i], key)

        return info

    def find_which_lane(self, point: np.array) -> int:
        self.map_info: LaneletNetwork
        id = self.map_info.find_lanelet_by_position([point])
        if id == [[]]:
            return None
        return id[0][0]

    def find_local_orientation(self, point: np.array) -> float:
        id = self.find_which_lane(point)
        if id is None:
            return 0
        # info = self.load_lane_info(id)
        lanelet = self.map_info.find_lanelet_by_id(id)
        center_vertices = lanelet.center_vertices
        if point.shape != (1, 2):
            point = point.reshape(1, 2)
        distance = np.sum((point - center_vertices) ** 2, axis=1)
        ind = np.argmin(distance)
        list_ind = []
        if ind <2:
            list_ind = [0, 1, 2, 3, 4]
        elif ind > len(distance) -3:
            list_ind = [-5, -4, -3, -2, -1]
        else:
            list_ind = [ind-2, ind-1, ind, ind+1, ind+2]
        selected_vertice = center_vertices[list_ind]
        selected_vertice -= selected_vertice[0:1, :]
        m = np.sum(selected_vertice[1:, 0] * selected_vertice[1:, 1]) / np.sum(selected_vertice[1:, 0] ** 2)
        pos_neg = np.sum(selected_vertice[:, 0]) > 0
        if pos_neg:
            ori = math.atan2(m, 1)
        else:
            ori = math.atan2(-m, -1)
        return ori

    def load_lane_info(self, load_id: int) -> dict:
        lanelet = self.map_info.find_lanelet_by_id(load_id)
        left_vertices = lanelet.left_vertices
        right_vertices = lanelet.right_vertices
        ceneter_vertices = lanelet.center_vertices
        info = {}
        info['left_vertices'] = left_vertices
        info['right_vertices'] = right_vertices
        info['center_vertices'] = ceneter_vertices
        info['lane'] = self.lane_id2seq[load_id]
        return info

    def load_current_occupied_lane_info(self, point):
        id = self.find_which_lane(point)
        info = self.load_lane_info(id)
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
        if self.egoMode:
            egoVehicles = self.env.ego_vehicles
            ego_info = self.wrap_scenario(egoVehicles)
        else:
            ego_info = {}
        info = self.wrap_scenario(scenario)

        return ego_info, info

    def run(self):
        # Keys of Info : 'position', "orientation", "velocity"
        DT = 0.1
        A = 3.0
        self.egoMode = True
        self.env, self.map_info = self.init()

        for _ in range(100):
            ego_info, info = self.get_state()
            if self.egoMode:
                for key in ego_info.keys():
                    pos = ego_info[key]['position']
                    local_ori = self.find_local_orientation(pos)
                    ori = ego_info[key]['orientation']
                    velo = ego_info[key]['velocity']
                    map_info = self.load_current_occupied_lane_info(pos)

            self.step()

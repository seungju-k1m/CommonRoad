"""
SUMO simulation
"""

from sumocr.maps.scenario_wrapper import AbstractScenarioWrapper
from sumocr.interface.sumo_simulation import SumoSimulation
from sumocr.interface.ego_vehicle import EgoVehicle
# from sumocr.sumo_config import EGO_ID_START

from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet, GoalRegion
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.util import Interval, AngleInterval
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.geometry.shape import Rectangle

from sumoSimulation.cfg import simulationCfg
from config.sumo_config import SumoConf
from scipy.optimize import curve_fit
from typing import List

import matplotlib.pyplot as plt
import numpy as np

import pickle
import random
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
        self.road_curve = {}

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
        self.lane_let = {}
        for i, lane in enumerate(lanelets):
            self.lane_id2seq[lane.lanelet_id] = i+1
            self.lane_let[lane.lanelet_id] = lane.center_vertices
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
                    # local_ori = self.find_local_orientation(pos)
                    local_ori = self.find_local_orientation_(pos)
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

    def calculate_road_curve(self, id, p=3):
        def forward(x, a1, a2, a3, a4, a5):
            return a1 * x + a2 * (x ** 2) + a3 * (x**3) + a4 * x**4 + a5 * x**5

        def forward_(x, a1, a2, a3):
            return a1 * x + a2 * x**2 + a3 * x ** 3
        lanelet = self.map_info.find_lanelet_by_id(id)
        center_vertices = lanelet.center_vertices
        center_vertices_ = center_vertices - center_vertices[0:1]
        # center_vertices_ = self.interpolate_lane_info(center_vertices_)
        dim_weight = len(center_vertices)
        while dim_weight < 10:
            center_vertices_ = self.interpolate_lane_info(center_vertices_)
            dim_weight = len(center_vertices_)
        if dim_weight > 5:
            popt, pconv = curve_fit(
                forward, center_vertices_[:, 0], center_vertices_[:, 1])
        else:
            popt, pconv = curve_fit(
                forward_, center_vertices_[:, 0], center_vertices_[:, 1])
        self.road_curve[id] = popt

    @staticmethod
    def interpolate_lane_info(center_vertices):
        num_vertices = len(center_vertices)
        prior_vertice = center_vertices[:-1]
        second_vertice = center_vertices[1:]
        inter_vertice = (prior_vertice + second_vertice) / 2
        vertices = []
        for i in range(num_vertices * 2 - 1):
            if i % 2 == 0:
                vertices.append(center_vertices[int(i/2)])
            else:
                vertices.append(inter_vertice[int(i/2)])
        vertices = np.array(vertices)
        return vertices

    def find_local_orientation_(self, point: np.array) -> float:
        id = self.find_which_lane(point)
        if id is None:
            return 0
        if id in list(self.road_curve.keys()):
            popt = self.road_curve[id]
        else:
            self.calculate_road_curve(id)
            popt = self.road_curve[id]
        # a1, a2, a3, a4, a5 = popt
        lanelet = self.map_info.find_lanelet_by_id(id)
        center_vertices = lanelet.center_vertices
        x_pt, y_pt = point
        if point.shape != (1, 2):
            point = point.reshape(1, 2)
        distance = np.sum((point - center_vertices) ** 2, axis=1) ** 0.5

        def calculated_grad(x):
            if len(popt) == 5:
                a1, a2, a3, a4, a5 = popt
                return a1 + 2 * a2 * x + 3 * a3 * x**2 + 4 * a4 * x**3 + 5 * a5 * x ** 4
            else:
                a1, a2, a3 = popt
                return a1 + 2 * a2 * x + 3 * a3 * x**2
        list_ind = distance.argsort()[0]
        if list_ind == (len(distance) - 1):
            list_ind = [list_ind - 2, list_ind]
        else:
            list_ind = [list_ind, list_ind + 1]
        if 0 in list_ind:
            x_pt, y_pt = center_vertices[0]
        selected_vertice = center_vertices[list_ind]
        m1 = calculated_grad(selected_vertice[0][0] - center_vertices[0][0])
        m2 = calculated_grad(selected_vertice[1][0] - center_vertices[0][0])
        b1, b2 = selected_vertice[0]
        t1, t2 = selected_vertice[1]
        if abs(m1 - m2) < 0.001:
            # it is linear equation
            alpha = -1 * (m2 * (y_pt - b2) + x_pt - b1) / (m2 * (
                b2 - t2) + b1 - t1)
        else:
            A = (m1 - m2) * (b2 - t2)
            B = (m1 - m2) * (y_pt - b2) + m2 * (b2 - t2) + (b1 - t1)
            C = m2 * (y_pt - b2) + x_pt - b1

            alpha0 = (-B + (B**2 - 4 * A * C)**0.5)/(2 * A)
            alpha1 = (-B - (B**2 - 4 * A * C)**0.5)/(2 * A)
            result = np.array([alpha0, alpha1])
            result = result[result > 0]
            result = result[result < 1]
            if len(result) == 0:
                result = [0]
            alpha = result[0]

        m = alpha * m1 + (1 - alpha) * m2
        ori = math.atan2(m, 1)

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

    def plot_base(self):
        lane_infos = []
        for key in self.lane_id2seq.keys():
            lane_infos.append(self.map_info.find_lanelet_by_id(key))
        for info in lane_infos:
            plt.plot(info.center_vertices[:, 0], info.center_vertices[:, 1])

    def plot_info(self, info):
        pos = np.array(info['position'])
        print(info['lane_ori'])
        lane_ori = info['lane_ori']
        temp_x = math.cos(lane_ori) * 1
        temp_y = math.sin(lane_ori) * 1
        return [pos[0], pos[0] + temp_x], [pos[1], pos[1] + temp_y]

    def run(self):
        # Keys of Info : 'position', "orientation", "velocity"
        # DT = 0.1
        # A = 3.0
        self.egoMode = False
        self.env, self.map_info = self.init()
        self.step()

        for _ in range(1000):
            ego_info, info = self.get_state()
            # uncomment : check the lane orientation
            # -------------------
            if _ == 0:
                xx = random.randint(0, len(list(info.keys())) - 1)
                key = list(info.keys())[xx]
                self.plot_base()

            if key in list(info.keys()) and _ < 100:
                x_list, y_list = self.plot_info(info[key])

                plt.plot(x_list, y_list, '--b')
            elif _ > 100:
                plt.show()
                print("he")
                return None
            else:
                plt.show()
                print("hello")
                return None
            # -----------------------
            if self.egoMode:
                for key in ego_info.keys():
                    pos = ego_info[key]['position']
                    local_ori = self.find_local_orientation(pos)
                    ori = ego_info[key]['orientation']
                    velo = ego_info[key]['velocity']
                    map_info = self.load_current_occupied_lane_info(pos)

            self.step()

import math
import xml.etree.ElementTree as ET

from visualizer import Visualizer
from utils.observation import Observation
from utils.functions import detectCollision, is_point_inside_rect
from opendrive2discretenet.opendriveparser.parser import parse_opendrive
from opendrive2discretenet.network import Network

import xml.dom.minidom
from lxml import etree
import json
import re
import numpy as np
import copy


class ReplayInfo():

    def __init__(self):
        self.vehicle_traj = {}
        self.bicycle_traj = {}
        self.pedestrian_traj = {}
        self.ego_info = {
            "length": 4.924,
            "width": 1.872,
            "x": 0,
            "y": 0,
            "v": 0,
            "a": 0,
            "yaw": 0
        }
        self.light_info = {}
        self.road_info = {}
        self.test_setting = {
            "scenario_name": None,
            "scenario_type": None,
            "t": 0,
            "dt": 0.01,
            "max_t": 10,
            "goal": {
                "x": [-10, 10],
                "y": [-10, 10]
            },
            "end": -1,
            "map_type":None,
            "map_range": {
                "x": [0, 0],
                "y": [0, 0]
            }
        }

    def __str__(self):
        output = ""
        output += "*"*100
        output += f"\n<vehicle_traj>\n{self.vehicle_traj}\n" + "-"*50
        output += f"\n<bicycle_traj>\n{self.bicycle_traj}\n" + "-"*50
        output += f"\n<pedestrian_traj>\n{self.pedestrian_traj}\n" + "-"*50
        output += f"\n<ego_info>\n{self.ego_info}\n" + "-"*50
        output += f"\n<light_info>\n{self.light_info}\n" + "-"*50
        output += f"\n<road_info>\n{self.road_info}\n" + "-"*50
        output += f"\n<test_setting>\n{self.test_setting}\n"
        output += "*"*100
        return output
    
    def add_vehicle(self, id, t, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """
        该函数实现向vehicle_trajectiry中添加背景车轨迹信息的功能
        """
        if id == "ego":
            self._add_vehicle_ego(x, y, v, a, yaw, length, width)
        else:
            if id not in self.vehicle_traj.keys():
                self.vehicle_traj[id] = {}
                self.vehicle_traj[id]['shape'] = {}
            if t not in self.vehicle_traj[id].keys():
                self.vehicle_traj[id][t] = {}
            for key, value in zip(['x', 'y', 'v', 'a', 'yaw'], [x, y, v, a, yaw]):
                if value is not None:
                    self.vehicle_traj[id][t][key] = value
            for key, value in zip(['length', 'width'], [length, width]):
                if value is not None:
                    self.vehicle_traj[id]['shape'][key] = value

    def add_bicycle(self, id, t, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """
        该函数实现向bicycle_trajectiry中添加背景车轨迹信息的功能
        """
        if id not in self.bicycle_traj.keys():
            self.bicycle_traj[id] = {}
            self.bicycle_traj[id]['shape'] = {}
        if t not in self.bicycle_traj[id].keys():
            self.bicycle_traj[id][t] = {}
        for key, value in zip(['x', 'y', 'v', 'a', 'yaw'], [x, y, v, a, yaw]):
            if value is not None:
                self.bicycle_traj[id][t][key] = value
        for key, value in zip(['length', 'width'], [length, width]):
            if value is not None:
                self.bicycle_traj[id]['shape'][key] = value

    def add_pedestrian(self, id, t, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """
        该函数实现向bicycle_trajectiry中添加背景车轨迹信息的功能
        """
        if id not in self.pedestrian_traj.keys():
            self.pedestrian_traj[id] = {}
            self.pedestrian_traj[id]['shape'] = {}
        if t not in self.pedestrian_traj[id].keys():
            self.pedestrian_traj[id][t] = {}
        for key, value in zip(['x', 'y', 'v', 'a', 'yaw'], [x, y, v, a, yaw]):
            if value is not None:
                self.pedestrian_traj[id][t][key] = value
        for key, value in zip(['length', 'width'], [length, width]):
            if value is not None:
                self.pedestrian_traj[id]['shape'][key] = value

    def add_settings(self, scenario_name=None, scenario_type=None, dt=None, max_t=None, goal_x=None, goal_y=None):
        """
        该函数实现向test_setting中添加测试环境相关信息
        """
        for key, value in zip(['scenario_name', 'scenario_type', 'dt', 'max_t'],
                              [scenario_name, scenario_type, dt, max_t]):
            if value is not None:
                self.test_setting[key] = value
        for key, value in zip(['x', 'y'], [goal_x, goal_y]):
            if value is not None:
                self.test_setting['goal'][key] = value

    def _add_vehicle_ego(self, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """
        该函数实现向ego_info中增加主车信息的功能
        注意：ego_info中只含有主车当前帧的信息
        """
        for key, value in zip(['x', 'y', 'v', 'a', 'yaw', 'length', 'width'], [x, y, v, a, yaw, length, width]):
            if value is not None:
                self.ego_info[key] = value

    def _get_dt_maxt(self):
        """
        该函数实现得到最大仿真时长阈值以及采样率的功能
        """
        max_t = 0
        for i in self.vehicle_traj.keys():
            t_i = list(self.vehicle_traj[i].keys())
            max_t_i = float(t_i[-1])
            if max_t_i > max_t:
                max_t = max_t_i

        dt = np.around(float(t_i[-1]) - float(t_i[-2]), 3)
        self.add_settings(dt=dt, max_t=max_t)


class ReplayParser():
    """
    解析场景文件
    """
    def __init__(self):
        self.replay_info = ReplayInfo()

    def parse(self, scenario_info: dict) -> ReplayInfo:
        self.replay_info = ReplayInfo()
        self._parse_openscenario(scenario_info['xosc_file_path'])
        self._parse_opendrive(scenario_info['xodr_file_path'])
        if scenario_info.get('json_file_path'):
            self._parse_light_json(scenario_info['json_file_path'])
        self.replay_info.add_settings(scenario_name=scenario_info['scenarioName'], scenario_type= scenario_info['scenarioType'])
        return self.replay_info

    def _parse_light_json(self, file_dir: str) -> None:
        with open(file_dir, 'r') as read_f:
            self.replay_info.light_info = json.load(read_f)
        return

    def _parse_openscenario(self, file_dir: str):
        # 读取OpenScenario文件
        opens = xml.dom.minidom.parse(file_dir).documentElement
        open0_tree = ET.parse(file_dir)
        open0 = open0_tree.getroot()
        car_elements = open0.findall('.//Vehicle[@vehicleCategory="car"]')
        bicycle_elements = open0.findall('.//Vehicle[@vehicleCategory="bicycle"]')
        pedestrian_elements = open0.findall('.//Pedestrian[@pedestrianCategory="pedestrian"]')
        car_num = 0
        bicycle_num = 0
        pedestrian_num = 0
        # 读取车辆长度与宽度信息，录入replay_info。背景车id从1号开始
        for car_element in car_elements:
            # 获取 Dimensions 元素
            dimensions_element = car_element.find('./BoundingBox/Dimensions')
            if car_num == 0:
                self.replay_info.add_vehicle(
                    id="ego",
                    t=-1,
                    width=float(dimensions_element.get('width')),
                    length=float(dimensions_element.get('length'))
                )
                car_num += 1
            else:
                self.replay_info.add_vehicle(
                    id="car"+str(car_num),
                    t=-1,
                    width=float(dimensions_element.get('width')),
                    length=float(dimensions_element.get('length'))
                )
                car_num += 1

        for bicycle_element in bicycle_elements:
            # 获取 Dimensions 元素
            dimensions_element = bicycle_element.find('./BoundingBox/Dimensions')
            self.replay_info.add_bicycle(
                id="bicycle"+str(bicycle_num+1),
                t=-1,
                width=float(dimensions_element.get('width')),
                length=float(dimensions_element.get('length'))
            )
            bicycle_num += 1

        for pedestrian_element in pedestrian_elements:
            # 获取 Dimensions 元素
            dimensions_element = pedestrian_element.find('./BoundingBox/Dimensions')
            self.replay_info.add_pedestrian(
                id="pedestrian"+str(pedestrian_num+1),
                t=-1,
                width=float(dimensions_element.get('width')),
                length=float(dimensions_element.get('length'))
            )
            pedestrian_num += 1


        # 读取本车信息, 记录为ego_v,ego_x,ego_y,ego_head
        ego_node = opens.getElementsByTagName('Private')[0]
        ego_init = ego_node.childNodes[3].data
        ego_v, ego_x, ego_y, ego_head = [float(i.split('=')[1]) for i in ego_init.split(',')]
        ego_v = abs(ego_v)
        ego_head = (ego_head + 2 * math.pi) if -math.pi <= ego_head < 0 else ego_head
        self.replay_info.add_vehicle(
            id="ego",
            t=-1,
            x=ego_x,
            y=ego_y,
            v=ego_v,
            a=0,
            yaw=ego_head
        )
        """以下读取背景车相关信息，车辆编号从1号开始，轨迹信息记录在vehicle_traj中"""
        """新版场景中采用更general的定义方式，在初始时仅初始化主车，背景车的采用Event中的AddEntityAction和DeleteEntityAction"""
        act_list = opens.getElementsByTagName('Act')
        for id, act in zip(np.arange(1, len(car_elements)), act_list):
            # 记录OpenScenario中存在的位置、航向角、时间信息
            t_list, x_list, y_list, yaw_list = [[] for i in range(4)]
            for point in act.getElementsByTagName('Vertex'):
                t_list.append(round(float(point.getAttribute('time')), 3))  # 记录时间，保留三位小数
                loc = point.getElementsByTagName('WorldPosition')[0]
                x_list.append(float(loc.getAttribute('x')))  # 记录横向位置
                y_list.append(float(loc.getAttribute('y')))  # 记录纵向位置
                yaw = float(loc.getAttribute('h'))  # 记录航向角
                yaw = (yaw + 2 * math.pi) if -math.pi <= yaw < 0 else yaw  # 航向角范围调整到(0, 2pi)
                yaw_list.append(yaw)
            # 计算速度信息
            x_diff = np.diff(x_list)  # 横向距离差
            y_diff = np.diff(y_list)  # 纵向距离差
            t_diff = np.diff(t_list)
            v_list = np.sqrt(x_diff**2+y_diff**2)/t_diff  # 此时v维度比其他参数低
            v_list = list(np.around(v_list, 2))  # 保留2位小数
            v_list.append(v_list[-1])  # 补全维度
            # 计算加速度信息
            v_diff = np.diff(v_list)
            a_list = v_diff/t_diff
            a_list = list(np.around(a_list, 2))  # 保留2位小数
            a_list.append(0.00)  # 补全维度
            # 把时间t_list的内容以字符串形式保存，作为key
            t_list = [str(t) for t in t_list]
            for t, x, y, v, a, yaw in zip(t_list, x_list, y_list, v_list, a_list, yaw_list):
                self.replay_info.add_vehicle(
                    id="car"+str(id),
                    t=t,
                    x=round(x, 2),
                    y=round(y, 2),
                    v=round(v, 2),
                    a=round(a, 2),
                    yaw=round(yaw, 3)
                )

        for id, act in zip(np.arange(1, len(bicycle_elements)+1), act_list[len(car_elements)-1:len(bicycle_elements)+len(car_elements)-1]):
            # 记录OpenScenario中存在的位置、航向角、时间信息
            t_list, x_list, y_list, yaw_list = [[] for i in range(4)]
            for point in act.getElementsByTagName('Vertex'):
                t_list.append(round(float(point.getAttribute('time')), 3))  # 记录时间，保留三位小数
                loc = point.getElementsByTagName('WorldPosition')[0]
                x_list.append(float(loc.getAttribute('x')))  # 记录横向位置
                y_list.append(float(loc.getAttribute('y')))  # 记录纵向位置
                yaw = float(loc.getAttribute('h'))  # 记录航向角
                yaw = (yaw + 2 * math.pi) if -math.pi <= yaw < 0 else yaw  # 航向角范围调整到(0, 2pi)
                yaw_list.append(yaw)
            # 计算速度信息
            x_diff = np.diff(x_list)  # 横向距离差
            y_diff = np.diff(y_list)  # 纵向距离差
            t_diff = np.diff(t_list)
            v_list = np.sqrt(x_diff**2+y_diff**2)/t_diff  # 此时v维度比其他参数低
            v_list = list(np.around(v_list, 2))  # 保留2位小数
            v_list.append(v_list[-1])  # 补全维度
            # 计算加速度信息
            v_diff = np.diff(v_list)
            a_list = v_diff/t_diff
            a_list = list(np.around(a_list, 2))  # 保留2位小数
            a_list.append(0.00)  # 补全维度
            # 把时间t_list的内容以字符串形式保存，作为key
            t_list = [str(t) for t in t_list]
            for t, x, y, v, a, yaw in zip(t_list, x_list, y_list, v_list, a_list, yaw_list):
                self.replay_info.add_bicycle(
                    id="bicycle"+str(id),
                    t=t,
                    x=round(x, 2),
                    y=round(y, 2),
                    v=round(v, 2),
                    a=round(a, 2),
                    yaw=round(yaw, 3)
                )

        for id, act in zip(np.arange(1 , len(pedestrian_elements)+1 ), act_list[len(bicycle_elements)+len(car_elements)-1 : len(pedestrian_elements) +len(bicycle_elements)+len(car_elements)-1]):
            # 记录OpenScenario中存在的位置、航向角、时间信息
            t_list, x_list, y_list, yaw_list = [[] for i in range(4)]
            for point in act.getElementsByTagName('Vertex'):
                t_list.append(round(float(point.getAttribute('time')), 3))  # 记录时间，保留三位小数
                loc = point.getElementsByTagName('WorldPosition')[0]
                x_list.append(float(loc.getAttribute('x')))  # 记录横向位置
                y_list.append(float(loc.getAttribute('y')))  # 记录纵向位置
                yaw = float(loc.getAttribute('h'))  # 记录航向角
                yaw = (yaw + 2 * math.pi) if -math.pi <= yaw < 0 else yaw  # 航向角范围调整到(0, 2pi)
                yaw_list.append(yaw)
            # 计算速度信息
            x_diff = np.diff(x_list)  # 横向距离差
            y_diff = np.diff(y_list)  # 纵向距离差
            t_diff = np.diff(t_list)
            v_list = np.sqrt(x_diff**2+y_diff**2)/t_diff  # 此时v维度比其他参数低
            v_list = list(np.around(v_list, 2))  # 保留2位小数
            v_list.append(v_list[-1])  # 补全维度
            # 计算加速度信息
            v_diff = np.diff(v_list)
            a_list = v_diff/t_diff
            a_list = list(np.around(a_list, 2))  # 保留2位小数
            a_list.append(0.00)  # 补全维度
            # 把时间t_list的内容以字符串形式保存，作为key
            t_list = [str(t) for t in t_list]
            for t, x, y, v, a, yaw in zip(t_list, x_list, y_list, v_list, a_list, yaw_list):
                self.replay_info.add_pedestrian(
                    id="pedestrian"+str(id),
                    t=t,
                    x=round(x, 2),
                    y=round(y, 2),
                    v=round(v, 2),
                    a=round(a, 2),
                    yaw=round(yaw, 3)
                )

        # 获取行驶目标, goal
        goal_init = ego_node.childNodes[5].data
        goal = [float(i) for i in re.findall('-*\d+\.\d+', goal_init)]
        self.replay_info.add_settings(
            goal_x=goal[:2],
            goal_y=goal[2:]
        )
        # 步长与最大时间
        self.replay_info._get_dt_maxt()

        return self.replay_info

    def _parse_opendrive(self, path_opendrive: str) -> None:
        """
        解析opendrive路网的信息，存储到self.replay_info.road_info。
        """
        with open(path_opendrive, 'r', encoding='utf-8') as fh:
            root = etree.parse(fh).getroot()
        #fh = open(path_opendrive, "r")
        # 返回OpenDrive类的实例对象（经过parser.py解析）
        #root = etree.parse(fh).getroot()
        openDriveXml = parse_opendrive(root)
        fh.close()

        # 将OpenDrive类对象进一步解析为参数化的Network类对象，以备后续转化为DiscreteNetwork路网并可视化
        self.loadedRoadNetwork = Network()
        self.loadedRoadNetwork.load_opendrive(openDriveXml)

        """将解析完成的Network类对象转换为DiscreteNetwork路网，其中使用的只有路网中各车道两侧边界的散点坐标
            车道边界点通过线性插值的方式得到，坐标点储存在<DiscreteNetwork.discretelanes.left_vertices/right_vertices> -> List"""

        open_drive_info = self.loadedRoadNetwork.export_discrete_network(
            filter_types=["driving","biking", "onRamp", "offRamp", "exit", "entry"])  # -> <class> DiscreteNetwork
        self.replay_info.road_info = open_drive_info

        # 求出一个包含所有地图的矩形
        points = np.empty(shape=[0, 2])
        for discrete_lane in open_drive_info.discretelanes:
            points = np.concatenate(
                [discrete_lane.left_vertices, discrete_lane.right_vertices, discrete_lane.center_vertices, points],
                axis=0)
        self.replay_info.test_setting['map_range']['x'] = [np.min(points[:, 0]), np.max(points[:, 0])]
        self.replay_info.test_setting['map_range']['y'] = [np.min(points[:, 1]), np.max(points[:, 1])]
        # 记录地图类型
        name = root.find('header').attrib['name']
        if name in ['highway','highD']:
            self.replay_info.test_setting['map_type'] = "highway"
        elif name in ['','SinD']:
            self.replay_info.test_setting['map_type'] = "intersection"
        elif name in ['NDS_ramp']:
            self.replay_info.test_setting['map_type'] = 'ramp'


class ReplayController():
    def __init__(self, visualize=False):
        self.parser = ReplayParser()
        self.visualizer = Visualizer(visualize)
        self.control_info = None

    def init(self, scenario_info: dict) -> Observation:
        self.control_info = self.parser.parse(scenario_info)
        self.visualizer.init(self.control_info)
        scenario_info['startPos'] = [self.control_info.ego_info['x'], self.control_info.ego_info['y']]
        scenario_info['targetPos'] = [
            [self.control_info.test_setting['goal']['x'][0], self.control_info.test_setting['goal']['y'][0]],
            [self.control_info.test_setting['goal']['x'][1], self.control_info.test_setting['goal']['y'][1]],
        ]
        scenario_info['dt'] = self.control_info.test_setting['dt']

        observation = Observation()
        observation.vehicle_info['ego'] = self.control_info.ego_info
        observation.test_info['dt'] = self.control_info.test_setting['dt']
        return observation
    
    def update_frame(self, observation: Observation):
        observation = self._update_other_objects_to_t(observation)
        if self.control_info.light_info:
            observation = self._update_light_info_to_t(observation)
        observation = self._update_end_status(observation)
        self.visualizer.update(observation)
        return observation
    
    def update_ego(self, action: list, observation: Observation) -> Observation:
        action = self._action_cheaker(action)
        observation = self._update_ego_and_t(action, observation)
        return observation

    def _action_cheaker(self, action):
        a = np.clip(action[0], -15, 15)
        rad = np.clip(action[1], -1, 1)
        return (a, rad)

    def _update_light_info_to_t(self, old_observation: Observation) -> Observation:
        new_observation = copy.copy(old_observation)
        new_observation.light_info = self.control_info.light_info[str(np.around(old_observation.test_info['t'], 3))]
        return new_observation

    def _update_ego_and_t(self, action: tuple, old_observation: Observation) -> Observation:
        # 拷贝一份旧观察值
        new_observation = copy.copy(old_observation)
        # 小数点位数，避免浮点数精度问题
        decimal_places = len(str(self.control_info.test_setting['dt']).split('.')[-1])
        # 首先修改时间，新时间=t+dt
        new_observation.test_info['t'] = round(float(
            old_observation.test_info['t'] +
            self.control_info.test_setting['dt']
        ), decimal_places)
        # 修改本车的位置，方式是前向欧拉更新，1.根据旧速度更新位置；2.然后更新速度。
        # 速度和位置的更新基于自行车模型。
        # 首先分别取出加速度和方向盘转角
        a, rot = action
        # 取出步长
        dt = self.control_info.test_setting['dt']
        # 取出本车的各类信息
        x, y, v, yaw, width, length = [float(old_observation.vehicle_info['ego'][key]) for key in ['x', 'y', 'v', 'yaw', 'width', 'length']]

        # 首先根据旧速度更新本车位置
        new_observation.vehicle_info['ego']['x'] = x + \
                                                   v * np.cos(yaw) * dt  # 更新X坐标

        new_observation.vehicle_info['ego']['y'] = y + \
                                                   v * np.sin(yaw) * dt  # 更新y坐标

        new_observation.vehicle_info['ego']['yaw'] = yaw + \
                                                     v / length * 1.7 * np.tan(rot) * dt  # 更新偏航角

        new_observation.vehicle_info['ego']['v'] = v + a * dt  # 更新速度
        if new_observation.vehicle_info['ego']['v'] < 0:
            new_observation.vehicle_info['ego']['v'] = 0

        new_observation.vehicle_info['ego']['a'] = a  # 更新加速度
        return new_observation

    def _update_other_objects_to_t(self, old_observation: Observation) -> Observation:
        # 删除除了ego之外的车辆观察值
        new_observation = copy.copy(old_observation)  # 复制一份旧观察值
        new_observation.vehicle_info = {}
        new_observation.bicycle_info = {}
        new_observation.pedestrian_info = {}
        # 将本车信息添加回来
        new_observation.vehicle_info['ego'] = old_observation.vehicle_info['ego']
        # 根据时间t，查询control_info,赋予新值
        t = old_observation.test_info['t']
        t = str(np.around(t, 3))  # t保留3位小数，与生成control_info时相吻合
        for vehi in self.control_info.vehicle_traj.items():
            id = vehi[0]  # 车辆id
            info = vehi[1]  # 车辆的轨迹信息
            if t in info.keys():
                new_observation.vehicle_info[id] = {}
                for key in ['x', 'y', 'v', 'a', 'yaw']:
                    new_observation.vehicle_info[id][key] = info[t][key]
                for key in ['width', 'length']:
                    new_observation.vehicle_info[id][key] = info['shape'][key]
        for bicyclei in self.control_info.bicycle_traj.items():
            id = bicyclei[0]  # 车辆id
            info = bicyclei[1]  # 车辆的轨迹信息
            if t in info.keys():
                new_observation.bicycle_info[id] = {}
                for key in ['x', 'y', 'v', 'a', 'yaw']:
                    new_observation.bicycle_info[id][key] = info[t][key]
                for key in ['width', 'length']:
                    new_observation.bicycle_info[id][key] = info['shape'][key]
        for pedestriani in self.control_info.pedestrian_traj.items():
            id = pedestriani[0]  # 车辆id
            info = pedestriani[1]  # 车辆的轨迹信息
            if t in info.keys():
                new_observation.pedestrian_info[id] = {}
                for key in ['x', 'y', 'v', 'a', 'yaw']:
                    new_observation.pedestrian_info[id][key] = info[t][key]
                for key in ['width', 'length']:
                    new_observation.pedestrian_info[id][key] = info['shape'][key]
        return new_observation

    def _update_end_status(self, observation: Observation) -> Observation:
        """计算T时刻, 测试是否终止, 更新observation.test_info中的end值
            end=
                1:回放测试运行完毕;
                2:发生碰撞;
        """
        status = -1

        # 检查是否已经驶出地图范围
        if not is_point_inside_rect(
            [[self.control_info.test_setting['map_range']['x'][0], self.control_info.test_setting['map_range']['y'][0]],
             [self.control_info.test_setting['map_range']['x'][1], self.control_info.test_setting['map_range']['y'][1]]],
            [observation.vehicle_info['ego']['x'], observation.vehicle_info['ego']['y']]
        ):
            status = 4

        # 检查是否已到达场景终止时间max_t
        if observation.test_info['t'] >= self.control_info.test_setting['max_t']:
            status = 2

        # 检查主车与背景车是否发生碰撞
        # 当测试时间大于0.5秒时，遍历所有车辆，绘制对应的多边形。这是因为数据问题，有些车辆在初始位置有重叠。也就是说0.5s以内不会判断是否碰撞。
        if observation.test_info['t'] > 0.5:
            if detectCollision(observation.object_info()):
                status = 3

        # 检查是否已经到达终点
        if is_point_inside_rect(
            [[self.control_info.test_setting['goal']['x'][0], self.control_info.test_setting['goal']['y'][0]],
             [self.control_info.test_setting['goal']['x'][1], self.control_info.test_setting['goal']['y'][1]]],
            [observation.vehicle_info['ego']['x'], observation.vehicle_info['ego']['y']]
        ):
            status = 1

        observation.test_info['end'] = status
        return observation

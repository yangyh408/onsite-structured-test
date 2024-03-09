import json
import xml.etree.ElementTree as ET
import xml.dom.minidom
import math
import numpy as np
import re

from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.opendrive2discretenet import parse_opendrive

from .ReplayInfo import ReplayInfo


class ReplayParser():
    """
    解析场景文件
    """
    def __init__(self):
        self.replay_info = ReplayInfo()

    def parse(self, scenario_info: ScenarioInfo) -> ReplayInfo:
        self.replay_info = ReplayInfo()
        self._parse_opendrive(scenario_info.source_file['xodr'])
        self._parse_openscenario(scenario_info.source_file['xosc'])
        if scenario_info.source_file['json']:
            self._parse_light_json(scenario_info.source_file['json'])
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
        # 步长与最大时间
        self.replay_info._get_dt_maxt()

        return self.replay_info

    def _parse_opendrive(self, path_opendrive: str) -> None:
        """
        解析opendrive路网的信息，存储到self.replay_info.road_info
        """
        self.replay_info.road_info = parse_opendrive(path_opendrive)

        # 求出一个包含所有地图的矩形
        points = np.empty(shape=[0, 2])
        for discrete_lane in self.replay_info.road_info.discretelanes:
            points = np.concatenate(
                [discrete_lane.left_vertices, discrete_lane.right_vertices, discrete_lane.center_vertices, points],
                axis=0)
        self.replay_info.test_setting['map_range']['x'] = [np.min(points[:, 0]), np.max(points[:, 0])]
        self.replay_info.test_setting['map_range']['y'] = [np.min(points[:, 1]), np.max(points[:, 1])]
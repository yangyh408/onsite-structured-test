import os
import copy
import math
import numpy as np
import re
import xml.dom.minidom

from .ScenarioInfo import ScenarioInfo
from .ScenarioManagerBase import ScenarioManagerBase

class ScenarioManagerForFragment(ScenarioManagerBase):
    def __init__(self, scenario_dir: str, config: dict):
        super().__init__(config)

        self.scenario_type = "FRAGMENT"
        self.vehicle_info = {
            'name': None,
            't': None,
            'x': None,
            'y': None,
            'v': None,
            'yaw': None,
            'length': None,
            'width': None,
        }

        self.task_dir = os.path.abspath(scenario_dir)
        tasks = config.get('tasks', [])
        self.tasks_without_tess = []    # 用于记录没有tess文件的任务
        if tasks:
            for scene_name in tasks:
                scene_path = os.path.join(self.task_dir, scene_name)
                if os.path.exists(scene_path):
                    if self._check_file_integrity(scene_path):
                        self.tasks.append(scene_name)
                    else:
                        self.tasks_without_tess.append(scene_name)
                        print(f"[LOAD SENARIO ERROR]: Check file integrity in {scene_name}, cannot find all necessary files!")
                else:
                    print(f"[LOAD SENARIO ERROR]: Cannot find task {scene_name}, please check the task name and retry!")
        else:
            for scene_name in os.listdir(self.task_dir):
                scene_path = os.path.join(self.task_dir, scene_name)
                if not scene_name.startswith('.') and scene_name != '__pycache__' and os.path.isdir(scene_path):
                    if self._check_file_integrity(scene_path):
                        self.tasks.append(scene_name)
                    else:
                        self.tasks_without_tess.append(scene_name)
                        print(f"[LOAD SENARIO ERROR]: Check file integrity in {scene_name}, cannot find all necessary files!")
        self.tot_scene_num = len(self.tasks)

    def _check_file_integrity(self, scene_path):
        if not self._find_file_with_suffix(scene_path, '.xodr'):
            return False
        if not self._find_file_with_suffix(scene_path, '.xosc'):
            return False
        if not self._find_file_with_suffix(scene_path, '.tess'):
            return False
        return True

    def _struct_scene_info(self):
        scene_dir = os.path.join(self.task_dir, self.tasks[self.cur_scene_num])
        goal, vehicles, dt = self._parse_openscenario(self._find_file_with_suffix(scene_dir, '.xosc'))
        output_name = f"{self.scenario_type}_{self.cur_scene_num}_{self.tasks[self.cur_scene_num]}_result.csv"
        return ScenarioInfo(
            num = self.cur_scene_num,
            name = self.tasks[self.cur_scene_num],
            type = self.scenario_type,
            source_file = {
                "xodr": self._find_file_with_suffix(scene_dir, '.xodr'), 
                "xosc": self._find_file_with_suffix(scene_dir, '.xosc'),
                "json": "", 
                "tess": self._find_file_with_suffix(scene_dir, '.tess')
                },
            output_path = os.path.join(self.output_path, output_name),
            task_info = {
                "startPos": [vehicles[0]['x'], vehicles[0]['y']], 
                "targetPos": goal, 
                "waypoints": [], 
                "dt": dt,
            },
            additional_info = {
                'vehicle_init_status': vehicles,
            }
        )
    
    def _parse_openscenario(self, file_dir: str):
        vehicles = {}

        # 读取OpenScenario文件
        opens = xml.dom.minidom.parse(file_dir).documentElement

        # 读取车辆长度与宽度信息，录入replay_info。背景车id从1号开始
        wl_node = opens.getElementsByTagName('Dimensions')
        for num, wl in zip(range(len(wl_node)), wl_node):
            cur_vehicle_info = copy.deepcopy(self.vehicle_info)
            cur_vehicle_info['length'] = round(float(wl.getAttribute('length')), 2)
            cur_vehicle_info['width'] = round(float(wl.getAttribute('width')), 2)
            if num == 0:
                cur_vehicle_info['name'] = 'ego'
            else:
                cur_vehicle_info['name'] = num

            vehicles[num] = cur_vehicle_info

        # 读取本车信息, 记录为ego_v,ego_x,ego_y,ego_head
        ego_node = opens.getElementsByTagName('Private')[0]
        ego_init = ego_node.childNodes[3].data
        ego_v, ego_x, ego_y, ego_head = [
            float(i.split('=')[1]) for i in ego_init.split(',')]
        ego_v = abs(ego_v)
        ego_head = (ego_head + 2 * math.pi) if -math.pi <= ego_head < 0 else ego_head
        self._improve_info(vehicles[0], 0.0, ego_x, ego_y, ego_v, ego_head)

        # 获取行驶目标, goal
        goal_init = ego_node.childNodes[5].data
        goal = [round(float(i), 3) for i in re.findall('-*\d+\.\d+', goal_init)]
        goal = [[goal[0], goal[2]], [goal[1], goal[3]]]

        """以下读取背景车相关信息，车辆编号从1号开始，轨迹信息记录在vehicle_traj中"""
        """新版场景中采用更general的定义方式，在初始时仅初始化主车，背景车的采用Event中的AddEntityAction和DeleteEntityAction"""
        act_list = opens.getElementsByTagName('Act')
        for id, act in zip(np.arange(1, len(act_list) + 1), act_list):
            acts = act.getElementsByTagName('Vertex')

            # 记录OpenScenario中存在的位置、航向角、时间信息
            t = float(acts[0].getAttribute('time'))
            x = float(acts[0].getElementsByTagName('WorldPosition')[0].getAttribute('x'))  # 记录横向位置
            y = float(acts[0].getElementsByTagName('WorldPosition')[0].getAttribute('y'))  # 记录纵向位置
            yaw = float(acts[0].getElementsByTagName('WorldPosition')[0].getAttribute('h'))  # 记录航向角
            yaw = (yaw + 2 * math.pi) if -math.pi <= yaw < 0 else yaw  # 航向角范围调整到(0, 2pi)

            if len(acts) > 1:
                t_1 = float(acts[1].getAttribute('time'))
                x_1 = float(acts[1].getElementsByTagName('WorldPosition')[0].getAttribute('x'))  # 记录横向位置
                y_1 = float(acts[1].getElementsByTagName('WorldPosition')[0].getAttribute('y'))  # 记录纵向位置
                dt = t_1 - t
                v = np.sqrt((x_1 - x) ** 2 + (y_1 - y) ** 2) / dt
            else:
                raise RuntimeError("Cannot catch correct vehicle speed!")
            self._improve_info(vehicles[id], t, x, y, v, yaw)
        return goal, vehicles, round(dt,3)
    
    def _improve_info(self, vehicle_info, t, x, y, v, yaw):
        vehicle_info['t'] = round(t, 3)
        vehicle_info['x'] = round(x, 3)
        vehicle_info['y'] = round(y, 3)
        vehicle_info['v'] = round(v, 3)
        vehicle_info['yaw'] = round(yaw, 3)
        return vehicle_info
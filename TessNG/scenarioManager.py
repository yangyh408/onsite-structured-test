# --coding: utf-8 --

import json
import os
import copy
import math
import numpy as np
import re
import xml.dom.minidom

class ScenarioManagerBase():
    def __init__(self, print_info: bool = False):
        self.tasks = []

        self.print_info = print_info
        self.record = {}

        self.cur_scene_num = -1
        self.cur_scene = None

    def next(self):
        self.cur_scene_num += 1
        if self.cur_scene_num >= self.tot_scene_num:
            return False
        try:
            self.cur_scene = self._struct_scene_info()
            self.record[self.cur_scene['scenarioName']] = 'Loaded Success'
            self._print()
            return True
        except Exception as e:
            print(repr(e))
            self.record[self.tasks[self.cur_scene_num]] = repr(e)
            return self.next()
        
    def _struct_scene_info(self):
        scene_info = {
            'scenarioName': self.tasks[self.cur_scene_num],
        }
        return scene_info

    def _print(self):
        if self.print_info:
            print('*'*100)
            for key, value in self.cur_scene.items():
                if key != 'waypoints' and key != 'vehicle_init_status':
                    print(f"   -{key:20s}: {value}")
            print('*'*100)

    def _find_file_with_suffix(self, dir: str, suffix: str):
        for f in os.listdir(dir):
            if not f.startswith('.') and f.endswith(suffix):
                return os.path.join(dir, f)
        print(f"[LOAD TASK ERROR]: Cannot find file with suffix \"{suffix}\" in {dir}!")
        return ""


class ScenarioManagerForFragment(ScenarioManagerBase):
    def __init__(self, scenario_dir: str, tasks: [str], print_info: bool = False):
        super().__init__(print_info)

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
        if len(tasks) == 0:
            for scene_name in os.listdir(self.task_dir):
                if not scene_name.startswith('.') and scene_name != '__pycache__' and os.path.isdir(os.path.join(self.task_dir, scene_name)):
                    self.tasks.append(scene_name)
        else:
            for scene_name in tasks:
                if os.path.exists(os.path.join(self.task_dir, scene_name)):
                    self.tasks.append(scene_name)
                else:
                    print(f"[LOAD SENARIO ERROR]: Cannot find task {scene_name}, please check the task name and retry!")
        self.tot_scene_num = len(self.tasks)

    def _struct_scene_info(self):
        scene_dir = os.path.join(self.task_dir, self.tasks[self.cur_scene_num])
        goal, vehicles = self._parse_openscenario(self._find_file_with_suffix(scene_dir, '.xosc'))
        scene_info = {
            'scenarioNum': self.cur_scene_num,
            'scenarioName': self.tasks[self.cur_scene_num],
            'tess_file_path': self._find_file_with_suffix(scene_dir, '.tess'),
            'xodr_file_path': self._find_file_with_suffix(scene_dir, '.xodr'),
            'xosc_file_path': self._find_file_with_suffix(scene_dir, '.xosc'),
            'targetPos': goal,
            'vehicle_init_status': vehicles,
        }
        return scene_info
    
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
        goal = [float(i) for i in re.findall('-*\d+\.\d+', goal_init)]
        goal = [goal[:2], goal[2:]]

        """以下读取背景车相关信息，车辆编号从1号开始，轨迹信息记录在vehicle_traj中"""
        """新版场景中采用更general的定义方式，在初始时仅初始化主车，背景车的采用Event中的AddEntityAction和DeleteEntityAction"""
        act_list = opens.getElementsByTagName('Act')
        for id, act in zip(np.arange(1, len(act_list) + 1), act_list):
            acts = act.getElementsByTagName('Vertex')

            # 记录OpenScenario中存在的位置、航向角、时间信息
            t = round(float(acts[0].getAttribute('time')), 3)
            x = float(acts[0].getElementsByTagName('WorldPosition')[0].getAttribute('x'))  # 记录横向位置
            y = float(acts[0].getElementsByTagName('WorldPosition')[0].getAttribute('y'))  # 记录纵向位置
            yaw = float(acts[0].getElementsByTagName('WorldPosition')[0].getAttribute('h'))  # 记录航向角
            yaw = (yaw + 2 * math.pi) if -math.pi <= yaw < 0 else yaw  # 航向角范围调整到(0, 2pi)

            if len(acts) > 1:
                t_1 = round(float(acts[1].getAttribute('time')), 3)
                x_1 = float(acts[1].getElementsByTagName('WorldPosition')[0].getAttribute('x'))  # 记录横向位置
                y_1 = float(acts[1].getElementsByTagName('WorldPosition')[0].getAttribute('y'))  # 记录纵向位置
                v = np.sqrt((x_1-x)**2+(y_1-y)**2)/(t_1-t)
            else:
                raise RuntimeError("Cannot catch correct vehicle speed!")
            self._improve_info(vehicles[id], t, x, y, v, yaw)
        return goal, vehicles
    
    def _improve_info(self, vehicle_info, t, x, y, v, yaw):
        vehicle_info['t'] = t
        vehicle_info['x'] = x
        vehicle_info['y'] = y
        vehicle_info['v'] = v
        vehicle_info['yaw'] = yaw
        return vehicle_info

class ScenarioManagerForSerial(ScenarioManagerBase):
    def __init__(self, scenario_dir: str, tasks: [str], print_info: bool = False):
        super().__init__(print_info)

        self.map_dir = os.path.join(scenario_dir, 'maps')
        self.task_dir = os.path.join(scenario_dir, 'tasks')

        if len(tasks) == 0:
            for task_name in os.listdir(self.task_dir):
                if not task_name.startswith('.') and task_name.endswith('.json'):
                    self.tasks.append(task_name)
        else:
            for task in tasks:
                if os.path.exists(os.path.join(self.task_dir, task)):
                    self.tasks.append(task)
                else:
                    print(f"[LOAD SCENARIO ERROR]: Cannot find task {task}, please check the task name and retry!")
        self.tot_scene_num = len(self.tasks)
  
    def _struct_scene_info(self):
        with open(os.path.join(self.task_dir, self.tasks[self.cur_scene_num]),'r', encoding='utf-8') as f:
            scene_json = json.load(f)
        map_path = os.path.join(self.map_dir, scene_json['map'])
        assert os.path.exists(map_path), f"Cannot find map folder {map_path}, please download the map first!"
        scene_info = {
            'scenarioName': self.tasks[self.cur_scene_num].split('.json')[0],
            'tess_file_path': self._find_file_with_suffix(map_path, '.tess'),
            'xodr_file_path': self._find_file_with_suffix(map_path, '.xodr'),
            'startPos': scene_json['startPos'],
            'targetPos': scene_json['targetPos'],
            'waypoints': scene_json['waypoints'],
        }
        return scene_info


def scenarioManager(mode: str, scenario_dir: str, tasks: [str], print_info: bool = False):
    if mode == 'fragment':
        return ScenarioManagerForFragment(scenario_dir, tasks, print_info)
    elif mode == "serial":
        return ScenarioManagerForSerial(scenario_dir, tasks, print_info)
    else:
        raise RuntimeError("There is no valid mode, please choose in 'fragment' and 'serial'")
            
if __name__ == '__main__':
    so = ScenarioManagerForFragment(r'E:\onsite_tessng\on-site\scenario\fragment', [], True)
    so.next()
    so.next()
    so.next()
    so.next()
    so.next()
    so.next()
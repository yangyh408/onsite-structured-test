import copy
import math
import numpy as np
import os
import re
import xml.dom.minidom

vehicle_info = {
    't': None,
    'x': None,
    'y': None,
    'v': None,
    'yaw': None,
    'length': None,
    'width': None,
}

def improve_info(vehicle_info, t, x, y, v, yaw):
    vehicle_info['t'] = t
    vehicle_info['x'] = x
    vehicle_info['y'] = y
    vehicle_info['v'] = v
    vehicle_info['yaw'] = yaw
    return vehicle_info

def parse_openscenario(file_dir: str):
    vehicles = {}

    # 读取OpenScenario文件
    opens = xml.dom.minidom.parse(file_dir).documentElement

    # 读取车辆长度与宽度信息，录入replay_info。背景车id从1号开始
    wl_node = opens.getElementsByTagName('Dimensions')
    for num, wl in zip(range(len(wl_node)), wl_node):
        cur_vehicle_info = copy.deepcopy(vehicle_info)
        cur_vehicle_info['length'] = round(float(wl.getAttribute('length')), 2)
        cur_vehicle_info['width'] = round(float(wl.getAttribute('width')), 2)
        vehicles[num] = cur_vehicle_info

    # 读取本车信息, 记录为ego_v,ego_x,ego_y,ego_head
    ego_node = opens.getElementsByTagName('Private')[0]
    ego_init = ego_node.childNodes[3].data
    ego_v, ego_x, ego_y, ego_head = [
        float(i.split('=')[1]) for i in ego_init.split(',')]
    ego_v = abs(ego_v)
    ego_head = (ego_head + 2 * math.pi) if -math.pi <= ego_head < 0 else ego_head
    improve_info(vehicles[0], 0.0, ego_x, ego_y, ego_v, ego_head)

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
        improve_info(vehicles[id], t, x, y, v, yaw)
    return goal, vehicles


if __name__ == '__main__':
    parse_openscenario(r"E:\onsite_tessng\on-site\scenario\fragment\0_249_merge_256\0_249_merge_256_exam.xosc")
import cv2
import numpy as np
from numpy import array, linalg
import os
import platform
import signal
from typing import Dict

from utils.netStruct import outSide, crash
from utils.logger import logger
from utils.observation import EgoStatus, ObjectStatus, Observation

def convertAngle(angle1: float):
    # 把TESSNG转角与OnSite转角互相转换
    angle2 = (90 - angle1) % 360
    return angle2

def is_point_inside_rect(rect, egoPos) -> bool:
    """
    判断一个点是否在一个矩形内部 (bottom-left and top-right).
    Returns:
    - bool: True 则到达了这个面域内部, False 则没有.
    """
    if egoPos:
        x, y = egoPos[0], egoPos[1]
        x1, y1 = rect[0]
        x2, y2 = rect[1]

        if x1 <= x <= x2 and y1 <= y <= y2:
            return True
        else:
            return False

def _is_collision(ego_info: dict, vehicle_info: dict) -> bool:
    rect1 = ((ego_info['x'], -ego_info['y']), (ego_info['length'], ego_info['width']),
             np.rad2deg(-ego_info['yaw']))
    rect2 = ((vehicle_info['x'], -vehicle_info['y']), (vehicle_info['length'], vehicle_info['width']),
             np.rad2deg(-vehicle_info['yaw']))
    return cv2.rotatedRectangleIntersection(rect1, rect2)[0]

def detectCollision(ego_info: EgoStatus, object_info: Dict[str, ObjectStatus]) -> dict:
    for obj_type in object_info.keys():
        for vehicle_id, vehicle_info in object_info[obj_type].items():
            if _is_collision(vars(ego_info), vars(vehicle_info)):
                return {
                    'collideVehicle': dict(**vars(vehicle_info), id=vehicle_id),
                    'ego': vars(ego_info),
                }
    return {}

def testFinish(goal: list, observation: Observation, outOfTime: bool, outOfMap: bool) -> int:
    # 测试结束有两个条件，Ego行驶到对应的终点面域或者达到极限测试批次
    if outOfMap:
        logger.debug(f"(CODE-4): 测试车驶出道路边界")
        outSide["outSide"] = True
        return 4

    if outOfTime:
        logger.debug(f"(CODE-2): 测试超时")
        return 2

    collideInfo = detectCollision(observation.ego_info, observation.object_info)
    if collideInfo:
        logger.debug(f"(CODE-3): 检测到测试车与背景车{collideInfo['collideVehicle']['id']}发生碰撞")
        # print(f"    --> 测试车状态 x:{collideInfo['ego']['x']} y:{collideInfo['ego']['y']} "
        #       f"yaw:{collideInfo['ego']['yaw']} length:{collideInfo['ego']['length']} "
        #       f"width:{collideInfo['ego']['width']}")
        # print(f"    --> 背景车状态 x:{collideInfo['collideVehicle']['x']} y:{collideInfo['collideVehicle']['y']} "
        #       f"yaw:{collideInfo['collideVehicle']['yaw']} length:{collideInfo['collideVehicle']['length']} "
        #       f"width:{collideInfo['collideVehicle']['width']}")
        crash["crash"] = True
        return 3

    if is_point_inside_rect(goal, [observation.ego_info.x, observation.ego_info.y]):
        logger.debug(f"(CODE-1): 测试车成功抵达目标区域")
        # print(f"    --> 测试车位置:{[vehicleInfo.get('ego')['x'], vehicleInfo.get('ego')['y']]} 终点区域:{goal}")
        return 1

    return -1

def calcDistance(egoPos: list, tessngPos: list) -> float:
    # 只给测试车周围一定范围内的背景车数据
    # 将点转换为 NumPy 数组
    point1_np = array(egoPos)
    point2_np = array(tessngPos)

    # 使用 numpy.linalg.norm 函数求两个点之间的直线距离
    distance = linalg.norm(point2_np - point1_np)
    return distance

def getTessNGCarLength(length: float) -> int:
    result_mapping = {
        range(4, 6): 1,
        range(11, 15): 2,
        range(6, 10): 4,
    }
    for key, result in result_mapping.items():
        if length in key:
            return result
    return 1

def updateEgoPos(action: list, dt: float, ego_info: EgoStatus) -> None:
    # 根据控制量更新参数ego_info中记录的主车状态
    acc, rot = action
    # 修改本车的位置，方式是前向欧拉更新，1.根据旧速度更新位置；2.然后更新速度。
    # 速度和位置的更新基于自行车模型。
    # 取出本车的各类信息
    x, y, v, yaw, length = [float(ego_info.__getattribute__(key)) for key in ['x', 'y', 'v', 'yaw', 'length']]

    ego_info.update(
        x = x + v * np.cos(yaw) * dt,
        y = y + v * np.sin(yaw) * dt,
        yaw = yaw + v / length * 1.7 * np.tan(rot) * dt,
        v = max(0, v + acc * dt),
        a = acc,
        rot = rot,
    )

def check_action(dt, prev_v, prev_action, new_action):
    """检验选手返回的action是否满足动力学约束
    Args:
        dt: float, 时间间隔
        prev_v: float, 上一帧的速度
        prev_action: list, 上一帧的action
        new_action: list, 选手返回的action
    Returns:
        list, 满足动力学约束的action
    """
    ACC_LIMIT = 9.8         # m/s^2
    JERK_LIMIT = 49.0       # m/s^3
    ROT_LIMIT = 0.699       # rad
    ROT_RATE_LIMIT = 1.397  # rad/s

    checked_acc, checked_rot = new_action

    if not np.isnan(prev_action[0]):
        cur_v = prev_v + checked_acc * dt
        if cur_v < 0:
            # 如果时减速到停车的情况
            checked_acc = -prev_v / dt
        else:
            jerk = (new_action[0] - prev_action[0]) / dt
            if abs(jerk) > JERK_LIMIT:
                # 根据执行器动力学修正加速度
                jerk = np.clip(jerk, -JERK_LIMIT, JERK_LIMIT)
                checked_acc = prev_action[0] + jerk * dt

    if not np.isnan(prev_action[1]):
        rot_rate = (new_action[1] - prev_action[1]) / dt
        if abs(rot_rate) > ROT_RATE_LIMIT:
            # 根据执行器动力学修正前轮转角
            rot_rate = np.clip(rot_rate, -ROT_RATE_LIMIT, ROT_RATE_LIMIT)
            checked_rot = prev_action[1] + rot_rate * dt

    return [np.clip(checked_acc, -ACC_LIMIT, ACC_LIMIT), np.clip(checked_rot, -ROT_LIMIT, ROT_LIMIT)]

def kill_process(targetPid):
    if platform.system().lower() == "windows":
        kill_sig = signal.SIGINT
    elif platform.system().lower() == "linux":
        kill_sig = signal.SIGKILL
    try:
        # print(f"[KILL] shutting down TessNG with pid-{targetPid}")
        os.kill(targetPid, kill_sig)
    except Exception as e:
        logger.critical(f"[ERROR] killing {targetPid} failed: {e}")

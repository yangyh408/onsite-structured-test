import os
import json

import cv2
import numpy as np
from PySide2.QtGui import QBrush, QColor
from PySide2.QtWidgets import QGraphicsEllipseItem
from numpy import array, linalg

import Tessng
from Tessng import p2m, m2p
from PySide2.QtCore import QPointF
from utils.netStruct import outSide, crash


def getVehicleInfo(observation) -> dict:
    if observation:
        vehicleInfo = observation.vehicle_info
        return vehicleInfo
    else:
        return {}


def getTessNGCar(length: float) -> int:
    result_mapping = {
        range(4, 6): 1,
        range(11, 15): 2,
        range(6, 10): 4,
    }

    for key, result in result_mapping.items():
        if length in key:
            return result

    return 1


def convertAngle(angle1: float):
    # 把TESSNG转角与OnSite转角互相转换
    angle2 = (90 - angle1) % 360
    return angle2


def judgeAcc(acc: float):
    if acc < 0.0001:
        return 0
    else:
        return acc


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
    # img = np.zeros((512,512,3), np.uint8)
    rect1 = ((ego_info['x'], ego_info['y']), (ego_info['width'], ego_info['length']),
             np.rad2deg(np.pi / 2 - ego_info['yaw']))
    # box1 = np.int0(cv2.boxPoints(rect1))
    # cv2.drawContours(img, [box1], 0, (0,0,255),2)
    rect2 = ((vehicle_info['x'], vehicle_info['y']), (vehicle_info['width'], vehicle_info['length']),
             np.rad2deg(np.pi / 2 - vehicle_info['yaw']))
    # box2 = np.int0(cv2.boxPoints(rect2))
    # cv2.drawContours(img, [box2], 0, (0,0,255),2)
    # cv2.imshow('Rotated Rectangle', img)
    # cv2.waitKey(0)
    return cv2.rotatedRectangleIntersection(rect1, rect2)[0]


def detectCollision(vehicleInfo: dict) -> dict:
    ego = vehicleInfo.get('ego')
    if ego:
        for vehicle_id, vehicle_info in vehicleInfo.items():
            if vehicle_id != 'ego':
                if _is_collision(ego, vehicle_info):
                    collideVehicle = dict(**vehicle_info, id=vehicle_id)
                    return {
                        'collideVehicle': collideVehicle,
                        'ego': ego,
                    }
    return {}


def testFinish(goal: list, vehicleInfo: dict, outOfTime: bool, outOfMap: bool) -> int:
    # 测试结束有两个条件，Ego行驶到对应的终点面域或者达到极限测试批次
    if outOfMap:
        print(f"[FAILED](CODE-4): 测试车驶出道路边界")
        outSide["outSide"] = True
        return 4

    if outOfTime:
        print(f"[FAILED](CODE-2): 测试超时")
        return 2

    if vehicleInfo.get('ego'):
        collideInfo = detectCollision(vehicleInfo)
        if collideInfo:
            print(f"[FAILED](CODE-3): 检测到测试车与背景车{collideInfo['collideVehicle']['id']}发生碰撞")
            print(f"    --> 测试车状态 x:{collideInfo['ego']['x']} y:{collideInfo['ego']['y']} "
                  f"yaw:{collideInfo['ego']['yaw']} length:{collideInfo['ego']['length']} "
                  f"width:{collideInfo['ego']['width']}")
            print(f"    --> 背景车状态 x:{collideInfo['collideVehicle']['x']} y:{collideInfo['collideVehicle']['y']} "
                  f"yaw:{collideInfo['collideVehicle']['yaw']} length:{collideInfo['collideVehicle']['length']} "
                  f"width:{collideInfo['collideVehicle']['width']}")
            crash["crash"] = True
            return 3

        if is_point_inside_rect(goal, [vehicleInfo.get('ego')['x'], vehicleInfo.get('ego')['y']]):
            print(f"[SUCCESS](CODE-1): 测试车成功抵达目标区域")
            print(f"    --> 测试车位置:{[vehicleInfo.get('ego')['x'], vehicleInfo.get('ego')['y']]} 终点区域:{goal}")
            return 1
    else:
        print(f"[ERROR](CODE-0): 测试车已在仿真环境中删除")

    return -1


def calcDistance(egoPos: list, tessngPos: list) -> float:
    # 只给测试车周围一定范围内的背景车数据
    # 将点转换为 NumPy 数组
    point1_np = array(egoPos)
    point2_np = array(tessngPos)

    # 使用 numpy.linalg.norm 函数求两个点之间的直线距离
    distance = linalg.norm(point2_np - point1_np)
    return distance


# 根据路网对象去找对应的Id
def findLinkConnId(edge, netiface: Tessng.NetInterface):
    links = netiface.links()
    conns = netiface.connectors()
    if type(edge) == Tessng.ILaneConnector:
        for conn in conns:
            for conn_lane in conn.laneConnectors():
                if conn_lane.id() == edge.id():
                    return conn.id()
    else:
        for link in links:
            for link_lane in link.lanes():
                if link_lane.id() == edge.id():
                    return link.id()


# 根据起讫点找到路网对象
def getStartEndEdge(netiface, o: list, d: list):
    """

    Args:
        netiface: tessng路网接口
        o: 起点
        d: 终点

    Returns:起始边的Id，终点边的Id

    """
    # 先定位到车道对象
    startEdgeList = netiface.locateOnCrid(QPointF(o[0], -o[1]), 9)
    endEdgeList = netiface.locateOnCrid(QPointF(d[0], -d[1]), 9)
    startLane = startEdgeList[0].pLaneObject
    startEdge = findLinkConnId(startLane, netiface)
    endLane = endEdgeList[0].pLaneObject
    endEdge = findLinkConnId(endLane, netiface)
    return startEdge, endEdge


def getFinishSection(goalX, goalY):
    """

    Args:
        goalX: 终点X范围
        goalY: 终点Y范围

    Returns: 测试结束面域

    """
    return [[min(goalX), min(goalY)], [max(goalX), max(goalY)]]


def findTessngFile(base, suffix):
    for root, ds, fs in os.walk(base):
        for f in fs:
            if f.endswith(suffix):
                fullname = os.path.join(root, f)
                yield fullname


def getTessngFile(path, suffix):
    tessngFiles = []
    dirs = os.listdir(path)
    del dirs[-1]
    for dirName in dirs:
        base = path + "/" + dirName
        for i in findTessngFile(base, suffix):
            tessngFiles.append(i)
    return tessngFiles


def convert_angle(angle1: float):
    angle2 = (90 - angle1) % 360
    return angle2


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


def updateEgoPos(action: tuple, observation) -> dict:
    ego_info = {}
    # 分别取出加速度和前轮转向角
    a, rot = action
    # 取出步长
    _dt = observation.test_info['dt']
    # 取出本车的各类信息
    try:
        x, y, v, yaw, width, length = [float(observation.vehicle_info['ego'][key]) for key in [
            'x', 'y', 'v', 'yaw', 'width', 'length']]
        ego_info['x'] = x + v * np.cos(yaw) * _dt  # 更新X坐标
        ego_info['y'] = y + v * np.sin(yaw) * _dt  # 更新y坐标
        ego_info['yaw'] = yaw + v / length * 1.7 * np.tan(rot) * _dt  # 更新偏航角
        ego_info['v'] = max(0, v + a * _dt)  # 更新速度
        ego_info['a'] = a  # 更新加速度
        ego_info['width'] = width
        ego_info['length'] = length
    except KeyError:
        pass
    return ego_info


def getRouteLinks(tessngRoute):
    if tessngRoute:
        return tessngRoute.getLinks()
    else:
        return None


def originList(originalList, step):
    return originalList[:: step]


def getWayPoints(fullRoute, netFace):
    """

    Args:
        fullRoute: 包含了路径中所有路段和连接段的id
        netFace: TessNG路网接口

    Returns: 路径中路段和连接段缩减后的点序列

    """
    # 总车道数和对应取哪根车道
    getLane = {
        1: 0,
        2: 1,
        3: 1,
        4: 2,
        5: 2,
        6: 3
    }
    # 路径组成对象点的数量应每隔几个单位取点
    pointNumLabels = {
        0: 2,
        10: 2,
        20: 3,
        "inf": 3
    }
    wayPoints = []
    sidesCount = len(fullRoute)
    for i in range(sidesCount):
        sideId = fullRoute[i]
        if i % 2 == 0:
            # 路段
            link = netFace.findLink(sideId)
            centerBreakPoints = link.centerBreakPoints()
            countPoints = len(centerBreakPoints)
            newPoints = originList(centerBreakPoints,
                                   pointNumLabels[countPoints // 10 * 10] if countPoints < 20 else pointNumLabels[
                                       "inf"])
            wayPoints += newPoints
        else:
            # 连接段
            connector = netFace.findConnector(sideId)
            laneConnectors = connector.laneConnectors()
            laneNum = len(laneConnectors)
            connLane = laneConnectors[getLane.get(laneNum)]
            centerBreakPoints = connLane.centerBreakPoints()
            countPoints = len(centerBreakPoints)
            newPoints = originList(centerBreakPoints,
                                   pointNumLabels[countPoints // 10 * 10] if countPoints < 20 else pointNumLabels[
                                       "inf"])
            wayPoints += newPoints
    return wayPoints


def writeWayPoints(wayPoints, fileName, fileNum):
    wayPointCount = len(wayPoints)
    pointInfo = dict()
    for i in range(wayPointCount):
        newPos = [p2m(wayPoints[i].x()), -p2m(wayPoints[i].y())]
        pointInfo[i + 1] = newPos
    with open(f"./{fileName}_{fileNum}.json", "w") as f:
        json.dump(pointInfo, f)


def paintEgoPos(iface, egoPos):
    if egoPos:
        egoPosPoint = QPointF(egoPos[0], egoPos[1])
        netiface = iface.netInterface()
        scene = netiface.graphicsScene()
        brush = QBrush(QColor(0, 255, 255))
        circle = QGraphicsEllipseItem(p2m(egoPosPoint.x()), p2m(egoPosPoint.y()), m2p(1),
                                      m2p(1))  # (x, y, width, height)
        circle.setBrush(brush)
        scene.addItem(circle)

def check_action(dt, prev_action, new_action):
    """检验选手返回的action是否满足动力学约束
    Args:
        dt: float, 时间间隔
        prev_action: list, 上一帧的action
        new_action: list, 选手返回的action
    Returns:
        list, 满足动力学约束的action
    """
    ACC_LIMIT = 9.8         # m/s^2
    JERK_LIMIT = 49.0       # m/s^3
    ROT_LIMIT = 0.7         # rad
    ROT_RATE_LIMIT = 1.4    # rad/s

    checked_acc, checked_rot = new_action

    if not np.isnan(prev_action[0]):
        # 检验加速度
        jerk = (new_action[0] - prev_action[0]) / dt
        if abs(jerk) > JERK_LIMIT:
            jerk = np.clip(jerk, -JERK_LIMIT, JERK_LIMIT)
            checked_acc = prev_action[0] + jerk * dt

    if not np.isnan(prev_action[1]):
        # 检验前轮转角
        rot_rate = (new_action[1] - prev_action[1]) / dt
        if abs(rot_rate) > ROT_RATE_LIMIT:
            rot_rate = np.clip(rot_rate, -ROT_RATE_LIMIT, ROT_RATE_LIMIT)
            checked_rot = prev_action[1] + rot_rate * dt

    return [np.clip(checked_acc, -ACC_LIMIT, ACC_LIMIT), np.clip(checked_rot, -ROT_LIMIT, ROT_LIMIT)]

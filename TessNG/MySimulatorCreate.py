import math
import time
import _thread

from typing import Tuple

from DockWidget import *
from Tessng import *
import Tessng
from TessNG.scenarioManager import scenarioManager
from TessNG.recorder import Recorder
from TessNG.observation import Observation

from utils.config import *
from utils.functions import *
from utils.netStruct import paintPos, startEndPos, waypoints
from utils.AutoSelfDrivingCar import Car

# todo 准备接入penScenario文件的第一帧所有车的数据，用官方案例的inputs和outputs
startTest = True
finishTest = False

task_num = 25
start_lane = 0
linkIds = [284, 181, 304, 215, 45, 178]


task_name = f"serial_task_{task_num}"
task_path = f"./scenario/serial/tasks/{task_name}.json"
if os.path.exists(task_path):
    with open(task_path, "r") as f:
        scenario_info = json.load(f)
        scenario_info["scenarioName"] = task_name
        startEndPos["startPos"] = scenario_info.get('startPos', [])
        startEndPos["endPos"] = scenario_info.get('targetPos', [])
        waypoints["waypoints"] = scenario_info.get('waypoints', {})
else:
    scenario_info = {"scenarioName": task_name}

def getGoalArea(goal_point):
    center_x, center_y = goal_point
    gap_dis = 1.5
    return [[center_x - gap_dis, center_y + gap_dis], [center_x + gap_dis, center_y - gap_dis]]


def shouldIgnoreCrossPoint(pIVehicle: Tessng.IVehicle, curVehicleToStartPointDistance: float,
                           fromStartToCrossPointDistance: float) -> bool:
    iface = tessngIFace()
    simuiface = iface.simuInterface()
    curVehicleLength = pIVehicle.length()
    laneConnector = pIVehicle.laneConnector()
    # 如果当前车辆正在通过或者已经通过交叉点则不考虑当前交叉点
    if (curVehicleToStartPointDistance + (curVehicleLength / 2)) >= fromStartToCrossPointDistance:
        return True
    # 考虑到通行效率如果当前车辆距离交叉点距离大于3则也不考虑
    elif fromStartToCrossPointDistance - (curVehicleToStartPointDistance + curVehicleLength / 2) > 3:
        return True
    else:
        pass
    isClosest = True
    # 判断当前车辆是不是距离交叉点最近的车辆，如果不是直接跟车就行
    curLaneVehicles = simuiface.vehisInLaneConnector(laneConnector.connector().id(),
                                                     laneConnector.fromLane().id(),
                                                     laneConnector.toLane().id())
    for vehicle in curLaneVehicles:
        if vehicle.id() == pIVehicle.id():
            continue
        vehicleToStartPointDistance = laneConnector.distToStartPoint(vehicle.pos())
        if vehicleToStartPointDistance + vehicle.length() / 2 >= fromStartToCrossPointDistance:
            continue
        elif vehicleToStartPointDistance > curVehicleToStartPointDistance:
            isClosest = False
            break
    if not isClosest:
        return True
    return False


def shouldAvoidCrossLaneVehicle(crossPoint: Tessng.Online.CrossPoint, curVehicleToCrossPointDistance: float):
    iface = tessngIFace()
    simuiface = iface.simuInterface()
    crossLane = crossPoint.mpLaneConnector
    crossLaneCrossToStartPointDistance = crossLane.distToStartPoint(crossPoint.mCrossPoint)
    crossLaneVehicles = simuiface.vehisInLaneConnector(crossLane.connector().id(),
                                                       crossLane.fromLane().id(),
                                                       crossLane.toLane().id())
    pClosestVehicle = None
    closetVehicleToCrossPointDistance = 0
    maxStartToCrossPointDistance = 0
    hasCrossingCarInCrossLane = False
    for vehicle in crossLaneVehicles:
        vehicleLength = vehicle.length()
        vehicleToStartPointDistance = crossLane.distToStartPoint(vehicle.pos())
        if vehicleToStartPointDistance - vehicleLength / 2 > crossLaneCrossToStartPointDistance:
            continue
        elif vehicleToStartPointDistance + vehicleLength / 2 < crossLaneCrossToStartPointDistance:
            if vehicleToStartPointDistance > maxStartToCrossPointDistance:
                maxStartToCrossPointDistance = vehicleToStartPointDistance
                pClosestVehicle = vehicle
                closetVehicleToCrossPointDistance = crossLaneCrossToStartPointDistance - vehicleToStartPointDistance - vehicleLength / 2
        else:
            hasCrossingCarInCrossLane = True
            break
    if hasCrossingCarInCrossLane:
        return True
    if pClosestVehicle:
        if pClosestVehicle.currSpeed() <= 1:
            if curVehicleToCrossPointDistance > closetVehicleToCrossPointDistance:
                return True
    return False


def updateSimuStatus(MySimu):
    global finishTest, startTest
    iface = tessngIFace()
    simuiface = iface.simuInterface()
    while True:
        if startTest:
            startTest = False
            time.sleep(1)
            finishTest = True
        if finishTest and (simuiface.isRunning() and not simuiface.isPausing()):
            startTest = False
            MySimu.forStopSimu.emit()
            while True:
                # 检查 tessng 是否成功停止
                if not simuiface.isRunning():
                    time.sleep(0.5)
                    break
        else:
            time.sleep(1)


# 仿真测试模块
class MySimulator(QObject, PyCustomerSimulator):
    signalRunInfo = Signal(str)
    forStopSimu = Signal()
    forReStartSimu = Signal()
    forPauseSimu = Signal()

    def __init__(self):
        QObject.__init__(self)
        PyCustomerSimulator.__init__(self)

        self.recorder = []
        self.observation = Observation()
        # 实例化规控器
        self.planner = PLANNER()
        # 规控器获取初始场景信息
        self.planner.init(scenario_info["scenarioName"])
        # 测试车的名字
        self.EgoName = EGO_INFO['name']
        # 是否驶出路网
        self.outSideTessngNet = False
        # 主车是否与背景车发生碰撞
        self.collision = False
        # 测试车ID Name对应表
        self.EgoIndex = {}
        # 车辆创建对象锁
        self.createCarLock = 0
        # 场景解析锁
        self.scenarioLock = 0
        # 最大测试时长
        self.maxTestTime = 0
        # 仿真预热时间
        self.preheatingTime = 0
        # 车辆变道企图
        self.egoAct = []
        self.EgoPos = []
        self.temp_attempt = None
        # 车辆变道企图记录
        self.Ego_Car_Attempt = {}
        # 下一帧主车信息
        self.nextEgoInfo = {}
        # 启动监测线程
        _thread.start_new_thread(updateSimuStatus, (self,))

    def ref_beforeStart(self, ref_keepOn):
        global finishTest
        # 是否完成测试
        startTest = True
        finishTest = False

        iface = tessngIFace()
        simuiface = iface.simuInterface()
        simuiface.setSimuAccuracy(1 / dt)
        # 最大仿真测试时间
        self.maxTestTime = int(dt * calculateBatchesFinish)
        self.preheatingTime = preheatingTime
        # startEndPos["startPos"] = scenario_info.get('startPos', [])
        # startEndPos["endPos"] = scenario_info.get('targetPos', [])
        # waypoints["waypoints"] = scenario_info.get('waypoints', {})

        return True

    @staticmethod
    def shouldSlowDownInCrossroads(pIVehicle: Tessng.IVehicle, shouldAcce: bool) -> Tuple[bool, bool]:
        iface = tessngIFace()
        netiface = iface.netInterface()

        if not pIVehicle:
            return False, shouldAcce
        if not pIVehicle.roadIsLink() and pIVehicle.vehicleTypeCode() == 1:
            laneConnector = pIVehicle.laneConnector()
            if laneConnector:
                crossPoints = netiface.crossPoints(laneConnector)
                if crossPoints and len(crossPoints) > 0:
                    curVehicleLength = pIVehicle.length()
                    curVehicleToStartPointDistance = laneConnector.distToStartPoint(pIVehicle.pos())
                    for crossPoint in crossPoints:
                        fromStartToCrossPointDistance = laneConnector.distToStartPoint(crossPoint.mCrossPoint)
                        curVehicleToCrossPointDistance = fromStartToCrossPointDistance - curVehicleToStartPointDistance - curVehicleLength / 2
                        if shouldIgnoreCrossPoint(pIVehicle, curVehicleToCrossPointDistance,
                                                  fromStartToCrossPointDistance):
                            continue
                        if shouldAvoidCrossLaneVehicle(crossPoint, curVehicleToCrossPointDistance):
                            return True, shouldAcce
                    if p2m(pIVehicle.currSpeed()) < 3:
                        if pIVehicle.vehicleFront() is None and p2m(pIVehicle.vehiDistFront() > 20):
                            shouldAcce = True
        return False, shouldAcce

    def delVehicle(self, pIVehicle) -> bool:
        # 删除在指定消失路段的车辆
        if pIVehicle.name() != self.EgoName and self.outSideTessngNet:
            return True
        else:
            return False

    # 控制车辆的变道
    def setEgoCarChangeLane(self, pIVehicle):
        if self.Ego_Car_Attempt.get(pIVehicle.name()):
            attempt = self.Ego_Car_Attempt.get(pIVehicle.name())
            # 企图与上一次不同
            if attempt != self.temp_attempt:
                self.temp_attempt = attempt
                if attempt == "Right":
                    pIVehicle.vehicleDriving().toRightLane(True)
                    print(pIVehicle.name(), "向右变道")
                elif attempt == "Left":
                    pIVehicle.vehicleDriving().toLeftLane(True)
                    print(pIVehicle.name(), "向左变道")

    # 收集航向角并获取分析车辆的变道企图
    def getHeaderCarChangeLaneAttempt(self, pIVehicle):
        if self.egoAct:
            if self.EgoName == pIVehicle.name():
                if self.egoAct[1] == 0:
                    self.Ego_Car_Attempt[self.EgoName] = 'Straight'
                elif self.egoAct[1] > 0:
                    self.Ego_Car_Attempt[self.EgoName] = 'Left'
                elif self.egoAct[1] < 0:
                    self.Ego_Car_Attempt[self.EgoName] = 'Right'

    def tessngServerMsg(self, tessngSimuiface, currentBatchNum, currentTestTime):
        """
        :param tessngSimuiface: TESSNG Simuiface接口
        :param currentBatchNum: 当前TESSNG的仿真计算批次号
        :param currentTestTime: 当前TESSNG的仿真计算时间（单位：ms）
        :return: 返回给控制算法的observation信息
        """
        lAllVehiStatus = tessngSimuiface.getVehisStatus()

        vehicleInfo = {}
        vehicleTotal = Observation()
        for vehicleStatus in lAllVehiStatus:
            vehicleInfo[self.EgoIndex.get(vehicleStatus.vehiId, vehicleStatus.vehiId)] = {
                'length': p2m(vehicleStatus.mrLength),
                'width': p2m(vehicleStatus.mrWidth),
                'x': p2m(vehicleStatus.mPoint.x()),
                'y': -p2m(vehicleStatus.mPoint.y()),
                'v': p2m(vehicleStatus.mrSpeed),
                'a': judgeAcc(p2m(vehicleStatus.mrAcce)),
                'yaw': math.radians(convertAngle(vehicleStatus.mrAngle))
            }
        # if self.nextEgoInfo:
        #     vehicleInfo[self.EgoName] = self.nextEgoInfo
        #     if self.EgoName not in vehicleInfo.keys():
        #         global finishTest
        #         finishTest = True
        vehicleTotal.vehicle_info = vehicleInfo
        vehicleTotal.light_info = {'green'}

        vehicleTotal.test_setting = {"scenario_name": f"{scenario_info['scenarioName']}",
                                     "scenario_type": "TESSNG",
                                     "max_t": self.maxTestTime, "t": currentTestTime / 1000, "dt": dt,
                                    #  "goal": {"x": [scenario_info['targetPos'][0][0],
                                    #                 scenario_info['targetPos'][1][0]],
                                    #           "y": [scenario_info['targetPos'][0][1],
                                    #                 scenario_info['targetPos'][1][1]]},
                                     "end": "-1",
                                     "map_type": "testground", 'x_max': 2000.0, 'x_min': -2000.0, 'y_max': 2000,
                                     'y_min': -2000}

        return vehicleTotal

    # def ref_reSetAcce(self, vehi, inOutAcce):
    #     if vehi.name() == self.EgoName and self.egoAct:
    #         inOutAcce.value = self.egoAct[0]
    #         return True
    #     else:
    #         return False

    def createEgoWithLinkid(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface, linkId, speed, vehicleTypeCode):
        dvp = Online.DynaVehiParam()
        dvp.vehiTypeCode = vehicleTypeCode
        dvp.dist = 0
        dvp.speed = m2p(speed)
        dvp.color = "#00BFFF"
        dvp.name = self.EgoName
        # dvp.roadId = netiface.findLink(linkId)
        dvp.roadId = linkId
        dvp.laneNumber = start_lane
        vehi = simuiface.createGVehicle(dvp)
        if vehi:
            self.EgoIndex[vehi.id()] = vehi.name()

    def carCreateCar(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface, startPos, speed,
                     vehicleTypeCode):
        # 获取到车辆的x,y坐标
        lLocations = netiface.locateOnCrid(QPointF(startPos[0], -startPos[1]), 9)
        if lLocations:
            dvp = Online.DynaVehiParam()
            dvp.vehiTypeCode = vehicleTypeCode
            dvp.dist = lLocations[0].distToStart
            dvp.speed = m2p(speed)
            # 来自VTD的车，TESSNG车辆被创建后，名字与VTD车名字相同
            dvp.color = "#00BFFF"
            dvp.name = self.EgoName
            # 如果是路段
            if lLocations[0].pLaneObject.isLane():
                lane = lLocations[0].pLaneObject.castToLane()
                dvp.roadId = lane.link().id()
                dvp.laneNumber = lane.number()
            # 如果是连接段
            else:
                lane_connector = lLocations[0].pLaneObject.castToLaneConnector()
                dvp.roadId = lane_connector.connector().id()
                dvp.laneNumber = lane_connector.fromLane().number()
                dvp.toLaneNumber = lane_connector.toLane().number()
            vehi = simuiface.createGVehicle(dvp)
            if vehi:
                self.EgoIndex[vehi.id()] = vehi.name()
                self.EgoPos = startPos

    # ego = Car(id=vehi.id(), name=vehi.name(), Xpos=p2m(vehi.pos().x()), Ypos=-p2m(vehi.pos().y()),
    #           speed=p2m(vehi.currSpeed()), roadType=vehi.roadType(), frameCount=5, threshold=0.11)
    # # 创建计算对象
    # self.egoList.append(ego)
                
    def updateEgoRoute(self, pIVehicle):
        if pIVehicle.name() == self.EgoName:
            if linkIds:
                if pIVehicle.roadId() == linkIds[0]:
                    if len(linkIds) > 1:
                        iface = tessngIFace()
                        netiface = iface.netInterface()
                        egoRoute = netiface.shortestRouting(netiface.findLink(linkIds[0]), netiface.findLink(linkIds[1]))
                        pIVehicle.vehicleDriving().setRouting(egoRoute)
                    linkIds.pop(0)
                    print(linkIds)
            else:
                global finishTest
                finishTest = True
            

    # 实时获取Ego的坐标
    def getEgoPos(self, pIVehicle):
        vehicle_name = pIVehicle.name()
        if vehicle_name and vehicle_name == self.EgoName:
            self.EgoPos = [p2m(pIVehicle.pos().x()), -p2m(pIVehicle.pos().y())]
            self.EgoLink = pIVehicle.roadId()
            return

    @staticmethod
    def isRoadLink(pIVehicle) -> bool:
        if pIVehicle.roadIsLink():
            return True
        else:
            return False

    def moveEgo(self, vehicle, position):
        if vehicle.name() == self.EgoName and position:
            iface = tessngIFace()
            netiface = iface.netInterface()
            lLocations = netiface.locateOnCrid(QPointF(m2p(position['x']), - m2p(position['y'])), 9)
            if not lLocations:
                self.outSideTessngNet = True
                return

            # 计算车辆当前所在车道或所在车道连接的上游车道，作为匹配时的目标车道
            if vehicle.roadIsLink():
                currentLinkLane = vehicle.lane()
                # 如果在路段上，获取到正在路段上的车道编号
                target_lane_id = currentLinkLane.id()
            else:
                # 如果当前车辆在连接段上，.lane() 取到的是车道连接所在的上游车道
                currentLinkLaneConnector = vehicle.lane()
                # 如果现在在连接段上，获取到正在连接段上的车道，并且获取到连接段车道的上游车道
                target_lane_id = currentLinkLaneConnector.id()

            # 选取 最优 location
            location = lLocations[0]  # 默认取第一个
            for demo_location in lLocations:
                if demo_location.pLaneObject.isLane():
                    # 如果匹配到了正常车道，不需要做车道预期判断
                    location = demo_location
                    break
                else:
                    # 如果匹配到了车道连接，做车道预期判断，只取符合条件的车道
                    lane = demo_location.pLaneObject.castToLaneConnector()
                    if lane.fromLane().id() == target_lane_id:
                        location = demo_location
                        break

            # 强制移动av/mv车辆
            vehicle.vehicleDriving().move(location.pLaneObject, location.distToStart)
            return

    def paintMyVehicle(self, pIVehicle: Tessng.IVehicle):
        if pIVehicle.name() == self.EgoName:
            # 主车变蓝
            pIVehicle.setColor("#00BFFF")
        elif pIVehicle.id() in self.observation.vehicle_info.keys():
            # 被Observation记录的背景车变紫
            pIVehicle.setColor("#C318FF")
        else:
            # 其余背景车保持白色
            pIVehicle.setColor("#F8F8FF")

    def afterStep(self, pIVehicle: Tessng.IVehicle) -> None:
        self.getEgoPos(pIVehicle)
        # 采集测试车的航向角
        self.getHeaderCarChangeLaneAttempt(pIVehicle)
        self.setEgoCarChangeLane(pIVehicle)
        self.updateEgoRoute(pIVehicle)
        # self.paintMyVehicle(pIVehicle)
        # self.moveEgo(pIVehicle, self.nextEgoInfo)
        self.delVehicle(pIVehicle)

    # 主要测试步骤和逻辑
    def mainStep(self, simuiface, netiface):
        if not self.createCarLock:
            # self.carCreateCar(simuiface, netiface, scenario_info['startPos'], 10, EgoTypeCode)
            self.createEgoWithLinkid(simuiface, netiface, linkIds[0], 10, EgoTypeCode)
            self.createCarLock = 1
        if self.EgoPos:
            self.recorder.append(self.EgoPos)
        action = self.planner.act({self.observation})  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
        self.egoAct = action
        # global finishTest
        # simuTime = simuiface.simuTimeIntervalWithAcceMutiples()
        # batchNum = simuiface.batchNumber()
        # if simuTime >= self.preheatingTime * 1000:
        #     if not self.createCarLock:
        #         self.carCreateCar(simuiface, netiface, scenario_info['startPos'], 10, EgoTypeCode)
        #         self.createCarLock = 1
        #     # observation 用于轨迹记录
        #     self.observation = self.tessngServerMsg(simuiface, batchNum, simuTime)
        #     # print(self.observation.test_setting['t'], self.observation.test_setting['end'])
        #     try:
        #         self.recorder.append([self.observation.vehicle_info[self.EgoName]['x'], self.observation.vehicle_info[self.EgoName]['y']])
        #     except:
        #         print(f"{simuTime}: Not find ego!")
        #     if getVehicleInfo(self.observation):
        #         if self.observation.test_setting['end'] == -1:
        #             action = self.planner.act({self.observation})  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
        #             # self.nextEgoInfo = updateEgoPos(action, self.observation)
        #             # paintPos["pos"] = self.nextEgoInfo
        #             self.egoAct = action
        #         # print("算出新点位", self.nextEgoInfo, action)
        #         else:
        #             finishTest = True

    def afterOneStep(self):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        netiface = iface.netInterface()
        self.mainStep(simuiface, netiface)

    def clearLastTest(self):
        # 清空上一次场景记录信息
        self.nextEgoInfo = dict()
        self.maxTestTime = 0
        self.preheatingTime = 0
        self.createCarLock = 0
        self.scenarioLock = 0
        self.outSideTessngNet = False
        self.collision = False

    def afterStop(self):
        wp = {}
        last_pos = None
        num = 1
        for index, ego_pos in enumerate(self.recorder):
            if index % 10 == 0 and ego_pos != last_pos:
                wp[str(num)] = [round(ego_pos[0], 4), round(ego_pos[1], 4)]
                num += 1
                last_pos = ego_pos
        task_info = {
            "map": "TJST",
            "startPos": list(wp.values())[0],
            "targetPos": getGoalArea(list(wp.values())[-1]),
            "waypoints": wp,
        }
        with open(f"./scenario/serial/tasks/{scenario_info['scenarioName']}.json", "w") as f:
            json.dump(task_info, f)
        pidDict = {"done": 1}
        with open("./cache.json", "w") as f:
            json.dump(pidDict, f)
        print("All test finished.")
        return

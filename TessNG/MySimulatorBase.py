import os
import math

from DockWidget import *
from Tessng import *
import Tessng

from utils.recorder import Recorder
from utils.observation import Observation
from utils.functions import convertAngle, calcDistance, testFinish, check_action, updateEgoPos, kill_process
from utils.netStruct import paintPos, startEndPos, waypoints

# 仿真测试模块
class MySimulatorBase(QObject, PyCustomerSimulator):
    signalRunInfo = Signal(str)
    forStopSimu = Signal()
    forReStartSimu = Signal()
    forPauseSimu = Signal()

    def __init__(self):
        QObject.__init__(self)
        PyCustomerSimulator.__init__(self)

        # 测试类型
        self.scenario_type = None
        # 测试间隔
        self.dt = None
        # 最大测试时长
        self.maxTestTime = None
        # 仿真预热时间
        self.preheatingTime = 5
        # 背景车探测范围
        self.radius = 50

        # 测试场景信息
        self.scenario_info = None
        # 实例化规控器
        self.planner = None

        # 仿真流程控制参数
        self.finishTest = False

        self.recorder = Recorder()
        self.observation = Observation()
        
        # 测试车的名字
        self.EgoName = 'ego'
        # 是否驶出路网
        self.outSideTessngNet = False
        # 测试车ID Name对应表
        self.vehicleMap = {}
        # 车辆创建对象锁
        self.createCarLock = 0
        # 场景解析锁
        self.scenarioLock = 0
        # 主车控制量
        self.action = [float('nan'), float('nan')]
        # 下一帧主车信息
        self.nextEgoInfo = {}
        # TODO: 手动开启仿真，但存在稳定性问题，暂时停止使用
        # iface = tessngIFace()
        # simuiface = iface.simuInterface()
        # simuiface.startSimu()

    def ref_beforeStart(self, ref_keepOn):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        simuiface.setSimuAccuracy(1 / self.dt)
        startEndPos["startPos"] = self.scenario_info.get('startPos', [])
        startEndPos["endPos"] = self.scenario_info.get('targetPos', [])
        waypoints["waypoints"] = self.scenario_info.get('waypoints', [])
        return True

    def _paintMyVehicle(self, pIVehicle: Tessng.IVehicle):
        if pIVehicle.name() == self.EgoName:
            # 主车变蓝
            pIVehicle.setColor("#00BFFF")
        elif pIVehicle.name() in self.observation.vehicle_info.keys():
            # 被Observation记录的背景车变紫
            pIVehicle.setColor("#C318FF")
        else:
            # 其余背景车保持白色
            pIVehicle.setColor("#F8F8FF")

    def _moveEgo(self, vehicle, position):
        if vehicle.name() == self.EgoName and position:
            iface = tessngIFace()
            netiface = iface.netInterface()
            lLocations = netiface.locateOnCrid(QPointF(m2p(position['x']), - m2p(position['y'])), 9)
            
            if lLocations:
                # 计算车辆当前所在车道或所在车道连接的上游车道，作为匹配时的目标车道
                if vehicle.roadIsLink():
                    currentLinkLane = vehicle.lane()
                    # 如果在路段上，获取到正在路段上的车道编号
                    target_lane_id = currentLinkLane.id()
                else:
                    # 如果当前车辆在连接段上，.laneConnector() 取到的是车道连接所在的上游车道
                    currentLinkLaneConnector = vehicle.laneConnector()
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

    def _delVehicle(self, pIVehicle) -> bool:
        # 删除在指定消失路段的车辆
        if pIVehicle.name() != self.EgoName and self.outSideTessngNet:
            return True
        else:
            return False
    
    def _checkOutSideMap(self):
        iface = tessngIFace()
        netiface = iface.netInterface()
        if self.nextEgoInfo:
            lLocations = netiface.locateOnCrid(QPointF(m2p(self.nextEgoInfo['x']), - m2p(self.nextEgoInfo['y'])), 9)
            if not lLocations:
                self.outSideTessngNet = True
        
    def afterStep(self, pIVehicle: Tessng.IVehicle) -> None:
        self._paintMyVehicle(pIVehicle)
        self._moveEgo(pIVehicle, self.nextEgoInfo)
        # self._delVehicle(pIVehicle)
        self._checkOutSideMap()

    def _tessngServerMsg(self, tessngSimuiface, currentTestTime):
        """
		:param tessngSimuiface: TESSNG Simuiface接口
		:param currentTestTime: 当前TESSNG的仿真计算时间（单位：ms）
		:return: 返回给控制算法的observation信息
		"""
        lAllVehiStatus = tessngSimuiface.getVehisStatus()
        if self.nextEgoInfo:
            egoPos = [self.nextEgoInfo['x'], self.nextEgoInfo['y']]
        else:
            egoPos = self.scenario_info['startPos']

        vehicleInfo = {}
        vehicleTotal = Observation()
        for vehicleStatus in lAllVehiStatus:
            if calcDistance(egoPos, [p2m(vehicleStatus.mPoint.x()), -p2m(vehicleStatus.mPoint.y())]) < self.radius:
                vehicleInfo[self.vehicleMap.get(vehicleStatus.vehiId, vehicleStatus.vehiId)] = {
                    'length': round(p2m(vehicleStatus.mrLength), 2),
                    'width': round(p2m(vehicleStatus.mrWidth), 2),
                    'x': round(p2m(vehicleStatus.mPoint.x()), 3),
                    'y': round(-p2m(vehicleStatus.mPoint.y()), 3),
                    'v': round(p2m(vehicleStatus.mrSpeed), 3),
                    'a': round(p2m(vehicleStatus.mrAcce), 3),
                    'yaw': round(math.radians(convertAngle(vehicleStatus.mrAngle)), 3),
                }
        if self.nextEgoInfo:
            vehicleInfo[self.EgoName] = self.nextEgoInfo
        vehicleTotal.vehicle_info = vehicleInfo
        vehicleTotal.light_info = {}

        # TODO: 目前设定在预热时间3秒后开始检测终止条件
        if tessngSimuiface.simuTimeIntervalWithAcceMutiples() >= self.preheatingTime * 1000 + 3000:
            end = testFinish(goal=self.scenario_info['targetPos'],
                              vehicleInfo=vehicleInfo,
                              outOfTime=(currentTestTime/1000)>=self.maxTestTime,
                              outOfMap=self.outSideTessngNet
                              )
        else:
            end = -1
        
        vehicleTotal.test_info = {
            "t": currentTestTime / 1000, 
            "dt": self.dt,
            "end": end,
            "acc": self.action[0],
            "rot": self.action[1],
        }
        return vehicleTotal
    
    def _createVehicle(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface, veh_info: dict):
        lLocations = netiface.locateOnCrid(QPointF(veh_info['x'], -veh_info['y']), 9)
        if lLocations:
            dvp = Online.DynaVehiParam()
            dvp.vehiTypeCode = veh_info['type']
            dvp.dist = lLocations[0].distToStart
            dvp.speed = m2p(veh_info['v'])
            # dvp.color = "#00FFFF" if veh_info['name'] == self.EgoName else "#DC143C"
            dvp.name = str(veh_info['name'])
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
                self.vehicleMap[vehi.id()] = vehi.name()

    def _addCar(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface):
        pass

    def mainStep(self, simuiface, netiface):
        simuTime = simuiface.simuTimeIntervalWithAcceMutiples()
        # batchNum = simuiface.batchNumber()
        if simuTime >= self.preheatingTime * 1000 and not self.finishTest:
            if not self.createCarLock:
                self._addCar(simuiface, netiface)
                self.createCarLock = 1

            self.observation = self._tessngServerMsg(simuiface, simuTime)
            if self.observation.test_info['end'] == -1:
                if self.observation.vehicle_info.get(self.EgoName) is not None:
                    self.recorder.record(self.action, self.observation)
                    new_action = self.planner.act(self.observation)  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
                    self.action = check_action(self.observation.test_info['dt'], self.action, new_action)
                    self.nextEgoInfo = updateEgoPos(self.action, self.observation)
                    paintPos["pos"] = self.nextEgoInfo
                # else:
                #     print("===================Ego not found.===================")
            else:
                self.finishTest = True
                self.recorder.record(self.action, self.observation)
                self.forStopSimu.emit()

    def afterOneStep(self):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        netiface = iface.netInterface()
        self.mainStep(simuiface, netiface)
                      
    def afterStop(self):
        self.recorder.output(self.scenario_info['output_path'])
        kill_process(os.getpid())


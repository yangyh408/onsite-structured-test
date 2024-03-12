import os
import math
import copy
import time

from ..DockWidget import *
from ..DLLs.Tessng import *

from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.recorder import Recorder
from utils.observation import Observation, EgoStatus
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
        self.scenario_info = ScenarioInfo()
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
        # 记录主车信息
        self.ego_id = None
        self.ego_info = None

    def ref_beforeStart(self, ref_keepOn):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        simuiface.setSimuAccuracy(1 / self.dt)
        startEndPos["startPos"] = self.scenario_info.task_info['startPos']
        startEndPos["endPos"] = self.scenario_info.task_info['targetPos']
        waypoints["waypoints"] = self.scenario_info.task_info['waypoints']
        return True

    def _tessngServerMsg(self, tessngSimuiface, currentTestTime):
        """
		:param tessngSimuiface: TESSNG Simuiface接口
		:param currentTestTime: 当前TESSNG的仿真计算时间（单位：ms）
		:return: 返回给控制算法的observation信息
		"""
        lAllVehiStatus = tessngSimuiface.getVehisStatus()

        new_observation = Observation()
        # 添加主车信息
        new_observation.ego_info = self.ego_info
        # 添加背景车信息
        for vehicleStatus in lAllVehiStatus:
            if vehicleStatus.vehiId != self.ego_id:
                if calcDistance([self.ego_info.x, self.ego_info.y], [p2m(vehicleStatus.mPoint.x()), -p2m(vehicleStatus.mPoint.y())]) < self.radius:
                    new_observation.update_object_info(
                        'vehicle', 
                        self.vehicleMap.get(vehicleStatus.vehiId, str(vehicleStatus.vehiId)), 
                        length=p2m(vehicleStatus.mrLength),
                        width=p2m(vehicleStatus.mrWidth),
                        x=p2m(vehicleStatus.mPoint.x()),
                        y=-p2m(vehicleStatus.mPoint.y()),
                        v=p2m(vehicleStatus.mrSpeed),
                        a=p2m(vehicleStatus.mrAcce),
                        yaw=math.radians(convertAngle(vehicleStatus.mrAngle)),
                    )
                    tessngSimuiface.getVehicle(vehicleStatus.vehiId).setColor("#C318FF")
                else:
                    tessngSimuiface.getVehicle(vehicleStatus.vehiId).setColor("#F8F8FF")
            else:
                tessngSimuiface.getVehicle(vehicleStatus.vehiId).setColor("#00BFFF")
        # 更新测试结束信息
        if tessngSimuiface.simuTimeIntervalWithAcceMutiples() >= self.preheatingTime * 1000 + 3000:
            end = testFinish(
                goal=self.scenario_info.task_info['targetPos'],
                observation=new_observation,
                outOfTime=(currentTestTime/1000)>=self.maxTestTime,
                outOfMap=self.outSideTessngNet
            )
        else:
            end = -1
        new_observation.update_test_info(t=currentTestTime/1000, dt=self.dt, end=end)
        return new_observation
    
    def _createVehicle(self, simuiface: SimuInterface, netiface: NetInterface, veh_info: dict):
        lLocations = netiface.locateOnCrid(QPointF(veh_info['x'], -veh_info['y']), 9)
        if lLocations:
            dvp = Online.DynaVehiParam()
            dvp.vehiTypeCode = veh_info['type']
            dvp.dist = lLocations[0].distToStart
            dvp.speed = m2p(veh_info['v'])
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
                if self.EgoName in vehi.name():
                    self.ego_id = vehi.id()
                else:
                    self.vehicleMap[vehi.id()] = vehi.name()
            return vehi.id()

    def _addCar(self, simuiface: SimuInterface, netiface: NetInterface):
        pass

    def _createEgoInfo(self, simuiface: SimuInterface, netiface: NetInterface):
        lAllVehiStatus = simuiface.getVehisStatus()
        for vehicleStatus in lAllVehiStatus:
            if vehicleStatus.vehiId == self.ego_id:
                self.ego_info = EgoStatus(
                    length=p2m(vehicleStatus.mrLength),
                    width=p2m(vehicleStatus.mrWidth),
                    x=p2m(vehicleStatus.mPoint.x()),
                    y=-p2m(vehicleStatus.mPoint.y()),
                    v=p2m(vehicleStatus.mrSpeed),
                    yaw=math.radians(convertAngle(vehicleStatus.mrAngle)),
                    a=0,
                    rot=0
                )

    def mainStep(self, simuiface, netiface):
        simuTime = simuiface.simuTimeIntervalWithAcceMutiples()
        # batchNum = simuiface.batchNumber()
        if simuTime >= self.preheatingTime * 1000 and not self.finishTest:
            if not self.createCarLock:
                self._addCar(simuiface, netiface)
                self.createCarLock = 1
            
            if self.ego_info == None:
                self._createEgoInfo(simuiface, netiface)
            else:
                self.observation = self._tessngServerMsg(simuiface, simuTime)
                if self.observation.test_info['end'] == -1:
                        self.recorder.record(self.action, self.observation)
                        self.action = self.planner.act(self.observation)  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
                        ego_action = check_action(
                            dt = self.dt, 
                            prev_v = self.observation.ego_info.v,
                            prev_action = [self.observation.ego_info.a, self.observation.ego_info.rot],
                            new_action = self.action
                        )
                        updateEgoPos(ego_action, self.dt, self.ego_info)
                        paintPos["pos"] = self.ego_info.__dict__
                    # else:
                    #     print("===================Ego not found.===================")
                else:
                    self.finishTest = True
                    self.recorder.record(self.action, self.observation)
                    self.forStopSimu.emit()

    def afterStep(self, pIVehicle: IVehicle) -> None:
        # 每个step后对每个车辆进行遍历
        pass
    
    def _checkOutSideMap(self, netiface: NetInterface):
        if self.ego_info is None:
            return
        lLocations = netiface.locateOnCrid(QPointF(m2p(self.ego_info.x), - m2p(self.ego_info.y)), 9)
        if not lLocations:
            self.outSideTessngNet = True
        
    def _isEgoExist(self, pIVehicle: IVehicle):
        if pIVehicle:
            if not ((pIVehicle.roadIsLink() and pIVehicle.lane()) or (not pIVehicle.roadIsLink() and pIVehicle.laneConnector())): 
                return False
        else:
            return False
        return True
    
    def _moveEgo(self, pIVehicle: IVehicle):
        if self.ego_info is None:
            return

        iface = tessngIFace()
        netiface = iface.netInterface()
        lLocations = netiface.locateOnCrid(QPointF(m2p(self.ego_info.x), - m2p(self.ego_info.y)), 9)
        
        if lLocations:
            # 计算车辆当前所在车道或所在车道连接的上游车道，作为匹配时的目标车道
            if pIVehicle.roadIsLink():
                currentLinkLane = pIVehicle.lane()
                # 如果在路段上，获取到正在路段上的车道编号
                target_lane_id = currentLinkLane.id()
            else:
                # 如果当前车辆在连接段上，.laneConnector() 取到的是车道连接所在的上游车道
                currentLinkLaneConnector = pIVehicle.laneConnector()
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
            pIVehicle.vehicleDriving().move(location.pLaneObject, location.distToStart)
    
    def _updateShadowEgo(self, simuiface: SimuInterface, netiface: NetInterface):
        if self.ego_id:
            ego_vehicle = simuiface.getVehicle(self.ego_id)
            if self._isEgoExist(ego_vehicle):
                self._moveEgo(ego_vehicle)
            else:
                cur_ego_info = {
                    "x": self.ego_info.x,
                    "y": self.ego_info.y,
                    "v": self.ego_info.v,
                    "type": 1,
                    "name": f"{self.EgoName}_{time.time()}"
                }
                self._createVehicle(simuiface, netiface, cur_ego_info)

    def afterOneStep(self):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        netiface = iface.netInterface()
        self.mainStep(simuiface, netiface)
        self._checkOutSideMap(netiface)
        self._updateShadowEgo(simuiface, netiface)
                      
    def afterStop(self):
        self.recorder.output(self.scenario_info.output_path)
        kill_process(os.getpid())

    def get_observation(self):
        return copy.deepcopy(self.observation)


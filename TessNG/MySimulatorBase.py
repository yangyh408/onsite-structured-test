import math
import time
import _thread

from DockWidget import *
from Tessng import *
import Tessng

from utils.recorder import Recorder
from utils.observation import Observation
from utils.functions import *

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

        # 实例化测试场景管理器
        self.scenario_manager = None
        # 实例化规控器
        self.planner = None

        # 仿真流程控制参数
        self.startTest = True
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
        # 下一帧主车信息
        self.nextEgoInfo = {}
        # 启动监测线程
        _thread.start_new_thread(self.updateSimuStatus, ())

    def updateSimuStatus(self):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        while True:
            if self.startTest:
                self.startTest = False
                time.sleep(1)
                self.finishTest = True
            if self.finishTest and (simuiface.isRunning() and not simuiface.isPausing()):
                self.startTest = False
                self.forStopSimu.emit()
                while True:
                    # Check if tessng has successfully stopped
                    if not simuiface.isRunning():
                        time.sleep(0.5)
                        break
            else:
                time.sleep(1)

    def ref_beforeStart(self, ref_keepOn):
        pass

    def delVehicle(self, pIVehicle) -> bool:
        # 删除在指定消失路段的车辆
        if pIVehicle.name() != self.EgoName and self.outSideTessngNet:
            return True
        else:
            return False

    def tessngServerMsg(self, tessngSimuiface, currentTestTime):
        """
		:param tessngSimuiface: TESSNG Simuiface接口
		:param currentTestTime: 当前TESSNG的仿真计算时间（单位：ms）
		:return: 返回给控制算法的observation信息
		"""
        lAllVehiStatus = tessngSimuiface.getVehisStatus()
        if self.nextEgoInfo:
            egoPos = [self.nextEgoInfo['x'], self.nextEgoInfo['y']]
        else:
            egoPos = self.scenario_manager.cur_scene['startPos']

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
                    'a': round(judgeAcc(p2m(vehicleStatus.mrAcce)), 3),
                    'yaw': round(math.radians(convertAngle(vehicleStatus.mrAngle)), 3),
                }
        if self.nextEgoInfo:
            vehicleInfo[self.EgoName] = self.nextEgoInfo
        vehicleTotal.vehicle_info = vehicleInfo
        vehicleTotal.light_info = {'green'}

        vehicleTotal.test_info = {
            "t": currentTestTime / 1000, 
            "dt": self.dt,
            "end": testFinish(goal=self.scenario_manager.cur_scene['targetPos'],
                              vehicleInfo=vehicleInfo,
                              outOfTime=(currentTestTime/1000)>=self.maxTestTime,
                              outOfMap=self.outSideTessngNet
                              )
        }
        return vehicleTotal

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
        self.paintMyVehicle(pIVehicle)
        self.moveEgo(pIVehicle, self.nextEgoInfo)
        self.delVehicle(pIVehicle)

    def mainStep(self, simuiface, netiface):
        pass

    def afterOneStep(self):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        netiface = iface.netInterface()
        self.mainStep(simuiface, netiface)

    def clearLastTest(self):
        self.nextEgoInfo = dict()
        self.vehicleMap = dict()
        self.createCarLock = 0
        self.outSideTessngNet = False

    def dealRecord(self):
        if self.scenario_manager.cur_scene_num >= 0:
            output_dir = os.path.abspath(os.path.join(os.path.abspath(__file__), '..', '..', 'outputs'))
            output_name =  f"{self.scenario_type}_{self.scenario_manager.cur_scene_num}_{self.scenario_manager.cur_scene['scenarioName']}_result.csv"
            self.recorder.output(os.path.join(output_dir, output_name))
                                           
    def afterStop(self):
        pass

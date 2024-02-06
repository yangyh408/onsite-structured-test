from Tessng import *
import Tessng

from utils.scenarioManager import scenarioManager
from utils.functions import *
from utils.netStruct import paintPos, startEndPos, waypoints

from MySimulatorBase import MySimulatorBase

class MySimulatorSerial(MySimulatorBase):

    def __init__(self, config: dict, planner: object):
        MySimulatorBase.__init__(self)

        # 测试类型
        self.scenario_type = 'SERIAL'
        # 测试间隔
        self.dt = config.get('dt', 0.05)
        # 最大测试时长
        self.maxTestTime = config.get('maxTestTime', 60)
        # 仿真预热时间
        self.preheatingTime = 5
        # 背景车探测范围
        self.radius = 50

        # 实例化测试场景管理器
        self.scenario_manager = scenarioManager(self.scenario_type, config['tasks'], print_info=True)
        # 实例化规控器
        self.planner = planner()
        # 规控器获取初始场景信息
        self.planner.init(self.scenario_manager.cur_scene)

    def ref_beforeStart(self, ref_keepOn):
        # 是否完成测试
        self.finishTest = False

        iface = tessngIFace()
        simuiface = iface.simuInterface()
        simuiface.setSimuAccuracy(1 / self.dt)
        startEndPos["startPos"] = self.scenario_manager.cur_scene['startPos']
        startEndPos["endPos"] = self.scenario_manager.cur_scene['targetPos']
        waypoints["waypoints"] = self.scenario_manager.cur_scene['waypoints']

        return True

    def createCar(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface, startPos, speed,
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
                self.vehicleMap[vehi.id()] = vehi.name()

    def mainStep(self, simuiface, netiface):
        simuTime = simuiface.simuTimeIntervalWithAcceMutiples()
        # batchNum = simuiface.batchNumber()
        if simuTime >= self.preheatingTime * 1000:
            if not self.createCarLock:
                self.createCar(simuiface, netiface, self.scenario_manager.cur_scene['startPos'], 10, vehicleTypeCode=1)
                self.createCarLock = 1
            # observation 用于轨迹记录
            self.observation = self.tessngServerMsg(simuiface, simuTime)
            self.recorder.record(self.observation)
            if getVehicleInfo(self.observation):
                if self.observation.test_setting['end'] == -1:
                    action = self.planner.act(self.observation)  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
                    self.nextEgoInfo = updateEgoPos(action, self.observation)
                    paintPos["pos"] = self.nextEgoInfo
                # print("算出新点位", self.nextEgoInfo, action)
                else:
                    self.finishTest = True

    def afterStop(self):
        self.clearLastTest()
        self.dealRecord()

        # 开始下一个场景测试
        if self.scenario_manager.next():
            # 重置记录模块
            self.recorder.init()
            self.observation.init()
            self.planner.init(self.scenario_manager.cur_scene)
            iface = tessngIFace()
            simuiface = iface.simuInterface()
            netface = iface.netInterface()
            netface.openNetFle(self.scenario_manager.cur_scene['tess_file_path'])
            simuiface.startSimu()
        else:
            print(self.scenario_manager.record)
            pidDict = {"done": 1}
            with open("./cache.json", "w") as f:
                json.dump(pidDict, f)
            print("All test finished.")
            return

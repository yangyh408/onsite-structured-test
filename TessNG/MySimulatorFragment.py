from Tessng import *
import Tessng

from utils.scenarioManager import scenarioManager
from createTess.opendrive2tess import opendrive2tess

from utils.functions import *
from utils.netStruct import paintPos, startEndPos

from MySimulatorBase import MySimulatorBase


# 仿真测试模块
class MySimulatorFragment(MySimulatorBase):
    def __init__(self, config: dict, planner: object):
        MySimulatorBase.__init__(self)

        # 测试类型
        self.scenario_type = 'FRAGMENT'
        # 测试间隔
        self.dt = config.get('dt', 0.05)
        # 最大测试时长
        self.maxTestTime = config.get('maxTestTime', 60)
        # 仿真预热时间
        self.preheatingTime = 5
        # 背景车探测范围
        self.radius = 50

        # 实例化测试场景管理器
        self.scenario_manager = scenarioManager(mode=self.scenario_type, config=config)
    
        # 实例化规控器
        self.planner = planner()

    def ref_beforeStart(self, ref_keepOn):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        simuiface.setSimuAccuracy(1 / self.dt)
        startEndPos["startPos"] = self.scenario_manager.cur_scene['startPos']
        startEndPos["endPos"] = self.scenario_manager.cur_scene['targetPos']

        return True

    def createCar(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface, veh_info: dict):
        lLocations = netiface.locateOnCrid(QPointF(veh_info['x'], -veh_info['y']), 9)
        if lLocations:
            dvp = Online.DynaVehiParam()
            dvp.vehiTypeCode = getTessNGCarLength(veh_info['length'])
            dvp.dist = lLocations[0].distToStart
            dvp.speed = m2p(veh_info['v'])
            dvp.color = "#00FFFF" if veh_info['name'] == self.EgoName else "#DC143C"
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

    def mainStep(self, simuiface, netiface):
        simuTime = simuiface.simuTimeIntervalWithAcceMutiples()
        # batchNum = simuiface.batchNumber()
        if simuTime >= self.preheatingTime * 1000:
            if not self.createCarLock:
                for veh_id, veh_info in self.scenario_manager.cur_scene['vehicle_init_status'].items():
                    self.createCar(simuiface, netiface, veh_info)
                self.createCarLock = 1

            self.observation = self.tessngServerMsg(simuiface, simuTime)
            if self.observation.test_info['end'] == -1:
                if self.observation.vehicle_info.get('ego') is not None:
                    self.recorder.record(self.action, self.observation)
                    new_action = self.planner.act(self.observation)  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
                    self.action = check_action(self.observation.test_info['dt'], self.action, new_action)
                    self.nextEgoInfo = updateEgoPos(self.action, self.observation)
                    paintPos["pos"] = self.nextEgoInfo
                # else:
                #     print("===================Ego not found.===================")
            else:
                self.finishTest = True

    @staticmethod
    def openNetFile(netiface: Tessng.NetInterface, scene_info: dict):
        if not scene_info['tess_file_path']:
            new_tess_path = os.path.join(os.path.dirname(scene_info['xodr_file_path']), f"{scene_info['scenarioName']}.tess")
            netiface.createEmptyNetFile(new_tess_path)
            netiface.openNetFle(new_tess_path)
            parms = {
                "file_path": scene_info['xodr_file_path'],
                "step_length": 1,
                "lane_types": ["机动车道", "非机动车道", "人行道", "应急车道"]
                }
            opendrive2tess(netiface, parms)
            netiface.saveRoadNet()
            scene_info['tess_file_path'] = new_tess_path
        netiface.openNetFle(scene_info['tess_file_path'])

    def afterStop(self):
        self.finishTest = False
        self.clearLastTest()
        self.dealRecord()
        # 开始下一个场景测试
        if self.scenario_manager.next():
            # 重置记录模块
            self.recorder.init()
            self.planner.init(self.scenario_manager.current_scene_info())
            # self.createCarList = list(self.scenario_manager.cur_scene['vehicle_init_status'].keys())
            iface = tessngIFace()
            simuiface = iface.simuInterface()
            netface = iface.netInterface()
            self.openNetFile(netface, self.scenario_manager.cur_scene)
            simuiface.startSimu()
        else:
            print(self.scenario_manager.record)
            pidDict = {"done": 1}
            with open("./cache.json", "w") as f:
                json.dump(pidDict, f)
            print("All test finished.")
            return

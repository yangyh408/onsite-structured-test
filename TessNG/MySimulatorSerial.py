from Tessng import *
import Tessng

from utils.scenarioManager import scenarioManager

from MySimulatorBase import MySimulatorBase

class MySimulatorSerial(MySimulatorBase):
    def __init__(self, config: dict, planner: object):
        MySimulatorBase.__init__(self)

        # 测试类型
        self.scenario_type = 'SERIAL'
        # 测试间隔
        self.dt = config.get('dt', 0.05)
        # 最大测试时长
        self.maxTestTime = config.get('maxTestTime', 180)
        # 仿真预热时间
        self.preheatingTime = 5
        # 背景车探测范围
        self.radius = 50
        # 实例化测试场景管理器
        self.scenario_manager = scenarioManager(mode=self.scenario_type, config=config)
        # 实例化规控器
        self.planner = planner()

    def _addCar(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface):
        ego_info = {
            "x": self.scenario_manager.cur_scene['startPos'][0],
            "y": self.scenario_manager.cur_scene['startPos'][1],
            "v": 10,
            "type": 1,
            "name": self.EgoName
        }
        self._createVehicle(simuiface, netiface, ego_info)
    
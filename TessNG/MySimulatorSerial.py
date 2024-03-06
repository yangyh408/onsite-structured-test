from Tessng import *
import Tessng

from utils.scenarioManager import format_scenario_info
from MySimulatorBase import MySimulatorBase

class MySimulatorSerial(MySimulatorBase):
    def __init__(self, config: dict, planner: object, scene_info: dict):
        MySimulatorBase.__init__(self)

        # 测试类型
        self.scenario_type = 'SERIAL'
        # 测试间隔
        self.dt = config.get('dt', 0.05)
        # 最大测试时长
        self.maxTestTime = config.get('maxTestTime', 180)
        # 实例化规控器
        self.planner = planner
        self.planner.init(format_scenario_info(scene_info))
        # 加载场景信息
        self.scenario_info = scene_info
        # 仿真预热时间
        self.preheatingTime = 5
        # 背景车探测范围
        self.radius = 50

    def _addCar(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface):
        ego_info = {
            "x": self.scenario_info['startPos'][0],
            "y": self.scenario_info['startPos'][1],
            "v": 10,
            "type": 1,
            "name": self.EgoName
        }
        self._createVehicle(simuiface, netiface, ego_info)
    
from .MySimulatorBase import MySimulatorBase
from ..DLLs.Tessng import *

from utils.ScenarioManager.ScenarioInfo import ScenarioInfo

class MySimulatorSerial(MySimulatorBase):
    def __init__(self, config: dict, planner: object, scene_info: ScenarioInfo):
        MySimulatorBase.__init__(self)

        # 测试间隔
        self.dt = config.get('dt', 0.05)
        # 最大测试时长
        self.maxTestTime = config.get('maxTestTime', 180)
        # 实例化规控器
        self.planner = planner
        self.planner.init(scene_info.format())
        # 加载场景信息
        self.scenario_info = scene_info
        # 仿真预热时间
        self.preheatingTime = 5
        # 背景车探测范围
        self.radius = 50

    def _addCar(self, simuiface: SimuInterface, netiface: NetInterface):
        ego_info = {
            "x": self.scenario_info.task_info['startPos'][0],
            "y": self.scenario_info.task_info['startPos'][1],
            "v": 10,
            "type": 1,
            "name": self.EgoName
        }
        self._createVehicle(simuiface, netiface, ego_info)
    
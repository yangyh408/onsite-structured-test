from Tessng import *
import Tessng

from utils.scenarioManager import format_scenario_info
from utils.functions import getTessNGCarLength
from MySimulatorBase import MySimulatorBase


# 片段式仿真测试模块
class MySimulatorFragment(MySimulatorBase):
    def __init__(self, config: dict, planner: object, scene_info: dict):
        MySimulatorBase.__init__(self)

        # 测试类型
        self.scenario_type = 'FRAGMENT'
        # 测试间隔
        self.dt = config.get('dt', 0.05)
        # 最大测试时长
        self.maxTestTime = config.get('maxTestTime', 30)
        # 实例化规控器并初始化
        self.planner = planner
        self.planner.init(format_scenario_info(scene_info))
        # 加载场景信息
        self.scenario_info = scene_info
        # 仿真预热时间
        self.preheatingTime = 0
        # 背景车探测范围
        self.radius = 50

    def _addCar(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface):
        for veh_id, veh_info in self.scenario_info['vehicle_init_status'].items():
            veh_info['type'] = getTessNGCarLength(veh_info['length'])
            self._createVehicle(simuiface, netiface, veh_info)
    
    # @staticmethod
    # def openNetFile(netiface: Tessng.NetInterface, scene_info: dict):
    #     from createTess.opendrive2tess import opendrive2tess
    #     if not scene_info['tess_file_path']:
    #         new_tess_path = os.path.join(os.path.dirname(scene_info['xodr_file_path']), f"{scene_info['scenarioName']}.tess")
    #         netiface.createEmptyNetFile(new_tess_path)
    #         netiface.openNetFle(new_tess_path)
    #         parms = {
    #             "file_path": scene_info['xodr_file_path'],
    #             "step_length": 1,
    #             "lane_types": ["机动车道", "非机动车道", "人行道", "应急车道"]
    #             }
    #         opendrive2tess(netiface, parms)
    #         netiface.saveRoadNet()
    #         scene_info['tess_file_path'] = new_tess_path
    #     netiface.openNetFle(scene_info['tess_file_path'])


import copy

from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.observation import Observation
from utils.functions import detectCollision, is_point_inside_rect, updateEgoPos
from utils.logger import logger

from .ReplayParser import ReplayParser


class ReplayController():
    def __init__(self, visualize=False):
        self.parser = ReplayParser()
        self.control_info = None
        self.visualize = visualize
        self.observation = Observation()
        if self.visualize:
            from utils.visualizer import Visualizer
            self.visualizer = Visualizer()

    def init(self, scenario_info: ScenarioInfo) -> Observation:
        self.scenario_info = scenario_info
        self.control_info = self.parser.parse(scenario_info)
        if self.visualize:
            self.visualizer.live_init(scenario_info, self.control_info.road_info)

        self.observation.update_ego_info(**self.control_info.ego_info)
        self.observation.update_test_info(dt=self.scenario_info.task_info['dt'])
    
    def get_observation(self) -> Observation:
        return copy.deepcopy(self.observation)

    def update_frame(self):
        t = str(round(self.observation.test_info['t'], 3))
        self._update_other_objects_to_t(t, self.observation)
        self._update_light_info_to_t(t, self.observation)
        self._update_end_status(self.observation)
        if self.visualize:
            self.visualizer.live_update(self.get_observation())
    
    def update_ego(self, action: list) -> Observation:
        # 取出步长
        dt = self.scenario_info.task_info['dt']
        # 小数点位数，避免浮点数精度问题
        decimal_places = len(str(dt).split('.')[-1])
        # 首先修改时间，新时间=t+dt
        self.observation.update_test_info(t=round(float(self.observation.test_info['t'] + dt), decimal_places))
        updateEgoPos(action, dt, self.observation.ego_info)

    def _update_light_info_to_t(self, t: str, observation: Observation) -> None:
        observation.update_light_info(self.control_info.light_info.get(t, ""))

    def _update_other_objects_to_t(self, t: str, observation: Observation) -> None:
        observation.erase_object_info()
        for obj_type in ['vehicle', 'bicycle', 'pedestrian']:
            for obj_id, obj_info in self.control_info.__getattribute__(f"{obj_type}_traj").items():
                if t in obj_info.keys():
                    observation.update_object_info(obj_type, obj_id, **obj_info[t])
                    observation.update_object_info(obj_type, obj_id, **obj_info['shape'])

    def _update_end_status(self, observation: Observation) -> None:
        """计算T时刻, 测试是否终止, 更新observation.test_info中的end值
            end=
                1: 测试车成功抵达目标区域
                2: 测试超时
                3: 测试车与背景车发生碰撞
                4: 测试车驶出地图边界
        """
        status = -1

        # 检查是否已经驶出地图范围
        if not is_point_inside_rect(
            [[self.control_info.test_setting['map_range']['x'][0], self.control_info.test_setting['map_range']['y'][0]],
             [self.control_info.test_setting['map_range']['x'][1], self.control_info.test_setting['map_range']['y'][1]]],
            [observation.ego_info.x, observation.ego_info.y]
        ):
            status = 4
            logger.debug(f"(CODE-4): 测试车驶出地图边界")

        # 检查是否已到达场景终止时间max_t
        if observation.test_info['t'] >= self.control_info.test_setting['max_t']:
            status = 2
            logger.debug(f"(CODE-2): 测试超时")

        # 检查主车与背景车是否发生碰撞
        # 当测试时间大于0.5秒时，遍历所有车辆，绘制对应的多边形。这是因为数据问题，有些车辆在初始位置有重叠。也就是说0.5s以内不会判断是否碰撞。
        if observation.test_info['t'] > 0.5:
            collideInfo = detectCollision(observation.ego_info, observation.object_info)
            if collideInfo:
                status = 3
                logger.debug(f"(CODE-3): 检测到测试车与背景车{collideInfo['collideVehicle']['id']}发生碰撞")

        # 检查是否已经到达终点
        if is_point_inside_rect(
            self.scenario_info.task_info['targetPos'],
            [observation.ego_info.x, observation.ego_info.y]
        ):
            status = 1
            logger.debug(f"(CODE-1): 测试车成功抵达目标区域")

        observation.update_test_info(end=status)

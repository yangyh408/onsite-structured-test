from .parser import ReplayParser

from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.observation import Observation
from utils.functions import detectCollision, is_point_inside_rect
from utils.logger import logger

import numpy as np
import copy


class ReplayController():
    def __init__(self, visualize=False):
        self.parser = ReplayParser()
        self.control_info = None
        self.visualize = visualize
        if self.visualize:
            from utils.visualizer import Visualizer
            self.visualizer = Visualizer()

    def init(self, scenario_info: ScenarioInfo) -> Observation:
        self.control_info = self.parser.parse(scenario_info)
        if self.visualize:
            self.visualizer.live_init(scenario_info, self.control_info.road_info)

        observation = Observation()
        observation.vehicle_info['ego'] = self.control_info.ego_info
        observation.test_info['dt'] = self.control_info.test_setting['dt']
        return observation
    
    def update_frame(self, observation: Observation):
        observation = self._update_other_objects_to_t(observation)
        if self.control_info.light_info:
            observation = self._update_light_info_to_t(observation)
        observation = self._update_end_status(observation)
        if self.visualize:
            self.visualizer.live_update(observation)
        return observation
    
    def update_ego(self, action: list, observation: Observation) -> Observation:
        # action = self._action_cheaker(action)
        observation = self._update_ego_and_t(action, observation)
        observation.test_info['acc'] = action[0]
        observation.test_info['rot'] = action[1]
        return observation

    def _action_cheaker(self, action):
        a = np.clip(action[0], -15, 15)
        rad = np.clip(action[1], -1, 1)
        return (a, rad)

    def _update_light_info_to_t(self, old_observation: Observation) -> Observation:
        new_observation = copy.copy(old_observation)
        new_observation.light_info = self.control_info.light_info[str(np.around(old_observation.test_info['t'], 3))]
        return new_observation

    def _update_ego_and_t(self, action: tuple, old_observation: Observation) -> Observation:
        # 拷贝一份旧观察值
        new_observation = copy.copy(old_observation)
        # 小数点位数，避免浮点数精度问题
        decimal_places = len(str(self.control_info.test_setting['dt']).split('.')[-1])
        # 首先修改时间，新时间=t+dt
        new_observation.test_info['t'] = round(float(
            old_observation.test_info['t'] +
            self.control_info.test_setting['dt']
        ), decimal_places)
        # 修改本车的位置，方式是前向欧拉更新，1.根据旧速度更新位置；2.然后更新速度。
        # 速度和位置的更新基于自行车模型。
        # 首先分别取出加速度和方向盘转角
        a, rot = action
        # 取出步长
        dt = self.control_info.test_setting['dt']
        # 取出本车的各类信息
        x, y, v, yaw, width, length = [float(old_observation.vehicle_info['ego'][key]) for key in ['x', 'y', 'v', 'yaw', 'width', 'length']]

        # 首先根据旧速度更新本车位置
        new_observation.vehicle_info['ego']['x'] = x + \
                                                   v * np.cos(yaw) * dt  # 更新X坐标

        new_observation.vehicle_info['ego']['y'] = y + \
                                                   v * np.sin(yaw) * dt  # 更新y坐标

        new_observation.vehicle_info['ego']['yaw'] = yaw + \
                                                     v / length * 1.7 * np.tan(rot) * dt  # 更新偏航角

        new_observation.vehicle_info['ego']['v'] = v + a * dt  # 更新速度
        if new_observation.vehicle_info['ego']['v'] < 0:
            new_observation.vehicle_info['ego']['v'] = 0

        new_observation.vehicle_info['ego']['a'] = a  # 更新加速度
        return new_observation

    def _update_other_objects_to_t(self, old_observation: Observation) -> Observation:
        # 删除除了ego之外的车辆观察值
        new_observation = copy.copy(old_observation)  # 复制一份旧观察值
        new_observation.vehicle_info = {}
        new_observation.bicycle_info = {}
        new_observation.pedestrian_info = {}
        # 将本车信息添加回来
        new_observation.vehicle_info['ego'] = old_observation.vehicle_info['ego']
        # 根据时间t，查询control_info,赋予新值
        t = old_observation.test_info['t']
        t = str(np.around(t, 3))  # t保留3位小数，与生成control_info时相吻合
        for vehi in self.control_info.vehicle_traj.items():
            id = vehi[0]  # 车辆id
            info = vehi[1]  # 车辆的轨迹信息
            if t in info.keys():
                new_observation.vehicle_info[id] = {}
                for key in ['x', 'y', 'v', 'a', 'yaw']:
                    new_observation.vehicle_info[id][key] = info[t][key]
                for key in ['width', 'length']:
                    new_observation.vehicle_info[id][key] = info['shape'][key]
        for bicyclei in self.control_info.bicycle_traj.items():
            id = bicyclei[0]  # 车辆id
            info = bicyclei[1]  # 车辆的轨迹信息
            if t in info.keys():
                new_observation.bicycle_info[id] = {}
                for key in ['x', 'y', 'v', 'a', 'yaw']:
                    new_observation.bicycle_info[id][key] = info[t][key]
                for key in ['width', 'length']:
                    new_observation.bicycle_info[id][key] = info['shape'][key]
        for pedestriani in self.control_info.pedestrian_traj.items():
            id = pedestriani[0]  # 车辆id
            info = pedestriani[1]  # 车辆的轨迹信息
            if t in info.keys():
                new_observation.pedestrian_info[id] = {}
                for key in ['x', 'y', 'v', 'a', 'yaw']:
                    new_observation.pedestrian_info[id][key] = info[t][key]
                for key in ['width', 'length']:
                    new_observation.pedestrian_info[id][key] = info['shape'][key]
        return new_observation

    def _update_end_status(self, observation: Observation) -> Observation:
        """计算T时刻, 测试是否终止, 更新observation.test_info中的end值
            end=
                1:回放测试运行完毕;
                2:发生碰撞;
        """
        status = -1

        # 检查是否已经驶出地图范围
        if not is_point_inside_rect(
            [[self.control_info.test_setting['map_range']['x'][0], self.control_info.test_setting['map_range']['y'][0]],
             [self.control_info.test_setting['map_range']['x'][1], self.control_info.test_setting['map_range']['y'][1]]],
            [observation.vehicle_info['ego']['x'], observation.vehicle_info['ego']['y']]
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
            collideInfo = detectCollision(observation.object_info())
            if collideInfo:
                status = 3
                logger.debug(f"(CODE-3): 检测到测试车与背景车{collideInfo['collideVehicle']['id']}发生碰撞")

        # 检查是否已经到达终点
        if is_point_inside_rect(
            [[self.control_info.test_setting['goal']['x'][0], self.control_info.test_setting['goal']['y'][0]],
             [self.control_info.test_setting['goal']['x'][1], self.control_info.test_setting['goal']['y'][1]]],
            [observation.vehicle_info['ego']['x'], observation.vehicle_info['ego']['y']]
        ):
            status = 1
            logger.debug(f"(CODE-1): 测试车成功抵达目标区域")

        observation.test_info['end'] = status
        return observation

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from utils.scenarioManager import format_scenario_info
from utils.recorder import Recorder
from utils.functions import check_action
from .controller import ReplayController

def run(mode_config: dict, planner: object, scene_info: dict) -> None:
    controller = ReplayController(mode_config['visualize'])
    recorder = Recorder()

    action = [0, 0]

    # 回放测试控制器初始化，并返回主车第一帧信息
    observation = controller.init(scene_info)
    # 被测物根据场景信息进行初始化设置
    planner.init(format_scenario_info(scene_info))

    while True:
        observation = controller.update_frame(observation)
        recorder.record(action, observation)
        if observation.test_info['end'] != -1:
            recorder.output(scene_info['output_path'])
            break
        new_action = planner.act(observation)
        action = check_action(observation.test_info['dt'], action, new_action)
        observation = controller.update_ego(action, observation)

if __name__ == '__main__':
    run('serial', {'tasks': ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']})
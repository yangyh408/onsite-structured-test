from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.recorder import Recorder
from utils.functions import check_action

from .ReplayController import ReplayController

def run(mode_config: dict, planner: object, scene_info: ScenarioInfo) -> None:
    controller = ReplayController(mode_config['visualize'])
    recorder = Recorder()

    action = [0, 0]

    # 回放测试控制器初始化，并返回主车第一帧信息
    controller.init(scene_info)
    # 被测物根据场景信息进行初始化设置
    planner.init(scene_info.format())

    while True:
        controller.update_frame()
        recorder.record(controller.get_observation())
        if controller.observation.test_info['end'] != -1:
            recorder.output(scene_info.output_path)
            break
        new_action = planner.act(controller.get_observation())
        action = check_action(scene_info.task_info['dt'], controller.observation.ego_info.v, action, new_action)
        controller.update_ego(action)

if __name__ == '__main__':
    run('serial', {'tasks': ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']})
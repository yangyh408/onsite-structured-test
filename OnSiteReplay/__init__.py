import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from utils.scenarioManager import scenarioManager
from utils.recorder import Recorder
from controller import ReplayController

def run(mode_config: dict, PLANNER: object) -> None:
    controller = ReplayController(mode_config['visualize'])
    recorder = Recorder()
    planner = PLANNER()

    sm = scenarioManager(mode='REPLAY', tasks=mode_config.get('tasks', []), skip_exist=mode_config.get('skipExist', False), print_info=False)
    while sm.next():
        # 记录模块初始化
        action = [0, 0]
        recorder.init()
        # 回放测试控制器初始化，并返回主车第一帧信息
        observation = controller.init(sm.cur_scene)
        # sm._print(is_print=True)
        # 被测物根据场景信息进行初始化设置
        planner.init(sm.current_scene_info())

        while True:
            observation = controller.update_frame(observation)
            recorder.record(action, observation)
            if observation.test_info['end'] != -1:
                output_dir = os.path.abspath(os.path.join(os.path.abspath(__file__), '..', '..', 'outputs'))
                output_name = f"{sm.cur_scene['scenarioType']}_{sm.cur_scene['scenarioNum']}_{sm.cur_scene['scenarioName']}_result.csv"
                recorder.output(os.path.join(output_dir, output_name))
                break
            action = planner.act(observation)
            observation = controller.update_ego(action, observation)

if __name__ == '__main__':
    run('serial', {'tasks': ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']})
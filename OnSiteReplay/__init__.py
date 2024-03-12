from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.recorder import Recorder
from utils.functions import check_action

from .ReplayController import ReplayController

def run(mode_config: dict, planner: object, scene_info: ScenarioInfo) -> None:
    # 实例化回放测试流程控制模块
    controller = ReplayController(mode_config['visualize'])
    # 实例化测试记录模块
    recorder = Recorder()
    # 初始化控制量（加速度、前轮转角）
    action = [0, 0]
    
    # 回放测试流程控制模块初始化，在observation中加载主车信息及测试环境信息
    controller.init(scene_info)
    # 被测物根据场景信息进行初始化设置
    planner.init(scene_info.format())

    while True:
        # 更新测试场景背景要素状态及仿真运行状态
        controller.update_frame()
        # 记录当前测试信息
        recorder.record(controller.get_observation())
        # 判断仿真是否还在进行中
        if controller.observation.test_info['end'] != -1:
            # 如果仿真结束则输出记录的测试信息
            recorder.output(scene_info.output_path)
            break
        # 如果仿真还在运行，则获取规控模块回传的控制信息
        new_action = planner.act(controller.get_observation())
        # 对规控器回传的控制信息进行执行器动力学约束修正
        action = check_action(scene_info.task_info['dt'], action, new_action)
        # 根据修正后的控制量更新主车位置
        controller.update_ego(action)

if __name__ == '__main__':
    run('serial', {'tasks': ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']})
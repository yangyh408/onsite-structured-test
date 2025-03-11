import os
import json

from ..DockWidget import *
from ..DLLs.Tessng import *
from ..createTess.opendrive2tess import opendrive2tess

from utils.ScenarioManager import select_scenario_manager
from utils.functions import kill_process

# 仿真测试模块
class MySimulatorCreateTess(QObject, PyCustomerSimulator):
    signalRunInfo = Signal(str)
    forStopSimu = Signal()
    forReStartSimu = Signal()
    forPauseSimu = Signal()

    def __init__(self, config: dict):
        QObject.__init__(self)
        PyCustomerSimulator.__init__(self)

        iface = tessngIFace()
        simuiface = iface.simuInterface()
        netiface = iface.netInterface()

        self.scenario_manager = select_scenario_manager(mode='FRAGMENT', config=config)

        failed_tasks = []

        for scene_name in self.scenario_manager.tasks_without_tess:
            task_dir = os.path.join(self.scenario_manager.task_dir, scene_name)
            new_tess_path = os.path.join(task_dir, f"{scene_name}.tess")
            try:
                netiface.createEmptyNetFile(new_tess_path)
                netiface.openNetFle(new_tess_path)
                parms = {
                    "file_path": self.scenario_manager._find_file_with_suffix(task_dir, 'xodr'),
                    "step_length": 1,
                    "lane_types": ["机动车道", "非机动车道", "人行道", "应急车道"]
                    }
                opendrive2tess(netiface, parms)
                netiface.saveRoadNet()
            except Exception as e:
                failed_tasks.append(new_tess_path)
                print(f"Error when creating tess in {scene_name}: {repr(e)}")
            
        print('*'*50)
        print(f"SUCCESS CREATE TESS {len(self.scenario_manager.tasks_without_tess) - len(failed_tasks)}/{len(self.scenario_manager.tasks_without_tess)}")
        if failed_tasks:
            temp_dir = os.path.join(os.path.dirname(__file__), '../../temp/')
            if not os.path.exists(temp_dir):
                os.makedirs(temp_dir)
            with open(os.path.join(temp_dir, 'create_failed.json'), 'w') as f:
                json.dump(failed_tasks, f)
        print('*'*50)

        kill_process(os.getpid())
        return 

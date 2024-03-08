import os

from .ScenarioManagerBase import ScenarioManagerBase

class ScenarioManagerForReplay(ScenarioManagerBase):
    def __init__(self, scenario_dir: str, config:dict):
        super().__init__(config)

        self.scenario_type = "REPLAY"
        self.task_dir = os.path.abspath(scenario_dir)
        tasks = config.get('tasks', [])
        if tasks:
            for scene_name in tasks:
                if os.path.exists(os.path.join(self.task_dir, scene_name)):
                    self.tasks.append(scene_name)
                else:
                    print(f"[LOAD SENARIO ERROR]: Cannot find task {scene_name}, please check the task name and retry!")
        else:
            for scene_name in os.listdir(self.task_dir):
                if not scene_name.startswith('.') and scene_name != '__pycache__' and os.path.isdir(os.path.join(self.task_dir, scene_name)):
                    self.tasks.append(scene_name)
        self.tot_scene_num = len(self.tasks)

    def _struct_scene_info(self):
        scene_dir = os.path.join(self.task_dir, self.tasks[self.cur_scene_num])
        output_name = f"{self.scenario_type}_{self.cur_scene_num}_{self.tasks[self.cur_scene_num]}_result.csv"
        scene_info = {
            'scenarioNum': self.cur_scene_num,
            'scenarioName': self.tasks[self.cur_scene_num],
            'scenarioType': self.scenario_type,
            'xodr_file_path': self._find_file_with_suffix(scene_dir, '.xodr'),
            'xosc_file_path': self._find_file_with_suffix(scene_dir, '.xosc'),
            'json_file_path': self._find_file_with_suffix(scene_dir, '.json'),
            'output_path': os.path.join(self.output_path, output_name),
            'startPos': None,
            'targetPos': None,
            'dt': None,
        }
        return scene_info

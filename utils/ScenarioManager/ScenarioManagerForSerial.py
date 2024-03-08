import os
import json

from .ScenarioManagerBase import ScenarioManagerBase

class ScenarioManagerForSerial(ScenarioManagerBase):
    def __init__(self, scenario_dir: str, config: dict):
        super().__init__(config)

        self.scenario_type = "SERIAL"
        self.map_dir = os.path.join(scenario_dir, 'maps')
        self.task_dir = os.path.join(scenario_dir, 'tasks')

        tasks = config.get('tasks', [])
        self.dt = config.get('dt', 0.05)

        if tasks:
            for task in tasks:
                if os.path.exists(os.path.join(self.task_dir, f"{task}.json")):
                    self.tasks.append(task)
                else:
                    print(f"[LOAD SCENARIO ERROR]: Cannot find task {task}, please check the task name and retry!")
        else:
            for task_name in os.listdir(self.task_dir):
                if not task_name.startswith('.') and task_name.endswith('.json'):
                    self.tasks.append(task_name.split('.json')[0])
        self.tot_scene_num = len(self.tasks)
  
    def _struct_scene_info(self):
        with open(os.path.join(self.task_dir, f"{self.tasks[self.cur_scene_num]}.json"),'r', encoding='utf-8') as f:
            scene_json = json.load(f)
        map_path = os.path.join(self.map_dir, scene_json['map'])
        assert os.path.exists(map_path), f"Cannot find map folder {map_path}, please download the map first!"
        output_name = f"{self.scenario_type}_{self.cur_scene_num}_{self.tasks[self.cur_scene_num]}_result.csv"
        scene_info = {
            'scenarioNum': self.cur_scene_num,
            'scenarioName': self.tasks[self.cur_scene_num].split('.json')[0],
            'scenarioType': self.scenario_type,
            'tess_file_path': self._find_file_with_suffix(map_path, '.tess'),
            'xodr_file_path': self._find_file_with_suffix(map_path, '.xodr'),
            'output_path': os.path.join(self.output_path, output_name),
            'startPos': scene_json['startPos'],
            'targetPos': scene_json['targetPos'],
            'waypoints': scene_json['waypoints'],
            'dt': self.dt,
        }
        return scene_info


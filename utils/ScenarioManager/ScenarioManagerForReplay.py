import os
import re
import xml.dom.minidom

from .ScenarioInfo import ScenarioInfo
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
        return ScenarioInfo(
            num = self.cur_scene_num,
            name = self.tasks[self.cur_scene_num],
            type = self.scenario_type,
            source_file = {
                "xodr": self._find_file_with_suffix(scene_dir, '.xodr'), 
                "xosc": self._find_file_with_suffix(scene_dir, '.xosc'), 
                "json": self._find_file_with_suffix(scene_dir, '.json'), 
                "tess": "",
                },
            output_path = os.path.join(self.output_path, output_name),
            task_info = self._parse_openscenario(self._find_file_with_suffix(scene_dir, '.xosc'))
        )
    
    def _parse_openscenario(self, file_dir: str):
        opens = xml.dom.minidom.parse(file_dir).documentElement

        ego_node = opens.getElementsByTagName('Private')[0]
        ego_init = ego_node.childNodes[3].data
        ego_v, ego_x, ego_y, ego_head = [float(i.split('=')[1]) for i in ego_init.split(',')]

        goal_init = ego_node.childNodes[5].data
        goal = [round(float(i), 3) for i in re.findall('-*\d+\.\d+', goal_init)]

        points = opens.getElementsByTagName('Act')[0].getElementsByTagName('Vertex')
        return {
            "startPos": [ego_x, ego_y], 
            "targetPos": [[goal[0], goal[2]], [goal[1], goal[3]]], 
            "waypoints": [], 
            "dt": round(float(points[1].getAttribute('time')) - float(points[0].getAttribute('time')), 3),
        }
import os
from .ScenarioInfo import ScenarioInfo

class ScenarioManagerBase():
    def __init__(self, config: dict):
        self.skip_exist = config.get('skipExist', False)
        self.record = {}

        self.scenario_type = ""
        self.tasks = []
        self.cur_scene_num = -1
        self.cur_scene = ScenarioInfo()
        self.output_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../outputs'))
        if not os.path.exists(self.output_path):
            os.makedirs(self.output_path)
        self.tot_scene_num = 0
 
    def next(self) -> bool:
        self.cur_scene_num += 1
        if self.cur_scene_num >= self.tot_scene_num:
            return False
        if self.skip_exist and self._is_exist(self.tasks[self.cur_scene_num]):
            return self.next()
        try:
            self.cur_scene = self._struct_scene_info()
            return True
        except Exception as e:
            print(repr(e))
            self.record[self.tasks[self.cur_scene_num]] = repr(e)
            return self.next()
        
    def _struct_scene_info(self) -> ScenarioInfo:
        pass

    def _is_exist(self, scenario_name: str) -> bool:
        for file in os.listdir(self.output_path):
            if self.scenario_type in file and f"_{scenario_name}_" in file:
                return True
        return False

    def _find_file_with_suffix(self, dir: str, suffix: str) -> str:
        for f in os.listdir(dir):
            if not f.startswith('.') and f.endswith(suffix):
                return os.path.join(dir, f)
        # print(f"[LOAD TASK ERROR]: Cannot find file with suffix \"{suffix}\" in {dir}!")
        return ""
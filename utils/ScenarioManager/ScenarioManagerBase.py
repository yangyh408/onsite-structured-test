import os

class ScenarioManagerBase():
    def __init__(self, config: dict):
        self.skip_exist = config.get('skipExist', False)
        self.print_info = False
        self.record = {}

        self.scenario_type = ""
        self.tasks = []
        self.cur_scene_num = -1
        self.cur_scene = {}
        self.output_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../outputs'))
        if not os.path.exists(self.output_path):
            os.makedirs(self.output_path)
 
    def next(self):
        self.cur_scene_num += 1
        if self.cur_scene_num >= self.tot_scene_num:
            return False
        if self.skip_exist and self._is_exist(self.tasks[self.cur_scene_num]):
            return self.next()
        try:
            self.cur_scene = self._struct_scene_info()
            self.record[self.cur_scene['scenarioName']] = 'Loaded Success'
            self._print(self.print_info)
            return True
        except Exception as e:
            print(repr(e))
            self.record[self.tasks[self.cur_scene_num]] = repr(e)
            return self.next()
        
    def current_scene_info(self):
        return {
            'scenarioNum': self.cur_scene.get('scenarioNum', -1),
            'scenarioName': self.cur_scene.get('scenarioName'),
            'scenarioType': self.cur_scene.get('scenarioType'),
            'xodr_file_path': self.cur_scene.get('xodr_file_path'),
            'startPos': self.cur_scene.get('startPos'),
            'targetPos': self.cur_scene.get('targetPos'),
            'waypoints': self.cur_scene.get('waypoints', []),
            'dt': self.cur_scene.get('dt'),
        }
        
    def _struct_scene_info(self):
        scene_info = {
            'scenarioName': self.tasks[self.cur_scene_num],
        }
        return scene_info

    def _is_exist(self, scenario_name):
        for file in os.listdir(self.output_path):
            if self.scenario_type in file and scenario_name in file:
                return True
        return False

    def _print(self, is_print: bool):
        if is_print:
            print('*'*100)
            for key, value in self.cur_scene.items():
                if key != 'waypoints' and key != 'vehicle_init_status':
                    print(f"   -{key:20s}: {value}")
            print('*'*100)

    def _find_file_with_suffix(self, dir: str, suffix: str):
        for f in os.listdir(dir):
            if not f.startswith('.') and f.endswith(suffix):
                return os.path.join(dir, f)
        # print(f"[LOAD TASK ERROR]: Cannot find file with suffix \"{suffix}\" in {dir}!")
        return ""
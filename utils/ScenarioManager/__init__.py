import os

from .ScenarioManagerForFragment import ScenarioManagerForFragment
from .ScenarioManagerForSerial import ScenarioManagerForSerial
from .ScenarioManagerForReplay import ScenarioManagerForReplay

def select_scenario_manager(task_dir: str, mode: str, config: dict):
    if mode == 'FRAGMENT':
        return ScenarioManagerForFragment(task_dir, config)
    elif mode == "SERIAL":
        return ScenarioManagerForSerial(task_dir, config)
    elif mode == "REPLAY":
        return ScenarioManagerForReplay(task_dir, config)
    else:
        raise RuntimeError("There is no valid mode, please choose in 'fragment' and 'serial'")
    
def format_scenario_info(scene_info: dict) -> dict:
    return {
        'scenarioNum': scene_info.get('scenarioNum', -1),
        'scenarioName': scene_info.get('scenarioName'),
        'scenarioType': scene_info.get('scenarioType'),
        'xodr_file_path': scene_info.get('xodr_file_path'),
        'startPos': scene_info.get('startPos'),
        'targetPos': scene_info.get('targetPos'),
        'waypoints': scene_info.get('waypoints', []),
        'dt': scene_info.get('dt'),
    }
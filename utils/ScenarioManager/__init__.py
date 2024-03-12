import os

from .ScenarioManagerForFragment import ScenarioManagerForFragment
from .ScenarioManagerForSerial import ScenarioManagerForSerial
from .ScenarioManagerForReplay import ScenarioManagerForReplay

def select_scenario_manager(mode: str, config: dict, task_dir: str = None):
    if not task_dir:
        scene_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../scenario')
        task_dir = os.path.join(scene_dir, mode.lower())
    if mode == 'FRAGMENT':
        return ScenarioManagerForFragment(task_dir, config)
    elif mode == "SERIAL":
        return ScenarioManagerForSerial(task_dir, config)
    elif mode == "REPLAY":
        return ScenarioManagerForReplay(task_dir, config)
    else:
        raise RuntimeError("There is no valid mode, please choose in 'fragment' and 'serial'")
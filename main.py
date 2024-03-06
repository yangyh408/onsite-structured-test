import yaml

import TessNG
import OnSiteReplay

from utils.scenarioManager import scenarioManager
from planner import PLANNER

def main():
    planner_in_loop = PLANNER()

    with open('./config/tasks.yaml', 'r') as f:
        tasks = yaml.safe_load(f)
    for mode, config in tasks.items():
        scenario_manager = scenarioManager(mode, config)
        while scenario_manager.next():
            if mode == 'REPLAY':
                OnSiteReplay.run(config, planner_in_loop, scene_info=scenario_manager.cur_scene)
            else:
                TessNG.run(mode, config, planner_in_loop, scene_info=scenario_manager.cur_scene)


if __name__ == '__main__':
    main()
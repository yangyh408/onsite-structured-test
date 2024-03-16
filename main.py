import os
import yaml
import time

import TessNG
import OnSiteReplay

from utils.ScenarioManager import select_scenario_manager
from utils.logger import logger
from planner import PLANNER

def main():
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    SCENARIO_DIR = os.path.join(BASE_DIR, 'scenario')

    with open('./config/tasks.yaml', 'r') as f:
        tasks = yaml.safe_load(f)
    for mode, config in tasks.items():
        scenario_manager = select_scenario_manager(mode, config)
        while scenario_manager.next():
            try:
                tic = time.time()
                if mode == 'REPLAY':
                    OnSiteReplay.run(config, PLANNER(), scene_info=scenario_manager.cur_scene)
                else:
                    TessNG.run(mode, config, PLANNER(), scene_info=scenario_manager.cur_scene)
                toc = time.time()
                if os.path.exists(scenario_manager.cur_scene.output_path):
                    logger.info(f"[{mode:8s}-{scenario_manager.cur_scene_num+1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene.name}> Test finished in {round(toc - tic, 1)}s.")
                else:
                    logger.error(f"[{mode:8s}-{scenario_manager.cur_scene_num+1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene.name}> Cannot locate correct output file!")
            except Exception as e:
                logger.critical(f"[{mode:8s}-{scenario_manager.cur_scene_num+1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene.name}> Test Collapse with error: {repr(e)}.")


if __name__ == '__main__':
    main()
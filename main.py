import os
import yaml
import time

import TessNG
import OnSiteReplay

from utils.scenarioManager import scenarioManager
from utils.logger import logger
from planner import PLANNER

def main():
    planner_in_loop = PLANNER()

    with open('./config/tasks.yaml', 'r') as f:
        tasks = yaml.safe_load(f)
    for mode, config in tasks.items():
        scenario_manager = scenarioManager(mode, config)
        while scenario_manager.next():
            try:
                tic = time.time()
                if mode == 'REPLAY':
                    OnSiteReplay.run(config, planner_in_loop, scene_info=scenario_manager.cur_scene)
                else:
                    TessNG.run(mode, config, planner_in_loop, scene_info=scenario_manager.cur_scene)
                toc = time.time()
                if os.path.exists(scenario_manager.cur_scene['output_path']):
                    logger.info(f"[{mode:8s}-{scenario_manager.cur_scene_num+1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene['scenarioName']}> Test finished in {round(toc - tic, 1)}s.")
                else:
                    logger.error(f"[{mode:8s}-{scenario_manager.cur_scene_num+1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene['scenarioName']}> Cannot locate correct output file!")
            except Exception as e:
                logger.critical(f"[{mode:8s}-{scenario_manager.cur_scene_num+1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene['scenarioName']}> Test Collapse with error: {repr(e)}.")

                


if __name__ == '__main__':
    main()
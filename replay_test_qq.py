import sys
import os
import time
import yaml

# sys.path.append('/Research/Onsite/onsite-structured-test')
from utils.opendrive2discretenet import parse_opendrive
from utils.ScenarioManager import select_scenario_manager
from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.observation import Observation, EgoStatus, ObjectStatus
from utils.visualizer import Visualizer
from utils.logger import logger

import TessNG
import OnSiteReplay

import numpy as np
import pandas as pd
from typing import Dict

from planner import PLANNER

if __name__ == '__main__':
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    with open('./config/tasks.yaml', 'r') as f:
        tasks = yaml.safe_load(f)
    for mode, config in tasks.items():
        if mode != 'REPLAY':
            continue
        scenario_manager = select_scenario_manager(mode, config)
        while scenario_manager.next():
            # try:
            tic = time.time()
            if mode == 'REPLAY':
                OnSiteReplay.run(config, PLANNER(), scene_info=scenario_manager.cur_scene)
            else:
                TessNG.run(mode, config, PLANNER(), scene_info=scenario_manager.cur_scene)
            toc = time.time()
            if os.path.exists(scenario_manager.cur_scene.output_path):
                logger.info(
                    f"[{mode:8s}-{scenario_manager.cur_scene_num + 1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene.name}> Test finished in {round(toc - tic, 1)}s.")
            else:
                logger.error(
                    f"[{mode:8s}-{scenario_manager.cur_scene_num + 1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene.name}> Cannot locate correct output file!")
            # except Exception as e:
            #     logger.critical(
            #         f"[{mode:8s}-{scenario_manager.cur_scene_num + 1:03d}/{len(scenario_manager.tasks):03d}] <{scenario_manager.cur_scene.name}> Test Collapse with error: {repr(e)}.")
    

    # vis = Visualizer()
    # result_path = r'.\outputs\REPLAY_0_scenario_3ca42d7a_result.csv'
    # save_path = r'.\outputs\REPLAY_0_scenario_3ca42d7a_result.gif'
    # vis.replay_result(result_path=result_path, save_path=save_path)

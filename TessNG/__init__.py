# -*- coding: utf-8 -*-
# 添加本开发包绝对路径到搜索路径中
import os
import sys
from multiprocessing import Process

from .MyPlugin import MyPlugin
from .TESS_API_EXAMPLE import *

from utils.ScenarioManager.ScenarioInfo import ScenarioInfo

def startTessNG(mode: str, mode_config: dict, planner: object, scene_info: ScenarioInfo, auto_run: bool) -> None:
    workspace_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WorkSpace')
    if not os.path.exists(workspace_dir):
        os.makedirs(workspace_dir)
    app = QApplication()
    config = {'__workspace': workspace_dir,
              '__simuafterload': auto_run,
              '__custsimubysteps': False
              }
    tess_file = scene_info.source_file['tess']
    if tess_file:
        config['__netfilepath'] = rf"{tess_file}"
    plugin = MyPlugin(mode, mode_config, planner, scene_info)
    factory = TessngFactory()
    tessng = factory.build(plugin, config)

    if tessng is None:
        sys.exit(0)
    else:
        sys.exit(app.exec_())

def run(mode: str, mode_config: dict, planner: object=None, scene_info: ScenarioInfo=ScenarioInfo(), auto_run: bool=True) -> None:
    tessng_p = Process(target=startTessNG, args=(mode, mode_config, planner, scene_info, auto_run))
    tessng_p.start()
    tessng_p.join()

if __name__ == '__main__':
    run('serial', {'tasks': ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']})
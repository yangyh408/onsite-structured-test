# -*- coding: utf-8 -*-
# 添加本开发包绝对路径到搜索路径中
from multiprocessing import Process

import os
import sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

from .MyPlugin import MyPlugin
from .TESS_API_EXAMPLE import *


def startTessNG(mode: str, mode_config: dict, planner: object, scene_info: dict, auto_run: bool) -> None:
    app = QApplication()

    config = {'__workspace': os.path.join(BASE_DIR, 'WorkSpace'),
              '__simuafterload': auto_run,
              '__custsimubysteps': False
              }
    tess_file = scene_info.get('tess_file_path')
    if tess_file:
        config['__netfilepath'] = rf"{tess_file}"
    plugin = MyPlugin(mode, mode_config, planner, scene_info)
    factory = TessngFactory()
    tessng = factory.build(plugin, config)

    if tessng is None:
        sys.exit(0)
    else:
        sys.exit(app.exec_())

def run(mode: str, mode_config: dict, planner: object=None, scene_info: dict={}, auto_run: bool=True) -> None:
    tessng_p = Process(target=startTessNG, args=(mode, mode_config, planner, scene_info, auto_run))
    tessng_p.start()
    tessng_p.join()

if __name__ == '__main__':
    run('serial', {'tasks': ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']})
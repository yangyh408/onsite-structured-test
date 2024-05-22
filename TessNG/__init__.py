# -*- coding: utf-8 -*-
# 添加本开发包绝对路径到搜索路径中
import os
import sys
from multiprocessing import Process

from .MyPlugin import MyPlugin
from .TESS_API_EXAMPLE import *

from utils.ScenarioManager.ScenarioInfo import ScenarioInfo

def startTessNG(mode: str, mode_config: dict, planner: object, scene_info: ScenarioInfo, auto_run: bool) -> None:
    # 创建工作目录
    workspace_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WorkSpace')
    if not os.path.exists(workspace_dir):
        os.makedirs(workspace_dir)
    # 启动TessNG并传入测试相关参数
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
    """启动TessNG进行测试
    Args:
        mode (str): 测试模式
        mode_config (dict): 测试配置
        planner (object, optional): 规划器对象. Defaults to None.
        scene_info (ScenarioInfo, optional): 测试场景信息. Defaults to ScenarioInfo().
        auto_run (bool, optional): 是否自动运行测试. Defaults to True.
    """
    tessng_p = Process(target=startTessNG, args=(mode, mode_config, planner, scene_info, auto_run))
    tessng_p.start()

    if mode == "FRAGMENT":
        timeout = mode_config.get('timeout', 600)
        tessng_p.join(timeout)
        if tessng_p.is_alive():
            tessng_p.terminate()
            raise TimeoutError(f"Timeout: TessNG process is still running after {timeout} seconds.")
    else:
        tessng_p.join()

if __name__ == '__main__':
    run('serial', {'tasks': ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']})
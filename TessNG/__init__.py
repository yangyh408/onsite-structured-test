# -*- coding: utf-8 -*-
# 添加本开发包绝对路径到搜索路径中
import json
import time
import signal
import _thread
from multiprocessing import Process

import os
import sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

from MyPlugin import MyPlugin
from TESS_API_EXAMPLE import *


def startTessNG(mode: str, mode_config: dict, planner: object):
    app = QApplication()

    config = {'__workspace': BASE_DIR,
              '__simuafterload': True,
              '__custsimubysteps': False
              }
    plugin = MyPlugin(mode, mode_config, planner)
    factory = TessngFactory()
    tessng = factory.build(plugin, config)

    if tessng is None:
        sys.exit(0)
    else:
        sys.exit(app.exec_())

def kill(targetPid):
    try:
        os.kill(targetPid, signal.SIGINT)
        print("Kill", targetPid)
        # a = os.kill(pid, signal.9) #　与上等效
    except OSError as e:
        print(e)

def checkTessngTest(tessngPid):
    pidDict = {"done": 0}
    # 存入上一次启动的进程
    with open("./cache.json", "w") as f:
        json.dump(pidDict, f)
    while True:
        try:
            with open("./cache.json", 'r') as load_f:
                load_dict = json.load(load_f)
            done = load_dict['done']
            if done == 1:
                print("Waiting for shutdown...")
                time.sleep(5)
                kill(tessngPid)
                break
            else:
                time.sleep(2)
        except FileNotFoundError:
            pass

def run(mode: str, mode_config: dict, planner: object) -> None:
    tessng_p = Process(target=startTessNG, args=(mode, mode_config, planner))
    tessng_p.start()
    pid = tessng_p.pid
    # 观察Tessng是否需要结束
    _thread.start_new_thread(checkTessngTest, (pid,))
    tessng_p.join()

if __name__ == '__main__':
    run('serial', {'tasks': ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']})
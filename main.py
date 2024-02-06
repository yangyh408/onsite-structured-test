# -*- coding: utf-8 -*-
import signal
from multiprocessing import Process
import _thread
import json
import time
import sys

from TessNG.MyPlugin import MyPlugin
from TessNG.TESS_API_EXAMPLE import *
from config.config import *

def startTessNG():
    app = QApplication()

    config = {'__workspace': os.path.join(ROOT_PATH, 'TessNG'),
            #   '__netfilepath': r"C:\Users\89525\Desktop\onsite_tessng\scenario\serial\maps\TJST_pure\TJST_pure.tess",
              '__simuafterload': True,
              '__custsimubysteps': False
              }
    plugin = MyPlugin()
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

if __name__ == '__main__':
    tessng_p = Process(target=startTessNG)
    tessng_p.start()
    pid = tessng_p.pid
    # 观察Tessng是否需要结束
    _thread.start_new_thread(checkTessngTest, (pid,))

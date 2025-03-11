# -*- coding: utf-8 -*-
from .TESS_API_EXAMPLE import *
from .MyNet import MyNet
from .MySimulator import select_simulator

# 用户插件，继承自TessPlugin
class MyPlugin(TessPlugin):
    def __init__(self, mode: str, config: dict, planner: object, scene_info: dict=None):
        super(MyPlugin, self).__init__()
        self.mNetInf = None
        self.mSimuInf = None
        self.mode = mode
        self.config = config
        self.planner = planner
        self.scene_info = scene_info

    def initGui(self):
        # 在TESS NG主界面上增加 QDockWidget对象
        self.examleWindow = TESS_API_EXAMPLE()

        iface = tessngIFace()
        win = iface.guiInterface().mainWindow()

        dockWidget = QDockWidget("自定义与TESS NG交互界面", win)
        dockWidget.setObjectName("mainDockWidget")
        dockWidget.setFeatures(QDockWidget.NoDockWidgetFeatures)
        dockWidget.setAllowedAreas(Qt.LeftDockWidgetArea)
        dockWidget.setWidget(self.examleWindow.centralWidget())
        iface.guiInterface().addDockWidgetToMainWindow(Qt.DockWidgetArea(1), dockWidget)

        # 增加菜单及菜单项
        menuBar = iface.guiInterface().menuBar()
        menu = QMenu(menuBar)
        menu.setObjectName("menuExample")
        menuBar.addAction(menu.menuAction())
        menu.setTitle("范例菜单")
        actionOk = menu.addAction("范例菜单项")
        actionOk.setCheckable(True)
        actionOk.triggered.connect(self.examleWindow.isOk)

        menuBar = iface.guiInterface().menuBar()
        menu = QMenu(menuBar)
        menu.setObjectName("Onsite")
        menuBar.addAction(menu.menuAction())
        menu.setTitle("Onsite")
        actionOk = menu.addAction("加载路径配置文件")
        actionOk.setCheckable(True)
        actionOk.triggered.connect(self.examleWindow.isOnSiteOk)

    # 过载父类方法，在 TESS NG工厂类创建TESS NG对象时调用
    def init(self):
        self.initGui()
        self.mNetInf = MyNet()
        self.mSimuInf = select_simulator(self.mode, self.config, self.planner, self.scene_info)
        self.mSimuInf.signalRunInfo.connect(self.examleWindow.showRunInfo)
        iface = tngIFace()
        win = iface.guiInterface().mainWindow()
        # 将信号mSimuInf.forReStopSimu关联到主窗体的槽函数doStopSimu，可以借安全地停止仿真运行
        self.mSimuInf.forStopSimu.connect(win.doStopSimu, Qt.QueuedConnection)
        # 将信号mSimuInf.forReStartSimu关联到主窗体的槽函数doStartSimu，可以借此实现自动重复仿真
        self.mSimuInf.forReStartSimu.connect(win.doStartSimu, Qt.QueuedConnection)
        # 将信号mSimuInf.forReStartSimu关联到主窗体的槽函数doStartSimu，可以借此实现自动重复仿真
        self.mSimuInf.forPauseSimu.connect(win.pauseMode, Qt.QueuedConnection)

    # 过载父类方法，返回插件路网子接口，此方法由TESS NG调用
    def customerNet(self):
        return self.mNetInf

    # 过载父类方法，返回插件仿真子接口，此方法由TESS NG调用
    def customerSimulator(self):
        return self.mSimuInf

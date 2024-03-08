# -*- coding: utf-8 -*-

import os
from pathlib import Path

from .DockWidget import *
from .DLLs.Tessng import *

from utils.netStruct import waypoints, startEndPos

class TESS_API_EXAMPLE(QMainWindow):
    def __init__(self, parent=None):
        super(TESS_API_EXAMPLE, self).__init__(parent)
        self.ui = Ui_TESS_API_EXAMPLEClass()
        self.ui.setupUi(self)
        self.createConnect()

    def createConnect(self):
        self.ui.btnOpenNet.clicked.connect(self.openNet)
        self.ui.btnStartSimu.clicked.connect(self.startSimu)
        self.ui.btnPauseSimu.clicked.connect(self.pauseSimu)
        self.ui.btnStopSimu.clicked.connect(self.stopSimu)
        self.ui.showOnSite.clicked.connect(self.showOnSite)
        self.ui.showWayPoints.clicked.connect(self.showWayPoints)

    def openNet(self):
        iface = tngIFace()
        if not iface:
            return
        if iface.simuInterface().isRunning():
            QMessageBox.warning(None, "提示信息", "请先停止仿真，再打开路网")
            return
        custSuffix = "TESSNG Files (*.tess);;TESSNG Files (*.backup)"
        dbDir = os.fspath(Path(__file__).resolve().parent / "Data")
        selectedFilter = "TESSNG Files (*.tess)"
        options = QFileDialog.Options(0)
        netFilePath, filtr = QFileDialog.getOpenFileName(self, "打开文件", dbDir, custSuffix, selectedFilter, options)

    def startSimu(self):
        iface = tngIFace()
        if not iface:
            return
        if not iface.simuInterface().isRunning() or iface.simuInterface().isPausing():
            iface.simuInterface().startSimu()

    def pauseSimu(self):
        iface = tngIFace()
        if not iface:
            return
        if iface.simuInterface().isRunning():
            iface.simuInterface().pauseSimu()

    def stopSimu(self):
        iface = tngIFace()
        if not iface:
            return
        if iface.simuInterface().isRunning():
            iface.simuInterface().stopSimu()

    def showRunInfo(self, runInfo):
        self.ui.txtMessage.clear()
        self.ui.txtMessage.setText(runInfo)

    def isOk(self):
        QMessageBox.information(None, "提示信息", "is ok!")

    def isOnSiteOk(self):
        QMessageBox.information(None, "待开发", "路径诱导模块待开发中...")

    def showOnSite(self):
        iface = tngIFace()
        netiface = iface.netInterface()
        scene = netiface.graphicsScene()
        pixmap = QPixmap()
        pixmap.load("./src/ONSITE.png")
        pixmap = pixmap.scaled(QSize(p2m(459), p2m(104)))
        item = scene.addPixmap(pixmap)
        item.setPos(QPoint(200, 200))

    def showWayPoints(self):
        iface = tngIFace()
        netiface = iface.netInterface()
        scene = netiface.graphicsScene()
        if waypoints["waypoints"]:
            waypoint = list(waypoints["waypoints"].values())
            for point in waypoint:
                qPoint = QPoint(p2m(point[0]), -p2m(point[1]))
                outline_color = QColor(0, 255, 255)
                circle = QGraphicsEllipseItem(p2m(qPoint.x())-1, p2m(qPoint.y())-1, m2p(1), m2p(1))  # (x, y, width, height)
                circle.setPen(outline_color)
                circle.setBrush(outline_color)
                circle.setZValue(200)
                scene.addItem(circle)
                
        # print(startEndPos)
        # if startEndPos["startPos"]:
        #     print(startEndPos["startPos"])
        #     qPoint = QPoint(p2m(startEndPos["startPos"][0]), -p2m(startEndPos["startPos"][1]))
        #     outline_color = QColor(255, 0, 0)
        #     circle = QGraphicsEllipseItem(p2m(qPoint.x()), p2m(qPoint.y()), m2p(1), m2p(1))  # (x, y, width, height)
        #     circle.setPen(outline_color)
        #     circle.setBrush(outline_color)
        #     circle.setZValue(300)
        #     scene.addItem(circle)

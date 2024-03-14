# -*- coding: utf-8 -*-
from PySide2.QtCore import Qt, QPoint
from PySide2.QtGui import QPen, QFont, QColor, QBrush
from PySide2.QtWidgets import QGraphicsTextItem, QGraphicsRectItem, QGraphicsEllipseItem

from utils.netStruct import *

from .DLLs.Tessng import PyCustomerNet
from .DLLs.Tessng import tessngIFace, p2m
from .DLLs.Tessng import NetItemType, GraphicsItemPropName

import numpy as np

# 用户插件子类，代表用户自定义与路网相关的实现逻辑，继承自MyCustomerNet
class MyNet(PyCustomerNet):
    def __init__(self):
        super(MyNet, self).__init__()
        self.testFinishNum = 0
        # 任务失败锁
        self.failedLock = 0
        self.crashLock = 0
        self.draw_static = False

    def beforeLoadNet(self) -> None:
        pass

    def afterLoadNet(self):
        # 代表TESS NG的接口
        iface = tessngIFace()
        # 代表TESS NG的路网子接口
        netiface = iface.netInterface()
        # 获取路段数
        count = netiface.linkCount()
        netiface.buildNetGrid(width=4)
        self.failedLock = 0
        self.crashLock = 0

        outSide["outSide"] = False
        paintPos["pos"] = []
        # startEndPos["startPos"] = None
        # startEndPos["endPos"] = None
        crash["crash"] = False
        self.draw_static = False

        '''获取路网中所有Link与Connector之间的拓扑关系'''
        links = netiface.links()
        connectors = netiface.connectors()

        for link in links:
            netTopo_table[link.id()] = [0, 0]  # 分别代表前导连接器与后续连接器ID。如果断了结束了，就是0。Link.id(): [前面连接段ID, 后面连接段ID]
        for connector in connectors:  # 遍历所有连接段
            preLink = connector.fromLink()
            nextLink = connector.toLink()
            temp = [preLink.id(), "c" + str(connector.id()), nextLink.id()]
            netTopo_table[preLink.id()][1] = connector.id()
            netTopo_table[nextLink.id()][0] = connector.id()
            connectionRelationship.append(temp)

        for conn in connectionRelationship:
            node1, _, node2 = conn
            if node1 in graph:
                graph[node1].append(node2)
            else:
                graph[node1] = [node2]

        for key, val in netTopo_table.items():  # 这里的key是路段的ID，val是前后的连接段ID[from,to]
            # print(key, val)
            if not val[0] and val[1]:  # 没有前导连接器但有后续连接器的Link即是起始Link        可不可以用[linkID, fromWhatConn, toWhatConn]
                startLinkID.append(key)
            if val[0] and not val[1]:
                endLinkId.append(key)
        # print("start joint", startLinkID, '++++++++++')
        # print("end joint", endLinkId, '++++++++++')

    # 是否允许用户对路网元素的绘制进行干预，如选择路段标签类型、确定绘制颜色等，本方法目的在于减少不必要的对python方法调用频次
    def isPermitForCustDraw(self):
        # 代表TESS NG的接口
        iface = tessngIFace()
        netface = iface.netInterface()
        netFileName = netface.netFilePath()
        if "Temp" in netFileName:
            return True
        else:
            return False

    def ref_labelNameAndFont(self, itemType, itemId, ref_outPropName, ref_outFontSize):
        # 代表TESS NG的接口
        iface = tessngIFace()
        # 代表TESS NG仿真子接口
        simuiface = iface.simuInterface()
        # 如果仿真正在进行，设置ref_outPropName.value等于GraphicsItemPropName.None_，路段和车道都不绘制标签
        if simuiface.isRunning():
            ref_outPropName.value = GraphicsItemPropName.None_
            return
        # 默认绘制ID
        ref_outPropName.value = GraphicsItemPropName.Id
        # 标签大小为6米
        ref_outFontSize.value = 6
        # 如果是连接段一律绘制名称
        if itemType == NetItemType.GConnectorType:
            ref_outPropName.value = GraphicsItemPropName.Name
        elif itemType == NetItemType.GLinkType:
            if itemId == 1 or itemId == 5 or itemId == 6:
                ref_outPropName.value = GraphicsItemPropName.Name

    # 过载父类方法，是否绘制车道中心线
    def isDrawLaneCenterLine(self, laneId):
        return True

    # 过载父类方法，是否绘制路段中心线
    def isDrawLinkCenterLine(self, linkId):
        return True

    def paint(self, itemType: int, itemId: int, painter):
        iface = tessngIFace()

        if not self.draw_static:
            self.draw_static = True
            netiface = iface.netInterface()
            scene = netiface.graphicsScene()
            if startEndPos["startPos"]:
                # painter.setPen(myPen)
                # pointStartQPoint = QPoint(startEndPos["startPos"][0], -startEndPos["startPos"][1])
                # painter.drawPoint(pointStartQPoint)
                qPoint = QPoint(startEndPos["startPos"][0], -startEndPos["startPos"][1])
                # outline_color = QColor(0, 255, 255)
                circle = QGraphicsEllipseItem(qPoint.x(), qPoint.y(), 1, 1)  # (x, y, width, height)
                outlinePen = QPen(Qt.green)
                outlinePen.setWidth(2)
                circle.setPen(outlinePen)
                circle.setBrush(Qt.NoBrush)
                circle.setZValue(300)
                scene.addItem(circle)

            if startEndPos["endPos"]:
                # myPen2 = QPen()
                # myPen2.setWidth(4)
                # myPen2.setColor(Qt.red)
                # painter.setPen(myPen2)
                # pointFinishPoint = [
                #     1 / 2 * (startEndPos["endPos"][0][0] + startEndPos["endPos"][1][0]),
                #     1 / 2 * (startEndPos["endPos"][0][1] + startEndPos["endPos"][1][1])]
                # pointFinishQPoint = QPoint(pointFinishPoint[0], -pointFinishPoint[1])
                # painter.drawPoint(pointFinishQPoint)
                # 从startEndPos获取矩形的起始点和结束点
                startPos = QPoint(startEndPos["endPos"][0][0], -startEndPos["endPos"][0][1])
                endPos = QPoint(startEndPos["endPos"][1][0], -startEndPos["endPos"][1][1])
                # 计算矩形的中心点
                centerPoint = QPoint(0.5 * (startPos.x() + endPos.x()), 0.5 * (startPos.y() + endPos.y()))
                # 计算矩形的宽度和高度
                width = abs(endPos.x() - startPos.x())
                height = abs(endPos.y() - startPos.y())
                # 创建矩形项
                rectangle = QGraphicsRectItem(centerPoint.x() - 0.5 * width, centerPoint.y() - 0.5 * height, width, height)
                # 设置矩形的边框画笔
                outlinePen = QPen(Qt.red)
                outlinePen.setWidth(1)
                rectangle.setPen(outlinePen)
                # 设置矩形的填充画刷（20%透明度的红色）
                # fillColor = QColor(255, 0, 0, 51)  # 51对应20%的透明度
                # fillBrush = QBrush(fillColor)
                rectangle.setBrush(Qt.NoBrush)
                rectangle.setZValue(300)
                # 将矩形项添加到图形场景中
                scene.addItem(rectangle)

        if paintPos["pos"]:
            myPen = QPen()
            myPen.setWidth(1)
            myPen.setColor(Qt.blue)
            painter.setPen(myPen)
            painter.drawLine(paintPos["pos"]["x"], -paintPos["pos"]["y"], paintPos["pos"]["x"]+5*np.cos(paintPos["pos"]["yaw"]), -paintPos["pos"]["y"]-5*np.sin(paintPos["pos"]["yaw"]))
            myPen = QPen()
            myPen.setWidth(3)
            myPen.setColor(Qt.yellow)
            painter.setPen(myPen)
            pos = paintPos["pos"]
            pointQPoint = QPoint(pos["x"], -pos["y"])
            painter.drawPoint(pointQPoint)

        if outSide["outSide"] and not self.failedLock:
            self.failedLock = 1
            netiface = iface.netInterface()
            scene = netiface.graphicsScene()
            failedItem = QGraphicsTextItem("算法车辆驶出路网，测试失败")
            failedItem.setPos(QPoint(0, 0))
            failedItem.setFont(QFont("黑体", 20))
            failedItem.setDefaultTextColor(QColor(255, 0, 0))
            scene.addItem(failedItem)

        if crash["crash"] and not self.crashLock:
            self.crashLock = 1
            netiface = iface.netInterface()
            scene = netiface.graphicsScene()
            failedItem = QGraphicsTextItem("算法车辆碰撞背景交通流，测试失败")
            failedItem.setPos(QPoint(0, 0))
            failedItem.setFont(QFont("黑体", 20))
            failedItem.setDefaultTextColor(QColor(255, 0, 0))
            scene.addItem(failedItem)
        return False
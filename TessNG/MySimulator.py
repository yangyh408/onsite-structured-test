import math
import time
import _thread
import sys
import json
import os

import numpy as np
from typing import Tuple
from onsite.observation import Observation
from onsite.recorder import DataRecord

from PySide2.QtCore import *
from DockWidget import *
from pathlib import Path
import Tessng
from Tessng import *

from utils.config import *
from utils.functions import calcDistance, convertAngle, judgeAcc, testFinish, getVehicleInfo
from utils.AutoSelfDrivingCar import Car
from OnSitePlanner.idm import IDM
from Lattice.idm import IDM_Lattic
from Lattice.lattice import alg_1

from OnSitePlanner.manual_controller import ManualController


def shouldIgnoreCrossPoint(pIVehicle: Tessng.IVehicle, curVehicleToStartPointDistance: float,
						   fromStartToCrossPointDistance: float) -> bool:
	iface = tessngIFace()
	simuiface = iface.simuInterface()
	curVehicleLength = pIVehicle.length()
	laneConnector = pIVehicle.laneConnector()
	# 如果当前车辆正在通过或者已经通过交叉点则不考虑当前交叉点
	if (curVehicleToStartPointDistance + (curVehicleLength / 2)) >= fromStartToCrossPointDistance:
		return True
	# 考虑到通行效率如果当前车辆距离交叉点距离大于3则也不考虑
	elif fromStartToCrossPointDistance - (curVehicleToStartPointDistance + curVehicleLength / 2) > 3:
		return True
	else:
		pass
	isClosest = True
	# 判断当前车辆是不是距离交叉点最近的车辆，如果不是直接跟车就行
	curLaneVehicles = simuiface.vehisInLaneConnector(laneConnector.connector().id(),
													 laneConnector.fromLane().id(),
													 laneConnector.toLane().id())
	for vehicle in curLaneVehicles:
		if vehicle.id() == pIVehicle.id():
			continue
		vehicleToStartPointDistance = laneConnector.distToStartPoint(vehicle.pos())
		if vehicleToStartPointDistance + vehicle.length() / 2 >= fromStartToCrossPointDistance:
			continue
		elif vehicleToStartPointDistance > curVehicleToStartPointDistance:
			isClosest = False
			break
	if not isClosest:
		return True
	return False


def shouldAvoidCrossLaneVehicle(crossPoint: Tessng.Online.CrossPoint, curVehicleToCrossPointDistance: float):
	iface = tessngIFace()
	simuiface = iface.simuInterface()
	crossLane = crossPoint.mpLaneConnector
	crossLaneCrossToStartPointDistance = crossLane.distToStartPoint(crossPoint.mCrossPoint)
	crossLaneVehicles = simuiface.vehisInLaneConnector(crossLane.connector().id(),
													   crossLane.fromLane().id(),
													   crossLane.toLane().id())
	pClosestVehicle = None
	closetVehicleToCrossPointDistance = 0
	maxStartToCrossPointDistance = 0
	hasCrossingCarInCrossLane = False
	for vehicle in crossLaneVehicles:
		vehicleLength = vehicle.length()
		vehicleToStartPointDistance = crossLane.distToStartPoint(vehicle.pos())
		if vehicleToStartPointDistance - vehicleLength / 2 > crossLaneCrossToStartPointDistance:
			continue
		elif vehicleToStartPointDistance + vehicleLength / 2 < crossLaneCrossToStartPointDistance:
			if vehicleToStartPointDistance > maxStartToCrossPointDistance:
				maxStartToCrossPointDistance = vehicleToStartPointDistance
				pClosestVehicle = vehicle
				closetVehicleToCrossPointDistance = crossLaneCrossToStartPointDistance - vehicleToStartPointDistance - vehicleLength / 2
		else:
			hasCrossingCarInCrossLane = True
			break
	if hasCrossingCarInCrossLane:
		return True
	if pClosestVehicle:
		if pClosestVehicle.currSpeed() <= 1:
			if curVehicleToCrossPointDistance > closetVehicleToCrossPointDistance:
				return True
	return False


def updateSimuStatus(MySimu):
	iface = tessngIFace()
	simuiface = iface.simuInterface()
	while True:
		finishTest = sys.modules["__main__"].__dict__['finishTest']
		# print("测试状态为", finishTest, "仿真是否正在运行", simuiface.isRunning(), "仿真是否暂停", simuiface.isPausing(), "仿真批次", simuiface.batchNumber())
		# print("MySimu信息为", "ego坐标", MySimu.EgoPos, "maxTestTime", MySimu.maxTestTime, "preheatingTime", MySimu.preheatingTime)
		if finishTest and (simuiface.isRunning() and not simuiface.isPausing()):
			MySimu.forStopSimu.emit()
			while True:
				# 检查 tessng 是否成功停止
				if not simuiface.isRunning():
					# sys.modules["__main__"].__dict__['finishTest'] = False
					break
				time.sleep(0.5)

		else:
			time.sleep(0.5)


class Recorder():
	def __init__(self):
		self.scene_num = 0
		self.scene_name = None
		self.end_status = None
		self.init()

	def init(self):
		self.scene_num += 1
		self.scene_name = ''
		self.end_status = -1
		self.data = DataRecord()

	def record(self, observation: Observation):
		self.data.add_data(observation)
		self.end_status = observation.test_setting['end']
		if not self.scene_name:
			self.scene_name = '_'.join(observation.test_setting['scenario_name'].split('.')[:-1])

	def output(self):
		data_output = self.data.merge_frame()
		# 增加结束状态一列
		data_output.loc[:, 'end'] = -1
		data_output.iloc[-1, -1] = self.end_status
		data_output.to_csv(rf'.\outputs\{self.scene_num}_{self.scene_name}_result.csv')


class Recorder():
    def __init__(self):
        self.scene_num = 0
        self.scene_name = None
        self.end_status = None
        self.init()

    def init(self):
        self.scene_num += 1
        self.scene_name = ''
        self.end_status = -1
        self.data = DataRecord()

    def record(self, observation: Observation):
        self.data.add_data(observation)
        self.end_status = observation.test_setting['end']
        if not self.scene_name:
            self.scene_name = '_'.join(observation.test_setting['scenario_name'].split('.')[:-1])
    
    def output(self):
        data_output = self.data.merge_frame()
         # 增加结束状态一列
        data_output.loc[:, 'end'] = -1
        data_output.iloc[-1, -1] = self.end_status
        data_output.to_csv(rf'.\outputs\{self.scene_num}_{self.scene_name}_result.csv')

class MySimulator(QObject, PyCustomerSimulator):
	signalRunInfo = Signal(str)
	forStopSimu = Signal()
	forReStartSimu = Signal()
	forPauseSimu = Signal()

	def __init__(self):
		QObject.__init__(self)
		PyCustomerSimulator.__init__(self)
		# TODO: 选手在此实例化规控器
		# TODO: 修改控制器为键盘控制的ManualController, 有三种控制方法
		# self.planner = IDM()
		# self.planner = IDM_Lattic()
		self.planner = ManualController()
		# 测试场景数量
		self.testFinishNum = 0
		# 待测试场景
		self.testNum = len(TESSNG_File_Route_List)
		# 测试车的名字
		self.EgoName = EGO_Name
		# 测试车辆起点坐标
		self.EgoPos = [0, 0]
		# 测试车ID Name对应表
		self.EgoIndex = {}
		# 已被创建的测试车辆
		self.carIdAlreadyCreatedList = []
		# IDM反馈结果
		self.egoAct = []
		# 车辆创建对象锁
		self.lock = 0
		# 最大测试时长
		self.maxTestTime = 0
		# 仿真预热时间
		self.preheatingTime = 0
		# 是否完成测试
		self.finishTest = False
		sys.modules["__main__"].__dict__['finishTest'] = False
		# 车辆变道企图
		self.temp_attempt = None
		# 车辆变道企图记录
		self.Ego_Car_Attempt = {}
		# 测试对象列表
		self.egoList = []
		# 待移动点
		self.potentialPoint = {}
		# Ego 当前路段
		self.EgoLink = 0
		# Ego路径
		self.egoRoute = None
		# Ego OD
		self.o = []
		self.d = []
		self.startLane = None
		self.endLane = None
		self.startEdge = None
		self.endEdge = None
		# 启动监测线程
		_thread.start_new_thread(updateSimuStatus, (self,))
		# 引入记录模块recorder
		self.recorder = Recorder()
		# 是否需要TessNG完成路径规划
		self.routePlan = routePlan

	def ref_beforeStart(self, ref_keepOn):
		# 是否完成测试
		sys.modules["__main__"].__dict__['finishTest'] = False
		iface = tessngIFace()
		simuiface = iface.simuInterface()
		netiface = iface.netInterface()
		simuiface.setSimuAccuracy(1 / dt)
		# 仿真精度
		simuAccuracy = simuiface.simuAccuracy()
		print(f"accuracy {simuAccuracy} times per second")
		# 最大仿真测试时间
		self.maxTestTime = int(dt * calculateBatchesFinish)
		print(f"max test time {self.maxTestTime} seconds")
		self.preheatingTime = preheatingTime
		print(f"preheating time {self.preheatingTime} seconds")
		# 释放锁
		self.lock = 0
		# OD 点，作为轨迹评分
		self.o = EgoStartPosList[self.testFinishNum]
		self.d = [1 / 2 * (finishSectionList[self.testFinishNum][0][0] + finishSectionList[self.testFinishNum][1][0]),
				1 / 2 * (finishSectionList[self.testFinishNum][0][1] + finishSectionList[self.testFinishNum][1][1])]
		# 先定位到车道对象
		startEdgeList = netiface.locateOnCrid(QPointF(self.o[0], -self.o[1]), 9)
		endEdgeList = netiface.locateOnCrid(QPointF(self.d[0], -self.d[1]), 9)
		# 定位到路段/连接段
		self.startLane = startEdgeList[0].pLaneObject
		self.startEdge = self.findLinkConnId(self.startLane, netiface)
		self.endLane = endEdgeList[0].pLaneObject
		self.endEdge = self.findLinkConnId(self.endLane, netiface)
		return True

	@staticmethod
	def findLinkConnId(edge, netiface: Tessng.NetInterface):
		links = netiface.links()
		conns = netiface.connectors()
		if type(edge) == Tessng.ILaneConnector:
			for conn in conns:
				for conn_lane in conn.laneConnectors():
					if conn_lane.id() == edge.id():
						return conn.id()
		else:
			for link in links:
				for link_lane in link.lanes():
					if link_lane.id() == edge.id():
						return link.id()

	@staticmethod
	def shouldSlowDownInCrossroads(pIVehicle: Tessng.IVehicle, shouldAcce: bool) -> Tuple[bool, bool]:
		iface = tessngIFace()
		netiface = iface.netInterface()

		if not pIVehicle:
			return False, shouldAcce
		if not pIVehicle.roadIsLink() and pIVehicle.vehicleTypeCode() == 1:
			laneConnector = pIVehicle.laneConnector()
			if laneConnector:
				crossPoints = netiface.crossPoints(laneConnector)
				if crossPoints and len(crossPoints) > 0:
					curVehicleLength = pIVehicle.length()
					curVehicleToStartPointDistance = laneConnector.distToStartPoint(pIVehicle.pos())
					for crossPoint in crossPoints:
						fromStartToCrossPointDistance = laneConnector.distToStartPoint(crossPoint.mCrossPoint)
						curVehicleToCrossPointDistance = fromStartToCrossPointDistance - curVehicleToStartPointDistance - curVehicleLength / 2
						if shouldIgnoreCrossPoint(pIVehicle, curVehicleToCrossPointDistance,
												  fromStartToCrossPointDistance):
							continue
						if shouldAvoidCrossLaneVehicle(crossPoint, curVehicleToCrossPointDistance):
							return True, shouldAcce
					if p2m(pIVehicle.currSpeed()) < 3:
						if pIVehicle.vehicleFront() is None and p2m(pIVehicle.vehiDistFront() > 20):
							shouldAcce = True
		return False, shouldAcce

	def delVehicle(self, pIVehicle) -> bool:
		# 删除在指定消失路段的车辆
		if pIVehicle.roadId() in self.disappearRoadId and pIVehicle.name() != self.EgoName:
			return True
		else:
			return False

	# 控制车辆的变道
	def setEgoCarChangeLane(self, pIVehicle):
		# TODO:修改变道控制，原函数可能存在逻辑错误导致车辆行为异常
		# if self.Ego_Car_Attempt.get(pIVehicle.name()):
		#     attempt = self.Ego_Car_Attempt.get(pIVehicle.name())
		#     # 企图与上一次不同
		#     if attempt != self.temp_attempt:
		#         self.temp_attempt = attempt
		#         if attempt == "Right":
		#             pIVehicle.vehicleDriving().toRightLane(True)
		#         elif attempt == "Left":
		#             pIVehicle.vehicleDriving().toLeftLane(True)
		#         elif attempt == "Straight":
		#             pIVehicle.vehicleDriving().toRightLane(False)
		#             pIVehicle.vehicleDriving().toLeftLane(False)
		#             print(pIVehicle.name(), attempt, "直行直行！")
		#         else:
		#             pass
		#     elif attempt == self.temp_attempt and self.temp_attempt == "Straight":
		#         pIVehicle.vehicleDriving().toRightLane(False)
		#         pIVehicle.vehicleDriving().toLeftLane(False)
		#         print(pIVehicle.name(), attempt, "直行直行！")
		#     else:
		#         pIVehicle.vehicleDriving().toRightLane(False)
		#         pIVehicle.vehicleDriving().toLeftLane(False)
		#         print(pIVehicle.name(), attempt, "企图和上一次决策相同")
		if self.Ego_Car_Attempt.get(pIVehicle.name()):
			attempt = self.Ego_Car_Attempt.get(pIVehicle.name())
			# 企图与上一次不同
			if attempt != self.temp_attempt:
				self.temp_attempt = attempt
				if attempt == "Right":
					pIVehicle.vehicleDriving().toRightLane(True)
					print(pIVehicle.name(), "向右变道")
				elif attempt == "Left":
					pIVehicle.vehicleDriving().toLeftLane(True)
					print(pIVehicle.name(), "向左变道")

	# 收集航向角并获取分析车辆的变道企图
	def getHeader_VTDCarChangeLaneAttempt(self, pIVehicle):
		# TODO: 针对手动控制模块修改了变道企图的识别方式
		# for ego in self.egoList:
		#     if ego.name == pIVehicle.name():
		#         header = convertAngle(self.egoAct[1])
		#         ego.add_headerData(header)
		#         attempt = ego.check_monotonic()
		#         self.Ego_Car_Attempt[ego.name] = attempt
		if self.egoAct:
			for ego in self.egoList:
				if ego.name == pIVehicle.name():
					if self.egoAct[1] == 0:
						self.Ego_Car_Attempt[ego.name] = 'Straight'
					elif self.egoAct[1] == 90:
						self.Ego_Car_Attempt[ego.name] = 'Left'
					elif self.egoAct[1] == -90:
						self.Ego_Car_Attempt[ego.name] = 'Right'

	def tessngServerMsg(self, tessngSimuiface, vehicleAlreadyCreate, egoPos, currentBatchNum, currentTestTime):
		"""

		:param tessngSimuiface: TESSNG Simuiface接口
		:param vehicleAlreadyCreate: TESSNG中已经被创建的测试车辆Id
		:param egoPos: 测试车辆的坐标
		:param currentBatchNum: 当前TESSNG的仿真计算批次号
		:param currentTestTime: 当前TESSNG的仿真计算时间（单位：ms）
		:return: 返回给IDM的observation信息
		"""
		lAllVehiStatus = tessngSimuiface.getVehisStatus()
		vehicleInfo = {}
		vehicleTotal = Observation()
		if egoPos:
			for vehicleStatus in lAllVehiStatus:
				if calcDistance(egoPos, [p2m(vehicleStatus.mPoint.x()), -p2m(vehicleStatus.mPoint.y())]) < radius:
					if vehicleStatus.vehiId in vehicleAlreadyCreate:
						vehicleInfo[self.EgoIndex.get(vehicleStatus.vehiId)] = {'length': p2m(vehicleStatus.mrLength),
																				'width': p2m(vehicleStatus.mrWidth),
																				'x': p2m(vehicleStatus.mPoint.x()),
																				'y': -p2m(vehicleStatus.mPoint.y()),
																				'v': p2m(vehicleStatus.mrSpeed),
																				'a': judgeAcc(
																					p2m(vehicleStatus.mrAcce)),
																				'yaw': math.radians(convertAngle(
																					vehicleStatus.mrAngle))}
					else:
						vehicleInfo[vehicleStatus.vehiId] = {'length': p2m(vehicleStatus.mrLength),
															 'width': p2m(vehicleStatus.mrWidth),
															 'x': p2m(vehicleStatus.mPoint.x()),
															 'y': -p2m(vehicleStatus.mPoint.y()),
															 'v': p2m(vehicleStatus.mrSpeed),
															 'a': judgeAcc(p2m(vehicleStatus.mrAcce)),
															 'yaw': math.radians(convertAngle(vehicleStatus.mrAngle))}
				else:
					pass
			vehicleTotal.vehicle_info = vehicleInfo
			vehicleTotal.light_info = {'green'}
			# 判断是否到达了目的地
			file_name = os.path.basename(TESSNG_File_Route_List[self.testFinishNum])
			vehicleTotal.test_setting = {"scenario_name": f"{file_name}", "scenario_type": "TESSNG",
										 "max_t": self.maxTestTime,
										 "t": currentTestTime / 1000, "dt": dt,
										 "goal": {"x": [finishSectionList[self.testFinishNum][0][0],
														finishSectionList[self.testFinishNum][1][0]],
												  "y": [finishSectionList[self.testFinishNum][0][1],
														finishSectionList[self.testFinishNum][1][1]]},
										 "end": testFinish(rect=finishSectionList[self.testFinishNum], egoPos=egoPos,
														   currentBatchNum=currentBatchNum,
														   endBatchNum=calculateBatchesFinish),
										 "map_type": "testground", 'x_max': 2000.0, 'x_min': -2000.0, 'y_max': 2000,
										 'y_min': -2000}

			# file_name = "sssss"
			# vehicleTotal["test_setting"] = {"scenario_name": f"{file_name}", "scenario_type": "TESSNG", "max_t": self.maxTestTime,
			#                                 "t": currentTestTime, "dt": dt, "goal": {"x": [finishSection[0][0], finishSection[1][0]], "y": [finishSection[0][1], finishSection[1][1]]},
			#                                 "end": testFinish(rect=finishSection, egoPos=egoPos, currentBatchNum=currentBatchNum, endBatchNum=calculateBatchesFinish),
			#                                 "map_type": "testground", 'x_max': 2000.0, 'x_min': -2000.0, 'y_max': 2000, 'y_min': -2000}

			return vehicleTotal
		else:
			return {}

	def observationMsg(self, tessngSimuiface, vehicleAlreadyCreate, egoPos, currentBatchNum, currentTestTime):
		"""

		:param tessngSimuiface: TESSNG Simuiface接口
		:param vehicleAlreadyCreate: TESSNG中已经被创建的测试车辆Id
		:param egoPos: 测试车辆的坐标
		:param currentBatchNum: 当前TESSNG的仿真计算批次号
		:param currentTestTime: 当前TESSNG的仿真计算时间（单位：ms）
		:return: 返回给IDM的observation信息
		"""
		lAllVehiStatus = tessngSimuiface.getVehisStatus()
		vehicleInfo = {}
		vehicleTotal = {}
		if egoPos:
			for vehicleStatus in lAllVehiStatus:
				if calcDistance(egoPos, [p2m(vehicleStatus.mPoint.x()), -p2m(vehicleStatus.mPoint.y())]) < radius:
					if vehicleStatus.vehiId in vehicleAlreadyCreate:
						vehicleInfo[self.EgoIndex.get(vehicleStatus.vehiId)] = {'length': p2m(vehicleStatus.mrLength),
																				'width': p2m(vehicleStatus.mrWidth),
																				'x': p2m(vehicleStatus.mPoint.x()),
																				'y': -p2m(vehicleStatus.mPoint.y()),
																				'v': p2m(vehicleStatus.mrSpeed),
																				'a': judgeAcc(
																					p2m(vehicleStatus.mrAcce)),
																				'yaw': math.radians(convertAngle(
																					vehicleStatus.mrAngle))}
					else:
						vehicleInfo[vehicleStatus.vehiId] = {'length': p2m(vehicleStatus.mrLength),
															 'width': p2m(vehicleStatus.mrWidth),
															 'x': p2m(vehicleStatus.mPoint.x()),
															 'y': -p2m(vehicleStatus.mPoint.y()),
															 'v': p2m(vehicleStatus.mrSpeed),
															 'a': judgeAcc(p2m(vehicleStatus.mrAcce)),
															 'yaw': math.radians(convertAngle(vehicleStatus.mrAngle))}
				else:
					pass
			vehicleTotal["vehicle_info"] = vehicleInfo
			vehicleTotal["light_info"] = {'green'}
			# 判断是否到达了目的地
			file_name = os.path.basename(TESSNG_File_Route_List[self.testFinishNum])
			vehicleTotal["test_setting"] = {"scenario_name": f"{file_name}", "scenario_type": "TESSNG",
										 "max_t": self.maxTestTime,
										 "t": currentTestTime / 1000, "dt": dt,
										 "goal": {"x": [finishSectionList[self.testFinishNum][0][0],
														finishSectionList[self.testFinishNum][1][0]],
												  "y": [finishSectionList[self.testFinishNum][0][1],
														finishSectionList[self.testFinishNum][1][1]]},
										 "end": testFinish(rect=finishSectionList[self.testFinishNum], egoPos=egoPos,
														   currentBatchNum=currentBatchNum,
														   endBatchNum=calculateBatchesFinish),
										 "map_type": "testground", 'x_max': 2000.0, 'x_min': -2000.0, 'y_max': 2000,
										 'y_min': -2000}

			# file_name = "sssss"
			# vehicleTotal["test_setting"] = {"scenario_name": f"{file_name}", "scenario_type": "TESSNG", "max_t": self.maxTestTime,
			#                                 "t": currentTestTime, "dt": dt, "goal": {"x": [finishSection[0][0], finishSection[1][0]], "y": [finishSection[0][1], finishSection[1][1]]},
			#                                 "end": testFinish(rect=finishSection, egoPos=egoPos, currentBatchNum=currentBatchNum, endBatchNum=calculateBatchesFinish),
			#                                 "map_type": "testground", 'x_max': 2000.0, 'x_min': -2000.0, 'y_max': 2000, 'y_min': -2000}

			return vehicleTotal
		else:
			return {}

	def setStepsPerCall(self, vehi):
		pass

	# def initVehicle(self, pIVehicle):
	#     pIVehicle.setColor("#F8F8FF")

	# todo 不再手动删除车辆对象
	# def isStopDriving(self, pIVehicle: Tessng.IVehicle) -> bool:
	#     if self.delVehicle(pIVehicle):
	#         return True
	#     else:
	#         return False

	def ref_reSetAcce(self, vehi, inOutAcce):
		# print("加速度信息", self.egoAct)
		if vehi.name() == self.EgoName and self.egoAct:
			inOutAcce.value = self.egoAct[0]
			# print(f"IDM计算结果为{self.egoAct},加速度设置成功")
			return True
		else:
			return False

	def carCreateCar(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface, startPos, speed,
					 vehicleTypeCode):
		# 获取到车辆的x,y坐标
		lLocations = netiface.locateOnCrid(QPointF(startPos[0], -startPos[1]), 9)
		if lLocations:
			dvp = Online.DynaVehiParam()
			dvp.vehiTypeCode = vehicleTypeCode
			dvp.dist = lLocations[0].distToStart
			dvp.speed = m2p(speed)
			# 来自VTD的车，TESSNG车辆被创建后，名字与VTD车名字相同
			dvp.color = "#00BFFF"
			dvp.name = self.EgoName
			# 如果是路段
			if lLocations[0].pLaneObject.isLane():
				lane = lLocations[0].pLaneObject.castToLane()
				dvp.roadId = lane.link().id()
				dvp.laneNumber = lane.number()
			# 如果是连接段
			else:
				lane_connector = lLocations[0].pLaneObject.castToLaneConnector()
				dvp.roadId = lane_connector.connector().id()
				dvp.laneNumber = lane_connector.fromLane().number()
				dvp.toLaneNumber = lane_connector.toLane().number()
			vehi = simuiface.createGVehicle(dvp)
			if vehi:
				# print("创建成功")
				self.carIdAlreadyCreatedList.append(vehi.id())
				self.EgoIndex[vehi.id()] = vehi.name()
				ego = Car(id=vehi.id(), name=vehi.name(),
						  Xpos=p2m(vehi.pos().x()), Ypos=-p2m(vehi.pos().y()),
						  speed=p2m(vehi.currSpeed()), roadType=vehi.roadType(),
						  frameCount=5, threshold=0.11)
				# 创建计算对象
				self.egoList.append(ego)

	def moveSingleCar(self, vehicle, netiface: Tessng.NetInterface):
		pass

	# def ref_reCalcAngle(self, pIVehicle, ref_outAngle):
	# 	if pIVehicle.name() == self.EgoName:
	# 		# 根据IDM输出的转角来设置车辆角度
	# 		# TODO: 注释此处以使主车航向角始终与道路方向一致
	# 		ref_outAngle.value = convertAngle(self.egoAct[1])
	# 		return True

	# 实时获取Ego的坐标
	def getEgoPos(self, pIVehicle):
		vehicle_name = pIVehicle.name()
		if vehicle_name and vehicle_name == self.EgoName:
			self.EgoPos = [p2m(pIVehicle.pos().x()), -p2m(pIVehicle.pos().y())]
			self.EgoLink = pIVehicle.roadId()
			return

	@staticmethod
	def updateEgoPos(action: tuple, observation) -> dict:
		ego_info = {}
		# 分别取出加速度和前轮转向角
		a, rot = action
		# 取出步长
		_dt = observation.test_setting['dt']
		# 取出本车的各类信息
		try:
			x, y, v, yaw, width, length = [float(observation.vehicle_info['ego'][key]) for key in [
				'x', 'y', 'v', 'yaw', 'width', 'length']]

			ego_info['x'] = x + v * np.cos(yaw) * _dt  # 更新X坐标
			ego_info['y'] = y + v * np.sin(yaw) * _dt  # 更新y坐标
			ego_info['yaw'] = yaw + v / length * 1.7 * np.tan(rot) * _dt  # 更新偏航角
			ego_info['v'] = max(0, v + a * _dt)  # 更新速度
			ego_info['a'] = a  # 更新加速度
			ego_info['width'] = width
			ego_info['length'] = length
		except KeyError:
			pass
		return ego_info

	@staticmethod
	def isRoadLink(pIVehicle) -> bool:
		if pIVehicle.roadIsLink():
			return True
		else:
			return False

	def moveEgo(self, simuiface: Tessng.SimuInterface, netiface: Tessng.NetInterface):
		currentLinkLaneId = None
		currentLinkLaneConnectorFromLinkLaneId = None
		lAllVehi = simuiface.allVehiStarted()
		for vehicle in lAllVehi:
			if vehicle.name() == self.EgoName:
				if self.potentialPoint:
					vehicleShouldMoveToXy = [self.potentialPoint.get('x'), self.potentialPoint.get('y')]
					vehicleXPos, vehicleYPos = p2m(vehicleShouldMoveToXy[0]), -p2m(vehicleShouldMoveToXy[1])
					lLocations = netiface.locateOnCrid(QPointF(vehicleXPos, vehicleYPos), 9)
					if self.isRoadLink(vehicle):
						currentLinkLane = vehicle.lane()
						# 如果在路段上，获取到正在路段上的车道编号
						currentLinkLaneId = currentLinkLane.id()
					else:
						currentLinkLaneConnector = vehicle.lane()
						if type(currentLinkLaneConnector) == Tessng.ILaneConnector:
							currentLinkLaneConnectorFromLinkLane = currentLinkLaneConnector.fromLane()
							# 如果现在在连接段上，获取到正在连接段上的车道，并且获取到连接段车道的上游车道
							currentLinkLaneConnectorFromLinkLaneId = currentLinkLaneConnectorFromLinkLane.id()
					if len(lLocations) > 0:
						closestTarget = lLocations[0].pLaneObject
						for lLocation in lLocations:
							potentialTarget = lLocation.pLaneObject
							dist = lLocation.distToStart
							# 如果当前车已经在路段上了
							if currentLinkLaneId:
								# 并且下一帧的潜在移动目标是连接段，而且连接段车道的上游路段车道ID等于当前在的路段车道ID
								if type(potentialTarget) == Tessng.ILaneConnector:
									potentialTargetFromLane = potentialTarget.fromLane()
									if potentialTargetFromLane.id() == currentLinkLaneId:
										vehicle.vehicleDriving().move(potentialTarget, dist)
										print("从路段移动到连接段")
										break
								else:
									# 如果潜在目前还是路段，就直接移动
									vehicle.vehicleDriving().move(lLocations[0].pLaneObject, lLocations[0].distToStart)
									break
							# 如果当前车已经在连接段上，如何处理比较好?
							elif currentLinkLaneConnectorFromLinkLaneId is not None:
								# 如果潜在移动对象还是连接段
								if type(potentialTarget) == Tessng.ILaneConnector:
									potentialTargetFromLane = potentialTarget.fromLane()
									if potentialTargetFromLane.id() == currentLinkLaneConnectorFromLinkLaneId:
										if potentialTarget.id() == closestTarget.id():
											vehicle.vehicleDriving().move(potentialTarget, dist)
											print("在连接段上移动")
											break
										else:
											break
								else:
									break
							else:
								break
			else:
				pass

	def setRoute(self, pIVehicle, tessngInterFace, startLink, endLink):
		if pIVehicle.name() == self.EgoName:
			netiface = tessngInterFace.netInterface()
			start = netiface.findLink(startLink)
			end = netiface.findLink(endLink)
			# 如果本身对象不是路段，则找连接段的起始路段
			if not start:
				start = netiface.findConnector(startLink)
				start = start.toLink()
			if not end:
				end = netiface.findConnector(endLink)
				end = end.toLink()
			if start and end:
				egoRoute = netiface.shortestRouting(start, end)
				pIVehicle.vehicleDriving().setRouting(egoRoute)

	def paintMyVehicle(self, pIVehicle: Tessng.IVehicle):
		if pIVehicle.roadId() == self.EgoLink:
			if pIVehicle.name() != self.EgoName:
				if calcDistance(self.EgoPos, [p2m(pIVehicle.pos().x()), -p2m(pIVehicle.pos().y())]) < 10:
					# 变红
					pIVehicle.setColor("#FF0000")
				else:
					# 变白
					pIVehicle.setColor("#F8F8FF")
			else:
				# 主车变蓝
				pIVehicle.setColor("#00BFFF")
		else:
			# 变白
			pIVehicle.setColor("#F8F8FF")

	def afterStep(self, pIVehicle: Tessng.IVehicle) -> None:
		iface = tessngIFace()
		self.getEgoPos(pIVehicle)
		# 采集测试车的航向角
		self.getHeader_VTDCarChangeLaneAttempt(pIVehicle)
		self.setEgoCarChangeLane(pIVehicle)
		# todo 可以选择是否自己做路径规划
		if self.routePlan:
			self.setRoute(pIVehicle, iface, self.startEdge, self.endEdge)
		self.paintMyVehicle(pIVehicle)

	# 主要测试步骤和逻辑
	def mainStep(self, simuiface, netiface):
		simuTime = simuiface.simuTimeIntervalWithAcceMutiples()
		# 当前仿真计算批次
		batchNum = simuiface.batchNumber()
		if simuTime >= self.preheatingTime * 1000:
			if not self.lock:
				self.carCreateCar(simuiface, netiface, EgoStartPosList[self.testFinishNum], 10, EgoTypeCode)
				self.lock = 1
			observation = self.tessngServerMsg(simuiface, self.carIdAlreadyCreatedList, self.EgoPos, batchNum, simuTime)
			# 这个没有文件缓存，只用于给选手的算法
			observation_dict = self.observationMsg(simuiface, self.carIdAlreadyCreatedList, self.EgoPos, batchNum, simuTime)
			self.recorder.record(observation)
			if getVehicleInfo(observation):
				if observation.test_setting['end'] == -1:
					# todo IDM 规控器（需要实例化IDM对象）
					# action = self.planner.act(observation)  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
					# self.egoAct = action
					# new_ego_info = self.updateEgoPos(action, observation)
					# self.potentialPoint = new_ego_info
					# todo lattice（只需要给函数传入参数）
					goal_x = observation_dict['test_setting']['goal']['x']
					goal_y = observation_dict['test_setting']['goal']['y']
					action = alg_1(observation_dict, goal_x, goal_y)
					self.egoAct = action
					# 如果不要TessNG路径规划，则积分出下一帧坐标点
					if not self.routePlan:
						new_ego_info = self.updateEgoPos(action, observation)
						self.potentialPoint = new_ego_info
				else:
					sys.modules["__main__"].__dict__['finishTest'] = True

	def afterOneStep(self):
		# TESSNG 顶层接口
		iface = tessngIFace()
		# TESSNG 仿真子接口
		simuiface = iface.simuInterface()
		# TESSNG 路网子接口
		netiface = iface.netInterface()
		# batchNum = simuiface.batchNumber()
		self.mainStep(simuiface, netiface)

		# todo 坐标点移动
		# 如果不要路径规划，则给出坐标点
		if not self.routePlan:
			self.moveEgo(simuiface, netiface)

	def afterStop(self):
		self.EgoPos = [0, 0]
		self.maxTestTime = 0
		self.preheatingTime = 0
		self.lock = 0
		# 输出记录信息
		self.recorder.output()
		# 测试 + 1
		self.testFinishNum += 1
		if self.testFinishNum >= self.testNum:
			pidDict = {"done": 1}
			with open("./cache.json", "w") as f:
				json.dump(pidDict, f)
			print("All test finished.")
			return
		else:
			# 重置记录模块
			self.recorder.init()
			iface = tessngIFace()
			simuiface = iface.simuInterface()
			netface = iface.netInterface()
			if self.testFinishNum < self.testNum:
				netFilePath = TESSNG_File_Route_List[self.testFinishNum]

				from TessNG.openNetFile.openNetFile import openNetFile
				openNetFile(netface, netFilePath)
				# netface.openNetFle(netFilePath)
				
				print(f"Open tessng simulation {netFilePath}.")
				simuiface.startSimu()
				print("Start a new tessng simulation for onsite test.")

import os
import math
import copy
import time

import utils.observation
from ..DockWidget import *
from ..DLLs.Tessng import *

from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.recorder import Recorder
from utils.observation import Observation, EgoStatus
from utils.functions import convertAngle, calcDistance, testFinish, check_action, updateEgoPos, kill_process
from utils.netStruct import paintPos, startEndPos, waypoints
from utils.externVehicleLogic import ExternVehicleLogic


# 仿真测试模块
class MySimulatorBase(QObject, PyCustomerSimulator):
	signalRunInfo = Signal(str)
	forStopSimu = Signal()
	forReStartSimu = Signal()
	forPauseSimu = Signal()

	def __init__(self):
		QObject.__init__(self)
		PyCustomerSimulator.__init__(self)

		# 测试步长
		self.dt = None
		# 最大测试时长
		self.maxTestTime = None
		# 仿真预热时间
		self.preheatingTime = 5
		# 背景车探测范围
		self.radius = 50

		# 测试场景信息
		self.scenario_info = ScenarioInfo()
		# 实例化规控器
		self.planner = None

		# 仿真流程控制参数
		self.finishTest = False

		self.recorder = Recorder()
		self.observation = Observation()

		# 测试车的名字
		self.EgoName = 'ego'
		# 是否驶出路网
		self.outSideTessngNet = False
		# 测试车ID Name对应表
		self.vehicleMap = {}
		# 车辆创建对象锁
		self.createCarLock = 0
		# 场景解析锁
		self.scenarioLock = 0
		# 主车控制量
		self.action = [float('nan'), float('nan')]
		# 记录主车信息
		self.ego_id = None
		self.ego_info = None
		# 交互逻辑
		self.externObject = None
		self.externVehicleLogic = ExternVehicleLogic()

	def ref_beforeStart(self, ref_keepOn):
		iface = tessngIFace()
		simuiface = iface.simuInterface()
		simuiface.setSimuAccuracy(1 / self.dt)
		startEndPos["startPos"] = self.scenario_info.task_info['startPos']
		startEndPos["endPos"] = self.scenario_info.task_info['targetPos']
		waypoints["waypoints"] = self.scenario_info.task_info['waypoints']
		return True

	def ref_reSetSpeed(self, pIVehicle, ref_inOutSpeed):
		if pIVehicle.id() in self.externVehicleLogic.forcePassVehicleIdList:
			self.externVehicleLogic.forcePassVehicleIdList.remove(pIVehicle.id())
			ref_inOutSpeed.value = 5
			return True
		return False

	def ref_calcAcce(self, pIVehicle, acce) -> bool:
		shouldStopList = self.externVehicleLogic.vehicleShouldStopIdList
		if pIVehicle.id() in shouldStopList:
			acce.value = -200
			return True
		return False

	def _tessngServerMsg(self, tessngSimuiface, currentTestTime):
		"""
		:param tessngSimuiface: TESSNG Simuiface接口
		:param currentTestTime: 当前TESSNG的仿真计算时间（单位：ms）
		:return: 返回给控制算法的observation信息
		"""
		lAllVehiStatus = tessngSimuiface.allVehiStarted()

		new_observation = Observation()
		# 添加主车信息
		new_observation.ego_info = self.ego_info
		self.trans_ego_to_extern(self.ego_info)
		# 添加背景车信息
		for vehicleStatus in lAllVehiStatus:
			if vehicleStatus.id() != self.ego_id:
				if calcDistance([self.ego_info.x, self.ego_info.y],
								[p2m(vehicleStatus.pos().x()), -p2m(vehicleStatus.pos().y())]) < self.radius:
					new_observation.update_object_info(
						'vehicle',
						self.vehicleMap.get(vehicleStatus.id(), str(vehicleStatus.id())),
						length=p2m(vehicleStatus.length()),
						width=2.02,
						x=p2m(vehicleStatus.pos().x()),
						y=-p2m(vehicleStatus.pos().y()),
						v=p2m(vehicleStatus.currSpeed()),
						a=p2m(vehicleStatus.acce()),
						yaw=math.radians(convertAngle(vehicleStatus.angle())),
					)
					tessngSimuiface.getVehicle(vehicleStatus.id()).setColor("#C318FF")
				else:
					tessngSimuiface.getVehicle(vehicleStatus.id()).setColor("#F8F8FF")
			else:
				tessngSimuiface.getVehicle(vehicleStatus.id()).setColor("#00BFFF")

		# 更新测试结束信息
		if tessngSimuiface.simuTimeIntervalWithAcceMutiples() >= self.preheatingTime * 1000 + 3000:
			end = testFinish(
				goal=self.scenario_info.task_info['targetPos'],
				observation=new_observation,
				outOfTime=(currentTestTime / 1000) >= self.maxTestTime,
				outOfMap=self.outSideTessngNet
			)
		else:
			end = -1
		new_observation.update_test_info(t=currentTestTime / 1000, dt=self.dt, end=end)
		return new_observation

	def _createVehicle(self, simuiface: SimuInterface, netiface: NetInterface, veh_info: dict):
		lLocations = netiface.locateOnCrid(QPointF(veh_info['x'], -veh_info['y']), 9)
		if lLocations:
			dvp = Online.DynaVehiParam()
			dvp.vehiTypeCode = veh_info['type']
			dvp.dist = lLocations[0].distToStart
			dvp.speed = m2p(veh_info['v'])
			dvp.name = str(veh_info['name'])
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
				if self.EgoName in vehi.name():
					self.ego_id = vehi.id()
				else:
					self.vehicleMap[vehi.id()] = vehi.name()
			return vehi.id()

	def _addCar(self, simuiface: SimuInterface, netiface: NetInterface):
		pass

	def _createEgoInfo(self, simuiface: SimuInterface, netiface: NetInterface):
		lAllVehiStatus = simuiface.getVehisStatus()
		for vehicleStatus in lAllVehiStatus:
			if vehicleStatus.vehiId == self.ego_id:
				self.ego_info = EgoStatus(
					length=p2m(vehicleStatus.mrLength),
					width=p2m(vehicleStatus.mrWidth),
					x=p2m(vehicleStatus.mPoint.x()),
					y=-p2m(vehicleStatus.mPoint.y()),
					v=p2m(vehicleStatus.mrSpeed),
					yaw=math.radians(convertAngle(vehicleStatus.mrAngle)),
					a=0,
					rot=0
				)

	def mainStep(self, simuiface, netiface):
		simuTime = simuiface.simuTimeIntervalWithAcceMutiples()
		# 判断是否到达预热时间
		if simuTime >= self.preheatingTime * 1000 and not self.finishTest:
			# 判断是否在仿真中添加主车及背景车
			if not self.createCarLock:
				self._addCar(simuiface, netiface)
				self.createCarLock = 1
			# 判断主车是否已经成功进入仿真环境
			if self.ego_info == None:
				self._createEgoInfo(simuiface, netiface)
			else:
				# 获取当前时刻仿真环境观测信息
				self.observation = self._tessngServerMsg(simuiface, simuTime)
				# 判断仿真是否还在进行中
				if self.observation.test_info['end'] == -1:
					# 记录当前测试信息
					self.recorder.record(self.action, self.observation)
					# 获取规控模块回传的控制信息
					self.action = self.planner.act(self.observation)
					# 对规控器回传的控制信息进行执行器动力学约束修正
					ego_action = check_action(
						dt=self.dt,
						prev_v=self.observation.ego_info.v,
						prev_action=[self.observation.ego_info.a, self.observation.ego_info.rot],
						new_action=self.action
					)
					# 根据修正后的控制量更新主车位置
					updateEgoPos(ego_action, self.dt, self.ego_info)
					# 在TessNG中绘制主车位置
					paintPos["pos"] = self.ego_info.__dict__
				else:
					# 如果仿真到达终止条件则停止仿真
					self.finishTest = True
					self.recorder.record(self.action, self.observation)
					self.forStopSimu.emit()

		# 交互
		self.observation.ego_info.update_extern_object(self.externObject)
		self.externVehicleLogic.vehicleStatus = simuiface.allVehiStarted()
		self.externVehicleLogic.externObjectsDict = self.observation.ego_info.extern_obj.externObjDict
		self.externVehicleLogic.searchVehicleClosePed()

	def afterStep(self, pIVehicle: IVehicle) -> None:
		# 每个step后对每个车辆进行遍历
		pass

	def _checkOutSideMap(self, netiface: NetInterface):
		if self.ego_info is None:
			return
		lLocations = netiface.locateOnCrid(QPointF(m2p(self.ego_info.x), - m2p(self.ego_info.y)), 9)
		if not lLocations:
			self.outSideTessngNet = True

	def _isEgoExist(self, pIVehicle: IVehicle):
		if pIVehicle:
			if not ((pIVehicle.roadIsLink() and pIVehicle.lane()) or (
					not pIVehicle.roadIsLink() and pIVehicle.laneConnector())):
				return False
		else:
			return False
		return True

	def _moveEgo(self, pIVehicle: IVehicle):
		if self.ego_info is None:
			return

		iface = tessngIFace()
		netiface = iface.netInterface()
		lLocations = netiface.locateOnCrid(QPointF(m2p(self.ego_info.x), - m2p(self.ego_info.y)), 9)
		targetLocation = None

		if lLocations:
			# 计算车辆当前所在车道或所在车道连接的上游车道，作为匹配时的目标车道
			if pIVehicle.roadIsLink():
				tempLaneId = pIVehicle.lane().id()
			else:
				# 如果当前车辆在连接段上，.laneConnector() 取到的是车道连接所在的上游车道
				tempLaneId = pIVehicle.laneConnector().fromLane().id()

			for location in lLocations:
				laneObject = location.pLaneObject
				if laneObject.length() < 0.1:
					continue

				if laneObject.isLane():
					targetLocation = location
					break
				else:
					locCastToLaneConnector = laneObject.castToLaneConnector()
					if locCastToLaneConnector and locCastToLaneConnector.fromLane().id() == tempLaneId:
						targetLocation = location
						break

			laneObj = targetLocation.pLaneObject
			pIVehicle.vehicleDriving().move(laneObj, targetLocation.distToStart)

	def _updateShadowEgo(self, simuiface: SimuInterface, netiface: NetInterface):
		if self.ego_id:
			ego_vehicle = simuiface.getVehicle(self.ego_id)
			if self._isEgoExist(ego_vehicle):
				self._moveEgo(ego_vehicle)
			else:
				cur_ego_info = {
					"x": self.ego_info.x,
					"y": self.ego_info.y,
					"v": self.ego_info.v,
					"type": 1,
					"name": f"{self.EgoName}_{time.time()}"
				}
				self._createVehicle(simuiface, netiface, cur_ego_info)

	def afterOneStep(self):
		iface = tessngIFace()
		simuiface = iface.simuInterface()
		netiface = iface.netInterface()
		# 主要测试流程
		self.mainStep(simuiface, netiface)
		# 检查测试环境中主车是否驶出路网
		self._checkOutSideMap(netiface)
		# 更新主车对应TessNG中影子车的状态信息
		self._updateShadowEgo(simuiface, netiface)

	def afterStop(self):
		# 输出测试结果
		self.recorder.output(self.scenario_info.output_path)
		# 退出仿真
		kill_process(os.getpid())

	def get_observation(self):
		return copy.deepcopy(self.observation)

	def trans_ego_to_extern(self, ego_info):
		iface = tessngIFace()
		simuiface = iface.simuInterface()
		batchNumber = simuiface.batchNumber()
		self.externObject = {'type': 'trajectory', 'onsite': {'timestamp': time.time(), 'value': {
			'主车1': {'frameId': batchNumber, 'speed': ego_info.v, 'courseAngle': ego_info.yaw,
					  'tessngPos': [ego_info.x, ego_info.y], 'length': ego_info.length * 100,
					  'width': ego_info.width * 100}}}}

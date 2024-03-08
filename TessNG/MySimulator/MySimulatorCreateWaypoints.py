import os
import json

from ..DockWidget import *
from ..DLLs.Tessng import *

from utils.netStruct import startEndPos, waypoints
from utils.functions import kill_process

def getGoalArea(goal_point):
    center_x, center_y = goal_point
    gap_dis = 1.5
    return [[center_x - gap_dis, center_y + gap_dis], [center_x + gap_dis, center_y - gap_dis]]

# 仿真测试模块
class MySimulatorCreateWaypoints(QObject, PyCustomerSimulator):
    signalRunInfo = Signal(str)
    forStopSimu = Signal()
    forReStartSimu = Signal()
    forPauseSimu = Signal()

    def __init__(self, config: dict):
        QObject.__init__(self)
        PyCustomerSimulator.__init__(self)

        map_dir = os.path.join(os.path.dirname(__file__), f"../../scenario/serial/maps/{config['map']}")
        for f in os.listdir(map_dir):
            if not f.startswith('.') and f.endswith('.tess'):
                tess_file_path = os.path.join(map_dir, f)
                break
        iface = tessngIFace()
        netface = iface.netInterface()
        simuiface = iface.simuInterface()
        netface.openNetFle(tess_file_path)

        task_name = f"serial_task_{config['task_num']}"
        self.task_path = os.path.abspath(os.path.join(os.path.dirname(__file__), f"../../scenario/serial/tasks/{task_name}.json"))
        if os.path.exists(self.task_path):
            with open(self.task_path, "r") as f:
                scenario_info = json.load(f)
                startEndPos["startPos"] = scenario_info.get('startPos', [])
                startEndPos["endPos"] = scenario_info.get('targetPos', [])
                waypoints["waypoints"] = scenario_info.get('waypoints', {})

        self.config = config
        self.linkIds = config.get('link_ids', [])
        # 测试车的名字
        self.EgoName = 'ego'
        # 测试间隔
        self.dt = 0.05
        # 测试车的位置
        self.EgoPos = []
        # 车辆位置记录
        self.recorder = []
        # 车辆创建对象锁
        self.createCarLock = 0

    def ref_beforeStart(self, ref_keepOn):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        simuiface.setSimuAccuracy(1 / self.dt)
        return True
    
    def _getEgoPos(self, pIVehicle):
        vehicle_name = pIVehicle.name()
        if vehicle_name and vehicle_name == self.EgoName:
            self.EgoPos = [p2m(pIVehicle.pos().x()), -p2m(pIVehicle.pos().y())]
            return
        
    def _updateEgoRoute(self, pIVehicle):
        if pIVehicle.name() == self.EgoName:
            if self.linkIds:
                if pIVehicle.roadId() == self.linkIds[0]:
                    if len(self.linkIds) > 1:
                        iface = tessngIFace()
                        netiface = iface.netInterface()
                        egoRoute = netiface.shortestRouting(netiface.findLink(self.linkIds[0]), netiface.findLink(self.linkIds[1]))
                        pIVehicle.vehicleDriving().setRouting(egoRoute)
                    self.linkIds.pop(0)
                    print(self.linkIds)
            else:
                self.forStopSimu.emit()
        
    def afterStep(self, pIVehicle: IVehicle) -> None:
        self._getEgoPos(pIVehicle)
        self._updateEgoRoute(pIVehicle)

    def _createEgoWithLinkid(self, simuiface: SimuInterface, netiface: NetInterface, linkId, speed, vehicleTypeCode):
        dvp = Online.DynaVehiParam()
        dvp.vehiTypeCode = vehicleTypeCode
        dvp.dist = 0
        dvp.speed = m2p(speed)
        dvp.color = "#00BFFF"
        dvp.name = self.EgoName
        # dvp.roadId = netiface.findLink(linkId)
        dvp.roadId = linkId
        dvp.laneNumber = self.config.get('start_lane', 0)
        vehi = simuiface.createGVehicle(dvp)

    def mainStep(self, simuiface, netiface):
        if not self.createCarLock:
            self._createEgoWithLinkid(simuiface, netiface, self.linkIds[0], 10, 1)
            self.createCarLock = 1
        if self.EgoPos:
            self.recorder.append(self.EgoPos)

    def afterOneStep(self):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        netiface = iface.netInterface()
        self.mainStep(simuiface, netiface)
                                           
    def afterStop(self):
        wp = {}
        last_pos = None
        num = 1
        for index, ego_pos in enumerate(self.recorder):
            if index % 10 == 0 and ego_pos != last_pos:
                wp[str(num)] = [round(ego_pos[0], 4), round(ego_pos[1], 4)]
                num += 1
                last_pos = ego_pos
        task_info = {
            "map": "TJST",
            "startPos": list(wp.values())[0],
            "targetPos": getGoalArea(list(wp.values())[-1]),
            "waypoints": wp,
        }
        with open(self.task_path, "w") as f:
            json.dump(task_info, f)
        kill_process(os.getpid())


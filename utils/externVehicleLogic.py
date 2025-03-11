import time
import numpy as np


class ExternVehicleLogic:
    def __init__(self):
        self.gap = 16
        self.tau = 1.5

        self._vehicleStatus = []
        self._externObjectsDict = {}

        self.vehicleShouldStopIdList = []
        self.vehicleStopRecord = {}
        self.vehicleReleaseRecord = {}
        self.forcePassVehicleIdList = []

    @staticmethod
    def calculateDistance(point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    @property
    def vehicleStatus(self):
        return self._vehicleStatus

    @vehicleStatus.setter
    def vehicleStatus(self, vehicleStatus):
        if isinstance(vehicleStatus, list):
            self._vehicleStatus = vehicleStatus

    @property
    def externObjectsDict(self):
        return self._externObjectsDict

    @externObjectsDict.setter
    def externObjectsDict(self, outSideTargetStatus):
        self._externObjectsDict = outSideTargetStatus if isinstance(outSideTargetStatus, dict) else None

    def getVehicleShouldStopIdList(self):
        return self.vehicleShouldStopIdList

    @staticmethod
    def isPointInRectangle(p, vertices):
        x, y = p
        inside = False
        n = len(vertices)
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]
            if min(y1, y2) < y <= max(y1, y2):
                x_intersection = (y - y1) * (x2 - x1) / (y2 - y1) + x1 if y1 != y2 else x1
                if x_intersection > x:
                    inside = not inside
        return inside

    def judgeVehicleStop(self, tessngVehiclePos, avBoundList, avName, avSpeed, tessngVehicleId):
        currentTime = int(time.time() * 1000)

        if self.isPointInRectangle(tessngVehiclePos, avBoundList):
            if tessngVehicleId not in self.vehicleReleaseRecord and tessngVehicleId not in self.forcePassVehicleIdList:
                if tessngVehicleId not in self.vehicleShouldStopIdList:
                    self.vehicleShouldStopIdList.append(tessngVehicleId)
                if tessngVehicleId not in self.vehicleStopRecord:
                    self.vehicleStopRecord[tessngVehicleId] = {"time": currentTime, "target": avName}

            if tessngVehicleId in self.vehicleReleaseRecord:
                releaseRecord = self.vehicleReleaseRecord[tessngVehicleId]
                if (currentTime - releaseRecord["time"]) < 1000 and releaseRecord["target"] == avName:
                    if tessngVehicleId in self.vehicleShouldStopIdList:
                        self.vehicleShouldStopIdList.remove(tessngVehicleId)
                    self.forcePassVehicleIdList.append(tessngVehicleId)

        if tessngVehicleId in self.vehicleStopRecord:
            record = self.vehicleStopRecord[tessngVehicleId]
            if (currentTime - record["time"]) > self.tau:
                if tessngVehicleId in self.vehicleShouldStopIdList:
                    self.vehicleShouldStopIdList.remove(tessngVehicleId)
                self.vehicleStopRecord.pop(tessngVehicleId, None)
                if tessngVehicleId not in self.forcePassVehicleIdList:
                    self.forcePassVehicleIdList.append(tessngVehicleId)
                self.vehicleReleaseRecord[tessngVehicleId] = {"time": currentTime, "target": avName}

    def searchVehicleClosePed(self):
        if not self._vehicleStatus or not self._externObjectsDict:
            return

        for vehicle in self._vehicleStatus:
            vehicleId = vehicle.id()
            vehicleCenterPos = [vehicle.pos().x(), -vehicle.pos().y()]
            tessngVehicleBound = vehicle.boundingPolygon()

            for avName, avStruct in self._externObjectsDict.items():
                avPos, avBound, avSpeed = avStruct.pos, avStruct.bound, avStruct.speed
                if self.calculateDistance(vehicleCenterPos, avPos) > self.gap:
                    continue

                for tessngVehicleBoundPoint in tessngVehicleBound:
                    vehiclePos = [tessngVehicleBoundPoint.x(), -tessngVehicleBoundPoint.y()]
                    self.judgeVehicleStop(vehiclePos, avBound, avName, avSpeed, vehicleId)

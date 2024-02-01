import numpy as np
import math


class DataStructureAnalyse:
    def __init__(self, frameCount, threshold):
        self.headerData = []
        self.frameCount = frameCount
        self.threshold = threshold

    def add_headerData(self, value):
        self.headerData.append(value)
        if len(self.headerData) > self.frameCount:
            self.headerData.pop(0)

    def check_monotonic(self) -> str:
        if self.frameCount > len(self.headerData) > 0:
            return "Straight"

        differences = [self.headerData[i+1] - self.headerData[i] for i in range(self.frameCount - 1)]

        if all(abs(diff) > self.threshold for diff in differences):
            is_increasing = all(self.headerData[i] <= self.headerData[i + 1] for i in range(self.frameCount - 1))
            is_decreasing = all(self.headerData[i] >= self.headerData[i + 1] for i in range(self.frameCount - 1))
            if is_increasing:
                return "Right"
            elif is_decreasing:
                return "Left"
            else:
                return "Straight"
        else:
            return "Straight"

# 普通车
class Car(DataStructureAnalyse):
    def __init__(self, id: int, name: str, Xpos: float, Ypos: float, speed: float, roadType: int, frameCount: int, threshold: float):
        super().__init__(frameCount, threshold)
        self.id = id
        self.name = name
        self.Xpos = Xpos
        self.Ypos = Ypos
        self.speed = speed
        self.roadType = roadType
        self.distance = 0
        self.roadId = 0
        self.laneNum = None
        self.previousPos = [0, 0]
        self.currentPos = [0, 0]
        self.disInRoad = 0
        self.vehicleType = 0

    def isConnector(self) -> bool:
        if self.roadType == 0:
            return True
        else:
            return False

    def add_frame(self, frame_data):
        self.previousPos = self.currentPos
        self.currentPos = frame_data

    def get_current_framePos(self):
        return self.currentPos

    def get_previous_framePos(self):
        return self.currentPos

    def calculateTravelDistance(self, distance):
        self.distance += distance

    # 计算前后两帧行驶的距离
    def calculatePointDistance(self):
        p1 = np.array([self.previousPos[0], self.previousPos[1]])
        p2 = np.array([self.currentPos[0], self.currentPos[1]])
        p3 = p2 - p1
        d = math.hypot(p3[0], p3[1])
        return d

    # 计算AV与SV之间的距离
    def pointDistanceBetweenSV(self, SV_Pos):
        p1 = np.array([SV_Pos[0], SV_Pos[1]])
        p2 = np.array([self.currentPos[0], self.currentPos[1]])
        p3 = p2 - p1
        d = math.hypot(p3[0], p3[1])
        return d

    # 将y轴正轴顺时钟方向的角转为x正轴逆时针方向的角
    @staticmethod
    def convert_clockwise_to_counterclockwise(angle) -> float:
        counterclockwise_angle = (90 - angle) % 360
        return counterclockwise_angle

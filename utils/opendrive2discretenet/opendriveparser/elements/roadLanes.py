# -*- coding: utf-8 -*-

from .road_record import RoadRecord


class Lanes:
    """ """

    def __init__(self):
        self._laneOffsets = []  # 车道偏移用来表示将中心车道(center lane)从道路参考线上位移
        self._lane_sections = []  # 每个车道段都必须保持一个固定的车道编号，当车道编号发生变更（如展宽），便随之产生一个新车道段

    @property
    def laneOffsets(self):
        """ """
        self._laneOffsets.sort(key=lambda x: x.start_pos)
        return self._laneOffsets

    @property
    def lane_sections(self):
        """ """
        self._lane_sections.sort(key=lambda x: x.sPos)
        return self._lane_sections

    def getLaneSection(self, laneSectionIdx):
        """

        Args:
          laneSectionIdx:

        Returns:

        """
        for laneSection in self.lane_sections:
            if laneSection.idx == laneSectionIdx:
                return laneSection

        return None

    def getLastLaneSectionIdx(self):
        """ """

        numLaneSections = len(self.lane_sections)

        if numLaneSections > 1:
            return numLaneSections - 1

        return 0


# 继承自抽象基类RoadRecord，其线型是通过三次多项式表示：attr = a + b*ds + c*ds² + d*ds³
# 其包含两个属性，该区段起点s坐标start_pos，对应三次多项式参数[a, b, c, d]
class LaneOffset(RoadRecord):
    """The lane offset record defines a lateral shift of the lane reference line
    (which is usually identical to the road reference line).

    (Section 5.3.7.1 of OpenDRIVE 1.4)

    """


class LeftLanes:
    """ """

    sort_direction = False

    def __init__(self):
        self._lanes = []

    @property
    def lanes(self):
        """ """
        self._lanes.sort(key=lambda x: x.id, reverse=self.sort_direction)  # 左车道组车道ID升序排列
        return self._lanes


class CenterLanes(LeftLanes):
    """ """


class RightLanes(LeftLanes):
    """ """

    sort_direction = True  # 右车道组车道ID降序排列


class Lane:
    """ """

    laneTypes = [
        "none",
        "driving",
        "stop",
        "shoulder",
        "biking",
        "sidewalk",
        "border",
        "restricted",
        "parking",
        "bidirectional",
        "median",
        "special1",
        "special2",
        "special3",
        "roadWorks",
        "tram",
        "rail",
        "entry",
        "exit",
        "offRamp",
        "onRamp",
    ]

    def __init__(self, parentRoad, lane_section):
        self._parent_road = parentRoad
        self._id = None
        self._type = None
        self._level = None
        self._link = LaneLink()
        self._widths = []
        self._borders = []
        self.lane_section = lane_section
        self.has_border_record = False

    @property
    def parentRoad(self):
        """ """
        return self._parent_road

    @property
    def id(self):
        """ """
        return self._id

    @id.setter
    def id(self, value):
        self._id = int(value)

    @property
    def type(self):
        """ """
        return self._type

    @type.setter
    def type(self, value):
        if value not in self.laneTypes:
            raise Exception()

        self._type = str(value)

    @property
    def level(self):
        """ """
        return self._level

    @level.setter
    def level(self, value):
        if value not in ["true", "false"] and value is not None:
            raise AttributeError("Value must be true or false.")

        self._level = value == "true"

    @property
    def link(self):
        """ """
        return self._link

    @property
    def widths(self):
        """ """
        self._widths.sort(key=lambda x: x.start_offset)
        return self._widths

    @widths.setter
    def widths(self, value):
        """"""
        self._widths = value

    def getWidth(self, widthIdx):
        """

        Args:
          widthIdx:

        Returns:

        """
        for width in self._widths:
            if width.idx == widthIdx:
                return width

        return None

    def getLastLaneWidthIdx(self):
        """Returns the index of the last width sector of the lane"""

        numWidths = len(self._widths)

        if numWidths > 1:
            return numWidths - 1

        return 0

    @property
    def borders(self):
        """ """
        return self._borders


class LaneLink:
    """记录车道的连接关系，储存对应前导车道与后续车道的id
    【不同于道路road的连接关系，laneLink只包含对应车道的id，而road的连接则包含对应road的type\id\contactPoint】
    """

    def __init__(self):
        self._predecessor = None
        self._successor = None

    @property
    def predecessorId(self):
        """ """
        return self._predecessor

    @predecessorId.setter
    def predecessorId(self, value):
        self._predecessor = int(value)

    @property
    def successorId(self):
        """ """
        return self._successor

    @successorId.setter
    def successorId(self, value):
        self._successor = int(value)


class LaneSection:
    """The lane section record defines the characteristics of a road cross-section.

    (Section 5.3.7.2 of OpenDRIVE 1.4)

    """

    def __init__(self, road=None):
        self.idx = None
        self.sPos = None
        self._singleSide = None
        self._leftLanes = LeftLanes()
        self._centerLanes = CenterLanes()
        self._rightLanes = RightLanes()

        self._parentRoad = road

    @property
    def singleSide(self):
        """Indicator if lane section entry is valid for one side only."""
        return self._singleSide

    @singleSide.setter
    def singleSide(self, value):
        if value not in ["true", "false"] and value is not None:
            raise AttributeError("Value must be true or false.")

        self._singleSide = value == "true"

    @property
    def leftLanes(self):
        """Get list of sorted lanes always starting in the middle (lane id -1)"""
        return self._leftLanes.lanes  # 返回列表

    @property
    def centerLanes(self):
        """ """
        return self._centerLanes.lanes

    @property
    def rightLanes(self):
        """Get list of sorted lanes always starting in the middle (lane id 1)"""
        return self._rightLanes.lanes

    @property
    def allLanes(self):
        """Attention! lanes are not sorted by id"""
        return self._leftLanes.lanes + self._centerLanes.lanes + self._rightLanes.lanes

    def getLane(self, lane_id: int) -> Lane:
        """

        Args:
          lane_id:

        Returns:

        """
        for lane in self.allLanes:
            if lane.id == lane_id:
                return lane

        return None

    @property
    def parentRoad(self):
        """ """
        return self._parentRoad


class LaneWidth(RoadRecord):
    """Entry for a lane describing the width for a given position.
    (Section 5.3.7.2.1.1.2 of OpenDRIVE 1.4)


    start_offset being the offset of the entry relative to the preceding lane section record

"""

    def __init__(
        self,
        *polynomial_coefficients: float,
        idx: int = None,
        start_offset: float = None
    ):
        self.idx = idx  # 标识当前width公式的索引
        self.length = 0
        super().__init__(*polynomial_coefficients, start_pos=start_offset)  # 继承回父类的init方法

    @property
    def start_offset(self):
        """Return start_offset, which is the offset of the entry to the
        start of the lane section.
        """
        return self.start_pos

    @start_offset.setter
    def start_offset(self, value):
        self.start_pos = value


class LaneBorder(LaneWidth):
    """Describe lane by width in respect to reference path.

    (Section 5.3.7.2.1.1.3 of OpenDRIVE 1.4)

    Instead of describing lanes by their width entries and, thus,
    invariably depending on influences of inner
    lanes on outer lanes, it might be more convenient to just describe
    the outer border of each lane
    independent of any inner lanes’ parameters.
    """

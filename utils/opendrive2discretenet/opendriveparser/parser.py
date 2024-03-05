# -*- coding: utf-8 -*-

import numpy as np
from lxml import etree

from .elements.opendrive import OpenDrive, Header
from .elements.road import Road
from .elements.roadLink import (
    Predecessor as RoadLinkPredecessor,
    Successor as RoadLinkSuccessor,
    Neighbor as RoadLinkNeighbor,
)
from .elements.roadtype import (
    RoadType,
    Speed as RoadTypeSpeed,
)
from .elements.roadElevationProfile import (
    ElevationRecord as RoadElevationProfile,
)
from .elements.roadLateralProfile import (
    Superelevation as RoadLateralProfileSuperelevation,
    Crossfall as RoadLateralProfileCrossfall,
    Shape as RoadLateralProfileShape,
)
from .elements.roadLanes import (
    LaneOffset as RoadLanesLaneOffset,
    Lane as RoadLaneSectionLane,
    LaneSection as RoadLanesSection,
    LaneWidth as RoadLaneSectionLaneWidth,
    LaneBorder as RoadLaneSectionLaneBorder,
)
from .elements.junction import (
    Junction,
    Connection as JunctionConnection,
    LaneLink as JunctionConnectionLaneLink,
)


def parse_opendrive(root_node) -> OpenDrive:
    """Tries to parse XML tree, returns OpenDRIVE object

    Args:
      root_node:  经过lxml解析后的_ElementTree对象中的根节点对象lxml.etree._Element

    Returns:
      The object representing an OpenDrive specification.

    """

    # Only accept lxml element
    if not etree.iselement(root_node):
        raise TypeError("Argument root_node is not a xml element")

    opendrive = OpenDrive()  # 实例化OpenDrive类对象

    # Header
    header = root_node.find("header")  # xodr文件中有且只有一个header标签，故root.find()即可（find是找到第一个匹配的节点）
    if header is not None:
        parse_opendrive_header(opendrive, header)  # 解析header标签的相关属性，并以Header类作为OpenDrive类对象的header属性值

    # Junctions
    for junction in root_node.findall("junction"):  # 遍历xodr文件中所有junction标签，故root.findall()（findall找到所有匹配节点）
        parse_opendrive_junction(opendrive, junction)  # 解析每个junction节点，并以Junction类对象作为OpenDrive类对象属性值

    # Load roads
    for road in root_node.findall("road"):
        parse_opendrive_road(opendrive, road)  # 解析每个road节点，并以Road类对象作为OpenDrive类对象属性值（填充至List中）

    return opendrive


# 获取OpenD路网各路段的连接关系
def parse_opendrive_road_link(newRoad, opendrive_road_link):
    """对road节点下的link子节点进行解析，补充Road类对象中包含的拓扑信息
    Road -> Road.link.predecessor (对应RoadLinkPredecessor类) / Road.link.successor (对应RoadLinkSuccessor类)

    Args:
      newRoad: Road的实例化对象
      opendrive_road_link: road节点下的link子节点

    """
    predecessor = opendrive_road_link.find("predecessor")  # 当前Road的前驱道路predecessor

    if predecessor is not None:

        newRoad.link.predecessor = RoadLinkPredecessor(  # predecessor类，属性包括被连接的predecessorID、类型以及连接点
            predecessor.get("elementType"),  # road / junction
            predecessor.get("elementId"),
            predecessor.get("contactPoint"),  # start / end
        )

    successor = opendrive_road_link.find("successor")  # 当前Road的后继道路successor

    if successor is not None:

        newRoad.link.successor = RoadLinkSuccessor(  # successor类，属性包括被连接的successor对应的元素ID、类型以及连接点
            successor.get("elementType"),
            successor.get("elementId"),
            successor.get("contactPoint"),
        )

    for neighbor in opendrive_road_link.findall("neighbor"):

        newNeighbor = RoadLinkNeighbor(
            neighbor.get("side"), neighbor.get("elementId"), neighbor.get("direction")
        )

        newRoad.link.neighbors.append(newNeighbor)


def parse_opendrive_road_type(road, opendrive_xml_road_type: etree.ElementTree):
    """Parse opendrive road type and append to road object.
    Road -> Road.types -> Road.types.speed

    Args:
      road: Road to append the parsed road_type to types.
      opendrive_xml_road_type: XML element which contains the information.
      opendrive_xml_road_type: etree.ElementTree:

    """
    speed = None
    if opendrive_xml_road_type.find("speed") is not None:  # 可以为不同的道路类型添加对应的速度限制

        speed = RoadTypeSpeed(
            max_speed=opendrive_xml_road_type.find("speed").get("max"),
            unit=opendrive_xml_road_type.find("speed").get("unit"),
        )

    road_type = RoadType(
        s_pos=opendrive_xml_road_type.get("s"),  # 该类型区段起始位置在road参考线坐标系中的s坐标
        use_type=opendrive_xml_road_type.get("type"),
        speed=speed,
    )
    road.types.append(road_type)


def parse_opendrive_road_geometry(newRoad, road_geometry):
    """对于传入的road对应区段的几何线型进行解析
    Road -> Road.planView (class PlanView) -> Road.planView._geometries (list (class Geometry))

    Args:
      newRoad:
      road_geometry:

    """

    startCoord = [float(road_geometry.get("x")), float(road_geometry.get("y"))]  # 获取该区段起始位置的xy坐标

    # Line直线的几何参数：起始位置xy坐标、航向角、区段对应参考线长度（抽象基类Geometry的传入参数）
    if road_geometry.find("line") is not None:
        newRoad.planView.addLine(
            startCoord,
            float(road_geometry.get("hdg")),
            float(road_geometry.get("length")),
        )

    # Spiral欧拉螺线的几何参数：除Geometry的三参数外，还包括起始位置的曲率curvStart及终点位置的曲率curvEnd（沿着参考线，曲率从头至尾呈线性）
    elif road_geometry.find("spiral") is not None:
        newRoad.planView.addSpiral(
            startCoord,
            float(road_geometry.get("hdg")),
            float(road_geometry.get("length")),
            float(road_geometry.find("spiral").get("curvStart")),
            float(road_geometry.find("spiral").get("curvEnd")),
        )

    # Arc弧线的几何参数：除Geometry的三参数外，还包括参考线的曲率curvature（恒定）
    elif road_geometry.find("arc") is not None:
        newRoad.planView.addArc(
            startCoord,
            float(road_geometry.get("hdg")),
            float(road_geometry.get("length")),
            float(road_geometry.find("arc").get("curvature")),
        )

    # Poly3三次样条曲线：y(x) = a + b * x + c * x ** 2 + d * x ** 3
    elif road_geometry.find("poly3") is not None:
        newRoad.planView.addPoly3(
            startCoord,
            float(road_geometry.get("hdg")),
            float(road_geometry.get("length")),
            float(road_geometry.find("poly3").get("a")),
            float(road_geometry.find("poly3").get("b")),
            float(road_geometry.find("poly3").get("c")),
            float(road_geometry.find("poly3").get("d")),
        )
        # raise NotImplementedError()

    # paramPoly3三次参数样条曲线：针对Poly3的改进（要求x递增，曲线不能出现绕回打圈），利用参数（曲线上的累计弦长，可归一化）方程表示曲线
    # u(p) = aU + bU*p + cU*p2 + dU*p³
    # v(p) = aV + bV*p + cV*p2 + dV*p³
    elif road_geometry.find("paramPoly3") is not None:
        if road_geometry.find("paramPoly3").get("pRange"):

            if road_geometry.find("paramPoly3").get("pRange") == "arcLength":  # arcLength / normalized
                pMax = float(road_geometry.get("length"))  # 采用参考线累计弦长作为插值参数p
            else:
                pMax = None  # 插值参数p采用归一化值，[0, 1]
        else:
            pMax = None  # 未说明，默认采用归一化值

        newRoad.planView.addParamPoly3(
            startCoord,
            float(road_geometry.get("hdg")),
            float(road_geometry.get("length")),
            float(road_geometry.find("paramPoly3").get("aU")),
            float(road_geometry.find("paramPoly3").get("bU")),
            float(road_geometry.find("paramPoly3").get("cU")),
            float(road_geometry.find("paramPoly3").get("dU")),
            float(road_geometry.find("paramPoly3").get("aV")),
            float(road_geometry.find("paramPoly3").get("bV")),
            float(road_geometry.find("paramPoly3").get("cV")),
            float(road_geometry.find("paramPoly3").get("dV")),
            pMax,
        )

    else:
        raise Exception("invalid xml")


def parse_opendrive_road_elevation_profile(newRoad, road_elevation_profile):
    """OpenD中采用三次多项式函数来计算道路高程：elev(ds) = a + b*ds + c*ds² + d*ds³
    其中ds为当前<elevationProfile>节点对应Road区段起点与给定点之间的距离

    Args:
      newRoad:
      road_elevation_profile:

    """

    for elevation in road_elevation_profile.findall("elevation"):

        newElevation = (
            RoadElevationProfile(
                float(elevation.get("a")),
                float(elevation.get("b")),
                float(elevation.get("c")),
                float(elevation.get("d")),
                start_pos=float(elevation.get("s")),  # 对应区段起始位置的s坐标
            ),
        )

        newRoad.elevationProfile.elevations.append(newElevation)


def parse_opendrive_road_lateral_profile(newRoad, road_lateral_profile):
    """

    Args:
      newRoad:
      road_lateral_profile:

    """

    for superelevation in road_lateral_profile.findall("superelevation"):

        newSuperelevation = RoadLateralProfileSuperelevation(
            float(superelevation.get("a")),
            float(superelevation.get("b")),
            float(superelevation.get("c")),
            float(superelevation.get("d")),
            start_pos=float(superelevation.get("s")),
        )

        newRoad.lateralProfile.superelevations.append(newSuperelevation)

    for crossfall in road_lateral_profile.findall("crossfall"):

        newCrossfall = RoadLateralProfileCrossfall(
            float(crossfall.get("a")),
            float(crossfall.get("b")),
            float(crossfall.get("c")),
            float(crossfall.get("d")),
            side=crossfall.get("side"),
            start_pos=float(crossfall.get("s")),
        )

        newRoad.lateralProfile.crossfalls.append(newCrossfall)

    for shape in road_lateral_profile.findall("shape"):

        newShape = RoadLateralProfileShape(
            float(shape.get("a")),
            float(shape.get("b")),
            float(shape.get("c")),
            float(shape.get("d")),
            start_pos=float(shape.get("s")),
            start_pos_t=float(shape.get("t")),
        )

        newRoad.lateralProfile.shapes.append(newShape)


def parse_opendrive_road_lane_offset(newRoad, lane_offset):
    """OpenD中采用三次多项式对道路中心线的偏移进行描述：offset (ds) = a + b*ds + c*ds² + d*ds³
    road -> (Class) lanes -> (List) laneOffsets -> (Class) RoadLanesOffset (start_pos, [a, b, c, d])

    Args:
      newRoad:
      lane_offset:

    """

    newLaneOffset = RoadLanesLaneOffset(
        float(lane_offset.get("a")),
        float(lane_offset.get("b")),
        float(lane_offset.get("c")),
        float(lane_offset.get("d")),
        start_pos=float(lane_offset.get("s")),
    )

    newRoad.lanes.laneOffsets.append(newLaneOffset)


def parse_opendrive_road_lane_section(newRoad, lane_section_id, lane_section):
    """OpenD在<laneSection>节点下对各车道组<left/center/right>中的各车道进行描述
    Road -> lanes -> (List) lane_section -> (List) leftLanes/centerLanes/rightLanes.lanes -> (Class) Lane


    Args:
      newRoad: Road类的实例化对象
      lane_section_id: OpenD文件中当前Road节点下所有lane_section集合中对应当前lane_section的索引
      lane_section: OpenD文件中当前lane_section子节点

    """

    newLaneSection = RoadLanesSection(road=newRoad)  # 表明该lane_section对应的父节点<road>

    # Manually enumerate lane sections for referencing purposes
    newLaneSection.idx = lane_section_id

    newLaneSection.sPos = float(lane_section.get("s"))
    newLaneSection.singleSide = lane_section.get("singleSide")

    # 哈希表储存该lane_section对应的车道组信息
    # key为<left\center\right>，value为列表，其中元素为Lane类的实例化对象
    sides = dict(
        left=newLaneSection.leftLanes,  # (List) LeftLanes.lanes，车道组id升序排列 (1, 2, ...)
        center=newLaneSection.centerLanes,  # 继承自LeftLanes，(List) CenterLanes.lanes，每个roadSection有且仅有一个中心车道，id为0
        right=newLaneSection.rightLanes,  # 继承自LeftLanes，(List) RightLanes.lanes，但车道组id降序排列（-1, -2, ...)
    )

    # 按照左中右顺序遍历该lane_section中所有的车道lanes
    for sideTag, newSideLanes in sides.items():

        side = lane_section.find(sideTag)  # 获取当前<laneSection>节点下对应的<left\center\right>子节点

        # 对于单向road而言，会存在缺少一侧车道组的情况，直接跳过
        if side is None:
            continue
        # 遍历该车道组<left\center\right>下所有车道，并保存在对应key下的value中
        for lane in side.findall("lane"):

            # 实例化Lane类对象，初始化其所属的Road与Road_section
            new_lane = RoadLaneSectionLane(
                parentRoad=newRoad, lane_section=newLaneSection
            )
            new_lane.id = lane.get("id")
            new_lane.type = lane.get("type")

            # In some sample files the level is not specified according to the OpenDRIVE spec
            new_lane.level = (
                "true" if lane.get("level") in [1, "1", "true"] else "false"
            )

            # 获取车道连接关系，每个车道可以拥有对应的前导车道与后继车道（该车道可以同属于一个road也可以不是一个road）
            if lane.find("link") is not None:

                if lane.find("link").find("predecessor") is not None:
                    new_lane.link.predecessorId = (
                        lane.find("link").find("predecessor").get("id")
                    )

                if lane.find("link").find("successor") is not None:
                    new_lane.link.successorId = (
                        lane.find("link").find("successor").get("id")
                    )

            # Width
            '''对车道Lane的宽度属性进行解析。车道的宽度是沿t坐标而定义的。车道的宽度有可能在车道段内产生变化。
            故对于同一条车道，可能存在多组width参数，Width (ds) = a + b*ds + c*ds² + d*ds
            '''
            for widthIdx, width in enumerate(lane.findall("width")):
                # 对于一条lane，可能需要使用多个三次多项式来描述其宽度
                newWidth = RoadLaneSectionLaneWidth(
                    float(width.get("a")),
                    float(width.get("b")),
                    float(width.get("c")),
                    float(width.get("d")),
                    idx=widthIdx,  # 由于存在多个宽度公式，故需要标识对应编号
                    start_offset=float(width.get("sOffset")),  # s坐标系下，当前width元素起始位置相对于前导车道的s坐标
                )

                new_lane.widths.append(newWidth)

            # Border
            '''车道边界是用来描述车道宽度的另一种方法，see Section 9.4.2 in OpenDRIVE 1.6'''
            for borderIdx, border in enumerate(lane.findall("border")):

                newBorder = RoadLaneSectionLaneBorder(
                    float(border.get("a")),
                    float(border.get("b")),
                    float(border.get("c")),
                    float(border.get("d")),
                    idx=borderIdx,
                    start_offset=float(border.get("sOffset")),
                )

                new_lane.borders.append(newBorder)

            '''车道宽度与车道边界元素在相同的车道组内互相排斥。
            若宽度以及车道边界元素在OpenDRIVE文件中同时供车道段使用，那么应用必须使用<width>元素提供的信息。
            '''
            if lane.find("width") is None and lane.find("border") is not None:
                new_lane.widths = new_lane.borders
                new_lane.has_border_record = True

            # Road Marks
            # TODO implementation

            # Material
            # TODO implementation

            # Visiblility
            # TODO implementation

            # Speed
            # TODO implementation

            # Access
            # TODO implementation

            # Lane Height
            # TODO implementation

            # Rules
            # TODO implementation

            newSideLanes.append(new_lane)

    '''将解析完成的Lane_section信息添加至Road对应属性中, road.lanes.lane_section -> List[<class>LaneSection]
    road -> lanes -> laneSection -> left/center/right -> lane
    '''
    newRoad.lanes.lane_sections.append(newLaneSection)


def parse_opendrive_road(opendrive, road):
    """对OpenD中的road节点进行解析，填充Road类对象内对应属性
    连接关系link / 区段类型type / 区段参考线几何形状geometry / 区段高程方程elevationProfile
    区段超高程方程lateralProfile / 车道lanes -> 区段中心车道偏移laneOffset / 车道区段laneSection

    Args:
      opendrive: OpenDrive的实例化对象
      road: OpenD文件中的road根节点

    """

    newRoad = Road()  # 实例化Road类对象

    newRoad.id = int(road.get("id"))
    newRoad.name = road.get("name")

    junctionId = int(road.get("junction")) if road.get("junction") != "-1" else None  # -1表示该road非交叉口内的connecting road

    if junctionId:
        newRoad.junction = opendrive.getJunction(junctionId)  # 对于connecting road对其junction属性进行赋值（Junction类对象）

    # TODO verify road length
    newRoad.length = float(road.get("length"))

    # Links
    opendrive_road_link = road.find("link")  # Road仅能与一条道路存在link关系，故只有一个link子节点；若有多个则为junction
    if opendrive_road_link is not None:
        # 获得每个非孤立road的连接关系，road.link.predecessor/road.link.successor
        parse_opendrive_road_link(newRoad, opendrive_road_link)

    # Type
    for opendrive_xml_road_type in road.findall("type"):  # 对于一个raod节点，其可能含有多个type子节点（不同区段的类型存在差异，需要分别定义）
        # 对于车道类型属性，其在road类中以list形式储存，road.types: List
        parse_opendrive_road_type(newRoad, opendrive_xml_road_type)

    # Plan view
    for road_geometry in road.find("planView").findall("geometry"):  # OpenD中，road参考线的几何形状通过planView节点下的geometry子节点表示
        # 对于一条road，其几何形状可能由多个不同线形的区段构成，对于一个geometry节点有且仅有一个表征线型的子节点<line\spiral\arc\poly3\paramPoly3/>
        parse_opendrive_road_geometry(newRoad, road_geometry)

    # Elevation profile
    road_elevation_profile = road.find("elevationProfile")  # 在OpenDRIVE中，高程用<elevationProfile>元素中的<elevation>元素来表示
    if road_elevation_profile is not None:
        # 对于一条Road，其高程变化可能需要分区段用多个三次多项式来描述
        parse_opendrive_road_elevation_profile(newRoad, road_elevation_profile)

    # Lateral profile
    road_lateral_profile = road.find("lateralProfile")  # 在OpenDRIVE中，超高程用<lateralProfile>元素中的<superelevation>元素来表示
    if road_lateral_profile is not None:
        # 超高程用于描述道路横截面的倾斜角，与高程一样同样使用三次多项式描述
        parse_opendrive_road_lateral_profile(newRoad, road_lateral_profile)

    # Lanes
    lanes = road.find("lanes")  # 在OpenDRIVE中，车道由<road>元素中的<lanes>子节点表示，每条road有且只有一个<lanes>节点

    if lanes is None:
        raise Exception("Road must have lanes element")

    # Lane offset 车道偏移可用于将中心车道从道路参考线上位移
    for lane_offset in lanes.findall("laneOffset"):  # 在OpenDRIVE中，车道偏移用<lanes>元素内的<laneOffset>元素来表示
        # 对于一条Road，其中心车道的偏移可能需要通过多个三次多项式来描述
        parse_opendrive_road_lane_offset(newRoad, lane_offset)

    # Lane sections 车道可被分成多个段，每个车道段包含车道的一个固定编号（发生车道编号改变时，必须定义一个新的laneSection）
    for lane_section_id, lane_section in enumerate(
        road.find("lanes").findall("laneSection")  # 在OpenDRIVE中 ，车道段用<lanes>元素里的<laneSection>元素来表示
    ):
        parse_opendrive_road_lane_section(newRoad, lane_section_id, lane_section)

    # Objects
    # TODO implementation

    # Signals
    # TODO implementation

    calculate_lane_section_lengths(newRoad)

    opendrive.roads.append(newRoad)


def calculate_lane_section_lengths(newRoad):
    """计算road中每个section的长度，及section中各车道每个宽度公式下对应的长度
    road -> lanes -> lane_sections (lane_section.length) -> lane.widths (width.length)

    Args:
      newRoad:

    """
    # OpenDRIVE does not provide lane section lengths by itself, calculate them by ourselves
    for lane_section in newRoad.lanes.lane_sections:

        # 最后一个laneSection的长度可以通过road总长度减当前section起始位置s坐标得到
        if lane_section.idx + 1 >= len(newRoad.lanes.lane_sections):
            lane_section.length = newRoad.planView.length - lane_section.sPos

        # 非最后一个section的长度可以通过其下一个section的起始位置减去当前section的起始位置得到
        else:
            lane_section.length = (
                newRoad.lanes.lane_sections[lane_section.idx + 1].sPos
                - lane_section.sPos
            )

    # OpenDRIVE does not provide lane width lengths by itself, calculate them by ourselves
    for lane_section in newRoad.lanes.lane_sections:
        for lane in lane_section.allLanes:
            widthsPoses = np.array(
                [x.start_offset for x in lane.widths] + [lane_section.length]
            )
            widthsLengths = widthsPoses[1:] - widthsPoses[:-1]  # 后一个宽度的起始位置坐标减当前的，得到当前宽度公式下的长度

            for widthIdx, width in enumerate(lane.widths):
                width.length = widthsLengths[widthIdx]  # 根据宽度公式id确定对应宽度公式下对应的道路长度


def parse_opendrive_header(opendrive, header):
    """

    Args:
      opendrive: OpenDrive对象
      header: xodr文件树结构中的header节点lxml.etree._Element

    """

    # 实例化Header类
    parsed_header = Header(
        header.get("revMajor"),  # _Element.get()获得当前节点的对应属性
        header.get("revMinor"),
        header.get("name"),
        header.get("version"),
        header.get("date"),
        header.get("north"),
        header.get("south"),
        header.get("west"),
        header.get("vendor"),
    )
    # Reference
    if header.find("geoReference") is not None:  # 地理参考系，对于可视化而言并不涉及
        pass
        # TODO not implemented

    opendrive.header = parsed_header  # OpenDrive类实例对象的header属性赋值


def parse_opendrive_junction(opendrive, junction):
    """对xodr中单个junction对象进行解析，结果更新至OpenDrive对象中junctions属性中（append至列表中）
    Junction类的结构: Junction -> JunctionConnection -> JunctionConnectionLaneLink

    Args:
      opendrive:
      junction: OpenD文件中的junction节点lxml.etree._Element

    """
    newJunction = Junction()

    newJunction.id = int(junction.get("id"))
    newJunction.name = str(junction.get("name"))

    for connection in junction.findall("connection"):  # 获得connection子节点，表征交叉口涉及到的road与connecting road的连接关系

        newConnection = JunctionConnection()  # 连接关系确定后，相当于增加一条虚拟的connecting road

        newConnection.id = connection.get("id")
        newConnection.incomingRoad = connection.get("incomingRoad")  # 来路ID
        newConnection.connectingRoad = connection.get("connectingRoad")  # 联接道路ID，即交叉口内部的虚拟道路connectiong road
        newConnection.contactPoint = connection.get("contactPoint")  # 连接方式(start/end)

        for laneLink in connection.findall("laneLink"):  # 获得laneLink子节点，表征交叉口各road具体车道lane的连接关系

            newLaneLink = JunctionConnectionLaneLink()

            newLaneLink.fromId = laneLink.get("from")  # 来路对应的车道ID
            newLaneLink.toId = laneLink.get("to")  # connecting road对应的车道ID

            newConnection.addLaneLink(newLaneLink)

        newJunction.addConnection(newConnection)

    opendrive.junctions.append(newJunction)

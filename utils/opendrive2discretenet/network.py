# -*- coding: utf-8 -*-


"""Module to contain Network which can load an opendrive object and then export
to lanelets. Iternally, the road network is represented by ParametricLanes."""

from .discrete_network import *
from .opendriveparser.elements.opendrive import OpenDrive

from .utils import encode_road_section_lane_width_id
from .converter import OpenDriveConverter


def convert_to_new_lanelet_id(old_lanelet_id: str, ids_assigned: dict) -> int:
    """Convert the old lanelet ids (format 501.1.-1.-1) to newer,
    simpler ones (100, 101 etc.).

    Do this by consecutively assigning
    numbers, starting at 100, to the old_lanelet_id strings. Save the
    assignments in the dict which is passed to the function as ids_assigned.

    Args:
      old_lanelet_id: Old id with format "501.1.-1.-1".
      ids_assigned: Dict with all previous assignments

    Returns:
      The new lanelet id.

    """

    starting_lanelet_id = 100

    if old_lanelet_id in ids_assigned.keys():
        new_lanelet_id = ids_assigned[old_lanelet_id]
    else:
        try:
            new_lanelet_id = max(ids_assigned.values()) + 1
        except ValueError:
            new_lanelet_id = starting_lanelet_id
        ids_assigned[old_lanelet_id] = new_lanelet_id

    return new_lanelet_id


class Network:
    """Represents a network of parametric lanes, with a LinkIndex
    which stores the neighbor relations between the parametric lanes.

    Args:

    """

    def __init__(self):
        self._planes = []
        self._link_index = None

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def load_opendrive(self, opendrive: OpenDrive):
        """Load all elements of an OpenDRIVE network to a parametric lane representation

        Args:
          opendrive:

        """

        if not isinstance(opendrive, OpenDrive):
            raise TypeError()

        self._link_index = LinkIndex()
        self._link_index.create_from_opendrive(opendrive)  # 完成OpenD所有拓扑关系的提取，self._link_index._successors

        # Convert all parts of a road to parametric lanes (planes)
        for road in opendrive.roads:
            '''对该road的参考线进行线形插值的预计算
            返回num_steps * 4的浮点数组，num_steps为线性插值的粒度，4对应[pos, coord[0], coord[1], tang]
            其中，pos对应参考线s坐标系的位置，coord[0]、coord[1]分别为笛卡尔坐标系下的xy，tang为s坐标系pos位置对应的航向角
            '''
            road.planView.precalculate()

            # The reference border is the base line for the whole road
            # 在原先参考线基础上，添加offset信息，包括中心车道偏移的起始位置s_pos及其对应的线形参数[a,b,c,d]
            '''返回Border类的实例化对象reference_border
            其包含两个关键属性，<width_coefficient_offsets> <width_coefficients>，tborder (ds) = a + b*ds + c*ds² + d*ds³
            <width_coefficient_offsets>的List中每个元素offset，对应<width_coefficients>的List中每个元素[coeffs]
            '''
            reference_border = OpenDriveConverter.create_reference_border(
                road.planView, road.lanes.laneOffsets
            )

            # A lane section is the smallest part that can be converted at once
            '''将从OpenD文件中解析出的lane_section信息进行参数化
            输入：lane_section -> List[<class> LaneSection], reference_border -> <class> Border
            输出：parametric_lane_groups -> List[<class> ParametricLane] 当前lane_section所包含的所有车道的参数化信息
            '''
            for lane_section in road.lanes.lane_sections:

                parametric_lane_groups = OpenDriveConverter.lane_section_to_parametric_lanes(
                    lane_section, reference_border
                )

                self._planes.extend(parametric_lane_groups)

    # TODO: 基于解析得到的OpenD进一步将车道边界由参数形式转换为对应散点，以便进行地图可视化
    def export_discrete_network(
        self, filter_types: list = None
    ) -> DiscreteNetwork:
        """Export network as lanelet network.

        Args:
          filter_types: types of ParametricLane objects to be filtered. (Default value = None)

        Returns:
          The converted LaneletNetwork object.
        """

        # Convert groups to lanelets
        discrete_network = DiscreteNetwork()

        for parametric_lane in self._planes:
            if filter_types is not None and parametric_lane.type not in filter_types:
                continue

            discrete_lane = parametric_lane.to_discretelane()

            discrete_lane.predecessor = self._link_index.get_predecessors(parametric_lane.id_)
            discrete_lane.successor = self._link_index.get_successors(parametric_lane.id_)

            discrete_network.add_discretelane(discrete_lane)

        return discrete_network


class LinkIndex:
    """Overall index of all links in the file, save everything as successors, predecessors can be found via a reverse search"""

    def __init__(self):  # LinkIndex类对象储存各车道lane的连接关系，key为lane，value为[successor_encode_id]
        self._successors = {}

    def create_from_opendrive(self, opendrive):
        """对OpenD包含的拓扑关系进行解析，相关结果储存在self._successors中，key为lane_id，value为[successor_encode_id]

        Args:
          opendrive: OpenDrive style object.

        Returns:

        """
        # 将交叉口处包含的车道拓扑关系提取出来
        self._add_junctions(opendrive)
        # Extract link information from road lanes
        '''循环结构遵循Section 9 in OpenDRIVE 1.4
        树结构：Road -> lanes -> laneSection -> left\center\right -> lane
        '''
        for road in opendrive.roads:
            for lane_section in road.lanes.lane_sections:
                for lane in lane_section.allLanes:
                    # 对参数化车道的ID信息进行编码，输出'<road.id>.<lane_section.id>.<lane.id>.<width.id>'
                    parametric_lane_id = encode_road_section_lane_width_id(
                        road.id, lane_section.idx, lane.id, -1
                    )

                    '''对该Road所包含各section（除去最后一个section）对应后继successor信息进行编码
                    输出'<road.id>.<section_successor.id>.<lane_successor.id>.<width.id>'
                    并将该successor编码添加至储存路网拓扑关系的哈希表<self._successors>中
                    -------------------------------------------------------------------------
                    elif对该Road中最后一个section进行单独讨论，由于Road最后一个sectio仅可能被下一个Road/Junction后继
                    故若该section存在successor且successor为Road，则获得该Road的successorID（OpenD文件中的<road>节点）
                    同时完成编码（需考虑contactPoint）及路网拓扑关系哈希表<self._successors>的更新
                    '''
                    # Not the last lane section? > Next lane section in same road
                    if lane_section.idx < road.lanes.getLastLaneSectionIdx():
                        successorId = encode_road_section_lane_width_id(
                            road.id, lane_section.idx + 1, lane.link.successorId, -1
                        )

                        # bool值用来表征车道组，False为右车道组<right>
                        self.add_link(parametric_lane_id, successorId, lane.id >= 0)

                    # Last lane section! > Next road in first lane section
                    # Try to get next road
                    elif (
                        road.link.successor is not None and road.link.successor.elementType != "junction"  # 交叉口的拓扑关系提取已在self._add_junction()中完成
                    ):

                        next_road = opendrive.getRoad(road.link.successor.element_id)

                        if next_road is not None:

                            if road.link.successor.contactPoint == "start":
                                successorId = encode_road_section_lane_width_id(
                                    next_road.id, 0, lane.link.successorId, -1
                                )

                            else:  # contact point = end
                                successorId = encode_road_section_lane_width_id(
                                    next_road.id,
                                    next_road.lanes.getLastLaneSectionIdx(),
                                    lane.link.successorId,
                                    -1,
                                )
                            self.add_link(parametric_lane_id, successorId, lane.id >= 0)

                    '''对该Road所包含各section（除去第一个section）对应前驱predecessor信息进行编码
                    输出'<road.id>.<section_predecessor.id>.<lane_predecessor.id>.<width.id>'
                    并将该predecessor编码添加至储存路网拓扑关系的哈希表<self._successors>中
                    -------------------------------------------------------------------------
                    elif对该Road中第一个section进行单独讨论，由于Road第一个sectio仅可能被上一个Road/Junction前驱
                    故若该section存在predecessor且predecessor为Road，则获得该Road的predecessorID（OpenD文件中的<road>节点）
                    同时完成编码（需考虑contactPoint）及路网拓扑关系哈希表<self._successors>的更新
                    '''
                    # Not first lane section? > Previous lane section in same road
                    if lane_section.idx > 0:
                        predecessorId = encode_road_section_lane_width_id(
                            road.id, lane_section.idx - 1, lane.link.predecessorId, -1
                        )

                        self.add_link(predecessorId, parametric_lane_id, lane.id >= 0)

                    # First lane section! > Previous road
                    # Try to get previous road
                    elif (
                        road.link.predecessor is not None
                        and road.link.predecessor.elementType != "junction"
                    ):

                        prevRoad = opendrive.getRoad(road.link.predecessor.element_id)

                        if prevRoad is not None:

                            if road.link.predecessor.contactPoint == "start":
                                predecessorId = encode_road_section_lane_width_id(
                                    prevRoad.id, 0, lane.link.predecessorId, -1
                                )

                            else:  # contact point = end
                                predecessorId = encode_road_section_lane_width_id(
                                    prevRoad.id,
                                    prevRoad.lanes.getLastLaneSectionIdx(),
                                    lane.link.predecessorId,
                                    -1,
                                )
                            self.add_link(
                                predecessorId, parametric_lane_id, lane.id >= 0
                            )

    def add_link(self, parametric_lane_id, successor, reverse: bool = False):
        """

        Args:
          parametric_lane_id:
          successor:
          reverse:  (Default value = False)

        Returns:

        """

        # if reverse, call function recursively with switched parameters
        if reverse:
            self.add_link(successor, parametric_lane_id)
            return

        # 若该参数化lane未包含在路网集合<self._successors>中，则在该哈希表中建立对应key，value初始化为空列表。元素为对应successor
        if parametric_lane_id not in self._successors:
            self._successors[parametric_lane_id] = []

        # 若该参数化lane未包含该successor，则将其添加至哈希表对应key的value中
        if successor not in self._successors[parametric_lane_id]:
            self._successors[parametric_lane_id].append(successor)

    def _add_junctions(self, opendrive):
        """

        Args:
          opendrive:

        Returns:

        """
        # 交叉口内部联接道路connecting_road的行驶方向规定（每个connecting_road对应junction中的一个<connection>节点）：
        # <connection>节点描述了交叉口incoming_road与connecting_road的连接关系（与incomingRoad可为来路或去路，根据节点属性具体判断）
        # 1. 如果incomingRoad与connectingRoad的contact_point是start，则说明connecting_road正在沿着<laneLink>元素中的连接延申（from 来路laneid to 去路laneid）
        # 1.1 如果connecting_road的id为负数（下行），则connecting_road为incoming_road的后继道路（此处的连接关系指的是road层面的，而非lane层面）
        # 1.2 如果connecting_road的id为正数（上行），则connecting_road为incoming_road的前导道路
        # 2. 如果contact_point是end，则说明connecting road正在沿着<laneLink>元素中的连接的反方向延申
        for junction in opendrive.junctions:
            for connection in junction.connections:
                incoming_road = opendrive.getRoad(connection.incomingRoad)
                connecting_road = opendrive.getRoad(connection.connectingRoad)
                contact_point = connection.contactPoint

                for lane_link in connection.laneLinks:
                    if contact_point == "start":

                        # decide which lane section to use (first or last)
                        if lane_link.fromId < 0:  # 右车道组（下行），则说明与交叉口连接的必定为incoming_road的最后一个laneSection
                            lane_section_idx = (
                                incoming_road.lanes.getLastLaneSectionIdx()  # 获得最后一个laneSection的索引
                            )
                        else:
                            lane_section_idx = 0  # 左车道组（上行），说明与交叉口连接的必定为第一个laneSection

                        # 对incoming_road进行id重编码，编码结构：<road.id>.<lane_section.id>.<lane.id>.<width.id>
                        # 注意到width.id恒为-1，因为width.id在定义时便是根据sOffset属性进行升序排列，即距离上游前导车道的距离，故连接交叉口的必为最后一段
                        incoming_road_id = encode_road_section_lane_width_id(
                            incoming_road.id, lane_section_idx, lane_link.fromId, -1
                        )
                        connecting_road_id = encode_road_section_lane_width_id(
                            connecting_road.id, 0, lane_link.toId, -1
                        )
                        # 将该<connection>对应的拓扑关系储存进self._successor中，key为lane_id，value为successor_id
                        self.add_link(
                            incoming_road_id, connecting_road_id, lane_link.toId > 0  # 若下游lane_id为正，表示上行，实际走向变成connecting_road为incoming_road的前导，需翻转
                        )
                    else:
                        # decide which lane section to use (first or last)
                        if lane_link.fromId < 0:
                            lane_section_idx = 0  # end连接，fromId实质为toId，则对于下行车道，其必为去路，故对应车道组为0

                        else:
                            lane_section_idx = (
                                incoming_road.lanes.getLastLaneSectionIdx()  # fromId实质为toId，则对于上行车道，其必为来路，故对应车道组为-1
                            )
                        incoming_road_id = encode_road_section_lane_width_id(
                            incoming_road.id, 0, lane_link.fromId, -1
                        )
                        connecting_road_id = encode_road_section_lane_width_id(
                            connecting_road.id,
                            connecting_road.lanes.getLastLaneSectionIdx(),
                            lane_link.toId,
                            -1,
                        )
                        self.add_link(
                            incoming_road_id, connecting_road_id, lane_link.toId < 0
                        )

    def remove(self, parametric_lane_id):
        """

        Args:
          parametric_lane_id:

        """
        # Delete key
        if parametric_lane_id in self._successors:
            del self._successors[parametric_lane_id]

        # Delete all occurances in successor lists
        for successorsId, successors in self._successors.items():
            if parametric_lane_id in successors:
                self._successors[successorsId].remove(parametric_lane_id)

    def get_successors(self, parametric_lane_id: str) -> list:
        """

        Args:
          parametric_lane_id: Id of ParametricLane for which to search
            successors.

        Returns:
          List of successors belonging the the ParametricLane.
        Par
        """
        if parametric_lane_id not in self._successors:
            return []

        return self._successors[parametric_lane_id]

    def get_predecessors(self, parametric_lane_id: str) -> list:
        """

        Args:
          parametric_lane_id: Id of ParametricLane for which to search predecessors.

        Returns:
          List of predecessors of a ParametricLane.
        """
        predecessors = []
        for successor_plane_id, successors in self._successors.items():
            if parametric_lane_id not in successors:
                continue

            if successor_plane_id in predecessors:
                continue

            predecessors.append(successor_plane_id)

        return predecessors

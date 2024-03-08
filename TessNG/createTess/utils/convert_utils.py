import collections

from collections import defaultdict
from typing import List, Dict
from numpy import linspace, sqrt
from commonroad.scenario.scenario import Scenario

from ..opendrive2lanelet.network import Network
from ..opendrive2lanelet.opendriveparser.elements.opendrive import OpenDrive
from ..opendrive2lanelet.opendriveparser.elements.roadLanes import LaneSection
from ..opendrive2lanelet.opendriveparser.elements.roadElevationProfile import ElevationRecord


def convert_opendrive(opendrive: OpenDrive, filter_types: List, roads_info: Dict, context: Dict = None) -> Scenario:
    """
        将路网模型转换为 Scenario 对象
    Args:
        opendrive: 
        filter_types: 需提取的 opendrive 车道类型列表
        roads_info: 已被转换的路段详情字典
        context: 上下文对象

    Returns:

    """
    road_network = Network()
    road_network.load_opendrive(opendrive)
    return road_network.export_commonroad_scenario(filter_types=filter_types, roads_info=roads_info, context=context)


def calc_elevation(pos: float, elevations: List[ElevationRecord]):
    """
     计算路网任意一点的高程
    Args:
        pos: 距离路段起点位置
        elevations: 已排序的高程元素列表

    Returns:
     高程
    """
    if not elevations:
        return 0  # 无高程元素，默认高度为0
    # 获取相应的 elevation
    for e in elevations:
        if pos >= e.start_pos:
            elevation = e
    a, b, c, d = elevation.polynomial_coefficients
    ds = pos - elevation.start_pos  # 每当新的元素出现，`ds`则清零
    high = a + b * ds + c * ds ** 2 + d * ds ** 3
    return high


def calc_width(l1: List, l2: List):
    """
        通过两列表相应位置的点对比计算宽度序列
    """
    width_list = []
    for index in range(len(l1)):
        width = sqrt((l1[index][0] - l2[index][0]) ** 2 + (l1[index][1] - l2[index][1]) ** 2)
        width_list.append(width)
    return width_list


def convert_roads_info(opendrive: OpenDrive, step_length: int,
                       filter_types: List[str]) -> Dict:  # step_length需要对第三方包进行修改
    """
        对路段级别信息进行二次处理，获取有效信息，将其保存为字典
    Args:
        opendrive: 
        step_length: 
        filter_types: 

    Returns:

    """
    roads_info = {}
    for road in opendrive.roads:
        road_length = road.length
        planView = road.planView  # 每条道路有且仅有一条参考线，参考线通常在道路中心，但也可能有侧向偏移。
        road_points = {}

        # 计算高程信息
        elevations = [elevation[0] for elevation in road.elevationProfile.elevations]
        # 为了适配tess，将road按照section切分为多个link
        for section in road.lanes.lane_sections:
            section_id = section.idx
            section_length = section.length
            section_sPos = section.sPos
            section_ePos = section_sPos + section_length

            steps = int(section_length // step_length + 2)  # steps >= 2
            lengths = list(linspace(section_sPos, section_ePos, steps))
            points = []
            # offsets = []
            # right_offsets = []
            # lengths = list(road_section_distance[road.id][section_id].values())[0] # 尽量拟合路段与车道
            # 计算每一点的坐标和角度
            for length in lengths:
                # 根据点位 计算左右来向相应点的坐标/角度/高程
                position, angle = planView.calc_geometry(length)
                elevation_result = calc_elevation(length, elevations)

                points.append(
                    {
                        "position": list(position) + [elevation_result],
                        'angle': angle,
                        "offset": length,  # 记录在本section内此点的移动位置
                    }
                )

            # 左右方向参考线点计算不一样
            road_points[section_id] = {
                "right_points": points,
                "left_points": points[::-1],
                'sPos': section_sPos,
                'ePos': section_ePos,
                'length': section_length,
                'steps': steps,
                'lengths': lengths,
                "left_offsets": [road_length - offset for offset in lengths],
                "right_offsets": lengths,
                "elevations": [],
            }

        # # 计算每一段section 的高程信息
        # # 获取高程分段列表
        # elevations = [elevation[0] for elevation in road.elevationProfile.elevations]
        # for section_id, section_info in road_points.items():
        #     sPos = section_info["sPos"]
        #     ePos = section_info["ePos"]
        #     section_info["start_high"] = calc_elevation(sPos, elevations)
        #     section_info["end_high"] = calc_elevation(ePos, elevations)
        #     for index, length in enumerate(section_info["lengths"]):
        #         section_info["elevations"].append(calc_elevation(length, elevations))

        sections_mapping = convert_section_info(road.lanes.lane_sections, filter_types)
        # elevations [(e1),(e2),(e3)]  start_pos, road.elevationProfile.elevations[0][0].polynomial_coefficients
        roads_info[road.id] = {
            "name": road.name,
            "junction_id": road.junction and road.junction.id,  # -1 为非junction，此道路是在交叉口内部
            'road_points': road_points,  # 每个section 分开，方便微观处理,每条lane点位详情
            'length': road_length,
            'lane_sections': sections_mapping,  # lane 概况
        }

    return roads_info


def convert_lanes_info(opendrive: OpenDrive, scenario: Scenario, roads_info: Dict) -> Dict:
    """
        对车道级别信息进行二次处理，获取有效信息，将其保存为字典
    Args:
        opendrive: 
        scenario: 
        roads_info: 路段信息字典

    Returns:
        转换后的车道信息
    """
    # 获取 link与交叉口关系
    scenario_mapping = {
        "roads": {},
        "sections": {},
        "lanes": {},
    }
    for road in opendrive.roads:
        scenario_mapping["roads"][road.id] = road
        for sectionidx, section in enumerate(road.lanes.lane_sections):
            scenario_mapping["sections"][f"{road.id},{sectionidx}"] = section
            for lane in section.allLanes:
                scenario_mapping["lanes"][f"{road.id},{sectionidx},{lane.id}"] = lane
                if lane.id == 0:  # 中心车道信息保存在road info中
                    roads_info[road.id]['lane_sections'][sectionidx]['center_lane'] = {
                        "lane_id": lane.id,
                        "road_marks": lane.road_marks,
                        "widths": lane.widths,
                    }

    # 获取道路与路段关系
    lanes_info = defaultdict(dict)
    # 中心车道未转换，用参考线代替
    for lane in scenario.lanelet_network.lanelets:
        # center_lane
        # 获取所在路段
        lane_name = lane.lanelet_id
        ids = lane_name.split('.')
        road_id = int(ids[0])
        section_id = int(ids[1])
        lane_id = int(ids[2])
        road_marks = scenario_mapping["lanes"][f"{road_id},{section_id},{lane_id}"].road_marks

        # 计算车道宽度
        center_vertices, left_vertices, right_vertices = lane.center_vertices.tolist(), lane.left_vertices.tolist(), lane.right_vertices.tolist()
        widths = calc_width(left_vertices, right_vertices)

        # 添加高程
        if lane_id > 0:
            elevtions = [i["position"][2] for i in roads_info[road_id]["road_points"][section_id]['left_points']]
        else:
            elevtions = [i["position"][2] for i in roads_info[road_id]["road_points"][section_id]['right_points']]
        center_vertices = [list(center_vertices[index]) + [elevtions[index]] for index in range(len(elevtions))]
        left_vertices = [list(left_vertices[index]) + [elevtions[index]] for index in range(len(elevtions))]
        right_vertices = [list(right_vertices[index]) + [elevtions[index]] for index in range(len(elevtions))]

        # lane.lanelet_id 自定义的车道编号,取消转换后，指的就是原始编号
        lanes_info[lane.lanelet_id] = {
            "road_id": road_id,
            "section_id": section_id,
            "lane_id": lane_id,
            "left": {
                "lane_id": lane.adj_left,
                "same_direction": lane.adj_left_same_direction,
            },
            "right": {
                "lane_id": lane.adj_right,
                "same_direction": lane.adj_right_same_direction,
            },
            "predecessor_ids": lane.predecessor,
            "successor_ids": lane.successor,
            "type": lane.type,
            "name": lane_name,  # road_id+lane_section+lane_id+-1
            "center_vertices": center_vertices,
            "left_vertices": left_vertices,
            "right_vertices": right_vertices,
            "widths": widths,
            "road_marks": road_marks,
            'traffic_lights': list(lane.traffic_lights),
            'traffic_signs': list(lane.traffic_signs),
            'distance': list(lane.distance),
        }
    # 车道ID，中心车道为0， 正t方向升序，负t方向降序(基本可理解为沿参考线从左向右下降)
    return lanes_info


def convert_section_info(sections: LaneSection, filter_types: List[str]):
    """
    按section级别提取车道信息
    Args:
        sections: 所有的 section 元素
        filter_types: 需要被提取的opendrive车道类型

    Returns:

    """
    sections_mapping = collections.defaultdict(lambda: {
        'right': [],
        'center': [],
        'left': [],
        'all': [],
        'infos': {}
    }
                                               )
    for section in sections:
        section_id = sections.index(section)
        for lane in section.allLanes:
            if filter_types is not None and lane.type not in filter_types:
                continue
            if lane.id == 0:
                direction = 'center'
            elif lane.id >= 0:
                direction = 'left'
            else:
                direction = 'right'
            sections_mapping[section_id][direction].append(lane.id)
        sections_mapping[section_id]['all'] = sections_mapping[section_id]['right'] + sections_mapping[section_id][
            'left'] + sections_mapping[section_id]['center']
    return sections_mapping


# 提取车道限制信息
def lane_restrictions(lane):
    speed_info = []
    access_info = []
    for speed in lane.getElementsByTagName('speed'):
        speed_info.append(
            {
                "sOffset ": speed.getAttribute("sOffset"),
                "max": float(speed.getAttribute("max")),
                "unit": speed.getAttribute("unit"),
            }
        )
    for access in lane.getElementsByTagName('access'):
        access_info.append(
            {
                "sOffset ": access.getAttribute("sOffset"),
                "rule ": access.getAttribute("rule"),
                "restriction ": access.getAttribute("restriction"),
            }
        )
    return {
        "speeds": speed_info,
        'accesss': access_info,
    }

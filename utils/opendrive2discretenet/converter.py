# -*- coding: utf-8 -*-

"""Module for logic behind converting OpenDrive to ParametricLanes."""

from typing import Tuple, List
from .plane_elements.plane import (
    ParametricLane,
    ParametricLaneBorderGroup,
)
from .plane_elements.plane_group import ParametricLaneGroup
from .plane_elements.border import Border
from .utils import encode_road_section_lane_width_id


class OpenDriveConverter:
    """Class for static methods to convert lane_sections to parametric_lanes."""

    @staticmethod
    def create_reference_border(plan_view, lane_offsets) -> Border:
        """Create the most inner border from a PlanView.
        This border is used as a reference for other
        borders which rely on the PlanView.

        Args:
          plan_view: PlanView object from OpenDrive which specifies the geometry
            of the reference path.
          lane_offsets: Object which contains information about width offset of reference
            path the plain_view path.

        Returns:
           The reference border on which all other borders in this lane section are based upon.

        """

        reference_border = Border()

        # Set reference to plan view
        reference_border.reference = plan_view

        # Lane offsets will be coeffs
        # this has to be done if the reference path has the laneoffset attribute
        # and thus is different to the geometry described in the plan_view
        # openDRIVE lets multiple laneOffsets start at the same position
        # but only the last one counts -> delete all previous ones
        if any(lane_offsets):
            '''LaneOffset类对象，继承于RoadRecord基类，包含起始位置s_pos与线形参数[a,b,c,d]'''
            for lane_offset in lane_offsets:
                if lane_offset.start_pos in reference_border.width_coefficient_offsets:
                    # offset is already there, delete previous entries
                    # 删除重复的offset信息
                    idx = reference_border.width_coefficient_offsets.index(
                        lane_offset.start_pos
                    )
                    del reference_border.width_coefficient_offsets[idx]
                    del reference_border.width_coefficients[idx]
                reference_border.width_coefficient_offsets.append(lane_offset.start_pos)
                reference_border.width_coefficients.append(
                    lane_offset.polynomial_coefficients  # List[coefficients]
                )
        else:
            reference_border.width_coefficient_offsets.append(0.0)
            reference_border.width_coefficients.append([0.0])

        '''返回Border类的实例化对象，其包含两个关键属性，<width_coefficient_offsets> <width_coefficients>
        tborder (ds) = a + b*ds + c*ds² + d*ds³
        <width_coefficient_offsets>的List中每个元素offset，对应<width_coefficients>的List中每个元素[coeffs]
        '''
        return reference_border

    @staticmethod
    def lane_section_to_parametric_lanes(
        lane_section, reference_border
    ) -> List[ParametricLaneGroup]:
        """Convert a whole lane section into a list of ParametricLane objects.

        Args:
          lane_section: List[<class> LaneSection] (road.lanes.lane_sections -> List[lane_section])
          reference_border: <class> Border in border.py

        Returns:

        """

        plane_groups = []  # 获取当前lane_section所包含的所有车道lane的参数化信息

        for side in ["right", "left"]:

            # lanes loaded by opendriveparser are aleady sorted by id
            # coeff_factor decides if border is left or right of the reference line
            lanes = (
                lane_section.rightLanes if side == "right" else lane_section.leftLanes
            )
            coeff_factor = -1.0 if side == "right" else 1.0  # 右车道组的车道id从-1开始，左车道组从1开始

            # Most inner border gets added first
            '''储存车道边界，其中参考线边界是最内侧的车道边界'''
            lane_borders = [reference_border]

            # copy reference border, but set refOffset to start of lane_section

            for lane in lanes:
                '''对车道Lane的相邻车道信息进行编码，返回相应的编码值，内侧（靠近参考线）车道编码、外侧车道编码、内侧车道是否同向(bool)
                inner/outer_neighbour_id -> <road.id>.<lane_section.id>.<inner_lane.id/outer_lane.id>.<width.id>
                '''
                inner_neighbour_id, outer_neighbour_id, inner_neighbour_same_dir = OpenDriveConverter.determine_neighbours(
                    lane
                )

                # Create outer lane border
                '''获得车道的外侧边界（内侧不需要计算，因为其是内侧车道的外侧边界，必然已经完成计算）
                输入：本侧车道组所有车道边界<lane_borders>，当前车道对象<lane>，方位标识<coeff_factor>（左为1，右为-1）
                '''
                # 获得该lane的外侧边界，Border类的实例化对象outer_parametric_lane_border
                outer_parametric_lane_border = OpenDriveConverter._create_outer_lane_border(
                    lane_borders, lane, coeff_factor
                )

                lane_borders.append(outer_parametric_lane_border)

                '''储存车道Lane的相关参数化信息，包括本车道id编码，内侧车道id编码，内侧车道同向指标，外侧车道id编码
                本车道id编码 -> (str) 'road_id.lane_section_id.lane_id.width_id'
                内侧/外侧车道id编码 -> (str) 'road_id.lane_section_id.inner/outer_lane_id.width_id'
                内侧车道同向指标<inner_neighbour_same_direction> -> bool
                '''
                plane_group = ParametricLaneGroup(
                    id_=encode_road_section_lane_width_id(
                        lane_section.parentRoad.id, lane_section.idx, lane.id, -1
                    ),
                    inner_neighbour=inner_neighbour_id,
                    inner_neighbour_same_direction=inner_neighbour_same_dir,
                    outer_neighbour=outer_neighbour_id,
                )

                # Create new lane for each width segment
                '''针对车道Lane，对于每个宽度变化（准确而言是，宽度变化到需要修改表达式参数）创建单独的车道参数'''
                for width in lane.widths:
                    parametric_lane = OpenDriveConverter.create_parametric_lane(
                        lane_borders, width, lane
                    )
                    parametric_lane.reverse = bool(lane.id > 0)
                    plane_group.append(parametric_lane)

                # if lane borders are specified by offsets instead of widths
                # for borders in lane.borders:

                if plane_group.length > 0:
                    plane_groups.append(plane_group)

        return plane_groups

    @staticmethod
    def create_parametric_lane(lane_borders, width, lane) -> ParametricLane:
        """Create a parametric lane for a certain width section.

        Args:
          lane_borders: Array with already created lane borders.
          width: Width section with offset and coefficient information.
          lane: Lane in which new parametric lane is created.
          prev_inner_neighbours: Inner neighbours of parametric lane.

        Returns:
          A ParametricLane object with specified borders and a unique id.
        """

        border_group = ParametricLaneBorderGroup(
            inner_border=lane_borders[-2],  # 内侧边界，Border类对象
            outer_border=lane_borders[-1],  # 外侧边界
            inner_border_offset=width.start_offset + lane_borders[-1].ref_offset,  # 除1/-1车道的ref_offset为参考线的sPos外，其他车道都为0
            outer_border_offset=width.start_offset,
        )
        parametric_lane = ParametricLane(
            id_=encode_road_section_lane_width_id(
                lane.lane_section.parentRoad.id,
                lane.lane_section.idx,
                lane.id,
                width.idx,
            ),
            type_=lane.type,
            length=width.length,
            border_group=border_group,
        )
        return parametric_lane

    @staticmethod
    def _create_outer_lane_border(lane_borders, lane, coeff_factor) -> Border:
        """Create an outer lane border of a lane.
        InnerBorder is already saved in lane_borders, as it is
        the outer border of the inner neighbour of the lane.

        Args:
          lane_borders: Previous calculated lane borders of more inner lanes.
          lane: Lane for which outer border shall be created.
            This is specified in parameter ds of curve length.
          coeff_factor: factor of -1 or 1, dependent on which side of the reference
            path the lane is (right side is -1).

        Returns:
          The created outer lane border.

        """
        # Create outer lane border
        # Offset from reference border is already included in first inner border
        # (lane_border[0])
        # reference_border starts at beginning of road, prev: lane section
        border = Border()
        if len(lane_borders) == 1:
            border.ref_offset = lane.lane_section.sPos  # 当前section的中心线起始位置的s坐标

        # last created border
        if lane.has_border_record:
            border.reference = lane_borders[0]
        else:
            border.reference = lane_borders[-1]

        for width in lane.widths:
            border.width_coefficient_offsets.append(width.start_offset)  # 区段起点对应s坐标
            border.width_coefficients.append(
                [x * coeff_factor for x in width.polynomial_coefficients]  # 三次多项式参数
            )
        return border

    @staticmethod
    def determine_neighbours(lane) -> Tuple[str, str, bool]:
        """

        Args:
          lane:

        Returns:

        """
        if abs(lane.id) > 1:

            if lane.id > 0:
                inner_lane_id = lane.id - 1
                outer_lane_id = lane.id + 1
            else:
                inner_lane_id = lane.id + 1
                outer_lane_id = lane.id - 1

            inner_neighbour_same_dir = True

        else:
            # Skip lane id 0

            if lane.id == 1:
                inner_lane_id = -1
                outer_lane_id = 2
            else:
                inner_lane_id = 1
                outer_lane_id = -2
            inner_neighbour_same_dir = False

        inner_neighbour_id = encode_road_section_lane_width_id(
            lane.lane_section.parentRoad.id, lane.lane_section.idx, inner_lane_id, -1
        )

        outer_neighbour_id = encode_road_section_lane_width_id(
            lane.lane_section.parentRoad.id, lane.lane_section.idx, outer_lane_id, -1
        )

        return inner_neighbour_id, outer_neighbour_id, inner_neighbour_same_dir

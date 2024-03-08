# -*- coding: utf-8 -*-

from typing import Tuple
import numpy as np
from ..discrete_network import DiscreteLane


class ParametricLaneGroup:
    """A group of parametric_lanes can be converted to a
    lanelet just like a single parametric_lane.
    """

    def __init__(
        self,
        id_=None,
        parametric_lanes=None,
        inner_neighbour=None,
        inner_neighbour_same_direction=True,
        outer_neighbour=None,
    ):

        self._geo_lengths = [np.array([0.0])]
        self.parametric_lanes = []
        self.id_ = id_
        self.inner_neighbour = inner_neighbour
        self.inner_neighbour_same_direction = inner_neighbour_same_direction
        self.outer_neighbour = outer_neighbour

        if parametric_lanes is not None:
            if isinstance(parametric_lanes, list):
                self.extend(parametric_lanes)
            else:
                self.append(parametric_lanes)

    def append(self, parametric_lane):
        """Append lane to start or end of interal list of ParametricLane objects.

        If the parametric_lane is reverse, it is inserted at the start.
        Args:
          parametric_lane: Lane to be inserted either at beginning or end of list.
        """
        if parametric_lane.reverse:
            self.parametric_lanes.insert(0, parametric_lane)
        else:
            self.parametric_lanes.append(parametric_lane)

        self._add_geo_length(parametric_lane.length, parametric_lane.reverse)

    def extend(self, plane_list: list):
        """Extend own ParametricLanes with new ones.

        Assumes ParametricLane objects in plane_list are already in order.

        Args:
          plane_list: List with ParametricLane objects.
        """
        for plane in plane_list:
            self.parametric_lanes.append(plane)
            self._add_geo_length(plane.length)

    def _add_geo_length(self, length: float, reverse: bool = False):
        """Add length of a ParametricLane to the array which keeps track
        at which position which ParametricLane is placed.

        This array is used for quickly accessing
        the proper ParametricLane for calculating a position.

        Args:
          length: Length of ParametricLane to be added.

        """
        if reverse:
            self._geo_lengths = np.insert(self._geo_lengths, 1, length)
            self._geo_lengths[2:] = [
                x + length for i, x in enumerate(self._geo_lengths) if i > 1
            ]
        else:
            self._geo_lengths = np.append(
                self._geo_lengths, length + self._geo_lengths[-1]
            )

    @property
    def type(self) -> str:
        """Get type of ParametricLaneGroup.


        Returns:
          Type of first ParametricLane in this Group.
        """
        return self.parametric_lanes[0].type_

    @property
    def length(self) -> float:
        """Length of all ParametricLanes which are collected in this ParametricLaneGroup.

        Returns:
          Accumulated length of ParametricLaneGroup.
        """

        return sum([x.length for x in self.parametric_lanes])

    def has_zero_width_everywhere(self) -> bool:
        """Checks if width is zero at every point of this ParametricLaneGroup.

        Returns:
          True if every ParametricLane has width_coefficients equal to only zero.
        """
        return all(
            [plane.has_zero_width_everywhere() for plane in self.parametric_lanes]
        )

    def to_discretelane(self, precision: float = 0.5) -> DiscreteLane:
        """Convert a ParametricLaneGroup to a Lanelet.

        Args:
          precision: Number which indicates at which space interval (in curve parameter ds)
            the coordinates of the boundaries should be calculated.
          mirror_border: Which lane to mirror, if performing merging or splitting of lanes.
          distance: Distance at start and end of lanelet, which mirroring lane should
            have from the other lane it mirrors.

        Returns:
          Created Lanelet.

        """
        left_vertices, right_vertices = np.array([]), np.array([])

        for parametric_lane in self.parametric_lanes:  # 遍历每一宽度区段对应的ParametricLane类对象

            # 获得当前宽度区段下车道的左右边界散点
            local_left_vertices, local_right_vertices = parametric_lane.calc_vertices(
                precision=precision
            )

            if local_left_vertices is None:
                continue

            try:  # 判断当前区段起始位置与上一区段终点位置是否为同一个点
                if np.isclose(left_vertices[-1], local_left_vertices[0]).all():
                    idx = 1
                else:
                    idx = 0
                # 若当前lane起点与上游lane相同，则从起点下游的位置点开始，将local_left_vertices与left_vertices垂直相连
                left_vertices = np.vstack((left_vertices, local_left_vertices[idx:]))
                right_vertices = np.vstack((right_vertices, local_right_vertices[idx:]))
            except IndexError:  # left_vertices为空，当前lane为第一个lane
                left_vertices = local_left_vertices
                right_vertices = local_right_vertices

        # 计算lane中线对应的s_pos
        center_vertices = np.array(
            [(l + r) / 2 for (l, r) in zip(left_vertices, right_vertices)]
        )

        lanelet = DiscreteLane(
            self, left_vertices, center_vertices, right_vertices, self.id_
        )

        return lanelet

    def calc_border(self, border: str, s_pos: float, width_offset: float = 0.0):
        """Calc vertices point of inner or outer Border.

        Args:
          border: Which border to calculate (inner or outer).
          s_pos: Position of parameter ds where to calc the
        Cartesian coordinates
          width_offset: Offset to add to calculated width in reference
           to the reference border. (Default value = 0.0)

        Returns:
          Cartesian coordinates of point on inner border
            and tangential direction, too.

        """
        try:
            # get index of geometry which is at s_pos
            mask = self._geo_lengths > s_pos
            sub_idx = np.argmin(self._geo_lengths[mask] - s_pos)
            plane_idx = np.arange(self._geo_lengths.shape[0])[mask][sub_idx] - 1
        except ValueError:
            # s_pos is after last geometry because of rounding error
            if np.isclose(s_pos, self._geo_lengths[-1]):
                plane_idx = self._geo_lengths.size - 2
            else:
                raise Exception(
                    f"Tried to calculate a position outside of the borders of the reference path at s={s_pos}"
                    f", but path has only length of l={ self._geo_lengths[-1]}"
                )

        return self.parametric_lanes[plane_idx].calc_border(
            border, s_pos - self._geo_lengths[plane_idx], width_offset
        )

    def _calc_border_positions(self, precision: float) -> np.ndarray:
        """Determine the positions along the border where the coordinates
        of the border should be calculated.

        Args:
          precision: Number which indicates at which space interval (in curve parameter ds)
            the coordinates of the boundaries should be calculated.

        Returns:
          Array with the ordered positions.
        """
        poses = np.array([])
        for i, parametric_lane in enumerate(self.parametric_lanes):
            num_steps = int(max(2, np.ceil(parametric_lane.length / float(precision))))
            if not i:
                idx = 0
            else:
                idx = 1

            poses = np.append(
                poses,
                np.linspace(0, parametric_lane.length, num_steps)[idx::] + self._geo_lengths[i],
            )

        return poses

    def maximum_width(self) -> float:
        """Get the maximum width of the lanelet.

        Returns:
          Maximum width of all ParametricLanes in this Group.
        """
        total_maximum = 0

        for plane in self.parametric_lanes:
            _, maximum = plane.maximum_width()
            if maximum > total_maximum:
                total_maximum = maximum

        return total_maximum

    def first_zero_width_change_position(
        self, reverse: bool = False, reference_width: float = 0.0
    ) -> float:
        """Get the earliest point of the ParametricLaneGroup where the width change is zero.

        Args:
          reverse: True if checking should start from end of lanelet.
          reference_width: Width for which width at zero width change position has
            to be greater as.

        Returns:
          Position of ParametricLaneGroup (in curve parameter ds) where width change is zero.
        """
        s_pos = 0
        positions = []

        # total_maximum = self.maximum_width()

        for plane in self.parametric_lanes:
            zero_change_positions = plane.zero_width_change_positions()
            for pos in zero_change_positions:
                positions.append((pos + s_pos, plane.calc_width(pos)))
            s_pos += plane.length

        if reverse:
            positions = list(reversed(positions))

        # if lanelet has zero width change and its width
        # is either near the maximum or it is greater than the reference width
        # the position can be used
        for pos, val in positions:
            if val > 0.9 * reference_width or val > 0.9 * self.maximum_width():
                if (pos == 0.0 and not reverse) or (pos == self.length and reverse):
                    continue
                return pos, val

        return None, None

    # FIXME: unused, candidate for deletion
    def first_width_change_reversal_point(self, reverse: bool = False) -> float:
        """Get the first point where width change is zero or point is between two ParametricLanes.

        This method considers that not all positions where the derivative of the width
        (which is a polynomial in s) is a position where the width derivative is zero.
        The other option is that between two ParametricLanes of this Group the width
        derivative changes not constant.

        Args:
          reverse: True if checking should start from end of lanelet.
        Returns:
          Position of ParametricLaneGroup (in curve parameter ds) where width derivate
            changes from positive to negative.
        """

        width_derivative_zero = self.first_zero_width_change_position(reverse)
        between_plane_width_dev_changes = [width_derivative_zero]
        for i in range(len(self.parametric_lanes) - 1):
            first_plane = self.parametric_lanes[i]
            second_plane = self.parametric_lanes[i + 1]
            if first_plane.calc_width(
                0.99 * first_plane.length
            ) > second_plane.calc_width(0 + 0.01 * second_plane.length):
                between_plane_width_dev_changes.append(
                    first_plane.length + self._geo_lengths[i]
                )

        return next(sorted(between_plane_width_dev_changes))

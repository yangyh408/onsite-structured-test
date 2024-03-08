# -*- coding: utf-8 -*-

from typing import Tuple
import numpy as np

from .geometry import (
    Geometry,
    Line,
    Spiral,
    ParamPoly3,
    Arc,
    Poly3,
)


class PlanView:
    """The plan view record contains a series of geometry records
    which define the layout of the road's
    reference line in the x/y-plane (plan view).

    (Section 5.3.4 of OpenDRIVE 1.4)
    """

    def __init__(self):
        self._geometries = []
        self._precalculation = None
        self.should_precalculate = 0
        self._geo_lengths = np.array([0.0])
        self.cache_time = 0
        self.normal_time = 0

    def _add_geometry(self, geometry: Geometry, should_precalculate: bool):
        """

        Args:
          geometry:
          should_precalculate:

        """
        self._geometries.append(geometry)
        if should_precalculate:
            self.should_precalculate += 1
        else:
            self.should_precalculate -= 1
        self._add_geo_length(geometry.length)

    def addLine(self, start_pos, heading, length):
        """添加线型为Line的区段对应的几何参数，以Line类对象append至PlanView类对象的geometries属性中
        Line类对象继承自抽象基类Geometry，传入三个参数：起始位置xy坐标、航向角、长度；且包含一个必须实现的基类方法calc_position(s_pos)，即基于s坐标计算对应xy坐标

        Args:
          start_pos:
          heading:
          length:

        """
        self._add_geometry(Line(start_pos, heading, length), False)

    def addSpiral(self, start_pos, heading, length, curvStart, curvEnd):
        """

        Args:
          start_pos:
          heading:
          length:
          curvStart:
          curvEnd:

        """
        self._add_geometry(Spiral(start_pos, heading, length, curvStart, curvEnd), True)

    def addArc(self, start_pos, heading, length, curvature):
        """

        Args:
          start_pos:
          heading:
          length:
          curvature:

        """
        self._add_geometry(Arc(start_pos, heading, length, curvature), True)

    def addParamPoly3(
        self, start_pos, heading, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange
    ):
        """

        Args:
          start_pos:
          heading:
          length:
          aU:
          bU:
          cU:
          dU:
          aV:
          bV:
          cV:
          dV:
          pRange:

        """
        self._add_geometry(
            ParamPoly3(
                start_pos, heading, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange
            ),
            True,
        )

    def addPoly3(self, start_pos, heading, length, a, b, c, d):
        """

        Args:
          start_pos:
          heading:
          length:
          a:
          b:
          c:
          d:
        """
        self._add_geometry(Poly3(start_pos, heading, length, a, b, c, d), True)

    # 储存构成该参考线的每种线形对应的长度
    def _add_geo_length(self, length: float):
        """Add length of a geometry to the array which keeps track at which position
        which geometry is placed. This array is used for quickly accessing the proper geometry
        for calculating a position.

        Args:
          length: Length of geometry to be added.

        """
        self._geo_lengths = np.append(self._geo_lengths, length + self._geo_lengths[-1])

    @property
    def length(self) -> float:
        """Get length of whole plan view"""

        return self._geo_lengths[-1]

    def calc(self, s_pos: float) -> Tuple[np.ndarray, float]:
        """Calculate position and tangent at s_pos.

        Either interpolate values if it possible or delegate calculation
        to geometries.

        Args:
          s_pos: Position on PlanView in ds.

        Returns:
          Position (x,y) in cartesion coordinates.
          Angle in radians at position s_pos.
        """

        if self._precalculation is not None:
            # interpolate values
            return self.interpolate_cached_values(s_pos)

        # start = time.time()
        result_pos, result_tang = self.calc_geometry(s_pos)  # Tuple[np.ndarray, float]
        # end = time.time()
        # self.normal_time += end - start
        return result_pos, result_tang

    def interpolate_cached_values(self, s_pos: float) -> Tuple[np.ndarray, float]:
        """Calc position and tangent at s_pos by interpolating values
        in _precalculation array.

        Args:
          s_pos: Position on PlanView in ds.

        Returns:
          Position (x,y) in cartesion coordinates.
          Angle in radians at position s_pos.

        """
        # start = time.time()
        # we need idx for angle interpolation
        # so idx can be used anyway in the other np.interp function calls
        idx = np.abs(self._precalculation[:, 0] - s_pos).argmin()
        if s_pos - self._precalculation[idx, 0] < 0 or idx + 1 == len(
            self._precalculation
        ):
            idx -= 1
        result_pos_x = np.interp(
            s_pos,
            self._precalculation[idx:idx + 2, 0],
            self._precalculation[idx:idx + 2, 1],
        )
        result_pos_y = np.interp(
            s_pos,
            self._precalculation[idx:idx + 2, 0],
            self._precalculation[idx:idx + 2, 2],
        )
        result_tang = self.interpolate_angle(idx, s_pos)
        result_pos = np.array((result_pos_x, result_pos_y))
        # end = time.time()
        # self.cache_time += end - start
        return result_pos, result_tang

    def interpolate_angle(self, idx: int, s_pos: float) -> float:
        """Interpolate two angular values using the shortest angle between both values.

        Args:
          idx: Index where values in _precalculation should be accessed.
          s_pos: Position at which interpolated angle should be calculated.

        Returns:
          Interpolated angle in radians.

        """
        angle_prev = self._precalculation[idx, 3]
        angle_next = self._precalculation[idx + 1, 3]
        pos_prev = self._precalculation[idx, 0]
        pos_next = self._precalculation[idx + 1, 0]

        shortest_angle = ((angle_next - angle_prev) + np.pi) % (2 * np.pi) - np.pi
        return angle_prev + shortest_angle * (s_pos - pos_prev) / (pos_next - pos_prev)

    def calc_geometry(self, s_pos: float) -> Tuple[np.ndarray, float]:
        """Calc position and tangent at s_pos by delegating calculation to geometry.

        Args:
          s_pos: Position on PlanView in ds.

        Returns:
          Position (x,y) in cartesion coordinates.
          Angle in radians at position s_pos.

        """
        try:
            # get index of geometry which is at s_pos
            mask = self._geo_lengths > s_pos  # 返回np.array[bool,..]
            sub_idx = np.argmin(self._geo_lengths[mask] - s_pos)  # 返回子数组对应的索引
            '''获得该s_pos对应的参考线子段索引
            np.arange(self._geo_lengths.shape[0])获得self._geo_lengths数组每个元素的索引
            np.arange(self._geo_lengths.shape[0])[mask]获得mask中为true的对应元素构成的子数组
            np.arange(self._geo_lengths.shape[0])[mask][sub_idx] - 1获得参考线对应子段索引（-1：1.5位于1与2之间，属于以1开头的子段）
            '''
            geo_idx = np.arange(self._geo_lengths.shape[0])[mask][sub_idx] - 1
        except ValueError:
            # s_pos is after last geometry because of rounding error
            if np.isclose(s_pos, self._geo_lengths[-1]):
                geo_idx = self._geo_lengths.size - 2
            else:
                raise Exception(
                    f"Tried to calculate a position outside of the borders of the reference path at s={s_pos}"
                    f", but path has only length of l={ self._geo_lengths[-1]}"
                )

        # geo_idx is index which geometry to use
        # 输入当前s_pos在对应参考线子段上的相对位置（相对于start_position），输出对应的xy坐标及航向角
        return self._geometries[geo_idx].calc_position(
            s_pos - self._geo_lengths[geo_idx]
        )

    def precalculate(self, precision: float = 0.5):
        """Precalculate coordinates of planView to save computing resources and time.
        Save result in _precalculation array.

        Args:
          precision: Precision with which to calculate points on the line

        """
        # start = time.time()
        # this threshold was determined by quick prototyping tests
        # (trying different numbers and minimizing runtime)
        if self.should_precalculate < 1:
            return

        num_steps = int(max(2, np.ceil(self.length / precision)))
        positions = np.linspace(0, self.length, num_steps)  # 对参考线进行线性插值
        self._precalculation = np.empty([num_steps, 4])  # 返回num_steps*4的浮点数组，4对应[pos, coord[0], coord[1], tang]
        for i, pos in enumerate(positions):
            coord, tang = self.calc_geometry(pos)  # 输出对应点的xy坐标及航向角Tuple[np.ndarray, float]
            self._precalculation[i] = (pos, coord[0], coord[1], tang)
        # end = time.time()
        # self.cache_time += end - start

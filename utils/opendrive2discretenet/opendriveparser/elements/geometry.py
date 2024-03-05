# -*- coding: utf-8 -*-

import abc
import numpy as np
from .eulerspiral import EulerSpiral


class Geometry(abc.ABC):
    '''将道路的几何特征（s轴起始点、长度、航向角）定义为抽象基类，用以其相关子类继承（OpenD中为道路参考线采用的线形）
    抽象基类(abstract base class, abc)不能被实例化，只能被继承，其定义了一些事物所共同具备的规则和属性
    '''
    """A road geometry record defines the layout of the road's reference
    line in the in the x/y-plane (plan view).

    The geometry information is split into a header which is common to all geometric elements.

    (Section 5.3.4.1 of OpenDRIVE 1.4)
    """

    __metaclass__ = abc.ABCMeta

    def __init__(self, start_position: float, heading: float, length: float):
        self._start_position = np.array(start_position)
        self._length = length
        self._heading = heading

    @property
    def start_position(self) -> float:
        """Returns the overall geometry length"""
        return self._start_position

    @property
    def length(self) -> float:
        """Returns the overall geometry length"""
        return self._length

    @property
    def heading(self) -> float:
        """Get heading of geometry.

        Returns:
          Heading, in which direction the geometry heads at start.
        """
        return self._heading

    # 定义抽象方法，任何从该抽象基类中继承的子类都必须实现对应的抽象方法，否则子类将无法实例化
    @abc.abstractmethod
    def calc_position(self, s_pos):
        """Calculates the position of the geometry as if the starting point is (0/0)
        基于参考线s坐标计算对应的xy坐标

        Args:
          s_pos:

        Returns:

        """
        return


class Line(Geometry):
    """This record describes a straight line as part of the road’s reference line.


    (Section 5.3.4.1.1 of OpenDRIVE 1.4)
    """

    def calc_position(self, s_pos):
        """

        Args:
          s_pos:

        Returns:

        """
        # 由s轴坐标计算对应的xy轴坐标，以array数组储存坐标
        pos = self.start_position + np.array(
            [s_pos * np.cos(self.heading), s_pos * np.sin(self.heading)]
        )
        tangent = self.heading

        return (pos, tangent)


class Arc(Geometry):
    """This record describes an arc as part of the road’s reference line.


    (Section 5.3.4.1.3 of OpenDRIVE 1.4)
    """

    def __init__(self, start_position, heading, length, curvature):
        self.curvature = curvature
        super().__init__(start_position=start_position, heading=heading, length=length)

    def calc_position(self, s_pos):
        """

        Args:
          s_pos:

        Returns:

        """
        c = self.curvature
        hdg = self.heading - np.pi / 2

        a = 2 / c * np.sin(s_pos * c / 2)
        alpha = (np.pi - s_pos * c) / 2 - hdg

        dx = -1 * a * np.cos(alpha)
        dy = a * np.sin(alpha)

        pos = self.start_position + np.array([dx, dy])
        tangent = self.heading + s_pos * self.curvature

        return (pos, tangent)


class Spiral(Geometry):
    """This record describes a spiral as part of the road’s reference line.

    For this type of spiral, the curvature
    change between start and end of the element is linear.

    (Section 5.3.4.1.2 of OpenDRIVE 1.4)
    """

    def __init__(self, start_position, heading, length, curvStart, curvEnd):
        self._curvStart = curvStart
        self._curvEnd = curvEnd

        # 子类Spiral的init方法覆盖了父类Geometry的init，通过super().__init__()将父类init继承回来
        super().__init__(start_position=start_position, heading=heading, length=length)

        # EulerSpiral.createFromLengthAndCurvature为静态方法，不需要实例化类便可直接访问
        self._spiral = EulerSpiral.createFromLengthAndCurvature(
            self.length, self._curvStart, self._curvEnd
        )

    def calc_position(self, s_pos):
        """

        Args:
          s_pos:

        Returns:

        """
        (x, y, t) = self._spiral.calc(
            s_pos,
            self.start_position[0],
            self.start_position[1],
            self._curvStart,
            self.heading,
        )

        return (np.array([x, y]), t)


class Poly3(Geometry):
    """This record describes a cubic polynomial as part of the road’s reference line.


    (Section 5.3.4.1.4 of OpenDRIVE 1.4)
    """

    def __init__(self, start_position, heading, length, a, b, c, d):
        self._a = a
        self._b = b
        self._c = c
        self._d = d
        super().__init__(start_position=start_position, heading=heading, length=length)

        # raise NotImplementedError()

    def calc_position(self, s_pos):
        """对于三次多项式曲线而言，其在惯性坐标xy中，会因为对应参考线起点航向角的原因，出现一个xy不是唯一对应的情况
        需要在计算位置时，将参考坐标系转至参考系坐标系st中，即t(s)=a + b*t + c*s**2 + d*s**3

        Args:
          s_pos:

        Returns:

        """
        # TODO untested

        # Calculate new point in s_pos/t coordinate system
        coeffs = [self._a, self._b, self._c, self._d]

        # 基于s坐标求得其对应的t坐标
        t = np.polynomial.polynomial.polyval(s_pos, coeffs)

        # Rotate and translate
        # 基于st坐标计算对应在xy坐标中的对应截距（不包含参考线起点的截距）
        srot = s_pos * np.cos(self.heading) - t * np.sin(self.heading)
        trot = s_pos * np.sin(self.heading) + t * np.cos(self.heading)

        # Derivate to get heading change
        # 计算当前点在st坐标中的航向角，原方程求导得曲线切线
        dCoeffs = coeffs[1:] * np.array(np.arange(1, len(coeffs)))
        tangent = np.polynomial.polynomial.polyval(s_pos, dCoeffs)

        # 考虑参考线起点的坐标及航向角，反推对应点在惯性坐标系xy中的完整信息
        return (self.start_position + np.array([srot, trot]), self.heading + tangent)


class ParamPoly3(Geometry):
    """This record describes a parametric cubic curve as part
    of the road’s reference line in a local u/v co-ordinate system.

    This record describes an arc as part of the road’s reference line.


    (Section 5.3.4.1.5 of OpenDRIVE 1.4)
    """

    def __init__(
        self, start_position, heading, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange
    ):
        super().__init__(start_position=start_position, heading=heading, length=length)

        self._aU = aU
        self._bU = bU
        self._cU = cU
        self._dU = dU
        self._aV = aV
        self._bV = bV
        self._cV = cV
        self._dV = dV

        if pRange is None:
            self._pRange = 1.0
        else:
            self._pRange = pRange

    def calc_position(self, s_pos):
        """

        Args:
          s_pos:

        Returns:

        """

        # Position
        pos = (s_pos / self.length) * self._pRange  # 计算对应插值参数p

        # 曲线参数方程的参数序列
        coeffsU = [self._aU, self._bU, self._cU, self._dU]
        coeffsV = [self._aV, self._bV, self._cV, self._dV]

        # 基于参数方程计算xy坐标
        x = np.polynomial.polynomial.polyval(pos, coeffsU)
        y = np.polynomial.polynomial.polyval(pos, coeffsV)

        # 参考线st坐标系转惯性xy坐标系
        xrot = x * np.cos(self.heading) - y * np.sin(self.heading)
        yrot = x * np.sin(self.heading) + y * np.cos(self.heading)

        # Tangent is defined by derivation
        # 曲线参数微分，用来得对应切线斜率方程
        dCoeffsU = coeffsU[1:] * np.array(np.arange(1, len(coeffsU)))
        dCoeffsV = coeffsV[1:] * np.array(np.arange(1, len(coeffsV)))

        dx = np.polynomial.polynomial.polyval(pos, dCoeffsU)
        dy = np.polynomial.polynomial.polyval(pos, dCoeffsV)

        tangent = np.arctan2(dy, dx)  # 反正切得最终在st坐标系中得斜率

        return (self.start_position + np.array([xrot, yrot]), self.heading + tangent)

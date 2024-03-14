# -*- coding: utf-8 -*-

from .roadPlanView import PlanView
from .roadLink import Link
from .roadLanes import Lanes
from .roadElevationProfile import (
    ElevationProfile,
)
from .roadLateralProfile import LateralProfile
from .junction import Junction


class Road:
    """ """

    def __init__(self):
        self._id = None
        self._name = None
        self._junction = None  # 交叉口的ID，此道路作为链接道路时属于该交叉口，否则为-1
        self._length = None

        self._header = None  # TODO
        self._link = Link()  # 道路连接Road Linkage，道路间相互连接关系，以Link类对象储存
        self._types = []
        self._planView = PlanView()  # 道路参考线Reference Line
        self._elevationProfile = ElevationProfile()  # 道路高程信息(Section 8.4.1 in OpenD 1.6)
        self._lateralProfile = LateralProfile()  # 道路超高程信息（Section 8.4.2 in OpenD 1.6）
        self._lanes = Lanes()  # 车道Lanes

    # 内置方法，用于判断该类的两个实例化对象是否属性值是否相等
    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    '''@property修饰器，使得开发者可以通过classname.property的方式操作类属性，而不需在方法名后添加小括号()
    使用@property修饰了id()方法，使其变成了id属性的getter方法，从而id属性将是一个只读属性
    '''
    @property
    def id(self):
        """ """
        return self._id

    '''@id.setter装饰器，实现修改该属性值的功能，从而id属性同时具有getter与setter方法，该属性变成可读写属性'''
    @id.setter
    def id(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._id = int(value)

    @property
    def name(self):
        """ """
        return self._name

    @name.setter
    def name(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._name = str(value)

    @property
    def junction(self):
        """ """
        return self._junction

    @junction.setter
    def junction(self, value):
        """

        Args:
          value:

        Returns:

        """
        if not isinstance(value, Junction) and value is not None:
            raise TypeError("Property must be a Junction or NoneType")

        if value == -1:
            value = None

        self._junction = value

    @property
    def link(self):
        """ """
        return self._link

    @property
    def types(self):
        """ """
        return self._types

    @property
    def planView(self):
        """ """
        return self._planView

    @property
    def elevationProfile(self):
        """ """
        return self._elevationProfile

    @property
    def lateralProfile(self):
        """ """
        return self._lateralProfile

    @property
    def lanes(self):
        """ """
        return self._lanes

# -*- coding: utf-8 -*-

class OpenDrive:
    """定义OpenDrive类
    【类变量定义规则】
    单下划线开头变量_xxx表明其是一个受保护(protected)变量，原则上不允许直接对其进行访问，但外部类依旧可以访问self._xxx
    双下划线开头变量__xxx表明其是一个私有(private)变量，其只允许类内访问，但可以通过名称转写实现外部访问self.__classname_xxx
    """

    def __init__(self):
        self.header = None
        self._roads = []
        self._controllers = []
        self._junctions = []  # 存入juction类对象
        self._junctionGroups = []
        self._stations = []

    # @property
    # def header(self):
    #     return self._header

    @property
    def roads(self):
        """ """
        return self._roads

    def getRoad(self, id_):
        """

        Args:
          id_: 后置下划线，一种编程风格，用来与python内置关键字做区分

        Returns:

        """
        for road in self._roads:
            if road.id == id_:
                return road

        return None

    @property
    def controllers(self):
        """ """
        return self._controllers

    @property
    def junctions(self):
        """ """
        return self._junctions

    def getJunction(self, junctionId):
        """

        Args:
          junctionId: 

        Returns:

        """
        for junction in self._junctions:
            if junction.id == junctionId:
                return junction
        return None

    @property
    def junctionGroups(self):
        """ """
        return self._junctionGroups

    @property
    def stations(self):
        """ """
        return self._stations


class Header:
    """ """

    def __init__(
        self,
        rev_major=None,
        rev_minor=None,
        name: str = None,
        version=None,
        date=None,
        north=None,
        south=None,
        east=None,
        west=None,
        vendor=None,
    ):
        self.revMajor = rev_major
        self.revMinor = rev_minor
        self.name = name
        self.version = version
        self.date = date
        self.north = north
        self.south = south
        self.east = east
        self.west = west
        self.vendor = vendor

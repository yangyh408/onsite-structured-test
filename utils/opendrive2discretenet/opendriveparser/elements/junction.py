# -*- coding: utf-8 -*-

 


class Junction:
    """ """

    # TODO priority
    # TODO controller

    def __init__(self):
        self._id = None
        self._name = None
        self._connections = []  # 交叉口的连接关系inconming_road+connecting_road，以Connection类的实例对象作为列表元素

    @property
    def id(self):
        """ """
        return self._id

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
    def connections(self):
        """ """
        return self._connections

    def addConnection(self, connection):
        """

        Args:
          connection:

        Returns:

        """
        if not isinstance(connection, Connection):
            raise TypeError("Has to be of instance Connection")

        self._connections.append(connection)


class Connection:
    """ xodr样例
    <connection id="0"
            incomingRoad="1"
            connectingRoad="2"
            contactPoint="start">
        <laneLink from="-2" to="-1"/>
    </connection>
    """

    def __init__(self):
        self._id = None
        self._incomingRoad = None
        self._connectingRoad = None
        self._contactPoint = None  # @contactPoint属性用于对链接道路的方向做出说明，start表示沿laneLink方向连接延伸
        self._laneLinks = []  # 当前连接段对应的车道ID连接关系，以LaneLink类的实例对象作为列表元素

    @property
    def id(self):
        """ """
        return self._id

    @id.setter
    def id(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._id = int(value)

    @property
    def incomingRoad(self):
        """ """
        return self._incomingRoad

    @incomingRoad.setter
    def incomingRoad(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._incomingRoad = int(value)

    @property
    def connectingRoad(self):
        """ """
        return self._connectingRoad

    @connectingRoad.setter
    def connectingRoad(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._connectingRoad = int(value)

    @property
    def contactPoint(self):
        """ """
        return self._contactPoint

    @contactPoint.setter
    def contactPoint(self, value):
        """

        Args:
          value:

        Returns:

        """
        if value not in ["start", "end"]:
            raise AttributeError("Contact point can only be start or end.")

        self._contactPoint = value

    @property
    def laneLinks(self):
        """ """
        return self._laneLinks

    def addLaneLink(self, laneLink):
        """

        Args:
          laneLink:

        Returns:

        """
        if not isinstance(laneLink, LaneLink):
            raise TypeError("Has to be of instance LaneLink")

        self._laneLinks.append(laneLink)


class LaneLink:
    """ """

    def __init__(self):
        self._from = None
        self._to = None

    def __str__(self):
        return str(self._from) + " > " + str(self._to)

    @property
    def fromId(self):
        """ """
        return self._from

    @fromId.setter
    def fromId(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._from = int(value)

    @property
    def toId(self):
        """ """
        return self._to

    @toId.setter
    def toId(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._to = int(value)

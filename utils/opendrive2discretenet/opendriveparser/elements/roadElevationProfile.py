# -*- coding: utf-8 -*-

from .road_record import RoadRecord


class ElevationProfile:
    """The elevation profile record contains a series of elevation records
    which define the characteristics of
    the road's elevation along the reference line.

    (Section 5.3.5 of OpenDRIVE 1.4)
    """

    def __init__(self):
        self.elevations = []


class ElevationRecord(RoadRecord):
    """The elevation record defines an elevation entry at a given reference line position.

    (Section 5.3.5.1 of OpenDRIVE 1.4)
    """

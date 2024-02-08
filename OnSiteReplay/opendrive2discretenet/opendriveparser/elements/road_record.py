# -*- coding: utf-8 -*-

 

from abc import ABC


class RoadRecord(ABC):
    """Abstract base class to model Records (e.g. ElevationRecord) of the OpenDRIVE
    specification.

    These Records all have attributes start_pos, a, b, c, d.
    The attribute attr which is defined the RoadRecord at a given reference line position
    is calculated with the following equation:
    attr = a + b*ds + c*ds² + d*ds³
    where ds being the distance along the reference line between the start of the entry
    and the actual position.

    ds starts at zero for each RoadRecord.

    The absolute position of an elevation value is calculated by
      s = start_pos + ds



    Attributes:
      start_pos: Position in curve parameter ds where the RoadRecord starts.
      polynomial_coefficients: List of values [a, b, c, d, ...] which can be evaluated with an
        polynomial function.
    """

    def __init__(self, *polynomial_coefficients: float, start_pos: float = None):
        # *参数收集所有未匹配的位置参数组成一个tuple对象，故*polynomial_coefficients可以接受多个float参数
        self.start_pos = start_pos
        self.polynomial_coefficients = []
        for coeff in polynomial_coefficients:
            self.polynomial_coefficients.append(coeff)

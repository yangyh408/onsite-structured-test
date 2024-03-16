# -*- coding: utf-8 -*-

import numpy

 


def encode_road_section_lane_width_id(roadId, sectionId, laneId, widthId):
    """

    Args:
      roadId:
      sectionId:
      laneId:
      widthId:

    Returns:

    """
    return ".".join([str(roadId), str(sectionId), str(laneId), str(widthId)])


def decode_road_section_lane_width_id(encodedString: str):
    """

    Args:
      encodedString:

    Returns:

    """

    parts = encodedString.split(".")

    if len(parts) != 4:
        raise Exception()

    return (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]))


def allCloseToZero(array):
    """Tests if all elements of array are close to zero.

    Args:
      array:

    Returns:

    """

    return numpy.allclose(array, numpy.zeros(numpy.shape(array)))

from lxml import etree

from .utils import config


def opendrive2tess(netiface, parms):
    # parms:
    # - file_path
    # - step_length
    # - lane_types
    
    file_path = parms["file_path"]
    step_length = parms["step_length"]
    lane_types = parms["lane_types"]
    
    config.sceneScale = netiface.sceneScale()
    
    from .utils.network_utils import Network
    from .opendrive2lanelet.opendriveparser.parser import parse_opendrive
    
    with open(file_path, "r", encoding='utf-8') as file_in:
        root_node = etree.parse(file_in).getroot()
        opendrive = parse_opendrive(root_node)
    
    network = Network(opendrive)
    network.convert_network(step_length)
    error_junction = network.create_network(lane_types, netiface)

    return error_junction


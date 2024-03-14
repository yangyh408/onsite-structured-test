from .MySimulatorSerial import MySimulatorSerial
from .MySimulatorFragment import MySimulatorFragment
from .MySimulatorCreateTess import MySimulatorCreateTess
from .MySimulatorCreateWaypoints import MySimulatorCreateWaypoints

def select_simulator(mode: str, config: dict, planner: object, scene_info):
    if mode == 'SERIAL':
        return MySimulatorSerial(config, planner, scene_info)
    elif mode == 'FRAGMENT':
        return MySimulatorFragment(config, planner, scene_info)
    elif mode == 'CREATE_TESS':
        return MySimulatorCreateTess(config)
    elif mode == 'CREATE_WAYPOINTS':
        return MySimulatorCreateWaypoints(config)
    else:
        return None
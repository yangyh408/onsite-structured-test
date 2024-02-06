class Observation():
    """
    vehicle_info={
        "ego":{
            "x":,
            "y":,
            "v":,
            "yaw":,
            "width":,
            "length":,
        },
        "0":{...},
        ...
    }
    light_info = {}
    test_setting = {
            "t":,
            "dt":,
            "max_t",
            "goal":,
            "end":
        }
    """

    def __init__(self):
        self.init()
    
    def init(self):
        self.vehicle_info = {
            "ego": {
                "x": -1,
                "y": -1,
                "v": -1,
                "a": -1,
                "yaw": -1,
                "width": -1,
                "length": -1
            },
        }
        self.road_info = {}
        self.light_info = {}
        self.test_setting = {
            "scenario_name": "",
            "scenario_type": "",
            "t": 0.00,
            "dt": -1,
            "max_t": -1,
            "goal": {
                "x": [-1, -1],
                "y": [-1, -1]
            },
            "end": -1
        }
        
    def format(self):
        return {
            "vehicle_info": self.vehicle_info,
            "light_info": self.light_info,
            "test_setting": self.test_setting,
        }
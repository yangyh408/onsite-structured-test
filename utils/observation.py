class Observation():
    def __init__(self):
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
        self.bicycle_info={}
        self.pedestrian_info={}
        self.light_info = {}
        self.test_info = {
            "t": 0.00, 
            "dt": 0.00,
            "end": -1,
        }
        
    def format(self):
        return {
            "vehicle_info": self.vehicle_info,
            "bicycle_info": self.bicycle_info,
            "pedestrian_info": self.pedestrian_info,
            "light_info": self.light_info,
            "test_info": self.test_info,
        }
    
    def object_info(self):
        return {
            **self.vehicle_info,
            **self.bicycle_info,
            **self.pedestrian_info
        }

if __name__ == "__main__":
    pass

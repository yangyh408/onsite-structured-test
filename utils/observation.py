from typing import Dict, Union
import math
class ObjectStatus():
    def __init__(self, **kwargs):
        self.x = 0
        self.y = 0
        self.v = 0
        self.a = 0
        self.yaw = 0
        self.width = 0
        self.length = 0
        self.update(**kwargs)
    
    def __str__(self):
        return str(vars(self))
    
    def update(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                if key == 'yaw':
                    value = value % (2 * math.pi)
                setattr(self, key, round(value, 3))

class EgoStatus(ObjectStatus):
    def __init__(self, **kwargs):
        self.a = 0
        self.rot = 0
        super().__init__()
        self.update(**kwargs)

class Observation():
    def __init__(self):
        self.ego_info = EgoStatus()
        self.object_info: Dict[str, Dict[str, ObjectStatus]] = {
            'vehicle': {},
            'bicycle': {},
            'pedestrian': {},
        }
        self.light_info = ""
        self.test_info = {
            "t": 0.00, 
            "dt": 0.00,
            "end": -1,
        }

    def __str__(self):
        result = ""
        result += f"- ego_info: {str(self.ego_info)}\n"
        result += "- object_info: \n"
        for category, objects in self.object_info.items():
            if objects:
                result += f"  + \"{category}\":\n"
                for obj_name, obj_status in objects.items():
                    result += f"      \"{obj_name}\" - {str(obj_status)}\n"
        result += f"- light_info: {self.light_info}\n"
        result += f"- test_info: {self.test_info}\n"
        return result
    
    def update_ego_info(self, **kwargs):
        self.ego_info.update(**kwargs)

    def update_light_info(self, light_info: str=""):
        self.light_info = light_info

    def update_test_info(self, **kwargs):
        self.test_info.update(**kwargs)

    def erase_object_info(self):
        self.object_info = {
            'vehicle': {},
            'bicycle': {},
            'pedestrian': {},
        }

    def update_object_info(self, category: str, obj_name: str, **kwargs):
        if category in self.object_info.keys():
            obj_name = str(obj_name)
            if obj_name not in self.object_info[category].keys():
                self.object_info[category][obj_name] = ObjectStatus()
            self.object_info[category][obj_name].update(**kwargs)

if __name__ == "__main__":
    observation = Observation()
    print(observation)

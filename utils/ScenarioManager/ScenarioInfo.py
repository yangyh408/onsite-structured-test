class ScenarioInfo():
    def __init__(self, **kwargs) -> None:
        self.num = -1
        self.name = ""
        self.type = ""
        self.source_file = {"tess": "", "xodr": "", "xosc": "", "json": ""}
        self.output_path = ""
        self.task_info = {"startPos": [], "targetPos": [], "waypoints": [], "dt": float('nan')}
        self.additional_info = {}
        self.update(**kwargs)

    def __str__(self):
        output = '*' * 100 + '\n'
        for key, value in vars(self).items():
            output += f"   - {key:20s}: {value}\n"
        output += '*' * 100
        return output
    
    def update(self, **kwargs):
        for key, value in kwargs.items():
            if not hasattr(self, key):
                raise AttributeError(f"ScenarioInfo has no attribute {key}")
            else:
                setattr(self, key, value)

    def format(self):
        scene_dict = vars(self).copy()
        scene_dict.pop('additional_info')
        return scene_dict


if __name__ == "__main__":
    info = ScenarioInfo()
    print(info)
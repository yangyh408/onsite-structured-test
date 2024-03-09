import numpy as np

class ReplayInfo():

    def __init__(self):
        self.vehicle_traj = {}
        self.bicycle_traj = {}
        self.pedestrian_traj = {}
        self.ego_info = {
            "length": 4.924,
            "width": 1.872,
            "x": 0,
            "y": 0,
            "v": 0,
            "a": 0,
            "yaw": 0
        }
        self.light_info = {}
        self.road_info = {}
        self.test_setting = {
            "t": 0,
            "dt": 0.01,
            "max_t": 10,
            "map_type":None,
            "map_range": {
                "x": [0, 0],
                "y": [0, 0]
            }
        }

    def __str__(self):
        output = ""
        output += "*"*100
        output += f"\n<vehicle_traj>\n{self.vehicle_traj}\n" + "-"*50
        output += f"\n<bicycle_traj>\n{self.bicycle_traj}\n" + "-"*50
        output += f"\n<pedestrian_traj>\n{self.pedestrian_traj}\n" + "-"*50
        output += f"\n<ego_info>\n{self.ego_info}\n" + "-"*50
        output += f"\n<light_info>\n{self.light_info}\n" + "-"*50
        output += f"\n<road_info>\n{self.road_info}\n" + "-"*50
        output += f"\n<test_setting>\n{self.test_setting}\n"
        output += "*"*100
        return output
    
    def add_vehicle(self, id, t, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """
        该函数实现向vehicle_trajectiry中添加背景车轨迹信息的功能
        """
        if id == "ego":
            self._add_vehicle_ego(x, y, v, a, yaw, length, width)
        else:
            if id not in self.vehicle_traj.keys():
                self.vehicle_traj[id] = {}
                self.vehicle_traj[id]['shape'] = {}
            if t not in self.vehicle_traj[id].keys():
                self.vehicle_traj[id][t] = {}
            for key, value in zip(['x', 'y', 'v', 'a', 'yaw'], [x, y, v, a, yaw]):
                if value is not None:
                    self.vehicle_traj[id][t][key] = value
            for key, value in zip(['length', 'width'], [length, width]):
                if value is not None:
                    self.vehicle_traj[id]['shape'][key] = value

    def add_bicycle(self, id, t, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """
        该函数实现向bicycle_trajectiry中添加背景车轨迹信息的功能
        """
        if id not in self.bicycle_traj.keys():
            self.bicycle_traj[id] = {}
            self.bicycle_traj[id]['shape'] = {}
        if t not in self.bicycle_traj[id].keys():
            self.bicycle_traj[id][t] = {}
        for key, value in zip(['x', 'y', 'v', 'a', 'yaw'], [x, y, v, a, yaw]):
            if value is not None:
                self.bicycle_traj[id][t][key] = value
        for key, value in zip(['length', 'width'], [length, width]):
            if value is not None:
                self.bicycle_traj[id]['shape'][key] = value

    def add_pedestrian(self, id, t, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """
        该函数实现向bicycle_trajectiry中添加背景车轨迹信息的功能
        """
        if id not in self.pedestrian_traj.keys():
            self.pedestrian_traj[id] = {}
            self.pedestrian_traj[id]['shape'] = {}
        if t not in self.pedestrian_traj[id].keys():
            self.pedestrian_traj[id][t] = {}
        for key, value in zip(['x', 'y', 'v', 'a', 'yaw'], [x, y, v, a, yaw]):
            if value is not None:
                self.pedestrian_traj[id][t][key] = value
        for key, value in zip(['length', 'width'], [length, width]):
            if value is not None:
                self.pedestrian_traj[id]['shape'][key] = value

    def add_settings(self, dt=None, max_t=None):
        """
        该函数实现向test_setting中添加测试环境相关信息
        """
        for key, value in zip(['dt', 'max_t'],
                              [dt, max_t]):
            if value is not None:
                self.test_setting[key] = value

    def _add_vehicle_ego(self, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """
        该函数实现向ego_info中增加主车信息的功能
        注意：ego_info中只含有主车当前帧的信息
        """
        for key, value in zip(['x', 'y', 'v', 'a', 'yaw', 'length', 'width'], [x, y, v, a, yaw, length, width]):
            if value is not None:
                self.ego_info[key] = value

    def _get_dt_maxt(self):
        """
        该函数实现得到最大仿真时长阈值以及采样率的功能
        """
        max_t = 0
        for i in self.vehicle_traj.keys():
            t_i = list(self.vehicle_traj[i].keys())
            max_t_i = float(t_i[-1])
            if max_t_i > max_t:
                max_t = max_t_i

        dt = np.around(float(t_i[-1]) - float(t_i[-2]), 3)
        self.add_settings(dt=dt, max_t=max_t)
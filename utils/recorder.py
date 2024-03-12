import os
import pandas as pd
from functools import reduce

from utils.observation import Observation


class DataRecord:
    def __init__(self):
        self.control_data  = pd.DataFrame(columns=['acc', 'rot'])
        self.ego_data  = pd.DataFrame(columns=['x', 'y', 'v', 'a', 'yaw', 'rot', 'width', 'length'])
        self.object_data = {}
        self.object_column = ['x', 'y', 'v', 'a', 'yaw', 'width', 'length']
        self.end_data = pd.DataFrame(columns=['end'])

    def add_data(self, action: list, observation: Observation):
        """将输入的观察值进行存储
        """
        # 首先将已经建立了对应DataFrame的车辆名提取出来
        stored_vehicles = self.object_data.keys()
        # 提取observation对应的时刻
        t = observation.test_info['t']
        
        # 记录控制量
        self.control_data.loc[t] = [round(action[0], 3), round(action[1], 3)]
        # 记录ego车辆信息
        self.ego_data.loc[t] = observation.ego_info.__dict__
        # 记录结束状态
        self.end_data.loc[t] = observation.test_info['end']
        
        # 遍历observation中的所有车辆
        for obj_type in observation.object_info:
            for vehicle_name, vehicle_info in observation.object_info[obj_type].items():
                # 如果vehi_name对应的车还没有建立DataFrame,则先建立
                if vehicle_name not in stored_vehicles:
                    self._add_vehicle_frame(vehicle_name)
                # 为t时刻的数据建立当前时刻的DataFrame
                self.extend_vehicle_info(t, vehicle_name, vehicle_info.__dict__)
    
    def extend_vehicle_info(self, t, vehicle_name: str, veh_info: dict) -> None:
        """为某一交通参与者的DataFrame增加一行
        """
        sub_frame = pd.DataFrame(
            veh_info,
            columns=self.object_column,
            index=[t],
        )
        sub_frame.columns = list(self.object_data[vehicle_name].columns)
        self.object_data[vehicle_name] = pd.concat([self.object_data[vehicle_name], sub_frame])

    def merge_frame(self) -> pd.DataFrame:
        """将存储的所有交通参与者的DataFrame，按照时间进行合并，返回完整的DataFrame

        """
        self.ego_data.columns = [f"{i}_ego" for i in self.ego_data.columns]
        # 取出每辆车的DataFrame，组成列表
        object_dataframe_group = [self.object_data[vehi_name] for vehi_name in list(self.object_data.keys())]
    
        return reduce(lambda x, y: pd.merge(x, y, how="outer", left_index=True, right_index=True), [self.control_data, self.ego_data, *object_dataframe_group, self.end_data])

    def _add_vehicle_frame(self, vehicle_name: str):
        """为某一交通参与者创造对应的小DataFrame
        """
        self.object_data[vehicle_name] = pd.DataFrame(
            None,
            columns=[i + "_" + vehicle_name for i in self.object_column]
        )


# 记录模块
class Recorder:
    def __init__(self):
        self.end_status = -1
        self.data = DataRecord()

    def record(self, action: list, observation: Observation):
        if self.end_status == -1:
            self.data.add_data(action, observation)
            self.end_status = observation.test_info['end']
    
    def output(self, output_path):
        if not os.path.exists(os.path.dirname(output_path)):
            os.makedirs(os.path.dirname(output_path))
        data_output = self.data.merge_frame()
        if not data_output.empty:
            data_output.to_csv(output_path)

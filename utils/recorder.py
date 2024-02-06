import os
import pandas as pd
from functools import reduce


class DataRecord:
    def __init__(self):
        self.data = {}
        self.vehicle_column = ['x', 'y', 'v', 'a', 'yaw', 'width', 'length']

    def add_data(self, observation):
        """将输入的观察值进行存储

        """
        # 首先将已经建立了对应DataFrame的车辆名提取出来
        stored_vehicles = self.data.keys()

        # 提取observation对应的时刻
        t = observation.test_setting['t']

        # 遍历observation中的所有车辆
        for vehicle_name, values in observation.vehicle_info.items():
            # 如果vehi_name对应的车还没有建立DataFrame,则先建立
            if vehicle_name not in stored_vehicles:
                self._add_vehicle_frame(vehicle_name)

            # 为t时刻的数据建立当前时刻的DataFrame
            sub_frame = pd.DataFrame(
                values,
                columns=self.vehicle_column,
                index=[t]
            )
            # 修改列名，便于合并
            sub_frame.columns = list(self.data[vehicle_name].columns)
            # 将当前时刻的DataFrame加入车辆的DataFrame中
            self.data[vehicle_name] = pd.concat([self.data[vehicle_name], sub_frame])

    def merge_frame(self) -> pd.DataFrame:
        """将存储的所有交通参与者的DataFrame，按照时间进行合并，返回完整的DataFrame

        """
        # 取出每辆车的DataFrame，组成列表
        vehicle_dataframe_group = [self.data[vehi_name]
                                   for vehi_name in list(self.data.keys())]
        # 返回合并后的DataFrame
        return reduce(lambda x, y: pd.merge(x, y, how="outer", left_index=True, right_index=True),
                      vehicle_dataframe_group)

    def _add_vehicle_frame(self, vehicle_name: str):
        """为某一交通参与者创造对应的小DataFrame

        """
        self.data[vehicle_name] = pd.DataFrame(
            None,
            columns=[i + "_" + str(vehicle_name) for i in self.vehicle_column]
        )


# 记录模块
class Recorder:
    def __init__(self):
        self.end_status = None
        self.data = None
        self.init()

    def init(self):
        self.end_status = -1
        self.data = DataRecord()

    def record(self, observation):
        if self.end_status == -1:
            self.data.add_data(observation)
            self.end_status = observation.test_setting['end']

    def output(self, output_path):
        if not os.path.exists(os.path.dirname(output_path)):
            os.makedirs(os.path.dirname(output_path))
        data_output = self.data.merge_frame()
        # 增加结束状态一列
        data_output.loc[:, 'end'] = -1
        data_output.iloc[-1, -1] = self.end_status
        data_output.to_csv(output_path)

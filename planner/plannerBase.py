from abc import ABC, abstractmethod
from typing import List

from utils.observation import Observation

class PlannerBase(ABC):
    def __init__(self) -> None:
        """ 构造函数，用于初始化自定义规控器模块的相关属性设置
        """
        pass
    
    @abstractmethod
    def init(self, scenario_info: dict) -> None:
        """ 初始化函数，通过scenario_info参数传入当前测试任务信息，可用于进行地图解析等工作
        
        Arguments:
            scenario_info {dict}
                "num": {int} 测试场景编号
                "name": {str} 测试场景名称
                "type": {str} 测试模式，取值为"REPLAY"|"FRAGMENT"|"SERIAL"，分别对应回放测试、片段式双向交互测试和无限里程双向交互测试
                "output_path": {str} 测试结果输出路径
                "source_file": {dict} 场景相关源文件路径，包含 xodr, xosc, json, tess四种后缀文件
                "task_info": {dict} 测试场景任务相关参数，包含主车初始位置  startPos 、目标行驶区域targetPos、途径点waypoints序列和仿真步长dt
        """
        pass
    
    @abstractmethod
    def act(self, observation: Observation) -> List[float]:
        """ 响应函数，读入当前时刻的仿真环境状态信息，进行相关规划控制运算并返回主车的控制量

        Arguments:
            observation {Observation} -- 上一帧背景交通流及仿真状态的记录信息，详见<README: # utils.observation>
                ego_info: {EgoStatus} 仿真中主车（被测物）的车辆状态
                object_info: {Dict[str, Dict[str, ObjectStatus]]} 用于描述仿真环境中的背景要素状态
                light_info: {str} 表示仿真环境当前与主车通行相关的信号灯灯色
                test_info: {dict} 描述当前仿真环境的状态信息

        Returns:
            [acc, rotate] {list}
                acc {float} -- 主车下一时刻的纵向加速度(m/s^2)    
                rotate {float} -- 主车下一时刻的前轮转角(rad)    
        """
        pass
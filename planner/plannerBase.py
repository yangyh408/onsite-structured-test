from utils.observation import Observation

class PlannerBase:
    def __init__(self) -> None:
        """ 构造函数，在程序初始时实例化Planner模块
        """
        pass

    def init(self, scenario_info: dict) -> None:
        """ 初始化函数，在每个场景加载时调用，通过scenario_dic变量向规控器传入场景的起始坐标、终点面域和途径点
        
        Arguments:
            scenario_info {dict} -- 包含了该场景使用的静态路网、起始坐标、终点面域和途径点信息
                'scenarioName': 场景名称
                'tess_file_path': 场景对应.tess文件路径
                'xodr_file_path': 场景对应.xodr文件路径
                'startPos': 主车起始坐标
                'targetPos': 主车目标终点面域
                'waypoints': 起终点间的参考轨迹
        """
        pass

    def act(self, observation: Observation) -> [float, float]:
        """ 响应函数，读入当前时刻的背景交通流状态信息，返回主车的控制量

        Arguments:
            observation {Observation} -- 上一帧背景交通流及仿真状态的记录信息，具体信息参考onsite网站相关介绍（https://onsite.run/#/trackOutput "回放测试"->"用户读取observation数据说明"）

        Returns:
            [acc, rotate] {list} -- 主车下一时刻的控制量
                acc {float} -- 主车下一时刻的加速度(m/s^2)    
                rotate {float} -- 主车下一时刻的方向盘转角(rad)    
        """
        pass
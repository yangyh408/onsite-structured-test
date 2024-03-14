# API Reference



## `utils.observation`



### `ObjectStatus` objects

*class utils.observation.ObjectStatus(\*, x=0, y=0, v=0, a=0, yaw=0, width=0, length=0)*

​	`EgoStatus`的基类，用于描述仿真中物体的状态

+ **Attributes:**

  + **x** : *float*

    物体在与底图匹配的局部坐标系下的横坐标，单位：`m`

  + **y** : *float*

    物体在与底图匹配的局部坐标系下的纵坐标，单位：`m`

  + **v** : *float*

    物体的移动速率，单位：`m/s`

    *取值范围大于等于0*

  + **a** : *float*

    物体移动纵向加速度，单位：`m/s^2`

  + **yaw** : *float*

    物体移动航向角，单位：`rad`

    *以x轴正方向为0，顺时针方向增加，取值范围为0~2π*

  + **width** : *float*

    物体的宽度，单位：`m`

  + **length** : *float*

    物体的长度，单位：`m`

+ **Methods:**

  + **`update`(self, \*\*kwargs)**

    用于更新类内的各种参数取值



###  `EgoStatus` objects

*class utils.observation.EgoStatus(\*, x=0, y=0, v=0, a=0, yaw=0, width=0, length=0, rot=0)*

​	以`ObjectStatus`为基类，用于描述仿真中主车的状态

+ **Attributes:**

  + **rot** : *float*

    主车的前轮转角，单位：`rad`

    *以主车行驶方向为0，顺时针方向为正*

+ **Methods:**

  + **`update`(self, \*\*kwargs)**

    用于更新类内的各种参数取值



### `Observation` objects

*class utils.observation.Observation()*

​	用于描述仿真中背景要素的当前状态信息

+ **Sources:**

  ```python
  class Observation():
      def __init__(self):
          self.ego_info: EgoStatus = EgoStatus()
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
      ...
  ```

+ **Attributes:**

  + **ego_info** : [*EgoStatus*](#egostatus-objects)

    仿真中主车（被测物）的车辆状态

  + **object_info** : *Dict[str, Dict[str, [ObjectStatus](#objectstatus-objects)]]*

    用于描述仿真环境中的背景要素状态

    字典结构如下：

    ```python
    {
        "vehicle": {
            "veh_name": ObjectStatus(),
            ...
        }
        "bicycle": {
            "bic_name": ObjectStatus(),
            ...
        }
        "pedetrian": {
            "ped_name": ObjectStatus(),
            ...
        }
    }
    ```

    +   一级键：表示*背景要素类型*，`"vehicle"`为背景车辆、 `"bicycle"`为自行车、`"pedestrain"`为行人
    +   二级键：表示*背景要素的名称*，类型为`str`
    +   二级值：表示*对应背景要素的状态信息*，类型为`ObjectStatus`

  + **light_info** : *str*

    表示仿真环境当前与主车通行相关的信号灯灯色

    *`"green"`为绿灯可通行、 `"yellow"`为黄灯、`"red"`为红灯禁止通行*

    >   当前回传的信号灯信息仅为主车即将经过的下一信控路口按任务设置行驶方向的信号灯灯色

  + **test_info** : *dict*

    描述当前仿真环境的状态信息

    | key   | value                                                        |
    | ----- | ------------------------------------------------------------ |
    | "t"   | 类型为`float`，单位为`s`，表示当前仿真的时间戳               |
    | "dt"  | 类型为`float`，单位为`s`，表示当前仿真步长                   |
    | "end" | 类型为`int`，表示当前仿真运行状态。`-1`表示仿真在运行中，`1`表示主车完成测试任务（抵达目标区域），`2`表示仿真超时，`3`表示主车与背景要素发生碰撞，`4`表示主车驶出可行驶范围 |

+ **Methods:**

  + **`update_ego_info`(self, \*\*kwargs)**

    用于更新ego_info属性的值

    *Parameters：*

    +   与EgoStatus的属性一致

  + **`erase_object_info`(self)**

    用于初始化object_info属性的值

    初始化后值为`{"vehicle": {}, "bicycle": {}, "pedestrian": {}}`

  + **`update_object_info`(self, category, obj_name, \*\*kwargs)**

    用于更新object_info属性的值

    *Parameters：*

    + `category` : *str*

      对应object_info属性的一级键，表示待更新的背景要素类型

    + `obj_name` : *str*

      对应object_info属性的二级键，表示待更新的背景要素名称

    + 其余参数与ObjectStatus类内属性一致

  + **`update_light_info`(self, light_info)**

    用于更新light_info属性的值

    *Parameters：*

    + `light_info` : *str*

      待更新的红绿灯灯色

  + **`update_test_info`(self, \*\*kwargs)**

    用于更新test_info属性的值





## `utils.ScenarioManager`



### `ScenarioInfo` objects

*class utils.ScenarioManager.ScenarioInfo()*

​	用于描述测试任务相关信息，服务于以`ScenarioManagerBase`为基类的各类

+ **Attributes:**

  + **num** : *int*

    测试场景编号

  + **name** : *str*

    测试场景名称

  + **type** : *str*

    测试场景类型，取值为`"REPLAY" | "FRAGMENT" | "SERIAL"`

    *不同场景类型对应不同的测试能力，上述三个取值分别对应回放测试、片段式双向交互测试、连续式双向交互测试*

  + **source_file** : *Dict[str, str]*

    测试任务相关的源文件路径

    | key      | value                            |
    | -------- | -------------------------------- |
    | `"xodr"` | OpenDrive静态地图文件路径        |
    | `"xosc"` | OpenScenario动态场景文件路径     |
    | `"json"` | 红绿灯信息json文件路径           |
    | `"tess"` | TessNG仿真器运行所需tess文件路径 |

    *字典中对应字段为空字符串时表示无相应源文件*

  + **output_path** : *str*

    测试任务结果输出文件的路径

  + **task_info** : *dict*

    测试场景任务相关参数

    键值对说明如下：

    + `"startPos"` : *List[float]*

      表明主车的初始位置，列表包含两个浮点值，分别对应加载位置横、纵坐标

    + `"targetPos"` : *List[List[float]]*

      表明测试任务目标区域，列表包含两个点对，用于定位矩形目标区域位置

    + `"waypoints"` : *Dict[str, List[float]]*

      表明测试任务的参考轨迹点序列，形式如下：

      ```python
      {
          "1": [-352.40, 236.60], 
          "2": [-355.12, 234.67], 
          "3": [-358.38, 232.08],
          ...
      }
      ```

      *目前仅用于`SERIAL`连续式双向交互测试*

    + `"dt"` : *float*

      表明测试任务的步长，单位为`s`

  + **additional_info** : *dict*

    用于记录除上述信息的其他测试任务相关的字段

    >   *注：该属性在调用`format()`方法时不会被加入字典中返回*

+ **Methods:**

  + `update`(self, \*\*kwargs)

    用于更新类内的各种参数取值

  + `format`(self)

    将测试任务相关的信息以字典形式进行返回

    *Return type：* dict

    *Returns :*

    +   返回字典的键为：`num`,`name`, `type`, `source_file`,`output_path`,`task_info`
    +   返回字典的值为：与键对应类内属性的值



### `ScenarioManagerBase` objects

*class utils.ScenarioManager.ScenarioManagerBase(config)*

​	各测试方法对应测试场景管理工具的基类，如`ScenarioManagerForReplay`等

+ **Attributes:**

  + **skip_exist** : *bool, default: False*

    是否在运行时跳过在输出文件夹中已有输出文件的场景

  + **record** : *dict*

    用于记录加载失败的场景名称及其报错信息

  + **scenario_type** : *str*

    测试场景类型，取值为`"REPLAY" | "FRAGMENT" | "SERIAL"`

  + **tasks** : *List[str]*

    本轮测试的任务列表，列表中每一项为场景名称

  + **output_path** : *str*

    测试任务结果输出文件夹路径 

    > 输出文件默认保存在`./outputs/`文件夹

  + **cur_scene_num** : *int*

    当前测试的任务场景编号，即其在tasks属性中的位置

  + **cur_scene** : [*ScenarioInfo*](#scenarioinfo-objects)

    当前测试任务场景的相关信息

  + **tot_scene_num** : *int*

    加载到的测试任务总数

+ **Methods:**

  + **`next`(self)**

    用于加载下一个测试任务场景，执行后会更新cur_scene_num和cur_scene的值

    *Return type：* bool

    *Returns :*

    +   返回为`True`表示下一测试场景加载成功，为`False`表示所有测试任务已经加载完成

  + **`_struct_scene_info`(self)**

    用于更新类内cur_scene的属性，不同的测试类型有不同的更新方法，**需要在子类中复写**

    *Return type：* ScenarioInfo

    *Returns :*

    ​	返回 tasks[cur_scene_num] 对应的完整测试任务信息

  + **`_is_exist`(self, scenario_name)**

    检测该场景的输出文件是否已经在结果文件夹中存在

    *Parameters :*

    + `scenario_name` : *str*

      待检测的场景名称

    *Return type :* bool

    *Returns :*

    ​	返回为`True`表示该场景的输出文件已经在结果文件夹中存在，反之则否

  + **`_find_file_with_suffix`(self, dir, suffix)****

    寻找场景中特定后缀文件的路径

    *Parameters :*

    + `dir` : *str*

      待寻找的文件夹路径

    + `suffix` : *str*

      待寻找的文件后缀*（不包含）*

    *Return type :* str

    *Returns :*

    ​	返回待寻找文件夹中相应后缀文件路径，为空字符串时表示文件夹中无使用此后缀的文件




### `ScenarioManagerForReplay` objects

*class utils.ScenarioManager.ScenarioManagerForReplay(scenario_dir, config)*

​	`ScenarioManagerBase`的子类，用于无回放测试的测试任务管理

+ **Methods:**

  + **`_struct_scene_info`(self)**

    用于更新当前测试任务信息，具体内容包括：

    | 属性          | 取值说明                                                     |
    | ------------- | ------------------------------------------------------------ |
    | `type`        | "REPLAY"                                                     |
    | `num`         | 与cur_scene_num属性相同                                      |
    | `name`        | 对应`./scenario/replay`文件夹下某子文件夹名称                |
    | `source_file` | 需要寻找`xodr`, `xosc`, `json`三个文件路径，其中`json`文件用于记录主车相关交通信号灯信息，在部分场景中不存在 |
    | `output_path` | 输出文件以`{type}_{num}_{name}_result.csv`的形式命名         |
    | `task_info`   | 从xosc文件中解析主车初始位置 ` startPos` 、目标行驶区域`targetPos`和仿真步长`dt`的值 |

    *Return type：* ScenarioInfo

    *Returns :*

​			返回 tasks[cur_scene_num] 对应的完整测试任务信息



### `ScenarioManagerForFragment` objects

*class utils.ScenarioManager.ScenarioManagerForFragment(scenario_dir, config)*

​	`ScenarioManagerBase`的子类，用于片段式双向交互测试的测试任务管理

+ **Methods:**

  + **`_struct_scene_info`(self)**

    用于更新当前测试任务信息，具体内容包括：

    | 属性              | 取值说明                                                     |
    | ----------------- | ------------------------------------------------------------ |
    | `type`            | "FRAGMENT"                                                   |
    | `num`             | 与cur_scene_num属性相同                                      |
    | `name`            | 对应`./scenario/fragment`文件夹下某子文件夹名称              |
    | `source_file`     | 需要寻找`xodr`, `xosc`, `tess`三个文件路径                   |
    | `output_path`     | 输出文件以`{type}_{num}_{name}_result.csv`的形式命名         |
    | `task_info`       | 从xosc文件中解析主车初始位置 ` startPos` 、目标行驶区域`targetPos`和仿真步长`dt`的值 |
    | `additional_info` | 从xosc文件中解析所有背景要素的初始状态信息以字典的形式保存在键`vehicle_init_status`中，用于仿真初始场景的构建 |

    *Return type：* ScenarioInfo

​		*Returns :*

​			返回 tasks[cur_scene_num] 对应的完整测试任务信息



### `ScenarioManagerForSerial` objects

*class utils.ScenarioManager.ScenarioManagerForSerial(scenario_dir, config)*

​	`ScenarioManagerBase`的子类，用于无限里程双向交互测试的测试任务管理

+ **Methods:**

  + **`_struct_scene_info`(self)**

    用于更新当前测试任务信息，具体内容包括：

    | 属性          | 取值说明                                                     |
    | ------------- | ------------------------------------------------------------ |
    | `type`        | "SERIAL"                                                     |
    | `num`         | 与cur_scene_num属性相同                                      |
    | `name`        | 对应`./scenario/serial/tasks`文件夹下`json`文件的文件名（不包含`.json`后缀） |
    | `source_file` | 需要寻找`xodr`, `tess`两个文件路径                           |
    | `output_path` | 输出文件以`{type}_{num}_{name}_result.csv`的形式命名         |
    | `task_info`   | 从任务`json`文件中获取主车初始位置 ` startPos` 和目标行驶区域`targetPos`及途径点`waypoints`序列信息，从config配置文件中获取仿真步长`dt`的值 |

    *Return type：* ScenarioInfo

​		*Returns :*

​			返回 tasks[cur_scene_num] 对应的完整测试任务信息





## `planner.plannerBase`



### `PlannerBase` Abstract Class

*class planner.plannerBase.PlannerBase(ABC)*

​	作为选手编写自定义规控器的抽象基类，定义了必须进行重载的抽象方法

+ **Definition:**

  ```python
  from abc import ABC, abstractmethod
  from typing import List
  
  from utils.observation import Observation
  
  class PlannerBase(ABC):
      def __init__(self) -> None:
          pass
      
      @abstractmethod
      def init(self, scenario_info: dict) -> None:
          pass
      
      @abstractmethod
      def act(self, observation: Observation) -> List[float]:
          pass
  ```

+ **Methods:**

  + **`__init__`(self)**

    构造函数，用于初始化自定义规控器模块的相关属性设置

+ **Abstract Methods:**

  + **`init`(self, scenario_info)**

    初始化函数，通过`scenario_info`参数传入当前测试任务信息，可用于进行地图解析等工作

    *Parameters :*

    + `scenario_info` : *dict*

      测试任务场景相关信息，其值为[`ScenarioInfo::format()`](#scenarioinfo-objects)方法的返回值，关于该字典中键值对的简单描述如下：

      | Key             | Type   | Value                                                        |
      | --------------- | ------ | ------------------------------------------------------------ |
      | `"num"`         | *int*  | 测试场景编号                                                 |
      | `"name"`        | *str*  | 测试场景名称                                                 |
      | `"type"`        | *str*  | 测试模式，取值为`"REPLAY"|"FRAGMENT"|"SERIAL"`，分别对应回放测试、片段式双向交互测试和无限里程双向交互测试 |
      | `"output_path"` | *str*  | 测试结果输出路径                                             |
      | `"source_file"` | *dict* | 场景相关源文件路径，包含 `xodr`, `xosc`, `json`, `tess`四种后缀文件 |
      | `"task_info"`   | *dict* | 测试场景任务相关参数，包含主车初始位置 ` startPos` 、目标行驶区域`targetPos`、途径点`waypoints`序列和仿真步长`dt` |
    
    *Raises:*

    ​	`NotImplementedError`: 子类必须实现这个初始化函数

  + **`act`(self, observation)**

    响应函数，读入当前时刻的仿真环境状态信息，进行相关规划控制运算并返回主车的控制量

    *Parameters :*

    + `observation` : [*Observation*](#observation-objects)

      仿真中背景要素的当前状态信息，关于该对象的属性简单描述如下：

      | Key           | Type                                                         | Value                                                        |
      | ------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
      | `ego_info`    | [*EgoStatus*](#egostatus-objects)                            | 仿真中主车的车辆状态                                         |
      | `object_info` | *Dict[str, Dict[str, [ObjectStatus](#objectstatus-objects)]]* | 仿真环境中的各背景要素状态，包含背景车、非机动车和行人       |
      | `light_info`  | *str*                                                        | 仿真环境当前与主车通行相关的信号灯灯色                       |
      | `test_info`   | *dict*                                                       | 仿真环境的状态信息，包括当前时间戳`t`、仿真步长`dt`和仿真运行状态`end` |
    
    *Return type :* List[float]
  
    *Returns :*

    ​	以列表的形式返回主车在当前背景交通流环境下的控制量，该返回列表中各值的描述如下：

    | index | 类型    | 控制参数   | 单位    | 说明                                |
    | ----- | ------- | ---------- | ------- | ----------------------------------- |
    | 0     | *float* | 纵向加速度 | `m/s^2` |                                     |
    | 1     | *float* | 前轮转角   | `rad`   | *以主车行驶方向为0，顺时针方向为正* |
    
    *Raises:*
  
    ​	`NotImplementedError`: 子类必须实现这个初始化函数




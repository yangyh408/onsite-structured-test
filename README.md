<div align="center">
<a href="https://onsite.com.cn/">
    <!-- Please provide path to your logo here -->
    <img src="assets/ONSITE-blue-logo-cn_name.svg" alt="OnSite" width="800">
</a>

# OnSite结构化测试工具

</div>

<div align="center">
<a href="https://onsite.com.cn/"><img src="https://img.shields.io/badge/OnSite-2.0-blue"></a>
&nbsp;&nbsp;&nbsp;&nbsp;
<a href="https://tops.tongji.edu.cn/"><img src="https://img.shields.io/badge/TCU-TOPS-purple"></a>
&nbsp;&nbsp;&nbsp;&nbsp;
<a href="./LICENSE"><img src="https://img.shields.io/badge/LICENSE-BSD%203-yellow"></a>
</div>

## 使用说明

### 1 快速开始

#### 1.1 环境配置

> 注：目前OnSite回放测试工具仅支持**Windows**操作系统及**Ubuntu 20.04**两种系统环境

+ 使用conda建立虚拟环境，需指定版本为 **\*python3.6.8\***

  ```bash
   conda create -n onsite python=3.6.8
  ```

+ 激活虚拟环境

  ```bash
  conda activate onsite
  ```

+ 载入依赖库

  ```bash
  pip install -r requirements.txt
  ```

#### 1.2 加载测试任务

+ 导入待测场景文件

  - 场景根文件夹路径为`scenario/`

  - 工具中提供了各测试模式的基础测试场景，用户可以直接使用

      >   关于场景文件说明详见各测试模式的 *场景文件说明* 章节

+ 配置测试任务

  + 配置文件路径为`config/tasks.yaml`

  + 初次使用，推荐用户使用如下样例配置文件：

    >   关于测试任务配置详见各测试模式的 *测试任务配置* 章节

    ```yaml
    FRAGMENT:
      tasks:
      maxTestTime: 25
      skipExist: True
    
    SERIAL:
      tasks:
      dt: 0.1
      maxTestTime: 200
      skipExist: True
    
    REPLAY:
      tasks:
      visualize: True
      skipExist: True
    ```

#### 1.3 选择待测试的规控算法

>   默认使用样例规控器中的**IDM规控器**进行测试

+ 规控算法放置路径：`planner/`

+ 通过`planner/__init__.py`中`PLANNER`字段指定规控算法

  + IDM

    ```python
    from planner.IDM.idm import IDM 
    PLANNER = IDM
    ```

  + Lattice

    ```python
    from planner.Lattice.lattice import LATTICE
    PLANNER = LATTICE
    ```

  + 键盘控制

    >   键盘的 ↑ ↓ 键控制车速的增加和减小，← → 键控制车头朝向的左右变化

    ```python
    from planner.Manual.manual_controller import ManualController
    PLANNER = ManualController
    ```

#### 1.4 运行测试

> **首次运行TessNG时需要导入激活密钥，点击`导入激活码`后选择`第二届OnSite自动驾驶算法挑战赛-结构化测试赛道-正式版.key`激活，提示激活成功后关闭程序重新运行即可**

+ Windows环境测试指令：

  `python -u './main.py'`

+ Ubuntu 20.04环境运行指令：

  `./run_ubuntu.sh`

  > *注：若提示`permission denied`需要先执行`sudo chmod +x ./run_ubuntu.sh`*



### 2 编写规控算法

在`planner/`文件夹下创建子文件夹用于存放规控算法相关文件，规控算法编写时务必继承`planner/plannerBase.py`文件中的`PlannerBase`抽象类并复写`init`和`act`两个抽象方法。

#### 2.1 规控算法基类说明

>   **详细的规控算法基类说明参照 <[API Reference: planner.plannerBase](./docs/API_Reference.md#plannerplannerbase)>**

1.   编写`__init__(self)`构造函数
     +   功能：构造函数，用于初始化自定义规控器模块的相关属性设置
     +   调用时间：仅在测试开始前被调用一次
2.   编写`init(self, scenario_info)`初始化函数 ***（必须）*** 
     +   功能：初始化函数，通过`scenario_info`参数传入当前测试任务信息，可用于进行地图解析等工作
     +   传入参数：类型为`dict`，表示测试任务场景相关信息，其值为[`ScenarioInfo::format()`](./docs/API_Reference.md#scenarioinfo-objects)方法的返回值
     +   调用时间：在每个测试任务加载后调用，调用次数与测试任务数量一致
3.   编写`act(self, observation)`响应函数 ***（必须）*** 
     +   功能：响应函数，读入当前时刻的仿真环境状态信息，进行相关规划控制运算并返回主车的控制量
     +   传入参数：类型为[`Observation`](./docs/API_Reference.md#observation-objects)，表示仿真中背景要素的当前状态信息
     +   返回值：类型为`List[float]`，以列表的形式返回主车在当前背景交通流环境下的**纵向加速度和前轮转角**
     +   调用时间：在测试任务的每一帧会调用一次，调用次数与测试任务的帧数一致

#### 2.2 样例规控器IDM

平台提供一个基础的IDM跟驰算法，算法相关代码放置在路径`planner/IDM/`文件夹内，通过`IDM`类进行组织。

IDM跟驰算法可以实现沿直线行驶时的自动跟车，但由于IDM仅做跟驰不涉及变换车道，因此该算法在规划时*只计算纵向加速度，前轮转角始终为0*。

+ 构造函数`__init__`

  定义IDM算法相关的模型参数

  ```python
  def __init__(self, a_bound=5.0, exv=40, t=1.2, a=2.22, b=2.4, gama=4, s0=1.0, s1=2.0):
      """跟idm模型有关的模型参数
          :param a_bound: 本车加速度绝对值的上下界
          :param exv: 期望速度
          :param t: 反应时间
          :param a: 起步加速度
          :param b: 舒适减速度
          :param gama: 加速度指数
          :param s0: 静止安全距离
          :param s1: 与速度有关的安全距离选择参数
      """
      self.a_bound = a_bound
      self.exv = exv
      self.t = t
      self.a = a
      self.b = b
      self.gama = gama
      self.s0 = s0
      self.s1 = s1
      self.s_ = 0
  ```

+ 初始化函数`init`

  获取当前场景信息，并对OpenDrive文件进行解析

  ```python
  from utils.opendrive2discretenet import parse_opendrive
  
  def init(self, scenario_dict):
      print("----------------------------IDM INIT----------------------------")
      print(scenario_dict)
      print("----------------------------------------------------------------")
      road_info = parse_opendrive(scenario_dict['source_file']['xodr'])
  ```

+ 响应函数`act`

  根据传入的仿真状态规划主车下一帧的纵向加速度

  ```python
  from utils.observation import Observation
  
  def act(self, observation: Observation):
      # 加载主车信息
      frame = pd.DataFrame(
          vars(observation.ego_info),
          columns=['x', 'y', 'v', 'yaw', 'length', 'width'], 
          index=['ego']
      )
      # 加载背景要素状态信息
      for obj_type in observation.object_info:
          for obj_name, obj_info in observation.object_info[obj_type].items():
              sub_frame = pd.DataFrame(vars(obj_info), columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[obj_name])
              frame = pd.concat([frame, sub_frame])
              state = frame.to_numpy()
  
              return [self.deside_acc(state), 0]
  ```

  在进行加速度规划时需要先判断主车的跟驰对象，再根据主车与跟驰车辆的相对速度和距离计算期望加速度。涉及的类内方法如下：

  + `getInformFront(self, state: pd.DataFrame) -> Tuple[float, float, float, float]`

    获取主车跟驰对象的信息

    *Parameters :*

    + `state` : *DataFrame*

      当前仿真时刻主车及背景要素的状态。其中第一行为主车信息，记录的状态包含位置、速度、航向角、形状。

    *Return type :* Tuple

    *Returns :*

    ​	返回为主车速度、前车速度、车头间距、行驶方向，当主车与前车距离超过100米时前车速度、车头间距项返回-1

    ```python
    def getInformFront(self, state):
            if state[0, 3] < np.pi / 2 or state[0, 3] > np.pi * 3 / 2:
                direction = 1.0
            else:
                direction = -1.0
            state[:,0] = state[:,0]*direction
            ego = state[0,:]
            v, fv, dis_gap = ego[2], -1, -1
            # 在本车前侧
            x_ind = ego[0] < state[:,0]
            y_ind = (np.abs(ego[1] - state[:,1])) < ((ego[5] + state[:,5])/2)
            ind = x_ind & y_ind
            if ind.sum() > 0:
                state_ind = state[ind,:]
                front = state_ind[(state_ind[:,0]-ego[0]).argmin(),:]
                fv = front[2]
                dis_gap = front[0] - ego[0] - (ego[4] + front[4])/2
            if dis_gap > 100:
                dis_gap = -1
                fv = -1
            return v, fv, dis_gap, direction
    ```

  + `deside_acc(self, state: pd.DataFrame) -> float`

    计算主车期望加速度

    *Parameters :*

    + `state` : *DataFrame*

      当前仿真时刻主车及背景要素的状态。其中第一行为主车信息，记录的状态包含位置、速度、航向角、形状。

    *Return type :* float

    *Returns :*

    ​	返回为主车期望加速度，单位`m/s^2`

    ```python
    def deside_acc(self, state):
            v, fv, dis_gap, direction = self.getInformFront(state)
            if dis_gap < 0:
                a_idm = self.a * (1 - (v / self.exv) ** self.gama)
            else:
                # 求解本车与前车的期望距离
                self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
                    v - fv) / 2 / (self.a * self.b) ** 0.5
                # 求解本车纵向加速度
                a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
            # 对加速度进行约束
            a_idm = np.clip(a_idm, -self.a_bound, 1e7)
            return a_idm
    ```



### 3 测试规控算法及打包上传

+ **修改规控器**

  在`planner/`文件夹下创建子文件夹并以`PlannerBase`为基类构建用户自己的规控算法类。编写完成后，在`planner/__init__.py`中将`PLANNER`变量指向上述规控算法类，写法如下：

  ```python
  from planner.customPlanner import CustomPlanner
  PLANNER = CustomPlanner
  ```

+ **本地运行测试**

  >   运行方法参见 <[1.4 运行测试](#1.4-运行测试)>

  在`outputs`文件夹中查看测试的轨迹输出

+ **更新依赖文件**

  如果在规控模块编写过程中需要引入其他python第三方库，需要重新导出并覆盖原本`requirements.txt`

  ```bash
  pip freeze > requirements.txt
  ```

+ **生成docker镜像**

  ```bash
  docker build -t <hub-user>/<repo-name>:<tag> .
  ```

  + *hub-user*：DockerHub上注册的用户名

  + *repo-name*：镜像名称

    >   为防止他人抄袭或直接上传您编写的代码，请不要在相关内容中体现出 OnSite等比赛相关内容。可以采用密码生成器等手段，生成镜像名）

  + *tag*：镜像标签，标注镜像的版本信息

+ **本地测试docker镜像**

  > *注：仅限在Ubuntu20.04系统环境下开发的用户执行下述指令进行本地docker测试*
  >
  > dockerID `<hub-user>/<repo-name>:<tag>`与上述生成的镜像保持一致
  
  ```bash
  #!/bin/bash
  
  # 本机结构化测试工具位置（需要运行激活过TessNG）
  local_root_dir="/home/yangyh408/Desktop/onsite_structured_test"
  
  # 分别挂载本地场景文件、配置文件、TessNG激活证书、输出文件这四个文件路径并以后台运行容器
  sudo docker run -d \
  -v $local_root_dir/scenario:/onsite_structured_test/scenario \
  -v $local_root_dir/config:/onsite_structured_test/config \
  -v $local_root_dir/TessNG/WorkSpace/Cert:/onsite_structured_test/TessNG/WorkSpace/Cert \
  -v $local_root_dir/outputs:/onsite_structured_test/outputs \
  <hub-user>/<repo-name>:<tag>
  ```
  
+ **上传至dockerHub**

  >   dockerID `<hub-user>/<repo-name>:<tag>`与上述生成的镜像保持一致

  ```bash
  docker push <hub-user>/<repo-name>:<tag>
  ```






## 工具说明

### 1 文件结构

```
onsite_structured_test
├── main.py
├── createTasks.py
├── visualize.py
├── requirements.txt
├── run_ubuntu.sh
├── README.md
├── Dockerfile
├── config
│   ├── logging.conf
│   └── tasks.yaml
├── scenario
│   ├── fragment
│   ├── replay
│   └── serial
├── planner
│   ├── __init__.py
│   ├── plannerBase.py
│   ├── IDM
│   ├── Lattice
│   └── Manual
├── OnSiteReplay
├── TessNG
├── utils
├── assets
└── docs
```

+ 文件功能说明

  | 文件名           | 功能                                    |
  | ---------------- | --------------------------------------- |
  | main.py          | 仿真测试主程序                          |
  | createTasks.py   | 生成tess场景文件                        |
  | visualize.py     | 仿真结果与测试任务可视化程序            |
  | requirements.txt | python环境依赖                          |
  | run_ubuntu.sh    | Ubuntu20.04系统下仿真测试运行的bash指令 |
  | README.md        | 测试工具说明文档                        |
  | Dockerfile       | Docker镜像构建指令                      |

+ 文件夹功能说明

  | 文件夹名称   | 功能                        |
  | ------------ | --------------------------- |
  | config       | 仿真测试相关配置文件        |
  | scenario     | 测试场景文件夹              |
  | planner      | 规控模块                    |
  | OnSiteReplay | 回放测试相关测试模块        |
  | TessNG       | 双向交互测试相关测试模块    |
  | utils        | 测试模块使用的组件和工具    |
  | assets       | 资源文件夹，用于存放图片等  |
  | docs         | 存放更新日志、API说明等文档 |



### 2 测试模式

结构化测试工具包含了回放测试、片段式双向交互测试、无限里程双向交互测试三种不同的测试模式。

+ **回放测试**【[工具说明](./docs/Replay_Instruction.md) | [相关网页](https://onsite.com.cn/#/dist/replayTest)】

  回放测试服务应用的设定较为简单，仿真场景真实性较高，背景车辆与主车之间不存在交互行为完全按照既定轨迹行驶，可以测试规控算法在已提取的真实场景下的运行安全

+ **片段式双向交互测试**【[工具说明](./docs/Fragment_Instruction.md) | [相关网页](https://onsite.com.cn/#/dist/wayInteractionTest)】

  片段式双向交互测试通过接入国产微观仿真软件TessNG，可以提供更具交互性的仿真环境，进而对被测算法进行更为科学、可信的测评

+ **无限里程双向交互测试**【[工具说明](./docs/Serial_Instruction.md) | [相关网页](https://onsite.com.cn/#/dist/twoWayInteractionTestTool)】

  无限里程双向交互测试可以实现更大空间区域和更长时域范围的“无限里程”连续式仿真测试，进一步提升虚拟仿真测试的难度和覆盖度





## API Reference

关于本工具中涉及到的`Observation`，`ScenarioManager`，`PlannerBase`等主要类的接口说明，参见[API Reference](./docs/API_Reference.md)





## [CHANGE LOG](./docs/CHANGELOG.md)

### Refact

+ 重构输出`Recorder`模块，在输出文件中会包含选手回传的两个控制量以及经过执行器动力学修正后实际作用在主车上的控制量

### Fix

+ 更新执行器动力学约束模块，当车速减为0时重新计算加速度
+ 修复了Lattice规控器读取新版接口报错的问题

### Add

+ 添加`docs`文件夹用于存放各类文档
+ 添加三种测试模式的详细说明文档

### Update

+ 更新执行器动力学约束参数`ROT_LIMIT`和`ROT_RATE_LIMIT`
+ 将`ScenarioManagerForFragment`中场景步长的获取方式由配置文件改为通过OpenSCENARIO文件获取
+ 在可视化回放中添加了执行器动力学约束修正前后的主车控制量显示
+ 将资源文件夹由`src`改为`assets`
+ 更新`requirements.txt`
+ 更新`planner/plannerBase`的注释文档
+ 更新`Dockerfile`中的build指令

### Doc

+ 在`OnSiteReplay`中添加代码注释
+ 在`MySimulatorBase`中添加代码注释


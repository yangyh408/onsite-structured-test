## 片段式双向交互测试模式说明

### 1 场景文件说明

#### 1.1 场景文件夹结构

片段式双向交互测试场景文件夹置于目录`scenario/fragment/`下，每个测试场景以子文件夹的形式单独组织，**子文件夹的名称为测试场景名**

```
scenario
├── fragment
│   ├── 5_3_straight_straight_9
│   │   ├── 5_3_straight_straight_9.json
│   │   ├── 5_3_straight_straight_9.xodr
│   │   ├── 5_3_straight_straight_9_exam.xosc
│   │   └── 5_3_straight_straight_9.tess
│   ├── highway_merge_2_1_139
│   │   ├── highway_merge_2_1_139.tess
│   │   ├── highway_merge_2_1_139.xodr
│   │   └── highway_merge_2_1_139_exam.xosc
│   └── ...
```

#### 1.2 场景文件组成

片段式双向交互测试场景子文件夹中包含3个文件，分别为OpenDRIVE(`.xodr`)、OpenSCENARIO(`.xosc`)和TessNG场景文件(`.tess`)。

+ **OpenDRIVE文件**

  OpenDRIVE是一种用于描述道路和交通场景的开放式数据交换格式。它被广泛用于自动驾驶系统的模拟、测试和验证，是一种标准化的数据格式，可用于在不同的仿真和实时系统之间进行数据交换。

  平台采用**OpenDRIVE 1.4.0版本**，其描述了道路的拓扑结构、几何形状、道路标识和交通标识，以及车辆运动和控制方面的信息。它提供了许多元素和属性，用于描述道路的各种特征，如车道、路缘、交叉口、人行横道、停车位等。

  平台提供的OpenDRIVE文件包含道路基本属性、车道属性、交叉口进口停车线信息，其中道路属性存储在`<road>`元素下，车道属性储存在`<lanes>`元素下，停车线信息存储在`<objects>`元素下，更多有关OpenDRIVE格式的内容见[官网](https://www.asam.net/standards/detail/opendrive)。

  + 道路属性信息`<road>`

    ```xml
    <road id="3" junction="-1" length="7.6451285377263559e+01" name="Road 3">
        <link>
            <successor elementId="9" elementType="junction" />
        </link>
        <type s="0.0000000000000000e+00" type="town">
            ...
        </type>
        <planView>
            ...
        </planView>
        <elevationProfile>
            ...
        </elevationProfile>
        <lateralProfile>
            ...
        </lateralProfile>
        <lanes>
            ...
        </lanes>
        <objects>
            ...
        </objects>
    </road>
    ```

  + 车道属性`<lanes>`

    ```xml
    <lanes>
        <laneOffset a="0.0000000000000000e+00" b="0.0000000000000000e+00" c="0.0000000000000000e+00" d="0.0000000000000000e+00" s="0.0000000000000000e+00" />
        <laneOffset a="0.0000000000000000e+00" b="0.0000000000000000e+00" c="0.0000000000000000e+00" d="0.0000000000000000e+00" s="2.2396181451497117e+01" />
        <laneSection s="0.0000000000000000e+00" singleSide="false">
            <center>
                ...
            </center>
            <right>
                ...
            </right>
            <userData>
                <vectorLaneSection>
                    <carriageway leftBoundary="0" rightBoundary="-2" />
                </vectorLaneSection>
            </userData>
        </laneSection>
    </lanes>
    ```

  + 停车线信息`<objects>`

    ```xml
    <objects>
        <object hdg="1.57" height="0.05" id="0" length="8.6340316756902" name="stopLocation" orientation="none" pitch="0.0" roll="0.0" s="76.45128537726356" t="-4.3170158378451" type="roadmark" width="0.45" zOffset="0.0">
            <outlines>
                <outline id="0">
                    <cornerRoad dz="0.0" height="0.05" id="0" s="76.45128537726356" t="0.0" />
                    <cornerRoad dz="0.0" height="0.05" id="1" s="76.45128537726356" t="-8.6340316756902" />
                </outline>
            </outlines>
        </object>
    </objects>
    ```

+ **OpenSCENARIO文件**

  OpenSCENARIO是一种用于描述自动驾驶场景的开放式数据交换格式，是由欧洲自动驾驶系统开发的。它可以被广泛用于自动驾驶系统的仿真、测试和验证，是一种标准化的数据格式，可用于在不同的仿真和实时系统之间进行数据交换。

  平台采用**OpenSCENARIO 1.0版本**，其用于描述自动驾驶场景的各种元素和属性，包括车辆、行人、路标、环境、行为等。它提供了一系列的元素和属性，用于描述场景的各种特征，如交通流、道路状态、天气条件、车辆行为等。

  平台提供的OpenSCENARIO文件包含车辆、行人、非机动车的基本属性和行为，其中属性存储在`<Entities>`元素下，行为储存在`<Storyboard>`元素下，如下图所示，更多有关OpenSCENARIO格式的内容见[官网](https://www.asam.net/standards/detail/openscenario)。

  + 动态要素属性`<Entities>`

    ```xml
    <Entities>
    		<ScenarioObject name="Ego">
    			<Vehicle name="Default_car" vehicleCategory="car">
    				<BoundingBox>
    					<Center x="1.5000000000000000e+00" y="0.0000000000000000e+00" z="9.0000000000000002e-01" />
    					<Dimensions height="1.8000000000000000e+00" length="4.4207773210000001e+00" width="2.1029498580000001e+00" />
    				</BoundingBox>
    				<Performance maxAcceleration="200" maxDeceleration="10.0" maxSpeed="200" />
    				<Axles>
    					<FrontAxle maxSteering="0.5" positionX="2.8" positionZ="0.25" trackWidth="1.75" wheelDiameter="0.5" />
    					<RearAxle maxSteering="0.0" positionX="0.0" positionZ="0.25" trackWidth="1.75" wheelDiameter="0.5" />
    				</Axles>
    				<Properties />
    			</Vehicle>
    		</ScenarioObject>
    		<ScenarioObject name="A1">
    			<Vehicle name="Default_car" vehicleCategory="car">
    				<BoundingBox>
    					<Center x="1.5000000000000000e+00" y="0.0000000000000000e+00" z="9.0000000000000002e-01" />
    					<Dimensions height="1.8000000000000000e+00" length="4.5361137390000001e+00" width="2.0744521620000000e+00" />
    				</BoundingBox>
    				<Performance maxAcceleration="200" maxDeceleration="10.0" maxSpeed="200" />
    				<Axles>
    					<FrontAxle maxSteering="0.5" positionX="2.8" positionZ="0.25" trackWidth="1.75" wheelDiameter="0.5" />
    					<RearAxle maxSteering="0.0" positionX="0.0" positionZ="0.25" trackWidth="1.75" wheelDiameter="0.5" />
    				</Axles>
    				<Properties />
    			</Vehicle>
    		</ScenarioObject>
    </Entities>
    ```

  + 动态要素行为`<Storyboard>`

    ```xml
    <Storyboard>
    		<Init>
    			<Actions>
    				<GlobalAction>
    					<EnvironmentAction>
    						<Environment name="Default_Environment">
    							<TimeOfDay animation="false" dateTime="2023-05-09T16:55:00" />
    							<Weather cloudState="free">
    								<Sun azimuth="0.0" elevation="1.571" intensity="1.0" />
    								<Fog visualRange="100000.0" />
    								<Precipitation intensity="0.0" precipitationType="dry" />
    							</Weather>
    							<RoadCondition frictionScaleFactor="1.0" />
    						</Environment>
    					</EnvironmentAction>
    				</GlobalAction>
    				<Private entityRef="Ego">
    					<!--Information of the ego vehicle will be hidden, and its initial state and driving task will be explained in the comments below-->
    					<!--[Initial State] v_init = 0.097872256, x_init = 3550.097412, y_init = 14111.82031, heading_init = 3.226336718-->
    					<!--[Driving Task] x_target = (3532.298537, 3542.298537), y_target = (14018.59865, 14020.59865)-->
    				</Private>
    			</Actions>
    		</Init>
        	<Story name="inside_intersection">
                ...
    		</Story>
        	<StopTrigger>
                ...
    		</StopTrigger>
    	</Storyboard>
    ```

+ **TessNG场景文件**

  本工具采用国产微观仿真软件TessNG驱动具备交互能力的背景交通流，在TessNG运行时需要通过`.tess`后缀的二进制文件启动相应场景。在测试工具提供的场景中均包含了相应的tess文件，可以保证TessNG中对场景的正确加载。

  若采用OnSite场景库中下载的场景，其中可能不包含TessNG启动所需的tess文件。此时，用户可以运行`createTasks.py`，该程序会自动检索`scenario/fragment/`目录下没有tess文件的测试场景，并自动通过OpenDRIVE地图文件生成tess文件。

  ```python
  # createTasks.py
  
  import os
  import json
  import TessNG
  
  def main():
      # FRAGMENT赛道下针对所有没有tess文件的测试场景批量生成tess文件
      mode = 'CREATE_TESS'
      config = {'tasks': None}
  
      TessNG.run(mode, config, auto_run=False)
      
      if mode == 'CREATE_TESS':
          temp_file_path = './temp/create_failed.json'
          if os.path.exists(temp_file_path):
              with open(temp_file_path, 'r') as f:
                  failed_list = json.load(f)
              for tess_path in failed_list:
                  if os.path.exists(tess_path):
                      os.remove(tess_path)
                      print(f"Remove {tess_path}")
              os.remove(temp_file_path)
  
  if __name__ == '__main__':
      main()
  ```

  

#### 1.3 场景可视化

测试工具中提供了测试场景可视化模块，便于用户查看场景的地图信息、测试任务起点和终点面域分布。

+ **静态地图可视化**

  <img src="..\assets\show_map.png" alt="show_map" style="zoom:50%;" />

  ```python
  # visualize.py
  
  from utils.visualizer import Visualizer
  
  if __name__ == '__main__':
      vis = Visualizer()
      vis.show_map(xodr_path='.\scenario\fragment\5_3_straight_straight_9\5_3_straight_straight_9.xodr')
  ```

  + **`show_map`(self, xodr_path)**

    *method utils.visualizer.Visualizer::show_map*

    对OpenDRIVE格式的静态地图进行可视化展示

    *Parameters :*

    + `xodr_path` : *str*

      待可视化的OpenDRIVE文件路径

+ **测试任务可视化**

  <img src="..\assets\show_task_fragment.png" alt="show_task_fragment" style="zoom:50%;" />

  ```python
  # visualize.py
  
  from utils.visualizer import Visualizer
  
  if __name__ == '__main__':
      vis = Visualizer()
      vis.show_task(mode='FRAGMENT', task='5_3_straight_straight_9')
  ```

  + **`show_task`(self, mode, task)**

    *method utils.visualizer.Visualizer::show_task*

    对测试任务进行可视化展示，包括静态地图、主车起点和终点面域

    *Parameters :*

    + `mode` : *str*

      测试任务所属的测试模式，片段式双向交互测试取值为`"FRAGMENT"`

    + `task` : *str*

      测试任务名称，对应测试场景文件夹`./scenario/fragment`下子文件夹名称



### 2 配置文件说明

#### 2.1 测试任务配置

+ 配置文件路径为`config/tasks.yaml`，片段式双向交互测试模式相关配置需要写在`FRAGMENT`字段下

+ 配置文件组织形式：

  > 以下提供两种测试形式对应的配置文件

  ```yaml
  # 配置一：测试指定任务（列表可以增删期望测试的场景名称）
  FRAGMENT:
    tasks:
    	- "0_76_merge_82"
    	- "5_3_straight_straight_9"
    maxTestTime: 25
    skipExist: False
  
  # 配置二：测试场景文件夹下所有任务（批量测试）
  FRAGMENT:
    tasks:
    maxTestTime: 25
    skipExist: True
  ```

+ 配置文件参数说明：

  `tasks`: *list, default: []*

  ​	选定测试用例，列表中每一项为测试任务名称（对应场景文件夹下各子文件夹的文件名）

  ​	字段默认值为`[]`，当字段为`[] | None`时代表测试场景文件夹下的所有测试任务

  `maxTestTime`: *int, default: 30*

  ​	单位为秒，表示片段式场景的最大测试时长*（即仿真环境内时间戳的最大值）*

  `skipExist`: *bool, default: False*

  ​	是否跳过输出文件夹中已有的输出文件的测试任务*（通常用于批量测试中对部分异常场景进行重新测试或补充测试）*

#### 2.2 日志文件配置

> *该配置文件用于设置测试日志文件的输出等级、输出形式及输出文件路径，**用户在使用时可忽略***

+ 配置文件路径为`config/logging.conf`

+ 日志文件相关配置：

  ```yaml
  [handler_fileHandler]
  class=FileHandler
  level=DEBUG
  formatter=simpleFormatter
  args=('./outputs/test_info.log', 'a', 'utf-8')
  
  [formatter_simpleFormatter]
  format=%(asctime)s - %(levelname)8s - %(message)s
  datefmt=%Y-%m-%d %H:%M:%S
  ```

+ 配置文件参数说明：

  + `handler_fileHandler`指定了输出等级和输出文件路径
    + `level`: 可选项为`DEBUG | INFO | ERROR`，表示不同等级的日志输出
    + `args`: 第一项为日志文件路径，第二项为文件读写模式，第三项为编码方式
  + `formatter_simpleFormatter`指定了文件输出的形式



### 3 仿真运行说明

#### 3.1 仿真接入架构

<img src="..\assets\onsite_tessng_process.png" alt="onsite_tessng_process" style="zoom:50%;" />

#### 3.2 TessNG仿真界面元素

<img src="..\assets\tessng-gui.png" alt="onsite_tessng_process" style="zoom:50%;" />

1. 绿色圆圈：主车起点
2. 红色矩形：测试终点面域
3. 紫色车辆：距离主车50m范围内的背景车（会在observation中记录）
4. 白色车辆：其余距离主车较远的背景车（不会在observation中记录）
5. 蓝色车辆：实际主车位置映射在TessNG中用于与背景交通流交互的“影子主车”*（可忽略）*
6. 黄色矩形：实际主车位置，蓝色短线指向主车航向

#### 3.3 仿真流程组织

+ 片段式双向交互测试的仿真流程在`TessNG/__init__.py`文件中的`run`函数下启动

  ```python
  def startTessNG(mode: str, mode_config: dict, planner: object, scene_info: ScenarioInfo, auto_run: bool) -> None:
      # 创建工作目录
      workspace_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WorkSpace')
      if not os.path.exists(workspace_dir):
          os.makedirs(workspace_dir)
      # 启动TessNG并传入测试相关参数
      app = QApplication()
      config = {'__workspace': workspace_dir,
                '__simuafterload': auto_run,
                '__custsimubysteps': False
                }
      tess_file = scene_info.source_file['tess']
      if tess_file:
          config['__netfilepath'] = rf"{tess_file}"
      plugin = MyPlugin(mode, mode_config, planner, scene_info)
      factory = TessngFactory()
      tessng = factory.build(plugin, config)
  
      if tessng is None:
          sys.exit(0)
      else:
          sys.exit(app.exec_())
  
  def run(mode: str, mode_config: dict, planner: object=None, scene_info: ScenarioInfo=ScenarioInfo(), auto_run: bool=True) -> None:
      """启动TessNG进行测试
      Args:
          mode (str): 测试模式
          mode_config (dict): 测试配置
          planner (object, optional): 规划器对象. Defaults to None.
          scene_info (ScenarioInfo, optional): 测试场景信息. Defaults to ScenarioInfo().
          auto_run (bool, optional): 是否自动运行测试. Defaults to True.
      """
      tessng_p = Process(target=startTessNG, args=(mode, mode_config, planner, scene_info, auto_run))
      tessng_p.start()
      tessng_p.join()
  ```

+ TessNG启动后对二次开发相关接口进行复写，以接入测试流程，主要流程由`.\TessNG\MySimulator\MySimulatorFragment.py`控制

  + **TessNG仿真初始化**：在此步骤调用`planner::init`函数传入测试场景相关信息

    ```python
    def __init__(self, config: dict, planner: object, scene_info: ScenarioInfo):
        MySimulatorBase.__init__(self)
    
        # 最大测试时长
        self.maxTestTime = config.get('maxTestTime', 30)
        # 实例化规控器并初始化
        self.planner = planner
        self.planner.init(scene_info.format())
        # 加载场景信息
        self.scenario_info = scene_info
        # 测试间隔
        self.dt = scene_info.task_info['dt']
        # 仿真预热时间
        self.preheatingTime = 0
        # 背景车探测范围
        self.radius = 50
    ```

  + **TessNG单步运行函数**：即每个仿真步后调用的函数，在此步骤调用`planner::act`传入仿真环境信息并接收主车控制量，对控制量进行执行器动力学修正后更新主车信息

    ```python
    def afterOneStep(self):
            iface = tessngIFace()
            simuiface = iface.simuInterface()
            netiface = iface.netInterface()
            # 主要测试流程
            self.mainStep(simuiface, netiface)
            # 检查测试环境中主车是否驶出路网
            self._checkOutSideMap(netiface)
            # 更新主车对应TessNG中影子车的状态信息
            self._updateShadowEgo(simuiface, netiface)
    
    def mainStep(self, simuiface, netiface):
        simuTime = simuiface.simuTimeIntervalWithAcceMutiples()
        # 判断是否到达预热时间
        if simuTime >= self.preheatingTime * 1000 and not self.finishTest:
            # 判断是否在仿真中添加主车及背景车
            if not self.createCarLock:
                self._addCar(simuiface, netiface)
                self.createCarLock = 1
            # 判断主车是否已经成功进入仿真环境
            if self.ego_info == None:
                self._createEgoInfo(simuiface, netiface)
            else:
                # 获取当前时刻仿真环境观测信息
                self.observation = self._tessngServerMsg(simuiface, simuTime)
                # 判断仿真是否还在进行中
                if self.observation.test_info['end'] == -1:
                    # 记录当前测试信息
                    self.recorder.record(self.action, self.observation)
                    # 获取规控模块回传的控制信息
                    self.action = self.planner.act(self.observation)  
                    # 对规控器回传的控制信息进行执行器动力学约束修正
                    ego_action = check_action(
                        dt = self.dt, 
                        prev_v = self.observation.ego_info.v,
                        prev_action = [self.observation.ego_info.a, self.observation.ego_info.rot],
                        new_action = self.action
                    )
                    # 根据修正后的控制量更新主车位置
                    updateEgoPos(ego_action, self.dt, self.ego_info)
                    # 在TessNG中绘制主车位置
                    paintPos["pos"] = self.ego_info.__dict__
                else:
                    # 如果仿真到达终止条件则停止仿真
                    self.finishTest = True
                    self.recorder.record(self.action, self.observation)
                    self.forStopSimu.emit()
    ```

  + **Tess仿真结束**：仿真结束后输出结果文件并终止TessNG进程

    ```python
    def afterStop(self):
        # 输出测试结果
        self.recorder.output(self.scenario_info.output_path)
        # 退出仿真
        kill_process(os.getpid())
    ```

#### 3.4 执行器动力学约束

当前测试工具使用的测试车辆模型为运动学模型，模型输入为纵向加速度和前轮转角，模型示意图和方程如下：

<img src="..\assets\dynamic_model.png" alt="dynamic_model" style="zoom:50%;" />

其中，$\phi$为航向角，$\varphi$ 为横摆角，$\beta$为质心侧偏角，$v_x$为纵向车速，$a_x$为纵向加速度，$\delta_f$为前轮转角，$l_r$和$l_f$为质心到前后轴的距离，$X_g$和$Y_g$为车辆在大地坐标系下的位置。

在运动学模型的基础上，基于实际执行器的物理特性，测试工具对涉及执行器动力学相关的参数进行了限制：

+ 驱动系统限制

  | 参数         | 限制条件               | 限制范围             | 描述                                                         |
  | ------------ | ---------------------- | -------------------- | ------------------------------------------------------------ |
  | 纵向车速     | $|v_x| \leq v_x^{lim}$ | $200km/h=55.5 m/s$   | 普通乘用车设计最高车速限制                                   |
  | 纵向加速度   | $|a_x| \leq a_x^{lim}$ | $1g=9.8 m/s^2$       | 普通乘用车在附着良好的路面上的极限加速度                     |
  | 纵向加加速度 | $|j_x|\leq j_x^{lim}$  | $1g/0.2s=49.0 m/s^3$ | 驱动电机的响应时间限制*（在速度减为0，即停车时允许较大的jerk值）* |

+ 转向系统限制

  | 参数     | 限制条件                         | 限制范围              | 描述                                     |
  | -------- | -------------------------------- | --------------------- | ---------------------------------------- |
  | 前轮转角 | $|\delta_f| \leq \delta_f^{lim}$ | $40^\circ=0.7rad$     | 大多数汽车的前轮最大转角在30度到40度之间 |
  | 前轮转速 | $|\omega_f|\leq \omega_f^{lim}$  | $80^\circ/s=1.4rad/s$ | 转向电机响应和执行时间限制               |

**当选手输入的前轮转角和加速度等参数超过上述执行器限制时，实际输入按照最大限制条件执行。** 车速为状态更新的中间变量在输入侧不做限制但会在动力学校核时进行检查。

#### 3.5 主车更新逻辑

主车状态信息采用前向欧拉的规则更新，采用**运动学自行车模型**。当$t\rightarrow t+\Delta t$时刻，**得到经过执行器动力约束修正的主车纵向加速度$\alpha$与前轮转角$\delta$作为输入**，按照以下步骤更新车辆状态：

<img src="..\assets\bicycle_model.png" alt="bicycle_model" style="zoom: 25%;" />

+ **更新主车位置**
  $$
  \begin{aligned}  &x_{t+\Delta t}=x_t+v_t\times \cos \left( \theta _t \right) \times \Delta t\\  &y_{t+\Delta t}=y_t+v_t\times \sin \left( \theta _t \right) \times \Delta t\\\end{aligned}
  $$
  其中：

  + $x_t,y_t$ 为当前时刻车辆的x与y坐标
  + $x_{t+\Delta t},y_{t+\Delta t}$为下一时刻车辆的x与y坐标
  + $v_t$为当前时刻的车辆速度
  + $\theta_t$为当前时刻的车辆航向角
  + $\Delta t $​为一个步长的时间长度, 环境更新周期

+ **更新自车航向角**
  $$
  \theta _{t+\Delta t}=\theta _t+v_t/l\times \tan\mathrm{(}\delta )\times \Delta t
  $$
  其中：

  - $\theta _{t+\Delta t}$ 为下一时刻的车辆偏航角
  - $\delta$为前轮转角
  - $l$为车辆轴距，这里约定其值为：车辆长度 ÷ 1.7
  - *其余未说明的符号与前述一致*

+ **更新主车速度**
  $$
  v_{t+\Delta t} = max(0, v_t + a *\Delta t)
  $$
  其中：

  - $v_{t+\Delta t}$为下一时刻的车辆速度
  - $a$​为纵向加速度
  - *其余未说明的符号与前述一致*

#### 3.6 测试输出文件

+ **输出文件路径**：`outputs/`

+ **输出文件命名**：`{测试模式}_{测试编号}_{场景名称}_result.csv`

+ **输出文件内容**：

  <img src="..\assets\result_file.png" alt="result_file"/>

  + 第1列：仿真时间戳

  + 第2-3列：规控模块返回的控制量

    *该控制量是**未经执行器动力学修正**的规控模块`planner::act`方法的返回值*

  + 第4-11列：主车状态信息

    分别为主车的横坐标、纵坐标、速度、加速度、航向角、前轮转角、宽度、长度，字段值与`observation.ego_info`对应

    *主车的加速度和前轮转角是**经过执行器动力学修正**的值*

  + 最后1列：**仿真运行状态**

    该列值与`observation.test_info['end']`值对应，一般而言，除最后一行外其余值均为-1。end列各值含义如下：

    + `-1`表示仿真在运行中
    + `1`表示主车完成测试任务，即成功抵达目标区域
    + `2`表示仿真运行超时，即超过OpenSCENARIO的最长记录时间
    + `3`表示主车与背景要素发生碰撞
    + `4`表示主车驶出地图边界，即超过了静态地图边界框范围

  + 其余列：表示仿真环境中其他背景要素的状态信息

    以7列为一组分别表示对应背景要素的横坐标、纵坐标、速度、加速度、航向角、宽度、长度，背景要素的名称为列名下划线后的部分

+ **输出文件可视化回放**

  <img src="..\assets\FRAGMENT_5_highway_merge_2_2_113_result.gif" alt="FRAGMENT_5_highway_merge_2_2_113_result" style="zoom:80%;" />

  ```python
  # visualize.py
  
  from utils.visualizer import Visualizer
  
  if __name__ == '__main__':
      result_path = r".\outputs\FRAGMENT_5_highway_merge_2_2_113_result.csv"
      save_path = r".\outputs\FRAGMENT_5_highway_merge_2_2_113_result.gif"
      
      vis = Visualizer()
      vis.replay_result(result_path=result_path, save_path=save_path)
  ```

  + **`replay_result`(self, result_path, save_path)**

    *method utils.visualizer.Visualizer::replay_result*

    对测试输出文件进行可视化回放或保存为GIF图片

    *Parameters :*

    + `result_path` : *str*

      测试输出`csv`文件路径

    + `save_path` : *str, default: None*

      输出gif文件路径，文件名称需要以`.gif`结尾

      >  当该值为`None`时仅做可视化回放展示，不保存为GIF图片；否则，会向`save_path`输出可视化结果对应的GIF图

  + **可视化界面介绍：**

    + 左上角为场景数据表格，其中`acc`、`rot`项括号外为经过执行器动力学修正实际作用在主车上的纵向加速度和前轮转角，而括号内是动力学修正前规控器回传的控制量
    + 左下角为缩略图概览，蓝色框表示右图的可视化范围
    + 右侧为以主车为中心的局部场景展示，主车为橙色，其余背景要素为蓝色
    + 下方为播放控制，按键分别为*上一帧*、*倒退*、*暂停*、*播放*、*下一帧*，右侧进度条表示当前展示的帧数




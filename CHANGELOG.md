# Change Log

## [Ver 3.1]

### Refact

+ 重构输出`Recorder`模块，在输出文件中会包含选手回传的两个控制量以及经过执行器动力学修正后实际作用在主车上的控制量


### Fix

+ 更新执行器动力学约束模块，当车速减为0时重新计算加速度

### Update

+ 更新执行器动力学约束参数`ROT_LIMIT`和`ROT_RATE_LIMIT`
+ 将`ScenarioManagerForFragment`中场景步长的获取方式由配置文件改为通过OpenSCENARIO文件获取
+ 在可视化回放中添加了执行器动力学约束修正前后的主车控制量显示

### Doc

+ 更新`planner/plannerBase`的注释文档
+ 在`OnSiteReplay`中添加代码注释


## [Ver 3.0] - 2024.03.09

### Refact

+ 重新封装了OpenDrive地图解析模块，可通过`utils.opendrive2discretenet.parse_opendrive`调用

+ **重构测试流程，将场景管理放在`main.py`中完成，每个测试任务会单独调用`TessNG.run`或`OnSiteReplay.run`**

+ 在场景运行中添加`try-except`结构以防止测试被个别异常场景中断

+ **支持Windows、Ubuntu20.04双系统环境启动，将TessNG相关动态链接库存储在`TessNG/DLLs`目录**

+ 重构`ScenarioManager`模块，引入`ScenarioInfo`类进行场景信息管理

+ 重构`Observation`模块，引入`ObjectInfo`类进行物体状态信息管理

  

### Fix

+ 修复了双向交互测试中个别场景主车驶出路网无法被检测的情况

+ 修复了Opendrive转Tess文件时汇入汇出区地图转换不全的问题(`TessNG.createTess.opendrive2tess`)

+ 修复了碰撞检测算法

+ 将TessNG连接段获取接口由`vehicle.lane()`修改为`vehicle.laneConnector()`

+ 将初始帧主车的控制量（加速度、前轮转角）由`nan`改为`0`

  > 作为测试约定，这样可以避免主车在第一帧传入过大的控制量

+ 修复了双向交互测试中影子主车异常消失的问题，在影子主车被删除后会重新加载新的影子主车



### Add

+ 在`utils.functions`中添加动力学约束检测函数`check_action`
+ 添加结果可视化回放功能`utils.visualizer::replay_result`
+ 添加OpenDrive地图可视化功能`utils.visualizer::show_map`
+ 添加测试任务可视化功能`utils.visualizer::show_task`
+ 添加日志记录模块及配置文件`utils.logger`
+ 添加支持键盘控制主车的规划控制模块`planner.Manual`



### Update

+ 调整场景文件中提供的样例场景类型和数量
  + 回放模式包含了高速基本段、高速汇入汇出、交叉口、机非混行
  + 片段式双向交互包含高速基本段、高速汇入汇出（对应5种不同底图）、交叉口（对应2种不同底图）
  + 无限里程双向交互包括两个任务场景（底图均为同济测试场，任务起终点、waypoints序列不同）
+ 更新`MySimulatorCreateTess`模块，可以标记生成tess文件失败的测试任务
+ 修改了片段式双向交互测试的预热时间：`5 -> 0`
+ 更新回放测试可视化界面，采用与结果可视化模块相同的模块



### Remove

+ 删除`TessNG.run`中启用额外线程监测程序运行状态的模块
+ 删除MySimulatorBase`中启用额外线程监测仿真运行情况的模块



### Doc

+ 更新`requirements.txt`
+ 更新`.gitignore`


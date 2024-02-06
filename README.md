## 双向交互模块使用说明

### 接入流程
![work_process](./src/onsite_tessng_process.png)

### 环境配置
  1. 使用conda建立虚拟环境，需指定版本为**python3.6.6**
     `conda create -n onsite python=3.6.6`
  2. 激活虚拟环境
     `conda activate onsite`
  3. 载入依赖库
      `pip install -r requirements.txt`

### 场景文件说明
```bash
   senario
   ├── fragment
   │   ├── 0019cutin1
   │   │   ├── xxxx.xodr
   │   │   └── xxxx.xosc
   │   └── ...
   └── serial
       ├── maps	
       │   ├── TJST
       │   │   ├── xxxx.xodr
       │   │   └── xxxx.tess
       │   └── ...
       └── tasks
           ├── xxxx.json
           └── ...
```
   1. 片段式场景文件`senario/fragment`
      + 该文件夹下存放各片段场景目录，文件夹名称即为场景名
      + 二级场景文件夹下存放对应的*静态路网文件(.xodr)*和*动态车辆信息(.xosc)*
   2. 无限里程场景文件`senario/serial`
      + `maps`文件夹中存放所有测试任务共用的静态地图文件，地图文件夹下包含*xodr格式路网*和*tess二进制启动文件*
      + `tasks`文件夹中存放各测试任务(.json)文件，每个json文件中包含测试任务对应的*静态地图(map)*、*测试起点(startPos)*、*测试终点面域(targetPos)*及*参考轨迹路径(waypoints)*
      > json文件中静态地图名称与maps文件夹下静态地图文件夹名称保持一致 

### 用户操作说明
  1. 修改配置文件`utils/config.py`
      + `PLANNER`为主车控制算法，需要选手以**PlannerBase**为基类编写自己的规控器，关于Planner的详情可以参考`./planner/plannerBase.py`
         > 模块提供了**IDM**和**键盘控制(ManualController)**两个样例控制器作为参考  
      + `MODE`为选择双向交互测试模式：`fragment`为片段式双向交互测试，`serial`为无限里程连续仿真测试
      + `TASKS`为指定测试任务，其值为空时会自动执行对应测试模式下所有可执行的场景
         > 片段式双向交互指定测试任务时使用`senario/fragment`目录下子文件夹名称，如：`0019cutin1`
         > 无限里程双向交互指定测试任务是使用`senario/serial/tasks`目录下json文件名,如：`Cyz_TJST_1.json`
  2. 在`planner`文件夹内新建文件夹并以`plannerBase.py->PlannerBase`作为基类编写自己的规划控制类，具体形式可以参考planner中给出的两个样例规控器（IDM和Manual）
  3. 运行`main.py`进行测试
     > 首次运行TessNG时需要导入激活密钥，点击`导入激活码`后选择`src/JidaTraffic_key.key`激活，提升激活成功后关闭程序重新运行即可
  4. 在`outputs`文件夹中查看所有测试的轨迹输出

### TessNG界面介绍
![TessNG_GUI](./src/tessng_GUI.png)
  1. 矩形标志： 
     + 绿色矩形：场景起始位置坐标 
     + 红色矩形：场景终点位置坐标 
     + 黄色矩形：测试车实际位置（与Observation中ego信息相同，蓝线为主车航向）
  2. 车辆标志： 
     + 蓝色车辆：测试车在TessNG路网中交互的投影（无实际意义）
     + 紫色车辆：记录在Observation中的背景车 *（以主车位置为中心某探测半径范围内的车辆，探测半径可在`utils/config.py`中通过`radius`变量设置）*
     + 白色车辆：未记录在Observation中的其他背景车
  3. 点击TessNG GUI界面左侧边栏`显示路径`按钮可以展示当前场景下的waypoints序列 *(仅针对无限里程双向交互测试)*
import os
from planner.IDM.idm import IDM 
from planner.Manual.manual_controller import ManualController
from planner.Lattice.lattice import alg_1

# ***************************************************************************用户配置的参数片段***************************************************************************

# 选手编写的Planner类
# PLANNER = IDM
PLANNER = ManualController

# 选择测试模式："fragment"表示片段式仿真，"serial"表示连续式仿真
MODE = 'serial'
# MODE = 'serial'

# 如果为空则表示运行所有场景

# 当MODE == 'fragment'时，TASK样例为：
# TASKS = ['test1']
# 当MODE == 'serial'时，TASK样例为：
TASKS = ['Cyz_TJST_1.json', 'Cyz_TJST_2.json']

# TESSNG被测试车，车辆类型
EgoTypeCode = 1

# 测试车周围半径，单位为米
radius = 50

# 仿真预热时间，单位为秒
preheatingTime = 5

# **********************************************************************************************************************************************************************


# ********************************************************************以下为自动配置的参数，用户无需修改********************************************************************

ROOT_PATH = os.path.abspath(os.path.join(os.path.abspath(__file__), '..', '..'))
SCENARIO_PATH = os.path.join(ROOT_PATH, 'scenario', MODE)
RESULT_PATH = os.path.join(ROOT_PATH, 'outputs')
if not os.path.exists(RESULT_PATH):
    os.makedirs(RESULT_PATH)

# 主车信息
EGO_INFO = {
    'name': 'ego',
    'width': 2.02,      # TessNG中无法修改主车的宽度信息
    'length': 4.55,
    'dynamic': '',
}
# 单个测试步长，单位为秒
dt = 0.05
# 每秒计算批次
batchNumPerSecond = 1 / dt
# 总计结算批次，达到这个批次后，本次测试结束，配置测试最大时间单位为秒（60秒）
calculateBatchesFinish = 60 * batchNumPerSecond

# ***********************************************************************************************************************************************************************

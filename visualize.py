from utils.visualizer import Visualizer

if __name__ == '__main__':
    result_path = r"H:\onsite结构化测试赛道\最新版本\onsite_structured_test\outputs\SERIAL_1_Cyz_TJST_2_result.csv"
    save_path = None
    xodr_path = r"H:\onsite结构化测试赛道\场景\机非混合赛题设计（来源：郭慧洁）\机非冲突.xodr"

    vis = Visualizer()
    # vis.replay_result(result_path=result_path, save_path=save_path)
    # vis.show_task(mode='REPLAY', task='1_282_merge_292')
    vis.show_map(xodr_path=xodr_path)
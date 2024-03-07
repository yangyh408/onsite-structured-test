from utils.visualizer import Visualizer

if __name__ == '__main__':
    result_path = r"H:\onsite结构化测试赛道\最新版本\onsite_structured_test\outputs\SERIAL_1_Cyz_TJST_2_result.csv"
    save_path = None

    vis = Visualizer()
    vis.replay_result(result_path=result_path, save_path=save_path)
    # vis.show_task(mode='REPLAY', task='1_282_merge_292')
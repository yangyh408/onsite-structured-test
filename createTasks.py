import os
import json
import TessNG

def main():
    # FRAGMENT赛道下针对所有没有tess文件的测试场景批量生成tess文件
    mode = 'CREATE_TESS'
    config = {'tasks': None}

    # SERAIL赛道下生成包含waypoints的赛题
    # mode = 'CREATE_WAYPOINTS'
    # config = {'task_num': 26, 'map': 'TJST_pure', 'start_lane':0, 'link_ids': [284, 181]}

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
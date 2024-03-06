import TessNG

def main():
    # FRAGMENT赛道下针对所有没有tess文件的测试场景批量生成tess文件
    mode = 'CREATE_TESS'
    config = {'tasks': None}

    # SERAIL赛道下生成包含waypoints的赛题
    # mode = 'CREATE_WAYPOINTS'
    # config = {'task_num': 26, 'map': 'TJST_pure', 'start_lane':0, 'link_ids': [284, 181, 304, 215, 45, 178]}

    TessNG.run(mode, config, None, auto_run=False)

if __name__ == '__main__':
    main()
import yaml

import TessNG
import OnSiteReplay

from planner import PLANNER

def main():
    with open('./config/tasks.yaml', 'r') as f:
        tasks = yaml.safe_load(f)
    for mode, config in tasks.items():
        if mode != 'REPLAY':
            TessNG.run(mode, config, PLANNER)
        else:
            OnSiteReplay.run(config, PLANNER)

if __name__ == '__main__':
    main()
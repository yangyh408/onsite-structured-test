import yaml

import TessNG
import OnSiteReplay

from planner.IDM.idm import IDM 
from planner.Manual.manual_controller import ManualController
from planner.Lattice.lattice import alg_1

PLANNER = IDM

def main():
    with open('./config/tasks.yaml', 'r') as f:
        tasks = yaml.safe_load(f)
    for mode, config in tasks.items():
        if mode != 'replay':
            TessNG.run(mode, config, PLANNER)
        else:
            OnSiteReplay.run(config, PLANNER)

if __name__ == '__main__':
    main()
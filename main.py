from sumoSimulation.cfg import simulationCfg
from sumoSimulation.simulation import Simulator

import os

print(os.getenv("SUDO_USER"))

path = "./cfg/hello.json"
cfg = simulationCfg(path)
simu = Simulator(cfg)
simu.run()

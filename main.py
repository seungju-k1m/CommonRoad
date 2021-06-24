from sumoSimulation.cfg import simulationCfg
from sumoSimulation.simulation import Simulator

path = "./cfg/hello.json"
cfg = simulationCfg(path)
simu = Simulator(cfg)
simu.run()

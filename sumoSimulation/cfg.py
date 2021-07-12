import json


class simulationCfg:

    def __init__(self, path):
        with open(path) as j:
            self._jsonFile = json.load(j)
        self.base_path = None
        self.scenario_name = None
        self.gui = True
        self.step = 1000
        self.wid = 1.6
        self.length = 4.3
        self.sumo_conf = {}
        self.generate_rou_file = False
        for key, value in self._jsonFile.items():
            setattr(self, key, value)

    @property
    def jsonFile(self):
        return self._jsonFile

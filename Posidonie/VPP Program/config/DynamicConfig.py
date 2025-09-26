import sys
import pathlib
import copy
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *

class DynamicConfig():
    def __init__(self):
        self.loadConfigFile()

    def loadConfigFile(self):
        """
        Load the infos of the config file
        """
        try:
            with open(PATH_CONFIG_FILE, 'r') as f:
                self.data = json.load(f)
        except:
            # Initialisation
            print("[INFO] - Initializing the dynamic config")
            self.data = copy.deepcopy(DEFAULT_DYNAMIC_CONFIG)
            with open(PATH_CONFIG_FILE, 'w') as f:
                f.write(json.dumps(self.data))

    def set(self, name, value):
        """
        Set a config parameter
        """
        self.data[name] = value

        with open(PATH_CONFIG_FILE, 'w') as f:
            f.write(json.dumps(self.data))

    def get(self, name):
        """
        Return a config parameter
        """
        if (name in self.data):
            return self.data[name]
        else:
            raise Exception(f"Unknow attribute {name} for dynamic config.")
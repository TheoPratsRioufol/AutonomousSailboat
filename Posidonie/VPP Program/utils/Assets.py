from PIL import Image, ImageTk
import os
import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *

def getAssetNames(path):
    """
    Return the list of the file name corresponding to png assets in a folder <path>
    """
    return [f.name[:-len('.png')] for f in os.scandir(path) if f.is_file() and (len(f.name) >= len('.png')) and (f.name[-len('.png'):] == '.png')]


class Assets():
    def __init__(self):
        self.loadAssets()
        self.loadSplash()

    def loadSplash(self):
        img = Image.open(PATH_RES_ICON+ "splash.png").resize((SPLASH_WIN_W, SPLASH_WIN_H))
        self.assets['splash'] = ImageTk.PhotoImage(img)

    def loadAssets(self):
        self.assets = {}
        for name in getAssetNames(PATH_RES_ICON):
            img = Image.open(PATH_RES_ICON + name + ".png").resize((ICON_SIZE, ICON_SIZE))
            self.assets[name] = ImageTk.PhotoImage(img)

    def get(self, name):
        if name in self.assets:
            return self.assets[name]
        return self.assets["unknow"]
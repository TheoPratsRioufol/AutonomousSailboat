import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Assets import *
from config.Config import *
from frontend.simu.BoatViewver import *

class ViewverWindow():
    def __init__(self, app):
        self.app = app
        self.boatViewver = None

    def display(self,x=None, y=None):
        self.win = tk.Toplevel()
        self.win.title("Posidonie Viewver")
        #win.wm_iconphoto(False, self.app.getAssets().get(APP_LOGO))
        if (x != None):
            self.win.geometry(f"{VIEWVER_WIDTH}x{VIEWVER_HEIGHT}+{x}+{y}")
        else:
            self.win.geometry(f"{VIEWVER_WIDTH}x{VIEWVER_HEIGHT}")

        def close():
            self.win.destroy()
            self.win.update()
            self.app.quit_me()

        self.boatViewver = BoatViewver(self.win, self.app)
        self.boatViewver.pack(fill=tk.BOTH, expand=True)

        self.win.protocol("WM_DELETE_WINDOW", lambda:close())
        #win.focus_force()
        #win.grab_set()

        self.buildMenuBar()

    def buildMenuBar(self):
        menubar = tk.Menu(self.win)

        # Camera Menu
        cam_menu = tk.Menu(menubar, tearoff=False)
        cam_menu.add_command(label='Fit',command=self.boatViewver.fitCamera, accelerator="F")
        cam_menu.add_command(label='Top view',command=self.boatViewver.setCameraToTopView)
        cam_menu.add_command(label='Side view',command=self.boatViewver.setCameraToSideView)
        cam_menu.add_command(label='Fron viewt',command=self.boatViewver.setCameraToFrontView)

        # Simulation Menu
        sim_menu = tk.Menu(menubar, tearoff=False)
        sim_menu.add_command(label='Start/Stop',command=self.boatViewver.getRunStopButton().toggle, accelerator="Space")
        sim_menu.add_command(label='Speed up',command=self.boatViewver.speedUpSimu)
        sim_menu.add_command(label='Slow down',command=self.boatViewver.slowdownSimu)
        sim_menu.add_command(label='Reset',command=self.boatViewver.resetSimu, accelerator="R")
        sim_menu.add_command(label='Step forward',command=self.boatViewver.nextStep, accelerator="N")
        sim_menu.add_command(label='Step backward',command=self.boatViewver.previousStep, accelerator="P")
        sim_menu.add_command(label='Plot State Vector',command=self.boatViewver.plotStateVector)

        menubar.add_cascade(label="Camera", menu=cam_menu)
        menubar.add_cascade(label="Simulation", menu=sim_menu)
        self.win.config(menu=menubar)

    def getBoatViewver(self):
        return self.boatViewver
    
    def getStabilityEditor(self):
        if (self.boatViewver == None):
            return None
        return self.boatViewver.getStabilityEditor()
    
    def run(self):
        # blocking:
        self.boatViewver.run()
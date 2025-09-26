import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *
from backend.naca.NACACalculator import *

class NACAEditor(tk.Frame):
    
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app

        # Create a figure
        self.fig = plt.figure(figsize=(4, 6), dpi=100)
        
        #self.fig.tight_layout()
        self.fig.subplots_adjust(right=0.85)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        NavigationToolbar2Tk(self.canvas, self).update() 
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.plot()

    def plot(self, fig=None, profile=None):
        """
        Plot the data from the NACA model
        """
        if (fig == None):
            fig = self.fig
        # Clear the axe
        fig.clf()
        ax = fig.add_subplot(1,1,1)
        ax2 = ax.twinx()
        
        ax.set_title('NACA Wing Force and Torque')

        naca = self.app.getBoat().getSolver().getNACACalculator()
        profiles = naca.getProfiles()
        if profile == None:
            profile = profiles[0]
        angs = np.array(naca.getModelFileAngs(profile))
        angshd = np.linspace(min(angs), max(angs), 200)

        # plot MODEL POINTS
        data = np.array([naca.getModelDic()[profile][a] for a in angs])
        ax.plot(np.rad2deg(angs), data[:,0], 'r.')
        ax.plot(np.rad2deg(angs), data[:,1], 'b.')
        ax2.plot(np.rad2deg(angs), data[:,2], 'g.')

        # plot POLY FIT
        data = np.array([naca.interpolateNACA(a, profile) for a in angshd])
        ax.plot(np.rad2deg(angshd), data[:,0], 'r', label='Cx (Drag)')
        ax.plot(np.rad2deg(angshd), data[:,1], 'b', label='Cy (Lift)')
        ax2.plot(np.rad2deg(angshd), data[:,2], 'g', label='CMz (Moment)')

        ax.set_ylabel('Aerodynamic Coefficient [-]')
        ax2.set_ylabel('Torque Coefficient [-]')
        ax2.tick_params(axis='y', labelcolor='g')
        ax.set_xlabel('Incidence Angle [deg]')

        ax.legend(loc='upper left')
        ax2.legend(loc='lower left')
        ax.grid()

        # Update
        self.canvas.draw()

        return ax, ax2
    
    def getProfiles(self):
        """
        Return the list of available profiles
        """
        return self.app.getBoat().getSolver().getNACACalculator().getProfiles()

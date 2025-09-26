import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.widgets import Slider
import matplotlib
import random

matplotlib.use('TkAgg')

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *

#tslider = None # Global variable for slider

class LivePlot():
    def __init__(self):
        self.axs = []
        self.traces = {}
        self.fig = plt.figure(figsize=(4, 3.5), dpi=100)
        self.canvas = None
        self.tlim = 1

    def reset(self):
        """
        Reset all the plots
        """
        for ax in self.axs:
            for trace in self.traces[ax]:
                trace.reset()

    def addSubplot(self, nrow, ncol, index):
        """
        Add a new subplot
        """
        ax = self.fig.add_subplot(nrow, ncol, index)
        self.axs.append(ax)
        self.traces[ax] = []

    def addTrace(self, datafct, artistic='b', label=''):
        """
        Add a trace to the live plot (attached to the current subplot)
        """
        ax = self.axs[-1] # Current ax
        self.traces[ax].append(LiveTrace(ax, datafct, artistic, label))

    def plot(self, solver, t, X, U, T, disp=True):
        """
        Update the plot
        """
        for ax in self.axs:
            for trace in self.traces[ax]:
                trace.plot(solver, t, X, U, T, disp)
            # Update the limits
            if disp and (len(trace.x) > 1):
                xm = trace.minx()
                xM = trace.maxx()
                ax.relim()
                ax.autoscale_view()
                ax.set_xlim(xM - (xM - xm)*self.tlim, xM)

        if disp:
            self.update()

    def forceUpdate(self):
        """
        Update the plot with the existing data
        """
        for ax in self.axs:
            for trace in self.traces[ax]:
                trace.forceUpdate()
            # Update the limits
            if (len(trace.x) > 1):
                xm = trace.minx()
                xM = trace.maxx()
                ax.relim()
                ax.autoscale_view()
                ax.set_xlim(xM - (xM - xm)*self.tlim, xM)
        self.update()

    def setTlim(self, value):
        """
        Set the time limit (x axis)
        """
        self.tlim = value
        self.update()

    def legend(self):
        """
        Legend the last ax
        """
        ax = self.axs[-1]
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        ax.grid()

    def set_ylabel(self, label):
        """
        Set the y label of the current ax
        """
        self.axs[-1].set_ylabel(label)

    def set_xlabel(self, label):
        """
        Set the x label of the current ax
        """
        self.axs[-1].set_xlabel(label)

    def set_title(self, label):
        """
        Set the title of the current ax
        """
        self.axs[-1].set_title(label)

    def finish(self):
        """
        Finish the configuration of the live plot
        """

    def attach(self, pane):
        """
        Attach the figure to a tkinter pane
        """
        self.fig.subplots_adjust(right=0.75)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=pane)
        NavigationToolbar2Tk(self.canvas, pane).update() 
        widget = self.canvas.get_tk_widget()
        widget.focus_force()
        widget.pack(fill=tk.BOTH, expand=True)

        # finish by drawing
        self.update()
    
    def update(self):
        if (self.canvas != None):
            self.canvas.draw()


class LiveTrace():
    def __init__(self, ax, datafct, artistic='b', label=''):
        """
        Used to create live graph
            - ax : the axe on which to plot
            - artistic : appearance of the trace
            - label: label
            - datafct (signature: X,U,T -> float) process the data
        """
        self.trace = ax.plot([], [], artistic, label=label)[0]
        self.label = label
        self.datafct = datafct
        self.reset()

    def reset(self):
        self.y = []
        self.x = []
        self.max = None
        self.min = None

    def plot(self, solver, t, X, U, T, disp=True):
        """
        Update the y-axis ith new data
        and update the plot
        """
        try:
            val = self.datafct(solver, X, U, T)
        except:
            val = None

        if (val == None):
            print(f"[ERROR] Invalid data for LiveTrace '{self.label}'")
            return
        
        if (self.max == None) or (val > self.max):
            self.max = val

        if (self.min == None) or (val < self.min):
            self.min = val

        self.y.append(val)
        self.x.append(t)
        if disp:
            self.trace.set_data(self.x, self.y)

    def forceUpdate(self):
        """
        Update the plot with the existing data
        """
        self.trace.set_data(self.x, self.y)

    def maxy(self):
        return self.max
    
    def miny(self):
        return self.min
    
    def maxx(self):
        return self.x[-1]
    
    def minx(self):
        return self.x[0]


class PlotViewver(tk.Frame):
    def __init__(self, master, result):
        super().__init__(master)

        # Create a notebook for the different type of curves
        noteBook = ttk.Notebook(self)
        
        self.cinematicPane = tk.Frame(noteBook)
        self.mechanicPane = tk.Frame(noteBook)

        noteBook.add(self.cinematicPane, text="Cinematic plots")
        noteBook.add(self.mechanicPane, text="Mechanical plots")

        noteBook.pack(fill=tk.BOTH, expand=True)

        self.initLivePlot()
        self.attachLivePlot()

    def initLivePlot(self):
        """
        Initialize a live plot
        """
        # START WITH THE CINEMATIC PLOT
        self.cinematiclplot = LivePlot()

        # Draw the speed data
        self.cinematiclplot.addSubplot(3, 1, 1)
        self.cinematiclplot.set_title("Cinematic")
        self.cinematiclplot.addTrace(lambda s, X, U, T: ms2noeud(U._u_wspeed), 'r', 'Wind Speed (knt)')
        self.cinematiclplot.addTrace(lambda s, X, U, T: ms2noeud(X.getBoatSpeedNorm()), 'b', 'Boat Speed (knt)')
        self.cinematiclplot.set_ylabel('Speeds')
        self.cinematiclplot.legend()

        # Draw the heading data
        self.cinematiclplot.addSubplot(3, 1, 2)
        self.cinematiclplot.addTrace(lambda s, X, U, T: np.rad2deg(T['target']), 'r', 'Target Angle (deg)')
        self.cinematiclplot.addTrace(lambda s, X, U, T: np.rad2deg(U._u_wang), 'r-.', 'Wind Angle (deg)')
        self.cinematiclplot.addTrace(lambda s, X, U, T: np.rad2deg(X.getBoatHeading()), 'b', 'Boat Heading (deg)')
        self.cinematiclplot.addTrace(lambda s, X, U, T: np.rad2deg(X.getTrueHeading()), 'g', 'True Heading (deg)')
        self.cinematiclplot.addTrace(lambda s, X, U, T: np.rad2deg(X._x_speed.getSpeed().getSolver().getRudderAng()), 'c', 'Rudder cmd (deg)')
        self.cinematiclplot.set_ylabel('Heading')
        self.cinematiclplot.legend()

        # Draw the orientation data
        self.cinematiclplot.addSubplot(3, 1, 3)
        self.cinematiclplot.addTrace(lambda s, X, U, T: np.rad2deg(X.getBoatRoll()), 'b', 'Roll  (deg)')
        self.cinematiclplot.addTrace(lambda s, X, U, T: np.rad2deg(X.getBoatPitch()), 'b-.', 'Pitch (deg)')
        self.cinematiclplot.set_ylabel('Angles')
        self.cinematiclplot.set_xlabel('Time(s)')
        self.cinematiclplot.legend()

        self.cinematiclplot.finish()

        # THEN THE MECHANICAL PLOT
        self.mechaniclplot = LivePlot()

        cname = 'sail'

        # Draw the speed data
        self.mechaniclplot.addSubplot(3, 1, 1)
        self.mechaniclplot.set_title("Mechanical")
        self.mechaniclplot.addTrace(lambda s, X, U, T: s.getHullToCompForce(cname).getNorm(), 'r', '||F||')
        self.mechaniclplot.set_ylabel(cname + ' linkage force')
        self.mechaniclplot.legend()

        self.mechaniclplot.addSubplot(3, 1, 2)
        self.mechaniclplot.addTrace(lambda s, X, U, T: s.getHullToCompMoment(cname).valueIn(Base.BOAT)[0], 'r', 'Mx')
        self.mechaniclplot.addTrace(lambda s, X, U, T: s.getHullToCompMoment(cname).valueIn(Base.BOAT)[1], 'b', 'My')
        self.mechaniclplot.addTrace(lambda s, X, U, T: s.getHullToCompMoment(cname).valueIn(Base.BOAT)[2], 'g', 'Mz')
        self.mechaniclplot.set_ylabel(cname + ' linkage moment')
        self.mechaniclplot.legend()

        self.mechaniclplot.finish()

    def attachLivePlot(self):
        """
        Attach the live plots
        """
        self.cinematiclplot.attach(self.cinematicPane)
        self.mechaniclplot.attach(self.mechanicPane)

    def plot(self, solver, t, X, U, T, disp=True):
        """
        Plot a live graph
            - solver: solver object
            - t: time
            - X: state vector
            - U: command state vector
            - T: target
        """
        self.cinematiclplot.plot(solver, t, X, U, T, disp)
        self.mechaniclplot.plot(solver, t, X, U, T, disp)

    def updateCurves(self):
        """
        Update the display of the curves
        """
        self.cinematiclplot.update()
        self.mechaniclplot.update()

    def forceUpdateCurves(self):
        """
        Update the display of the curves
        """
        self.cinematiclplot.forceUpdate()
        self.mechaniclplot.forceUpdate()

    def resetPlot(self):
        self.cinematiclplot.reset()
        self.mechaniclplot.reset()
        self.updateCurves()

    def setTlim(self, value):
        self.cinematiclplot.setTlim(value)
        self.mechaniclplot.setTlim(value)
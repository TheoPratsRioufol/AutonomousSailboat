import tkinter as tk
from tkinter import ttk
import matplotlib as mpl
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
from frontend.navigation.FctBlock import FctBlockType


class MxFigure():
    def __init__(self, mxPlot, fig):
        self.mxPlot = mxPlot
        self.ax = fig.add_subplot(3,1,1)
        self.ax2 = fig.add_subplot(3,1,2)
        self.ax3 = fig.add_subplot(3,1,3)
        self.clearPlot()

    def initAxs(self):
        self.ax.set_ylabel('Boat Speed [knt]')
        self.ax2.set_ylabel('Boat Rotation [deg]')
        self.ax3.set_ylabel('Capsizing Moments [Nm]')
        self.ax.set_xlabel('Wind speed [knt]')

    def newTrace(self, heading):
        newTrace = MxTrace(self.mxPlot.getApp().getBoat().getSolver(), [self.ax, self.ax2, self.ax3], heading, self.mxPlot.getApp().getGeomStamp())
        self.traces.append(newTrace)
        return newTrace

    def plot(self, wspeed):
        """
        Plot in the current trace
        """
        if len(self.traces) == 0:
            return
    
        self.traces[-1].plot(wspeed)

        # then update the limit
        self.fitTraces()

    def fitTraces(self):
        if len(self.traces) == 0:
            return
        xlim = self.traces[-1].getXlim()
        ylims = self.traces[-1].getYlims()
        for trace in self.traces:
            xm, xM = trace.getXlim()
            if (xm < xlim[0]):
                xlim[0] = xm
            if (xM > xlim[1]):
                xlim[1] = xM

            ylimsarr = trace.getYlims()
            for i in range(len(ylims)):
                ym, yM = ylimsarr[i]
                if (ym < ylims[i][0]):
                    ylims[i][0] = ym
                if (yM > ylims[i][1]):
                    ylims[i][1] = yM
            
        xlim = expandPlotLim(xlim, 0.05)
        for ax in [self.ax, self.ax2, self.ax3]:
            ax.set_xlim(xlim)

        self.ax.set_ylim(expandPlotLim(ylims[0], 0.2))
        self.ax2.set_ylim(expandPlotLim(ylims[1], 0.2))
        self.ax3.set_ylim(expandPlotLim(ylims[2], 0.2))

    def getSaveDic(self):
        """
        Return the save dic of this component
        """
        save = {'traces':[]}
        for trace in self.traces:
            save['traces'].append(trace.getSaveDic())
        return save

    def load(self, save):
        """
        Load from a save dictionnary
        """
        self.traces = []
        if ('traces' in save):
            print("load trace!")
            for traceDic in save['traces']:
                newTrace = self.newTrace(0)
                newTrace.load(traceDic)
        self.fitTraces()

    def clearPlot(self):
        """
        Clear the plot
        """
        self.ax.cla()
        self.ax2.cla()
        self.ax3.cla()

        self.traces = []

        self.initAxs()

    def getTraces(self):
        return self.traces

class MxTrace():
    def __init__(self, solver, axs, heading, geomStamp):
        """
        Mx Trace save the trace for a Mx Run
        """
        self._solver = solver
        self.axs = axs
        self.wspeeds = []
        self.bspeeds = []
        self.rolls = []
        self.pitchs = []
        self.mxs = []
        self.mys = []
        self.heading = heading
        self.geomStamp = geomStamp
        print("The Geom Stamp is",geomStamp)

        # create the traces
        self.strace = axs[0].plot([], [], '.-')[0]
        self.rtrace = axs[1].plot([], [], '.--')[0]
        self.ptrace = axs[1].plot([], [], '.--')[0]
        self.mxtrace = axs[2].plot([], [], '.-')[0]
        self.mytrace = axs[2].plot([], [], '.-')[0]
        self.setLabel()
        self.legend()

        self.xlim = [0,0]
        self.ylims = [[0,0],[0,0],[0,0]]

    def legend(self):
        for ax in self.axs:
            ax.legend()

    def setLabel(self):
        self.strace.set_label("Speed @h={:.2f}°".format(np.rad2deg(self.heading)))
        self.rtrace.set_label("Roll @h={:.2f}°".format(np.rad2deg(self.heading)))
        self.ptrace.set_label("Pitch @h={:.2f}°".format(np.rad2deg(self.heading)))
        self.mxtrace.set_label("Mx @h={:.2f}°".format(np.rad2deg(self.heading)))
        self.mytrace.set_label("My @h={:.2f}°".format(np.rad2deg(self.heading)))

    def plot(self, wspeed):
        """
        Load the info from the solver
        """
        self.wspeeds.append(wspeed)
        self.bspeeds.append(self._solver.getState().getBoatSpeedNorm())
        self.rolls.append(self._solver.getState().getBoatRoll())
        self.pitchs.append(self._solver.getState().getBoatPitch())

        moment = self._solver.getCapsizingMoment().getVec().valueIn(Base.BOAT)
        self.mxs.append(moment[0])
        self.mys.append(moment[1])

        self.updateTraces()

    def updateTraces(self):
        self.strace.set_xdata(self.wspeeds)
        self.strace.set_ydata(ms2noeud(self.bspeeds))

        self.rtrace.set_xdata(self.wspeeds)
        self.rtrace.set_ydata(np.rad2deg(self.rolls))

        self.ptrace.set_xdata(self.wspeeds)
        self.ptrace.set_ydata(np.rad2deg(self.pitchs))

        self.mxtrace.set_xdata(self.wspeeds)
        self.mxtrace.set_ydata(self.mxs)

        self.mytrace.set_xdata(self.wspeeds)
        self.mytrace.set_ydata(self.mys)

        self.updateLimits()

    def updateLimits(self):
        self.xlim = [min(self.wspeeds), max(self.wspeeds)]
        self.ylims = [[min(ms2noeud(self.bspeeds)), max(ms2noeud(self.bspeeds))], 
                np.rad2deg([min(min(self.rolls), min(self.pitchs)), max(max(self.rolls), max(self.pitchs))]),
                [min(min(self.mxs), min(self.mys)), max(max(self.mxs), max(self.mys))]]

    def getXlim(self):
        return self.xlim
    
    def getYlims(self):
        return self.ylims

    def getSaveDic(self):
        """
        Return the save dic of the composant
        """
        return {'wspeeds':list(self.wspeeds),
                'bspeeds':list(self.bspeeds),
                'rolls':list(self.rolls),
                'pitchs':list(self.pitchs),
                'mxs':list(self.mxs),
                'mys':list(self.mys),
                'heading':self.heading,
                'geomStamp':self.geomStamp}

    def load(self, save):
        """
        Load from a save dic
        """
        self.wspeeds = save['wspeeds']
        self.bspeeds = save['bspeeds']
        self.rolls = save['rolls']
        self.pitchs = save['pitchs']
        self.mxs = save['mxs']
        self.mys = save['mys']
        self.heading = save['heading']
        if ('geomStamp' in save):
            self.geomStamp = save['geomStamp']
        self.updateTraces()
        self.setLabel()
        self.legend()


class MxPlot(tk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app

        boxBorder = tk.Frame(self, background=COLOR_BOX_BORDER, padx=5, pady=5)
        box = tk.Frame(boxBorder, background=COLOR_BOX, padx=5, pady=5)

        tk.Label(box, text="Capsizing Moment Estimator", background=COLOR_BOX, font=TITLE_FONT).pack()
        inPane = tk.Frame(box, background=COLOR_BOX)
        ewidth = 5

        self.heading = tk.StringVar(value=90)
        ttk.Label(inPane, text="Wind Heading (deg):", background=COLOR_BOX).grid(row=0, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.heading, width=ewidth).grid(row=0, column=1, sticky=tk.NSEW)

        self.wmin = tk.StringVar(value=8)
        ttk.Label(inPane, text="Minimum Wind Speed (knt):", background=COLOR_BOX).grid(row=0, column=2, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.wmin, width=ewidth).grid(row=0, column=3, sticky=tk.NSEW)

        self.wmax = tk.StringVar(value=30)
        ttk.Label(inPane, text="Maximum Wind Speed (knt):", background=COLOR_BOX).grid(row=0, column=4, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.wmax, width=ewidth).grid(row=0, column=5, sticky=tk.NSEW)

        self.tslope = tk.StringVar(value=40)
        ttk.Label(inPane, text="Slope time (s):", background=COLOR_BOX).grid(row=1, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.tslope, width=ewidth).grid(row=1, column=1, sticky=tk.NSEW)

        self.anglecv = tk.StringVar(value=5)
        ttk.Label(inPane, text="Angle convergence (deg):", background=COLOR_BOX).grid(row=1, column=2, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.anglecv, width=ewidth).grid(row=1, column=3, sticky=tk.NSEW)

        self.cvTime = tk.StringVar(value=1)
        ttk.Label(inPane, text="Convergence time (s):", background=COLOR_BOX).grid(row=1, column=4, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.cvTime, width=ewidth).grid(row=1, column=5, sticky=tk.NSEW)

        self.N = tk.StringVar(value=30)
        ttk.Label(inPane, text="Number of points", background=COLOR_BOX).grid(row=2, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.N, width=ewidth).grid(row=2, column=1, sticky=tk.NSEW)

        inPane.pack(fill=tk.X, expand=True)

        butPane = tk.Frame(box, background=COLOR_BOX)

        ttk.Button(butPane, text="Clear", command=self.clearPlot).pack(side=tk.LEFT)
        ttk.Button(butPane, text="Generate", command=self.generateTestbench).pack(side=tk.LEFT)
        ttk.Button(butPane, text="Gen&Run", command=self.generateAndRun).pack(side=tk.LEFT)
        ttk.Button(butPane, text="Complete Caracteristic", command=self.completeCaracteristic).pack(side=tk.LEFT)
        

        butPane.pack(fill=tk.X, expand=True)
        box.pack(fill=tk.X, expand=True)
        boxBorder.pack(fill=tk.X, expand=False, padx=5, pady=5)

        """
        Add plot canvas
        """
        self.fig = plt.figure(figsize=(4, 3.5), dpi=100)
        self.mxFigure = MxFigure(self, self.fig)
        
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        NavigationToolbar2Tk(self.canvas, self).update() 
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.callback = None

    def getApp(self):
        return self.app

    def newTrace(self):
        self.mxFigure.newTrace(np.deg2rad(floatifyVar(self.heading)))
        
    def generateAndRun(self, auto=False):
        """
        Generate and run the testbench
        """
        self.app.getBoatViewver().resetSimu()
        self.generateTestbench(auto=auto)
        self.newTrace()
        self.app.getBoatViewver().setLiveComputation(True)
        self.app.getBoatViewver().getRunStopButton().set(True)
        self.app.getNavController().setBeginFromCurrentTime(True)
        
        self.after(200, self.app.getNavController().sendTargetFct)
        if (auto):
            self.app.getBoatViewver().setCrashCallback(self.nextAutoStep)

    def autoForm(self, n):
        """
        Auto fill the Mx caracteristic entry depending a run case n
        """
        self.wmin.set(COMPLETE_MX_CHARACTERISTIC_FORMS[n]['wmin'])
        self.wmax.set(COMPLETE_MX_CHARACTERISTIC_FORMS[n]['wmax'])
        self.tslope.set(COMPLETE_MX_CHARACTERISTIC_FORMS[n]['tslope'])
        self.anglecv.set(COMPLETE_MX_CHARACTERISTIC_FORMS[n]['anglecv'])
        self.cvTime.set(COMPLETE_MX_CHARACTERISTIC_FORMS[n]['cvTime'])
        self.heading.set(COMPLETE_MX_CHARACTERISTIC_FORMS[n]['heading'])
        self.N.set(COMPLETE_MX_CHARACTERISTIC_FORMS[n]['N'])

    def completeCaracteristic(self):
        """
        Create the complete caracteristic of the boat
        """
        self.completeRun = -1
        self.nextAutoStep()

    def generateTestbench(self, auto=False):
        """
        Generate the testbench
        """
        wmin = floatifyVar(self.wmin)
        wmax = floatifyVar(self.wmax)
        tslope= floatifyVar(self.tslope)
        anglecv = floatifyVar(self.anglecv)
        cvTime = floatifyVar(self.cvTime)
        heading = floatifyVar(self.heading)
        N = int(floatifyVar(self.N))

        wspeeds = np.linspace(wmin, wmax, N)
        teditorSave = {}

        # Create the block for the wind (angle)
        teditorSave['wang']   = {'gui':[{'type':FctBlockType.CONSTANT.value,
                                        'duration':1,
                                        'value':heading}]}
        
        # Create the block for the target (always 0)
        teditorSave['target']   = {'gui':[{'type':FctBlockType.CONSTANT.value,
                                        'duration':1,
                                        'value':0}]}
        
        # then create the slope of the wind speed
        blocks = []

        # First, wait for convergence at target = 0
        blocks.append({'type':FctBlockType.CONSTANT.value,
                        'duration':0.01,
                        'value':wmin})
                
        blocks.append({'type':FctBlockType.CONV.value,
                        'duration':cvTime,
                        'value':anglecv})
        
        # Then create a gentle slope of wind speed
        for i in range(len(wspeeds)):
            wspeed = wspeeds[i]
            blocks.append({'type':FctBlockType.SLOPE.value,
                        'duration':tslope/len(wspeeds),
                        'value':wspeed})
                
            logcode  =  f"if abs(X.getTrueHeading()) > {np.deg2rad(anglecv)}:\n"
            logcode += f"    print('THE HEADING DID NOT CONVERGE!')\n"
            logcode +=  "elif app != None:\n"
            logcode += f"    app.getMxPlot().plot({wspeed})\n"
            blocks.append({'type':FctBlockType.EXEC.value,
                        'duration':0.01,
                        'value':logcode})
                
            blocks.append({'type':FctBlockType.CONSTANT.value,
                        'duration':0.01,
                        'value':wspeed})
            
        # Add the call back at the end if needed
        if auto:
            finalCode  = "if app != None:\n"
            finalCode += "    app.getMxPlot().nextAutoStep()\n"
            blocks.append({'type':FctBlockType.EXEC.value,
                            'duration':0.01,
                            'value':finalCode})
            
        teditorSave['wspeed'] = {'gui':blocks}

        # Finally, load it
        self.app.getNavController().load({'editors':teditorSave})

    def nextAutoStep(self):
        self.completeRun += 1
        if (self.completeRun >= len(COMPLETE_MX_CHARACTERISTIC_FORMS)):
            tk.messagebox.showinfo("Mx Plot Generator", f"The caculation of the boat's characteristic is finished!") 
            return
        print(f"[INFO] - Complete caracteristic run {self.completeRun+1}/{len(COMPLETE_MX_CHARACTERISTIC_FORMS)}")
        self.autoForm(self.completeRun)
        self.generateAndRun(auto=True)
            
    def plot(self, wspeed):
        """
        Called to plot the Mx info
        """
        self.mxFigure.plot(wspeed)
        self.canvas.draw()

    def getSaveDic(self):
        """
        Return the save dic of this component
        """
        return {'wmin':self.wmin.get(),
                'wmax':self.wmax.get(),
                'tslope':self.tslope.get(),
                'anglecv':self.anglecv.get(),
                'cvTime':self.cvTime.get(),
                'heading':self.heading.get(),
                'N':self.N.get(),
                'mxFigure':self.mxFigure.getSaveDic()}
        

    def load(self, save):
        """
        Load a save dic
        """

        self.wmin.set(save['wmin'])
        self.wmax.set(save['wmax'])
        self.tslope.set(save['tslope'])
        self.anglecv.set(save['anglecv'])
        self.cvTime.set(save['cvTime'])
        self.heading.set(save['heading'])
        self.N.set(save['N'])

        self.mxFigure.load(save['mxFigure'])

        self.canvas.draw()

    def clearPlot(self):
        self.mxFigure.clearPlot()
        self.canvas.draw()

    def getMxFigure(self):
        return self.mxFigure


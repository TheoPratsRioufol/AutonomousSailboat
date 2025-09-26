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

class PolarPlotStrategy(Enum):
    RESET_STEP = "Reseted steps"
    SLOPE      = "Slope"

class PolarTrace():
    def __init__(self, axS, axR, wspeed, setup, geomStamp, color='r'):
        """
        Create a polar trace:
            - axS: axe for the speed polar plot
            - axR: axe for the roll  polar plot
            - wspeed: wind speed
            - color: color of the trace
            - setup: setup (dictionnary) used to obtain the trace
        """
        self.setup = setup
        self.wspeed = wspeed
        self.geomStamp = geomStamp

        self.tS = axS.plot([], [], color+'.-', label=self.getLabel())[0]
        self.tSMirror = axS.plot([], [], color+'.-')[0]

        self.tR = axR.plot([], [], color+'.-', label=self.getLabel())[0]
        self.tRMirror = axR.plot([], [], color+'.-')[0]

        self.angs = []
        self.speeds = []
        self.rolls = []
        self.pitchs = []

    def plot(self, ang, speed, roll, pitch):
        self.angs.append(ang)
        self.rolls.append(np.abs(roll))
        self.pitchs.append(pitch)
        self.speeds.append(speed)
        self.updateTrace()
    
    def getLabel(self):
        """
        Return the label of the trace
        """
        return 'w='+str(self.wspeed)+'knt'
    
    def updateTrace(self):
        """
        Update the trace
        """
        self.updateOnAx(self.tS, self.tSMirror, self.tR, self.tRMirror)

    def updateOnAx(self, tS, tSMirror, tR, tRMirror):
        """
        Update the data providing the axes
        """
        # The speed polar
        tS.set_xdata(self.angs)
        tS.set_ydata(self.speeds)
        tSMirror.set_xdata(-np.array(self.angs))
        tSMirror.set_ydata(self.speeds)

        # The roll polar
        tR.set_xdata(self.angs)
        tR.set_ydata(np.rad2deg(np.array(self.rolls)))
        tRMirror.set_xdata(-np.array(self.angs))
        tRMirror.set_ydata(np.rad2deg(np.array(self.rolls)))

    def getRLimit(self):
        """
        Return the max roll of the trace
        """
        if len(self.rolls) == 0:
            return 0
        return np.rad2deg(max(self.rolls))
    
    def getSLimit(self):
        """
        Return the max speed of the trace
        """
        if len(self.speeds) == 0:
            return 0
        return max(self.speeds)
    
    def getSaveDic(self):
        """
        Return the save dictionnary
        """
        return {'wspeed':self.wspeed,
                'angs':self.angs,
                'speeds':self.speeds,
                'rolls':self.rolls,
                'pitchs':self.pitchs,
                'setup':self.setup,
                'geomStamp':self.geomStamp}
    
    def load(self, save):
        """
        Load a save dictionnary
        """
        if ('geomStamp' in save):
            self.geomStamp = save['geomStamp']
        self.angs = save['angs']
        self.speeds = save['speeds']
        self.wspeed = save['wspeed']
        if ('pitchs' in save):
            self.pitchs = save['pitchs']
        self.rolls = save['rolls']
        if ('setup' in save):
            self.setup = save['setup']
        self.tS.set_label(self.getLabel())
        self.tR.set_label(self.getLabel())
        self.updateTrace()

    def getSetup(self):
        """
        Return the setup used to produce this trace
        """
        return self.setup
        

class PolarGenerator(tk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.recording = False
        self.polarTraces = []

        boxBorder = tk.Frame(self, background=COLOR_BOX_BORDER, padx=5, pady=5)
        box = tk.Frame(boxBorder, background=COLOR_BOX, padx=5, pady=5)
        
        tk.Label(box, text="Polar Speed Plot", background=COLOR_BOX, font=TITLE_FONT).pack()
        inPane = tk.Frame(box, background=COLOR_BOX)
        
        """
        Parameter of the polar plot:
            - Wind speed
            - Nb points
            - Cv time
            - Cv angle tollerance
        """
        
        # Wind speed entry
        self.windSpeedVar = tk.StringVar(value=15)
        ttk.Label(inPane, text="Wind speed (knt):", background=COLOR_BOX).grid(row=0, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.windSpeedVar).grid(row=0, column=1, sticky=tk.NSEW)

        # Nb of point entry
        self.nbHeadingVar = tk.StringVar(value=5)
        ttk.Label(inPane, text="Number of headings:", background=COLOR_BOX).grid(row=0, column=2, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.nbHeadingVar).grid(row=0, column=3, sticky=tk.NSEW)

        # Steady time entry
        self.steadyTimeVar = tk.StringVar(value=3)
        ttk.Label(inPane, text="Minimum steady time (s):", background=COLOR_BOX).grid(row=1, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.steadyTimeVar).grid(row=1, column=1, sticky=tk.NSEW)

        # Steady angle entry
        self.steadyAngVar = tk.StringVar(value=1)
        ttk.Label(inPane, text="Max. steady variation (deg):", background=COLOR_BOX).grid(row=1, column=2, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.steadyAngVar).grid(row=1, column=3, sticky=tk.NSEW)
        
        # Target Erorr angle entry
        self.targetErrorVar = tk.StringVar(value=5)
        ttk.Label(inPane, text="Max. heading error (deg):", background=COLOR_BOX).grid(row=2, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.targetErrorVar).grid(row=2, column=1, sticky=tk.NSEW)

        # Strategy combobox
        ttk.Label(inPane, text="Method:", background=COLOR_BOX).grid(row=2, column=2, sticky=tk.NSEW)
        self.strategyCb = ttk.Combobox(inPane, values=[e.value for e in PolarPlotStrategy], state="readonly")
        self.strategyCb.set(PolarPlotStrategy.SLOPE.value)
        self.strategyCb.grid(row=2, column=3, sticky=tk.NSEW)

        # Start buttons
        ttk.Button(inPane, text="Generate", command=self.generateTestbench).grid(row=3, column=0)
        ttk.Button(inPane, text="Clear Fig.", command=self.clearPlot).grid(row=3, column=1)
        ttk.Button(inPane, text="Gen&Run", command=self.generateAndRun).grid(row=3, column=2)

        inPane.pack(fill=tk.X, expand=True)
        box.pack(fill=tk.X, expand=True)
        boxBorder.pack(fill=tk.X, expand=False, padx=5, pady=5)

        # Create a figure
        self.fig = plt.figure(figsize=(4, 3.5), dpi=100)
        self.axS = self.fig.add_subplot(2,1,1, projection='polar')
        self.axR = self.fig.add_subplot(2,1,2, projection='polar')
        
        
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        NavigationToolbar2Tk(self.canvas, self).update() 
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.clearPlot()

    def newTrace(self, wspeed=0):
        """
        Add a new trace to the plot
        Return the trace added
        """
        self.polarTraces.append(PolarTrace(self.axS, self.axR, wspeed, self.getSetupDic(), self.app.getGeomStamp(), getPlotColor(len(self.polarTraces))))
        self.axS.legend(loc='center left', bbox_to_anchor=(1.15, 0.5))
        self.axR.legend(loc='center left', bbox_to_anchor=(1.15, 0.5))
        return self.polarTraces[-1]

    def plot(self, ang, speed, roll, pitch):
        """
        Called to plot the polar plot
        """
        if (len(self.polarTraces) == 0):
            return
        self.polarTraces[-1].plot(ang, speed, roll, pitch)
        self.fitTrace()

    def fitTrace(self):
        """
        Fit the trace to the visible
        """
        if (len(self.polarTraces) == 0):
            return
        maxS = max([trace.getSLimit() for trace in self.polarTraces])
        maxR = max([trace.getRLimit() for trace in self.polarTraces])

        self.axS.set_ylim([0, 1.05*maxS])
        self.axR.set_ylim([0, 1.05*maxR])
        self.canvas.draw()

    def clearPlot(self):
        """
        Clear the plot
        """
        self.axR.cla()
        self.axS.cla()
        self.axS.set_title("Speed Polar")
        self.axR.set_title("Roll Polar")
        self.polarTraces = []
        self.canvas.draw()

    def generateTestbench(self):
        """
        Generate the polar plot testbench
        """
        wspeed = floatifyVar(self.windSpeedVar)
        N = int(floatifyVar(self.nbHeadingVar))
        steadyTime = floatifyVar(self.steadyTimeVar)
        steadyAng = floatifyVar(self.steadyAngVar)

        headings = np.linspace(45, 170, N)
        blocks = []
        loop = 0
        teditorSave = {}
            
        if (self.strategyCb.get() == PolarPlotStrategy.RESET_STEP.value):
            """
            "Reset - Step" strategy
            """
            print(f"Generate {N} headings for RESET-STEP STRATEGY")

            # Create all the blocks for the wind dirrection
            for ang in headings:
                loop += 1
                # First, send a message
                blocks.append({'type':FctBlockType.MSG.value,
                            'duration':1,
                            'value':"{:.1f}%-Wind angle is {:.2f} deg".format(100*loop/N, ang)})

                # Then, constant target
                blocks.append({'type':FctBlockType.CONSTANT.value,
                            'duration':0.5,
                            'value':ang})
                
                # Finally, wait for convergence
                blocks.append({'type':FctBlockType.CONV.value,
                            'duration':steadyTime,
                            'value':steadyAng})
                
                # Reset the simulation
                logcode  =  f"if abs(X.getTrueHeading()) > {np.deg2rad(floatifyVar(self.targetErrorVar))}:\n"
                logcode += f"    print('THE HEADING DID NOT CONVERGE!')\n"
                logcode +=  "elif app != None:\n"
                logcode += f"    app.getPolarGenerator().plot({np.deg2rad(ang)}, X.getBoatSpeedNorm(), X.getBoatRoll(), X.getBoatPitch())\n"
                logcode += "raise ResetSimuException()"
                blocks.append({'type':FctBlockType.EXEC.value,
                            'duration':1,
                            'value':logcode})
                
            # Finally, finish
            blocks.append({'type':FctBlockType.END.value,
                            'duration':1,
                            'value':1})
            
            teditorSave['wang'] = {'gui':blocks}
            
            # Create the block for the wind (speed)
            teditorSave['wspeed'] = {'gui':[{'type':FctBlockType.CONSTANT.value,
                                            'duration':1,
                                            'value':wspeed}]}
            
            # Create the block for the target
            teditorSave['target']   = {'gui':[{'type':FctBlockType.CONSTANT.value,
                                            'duration':1,
                                            'value':0}]}
        
        else:
            """
            "Slope" strategy
            """
            print(f"Generating Testbench for SLOP STRATEGY")

            headings = np.linspace(0, 130, N)

            # First, wait for convergence at target = 0
            blocks.append({'type':FctBlockType.CONSTANT.value,
                            'duration':0.01,
                            'value':0})
                
            blocks.append({'type':FctBlockType.CONV.value,
                            'duration':steadyTime,
                            'value':steadyAng})
            
            # Then, use a slow slope
            tslope = 100
            for i in range(len(headings)):
                ang = headings[i]
                angPlot = 175 - ang
                blocks.append({'type':FctBlockType.SLOPE.value,
                            'duration':tslope/len(headings),
                            'value':ang})
                
                logcode  =  f"if abs(X.getTrueHeading()-{np.deg2rad(ang)}) > {np.deg2rad(floatifyVar(self.targetErrorVar))}:\n"
                logcode += f"    print('THE HEADING DID NOT CONVERGE!')\n"
                logcode +=  "elif app != None:\n"
                logcode += f"    app.getPolarGenerator().plot({np.deg2rad(angPlot)}, X.getBoatSpeedNorm(), X.getBoatRoll(), X.getBoatPitch())\n"
                blocks.append({'type':FctBlockType.EXEC.value,
                            'duration':1,
                            'value':logcode})
                
                blocks.append({'type':FctBlockType.CONSTANT.value,
                            'duration':0.01,
                            'value':ang})
                
            teditorSave['target'] = {'gui':blocks}
            
            # Create the block for the wind (speed)
            teditorSave['wspeed'] = {'gui':[{'type':FctBlockType.CONSTANT.value,
                                            'duration':1,
                                            'value':wspeed}]}
            
            # Create the block for the wind (angle)
            teditorSave['wang']   = {'gui':[{'type':FctBlockType.CONSTANT.value,
                                            'duration':1,
                                            'value':175}]}
        
        # Finally, load it
        self.app.getNavController().load({'editors':teditorSave})

    def generateAndRun(self):
        """
        Generate and run the polar speed analysis
        """
        self.generateTestbench()
        self.app.getBoatViewver().resetSimu()
        self.app.getBoatViewver().setLiveComputation(True)
        self.app.getNavController().setBeginFromCurrentTime(True)
        self.app.getNavController().sendTargetFct()
        self.app.getBoatViewver().getRunStopButton().set(True)
        self.newTrace(floatifyVar(self.windSpeedVar))

    def getSaveDic(self):
        """
        Return the save dic of this component
        """
        return {'windSpeed':self.windSpeedVar.get(),
                'nbHeading':self.nbHeadingVar.get(),
                'steadyTime':self.steadyTimeVar.get(),
                'steadyAng':self.steadyAngVar.get(),
                'strategy':self.strategyCb.get(),
                'targetError':self.targetErrorVar.get(),
                'traces':[pt.getSaveDic() for pt in self.polarTraces]}
    
    def getSetupDic(self):
        """
        Return the setup dic used to plot a trace
        """
        save = self.getSaveDic()
        save.pop('traces')
        return save
    
    def load(self, save):
        """
        Load a save dic
        """
        if ('windSpeed' in save):
            self.windSpeedVar.set(save['windSpeed'])
        
        if ('nbHeading' in save):
            self.nbHeadingVar.set(save['nbHeading'])

        if ('steadyTime' in save):
            self.steadyTimeVar.set(save['steadyTime'])

        if ('steadyAng' in save):
            self.steadyAngVar.set(save['steadyAng'])

        if ('targetError' in save):
            self.targetErrorVar.set(save['targetError'])

        if ('strategy' in save):
            self.strategyCb.set(save['strategy'])

        if ('traces' in save):
            for traceSave in save['traces']:
                self.newTrace().load(traceSave)
            # fit visible
            self.fitTrace()

    def getTraces(self):
        """
        Return the list of trace object
        """
        return self.polarTraces
    
    def getFigure(self):
        """
        Return the figure object
        """
        return self.fig

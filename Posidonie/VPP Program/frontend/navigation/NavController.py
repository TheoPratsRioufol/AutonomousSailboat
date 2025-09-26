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
from utils.HScrollPane import *
from config.Config import *
from backend.solver.Navigator import *
from frontend.navigation.FctBlock import *
from backend.solver.EDOSolver import *


def putT0InGoodUnit(T0):
    """
    Set T0 to a format with good units
    """
    for tname in T0:
        T0[tname] = T0Units[tname]['f'](T0[tname])
    return T0

class NavController(tk.Frame):

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        
        T0 = defaultT0.copy()
        self.editPanes = {} # dictionary that save target-editing pane


        """
                FIRST A MANUAL TUNING BOX
        """

        boxBorder = tk.Frame(self, background=COLOR_BOX_BORDER, padx=5, pady=5)
        box = tk.Frame(boxBorder, background=COLOR_BOX, padx=5, pady=5)
        
        tk.Label(box, text="Manual Override", background=COLOR_BOX, font=TITLE_FONT).pack()
        inPane = tk.Frame(box, background=COLOR_BOX)
        
        self.GUIVar = {}
        for tname in T0:
            self.GUIVar[tname] = tk.StringVar()
            ttk.Label(inPane, text=tname, background=COLOR_BOX).pack(side=tk.LEFT)
            ttk.Entry(inPane, textvariable=self.GUIVar[tname]).pack(side=tk.LEFT)
            #GUIVar[tname].trace_add('write', lambda x,y,z:entryUpdated())
        ttk.Button(inPane, text="Send", command=self.sendManualTargetFct).pack(side=tk.LEFT)

        inPane.pack(fill=tk.X, expand=True)
        box.pack(fill=tk.X, expand=True)
        boxBorder.pack(fill=tk.X, expand=False, padx=5, pady=5)

        """
                THEN, A FUNCTION EDITING BOX (FOR EACH PARAM OF THE TARGET)
        """

        boxBorder = tk.Frame(self, background=COLOR_BOX_BORDER, padx=5, pady=5)
        box = tk.Frame(boxBorder, background=COLOR_BOX, padx=5, pady=5)
        
        tk.Label(box, text="Mission Planner", background=COLOR_BOX, font=TITLE_FONT).pack()
        
        # Button bar
        butPane = tk.Frame(box, background=COLOR_BOX)
        tk.Label(butPane, text="Target function: ", background=COLOR_BOX).pack(side=tk.LEFT)
        targetFunctionNames = list(T0.keys())
        self.targetCb = ttk.Combobox(butPane, values=targetFunctionNames, state="readonly")
        self.targetCb.bind("<<ComboboxSelected>>", self.targetSelected)
        self.targetCb.pack(side=tk.LEFT)

        ttk.Button(butPane, text="Plot curve", command=self.drawCurve).pack(side=tk.LEFT)
        
        ttk.Button(butPane, text="Send", command=self.sendTargetFct).pack(side=tk.RIGHT)
        self.startFromCurrentTime = tk.IntVar()
        tk.Checkbutton(butPane, text="Start from current time", background=COLOR_BOX, variable=self.startFromCurrentTime).pack(side=tk.RIGHT)
        
        butPane.pack(fill=tk.X, expand=False)

        # Create a graph to visualise the target signal
        self.fig = plt.figure(figsize=(4, 2), dpi=100)
        self.ax = self.fig.add_subplot(1,1,1)
        self.ax.set_xlabel('Time(s)', fontsize = 10)
        self.ax.grid()
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=box)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw()

        # Tune pane (for the blocks)
        self.blockTunePane = tk.Frame(box, background=COLOR_BOX)
        
        # Then create the edits function panes
        for tname in T0:

            self.editPanes[tname] = {'i':0}
            self.editPanes[tname]['editPane'] = tk.Frame(box, background=COLOR_BOX)

            self.editPanes[tname]['scroll'] = HScrollPane(self.editPanes[tname]['editPane'], background=COLOR_BOX)
            self.editPanes[tname]['scroll'].pack(fill=tk.X, side=tk.LEFT, expand=True)
            self.editPanes[tname]['editContent'] = tk.Frame(self.editPanes[tname]['scroll'], background=COLOR_BOX)
            self.editPanes[tname]['scroll'].addContent(self.editPanes[tname]['editContent'])

            # add "Add" button
            ttk.Button(self.editPanes[tname]['editPane'], image=self.getApp().getAssets().get('add'), command=self.addBlock).pack(side=tk.RIGHT)

            # By default, add one block
            self.targetCb.set(tname)
            self.addBlock()

        # Pack the tune pane at the bottom
        self.blockTunePane.pack(fill=tk.X, expand=False)

        # Select the target for editing
        self.targetSelected()

        box.pack(fill=tk.BOTH, expand=True)
        boxBorder.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def addBlock(self, tname=None):
        """
        Add a block in the currently open (targetCb) trace editor if the tname is not specified
        """
        if (tname == None):
            tname = self.targetCb.get()

        self.editPanes[tname]['i'] += 1
        block = FctBlock(self.editPanes[tname]['editContent'], self, self.editPanes[tname]['i'])
        block.pack(side=tk.LEFT, padx=2)
        self.editPanes[tname]['editContent'].update()
        self.editPanes[tname]['scroll'].callConfigure()

        return block

    def targetSelected(self, event=None):
        """
        Used to edit the function associated to a target (open an editor)
        """
        tname = self.targetCb.get()

        # Forget the pack of all the editor
        for key in self.editPanes:
            self.editPanes[key]['editPane'].pack_forget()

        # Pack the correct one
        self.editPanes[tname]['editPane'].pack(fill=tk.X, expand=True)

        # set the last block selected
        self.setLastBlockSelected()
        # clear highlight
        self.targetCb.select_clear()
        # Update the curve
        self.drawCurve()

    def removeBlock(self, block):
        """
        Remove a block component
        """
        # Get the name of the current target parameter
        tname = self.targetCb.get()

        # It must last at least one
        if (len(self.editPanes[tname]['editContent'].winfo_children()) <= 1):
            return
        
        # Remove the block
        block.destroy()

        # Set the last block selected
        self.setLastBlockSelected()

        # Re-draw the curve
        self.drawCurve()

    def setLastBlockSelected(self):
        """
        Set the last block of the curent edited target function selected
        """
        # Get the name of the current target parameter
        tname = self.targetCb.get()
        # Set the last block selected
        self.editPanes[tname]['editContent'].winfo_children()[-1].attachTunePane()

    def getStrFunctionOfTarget(self, tname):
        """
        Return the string function of a target named tname
        """
        stepNb = 0
        fct   = "import numpy as np\n"
        fct  += "def signal(app, X, S, t):\n"
        tfinal = 0
        # Load all the function block
        for block in self.editPanes[tname]['editContent'].winfo_children():
            fct += f"    if S['step'] == {stepNb}:\n"
            fct += '\n        ' + '\n        '.join(block.getHeader()) + '\n'
            fct += f"        if {block.getFinishCondition()}:\n"
            fct += f"            S['step'] += 1\n"
            fct += f"            S['t0'] = t\n"
            fct += f"            S['y0'] = {block.getFinalValue()}\n"
            fct += f"        out = {block.getExpression()}\n"
            fct += '        ' + '\n        '.join(block.getFooter()) + '\n'
            tfinal += block.getDuration()
            stepNb += 1
            
        # Add the final block
        fct += f"    if S['step'] >= {stepNb}:\n"
        fct += f"        out = S['y0']\n"
        fct += f"    return out, S\n"

        return fct, tfinal

    def getTargetFunctionFromStr(fctStr):
        """
        Return a target function from a str definition
        """
        # Build the function
        exec(fctStr, globals())
        # Return the function
        return signal

    def drawCurve(self):
        """
        Draw the curve of the target over time function
        """
        # Get the name of the current target parameter
        tname = self.targetCb.get()

        # Get the function
        fctStr, tf = self.getStrFunctionOfTarget(tname)
        fct = NavController.getTargetFunctionFromStr(fctStr)
        time = np.linspace(0, tf, 200)

        # clear axis
        self.ax.cla()
        # plot the function
        y = []
        X = self.getApp().getBoat().getSolver().getX0()
        Sp = self.getApp().getBoat().getSolver().getNavigator().getS0()[tname]
        for t in time:
            try:
                T, Sp = fct(None, X, Sp, t)
                y.append(T)
            except SimuFinishedException:
                break
            except SimulatorException:
                pass

        self.ax.plot(time[:len(y)], y, color=COLOR_BOX_DARK)
        self.ax.grid()
        self.ax.set_ylabel(tname)
        self.ax.set_xlabel('Time (s)')
        # refresh
        self.canvas.draw()

    def sendManualTargetFct(self):
        """
        Send the manual target function to the navigator
        """
        T0 = {}
        for tname in self.GUIVar:
            value = self.GUIVar[tname].get()
            try:
                value = float(value)
            except:
                value = 1
            T0[tname] = value
        putT0InGoodUnit(T0)

        def manualTFct(X, S, t):
            return T0, S
        
        self.app.getBoat().getSolver().getNavigator().setTargetFct(manualTFct)
            
    def setBeginFromCurrentTime(self, state):
        """
        If state = true, the target function beggin from the current simulation date
        """
        self.startFromCurrentTime.set(state)

    def sendTargetFct(self):
        """
        Update the target function of the navigator by the one edited in tis editor
        """
        # Extract all the functions
        #with open('Tfct.py', 'w') as f:
        #    f.write(self.getStrFunctionOfTarget('wang')[0])

        Tpartials = {tname:lambda X, S, t, fct=NavController.getTargetFunctionFromStr(self.getStrFunctionOfTarget(tname)[0]): fct(self.app, X, S, t) for tname in self.editPanes}
        
        # If requiered, shift the time to start at the simu current time
        t0 = 0
        if (self.startFromCurrentTime.get() != 0):
            t0 = self.app.getBoat().getEDOSolver().getTime()

        def Tfct(X, S, t):
            T = {}
            for tname in Tpartials:
                try:
                    Tp, Sp = Tpartials[tname](X, S[tname], t-t0) 
                except SimulatorException as e:
                    raise e
                S[tname] = Sp
                T[tname] = T0Units[tname]['f'](Tp)
            return T, S

        # Reset the initial target state vector
        self.app.getBoat().getEDOSolver().resetS()
        self.app.getBoat().getSolver().getNavigator().setTargetFct(Tfct)
        print("Target function sent")

    def getApp(self):
        """
        Return the app object
        """
        return self.app
    
    def getTunePane(self):
        """
        Return the tune pane of the function editor
        """
        return self.blockTunePane
    
    def getSaveDic(self):
        """
        Return the save dictionnary of this object
        """
        save = {'editors':{}, 
                'selected':self.targetCb.get(), 
                'manual':{tname:self.GUIVar[tname].get() for tname in self.GUIVar},
                'startFromCurrentTime':self.startFromCurrentTime.get()}
        
        for tname in self.editPanes:
            fctStr, tf = self.getStrFunctionOfTarget(tname)
            save['editors'][tname] = {'gui':[], 'fct':{'str':fctStr, 'tf':tf}}
            for block in self.editPanes[tname]['editContent'].winfo_children():
                save['editors'][tname]['gui'].append(block.getSaveDic())
        
        return save
    
    def load(self, save):
        """
        Load a save dictionnary
        """
        if ('selected' in save):
            self.targetCb.set(save['selected'])

        if ('editors' in save):
            for tname in save['editors']:
                # First, clear the edit pane (if more than zero block to load)
                if (len(save['editors'][tname]) > 0):
                    for widget in self.editPanes[tname]['editContent'].winfo_children():
                        widget.destroy()
                # Then add all the saved blocks
                for blockSave in save['editors'][tname]['gui']:
                    # Create an empty block
                    block = self.addBlock(tname)
                    # load from the save
                    block.load(blockSave)
                
            # Then, set the selected one open (and redraw the curve)
            self.targetSelected()

        if ('manual' in save):
            for tname in save['manual']:
                self.GUIVar[tname].set(save['manual'][tname])

        if ('startFromCurrentTime' in save):
            self.startFromCurrentTime.set(save['startFromCurrentTime'])

        # And load the manual function into the solver
        self.sendManualTargetFct()

    def setAndRunManualWindSpeed(self, wspeed):
        """
        Update and run the manual wind speed
        """
        self.GUIVar['wspeed'].set(wspeed)
        self.sendManualTargetFct()

    def setAndRunManualWindAngle(self, wang):
        """
        Update and run the manual wind speed
        """
        self.GUIVar['wang'].set(wang)
        self.sendManualTargetFct()
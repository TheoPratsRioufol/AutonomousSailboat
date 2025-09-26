import tkinter as tk
from tkinter import ttk
from enum import Enum

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *
from backend.solver.Navigator import *

class FctBlockType(Enum):
    SLOPE    = "Slope"
    CONSTANT = "Constant"
    CONV     = "Convergence"
    CONVRP   = "R+P CV"
    END      = "End"
    MSG      = "Message"
    EXEC    = "Execute"

    def fromName(name):
        for e in FctBlockType:
            if e.value == name:
                return e
        raise Exception("Unknow block type: "+str(name))
    
BLOCK_SIZE = 40

class FctBlockTunePane(tk.Frame):
    def __init__(self, master, fctBlock):
        super().__init__(master, background=COLOR_BOX)
        self.fctBlock = fctBlock

    def destroy(self):
        super().destroy()
        # Try to notify the dad
        try:
            self.fctBlock.tunePaneDestroyed()
        except:
            pass

class FctBlock(tk.Frame):
    def __init__(self, master, navController, n):
        super().__init__(master, padx=5, pady=5, background=COLOR_BOX_BORDER)
        
        # navController:
        self.navController = navController
        
        # Type of this block:
        self.type = FctBlockType.CONSTANT

        self.pane = tk.Frame(self, background=COLOR_BOX)
        self.n = n

        # Create canvas
        self.canvas = tk.Canvas(self.pane, width=BLOCK_SIZE, height=BLOCK_SIZE, background=COLOR_BOX)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.attachTunePane)

        self.pane.pack(fill=tk.BOTH, expand=True)
        self.updateCanvas()

        # Parameter variable of this block
        self.durationVar = tk.StringVar(value=1)
        self.valueVar = tk.StringVar(value=0)
        
        # Open the editing pane
        self.attachTunePane()

    def updateCanvas(self):
        """
        Update the block visual according a type
        """
        #h, w = self.canvas.winfo_width(), self.canvas.winfo_height()
        w, h = BLOCK_SIZE, BLOCK_SIZE
        pad = 5
        uw = h-2*pad
        lw = 2
        lcolor = COLOR_BOX_DARK
        self.canvas.delete('all')
        if (self.type == FctBlockType.SLOPE):
            # SLOPE SYMBOL
            self.canvas.create_line(pad, h-pad, pad+uw/5, h-pad, width=lw, fill=lcolor)
            self.canvas.create_line(w-pad, pad, w-pad-uw/5, pad, width=lw, fill=lcolor)
            self.canvas.create_line(pad+uw/5, h-pad, w-pad-uw/5, pad, width=lw, fill=lcolor)
        elif (self.type == FctBlockType.CONSTANT):
            # CONSTANT SYMBOL
            self.canvas.create_line(pad, h//2, w-pad, h//2, width=lw, fill=lcolor)
        elif (self.type == FctBlockType.CONV) or (self.type == FctBlockType.CONVRP):
            # HOURGLASS SYMBOL
            self.canvas.create_line(2*pad, pad, w-2*pad, pad, width=lw, fill=lcolor)
            self.canvas.create_line(2*pad, h-pad, w-2*pad, h-pad, width=lw, fill=lcolor)
            self.canvas.create_line(2*pad, pad, w-2*pad, h-pad, width=lw, fill=lcolor)
            self.canvas.create_line(2*pad, h-pad, w-2*pad, pad, width=lw, fill=lcolor)
            if (self.type == FctBlockType.CONVRP):
                self.canvas.create_text(w//2, 2*h//3, text="RP", font=("consolas", 12, "bold"), fill="purple")
        elif (self.type == FctBlockType.MSG):
            # END SYMBOL
            self.canvas.create_text(w//2, 2*h//3, text="MSG", font=("consolas", 12, "bold"), fill=lcolor)
        elif (self.type == FctBlockType.EXEC):
            # EXEC SYMBOL
            self.canvas.create_text(w//2, 2*h//3, text="EXE", font=("consolas", 12, "bold"), fill="purple")
        else:
            # MSG SYMBOL
            self.canvas.create_text(w//2, 2*h//3, text="END", font=("consolas", 12, "bold"), fill="purple")

        self.canvas.create_text(pad, pad, text=self.n, anchor=tk.NW)

    def attachTunePane(self, event=None):
        """
        Attach the tune pane of this component
        """
        # change the color due to the focus
        self.config(background=COLOR_BOX_BORDER_FOCUSED)

        # First, clear the tune pane
        for widget in self.navController.getTunePane().winfo_children():
            widget.destroy()

        # Then attach the tune pane
        tunePane = FctBlockTunePane(self.navController.getTunePane(), self)

        ttk.Label(tunePane, font=TITLE_FONT, text=f"({self.n}) ", background=COLOR_BOX).pack(side=tk.LEFT)

        ttk.Label(tunePane, text="Type:", background=COLOR_BOX).pack(side=tk.LEFT)
        typeCb = ttk.Combobox(tunePane, values=[type.value for type in FctBlockType], state="readonly", width=8)
        typeCb.set(self.type.value)

        def typeCbSelected(event=None):
            self.type = FctBlockType.fromName(typeCb.get())
            self.updateCanvas()

            # Update the text of the entry
            if (typeCb.get() == FctBlockType.SLOPE.value):
                durationLabel.config(text="Slope time (s): ")
                valueLabel.config(text="Final value (s): ")
            
            if (typeCb.get() == FctBlockType.CONSTANT.value):
                durationLabel.config(text="Duration (s): ")
                valueLabel.config(text="Value (s): ")

            if (typeCb.get() == FctBlockType.CONV.value) or (typeCb.get() == FctBlockType.CONVRP.value):
                durationLabel.config(text="Minimum steady time (s): ")
                valueLabel.config(text="Angle Threashold (deg): ")

            if (typeCb.get() == FctBlockType.END.value):
                durationLabel.config(text="--")
                valueLabel.config(text="--")
                self.durationVar.set("1")
                
                # Disable entries
                valueEntry.config(state=tk.DISABLED)
                durationEntry.config(state=tk.DISABLED)

            elif (typeCb.get() == FctBlockType.MSG.value):
                durationLabel.config(text="--")
                valueLabel.config(text="Message:")
                
                # Disable/Enable entries
                valueEntry.config(state=tk.NORMAL)
                durationEntry.config(state=tk.DISABLED)

            elif (typeCb.get() == FctBlockType.EXEC.value):
                durationLabel.config(text="--")
                valueLabel.config(text="Code:")
                
                valueEntry.config(state=tk.NORMAL)
                durationEntry.config(state=tk.DISABLED)

            else:
                # Enable entries
                valueEntry.config(state=tk.NORMAL)
                durationEntry.config(state=tk.NORMAL)

            typeCb.selection_clear()
            self.GUIUpdated()
            
        typeCb.bind("<<ComboboxSelected>>", typeCbSelected)
        typeCb.pack(side=tk.LEFT)

        durationLabel = ttk.Label(tunePane, background=COLOR_BOX)
        durationLabel.pack(side=tk.LEFT)
        durationEntry = ttk.Entry(tunePane, textvariable=self.durationVar, width=8)
        durationEntry.pack(side=tk.LEFT)

        valueLabel = ttk.Label(tunePane, background=COLOR_BOX)
        valueLabel.pack(side=tk.LEFT)
        valueEntry = ttk.Entry(tunePane, textvariable=self.valueVar, width=8)
        valueEntry.pack(side=tk.LEFT)

        ttk.Button(tunePane, image=self.navController.getApp().getAssets().get('delete'), command=self.delete).pack(side=tk.RIGHT)

        tunePane.pack(fill=tk.X, expand=True)

        # Attach updated event
        self.durationVar.trace_add('write', lambda x,y,z: self.GUIUpdated())
        self.valueVar.trace_add('write', lambda x,y,z: self.GUIUpdated())

        # Update the selected type
        typeCbSelected()

    def tunePaneDestroyed(self):
        """
        Called when the tune pane is destroyed (lose focus)
        """
        self.config(background=COLOR_BOX_BORDER)

    def getExpression(self):
        """
        Return the trace vs time expression.
        This expression use a shifted time that start at t=S['t0'] and last <duration>
        This expression take as argument the previous value (S['y0']) (C0 transistion) of the target function
        """

        if (self.type == FctBlockType.SLOPE):
            # The SLOPE function
            return f"S['y0'] + (t - S['t0'])*({self.getFinalValue()} - S['y0'])/{self.getDuration()}"
        if (self.type == FctBlockType.CONSTANT):
            # The CONSTANT function
            return f"{self.getFinalValue()}"
        if (self.type == FctBlockType.CONV):
            # The CONV function
            return "S['y0']"
        
        # default: hold the value
        return "S['y0']"
        
    def getDuration(self):
        """
        Return the duration of this function
        """
        duration = floatifyVar(self.durationVar)
        if duration == 0:
            duration = 1
        return duration

    def getFinalValue(self):
        """
        Return the final value of this function
        """
        return floatifyVar(self.valueVar)
    
    def getHeader(self):
        """
        Return the target function header of this block
        """
        out = []
        if (self.type == FctBlockType.CONV):
            out.append(f"if (np.abs(X.getTrueHeading() - S['lastSteadyHeading']) > {np.deg2rad(self.getFinalValue())}):")
            out.append(f"   S['lastSteadyHeading'] = X.getTrueHeading()")
            out.append(f"   S['lastSteadyTime'] = t")
            out.append(f"if (np.linalg.norm(X.get('ang') - S['lastSteadyAng']) > {np.deg2rad(self.getFinalValue())}):")
            out.append(f"   S['lastSteadyAng'] = X.get('ang').copy()")
            out.append(f"   S['lastSteadyTime'] = t")

        if (self.type == FctBlockType.CONVRP):
            out.append(f"if (np.linalg.norm(X.get('ang')[:-1] - S['lastSteadyAng']) > {np.deg2rad(self.getFinalValue())}):")
            out.append(f"   S['lastSteadyAng'] = X.get('ang')[:-1].copy()")
            out.append(f"   S['lastSteadyTime'] = t")

        if (self.type == FctBlockType.END):
            out.append("raise SimuFinishedException()")
        
        if (self.type == FctBlockType.MSG):
            out.append(f"print('{self.valueVar.get()}')")

        return out
    
    def getFooter(self):
        out = []
        if (self.type == FctBlockType.EXEC):
            out = self.valueVar.get().split('\n')

        return out

    def getFinishCondition(self):
        """
        Return the condition to finish this block
        """
        if (self.type == FctBlockType.CONV) or (self.type == FctBlockType.CONVRP):
            return f"t - S['lastSteadyTime'] > {self.getDuration()}"
        elif (self.type == FctBlockType.MSG):
            return "True"
        elif (self.type == FctBlockType.END):
            return "True"
        elif (self.type == FctBlockType.EXEC):
            return "True"
        else:
            return f"t - S['t0'] > {self.getDuration()}"

    def GUIUpdated(self):
        """
        Call when the component is updated
        """
        #self.navController.drawCurve()

    def delete(self):
        self.navController.removeBlock(self)

    def getSaveDic(self):
        """
        Return the save dictionnary of this component
        """
        return {'type':self.type.value,
                'duration':self.durationVar.get(),
                'value':self.valueVar.get()}
    
    def load(self, save):
        """
        Load a save dictionnary
        """
        # Load the data
        if ('type' in save):
            self.type = FctBlockType.fromName(save['type'])
        if ('duration' in save):
            self.durationVar.set(save['duration'])
        if ('value' in save):
            self.valueVar.set(save['value'])
        # Update the visuals
        self.updateCanvas()
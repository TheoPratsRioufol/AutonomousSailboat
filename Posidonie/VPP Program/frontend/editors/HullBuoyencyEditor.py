import tkinter as tk
from tkinter import ttk
import threading
import queue
import time
import datetime
import os
import meshlib.mrmeshpy as mr
import meshlib as mm

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *
from backend.hull.HullBuoyencyCalculator import *
from frontend.navigation.FctBlock import FctBlockType

BOAT_ORIENTATIONS = ['+X','+Y','+Z','-X','-Y','-Z']
INERTIA_UNITS     = [Units.gmm2, Units.kgm2]

# Global var for progress bar
progressBar = None


class GenerateThread(threading.Thread):
    def __init__(self, hbe):
        super().__init__(daemon=True)
        self.progress = 0
        self.iskilled = False
        self.hbe = hbe
        self.hullMesh = HullMesh(hbe.getSaveDic())
        self.startTime = time.time()
        self.modelPath = hbe.getModelPath()
        
    def run(self):
        self.startTime = time.time()
        self.hullMesh.buildDisplacementFile(self.modelPath, self.setProgress, self.abortFct)
        
    def abortFct(self):
        return self.iskilled
    
    def setProgress(self, p, n):
        tr = 0 # Time reamining
        if p != 0:
            tr = (time.time() - self.startTime)*(1-p)/p
        self.hbe.getQueue().put((p, tr, n))
        
    def kill(self):
        self.iskilled = True



class HullBuoyencyEditor(tk.Frame):
    
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.generatorThread = None
        self.queue = queue.LifoQueue()
        mainPane = tk.Frame(self)

        boxBorder = tk.Frame(self, background=COLOR_BOX_BORDER, padx=5, pady=5)
        box = tk.Frame(boxBorder, background=COLOR_BOX, padx=5, pady=5)
        
        tk.Label(box, text="Hull Buoyency Model Generator", background=COLOR_BOX, font=TITLE_FONT).pack()
        inPane = tk.Frame(box, background=COLOR_BOX)
        
        """
        Parameter of the Hull Buoyency Model Generator:
            + STL File
            + Max Roll + Nb pts
            + Min, Max pitch + Nb pts
            + Nb pts z
            + longi direction (combobox)
            + top/bottom direction (combobox)
            + boat length
            + cdg
            - inertia matrix
            + mass of the hull
        """

        # Model input entry
        self.STLPath = tk.StringVar()
        self.ModelPath = tk.StringVar()
        ttk.Label(inPane, text="Model Path:", background=COLOR_BOX).grid(row=0, column=0, sticky=tk.NSEW)
        self.modelEntry = ttk.Entry(inPane, textvariable=self.ModelPath, state=tk.DISABLED)
        self.modelEntry.grid(row=0, column=1, columnspan=5, sticky=tk.NSEW)
        ttk.Button(inPane, image=self.app.getAssets().get('open'), command=self.openModel).grid(row=0, column=5, sticky=tk.NSEW)
        
        ttk.Label(inPane, text="STL File:", background=COLOR_BOX).grid(row=1, column=0, sticky=tk.NSEW)
        self.stlEntry = ttk.Entry(inPane, textvariable=self.STLPath, state=tk.DISABLED)
        self.stlEntry.grid(row=1, column=1, columnspan=5, sticky=tk.NSEW)
        ttk.Button(inPane, image=self.app.getAssets().get('open'), command=self.loadSTL).grid(row=1, column=5, sticky=tk.NSEW)

        # Model orientation entry
        ttk.Label(inPane, text="Back->Front direction:", background=COLOR_BOX).grid(row=2, column=0, sticky=tk.NSEW)
        self.longiCb = ttk.Combobox(inPane, values=BOAT_ORIENTATIONS, width=3, state="readonly")
        self.longiCb.set(BOAT_ORIENTATIONS[0])
        self.longiCb.grid(row=2, column=1, sticky=tk.NSEW)
        ttk.Label(inPane, text="Bottom->Top direction:", background=COLOR_BOX).grid(row=2, column=2, sticky=tk.NSEW)
        self.topBotCb = ttk.Combobox(inPane, values=BOAT_ORIENTATIONS, width=3, state="readonly")
        self.topBotCb.set(BOAT_ORIENTATIONS[0])
        self.topBotCb.grid(row=2, column=3, sticky=tk.NSEW)
        self.boatLength = tk.StringVar()
        ttk.Label(inPane, text="Boat length (m):", background=COLOR_BOX).grid(row=2, column=4, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.boatLength, width=4).grid(row=2, column=5, sticky=tk.NSEW)
        
        # Roll input entry
        self.maxRoll = tk.StringVar()
        self.NRoll = tk.StringVar()
        ttk.Label(inPane, text="Max Roll (deg):", background=COLOR_BOX).grid(row=3, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.maxRoll, width=4).grid(row=3, column=1, sticky=tk.NSEW)
        ttk.Label(inPane, text="Number of points:", background=COLOR_BOX).grid(row=3, column=4, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.NRoll, width=4).grid(row=3, column=5, sticky=tk.NSEW)
        
        # Pitch input entry
        self.maxPitch = tk.StringVar()
        self.minPitch = tk.StringVar()
        self.NPitch = tk.StringVar()
        ttk.Label(inPane, text="Max Pitch (deg):", background=COLOR_BOX).grid(row=4, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.maxPitch, width=4).grid(row=4, column=1, sticky=tk.NSEW)
        ttk.Label(inPane, text="Min Pitch (deg):", background=COLOR_BOX).grid(row=4, column=2, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.minPitch, width=4).grid(row=4, column=3, sticky=tk.NSEW)
        ttk.Label(inPane, text="Number of points:", background=COLOR_BOX).grid(row=4, column=4, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.NPitch, width=4).grid(row=4, column=5, sticky=tk.NSEW)
        
        # Z input entry
        self.NZ = tk.StringVar()
        ttk.Label(inPane, text="Nb. of points for Z:", background=COLOR_BOX).grid(row=5, column=4, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.NZ, width=4).grid(row=5, column=5, sticky=tk.NSEW)
       
        # Mass, cdg input entry
        self.mass = tk.StringVar()
        self.cdg = tk.StringVar(value='0,0,0')
        self.bvolume = tk.StringVar(value="{:.1f}L".format(self.app.getBoat().getSolver().getHull().getHullBuoyencyCalculator().getImergedVolume()*1e3))
        ttk.Label(inPane, text="Boat's mass:", background=COLOR_BOX).grid(row=6, column=0, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.mass, width=4).grid(row=6, column=1, sticky=tk.NSEW)
        ttk.Label(inPane, text="Boat's cdg (x,y,z):", background=COLOR_BOX).grid(row=6, column=2, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.cdg, width=5).grid(row=6, column=3, sticky=tk.NSEW)
        ttk.Label(inPane, text="Boat volume:", background=COLOR_BOX).grid(row=6, column=4, sticky=tk.NSEW)
        ttk.Entry(inPane, textvariable=self.bvolume, width=8, state=tk.DISABLED).grid(row=6, column=5, sticky=tk.NSEW)
        
        ttk.Button(inPane, text="Get CdG From STL", command=self.autoFillCdG).grid(row=7, column=0, sticky=tk.NSEW)

        for i in range(5):
            inPane.columnconfigure(i, weight=1)
        
        inPane.pack(fill=tk.X, expand=True)
        box.pack(fill=tk.X, expand=True)
        boxBorder.pack(fill=tk.X, expand=False, padx=5, pady=5)

        """
        Model generateor control panel
        """
        boxBorder = tk.Frame(self, background=COLOR_BOX_BORDER, padx=5, pady=5)
        box = tk.Frame(boxBorder, background=COLOR_BOX, padx=5, pady=5)
        
        tk.Label(box, text="Model", background=COLOR_BOX, font=TITLE_FONT).pack()
        inPane = tk.Frame(box, background=COLOR_BOX)

        global progressBar
        butPane = tk.Frame(inPane, background=COLOR_BOX)
        ttk.Button(butPane, text="Generate", command=self.generate).pack(side=tk.LEFT)
        ttk.Button(butPane, text="Abort", command=self.kill).pack(side=tk.LEFT)
        ttk.Button(butPane, text="Upload Model", command=self.refresh).pack(side=tk.LEFT)
        ttk.Button(butPane, text="Export tilted STL", command=self.exportSTLPositionned).pack(side=tk.LEFT)
        self.statusLabel = tk.Label(butPane, text='', background=COLOR_BOX)
        self.statusLabel.pack(side=tk.LEFT)
        butPane.pack(fill=tk.X)
        progressBar = ttk.Progressbar(inPane, mode='determinate')
        progressBar.pack(side=tk.TOP, fill=tk.X)

        inPane.pack(fill=tk.X, expand=True)
        box.pack(fill=tk.X, expand=True)
        boxBorder.pack(fill=tk.X, expand=False, padx=5, pady=5)

        """
        Inertia MAtrix Input
        """
        boxBorder = tk.Frame(self, background=COLOR_BOX_BORDER, padx=5, pady=5)
        box = tk.Frame(boxBorder, background=COLOR_BOX, padx=5, pady=5)
        
        tk.Label(box, text="Inertia Matrix", background=COLOR_BOX, font=TITLE_FONT).pack()
        
        formatPane = tk.Frame(box)
        ttk.Label(formatPane, text="Unit:", background=COLOR_BOX).pack(side=tk.LEFT)
        self.inertiaUnit = ttk.Combobox(formatPane, values=[e.value for e in INERTIA_UNITS], state="readonly")
        self.inertiaUnit.set(INERTIA_UNITS[0].value)
        self.inertiaUnit.pack(side=tk.LEFT)
        formatPane.pack()
                
        inPane = tk.Frame(box, background=COLOR_BOX)
        self.I = {}
        label = "xyz"
        for i in range(3):
            self.I[label[i]] = {}
            for j in range(3):
                self.I[label[i]][label[j]] = tk.StringVar()
                ttk.Label(inPane, text=f"I{label[i]}{label[j]}: ", background=COLOR_BOX).grid(row=i, column=2*j, sticky=tk.NSEW)
                ttk.Entry(inPane, textvariable=self.I[label[i]][label[j]], width=6).grid(row=i, column=2*j+1, sticky=tk.NSEW)
                

        inPane.pack(expand=True)

        # Equilibrium label
        eqPane = tk.Label(box, text="Equilibrium", font=TITLE_FONT, background=COLOR_BOX).pack(side=tk.TOP, fil=tk.X)
        # Equilibrium data
        eqPane = tk.Frame(box, background=COLOR_BOX)
        self.eqZ     = tk.StringVar(value='0')
        self.eqPitch = tk.StringVar(value='0')

        self.eqValid = TkValid(eqPane, app, background=COLOR_BOX)
        self.eqValid.set(False)
        self.eqValid.grid(row=0, column=0)
        ttk.Button(eqPane, text="Compute Eq.", command=self.computeEquilibrium).grid(row=0, column=1)
        tk.Label(eqPane, text="Eq. Water Line [m] (z):", background=COLOR_BOX).grid(row=0, column=2)
        tk.Entry(eqPane, textvariable=self.eqZ, width=10, state=tk.DISABLED).grid(row=0, column=3)
        tk.Label(eqPane, text="Eq. Pitch = 0.00 [deg]:", background=COLOR_BOX).grid(row=0, column=4)
        tk.Entry(eqPane, textvariable=self.eqPitch, width=10, state=tk.DISABLED).grid(row=0, column=5)
        
        eqPane.pack(side=tk.TOP, fill=tk.X)

        box.pack(fill=tk.X, expand=True)
        boxBorder.pack(fill=tk.X, expand=False, padx=5, pady=5)


        """
        Add a cdc viewver
        """
        boxBorder = tk.Frame(self, background=COLOR_BOX_BORDER, padx=5, pady=5)
        box = tk.Frame(boxBorder, background=COLOR_BOX, padx=5, pady=5)

        tk.Label(box, text="Cdc Viewver", font=TITLE_FONT, background=COLOR_BOX).pack(side=tk.TOP)

        controlPane = tk.Frame(box, background=COLOR_BOX)
        rollCtr = tk.StringVar(value="0")
        pitchCtr = tk.StringVar(value="0")
        yawCtr = tk.StringVar(value="0")
        zCtr = tk.StringVar(value="0")

        def setAttitude():
            """
            Set the boat attitude
            """
            X0 = self.app.getBoat().getSolver().getX0()
            X0._x_ang = np.array([np.deg2rad(floatifyVar(rollCtr)), np.deg2rad(floatifyVar(pitchCtr)), np.deg2rad(floatifyVar(yawCtr))])
            X0._x_pos = np.array([0, 0, floatifyVar(zCtr)])
            self.app.getBoatViewver().getRunStopButton().set(False)
            self.app.getBoat().getSolver().loadStateVector(X0)
            # update the forces
            self.app.getBoat().getSolver().compute()

        tk.Label(controlPane, text="roll =", background=COLOR_BOX).grid(row=0, column=1)
        tk.Entry(controlPane, textvariable=rollCtr, width=5).grid(row=0, column=2)
        tk.Label(controlPane, text="deg; pitch =", background=COLOR_BOX).grid(row=0, column=3)
        tk.Entry(controlPane, textvariable=pitchCtr, width=5).grid(row=0, column=4)
        tk.Label(controlPane, text="deg; yaw =", background=COLOR_BOX).grid(row=0, column=5)
        tk.Entry(controlPane, textvariable=yawCtr, width=5).grid(row=0, column=6)
        tk.Label(controlPane, text="deg; z =", background=COLOR_BOX).grid(row=0, column=7)
        tk.Entry(controlPane, textvariable=zCtr, width=5).grid(row=0, column=8)
        tk.Label(controlPane, text="m; ", background=COLOR_BOX).grid(row=0, column=9)
        ttk.Button(controlPane, text="Set Attitude", command=setAttitude).grid(row=0, column=10)
        controlPane.pack(side=tk.TOP, fill=tk.X)

        box.pack(fill=tk.X, expand=True)
        boxBorder.pack(fill=tk.X, expand=False, padx=5, pady=5)

        mainPane.pack(fill=tk.BOTH, expand=True)

    def getQueue(self):
        return self.queue

    def setProgress(self, p):
        """
        Set the progress to the progressbar (0 <= p <= 1)
        """
        global progressBar
        progressBar['value'] = 100*p

    def loadSTL(self):
        """
        Create a load STL dialog
        """
        sessionFile = tk.filedialog.askopenfile(filetypes =[("STL Files", '*.stl')],
                                                title="Open a hull STL file")
        if sessionFile != None:
            self.STLPath.set(sessionFile.name)
            self.stlEntry.icursor(tk.END)
            self.stlEntry.xview_moveto(1)

    def refresh(self):
        """
        Reload the model file
        """
        # Reload the file
        self.app.getBoat().getSolver().getHull().loadBuoyencyModel(self.getModelPath())
        # Update the geometry
        self.app.getGeomEditor().build()
        self.statusLabel.config(text="Model file successfully uploaded!")
        self.bvolume.set("{:.1f}L".format(self.app.getBoat().getSolver().getHull().getHullBuoyencyCalculator().getImergedVolume()*1e3))

    def generate(self):
        """
        Build the hull displacement model file
        """
        # Verify that we have a save location
        if self.ModelPath.get() == "":
            tk.messagebox.showerror("Model generation", "You mush specify a model path before generating & saving a model") 
            return
        
        self.statusLabel.config(text="Starting process...")
        self.update()
        self.generatorThread = GenerateThread(self)
        self.generatorThread.start()
        self.listenThread()
    
    def kill(self):
        """
        Kill the generating thread
        """
        self.statusLabel.config(text="Aborted")
        # Clear the queue
        self.queue = queue.LifoQueue()
        if (self.generatorThread == None):
            return
        self.generatorThread.kill()

    def listenThread(self):
        if (self.generatorThread == None):
            return
        if self.generatorThread.is_alive():
            self.after(1000, self.listenThread)
            try:
                data = self.queue.get(False)
                self.setProgress(data[0])
                txt = f"Number of points: {data[2]}, Time remaining: " + str(datetime.timedelta(seconds=int(data[1])))
                self.statusLabel.config(text=txt)
                self.update()
            except queue.Empty:
                pass
        else:
            self.setProgress(0)
            self.statusLabel.config(text="Computation done or aborted.")
            
    def getSaveDic(self):
        """
        Return the save dic of this object
        """
        return {'stlpath':self.STLPath.get(),
                'longiDir':self.longiCb.get(),
                'topBotDir':self.topBotCb.get(),
                'roll':{'max':self.getMaxRoll(),
                        'N':self.getNRoll()},
                'pitch':{'max':self.getMaxPitch(),
                         'min':self.getMinPitch(),
                         'N':self.getNPitch()},
                'Z':{'N':self.getNZ()},
                'mass':floatifyVar(self.mass),
                'length':floatifyVar(self.boatLength),
                'originalCdg':self.getCdg().tolist(),
                'originalCor':self.getCdg().tolist(),
                'inertia':self.getInertiaMatrix().tolist(),
                'inertiaUnit':self.inertiaUnit.get(),
                'modelpath':self.getModelPath(),
                'eqZ':self.eqZ.get(),
                'eqPitch':self.eqPitch.get(),
                'eqValid':self.eqValid.get()}
    
    def load(self, save):
        """
        Load a save dictionnary
        """
        self.STLPath.set(save['stlpath'])
        self.longiCb.set(save['longiDir'])
        self.topBotCb.set(save['topBotDir'])
        self.maxRoll.set(np.rad2deg(save['roll']['max']))
        self.NRoll.set(save['roll']['N'])
        self.maxPitch.set(np.rad2deg(save['pitch']['max']))
        self.minPitch.set(np.rad2deg(save['pitch']['min']))
        self.NPitch.set(save['pitch']['N'])
        self.NZ.set(save['Z']['N'])
        self.mass.set(save['mass'])
        self.boatLength.set(save['length'])
        self.cdg.set(','.join([str(e) for e in save['originalCdg']]))
        self.inertiaUnit.set(save['inertiaUnit'])
        self.setInertiaFromMatrix(np.array(save['inertia']), Units.fromName(save['inertiaUnit']))
        self.ModelPath.set(save['modelpath'])
        if ('eqZ' in save):
            self.eqZ.set(save['eqZ'])
        if ('eqPitch' in save):
            self.eqPitch.set(save['eqPitch'])
        if ('eqValid' in save):
            self.eqValid.set(save['eqValid'])

        # then update the model
        self.refresh()

    def setInertiaFromMatrix(self, matrix, unit):
        """
        Update the inertia entry from a matrix shape
        """
        for i in range(3):
            for j in range(3):
                self.I["xyz"[i]]["xyz"[j]].set(str(Units.toUnit(matrix[i,j], Units.kgm2, unit)))

    def loadModel(self, path):
        """
        Load a buoyency model at <path>
        """
        with open(path, 'r') as f:
            save = json.load(f)
        
        self.load(save['info'])

    def openModel(self):
        """
        Open a model file
        """
        sessionFile = tk.filedialog.asksaveasfilename(initialdir=PATH_BUOYENCY_MODEL_DIR,
                                                filetypes =[("Model Files", '*.'+BUOYENCY_MOD_FILETYPE)],
                                                title="Save a Buoyency Model file")
        
        if sessionFile != '':
            modelFileName = sessionFile
            if ('.' not in modelFileName):
                modelFileName = modelFileName + '.' + BUOYENCY_MOD_FILETYPE
            self.ModelPath.set(modelFileName)
            self.modelEntry.icursor(tk.END)
            self.modelEntry.xview_moveto(1)

            # Then load if possible
            if os.path.exists(modelFileName):
                self.loadModel(modelFileName)
        
        else:
            self.ModelPath.set('')



    def getSTLPath(self):
        """
        Return the stl path of the model
        """
        return self.STLPath.get()

    def getModelLength(self):
        """
        Return the boat length
        """
        return floatifyVar(self.boatLength)
    
    def getCenterOfRotation(self):
        """
        Return the center of rotation of the boat
        """
        return self.getCdg()
    
    def getCdg(self):
        """
        Return the position of the center of gravity
        """
        cdgStr = self.cdg.get()
        cdgSplit = cdgStr.split(',')
        if (len(cdgSplit) == 3):
            arr = []
            for i in cdgSplit:
                try:
                    arr.append(float(i))
                except:
                    raise Exception("Incorrect CDG input format (not float)")
            return np.array(arr)
        else:
            raise Exception("Incorrect CDG input format (comma)")
        
    def getInertiaMatrix(self):
        """
        Return the inertia matrix
        """
        mat = np.zeros((3,3))
        unit = Units.fromName(self.inertiaUnit.get())
        for i in self.I:
            for j in self.I[i]:
                iidx = "xyz".index(i)
                jidx = "xyz".index(j)
                mat[iidx, jidx] = Units.toSI(floatifyVar(self.I[i][j]), unit)
        return mat

    def getLongiDirection(self):
        """
        Return the boat longitudinal direction
        """
        return self.longiCb.get()

    def getTopBottomDirection(self):
        """
        Return the boat top/bottom direction
        """
        return self.topBotCb.get()
    
    def getModelPath(self):
        """
        Return the save path of the model
        """
        return self.ModelPath.get()
    
    def getMaxRoll(self):
        return np.deg2rad(floatifyVar(self.maxRoll))
    
    def getMaxPitch(self):
        return np.deg2rad(floatifyVar(self.maxPitch))
    
    def getMinPitch(self):
        return np.deg2rad(floatifyVar(self.minPitch))
    
    def getNRoll(self):
        return int(floatifyVar(self.NRoll))
    
    def getNPitch(self):
        return int(floatifyVar(self.NPitch))
    
    def getNZ(self):
        return int(floatifyVar(self.NZ))
    

    def generateTestbench(self):
        """
        Generate the equlibrium testbench
        """
        teditorSave = {}
        # Create the block for the wind (speed)
        teditorSave['wspeed'] = {'gui':[{'type':FctBlockType.CONSTANT.value,
                                            'duration':0.1,
                                            'value':0}]}
        
        # Create the block for the wind (ang)
        teditorSave['wspeed'] = {'gui':[{'type':FctBlockType.CONSTANT.value,
                                            'duration':0.1,
                                            'value':0}]}
            
        # Create the block for the target
        blocks = []
        blocks.append({'type':FctBlockType.CONSTANT.value,
                            'duration':0.1,
                            'value':90})
        
        blocks.append({'type':FctBlockType.CONVRP.value,
                            'duration':3,
                            'value':1})
        
        logcode  = "if app != None:\n"
        logcode += "    app.getHullBuoyencyEditor().setEquilibriumResult(X)"
        blocks.append({'type':FctBlockType.EXEC.value,
                            'duration':1,
                            'value':logcode})
        
        blocks.append({'type':FctBlockType.END.value,
                            'duration':0.1,
                            'value':0})
        
        teditorSave['target']  = {'gui':blocks}
        
        # Finally, load it
        self.app.getNavController().load({'editors':teditorSave})

    def generateAndRunEquilibrium(self):
        """
        Generate and run the equilibrium analysis
        """
        self.generateTestbench()
        self.app.getBoatViewver().setLiveComputation(True)
        self.app.getNavController().setBeginFromCurrentTime(True)
        self.app.getNavController().sendTargetFct()
        self.app.getBoatViewver().getRunStopButton().set(True)
    
    def computeEquilibrium(self):
        """
        Compute the equilibrium data (No wind, No speed)
        """
        self.generateAndRunEquilibrium()

    def setEquilibriumResult(self, X):
        self.eqZ.set(X._x_pos[2])
        self.eqPitch.set(np.rad2deg(X._x_ang[1]))
        self.setEqCalculusState(True)

    def setEqCalculusState(self, state):
        """
        Set the state of the calculus.
        If the geometry is edited, this state become invalid
        """
        self.eqValid.set(state)

    def getEqCalculusState(self):
        return self.eqValid.get()

    def getEqZ(self):
        return self.eqZ.get()
    
    def getEqPitch(self):
        return self.eqPitch.get()
    
    def exportSTLPositionned(self):
        """
        Export the stl file in the good orientation for verification
        """
        mesh = loadHull(self.getSTLPath(),
             self.getModelLength(),
             self.getCdg(),
             self.getCenterOfRotation(),
             self.getLongiDirection(),
             self.getTopBottomDirection())['mesh']
        mr.saveMesh(mesh,PATH_EXPORT_STL)
        print("[INFO] Exported STL file in the good orientation")

    def autoFillCdG(self):
        """
        Auto fill the postion of the CdG from the stl file
        """
        try:
            cdg = getCdgFromSTL(self.getSTLPath())
            cdg = "{:.2f}, {:.2f}, {:.2f}".format(*cdg)
            self.cdg.set(cdg)
        except Exception as e:
            print(e)
            print("[ERROR] Failed to found the CdG of the STL")
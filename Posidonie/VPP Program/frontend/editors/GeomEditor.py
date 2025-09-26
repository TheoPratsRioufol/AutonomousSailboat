import tkinter as tk
from tkinter import ttk
import copy

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *
import backend.sail.Sail as sailLib
import backend.drift.Drift as driftLib
import backend.rudder.Rudder as rudderLib
import backend.solver.Solver as solverLib
import backend.solver.Navigator as navigatorLib
import backend.Wind as windLib

defaultGeom = {'sail':sailLib.sailDefaultGeom,
               'drift':driftLib.driftDefaultGeom,
               'rudder':rudderLib.rudderDefaultGeom,
               'solver':solverLib.solverDefaultGeom,
               'wind':windLib.windDefaultGeom,
               'navigator':navigatorLib.navigatorDefaultGeom}

def fillMissingGeom(geom):
    """
    Fill the missing parameter of a geom dic
    """
    for comp in defaultGeom:
        if (comp not in geom):
            geom[comp] = copy.deepcopy(defaultGeom[comp])
        
        else:
            # check every sub item
            for elm in defaultGeom[comp]:
                if elm not in geom[comp]:
                    geom[comp][elm] = copy.deepcopy(defaultGeom[comp][elm])
    return geom

class GeomEditor(tk.Frame):
    
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app

        self.geom = defaultGeom

        mainPane = tk.Frame(self)

        butPane = tk.Frame(mainPane)
        ttk.Button(butPane, text='Build', command=self.build).pack(side=tk.LEFT)
        butPane.pack(side=tk.TOP, fill=tk.X)

        splitPane = tk.PanedWindow(mainPane, orient=tk.VERTICAL, sashwidth=8)

        # Add the geom parameter tree
        treePane = tk.Frame(splitPane, height=100)
        self.tree = ttk.Treeview(treePane, columns=("Value", 'Unit', 'Info'))
        self.tree.column("#0", width=150, stretch=False)
        self.tree.column("Value", width=80, stretch=False)
        self.tree.column("Unit", width=40, stretch=False)
        self.tree.column("Info")

        self.tree.heading("#0", text="Attribute")
        self.tree.heading("Value", text="Value")
        self.tree.heading("Unit", text="Unit")
        self.tree.heading("Info", text="Info")

        self.tree.bind("<Double-1>", self.onDoubleClick)

        self.tree.pack(fill=tk.BOTH, expand=True)

        # Add scroll bar to the tree
        sbTree = ttk.Scrollbar(self.tree, orient="vertical", command=self.tree.yview)
        sbTree.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.configure(yscrollcommand=sbTree.set)

        # Add the derivated quantity tree
        dqtreePane = tk.Frame(splitPane)
        self.dqtree = ttk.Treeview(dqtreePane, columns=("Value", 'Unit'))
        self.dqtree.column("#0", width=150, stretch=False)
        self.dqtree.column("Value", width=80, stretch=False)
        self.dqtree.column("Unit", width=40, stretch=False)

        self.dqtree.heading("#0", text="Derivated Quantity")
        self.dqtree.heading("Value", text="Value")
        self.dqtree.heading("Unit", text="Unit")

        self.dqtree.tag_configure("bold", font=TITLE_FONT)

        self.dqtree.pack(fill=tk.BOTH, expand=True)

        # Add scroll bar to the dqtree
        sbTree = ttk.Scrollbar(self.dqtree, orient="vertical", command=self.dqtree.yview)
        sbTree.pack(side=tk.RIGHT, fill=tk.Y)
        self.dqtree.configure(yscrollcommand=sbTree.set)

        # Pack the two tree
        splitPane.add(dqtreePane)
        splitPane.add(treePane)

        splitPane.pack(fill=tk.BOTH, expand=True)
        mainPane.pack(fill=tk.BOTH, expand=True)

        splitPane.sash_place(0, 200, 200)

        # Finally, build the tree
        self.buildTree()


    def onDoubleClick(self, event):
        selection = self.tree.selection()
        if (len(selection) == 1):
            if (len(self.tree.item(selection[0])['values']) > 0):
                value = self.tree.item(selection[0])['values'][0]
                self.app.editPopup(lambda x:self.edit(selection[0], x), value)

    def edit(self, leaf, newValue):
        """
        Edit the value of a leaf by using a popup window
        """
        # Load the old values attribute
        values = self.tree.item(leaf)['values']
        # Modify it
        values[0] = newValue
        # Actualize
        self.tree.item(leaf, values=values, image=self.app.getAssets().get('modified'))

        # update the boat
        self.build()

    def build(self, fromInit=False):
        """
        Update the geometry of the boat with the one of the editor
        """
        geom = self.buildGeomDic()
        self.app.getBoat().getSolver().updateGlobalGeom(geom)
        stabilityEditor = self.app.getStabilityEditor()
        if (stabilityEditor != None):
            stabilityEditor.redraw()
        # Then reset the state vector of the simulation
        X0continuous = self.app.getBoat().getSolver().getX0(self.app.getBoat().getSolver().getState())
        X0 = self.app.getBoat().getSolver().getX0()
        self.app.getBoat().getEDOSolver().setX0(X0)
        self.app.getBoat().getEDOSolver().setX(X0continuous)

        # Set the state of the eq calculus to invalid
        if not fromInit:
            self.app.getHullBuoyencyEditor().setEqCalculusState(False)

        # update the derivated quantity tree
        self.buildDerivatedQuantityTree()

        print("The GEOMEDITOR stamp is:", self.getGeomStamp())
        

    def buildGeomDic(self, setSaved=False):
        """
        Build the geommetry dictionnary
        """
        def buildSubTree(dad, geom):
            for child in self.tree.get_children(dad):
                data = self.tree.item(child)
                if (len(self.tree.get_children(child)) == 0):
                    # terminal node
                    if (len(data['values']) > 0):
                        try:
                            value = float(data['values'][0])
                        except:
                            value = data['values'][0]
                        geom[data['text']] = {'value':value,
                                              'unit':Units.fromName(data['values'][1]),
                                              'info':data['values'][2]}
                        if setSaved:
                            self.tree.item(child, image=self.app.getAssets().get('default'))
                else:
                    # build the subtree
                    geom[data['text']] = {}
                    buildSubTree(child, geom[data['text']])

        geom = {}
        buildSubTree('', geom)
        return geom
    
    def buildDerivatedQuantityTree(self):
        """
        Build the derivated quantity tree
        """
        # First, clear the tree
        self.dqtree.delete(*self.dqtree.get_children())

        solver = self.app.getBoat().getSolver()

        # Then add the infos

        # MASS
        massItem = self.dqtree.insert('', tk.END, text='Masses', open=True, image=self.app.getAssets().get('mass'))
        mdrift = solver.getDrift().getWeight()
        msail = solver.getSail().getWeight()
        mhull = solver.getHull().getWeight()
        mrudder = solver.getRudder().getWeight()
        mtot = mdrift + msail + mhull + mrudder
        vmax = solver.getHull().getHullBuoyencyCalculator().getImergedVolume()
        self.dqtree.insert(massItem, tk.END, text='drift', values=("{:.2f}".format(mdrift), Units.kg.value))
        self.dqtree.insert(massItem, tk.END, text='sail', values=("{:.2f}".format(msail), Units.kg.value))
        self.dqtree.insert(massItem, tk.END, text='hull', values=("{:.2f}".format(mhull), Units.kg.value))
        self.dqtree.insert(massItem, tk.END, text='rudder', values=("{:.2f}".format(mrudder), Units.kg.value))
        self.dqtree.insert(massItem, tk.END, text='Total', values=("{:.2f}".format(mtot), Units.kg.value), tag=("bold",))
        self.dqtree.insert(massItem, tk.END, text='M/µV', values=("{:.2f}".format(100*mtot/(vmax*PHY_RHO_SWATER)), Units.percent.value), tag=("bold",))

        # SURFACE
        surfaceItem = self.dqtree.insert('', tk.END, text='Surfaces', open=True, image=self.app.getAssets().get('surface'))
        sdrift = solver.getDrift().getSurface()
        sdriftA = solver.getDrift().getPlateASurface()
        sdriftB = solver.getDrift().getPlateASurface()
        ssail = solver.getSail().getSurface()
        srudder = solver.getRudder().getMovingSurface()
        self.dqtree.insert(surfaceItem, tk.END, text='drift (A)', values=("{:.2f}".format(sdriftA), Units.m2.value))
        self.dqtree.insert(surfaceItem, tk.END, text='drift (B)', values=("{:.2f}".format(sdriftB), Units.m2.value))
        self.dqtree.insert(surfaceItem, tk.END, text='drift', values=("{:.2f}".format(sdrift), Units.m2.value))
        self.dqtree.insert(surfaceItem, tk.END, text='sail', values=("{:.2f}".format(ssail), Units.m2.value))
        self.dqtree.insert(surfaceItem, tk.END, text='rudder (Moving)', values=("{:.2f}".format(srudder), Units.m2.value))
        self.dqtree.insert(surfaceItem, tk.END, text='drift/sail', values=("{:.2f}".format(100*sdrift/ssail), Units.percent.value), tag=("bold",))
        self.dqtree.insert(surfaceItem, tk.END, text='rudder/sail', values=("{:.2f}".format(100*srudder/ssail), Units.percent.value))
        self.dqtree.insert(surfaceItem, tk.END, text='A.R Sail', values=("{:.2f}".format(100*solver.getSail().getAspectRatio()), Units.percent.value))
        
        # POSITIONS
        posItem = self.dqtree.insert('', tk.END, text='Positions', open=True, image=self.app.getAssets().get('measure'))
        self.dqtree.insert(posItem, tk.END, text='z Boat CdG', values=("{:.2f}".format(solver.getBoatCdg().valueIn(Referential.BOAT)[2]), Units.m.value), tag=("bold",))
        
    
    def setGeom(self, geom):
        """
        Set the current geometry to geom
        """
        self.geom = copy.deepcopy(geom)
    
    def getSaveDic(self):
        """
        Return the save dic
        """
        return {'geom': serializeGeom(self.buildGeomDic())}
    
    def load(self, save):
        """
        Load a save dic
        """
        if ('geom' in save):
            geom = deserializeGeom(save['geom'])
            # check if some geometry are missing
            geom = fillMissingGeom(geom)
            self.setGeom(geom)
            self.buildTree()

            # update the boat
            self.build()

    def reset(self):
        self.tree.delete(*self.tree.get_children())

    def buildTree(self):
        self.reset()

        def buildSubTree(dad, subtree):
            for key in subtree:
                if (type(subtree[key]) == dict) and ('unit' not in subtree[key]):
                    newdad = self.tree.insert(dad, tk.END, text=key, open=True,
                                              image=self.app.getAssets().get('sailboat'))
                    buildSubTree(newdad, subtree[key])
                else:
                    self.tree.insert(dad, tk.END, 
                                    image=self.app.getAssets().get('default'),
                                    text=key, values=(subtree[key]['value'], subtree[key]['unit'].value, subtree[key]['info']))

        buildSubTree('', self.geom)

        # build the derivated quantity tree
        self.buildDerivatedQuantityTree()


    def getGeomStamp(self):
        """
        Return an identifier of the geometry
        """
        return hashFct(str(self.buildGeomDic()))
            
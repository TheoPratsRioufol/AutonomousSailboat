import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import os
import traceback

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from frontend.simu.Boat import *
from frontend.simu.BoatViewver import *
from frontend.editors.GeomEditor import *
from frontend.editors.HullBuoyencyEditor import *
from frontend.editors.NACAEditor import *
from backend.solver.Solver import *
from frontend.navigation.NavController import *
from frontend.app.ViewverWindow import *
from frontend.polar.PolarGenerator import *
from frontend.report.ReportGenerator import *
from frontend.mxPlot.MxPlot import *
from utils.Assets import *
from config.Config import *
from config.DynamicConfig import *

if __name__ == "__main__":
    import frontend.app.Main


class Splash(tk.Toplevel):
    def __init__(self, parent):
        tk.Toplevel.__init__(self, parent)
        self.title("Loading")
        self.geometry(f"{SPLASH_WIN_W}x{SPLASH_WIN_H}+100+100")

        pane = tk.Frame(self, background=COLOR_BG_VIEWVER_TOP)
        
        # Add image
        tk.Label(pane, image=parent.getAssets().get('splash')).pack(fill=tk.BOTH, expand=True)
        #tk.Label(pane, text="Loading...").pack(fill=tk.BOTH, expand=True)

        pane.pack(fill=tk.BOTH, expand=True)
        self.grab_set()

        self.attributes('-topmost', True)
        self.overrideredirect(True)

        self.update()


class App(tk.Tk):
    def __init__(self): 
        super().__init__()

        # Hide during loading
        self.withdraw()

        # Generate the assets
        self.assets = Assets()
        self.wm_iconphoto(True, self.getAssets().get(APP_LOGO))

        # Display splash screen
        splash = Splash(self)

        self.protocol("WM_DELETE_WINDOW", self.quit_me)
        self.title(MAIN_WINDOW_TITLE)

        # for the popup
        self.popup = None

        self.setStyle()

        self.geometry(f"{APP_WIDTH}x{APP_HEIGHT}+50+50")

        self.boat = Boat()
        self.editedFilePath = None
        self.dynamicConfig = DynamicConfig()

        # Define the viewver
        self.viewver = ViewverWindow(self)
        #self.viewver.display()

        noteBook = ttk.Notebook(self)
        
        self.geomEditor = GeomEditor(noteBook, self)
        self.navController = NavController(noteBook, self)
        self.polarGenerator = PolarGenerator(noteBook, self)
        self.hullBuoyencyEditor = HullBuoyencyEditor(noteBook, self)
        self.NACAEditor = NACAEditor(noteBook, self)
        self.mxPlot = MxPlot(noteBook, self)
        
        noteBook.add(self.geomEditor, text="Geometry Editor", image=self.getAssets().get('measure'), compound=tk.LEFT)
        noteBook.add(self.navController, text="Target Controller", image=self.getAssets().get('target'), compound=tk.LEFT)
        noteBook.add(self.polarGenerator, text="Polar plot", image=self.getAssets().get('polar'), compound=tk.LEFT)
        noteBook.add(self.hullBuoyencyEditor, text="Hull Buoyency Editor", image=self.getAssets().get('hull'), compound=tk.LEFT)
        noteBook.add(self.NACAEditor, text="NACA Editor", image=self.getAssets().get('NACA'), compound=tk.LEFT)
        noteBook.add(self.mxPlot, text="Mx Plot", image=self.getAssets().get('Mx'), compound=tk.LEFT)
        
        noteBook.pack(fill=tk.BOTH, expand=True)
        
        self.buildMenu()
        

        # Then load the previous session
        self.loadFile(self.dynamicConfig.get('lastFile'))
        self.deiconify()

        self.viewver.display(55 + APP_WIDTH, 50)

        # Load the hull model file
        buoyencyModelPath = self.getHullBuoyencyEditor().getModelPath()
        if buoyencyModelPath != None:
            self.getBoat().getSolver().getHull().loadBuoyencyModel(buoyencyModelPath)
        else:
            print("[ERROR] - No buyency model found")

        self.geomEditor.build(fromInit=True)

        # close the splash screen
        splash.destroy()

        self.reportGenerator = ReportGenerator(self)
        #self.generateReport()
        #self.quit_me()

        # Blocking
        self.viewver.run()

    def getMxPlot(self):
        return self.mxPlot
    
    def getPolarGenerator(self):
        return self.polarGenerator

    def getStabilityEditor(self):
        """
        Return the Stability Editor object
        """
        return self.viewver.getStabilityEditor()
    
    def getHullBuoyencyEditor(self):
        """
        Return the Hull Buoyency Editor object
        """
        return self.hullBuoyencyEditor
        
    def getBoatViewver(self):
        """
        Return the boat viewver object
        """
        return self.viewver.getBoatViewver()
    
    def getGeomEditor(self):
        """
        Return the geometry editor
        """
        return self.geomEditor
    
    def getNavController(self):
        """
        Return the frontend navController object
        """
        return self.navController
    
    def getNACAEditor(self):
        """
        Return the NACAEditor object
        """
        return self.NACAEditor

    def getBoat(self):
        """
        Return the backend boat object
        """
        return self.boat
    
    def getAssets(self):
        """
        Return the asset object
        """
        return self.assets
    
    def getSaveDic(self):
        """
        Return the save dictionnary of the session
        """
        saveDic = {}
        saveDic['geomEditor'] = self.geomEditor.getSaveDic()
        saveDic['navController'] = self.navController.getSaveDic()
        saveDic['polarGenerator'] = self.polarGenerator.getSaveDic()
        saveDic['hullBuoyencyEditor'] = self.hullBuoyencyEditor.getSaveDic()
        saveDic['mxPlot'] = self.mxPlot.getSaveDic()
        saveDic['reportGenerator'] = self.reportGenerator.getSaveDic()
        return saveDic

    def load(self, save):
        """
        Load a session from a dic
        """
        if ('geomEditor' in save):
            self.geomEditor.load(save['geomEditor'])
        if ('navController' in save):
            self.navController.load(save['navController'])
        if ('polarGenerator' in save):
            self.polarGenerator.load(save['polarGenerator'])
        if ('hullBuoyencyEditor' in save):
            self.hullBuoyencyEditor.load(save['hullBuoyencyEditor'])
        if ('mxPlot' in save):
            self.mxPlot.load(save['mxPlot'])
        if ('reportGenerator' in save):
            self.reportGenerator.load(save['reportGenerator'])
    
    def saveFile(self, path=None):
        if (path == None):
            path = self.getFileEdited()
            if (path == None):
                self.saveFileAs()
                return
        
        with open(path, 'w') as f:
            f.write(json.dumps(self.getSaveDic(), indent=4))

        print("[INFO] - Posidonie file saved!")
        self.setFileEdited(path)

    def loadFileAs(self):
        """
        Load a file at a location
        """
        sessionFile = tk.filedialog.askopenfile(filetypes =[("Posidonie Files", '*.json')],
                                                title="Open a Posidonie Session file")
        if sessionFile != None:
            self.loadFile(sessionFile.name)

    def saveFileAs(self):
        """
        Save a file at a location
        """
        f = tk.filedialog.asksaveasfile(mode='w', defaultextension=".json", title="Save a Posidonie Session file")
        if f != None:
            self.saveFile(f.name)
            
    def loadFile(self, path):
        try:
            with open(path, 'r') as f:
                self.load(json.load(f))
            print("[INFO] Successfully loaded from",path)
            self.setFileEdited(path)
        except Exception as e:
            traceback.print_exc()
            print("[ERROR] Impossible to load from",path)

    def buildMenu(self):
        self.menubar = tk.Menu(self)

        file_menu = tk.Menu(self.menubar, tearoff=False)
        file_menu.add_command(label='Save',command=self.saveFile)
        file_menu.add_command(label='Save As',command=self.saveFileAs)
        #file_menu.add_command(label='Load',command=self.loadFile)
        file_menu.add_command(label='Open...',command=self.loadFileAs)

        report_menu = tk.Menu(self.menubar, tearoff=False)
        report_menu.add_command(label='Generate',command=self.generateReport)
        report_menu.add_command(label='View',command=self.viewReport)

        self.menubar.add_cascade(label="File", menu=file_menu)
        self.menubar.add_cascade(label="Report", menu=report_menu)
        self.config(menu=self.menubar)

    def quit_me(self):
        """
        Close properly the application
        """
        self.getHullBuoyencyEditor().kill()
        self.kill()

    def kill(self):
        """
        Kill the application
        """
        self.quit()
        self.destroy()

    def editPopup(self, saveFct, default=""):
        # destroy previous if exit
        if (self.popup != None):
            self.popup.destroy()
        
        # create a new popup
        self.popup = tk.Toplevel(self)
        self.popup.title("Edit...")
        self.popup.focus_force()
        x, y = self.popup.winfo_pointerxy()
        self.popup.geometry(f"300x40+{x}+{y}")

        def close():
            self.popup.destroy()
            self.popup.update()

        def saveAndClose():
            saveFct(entry.get())
            close()

        ttk.Button(self.popup, image=self.getAssets().get('validate'), command=saveAndClose).pack(side=tk.LEFT)
        entry = ttk.Entry(self.popup)
        entry.pack(side=tk.LEFT)
        entry.insert(tk.END, default)
        entry.selection_range(0, tk.END)
        entry.bind('<Return>', lambda x:saveAndClose())
        entry.focus_force()

    def generateReport(self):
        """
        Generate a report of the boat sizing
        """
        self.reportGenerator.generate()

    def viewReport(self):
        """
        Display the report
        """
        os.startfile(self.reportGenerator.getPDFPath() + '.pdf')

    def setFileEdited(self, path):
        """
        Save the path of the file currently edited
        """
        self.editedFilePath = path
        self.title(MAIN_WINDOW_TITLE + " - " + self.editedFilePath)
        self.dynamicConfig.set('lastFile', self.editedFilePath)

    def getFileEdited(self):
        """
        Return the path of the file currently edited
        """
        return self.editedFilePath
    
    def setStyle(self):
        """
        Set the style attribute
        """
        style = ttk.Style()
        style.map("Treeview", 
                background=[("selected", SELECTING_COLOR)],
                foreground=[("selected", "black")])
        style.configure("Treeview.Heading", font=TITLE_FONT)

    def getGeomStamp(self):
        """
        Return an identifier of the geometry
        """
        return self.geomEditor.getGeomStamp()
import tkinter as tk
from tkinter import ttk
import time
from enum import Enum
import threading
import json
import traceback

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *
from backend.solver.Run import *
from frontend.simu.PlotViewver import *
from backend.solver.EDOSolver import *
from frontend.editors.StabilityEditor import *

class Camera():
    def __init__(self):
        self.rotationMatrix = None
        self.reset()

    def reset(self):
        self.pos = np.zeros(3)
        self.setAngle([np.deg2rad(-20), np.deg2rad(15), np.deg2rad(-25)])
        self.zoom = 100

    def setAngle(self, ang):
        self._ang = ang.copy()
        # Then update rotation matrix
        self.rotationMatrix = getMatRot(Dir.Z, self._ang[2])
        self.rotationMatrix = np.matmul(self.rotationMatrix, getMatRot(Dir.Y, self._ang[1]))
        self.rotationMatrix = np.matmul(self.rotationMatrix, getMatRot(Dir.X, self._ang[0]))

        self.invRotationMatrix = np.linalg.inv(self.rotationMatrix)

    def getPos(self):
        return self.pos

    def getAng(self):
        return self._ang

    def getRotationMatrix(self):
        return self.rotationMatrix
    
    def getInvRotationMatrix(self):
        return self.invRotationMatrix

# X Y Z -> roll, pitch, yaw
TOPVIEW   = np.array([-np.pi/2, 0, 0])
FRONTVIEW = np.array([0, 0, np.pi/2])
SIDEVIEW  = np.array([0.0, 0, 0])


class BoatViewver(tk.Frame):

    def __init__(self, master, app):
        super().__init__(master)
        self.pack_propagate(False)
        self.config(width=2*APP_WIDTH/3)
        self.plotViewver = None

        self.app = app
        self.camera = Camera()
        self.resetMinMaxProj()
        self.wirePolygons = tk.IntVar(value=1)
        self.liveComputation = tk.IntVar(value=1)
        self.filledPolygons = tk.IntVar()
        self.lastFrameTime = time.time()
        self.results = Run()
        self.timeSlider = None
        self.dtsim = 0
        self.dtLabel = None

        # Add a tool bar
        toolBar = tk.Frame(self)
        ttk.Button(toolBar, image=self.app.getAssets().get("fit-camera"), command=self.fitCamera).pack(side=tk.LEFT)
        ttk.Button(toolBar, image=self.app.getAssets().get("topview-camera"), command=self.setCameraToTopView).pack(side=tk.LEFT)
        ttk.Button(toolBar, image=self.app.getAssets().get("frontview-camera"), command=self.setCameraToFrontView).pack(side=tk.LEFT)
        ttk.Button(toolBar, image=self.app.getAssets().get("sideview-camera"), command=self.setCameraToSideView).pack(side=tk.LEFT)
        
        self.runStopButton = RunStopButton(toolBar, app, command=self.runStop)
        self.runStopButton.pack(side=tk.LEFT)

        ttk.Button(toolBar, image=self.app.getAssets().get("reset"), command=self.resetSimu).pack(side=tk.LEFT)
        ttk.Button(toolBar, image=self.app.getAssets().get("previous"), command=self.previousStep).pack(side=tk.LEFT)
        ttk.Button(toolBar, image=self.app.getAssets().get("next"), command=self.nextStep).pack(side=tk.LEFT)
        ttk.Button(toolBar, image=self.app.getAssets().get("slowdown"), command=self.slowdownSimu).pack(side=tk.LEFT)
        ttk.Button(toolBar, image=self.app.getAssets().get("speedup"), command=self.speedUpSimu).pack(side=tk.LEFT)
        
        ttk.Checkbutton(toolBar, text='LiveComputation', variable=self.liveComputation, command=self.simuSourceChanged).pack(side=tk.LEFT)
        ttk.Checkbutton(toolBar, text='fill', variable=self.filledPolygons).pack(side=tk.LEFT)
        ttk.Checkbutton(toolBar, text='wire', variable=self.wirePolygons).pack(side=tk.LEFT)
        
        self.dtLabel = tk.Label(toolBar, text='')
        self.dtLabel.pack(side=tk.RIGHT)
        ttk.Button(toolBar, image=self.app.getAssets().get("w25"), command=lambda:self.app.getNavController().setAndRunManualWindSpeed(25)).pack(side=tk.RIGHT)
        ttk.Button(toolBar, image=self.app.getAssets().get("w20"), command=lambda:self.app.getNavController().setAndRunManualWindSpeed(20)).pack(side=tk.RIGHT)
        ttk.Button(toolBar, image=self.app.getAssets().get("w15"), command=lambda:self.app.getNavController().setAndRunManualWindSpeed(15)).pack(side=tk.RIGHT)
        ttk.Button(toolBar, image=self.app.getAssets().get("w10"), command=lambda:self.app.getNavController().setAndRunManualWindSpeed(10)).pack(side=tk.RIGHT)
        ttk.Button(toolBar, image=self.app.getAssets().get("w5"), command=lambda:self.app.getNavController().setAndRunManualWindSpeed(5)).pack(side=tk.RIGHT)
        
        ttk.Button(toolBar, image=self.app.getAssets().get("135"), command=lambda:self.app.getNavController().setAndRunManualWindAngle(135)).pack(side=tk.RIGHT)
        ttk.Button(toolBar, image=self.app.getAssets().get("90"), command=lambda:self.app.getNavController().setAndRunManualWindAngle(90)).pack(side=tk.RIGHT)
        ttk.Button(toolBar, image=self.app.getAssets().get("45"), command=lambda:self.app.getNavController().setAndRunManualWindAngle(45)).pack(side=tk.RIGHT)
        ttk.Button(toolBar, image=self.app.getAssets().get("35"), command=lambda:self.app.getNavController().setAndRunManualWindAngle(35)).pack(side=tk.RIGHT)
        ttk.Separator(toolBar, orient=tk.VERTICAL).pack(side=tk.RIGHT)
        toolBar.pack(side=tk.TOP, fill=tk.X, expand=False)

        self.tabbedPane = ttk.Notebook(self)

        # Add the canvas (display window)
        self.canvas = tk.Canvas(self.tabbedPane)
        self.canvas.configure(bg=COLOR_BG_BOAT_VIEWVER)
        #self.canvas.pack(fill="both", expand=True)
        self.tabbedPane.add(self.canvas, text="3D Renderer", image=self.app.getAssets().get('3d'), compound=tk.LEFT)

        # Add a plot viewver
        self.plotViewver = PlotViewver(self.tabbedPane, self.results)
        self.tabbedPane.add(self.plotViewver, text="Plot Viewvers", image=self.app.getAssets().get('plot'), compound=tk.LEFT)

        # Add the staility editor
        self.stabilityEditor = StabilityEditor(self.tabbedPane, self.app)
        self.tabbedPane.add(self.stabilityEditor, text="Stability Editor", image=self.app.getAssets().get('stability'), compound=tk.LEFT)

        self.tabbedPane.pack(fill=tk.BOTH, expand=True)

        # Add plot event
        self.tabbedPane.bind("<<NotebookTabChanged>>", lambda e:self.plotViewver.forceUpdateCurves())

        # Add the time bar
        self.timeSlider = ttk.Scale(self)
        self.timeSlider.pack(side=tk.BOTTOM, fill=tk.X)
        self.timeSlider.bind("<Motion>", self.timeSliderMoved)

        # Add the display control menu
        self.buildDisplayPolicyMenu()

        # update the display policy
        self.updateDisplayPolicy()

        # load preset
        self.setDisplayPolicyFromDic(DEFAULT_DISPLAY_POLICIY)

        self.canvas.bind_all("<MouseWheel>", self.mouseWheel)
        self.canvas.bind("<Motion>", self.mouseMove)
        self.canvas.bind("<Button-1>", self.mouseLeftDown)
        self.canvas.bind("<ButtonRelease-1>", self.mouseLeftUp)

        self.cameraStartingPt = None

        self.newtonScale = 0.2 # correspondance newton to meter
        self.modifySpeed(value=DEFAULT_DT_SIM, type='live') # integration period

        try:
            self.results.load('simu.json', self.app.getBoat().getSolver())
            self.plotViewver.load()
        except:
            print("[ERROR] Fail to read the result file")

        self.simuSourceChanged()

        self.crashCallback = None

        self.canvas.bind_all("<Key>", self.keyEvent)

    def getStabilityEditor(self):
        """
        Return the stability editor object
        """
        return self.stabilityEditor

    def setCameraToTopView(self):
        self.camera.setAngle(TOPVIEW)

    def setCameraToSideView(self):
        self.camera.setAngle(SIDEVIEW)

    def setCameraToFrontView(self):
        self.camera.setAngle(FRONTVIEW)

    def slowdownSimu(self):
        self.modifySpeed(rel=0.5)

    def speedUpSimu(self):
        self.modifySpeed(rel=2)

    def keyEvent(self, e):
        """
        Triggered by a key event
        """
        if (e.char == 'f'):
            self.fitCamera()
        elif (e.char == ' '):
            self.getRunStopButton().toggle()
        elif (e.char == 'r'):
            self.resetSimu()
        elif (e.char == 'n'):
            self.nextStep()
        elif (e.char == 'p'):
            self.previousStep()

    def setLiveComputation(self, state):
        """
        Set the live computation state
        """
        self.liveComputation.set(state)
    
    def simuSourceChanged(self):
        """
        Call when the simulation source is changed
        """
        if (self.liveComputation.get() == 0):
            # Result play back
            # We update the animation
            self.results.setBoatState(self.app.getBoat().getSolver())
            self.results.setAnim(self.app.getBoat().getSolver(), self.runStopButton.get())
            if (self.plotViewver != None):
                self.plotViewver.load()
        else:
            # Live computation
            # We load the state vector of the live computation
            self.app.getBoat().getSolver().loadStateVector(self.app.getBoat().getEDOSolver().getState())
            #if (self.plotViewver != None):
            #    self.plotViewver.attachLivePlot()
        
        self.updateSpeedLabel()

    def runStop(self, state):
        """
        Set the state of the result animation OR the live computation
        """
        if (self.liveComputation.get() == 0):
            # Result play back
            if (self.timeSlider != None):
                self.results.setAnim(self.app.getBoat().getSolver(), state=state, date=self.timeSlider.get())
        # Update the screen
        self.simuSourceChanged()

    def getRunStopButton(self):
        """
        Return the run/stop button
        """
        return self.runStopButton

    def resetSimu(self, resetTargetFct=True):
        """
        Reset the simulation
        """
        if (self.liveComputation.get() == 0):
            # Reset the play back (set t=0)
            self.timeSlider.set(0)
            self.results.setAnim(self.app.getBoat().getSolver(), date=self.timeSlider.get())

        else:
            # Reset the live simulation
            self.app.getBoat().getEDOSolver().reset(resetTargetFct=resetTargetFct)
            # Get the reset state vector:
            X0 = self.app.getBoat().getEDOSolver().getState()
            self.app.getBoat().getSolver().loadStateVector(X0)
            
        self.plotViewver.resetPlot()
        self.modifySpeed(value=DEFAULT_DT_SIM, type='live')

    def modifySpeed(self, rel=None, value=None, type=None):
        if (type == 'live'):
            self.dtsim = value
            
        if (rel != None):
            if ((type != 'live') and (type != None)) or (self.liveComputation.get() == 0):
                # update the speed of the result playback
                self.results.setPlaybackSpeed(self.results.getPlaybackSpeed()*rel)
            else:
                # update the speed of the computation
                self.dtsim *= rel

        self.updateSpeedLabel()

    def updateSpeedLabel(self):
        if self.dtLabel == None:
            return
        
        if (self.liveComputation.get() == 0):
            txt = "speed: x{:.2f}".format(self.results.getPlaybackSpeed())
        else:
            txt = "dt={:.1f}ms".format(1000*self.dtsim)
        self.dtLabel.config(text=txt)

    def buildDisplayPolicyMenu(self):
        col = 0
        row = 0
        self.displayPolicyVar = {}

        boxBorder = tk.Frame(self, padx=5, pady=5, background=BOX_BORDER_COLOR)
        displayPolicyFrame = tk.Frame(boxBorder)

        for elm in ELM_DISPLAYABLE:
            self.displayPolicyVar[elm] = tk.IntVar(value=1)
            ttk.Checkbutton(displayPolicyFrame, 
                            text=elm,
                            variable=self.displayPolicyVar[elm],
                            #background=COLOR_BOX
                            ).grid(row=row, column=col, sticky=tk.NSEW)
            self.displayPolicyVar[elm].trace_add("write", lambda x,y,z:self.updateDisplayPolicy())
            
            col += 1
            if (col > 4):
                col = 0
                row += 1

        # update row weights for layout manager
        for i in range(row):
            displayPolicyFrame.rowconfigure(i, weight=1)
        # update column weights for layout manager
        for i in range(4):
            displayPolicyFrame.columnconfigure(i, weight=1)

        displayPolicyFrame.pack(fill=tk.X, expand=True, side=tk.BOTTOM)
        boxBorder.pack(fill=tk.X, expand=False, padx=5, pady=5, side=tk.BOTTOM)

    def updateDisplayPolicy(self):
        """
        Update the set that define which component should be displayed or not
        """
        self.displayPolicy = set()
        for elm in self.displayPolicyVar:
            if self.displayPolicyVar[elm].get() != 0:
                self.displayPolicy.add(elm)

    def drawScene(self, scene, updateBox=True):
        """
        Draw a 3D scene <scene>
        if <updateBox> is set to true, the boundary box is updated
        """
        screenSize = (self.canvas.winfo_width(), self.canvas.winfo_height())
        
        # Reset the box that hold all the point
        if updateBox:
            self.resetMinMaxProj()

        projectedPolys = {} #Dictionnary which hold the polygons projected
        distPolys = {} #Dictionnary which hold the polygons projected and their distance to the camera

        offy = 0 # Y-Offset

        if ('sinfo' in self.displayPolicy):
            offy = 80
        # For each components
        for comp in scene:
            if ('polygons' in scene[comp]):
                # Save each polygons on the scene
                projectedPolys[comp] = []

                # Project every polygon on the 2D plane
                polys = scene[comp]['polygons']
                if (self.liveComputation.get() == 0):
                    # Play back, set the color of all the boat
                    scene[comp]['color'] = COLOR_PLAYBACK

                for i in range(len(polys)):
                    dist = computeDistToCam(polys[i], self.camera)
                    light = computeLightCam(polys[i], self.camera)
                    fill = False
                    if ('fill' in scene[comp]):
                        fill = scene[comp]['fill']
                    polys[i] = projection(polys[i], self.camera, screenSize)
                    projectedPolys[comp].append(polys[i])
                    distPolys[dist] = {'polygon':polys[i], 
                                       'color':scene[comp]['color'],
                                       'light':light,
                                       'fill':fill}

        # Draw the filled area
        if (self.filledPolygons.get()):
            for dist in reversed(sorted(distPolys.keys())):
                if not distPolys[dist]['fill']:
                    continue
                coords = []
                for pt in distPolys[dist]['polygon']:
                    coords.append(pt[0])
                    coords.append(pt[1]+offy)
                light = np.abs(distPolys[dist]['light'])/2
                color = gradientColor(distPolys[dist]['color'], COLOR_BG_VIEWVER_BOTTOM, light)
                self.canvas.create_polygon(coords, fill=color)


        # For each components
        for comp in scene:

            # Draw a polygon if necessary
            if ('polygons' in scene[comp]):

                # get the projected polygon on the 2D plane
                polys = projectedPolys[comp]
                    
                # Get the width of the line
                width = 1
                if ('width' in scene[comp]):
                    width = scene[comp]['width']
                    
                # Draw the poly wire
                for poly in polys:
                    if (len(poly) > 0):
                        startPt = poly[0]
                        for i in range(1, len(poly)):
                            endPt = poly[i]
                            if (self.wirePolygons.get()):
                                x0, y0 = startPt
                                x1, y1 = endPt
                                self.canvas.create_line(x0, y0+offy,
                                                        x1, y1+offy,
                                                        width=width,
                                                        fill=scene[comp]['color'])
                            startPt = endPt

                            # save the box that hold all the points
                            if updateBox and 'hitbox' in scene[comp]:
                                self.saveMinMaxProj(startPt)


            # draw a point if necessary 
            if ('point' in scene[comp]):
                proj = projection([scene[comp]['point']], self.camera, screenSize)[0]
                self.canvas.create_oval(proj[0]-RENDERER_POINT_HSIZE,
                                        proj[1]-RENDERER_POINT_HSIZE+offy,
                                        proj[0]+RENDERER_POINT_HSIZE,
                                        proj[1]+RENDERER_POINT_HSIZE+offy,
                                        fill=scene[comp]['color'])
                
            # draw a vector if necessary 
            if ('vec' in scene[comp]):
                proj = projection([scene[comp]['point'], scene[comp]['vec']*self.newtonScale+scene[comp]['point']], self.camera, screenSize)
                x0, y0 = proj[0]
                x1, y1 = proj[1]
                self.canvas.create_line(x0, y0+offy,
                                        x1, y1+offy,
                                        width=2,
                                        fill=scene[comp]['color'], 
                                        arrow=tk.LAST)
                
        
            # darw a text if necessary
            if ('text' in scene[comp]):
                proj = projection([scene[comp]['point']], self.camera, screenSize)[0]
                self.canvas.create_text(proj[0], proj[1]+10+offy, text=scene[comp]['text'])

        # Draw the basis
        self.drawFixBasis()

        #self.canvas.create_rectangle(self.minSceneX, self.minSceneY,
        #                             self.maxSceneX, self.maxSceneY)
        

    def drawFixBasis(self):
        """
        Draw a fix basis at the bottom left corner
        """
        screenSize = (self.canvas.winfo_width(), self.canvas.winfo_height())
        offset = np.array([int(1.5*BASE_3DRENDERER_SIZE), int(screenSize[1]-1.5*BASE_3DRENDERER_SIZE)])
        basisGUI = {0:{'color':'red', 'label':'x'},
                    1:{'color':'green', 'label':'y'},
                    2:{'color':'blue', 'label':'z'}}
        for i in basisGUI:
            axis3D = np.zeros(3)
            axis3D[i] = BASE_3DRENDERER_SIZE
            proj = projectionOnlyRotation([axis3D,axis3D*1.2], self.camera)
            self.canvas.create_line(*list(offset),*list(proj[0]+offset),
                                width=2,fill=basisGUI[i]['color'],arrow=tk.LAST)
            self.canvas.create_text(*list(proj[1] + offset), text=basisGUI[i]['label'])

    def resetMinMaxProj(self):
        """
        Reset the min/max boundary of the displayed scene for
        x and y axis
        """
        w, h = self.winfo_width(), self.winfo_height()
        self.minSceneX = w
        self.maxSceneX = 0
        self.minSceneY = h
        self.maxSceneY = 0

    def saveMinMaxProj(self, pt):
        """
        Save the min/max boundary of the displayed scene for
        x and y axis
        """
        # Check the maximums
        if (pt[0] > self.maxSceneX):
            self.maxSceneX = pt[0]
        if (pt[1] > self.maxSceneY):
            self.maxSceneY = pt[1]

        # Check the minimums
        if (pt[0] < self.minSceneX):
            self.minSceneX = pt[0]
        if (pt[1] < self.minSceneY):
            self.minSceneY = pt[1]
            
    def mouseWheel(self, event):
        """
        Mouse wheel event (zoom)
        """
        # Check if inside the canvas
        if (event.widget != self.canvas):
            return
        
        if event.delta > 0:
            self.camera.zoom *= 1 + ZOOM_SPEED
        else:
            self.camera.zoom *= 1 - ZOOM_SPEED

    def mouseLeftDown(self, event):
        """
        Mouse left click down
        """
        self.cameraStartingPt = [event.x, event.y]

        # Save the initial rotation matrix
        self.cameraOriginalRot = self.camera.rotationMatrix.copy()

        # Save the vertical and horizontal axis
        self.camHorizontalAxis = np.matmul(self.camera.getRotationMatrix(), np.array([1,0,0]))
        self.camVerticalAxis = np.matmul(self.camera.getRotationMatrix(), np.array([0,0,1]))

    def mouseLeftUp(self, event):
        """
        Mouse left click up
        """
        self.cameraStartingPt = None

    def mouseMove(self, event):
        """
        Mouse motion
        """
        if (self.cameraStartingPt == None):
            return
        
        # Movement in the Screen-Y : around the horizontal axis
        # Movement in the Screen-X : around the vertical axis

        dx = event.x - self.cameraStartingPt[0]
        dy = event.y - self.cameraStartingPt[1]
        screenSize = (self.canvas.winfo_width(), self.canvas.winfo_height())

        v = ZOOM_ROT_SPEED
        rx = v*dx/screenSize[0]
        ry = v*dy/screenSize[1]

        self.camHorizontalAxis = np.array([1,0,0])
        self.camVerticalAxis = np.array([0,0,1])

        NewRotVertical = getMatRot(self.camVerticalAxis, rx)
        NewRotHorizontal = getMatRot(self.camHorizontalAxis, ry)

        M = NewRotVertical
        M = np.matmul(M, NewRotHorizontal)
        self.camera.rotationMatrix = np.matmul(M, self.camera.rotationMatrix)

        self.cameraStartingPt = [event.x, event.y]

    def timeSliderMoved(self, event):
        if (self.liveComputation.get() == 0):
            if (self.runStopButton.get() != False):
                self.runStopButton.set(False)
            self.results.setAnim(self.app.getBoat().getSolver(), date=self.timeSlider.get())
            self.results.updateAnim(self.app.getBoat().getSolver())
            self.app.getBoat().getSolver().compute()
        else:
            self.plotViewver.setTlim(1-self.timeSlider.get())
        
    def run(self):
        """
        Lunch the live interaction
        """
        self.runDisplay = True

        while self.runDisplay:
            self.displayFrame()

    def displayFrame(self):
        """
        Display a frame
        """
        self.canvas.delete('all')
        self.drawBackground()

        if self.liveComputation.get() == 0:
            # We play back the result
            if (self.runStopButton.get()):
                self.results.updateAnim(self.app.getBoat().getSolver())
                # update the forces
                self.app.getBoat().getSolver().compute()
                self.timeSlider.set(self.results.getRelDate())
        else:
            if (self.runStopButton.get()):
                # We play the computation
                self.nextStep()

        self.drawScene(self.app.getBoat().getScene(self.displayPolicy))
        self.drawInfo(self.app.getBoat().getSolver().getState())
                
        self.app.update()
        time.sleep(0.001)
        self.camera.pos = self.app.getBoat().getSolver().getBoatPos()


    def nextStep(self):
        """
        Compute the next step of the simulation
        """
        if self.liveComputation.get() == 1:
            try:
                self.app.getBoat().getEDOSolver().step(self.dtsim)
            except ResetSimuException:
                self.resetSimu(resetTargetFct=False)
                self.forgetCrashCallback()
            except SimuFinishedException:
                self.runStopButton.set(False)
                self.forgetCrashCallback()
            except Exception as e:
                traceback.print_exc()
                self.runStopButton.set(False)

                print("[ERROR] - SIMULATION FAILED")

                # if there is a crash callback
                if (self.crashCallback != None):
                    # Run it
                    self.crashCallback()
                    # Forget
                    self.forgetCrashCallback()

            # Then, plot
            EDOS = self.app.getBoat().getEDOSolver()
            self.plotViewver.plot(self.app.getBoat().getSolver(),
                                  EDOS.getTime(), 
                                  EDOS.getState(), 
                                  EDOS.getCommand(), 
                                  EDOS.getTarget(), 
                                  self.tabbedPane.index(self.tabbedPane.select()) == 1)

        else:
            self.timeSlider.set(self.results.step(self.app.getBoat().getSolver(), True))

    def previousStep(self):
        """
        Display the previous step of the simulation
        """
        if self.liveComputation.get() == 0:
            self.timeSlider.set(self.results.step(self.app.getBoat().getSolver(), False))

    def plotStateVector(self):
        """
        Display the current state vector
        """
        stateJSON = self.app.getBoat().getEDOSolver().getState().toJSON()
        for elm in stateJSON:
            print(elm,"->",stateJSON[elm])

    def drawCompass(self, x, y, r, aboat, atrue, atarget, awind, asail):
        """
        Draw a compass at (x, y)
        """
        self.canvas.create_oval(x, y, x+2*r, y+2*r, width=2, outline=COLOR_BOX_DARK)

        def getCoord(a, r0):
            return x+r+np.cos(a)*r0, y+r+np.sin(a)*r0
        
        # Then plot the graduation (Major)
        for i in np.linspace(0, 2*np.pi, 9):
            self.canvas.create_line(*getCoord(i, r*1.05), *getCoord(i, r*0.9), width=2, fill=COLOR_BOX_DARK)
        # Then plot the graduation (Minor)
        for i in np.linspace(0, 2*np.pi, 37):
            self.canvas.create_line(*getCoord(i, r), *getCoord(i, r*0.9), width=1, fill=COLOR_BOX_BORDER)

        # Display NSEW
        for i in range(4):
            self.canvas.create_text(*getCoord(i*np.pi/2, r*0.8), text="NSEW"[i], font=COMPASS_FONT, fill=LIGHT_TEXT_COLOR)

        # plot the needles
        self.canvas.create_line(*getCoord(atrue, 0), *getCoord(atrue, r*0.8), width=3, fill="black")
        self.canvas.create_line(*getCoord(aboat, 0), *getCoord(aboat, r*0.7), width=3, fill="cyan")
        self.canvas.create_line(*getCoord(atarget, 0), *getCoord(atarget, r*0.6), width=3, fill="red")
        self.canvas.create_line(*getCoord(awind, 0), *getCoord(awind, r*0.5), width=3, fill="magenta")
        self.canvas.create_line(*getCoord(asail, 0), *getCoord(asail, r*0.4), width=3, fill="pink")
        s = 1
        self.canvas.create_rectangle(x+r-s, y+r-s, x+r+s, y+r+s, fill="black")
        # display the angles
        pady = 8
        self.canvas.create_text(x-pady, y-pady, text="Tru={:03d}°".format(int(np.rad2deg(atrue))), font=COMPASS_FONT, anchor=tk.W, fill="black")
        self.canvas.create_text(x-pady, y+2*r+pady, text="Yaw={:03d}°".format(int(np.rad2deg(aboat))), font=COMPASS_FONT, anchor=tk.W, fill="blue")
        self.canvas.create_text(x-pady, y+2*r+pady+15, text="Sai={:03d}°".format(int(np.rad2deg(asail))), font=COMPASS_FONT, anchor=tk.W, fill=PINK_DARK)
        self.canvas.create_text(x+2*r+pady, y-pady, text="Tgt={:03d}°".format(int(np.rad2deg(atarget))), font=COMPASS_FONT, anchor=tk.E, fill="red")
        self.canvas.create_text(x+2*r+pady, y+2*r+pady, text="Wnd={:03d}°".format(int(np.rad2deg(awind))), font=COMPASS_FONT, anchor=tk.E, fill="magenta")
        self.canvas.create_text(x+2*r+pady, y+2*r+pady+15, text="Dri={:.2f}°".format(np.rad2deg(getAngleDif(aboat, atrue))), font=COMPASS_FONT, anchor=tk.E, fill=COLOR_BOX_DARK)

    def drawSpeedGauge(self, x, y, h, wspeed, bspeed):
        """
        Draw the speed gauge
        """
        # draw the text ticks
        h -= 10
        ylast = None
        pady = 8
        maxSpeed = 20 # in knot
        def getYOfSpeed(speed): # speed in knot
            return y+30+h*speed/maxSpeed

        for i in np.arange(0, 20, 5):
            yt = getYOfSpeed(i)
            if (ylast != None):
                self.canvas.create_line(x, ylast+pady, x, yt-pady, width=2, fill=COLOR_BOX_DARK)
            ylast = yt
            self.canvas.create_text(x, yt, text="{:02d}".format(i), font=COMPASS_FONT, fill=COLOR_BOX_DARK)
        
        # draw the cursor
        def drawCursor(speed, left, color):
            if (left):
                offx = 25
                length = 15
                heigth = 8
                w = 2
            else:
                offx = -25
                length = -15
                heigth = 8
                w = 2
            ys = getYOfSpeed(speed)
            self.canvas.create_line(x-offx, ys-heigth//2, x-offx+length*3//4, ys-heigth//2, width=w, fill=color)
            self.canvas.create_line(x-offx+length*3//4, ys-heigth//2, x-offx+length, ys, width=w, fill=color)
            self.canvas.create_line(x-offx+length, ys, x-offx+length*3//4, ys+heigth//2, width=w, fill=color)
            self.canvas.create_line(x-offx+length*3//4, ys+heigth//2, x-offx, ys+heigth//2, width=w, fill=color)
            self.canvas.create_line(x-offx, ys-heigth//2, x-offx, ys+heigth//2, width=w, fill=color)

        # Wind
        drawCursor(ms2noeud(wspeed), True, "black")
        self.canvas.create_text(x-10, y, text="Wind", font=COMPASS_FONT, anchor=tk.E, fill="black")
        self.canvas.create_text(x-10, y+10, text="{:.1f}".format(ms2noeud(wspeed)), font=COMPASS_FONT, anchor=tk.E, fill="black")

        # Boat
        drawCursor(ms2noeud(bspeed), False, "blue")
        self.canvas.create_text(x+10, y-2, text="Posidonie", font=COMPASS_FONT, anchor=tk.W, fill="blue")
        self.canvas.create_text(x+10, y+15, text="{:.2f}".format(ms2noeud(bspeed)), font=COMPASS_FONT_BIG, anchor=tk.W, fill="blue")

    def drawQuarterGauge(self, x, y, r, roll, pitch, rudder):
        """
        Draw a quarter gauge
        """
        def getCoord(a, r0):
            return x+r+np.sin(a)*r0, y-2*r+np.cos(a)*r0
        
        # Then plot the graduation (Major)
        lastpt = None
        for i in np.linspace(-np.pi/4, np.pi/4, 16):
            self.canvas.create_line(*getCoord(i, r*1.05), *getCoord(i, r*0.9), width=2, fill=COLOR_BOX_DARK)
            if (lastpt != None):
                self.canvas.create_line(*lastpt, *getCoord(i, r), width=2, fill=COLOR_BOX_DARK)
            lastpt = getCoord(i, r)
        xt, yt = getCoord(-np.pi/4, r)
        self.canvas.create_text(xt, yt-15, text="-45°", font=COMPASS_FONT, fill=COLOR_BOX_DARK)
        xt, yt = getCoord(np.pi/4, r)
        self.canvas.create_text(xt, yt-15, text="45°", font=COMPASS_FONT, fill=COLOR_BOX_DARK)

        # Draw the mark for the rudder max ang
        rudMax = 25
        self.canvas.create_line(*getCoord(np.deg2rad(rudMax), r*0.9), *getCoord(np.deg2rad(rudMax), r*1.1), width=3, fill="black")
        self.canvas.create_line(*getCoord(-np.deg2rad(rudMax), r*0.9), *getCoord(-np.deg2rad(rudMax), r*1.1), width=3, fill="black")

        # then plot the needles
        x0, y0 = getCoord(0, 0)
        self.canvas.create_line(x0, y0, *getCoord(roll, r*0.9), width=3, fill="red")
        self.canvas.create_line(x0, y0, *getCoord(pitch, r*0.8), width=3, fill="blue")
        self.canvas.create_line(x0, y0, *getCoord(rudder, r*0.7), width=3, fill="pink")
        
        # add legend
        self.canvas.create_text(x0, y0-12, text="Rol={:.1f}".format(np.rad2deg(roll)), font=COMPASS_FONT, fill="red")
        self.canvas.create_text(x0, y0-12*2, text="Pit={:.1f}".format(np.rad2deg(pitch)), font=COMPASS_FONT, fill="blue")
        self.canvas.create_text(x0, y0-12*3, text="Rud={:.1f}".format(np.rad2deg(rudder)), font=COMPASS_FONT, fill=PINK_DARK)


    def drawInfo(self, X):
        """
        Draw the information of the solver
        """
        w, h = self.winfo_width(), self.winfo_height()
        # Prepare size useful info
        CHAR_HEIGHT = int(INFO_FONT[1]*1.5)
        simuTime = self.app.getBoat().getEDOSolver().getTime()
        tframe = time.time() - self.lastFrameTime
        if tframe != 0:
            sire = self.dtsim/tframe
        else:
            sire = 0

        if ('sinfo' in self.displayPolicy):
            self.drawCompass(30, 30, 60,
                         X._x_ang[-1],
                         X.getTrueHeading(),
                         self.app.getBoat().getEDOSolver().getTarget()['target'],
                         self.app.getBoat().getSolver().getWind().getAngle(),
                         self.app.getBoat().getSolver().getSailAng())
        
            self.drawSpeedGauge(30 + 200, 30, 120, 
                                self.app.getBoat().getSolver().getWind().getSpeed(),
                                X.getBoatSpeedNorm())
            
            self.drawQuarterGauge(30 + 250, 200, 60, #w-180, h-120, 60,
                                X._x_ang[0],
                                X._x_ang[1],
                                self.app.getBoat().getSolver().getRudderAng())
        # Draw static info
        if (tframe != 0):
            fps = 1/(tframe)
        else:
            fps = 0
        self.lastFrameTime = time.time()
        msg = "fps: " + str(int(fps))
        ypos = CHAR_HEIGHT
        self.canvas.create_text(w-CHAR_HEIGHT, ypos, text=msg, anchor=tk.E, font=INFO_FONT, fill=COLOR_BOX_DARK)
        ypos += CHAR_HEIGHT

        if ('sinfo' in self.displayPolicy):
            self.canvas.create_text(w-CHAR_HEIGHT, ypos, text="s/r={:.2f}".format(sire), anchor=tk.E, font=INFO_FONT, fill=COLOR_BOX_DARK)
            ypos += CHAR_HEIGHT
            self.canvas.create_text(w-CHAR_HEIGHT, ypos, text="E={:.1f}°".format(np.rad2deg(self.app.getBoat().getEDOSolver().getEpsi()[0])), anchor=tk.E, font=INFO_FONT, fill=COLOR_BOX_DARK)
            ypos += CHAR_HEIGHT

        if (self.runStopButton.get()):
            self.canvas.create_text(w-CHAR_HEIGHT, ypos, text="RUNNING (t={:.1f}s)".format(simuTime), anchor=tk.E, font=INFO_FONT, fill=COLOR_BOX_BORDER_FOCUSED)
        else:
            self.canvas.create_text(w-CHAR_HEIGHT, ypos, text="STOP", anchor=tk.E, font=INFO_FONT, fill=COLOR_BOX_BORDER_FOCUSED)
        

    def drawBackground(self):
        """
        Draw a color gradient background
        """
        w, h = self.winfo_width(), self.winfo_height()
        
        def f(i):
            return i**2

        for i in range(h):
            self.canvas.create_line(0, i, w, i, fill=gradientColor(COLOR_BG_VIEWVER_TOP, COLOR_BG_VIEWVER_BOTTOM, f(i/h)))

    def kill(self):
        """
        Stop the simulation
        """
        self.runDisplay = False

    def resetCamera(self):
        """
        Set the camera to the initial view angle, zoom, and position
        """
        self.camera.reset()

    def fitCamera(self):
        """
        Make the scene fill all the screen
        """
        if self.minSceneY > self.maxSceneY:
            # Boundaries not updated yet
            return
        
        # Find the zoom to appy
        w, h = self.winfo_width(), self.winfo_height()

        # Percentage of the space between the top of the scene and the top of the window
        alphaTop = abs(self.maxSceneY-h/2)/h
        # Do the same for the other dirrections
        alphaBottom = abs(self.minSceneY-h/2)/h
        alphaLeft = abs(self.minSceneX-w/2)/w
        alphaRight = abs(self.maxSceneX-w/2)/w

        # Find the maximal alpha
        alphaMax = max(alphaTop, alphaBottom, alphaLeft, alphaRight)
        
        # Deduce and apply the zoom
        self.camera.zoom = (1-PAD_MAX_ZOOM)*self.camera.zoom/(2*alphaMax)

    def setProprety(self, id, value):
        """
        Set a proprety for this component
        """
        if (id == "newtonScale"):
            self.newtonScale = value
        else:
            raise Exception("Unknow proprety: "+id)
        
    def setDisplayPolicy(self, name, value):
        """
        Set the display policy of a parameter named name, to value
        """
        if (name in self.displayPolicyVar):
            self.displayPolicyVar[name].set(value==True)

    def setDisplayPolicyFromDic(self, dic):
        """
        Set the display policy from a dictionnary
        """
        for name in dic:
            self.setDisplayPolicy(name, dic[name])

    def getDisplayPolicy(self):
        """
        Return the display policy dictionary
        """
        out = {}
        for name in self.displayPolicyVar:
            out[name] = self.displayPolicyVar[name].get() != 0
        return out

    def setFilledPolygons(self, state):
        """
        Set the display style of the polygons
        """
        self.filledPolygons.set(state==True)

    def getFilledPolygons(self):
        """
        Get the display style of the polygons
        """
        return self.filledPolygons.get() != 0

    def getCanvas(self):
        """
        Return the canvas object
        """
        return self.canvas
    
    def setCrashCallback(self, fct):
        """
        Callback used in case of crash.
        Once used, the function is "forgeted" i.e. a new crash will not call this function
        If the simulation is a success / resetted, callback is also forgeted
        """
        self.crashCallback = fct

    def forgetCrashCallback(self):
        """
        Forget the crash callback
        """
        self.crashCallback = None
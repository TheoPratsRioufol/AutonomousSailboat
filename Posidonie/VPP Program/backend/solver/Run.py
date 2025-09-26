import time

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *
from utils.Utils import *

class Run():
    def __init__(self):
        """
        Run object used to save all the data relative to a simulation
        """
        self.timer = time.time()
        self.play = False # Variable to play or not the annimation
        self.simuTime = 0 # simulation time
        self.Xs = []
        self.Us = []
        self.ts = []
        self.Ts = []
        self.loaded = False
        self.playbackSpeed = 1

    def load(self, path, solver):
        """
        Load simulation results form a file
        """
        self.ts, self.Xs, self.Us, self.Ts = loadSimuFile(path, solver)
        self.ti = self.ts[0]
        self.tf = self.ts[-1]
        self.loaded = True

    def setBoatState(self, solver):
        """
        Set the boat at the state corresponding to the simulation time
        """
        if not self.loaded:
            return
        # get the closer index in the time
        #idx = getCloserIdx(self.simuTime, self.ts)
        i1, i2 = getBeforeAfterIdx(self.simuTime, self.ts)
        alpha = (self.simuTime - self.ts[i1])/(self.ts[i2] - self.ts[i1])
        # Linear interpollation
        X = self.Xs[i1] + (self.Xs[i2] - self.Xs[i1])*alpha
        solver.loadStateVector(X, self.Us[i1])
        #solver.loadStateVector(self.Xs[idx], self.Us[idx])

    def updateAnim(self, solver):
        """
        If running, set the simulation state to the real time
        """
        if not self.loaded:
            return
        if self.play:
            self.simuTime = (time.time() - self.timer)*self.getPlaybackSpeed()
            if (self.simuTime > self.tf) or (self.simuTime < self.ti):
                # out of bound, reset
                self.timer = time.time() - self.ti/self.getPlaybackSpeed()
                self.simuTime = self.ti
            
            self.setBoatState(solver)

    def setAnim(self, solver, state=None, date=None):
        """
        Stop/Play the annimation
            date is a relative time to begin/display the simulation with
        """
        if not self.loaded:
            return
        if (date != None):
            # Set the simulation time
            self.simuTime = date*(self.tf - self.ti) + self.ti
            self.timer = time.time() - self.simuTime/self.getPlaybackSpeed()
            # load the state
            self.setBoatState(solver)

        if (state != None):
            if (state == True):
                # start from the stoped time
                self.timer = time.time() - self.simuTime/self.getPlaybackSpeed()

            self.play = state

    def step(self, solver, way=True):
        """
        Make one step in the simulation points.
            - way = True  -> toward t > 0
            - way = False -> toward t < 0
        """
        if self.loaded == False:
            return
        
        dt = (self.tf - self.ti)/len(self.ts)
        if (way):
            if (self.simuTime + dt < self.tf):
                self.simuTime += dt
        else:
            if (self.simuTime - dt > self.ti):
                self.simuTime -= dt
        # Set the state
        self.setBoatState(solver)
        # Return the relative time
        return (self.simuTime - self.ti)/(self.tf - self.ti)

    def getRelDate(self):
        """
        Return the current relative date
        """
        if not self.loaded:
            return 0
        return (self.simuTime - self.ti)/(self.tf - self.ti)
        
    def getTime(self):
        return self.ts
    
    def getCurves(self):
        """
        Return a dictionnary of the relevent curves to plot
        """
        if (len(self.Xs) == 0):
            return {}
        curves = {}

        # First, the curve relative to the state vector
        for name in self.Xs[0].getParamNames():
            curves[name] = np.array([X.get(name) for X in self.Xs])

        # Then the one of the target vector
        for name in self.Ts[0]:
            curves[name] = np.array([T[name] for T in self.Ts])

        return curves
    
    def getPlaybackSpeed(self):
        """
        Get the playback speed
        """
        return self.playbackSpeed

    def setPlaybackSpeed(self, value):
        """
        Set the playback speed
        """
        alpha = self.playbackSpeed/value
        self.playbackSpeed = value

        # change the timer to get a continuous transition
        self.timer = time.time()*(1-alpha) + self.timer*alpha

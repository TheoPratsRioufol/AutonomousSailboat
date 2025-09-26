import numpy as np

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from utils.Force import *

windDefaultGeom = {}

class Wind():
    def __init__(self, solver):
        self._solver = solver
        self.updateGeom(windDefaultGeom)
        self.ang = 0
        self.speed = 0
        self.update()

    def updateGeom(self, geom):
        """
        Update the "geometry" of the wind
        """
        self.geom = geom

    def updateFromCommand(self, cmd):
        """
        Update this component from the command vector
        """
        self.ang = cmd._u_wang
        self.speed = cmd._u_wspeed
        self.update()

    def update(self):
        """
        Update the wind vector from the angle and speed
        """
        self.wind = np.array([self.speed, 0.0, 0.0])
        self.wind = rotation([self.wind], np.zeros(3), Dir.Z, self.ang)[0]

    def getAngle(self):
        """
        Return the angle of the wind
        """
        return self.ang
    
    def getSpeed(self):
        """
        Return the speed of the wind
        """
        return self.speed
    
    def getSpeedVec(self):
        """
        Return the wind velocity vector relative to the sea
        """
        return Vector(self._solver, self.wind, Base.SEA)
    
    def getGeom(self):
        """
        Return the geometry of this component
        """
        return self.geom
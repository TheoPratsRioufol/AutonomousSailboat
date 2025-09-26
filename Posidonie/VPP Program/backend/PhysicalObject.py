import numpy as np

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *
from utils.Force import *
from utils.Utils import *

class PhysicalObject():
    """
    This class represents a phycial component
    """

    def __init__(self, solver):
        self.geom = {} # The geometry of the component
        self.pos = np.zeros(3) # Position of the component relative to x0
        self._solver = solver # The solver object

    def getTotalForce(self):
        """
        Return the force <Force> class holding the force value, and the application point
        """
        raise Exception("Not implemented!")

    def getInertia(self):
        """
        Return the inertia matrix of this object
        """
        raise Exception("Not implemented!") 

    def getWeight(self):
        """
        Return the weight of this component
        """
        return 0
    
    def updateGeom(self, geom):
        """
        Update the geometry of this component.
        <geom> is a dictionaty {geometryName:value}
        """
        for key in geom:
            self.geom[key] = geom[key]
            
        # Then update the dependencies (may update the cdg!)
        self.updateGeomDependecies()
        # Update the gravity force
        self.computeGravityForce()

    def updateGeomDependecies(self):
        """
        Update this component after the geometry was updated
        """

    def getGeom(self):
        """
        Return the geometry of this object
        """
        return self.geom

    def getPolygons(self):
        """
        Return a list of polygons (list of 3d points) of this component. Used to plot it
        """
        return []
    
    def getCdg(self):
        """
        Return the center of gravity of this component
        """
        raise Exception("Non implemented!")
    
    def getGravityForce(self):
        """
        Return the gravity force on this object
        """
        return self.gravityForce
    
    def computeGravityForce(self):
        """
        Compute the gravity force
        """
        self.gravityForce = Force(Vector(self._solver, PHY_G_VECTOR*self.getWeight(), Base.SEA), self.getCdg())

    def getGeomP(self, name):
        """
        Return the value of the geometric parameter named 'name'
        """
        return Units.toSI(self.geom[name]['value'], self.geom[name]['unit'])
    
    def setGeomP(self, name, value):
        """
        Set the value of a geometry parameter
        """
        self.geom[name]['value'] = value
import numpy as np

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *
from utils.Utils import *
from utils.Force import *

class BulbForceCalculator():

    def __init__(self, solver):
        """
        Class used to compute the drag of the keel's bulb
        """
        self._solver = solver

    def getForce(self):
        """
        Return the water force exerced to the keel's bulb
        """
        # First, we get the speed at the bulb center
        bulbPos = self._solver.getDrift().getCdgLest()
        bulbVol = self._solver.getDrift().getLestVolume()
        bulbUVec = -PointSpeed.getVelocity(self._solver, bulbPos, Referential.BOAT, Referential.SEA).getSpeed().valueIn(Base.BOAT)
        
        # get the incidence angle and speed
        bulbUNorm = np.linalg.norm(bulbUVec)

        if (bulbUNorm == 0):
            return getANullForce(self._solver)

        fdir = bulbUVec/bulbUNorm
        cosang = fdir[0]
        sinang = -fdir[1]
        i = np.arctan2(sinang, cosang)

        # Compute force
        Fx, Fy, Mz = self.getXYM(bulbVol, i, bulbUNorm)

        return Force(Vector(self._solver, np.array([Fx, Fy, 0]), Base.BOAT), Moment(Vector(self._solver, np.array([0, 0, Mz]), Base.BOAT)))
    
    def getXYM(self, V, i, u):
        """
        Return the force for an incidence i; a bulb volume V; and a water speed u
        """
        
        i = getInPi(i)
        iabs = np.abs(i)

        # Bulb: Fx = 3N at i=16°, V = 0.64 L, 2m/s
        # Neglect lift and torque
        V0 = 0.64*0.001
        u0 = 2
        f0 = 3

        FxCase = f0*((V/V0)**(2/3))*(u/u0)**2
        FyCase = 0
        MzCase = 0

        # if i < 0, then swap the y direction
        if (i < 0):
            FxCase = -FxCase

        # FxCase, FyCase, MzCase are in the COMSOL's referential. Need to put them in the correct one
        Fx = np.cos(i)*FxCase + np.sin(i)*FyCase
        Fy = np.cos(i)*FyCase - np.sin(i)*FxCase
        Mz = MzCase

        return Fx, Fy, Mz
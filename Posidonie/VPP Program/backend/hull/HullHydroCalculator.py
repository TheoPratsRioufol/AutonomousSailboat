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

class HullHydroCalculator():

    def __init__(self, solver):
        """
        Class used to compute the drag of the hull
        """
        self._solver = solver

        self.alphaLat = 1 # Ratio between the longitudinal and lateral drag force at the same speed
        self.alphaZ = 200 # Linear damping coeeficient on the z direction

    def longiDrag(self, u):
        """
        u: scalar
        Return the longitudinal drag for a speed u > 0 (movement in the hull direction)
        """
        #tfinal = 20
        #alpha = -np.log(0.1)/tfinal
        return 9.47*u**2

    def getForce(self):
        """
        Return the hydrodynamic force exerced on the hull
        Hyp: no moment (handeled only by the buoyency force)
        """

        """
        Estimation of the drag:
            We have the drag in the boat's direction but not if the water is sideway
            drag = dragXY + dragZ

            dragXY = drag_longi + drag_lat

            with:
                - drag_longi = f(|v.u_longi|)*u_longi      (u_longi unitary vector in the longi direction)
                - drag_lat   = alphaLat*f(|v.u_lat|)*u_lat (u_longi unitary vector in the lateral direction)
                                alphaLat arbitrary scalar (~5, 5 time harder to push the boat sideway than in the front)

            and dragZ = - alphaZ * vz (alphaZ=200 works well)
        """

        speed = self._solver.getBoatSpeed().getSpeed().valueIn(Base.SEA)
        speedXY = speed[:2]
        speedZ = speed[2]
        boatHeading = self._solver.getBoatAng()[-1]

        #print("boatHeading = ",boatHeading)

        # First, compute the dragXY

        u_longi = np.array([np.cos(boatHeading), -np.sin(boatHeading)])
        u_lat = np.array([-u_longi[1], u_longi[0]])

        #print("u_longi =",u_longi)

        v_longi = np.dot(speedXY, u_longi)
        v_lat   = np.dot(speedXY, u_lat)

        drag_longi = -u_longi*self.longiDrag(abs(v_longi))*np.sign(v_longi)
        drag_lat   = -u_lat*self.alphaLat*self.longiDrag(abs(v_lat))*np.sign(v_lat)
        dragXY = drag_longi + drag_lat

        # Then the vertical drag

        dragZ = - self.alphaZ * speedZ

        drag = Vector(self._solver, np.array([dragXY[0], dragXY[1], dragZ]), Base.SEA)
        return Force(drag, Moment(Vector(self._solver, np.zeros(3), Base.SEA)))
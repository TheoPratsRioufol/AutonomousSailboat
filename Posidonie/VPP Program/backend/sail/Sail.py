
import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from backend.PhysicalObject import *
from utils.Utils import *
from config.Config import *
from utils.Force import *
from backend.naca.NACACalculator import *


"""
To use a component:
    First, compute      : self.compute()
    Then, get the force : self.getTotalForce()

To set a geometry:
    Use self.updateGeom(geom)
"""


sailDefaultGeom = {'ls':{'value':0.5, 'unit':Units.m, 'info':'Width of the sail'},
                   'hms':{'value':0.1, 'unit':Units.m, 'info':'Height of the sail mechanism (on the bottom)'},
                   'he':{'value':0.6, 'unit':Units.m, 'info':'Height of the slidable sail section'},
                   'ds':{'value':0.2, 'unit':Units.m, 'info': 'Vertical gap betwen the hull and the sail'},
                   'epsi':{'value':25, 'unit':Units.percent, 'info':'Relative longitudinal position of the aerodynamic center of effort'},
                   'alpha':{'value':50, 'unit':Units.percent, 'info':'Relative longitudinal position (1=aft)'}, # Percentage of the longitudinal boat length
                   'lambda':{'value':100, 'unit':Units.percent, 'info':'Extension of the sail (1=full sail)'},
                   'profile':{'value':"NACA0015", 'unit':Units.none, 'info':'NACA Profile of the sail'}}


class Sail(PhysicalObject):

    def __init__(self, solver, geom=sailDefaultGeom):
        super().__init__(solver)
        self.updateGeom(geom)
        self.aeroForce = getANullForce(solver)

    def updateGeomDependecies(self):
        """
        Update this component after the geometry was updated
        """
        # Return the position of the local referentiel depending on alpha and the hull shape
        self.pos = self._solver.getHull().getTopHullCoord(self.getGeomP('alpha'))

    def getRotationPoint(self):
        """
        Return the rotation point of the sail in the boat basis
        """
        return self.pos

    def getPolygons(self):
        poly = []
        # (x0, y0, z0) bottom left corner
        x0 = - (1 - self.getGeomP('epsi'))*self.getGeomP('ls') + self.pos[0]
        y0 = self.pos[1]
        z0 = self.getGeomP('ds') + self.pos[2]
        # (h, w) height and width
        h = (1 + self.getGeomP('lambda'))*self.getGeomP('he') + self.getGeomP('hms')
        w = self.getGeomP('ls')
        # Then create the polygon
        poly.append(np.array([x0, y0, z0]))
        poly.append(np.array([x0+w, y0, z0]))
        poly.append(np.array([x0+w, y0, z0+h]))
        poly.append(np.array([x0, y0, z0+h]))
        poly.append(np.array([x0, y0, z0]))

        # Finaly apply the rotation of angle getSailAng() and axis [(x0,y0,z0),z]
        poly = rotation(poly, self.getRotationPoint(), Dir.Z, self._solver.getSailAng())

        return [poly]
    
    def getWeight(self):
        """
        Return the weight of this object
        """
        ssail = (2*self.getGeomP('he') + self.getGeomP('hms'))*self.getGeomP('ls')
        hsail = 2*self.getGeomP('he') + self.getGeomP('hms') + self.getGeomP('ds')
        return ssail*PHY_RHOS_SAIL + hsail*PHY_RHOL_MAST
    
    def getTotalForce(self):
        """
        Forces exerced on {sail}:
            - Aerodynamic
            - Gravity
        This object must be computed force
        """
        return self.getGravityForce() + self.getAerodynamicForce()
    
    def compute(self):
        """
        Compute the value of the forces for this step
        """
        self.computeAerodynamicForce()
    
    def getCdg(self):
        """
        Return the position of the center of gravity
        """
        cdg1_z = self.getGeomP('ds') + self.getGeomP('hms') + self.getGeomP('he')/2 + self.getGeomP('he')*self.getGeomP('lambda')
        cdg1_x = self.getGeomP('ls')*self.getGeomP('epsi') - self.getGeomP('ls')/2
        cdg1 = np.array([cdg1_x, 0, cdg1_z])

        cdg2_z = self.getGeomP('ds') + self.getGeomP('hms') + self.getGeomP('he')/2
        cdg2 = np.array([cdg1_x, 0, cdg2_z])

        cdg = (cdg1*self.getGeomP('he') + cdg2*(self.getGeomP('he') + self.getGeomP('hms')))/(2*self.getGeomP('he') + self.getGeomP('hms'))
        cdg += self.pos

        return Point(self._solver, cdg, Referential.SAIL)
    
    def getAerodynamicForce(self):
        return self.aeroForce

    def computeAerodynamicForce(self):
        """
        Compute the aerodynamic force on this object
        """
        self.aeroForce = self._solver.getNACACalculator().getFluidForce(self.linepf, self.lengthpf, Fluids.AIR, Referential.SAIL, Referential.WIND, self.getGeomP('profile'))

    def linepf(self, t):
        """
        Return the position of the profile for a parametrized coordinate t
        """
        # First, find the postion of the origin of the sail
        sailOrigin = self.pos.copy()
        sailOrigin[2] += self.getGeomP('ds')
        # Then the height of the sail
        sailHeight = self.getGeomP('hms') + (1 + self.getGeomP('lambda'))*self.getGeomP('he')
        
        return Point(self._solver, sailOrigin+t*sailHeight*np.array([0, 0, 1]), Referential.BOAT)

    def lengthpf(self, t):
        """
        Return the length of the profile for a parametrized coordinate t
        """
        return self.getGeomP('ls')

    def getSurface(self):
        """
        Return the sail's surface
        """
        return self.getGeomP('ls')*(self.getGeomP('hms') + (1 + self.getGeomP('lambda'))*self.getGeomP('he'))
    
    def getDefaultCenterOfEffort(self):
        """
        Return the "default" center of effort:
            - mid height
            - 0 on y axis
        """
        return self.linepf(0.5) # Mid height
    
    def getInertia(self):
        """
        Return the inertia matrix of this component
        """
        matrix = Matrix(self._solver, getRectangularPrismInertia(self.getGeomP('ls'),
                                                                 self.getGeomP('ls')/6,
                                                                 2*self.getGeomP('he')+self.getGeomP('hms'),
                                                                 self.getWeight()), Base.BOAT)
        return Inertia(matrix, self.getCdg(), self.getWeight())
    
    def getAspectRatio(self):
        """
        Return the aspect ratio of the sail
        """
        return self.getGeomP('ls')/(2*self.getGeomP('he')+self.getGeomP('hms'))
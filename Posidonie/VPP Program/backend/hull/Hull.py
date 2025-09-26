
import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from backend.PhysicalObject import *
from backend.hull.HullBuoyencyCalculator import *
from backend.hull.HullHydroCalculator import *
from config.Config import *


hullDefaultGeom = {'lh':{'value':1, 'unit':Units.m, 'info':'Length of the hull'},
                   'whmid':{'value':1, 'unit':Units.m, 'info':'Mid-width of the hull'},
                   'whend':{'value':0.4, 'unit':Units.m, 'info':'End-width of the hull'},
                   'hhmid':{'value':0.3, 'unit':Units.m, 'info':'Mid-heigth of the hull'},
                   'hhend':{'value':0.25, 'unit':Units.m, 'info':'End-heigth of the hull'},
                   'hweight':{'value':50, 'unit':Units.kg, 'info':'Weight of the hull'},
                   }

class Hull(PhysicalObject):

    def __init__(self, solver):
        super().__init__(solver)
        self.hullBuoyencyCalculator = HullBuoyencyCalculator(solver)
        self.hullHydroCalculator = HullHydroCalculator(solver)

        # Dummy geometry
        self.cdg = Point(solver, np.zeros(3), Referential.BOAT)
        self.frontPos = np.zeros(3)
        self.updateGeom(hullDefaultGeom)

        # Load the buoyency model
        # self.loadBuoyencyModel()

        self.buoyencyForce = getANullForce(solver)
        self.hydroAeroForce = getANullForce(solver)

    def loadBuoyencyModel(self, path):
        """
        Load the buoyency model
        """

        print("Loading the Buoyency Model...")
        self.hullBuoyencyCalculator.loadModelFile(path)

        self.cdg = self.hullBuoyencyCalculator.getCdG()

        # ==============================================
        # Then deduce the geometry fron the loaded model
        # ==============================================

        box = self.hullBuoyencyCalculator.getBox()
        geom = hullDefaultGeom
        self.updateGeom(geom)

        # Compute the size of the boat:
        length = box['max'][0] - box['min'][0]
        height = box['max'][2] - box['min'][2]
        width = box['max'][1] - box['min'][1]

        self.setGeomP('lh', length)
        self.setGeomP('whmid', width)
        self.setGeomP('whend', width*0.7)
        self.setGeomP('hhmid', height)
        self.setGeomP('hhend', height*0.5)

        # Compute the position of the tip of the boat's font
        self.frontPos = np.array([box['max'][0], (box['max'][1]+box['min'][1])/2, box['max'][2]])

        # Set the geometry (and compute gravity force)
        self.updateGeom(geom)


    def getSolver(self):
        return self._solver
    
    def getHullBuoyencyCalculator(self):
        return self.hullBuoyencyCalculator

    def getWeight(self):
        return self.hullBuoyencyCalculator.getWeight()
    
    def getInertia(self):
        """
        Return the inertia matrix of this object
        """
        return Inertia(self.hullBuoyencyCalculator.getInertiaMatrix(), self.getCdg(), self.getWeight())
    
    def getCdg(self):
        """
        Return the position of the center of gravity
        in the boat referential
        """
        return self.cdg
    
    def getBuoyencyForce(self):
        """
        Return the buyency force of the hull
        in the boat referential
        """
        return self.buoyencyForce

    def computeBuoyencyForce(self):
        """
        Compute the buyency force of the hull
        in the boat referential
        """
        self.buoyencyForce = self.hullBuoyencyCalculator.getForce()

    def computeHydroAeroForce(self):
        """
        Compute the Hydrodynamic and aerodynamic force on the hull
        We neglect the aero dynamic force
        """
        self.hydroAeroForce = self.hullHydroCalculator.getForce()
    
    def compute(self):
        """
        Compute the force on this component for the current step
        """
        self.computeBuoyencyForce()
        self.computeHydroAeroForce()

    def getHydroAeroForce(self):
        """
        Return the Hydrodynamic and aerodynamic force on the hull
        We neglect the aero dynamic force
        """
        return self.hydroAeroForce
    
    def getTotalForce(self):
        """
        Forces exerced on {hull}:
            - Aerodynamic
            - Hydrodynamic
            - Buoyency
            - Gravity
        """
        return self.getGravityForce() + self.getBuoyencyForce() + self.getHydroAeroForce()

    def getBottomHullCoord(self, alpha):
        """
        Return the position of the drift for a longitudinal percentage <alpha>
        Indeed, by symetry: yd = 0 and the drift must touch the hull, thus xd, zd = fx(alpha), fy(alpha)
        For alpha=1, the drift is at the stern
            alpha=0, the drift is at the bow
        """
        # start by doing a poly fit in the (xb,yb) plane
        lh = self.getGeomP('lh')
        hhmid = self.getGeomP('hhmid')
        hhend = self.getGeomP('hhend')
        polyCoef = np.polyfit([0, lh/2, lh], [0, hhmid, hhend], 2)
        hullInterpol = np.poly1d(polyCoef)

        return np.array([-alpha*lh + self.frontPos[0], 
                         0 + self.frontPos[1], 
                         -hullInterpol(alpha*lh) + self.frontPos[2]])
    

    def getTopHullCoord(self, alpha):
        """
        Return the position of the sail for a longitudinal percentage <alpha>
        """
        lh = self.getGeomP('lh')
        return np.array([-alpha*lh + self.frontPos[0], 
                         0 + self.frontPos[1], 
                         0 + self.frontPos[2]])


    def getPolygons(self):
        poly = []
        poly1 = []
        poly2 = []
        poly1b = []
        poly2b = []
        # (x0, y0, z0) Top left corner
        x0 = self.frontPos[0]
        y0 = self.frontPos[1]
        z0 = self.frontPos[2]
        # (lh, whmid, whend, hhmid, hhend)
        lh = self.getGeomP('lh')
        whmid = self.getGeomP('whmid')
        whend = self.getGeomP('whend')
        hhmid = self.getGeomP('hhmid')
        hhend = self.getGeomP('hhend')

        poly.append(np.array([x0, y0, z0]))
        poly.append(np.array([x0-lh/2, y0+whmid/2, z0]))
        poly.append(np.array([x0-lh, y0+whend/2, z0]))
        poly.append(np.array([x0-lh, y0-whend/2, z0]))
        poly.append(np.array([x0-lh/2, y0-whmid/2, z0]))
        poly.append(np.array([x0, y0, z0]))

        poly1.append(np.array([x0, y0, z0]))
        poly1.append(np.array([x0-lh/2, y0+whmid/2, z0]))
        poly1.append(np.array([x0-lh/2, y0, z0-hhmid]))
        poly1.append(np.array([x0, y0, z0]))

        poly2.append(np.array([x0-lh/2, y0+whmid/2, z0]))
        poly2.append(np.array([x0-lh, y0+whend/2, z0]))
        poly2.append(np.array([x0-lh, y0, z0-hhend]))
        poly2.append(np.array([x0-lh/2, y0, z0-hhmid]))
        poly2.append(np.array([x0-lh/2, y0+whmid/2, z0]))

        poly1b.append(np.array([x0, y0, z0]))
        poly1b.append(np.array([x0-lh/2, y0-whmid/2, z0]))
        poly1b.append(np.array([x0-lh/2, y0, z0-hhmid]))
        poly1b.append(np.array([x0, y0, z0]))

        poly2b.append(np.array([x0-lh/2, y0-whmid/2, z0]))
        poly2b.append(np.array([x0-lh, y0-whend/2, z0]))
        poly2b.append(np.array([x0-lh, y0, z0-hhend]))
        poly2b.append(np.array([x0-lh/2, y0, z0-hhmid]))
        poly2b.append(np.array([x0-lh/2, y0-whmid/2, z0]))
        
        return [poly, poly1, poly1b, poly2, poly2b]
    
import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from backend.PhysicalObject import *
from config.Config import *
from utils.Force import *
from backend.naca.NACACalculator import *

rudderDefaultGeom = {'lr':{'value':0.2, 'unit':Units.m, 'info':'Width of the rudder'},
                    'hr':{'value':0.3, 'unit':Units.m, 'info':'Height of the rudder'},
                    'er':{'value':0.004, 'unit':Units.m, 'info':'Tickness of the rudder'},
                    'lrp':{'value':0.0, 'unit':Units.m, 'info':'Longitudinal length of the protect rudder plate'},
                    'epsi':{'value':25, 'unit':Units.percent, 'info':'Relative longitudinal position of the hydrodynamic center of effort'},
                    'alpha':{'value':100, 'unit':Units.percent, 'info':'Relative longitudinal position (1=aft)'}, # Percentage of the longitudinal boat length
                    'rProfile':{'value':"NACA0015", 'unit':Units.none, 'info':'NACA Profile of the rudder'},
                    'prProfile':{'value':"NACA0015", 'unit':Units.none, 'info':'NACA Profile of the protect rudder'},
                    }

class Rudder(PhysicalObject):

    def __init__(self, solver, geom=rudderDefaultGeom):
        super().__init__(solver)
        self.updateGeom(geom)
        self.hydroForce = getANullForce(solver)
        self.buoyencyForce = getANullForce(solver)
        self.protectRudderForce = getANullForce(solver)
        self.rudderForce = getANullForce(solver)

    def getRotationPoint(self):
        """
        Return the rotation point of the rudder in the boat basis
        """
        return self.pos

    def updateGeomDependecies(self):
        """
        Update this component after the geometry was updated
        """
        # Get the position of the local referentiel depending on alpha and the hull shape
        self.pos = self._solver.getHull().getBottomHullCoord(self.getGeomP('alpha'))
        self.computeBuoyencyForce()

    def getPolygons(self):
        polyr = []
        polyp = []
        # (x0, y0, z0) Top left corner
        x0 = self.pos[0] - (1-self.getGeomP('epsi'))*self.getGeomP('lr')
        y0 = self.pos[1]
        z0 = self.pos[2]
        # Size of the rudder
        lr = self.getGeomP('lr')
        hr = self.getGeomP('hr')
        lrp = self.getGeomP('lrp')
        # Then create the polygon
        #   (Rudder)
        polyr.append(np.array([x0, y0, z0]))
        polyr.append(np.array([x0+lr, y0, z0]))
        polyr.append(np.array([x0+lr, y0, z0-hr]))
        polyr.append(np.array([x0, y0, z0-hr]))
        polyr.append(np.array([x0, y0, z0]))
        #   (Protect Rudder)
        x0 += lr
        polyp.append(np.array([x0, y0, z0]))
        polyp.append(np.array([x0+lrp, y0, z0]))
        polyp.append(np.array([x0, y0, z0-hr]))
        polyp.append(np.array([x0, y0, z0]))

        # Finaly apply the rotation of angle getRudderAng() and axis [(x0,y0,z0),z]
        polyr = rotation(polyr, self.getRotationPoint(), Dir.Z, self._solver.getRudderAng())

        return [polyr, polyp]
    

    def getTotalForce(self):
        """
        Forces exerced on {rudder}:
            - Hydrodynamic
            - Gravity
            - Buoyency
        """
        return self.getGravityForce() + self.getBuoyencyForce() + self.getHydrodynamicForce()
    
    def getMovingSurface(self):
        """
        Return the surface of the moving part
        """
        return self.getGeomP('lr')*self.getGeomP('hr')
    
    def getProtectRudderSurface(self):
        """
        Return the surface of the protect rudder
        """
        return self.getGeomP('lrp')*self.getGeomP('hr')/2
    
    def getSurface(self):
        """
        Return the total surface
        """
        return self.getMovingSurface() + self.getProtectRudderSurface()
    
    def getRudderVolume(self):
        """
        Return the volume of the rudder
        """
        return self.getMovingSurface()*self.getGeomP('er')
    
    def getProtectRudderVolume(self):
        """
        Return the volume of the protect rudder
        """
        return self.getProtectRudderSurface()*self.getGeomP('er')
    
    def computeBuoyencyForce(self):
        """
        Compute the buyency force on this object
        and the center of buyency of the rudder+protect rudder
        """

        # bary centre, homogenous material: cdc = cdg
        vrudder = self.getRudderVolume()
        vprudder = self.getProtectRudderVolume()
        vrudderObj = vrudder + vprudder
        cdc = (self.getCdgRudder()*vrudder + self.getCdgProtectRudder()*vprudder)/vrudderObj

        # deduce
        self.buoyencyForce = Force(Vector(self._solver, -PHY_G_VECTOR*PHY_RHO_SWATER*vrudderObj, Base.SEA), cdc)
    

    def getCdg(self):
        """
        Return the center of gravity of this object
        """
        mrudder = self.getRudderVolume()*PHY_RHO_RUDDER
        mprudder = self.getProtectRudderVolume()*PHY_RHO_PROTECT_RUDDER
        cgdRudder = self.getCdgRudder()
        cgdProtectRudder = self.getCdgProtectRudder()
        cdg = (cgdRudder*mrudder + cgdProtectRudder*mprudder)/(mprudder + mrudder)
        return cdg

    def getCdgRudder(self):
        """
        Return the position of the center of gravity of the rudder
        """
        cg = np.array([(self.getGeomP('epsi')-0.5)*self.getGeomP('lr'), 0, -self.getGeomP('hr')/2])
        cg += self.pos # from rudder ref to boat ref
        return Point(self._solver, cg, Referential.BOAT)

    def getCdgProtectRudder(self):
        """
        Return the position of the cdg of the protect rudder
        """
        cg = np.array([(self.getGeomP('epsi')+1/3)*self.getGeomP('lr'), 0, -self.getGeomP('hr')/3])
        cg += self.pos # from rudder ref to boat ref
        return Point(self._solver, cg, Referential.BOAT)

    def getWeight(self):
        """
        Return the weight of this object
        """
        mrudder = self.getRudderVolume()*PHY_RHO_RUDDER
        mprudder = self.getProtectRudderVolume()*PHY_RHO_PROTECT_RUDDER
        return mrudder + mprudder
    
    def computeRudderHydrodynamicForce(self):
        """
        Compute the Rudder hydrodynamic force
        """
        # find the postion of the origin of the rudder (rotative part)
        rudderOrigin = self.pos.copy()
        lr = self.getGeomP('lr')
        hr = self.getGeomP('hr')
        
        def linepf(t):
            """
            Return the position of the profile for a parametrized coordinate t
            """
            return Point(self._solver, rudderOrigin+t*np.array([0, 0, -hr]), Referential.BOAT)

        def lengthpf(t):
            """
            Return the length of the profile for a parametrized coordinate t
            """
            return lr

        self.rudderForce = self._solver.getNACACalculator().getFluidForce(linepf, lengthpf, Fluids.WATER, Referential.RUDDER,  Referential.SEA, self.getGeomP('rProfile'))
    
    def computeProtectRudderHydrodynamicForce(self):
        """
        Compute the Protect Rudder hydrodynamic force
        """
        epsi = self.getGeomP('epsi')
        lr = self.getGeomP('lr')
        hr = self.getGeomP('hr')
        lrp = self.getGeomP('lrp')
        protectRudderOrigin = self.pos.copy() + np.array([1, 0, 0])*(epsi*lr + (1-epsi)*lrp)
        
        def linepf(t):
            """
            Return the position of the profile for a parametrized coordinate t
            """
            return Point(self._solver, protectRudderOrigin+t*np.array([-(1-epsi)*lrp, 0, -hr]), Referential.BOAT)

        def lengthpf(t):
            """
            Return the length of the profile for a parametrized coordinate t
            """
            return lrp*(1-t)

        self.protectRudderForce = self._solver.getNACACalculator().getFluidForce(linepf, lengthpf, Fluids.WATER, Referential.BOAT,  Referential.SEA, self.getGeomP('prProfile'))
        
    def computeHydrodynamicForce(self):
        """
        Compute the hydrodynamic force on this object
        """
        # FIRST : RUDDER FORCE
        self.computeRudderHydrodynamicForce()
        
        # SECOND : PROTECT RUDDER FORCE
        self.computeProtectRudderHydrodynamicForce()
        
        self.hydroForce = self.rudderForce + self.protectRudderForce

    def getBuoyencyForce(self):
        return self.buoyencyForce
    
    def getHydrodynamicForce(self):
        return self.hydroForce
    
    def getRudderHydrodynamicForce(self):
        return self.rudderForce
    
    def getProtectRudderHydrodynamicForce(self):
        return self.protectRudderForce

    def compute(self):
        """
        Compute the force on this component for the current step
        """
        #self.computeBuoyencyForce() Static force!
        self.computeHydrodynamicForce()

    def getInertia(self):
        """
        Return the inertia matrix of this component
        We omit the protect rudder
        """
        matrix = Matrix(self._solver, getRectangularPrismInertia(self.getGeomP('lr'),
                                                                 self.getGeomP('lr')/6,
                                                                 self.getGeomP('hr'),
                                                                 self.getRudderVolume()*PHY_RHO_RUDDER), Base.BOAT)
        return Inertia(matrix, self.getCdgRudder(), self.getRudderVolume()*PHY_RHO_RUDDER)
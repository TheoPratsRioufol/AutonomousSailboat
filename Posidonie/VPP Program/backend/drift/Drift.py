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
from backend.drift.BulbForceCalculator import *


driftDefaultGeom = {'ld':{'value':0.3, 'unit':Units.m, 'info':'Width of the drift'},
                    'hd':{'value':1, 'unit':Units.m, 'info':'Heigth of the drift'},
                    'gd':{'value':30, 'unit':Units.deg, 'info':'Slope angle of the drift'},
                    'Ld':{'value':0.3, 'unit':Units.m, 'info':'Drift Base extension'},
                    'ed':{'value':0.025, 'unit':Units.m, 'info':'Tickness of the drift'},
                    'mLest':{'value':50, 'unit':Units.kg, 'info':'Weight of the keel bulb'},
                    'alpha':{'value':40, 'unit':Units.percent, 'info':'Relative longitudinal position (1=aft)'}, # Percentage of the longitudinal boat length
                    'epsi':{'value':25, 'unit':Units.percent, 'info':'Relative longitudinal position of the aerodynamic center of effort'},
                    'profile':{'value':"NACA0015", 'unit':Units.none, 'info':'NACA Profile of the drift'}
                    }

class Drift(PhysicalObject):

    def __init__(self, solver, geom=driftDefaultGeom):
        super().__init__(solver)
        self.updateGeom(geom)

        # Object to compute the force of the bulb
        self.bulbForceCalculator = BulbForceCalculator(solver)

        self.hydroForce = getANullForce(solver)
        self.buoyencyForce = getANullForce(solver)
        self.bulbForce = getANullForce(solver)

    def updateGeomDependecies(self):
        """
        Update this component after the geometry was updated
        """
        # Get the position of the local referentiel depending on alpha and the hull shape
        self.pos = self._solver.getHull().getBottomHullCoord(self.getGeomP('alpha'))
        self.computeBuoyencyForce()

        # Compute the center of effort
        self.computeDefaultCenterOfEffort()

        # Verify than Ld is possible
        dd = np.tan(self.getGeomP('gd')) * self.getGeomP('hd')
        if (dd < self.getGeomP('Ld')):
            raise Exception("The Value of Ld is too big. Must be inforior to 'tan(gd)*hd'")

    def getPolygons(self):
        poly = []
        # (x0, y0, z0) Top right corner
        x0 = self.pos[0] + (self.getGeomP('ld') + self.getGeomP('Ld'))*self.getGeomP('epsi')
        y0 = self.pos[1]
        z0 = self.pos[2]
        # tan(gd) ) dd/hd
        dd = np.tan(self.getGeomP('gd')) * self.getGeomP('hd')
        # (h, w) height and width
        h = self.getGeomP('hd')
        w = self.getGeomP('ld')
        Ld = self.getGeomP('Ld')
        h0 = min(Ld * np.tan(np.pi/2 - self.getGeomP('gd')), h)
        # Then create the polygon
        poly.append(np.array([x0, y0, z0]))
        poly.append(np.array([x0-w-Ld, y0, z0]))
        poly.append(np.array([x0-w-Ld, y0, z0-h0]))
        poly.append(np.array([x0-w-dd, y0, z0-h]))
        poly.append(np.array([x0-dd, y0, z0-h]))
        poly.append(np.array([x0, y0, z0]))

        return [poly]
    

    def getTotalForce(self):
        """
        Forces exerced on {drift}:
            - Hydrodynamic
            - Gravity
            - Buoyency
        """
        return self.getGravityForce() + self.getBuoyencyForce() + self.getHydrodynamicForce() + self.getBulbForce()
    
    def getPlateVolume(self):
        """
        Return the volume of the plate
        """
        return self.getSurface()*self.getGeomP('ed')
    
    def getPlateASurface(self):
        """
        Return the surface of the plate (A)
        """
        return self.getGeomP('ld')*self.getGeomP('hd')
    
    def getPlateBSurface(self):
        """
        Return the surface of the plate (B)
        """
        return self.getGeomP('Ld')*np.tan(np.pi/2 - self.getGeomP('gd'))*self.getGeomP('Ld')/2
    
    def getLestVolume(self):
        """
        Return the volume of the lest
        """
        return self.getGeomP('mLest')/PHY_RHO_KEEL
    
    def computeBuoyencyForce(self):
        """
        Compute the buyency force on this object
        and the center of buyency of the drift+keel
        """

        # bary centre, homogenous material: cdc = cdg
        vplate = self.getPlateVolume()
        vlest = self.getLestVolume()
        VdriftKeel = vplate + vlest
        cdc = (self.getCdgPlate()*vplate + self.getCdgLest()*vlest)/(VdriftKeel)

        # deduce
        self.buoyencyForce =  Force(Vector(self._solver, -PHY_G_VECTOR*PHY_RHO_SWATER*VdriftKeel, Base.SEA), cdc)
    

    def getCdg(self):
        """
        Return the center of gravity of this object
        """
        mplate = self.getPlateMass()
        mLest = self.getGeomP('mLest')
        cgdPlate = self.getCdgPlate()
        cgdLest = self.getCdgLest()
        cdg = (cgdPlate*mplate + cgdLest*mLest)/(mLest + mplate)
        return  cdg

    def getCdgPlate(self):
        """
        Return the position of the center of gravity of the plate
        """
        dd = np.tan(self.getGeomP('gd')) * self.getGeomP('hd')
        epsi = self.getGeomP('epsi')
        ld = self.getGeomP('ld')
        hd = self.getGeomP('hd')
        Ld = self.getGeomP('Ld')
        h0 = np.tan(np.pi/2 - self.getGeomP('gd')) * Ld

        # First, compute the cdg of the 'a' part
        cg_a = np.array([epsi*ld - ld/2 - dd/2, 0, -hd/2])
        cg_a += self.pos # from drift ref to boat ref

        # then the 'b' part
        cg_b = np.array([epsi*(ld + Ld) - ld - Ld + Ld/3, 0, -h0/3])
        cg_b += self.pos # from drift ref to boat ref

        s_a = self.getPlateASurface()
        s_b = self.getPlateBSurface()
        cdg = (cg_a*s_a + cg_b*s_b)/(s_a + s_b)

        return Point(self._solver, cdg, Referential.BOAT)

    def getCdgLest(self):
        """
        Return the position of the cdg of the lest
        """
        dd = np.tan(self.getGeomP('gd')) * self.getGeomP('hd')
        cg = np.array([-(self.getGeomP('ld')/2+dd), 0, -self.getGeomP('hd')])
        cg += self.pos # from drift ref to boat ref
        return Point(self._solver, cg, Referential.BOAT)
    
    def getPlateMass(self):
        """
        Return the mass of the plate
        """
        return self.getPlateVolume()*PHY_RHO_DRIFT

    def getWeight(self):
        """
        Return the weight of this object
        """
        return self.getGeomP('mLest') + self.getPlateMass()
    
    def computeHydrodynamicForce(self):
        """
        Compute the hydrodynamic force on this object
        """
        self.hydroForce = self._solver.getNACACalculator().getFluidForce(self.linepf, self.lengthpf, Fluids.WATER, Referential.BOAT,  Referential.SEA, self.getGeomP('profile'))

    def linepf(self, t):
        """
        Return the position of the profile for a parametrized coordinate t
        """
        # first, find the postion of the origin of the drift
        driftOrigin = self.pos.copy()

        # Main geometric parameters:
        dd = np.tan(self.getGeomP('gd')) * self.getGeomP('hd')
        hd = self.getGeomP('hd')
        ld = self.getGeomP('ld')
        Ld = self.getGeomP('Ld')
        ep = self.getGeomP('epsi')
        h0 = np.tan(np.pi/2 - self.getGeomP('gd')) * Ld
        t0 = h0/hd # limit betwen the lower and upper part of the drift

        x_start = 0
        x_mid   = ep*(ld + Ld) - Ld - ep*ld
        x_end   = ep*(ld + Ld) - ep*ld -dd

        if (t0 != 0):
            if (t <= t0):
                ppos = [(t/t0)*(x_mid - x_start) + x_start, 0, -t*hd]
            else:
                ppos = [(t-t0)/(1-t0)*(x_end - x_mid) + x_mid, 0, -t*hd]
        else:
            ppos = [t*(x_end - x_start) + x_start, 0, -t*hd]

        return Point(self._solver, driftOrigin+np.array(ppos), Referential.BOAT)

    def lengthpf(self, t):
        """
        Return the length of the profile for a parametrized coordinate t
        """
        # Main geometric parameters:
        hd = self.getGeomP('hd')
        ld = self.getGeomP('ld')
        Ld = self.getGeomP('Ld')
        h0 = np.tan(np.pi/2 - self.getGeomP('gd')) * Ld
        t0 = h0/hd # limit betwen the lower and upper part of the drift

        if (t < t0):
            return ld + (1 - t/t0) * Ld
        else:
            return ld

    def computeBulbForce(self):
        """
        Compute the hydrodynamic force of the bulb
        """
        self.bulbForce = self.bulbForceCalculator.getForce()

    def getSurface(self):
        """
        Return the surface of the drift
        """
        return self.getPlateASurface() + self.getPlateBSurface()

    def computeDefaultCenterOfEffort(self):
        """
        Compute the default Center Of Effort
        """
        N = 40 # Number of integration sample
        hd = self.getGeomP('hd')
        dz = hd/N
        cumulativeSurface = 0
        medSurface = self.getSurface()/2 # Half Surface

        for t in np.linspace(0, 1, N):
            cumulativeSurface += self.lengthpf(t)*dz
            if (cumulativeSurface > medSurface):
                self.defaultCenterOfEffort = self.linepf(t)
                return
            
        self.defaultCenterOfEffort = self.linepf(1)

    def getDefaultCenterOfEffort(self):
        """
        Return the default Center Of Effort
        """
        return self.defaultCenterOfEffort

    def getBuoyencyForce(self):
        return self.buoyencyForce
    
    def getHydrodynamicForce(self):
        return self.hydroForce
    
    def getBulbForce(self):
        return self.bulbForce

    def compute(self):
        """
        Compute the force on this component for the current step
        """
        #self.computeBuoyencyForce() Static force!
        self.computeHydrodynamicForce()
        self.computeBulbForce()

    def getInertia(self):
        """
        Return the inertia matrix of this component
        """
        # Hyp: ponctual mass at the tip of the keel
        
        # Compute the inertia of the bulb
        bulbMatrix = Matrix(self._solver, np.zeros(3), Base.BOAT)
        bulbInertia = Inertia(bulbMatrix, self.getCdgLest(), self.getGeomP('mLest'))

        # Compute the inertia of the drift plate
        plateMatrix = Matrix(self._solver, np.zeros(3), Base.BOAT) # TODO
        plateInertia = Inertia(plateMatrix, self.getCdgPlate(), self.getPlateMass())

        return bulbInertia + plateInertia

import time

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from backend.drift.Drift import *
from backend.sail.Sail import *
from backend.hull.Hull import *
from backend.rudder.Rudder import *
from backend.solver.Navigator import *
from backend.solver.StateVector import *
from backend.Wind import *
from utils.Force import *
from config.Config import *
from backend.naca.NACACalculator import *
from utils.Utils import *

solverDefaultGeom = {'maxRudderAng':{'value':30, 'unit':Units.deg, 'info':'Maximum rudder angle'}}


"""

    State vector of the boat:

        - Boat Speed
        - Boat Pos
        - Boat RotSpeed
        - Boat RotPos
    
    Command vector of the boat

        - Sail ang
        - Rudder ang
        - Wind Vector

"""

class Solver():
    """
    This class implement all the computation to simulate the boat
        Provide the construction of the state vector X
        And the EDO Function F
    """

    def __init__(self):
        self.updateGeom(solverDefaultGeom)

        # For the linkage forces:
        self.FhullToComp = {}
        self.MhullToComp = {}
        
        # NACA calculator used for the drift and sail
        self.NACACalculator = NACACalculator(self)

        # Create a navigator
        self._navigator = Navigator(self)

        # Create the environement
        self._wind = Wind(self)

        # Dummy cdg just for loading
        self.cdg = Point(self, np.zeros(3), Referential.BOAT)

        # Dummy acceleration for loading
        self.boatAcceleration = PointAcceleration(Vector(self, np.zeros(3), Base.BOAT), self.getBoatCdg(), Referential.BOAT, Referential.SEA)

        # Load an initial state vector
        X0 = self.getX0()
        U0 = self._navigator.getInitU()
        self.loadStateVector(X0, U0)

        # Then, build the components of the boat
        self._hull = Hull(self)
        self._drift = Drift(self)
        self._sail = Sail(self)
        self._rudder = Rudder(self)

        # Update the geometry dependencies
        self.updateGlobalGeom()

        # Build usefull parameter for the simulation

        # Coef. friction for rotation
        self.alphaRotation = [60, 50, 20]


    def F(self, X, U):
        """
        Compute the derivative of the state vector X, giving the state vector X
        and the command U
        """
        # Set the state vector and the command in this component
        # i.e. load the state intot the simulator variables
        self.loadStateVector(X, U)

        # Compute the force for each components
        self.compute()
        
        # Compute the acceleration and rotation acceleration
        forceAndMoment = self.getBoatForceAndMoment()

        force  = forceAndMoment.getVec()
        moment = forceAndMoment.getOriginMoment().getVec()
        
        # FIRST, SOLVE NEWTON EQUATION
        accel = force/self.mass
        self.boatAcceleration = PointAcceleration(accel, self.getBoatCdg(), Referential.BOAT, Referential.SEA)

        # THEN THE EULER EQUATION FOR ANGLES
        momentSum = (self.getBoatOrigin() - self.getBoatCdg()).vectorial(self.boatAcceleration.getAccel())*(-self.mass)
        #momentSum = moment
        momentSum = -momentSum.valueIn(Base.BOAT) -moment.valueIn(Base.BOAT) - self.getBoatRotSpeed().getOmega().valueIn(Base.BOAT)*self.alphaRotation
        boatRotAcceleration = np.matmul(self.invInertia, momentSum)

        # Return the state vector (each component is the derivative or the original state vector)
        newX = StateVectorDerivative(X._x_speed,
                                     self.boatAcceleration,
                                     X._x_omega,
                                     boatRotAcceleration)
        
        # Finish by computing the linkage forces
        self.computeLikageForce()
        
        return newX
    
    def compute(self):
        """
        Compute the force for each components
        """
        self.getSail().compute()
        self.getDrift().compute()
        self.getRudder().compute()
        self.getHull().compute()

    def loadStateVector(self, X, U=None):
        """
        Load the state vector and command in this component
        """
        if U != None:
            # define the controlable surface value
            self.U = U
            # Update the wind
            self.getWind().updateFromCommand(U)

        # define the state vector
        self.X = X

        # Compute the Passing Matrix
        self.computePassingMatrix()

        # Compute the velocity fields
        self.computeVelocityField()

        # Compute the acceleration field
        self.computeAccelerationField()

    def getCapsizingMoment(self):
        """
        Return the moments reponsibles for capsizing
        """
        moment = self.getSail().getTotalForce()
        moment  +=  self.getHull().getTotalForce()
        moment += self.getRudder().getTotalForce()
        moment += self.getDrift().getTotalForce()
        return  moment.getOriginMoment()

    def getBoatForceAndMoment(self):
        """
        Return the total Force And Moment exerced on the boat
        """
        force =  self.getHull().getTotalForce()
        force += self.getSail().getTotalForce()
        force += self.getDrift().getTotalForce()
        force += self.getRudder().getTotalForce()
        return force
    
    def getBoatMass(self):
        """
        Return the mass of the boat
        """
        return self.mass
    
    def getBoatCdg(self):
        """
        Return the Cdg of the boat
        """
        return self.cdg

    def computeBoatMassAndCdg(self):
        """
        Compute the boat total mass and Center of gravity
        """
        mhull  = self.getHull().getWeight()
        mdrift = self.getDrift().getWeight()
        msail  = self.getSail().getWeight()
        mrudder = self.getRudder().getWeight()

        self.cdg  = self.getHull().getCdg()*mhull
        self.cdg += self.getDrift().getCdg()*mdrift
        self.cdg += self.getSail().getCdg()*msail
        self.cdg += self.getRudder().getCdg()*mrudder

        self.mass = msail + mdrift + mhull + mrudder
        self.cdg = self.cdg / self.mass

    def computeAndGetComponentsCdg(self):
        """
        Compute the position of the component's Cdg excluding the hull
        """
        mdrift = self.getDrift().getWeight()
        msail  = self.getSail().getWeight()
        mrudder = self.getRudder().getWeight()

        cdg = self.getDrift().getCdg()*mdrift
        cdg += self.getSail().getCdg()*msail
        cdg += self.getRudder().getCdg()*mrudder

        mass = msail + mdrift + mrudder
        return cdg / mass


    def updateGeom(self, geom):
        """
        Update the 'geometry' of the solver
        """
        self.geom = geom

    def updateGlobalGeom(self, geom=None):
        """
        Update the geometry of the boat + solver
        """
        if (geom != None):
            # Update the geometry of the components
            if ('sail' in geom):
                self.getSail().updateGeom(geom['sail'])
            if ('drift' in geom):
                self.getDrift().updateGeom(geom['drift'])
            if ('rudder' in geom):
                self.getRudder().updateGeom(geom['rudder'])
            if ('wind' in geom):
                self.getWind().updateGeom(geom['wind'])
            if ('navigator' in geom):
                self.getNavigator().updateGeom(geom['navigator'])
            if ('solver' in geom):
                self.updateGeom(geom['solver'])

        # Recompute the mass, inertia matrix and center of gravity
        self.computeBoatMassAndCdg()
        self.computeBoatInertia()

    def getGlobalGeom(self):
        """
        Return the global geometry dictionnary
        """
        return {'sail':self.getSail().getGeom(),
                'drift':self.getDrift().getGeom(),
                'rudder':self.getRudder().getGeom(),
                'wind':self.getWind().getGeom(),
                'navigator':self.getNavigator().getGeom(),
                'solver':self.getGeom()}
    
    def getGeom(self):
        """
        Return the geometry of this object
        """
        return self.geom

    def computeBoatInertia(self):
        """
        Update the boat inertia matrix
        """
        # Add the inertia matrix. By default, there are computed at the origin,
        # in BOAT basis. Nothing to do thus
        #self.inertia = self.getHull().getInertia().getMatrix().valueIn(Base.BOAT)
        #self.inertia *= 10
        self.inertia  = self.getHull().getInertia()
        self.inertia += self.getDrift().getInertia()
        self.inertia += self.getSail().getInertia()

        # Then compute the matrix:
        inertiaMatrix = self.inertia.getMatrix().valueIn(Base.BOAT)
        # Prepare the inverse inertia
        self.invInertia = np.linalg.inv(inertiaMatrix)

    def getX0(self, X=None):
        """
        Return the initial state vector of the boat
        If X != None, make a smouth transition
        """
        x0 = np.zeros(3)
        v0 = Vector(self, np.zeros(3), Base.SEA)
        a0 = np.zeros(3)
        r0 = Vector(self, np.zeros(3), Base.BOAT)
        if (X != None):
            x0 = X._x_pos
            v0 = X._x_speed._speed
            a0 = X._x_ang
            r0 = X._x_omega.getOmega()
                  
        X0 = StateVector(x0, # initial position of the boat
                         PointSpeed(self.getBoatCdg(), 
                            v0, 
                            Referential.BOAT, 
                            Referential.SEA), # initial velocity at the center of rotation relative to the sea
                         a0, # initial orientation of the boat
                         AngularSpeed(r0, 
                              Referential.BOAT, 
                              Referential.SEA) # initial Angular velocity of the boet relative to the sea
        )
        return X0
    
    def getState(self):
        """
        Return the state vector of the boat
        """
        return self.X.copy()

    """
        ===========================================================
                      GETTERS OF THE BOAT STATE
        ===========================================================
    """

    def getBoatAng(self):
        return self.X._x_ang
    
    def getSailAng(self):
        return self.U._u_sail
    
    def getRudderAng(self):
        return np.clip(self.U._u_rudder, -self.getGeomP('maxRudderAng'), self.getGeomP('maxRudderAng'))
    
    def getBoatPos(self):
        return self.X._x_pos

    def getBoatSpeed(self):
        return self.X._x_speed
    
    def getBoatRotSpeed(self):
        return self.X._x_omega
    
    def getBoatAccel(self):
        return self.boatAcceleration
    
    """
        ==================================================
                        COMPONENTS GETTERS
        ==================================================
    
    """

    def getBoatInertia(self):
        """
        Return the boat global inertia matrix at the origin
        """
        return self.inertia

    def getComponentByName(self, name):
        """
        Return a component from a name
        """
        if (name == 'sail'):
            return self.getSail()
        elif (name == 'rudder'):
            return self.getRudder()
        elif (name == 'drift'):
            return self.getDrift()
        elif (name == 'hull'):
            return self.getHull()
        else:
            raise Exception("Unknow component: "+str(name))

    def getHull(self):
        """
        Return the hull component
        """
        return self._hull

    def getDrift(self):
        """
        Return the drift component
        """
        return self._drift
    
    def getSail(self):
        """
        Return the sail component
        """
        return self._sail
    
    def getRudder(self):
        """
        Return the rudder component
        """
        return self._rudder
    
    def getWind(self):
        """
        Return the wind component
        """
        return self._wind
    
    def getNACACalculator(self):
        """
        Return the NACA calculator
        """
        return self.NACACalculator
    
    def getNavigator(self):
        """
        Return the navigator object
        """
        return self._navigator
    
    def getBoatOrigin(self):
        """
        Return the origin point of the boat
        """
        return Point(self, np.zeros(3), Referential.BOAT)
    
    """
        FIELDS
    """
    
    def computeVelocityField(self):
        """
        Compute all the velocity field at this step
        """
        # Compute the BOAT velocity field
        self.boatVelocityField = VelocityField(self.getBoatSpeed(), self.getBoatRotSpeed())
        
        # Compute the SAIL velocity field
        # We neglect the dynamic of the sail rotation
        self.sailVelocityField = self.getVelocityField(Referential.BOAT)
        
        # Compute the RUDDER velocity field
        # We neglect the dynamic of the rudder rotation
        self.rudderVelocityField = self.getVelocityField(Referential.BOAT)
        
        # Compute the WIND velocity field
        windPointSpeed = PointSpeed(self.getBoatOrigin(), self.getWind().getSpeedVec()*(-1), Referential.WIND, Referential.SEA)
        windAngSpeed   = AngularSpeed(Vector(self, np.zeros(3), Base.SEA), Referential.WIND, Referential.SEA)
        self.windVelocityField = VelocityField(windPointSpeed, windAngSpeed)
        
        # Compute the SEA velocity field
        seaPointSpeed = PointSpeed(self.getBoatOrigin(), Vector(self, np.zeros(3), Base.SEA), Referential.SEA, Referential.SEA)
        seaAngSpeed   = AngularSpeed(Vector(self, np.zeros(3), Base.SEA), Referential.SEA, Referential.SEA)
        self.seaVelocityField = VelocityField(seaPointSpeed, seaAngSpeed)
    
    def getVelocityField(self, R):
        """
        Return the velocity field associated with a referential + state and command vector
        """
        if (R == Referential.BOAT):
            return self.boatVelocityField
        
        if (R == Referential.SAIL):
            return self.sailVelocityField
        
        if (R == Referential.RUDDER):
            return self.rudderVelocityField
        
        if (R == Referential.WIND):
            return self.windVelocityField
        
        if (R == Referential.SEA):
            return self.seaVelocityField
        
        raise Exception("Unable to found a VelocityField for the ref: "+str(R.value))
    
    def computeAccelerationField(self):
        """
        Compute the boat's acceleration field
        """
        self.accelerationField = AccelerationField(self.boatAcceleration, self.getBoatRotSpeed())

    def getAccelerationField(self):
        """
        Return the boat's acceleration field
        """
        return self.accelerationField
    
    """
        LINKAGE FORCES
    """

    def computeLikageForce(self):
        """
        Compute the linkage force.
        Always the force HULL --> COMPONENT
            (1) atot * mtot   = Ftot
            (2) acomp * mcomp = Fcomp + F(hull->comp)
            => F(hull->comp) = acomp * mcomp - Fcomp
        """
        # Prepare the linkage moments quantity
        xO = self.getBoatOrigin()
        xOaccel = self.accelerationField.at(xO)
        omega = self.getBoatRotSpeed().getOmega().valueIn(Base.BOAT)

        # for each components...
        for cname in ['sail', 'rudder']:
            # Compute the linkage force
            self.FhullToComp[cname] = self.accelerationField.at(self.getComponentByName(cname).getCdg()).getAccel() * self.getComponentByName(cname).getWeight() - self.getComponentByName(cname).getTotalForce().getVec()
        
            # Compute the linkage moment
            self.MhullToComp[cname] = self.getComponentByName(cname).getInertia().getMatrix().valueIn(Base.BOAT).dot(omega) + self.getComponentByName(cname).getWeight()*(self.getComponentByName(cname).getCdg() - xO).vectorial(xOaccel.getAccel()).valueIn(Base.BOAT) - self.getComponentByName(cname).getTotalForce().getOriginMoment().valueIn(Base.BOAT)
            self.MhullToComp[cname] = Vector(self, self.MhullToComp[cname], Base.BOAT)

    def getHullToCompForce(self, cname):
        """
        Return the force: F(hull -> "cname")
        """
        if (cname not in self.FhullToComp):
            return None
        
        return self.FhullToComp[cname]
    
    def getHullToCompMoment(self, cname):
        """
        Return the moment: M(hull -> "cname")
        """
        if (cname not in self.MhullToComp):
            return None
        
        return self.MhullToComp[cname]
    

    """
        =============================================================
        ==                 PASSSING MATRIX ZONE                    ==
        =============================================================
    """
            
    def computePassingMatrix(self):
        """
        Compute all the passing matrix
        """
        M = getMatRot(Dir.Z, self.getBoatAng()[2])
        M = np.matmul(M, getMatRot(Dir.X, self.getBoatAng()[0]))
        M = np.matmul(M, getMatRot(Dir.Y, self.getBoatAng()[1]))

        self.boatToSeaMatrix = M
        # Othogonal matrices: the transposed one is equal to the inverse
        self.seaToBoatMatrix = np.transpose(self.boatToSeaMatrix)
        #self.seaToBoatMatrix = np.linalg.inv(self.boatToSeaMatrix)

        self.sailToBoatMatrix = getMatRot(Dir.Z, self.getSailAng())
        self.boatToSailMatrix = np.transpose(self.sailToBoatMatrix)
        #self.boatToSailMatrix = np.linalg.inv(self.sailToBoatMatrix)

        self.rudderToBoatMatrix = getMatRot(Dir.Z, self.getRudderAng())
        self.boatToRudderMatrix = np.transpose(self.rudderToBoatMatrix)
        #self.boatToRudderMatrix = np.linalg.inv(self.rudderToBoatMatrix)

        self.seaToSailMatrix = np.matmul(self.boatToSailMatrix, self.seaToBoatMatrix)

    def getBoatToSeaMatrix(self):
        """
        Return the boat to sea basis matrix
        """
        return self.boatToSeaMatrix
    
    def getSeaToBoatMatrix(self):
        """
        Return the sea to boat basis matrix
        """
        return self.seaToBoatMatrix
    
    def getSailToBoatMatrix(self):
        """
        Return the sail to boat basis matrix
        """
        return self.sailToBoatMatrix

    def getBoatToSailMatrix(self):
        """
        Return the boat to sail basis matrix
        """
        return self.boatToSailMatrix
    
    def getRudderToBoatMatrix(self):
        """
        Return the sail to boat basis matrix
        """
        return self.rudderToBoatMatrix

    def getBoatToRudderMatrix(self):
        """
        Return the boat to sail basis matrix
        """
        return self.boatToRudderMatrix
    
    def getSeaToSailMatrix(self):
        """
        Return the sea to sail basis matrix
        """
        return self.seaToSailMatrix
    
    def warning(self, warning):
        raise Exception(warning)
    
    def getGeomP(self, name):
        """
        Return the value of the geometric parameter named 'name'
        """
        return Units.toSI(self.geom[name]['value'], self.geom[name]['unit'])

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *
from utils.Utils import *
from backend.solver.CommandVector import *
from backend.solver.EDOSolver import *

navigatorDefaultGeom = {
                        'k':{'value':1, 'unit':Units.none, 'info':'Proportional coeefficient of the PID'},
                        'ki':{'value':0.5, 'unit':Units.none, 'info':'Integral coeefficient of the PID'},
                        'i':{'value':15, 'unit':Units.deg, 'info':'Target sail incidence angle upwind'},
                        'on':{'value':1, 'unit':Units.none, 'info':'Set to 0 to turn off the navigator'},
                        'tau':{'value':1, 'unit':Units.s, 'info':'Caracteristic time of the sail controller'}
                       }

defaultT0 = {'target':0,
             'wang':np.deg2rad(45),
             'wspeed':noeud2ms(3)}

T0Units = {'target':{'unit':'deg',
                     'f':np.deg2rad},
            'wang':{'unit':'deg',
                    'f':np.deg2rad},
            'wspeed':{'unit':'knt',
                      'f':noeud2ms}}

# state vector for the command
S0pState = { 'step':0,
            't0':0,
            'y0':0,
            'lastSteadyHeading':0,
            'lastSteadyAng':0,
            'lastSteadyTime':0}

Epsi0 = np.zeros(2)

def T0Fct(X, S, t):
    return defaultT0, S

class Navigator():
    def __init__(self, solver):
        self._solver = solver
        self.updateGeom(navigatorDefaultGeom)
        self._TFct = T0Fct # Constant target by default

    def setTargetFct(self, TFct):
        """
        Set the target function
        """
        self._TFct = TFct
    
    def getInitU(self):
        """
        Return the init command (used when solver object is build)
        """
        return CommandVector(0, 0, 0, 0)
    
    def getU0(self):
        """
        Return the init command (used when solver object is build)
        """
        return self.getInitU()
    
    def getEpsi0(self):
        """
        Return the initial integrated error
        """
        return Epsi0
    
    def updateGeom(self, geom):
        """
        Update the geometry of this component (= the parameters)
        """
        self.geom = geom

    def getGeomP(self, name):
        """
        Return the value of the geometric parameter named 'name'
        """
        return Units.toSI(self.geom[name]['value'], self.geom[name]['unit'])

    def T(self, X, S, t):
        """
        Target function. Give the target as a function of:
            - the time (t)
            - the state of the boat (X)
            - the last target (s)
        Return:
            - The target T
            - The state vector of the target S
        """
        try:
            return self._TFct(X, S, t)
        except SimulatorException as e:
            raise e

    def Fu(self, X, Epsi, T, Ulast):
        """
        Compute the command U from:
            - The boat state vector X
            - The Epsi (integrated error)
            - The target T
            - The last command Ulast
        """
        self._solver.loadStateVector(X)
        if (self.getGeomP('on') == 0):
            return CommandVector(0, 0, T['wang'], T['wspeed']), np.array([0, 0])

        # FIRST, COMPUTE THE COMMAND FOR THE RUDDER
            
        # Get the data angles
        target  = T['target']

        # Compute the true heading from the speed vector
        true_heading = X._x_speed.getHeading()
        error = getAngleDif(target, true_heading)
        rudder_ang = error*self.getGeomP('k') + Epsi[0]*self.getGeomP('ki') # Rudder angle command

        # THEN THE SAIL
        optimalSailAngle = getOptimalSailAng(self._solver, X, self.getGeomP('i'))

        sailAngle = Epsi[1]
        if (self.getGeomP('tau') == 0):
            # No filter
            sailAngle = optimalSailAngle
            dsail = 0
        else:
            dsail = (optimalSailAngle - sailAngle)/self.getGeomP('tau')

        U = CommandVector(
               sailAngle,
               rudder_ang,
               T['wang'],
               T['wspeed']
        )

        # Return the command and the derivative of the Epsi vector
        dEpsi = 0
        EpsiRudder = Epsi[0]
        if (np.abs(EpsiRudder) >= np.pi/4):
            # We only alow decreasing of the pid
            if (EpsiRudder > 0):
                EpsiRudder = np.pi/4
            else:
                EpsiRudder -= np.pi/4
            if (EpsiRudder > 0) and (error < 0):
                dEpsi = error
            if (EpsiRudder < 0) and (error > 0):
                dEpsi = error
        else:
            dEpsi = error
        
        return U, np.array([dEpsi, dsail])

    def getGeom(self):
        """
        Return the geometry of this component
        """
        return self.geom
    
    def getS0(self):
        """
        Return the initial state vector of the target
        """
        return {tname:S0pState.copy() for tname in defaultT0}
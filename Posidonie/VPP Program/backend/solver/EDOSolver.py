import time as tm
import copy

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *
from utils.Utils import *

"""
    Solve the boat's EDO
"""

class SimulatorException(Exception):
    def __init__(self, *args):
        super().__init__(*args)

class SimuFinishedException(SimulatorException):
    def __init__(self, *args):
        super().__init__(*args)

class ResetSimuException(SimulatorException):
    def __init__(self, *args):
        super().__init__(*args)

def RK4Step(X, F, Fu, T, Epsi, U, dt):
    #return F(X, Fu(X, Epsi, T, U)[0])*dt
    k1 = F(X, Fu(X, Epsi, T, U)[0])*dt
    k2 = F(X + k1/2, Fu(X + k1/2, Epsi, T, U)[0])*dt
    k3 = F(X + k2/2, Fu(X + k2/2, Epsi, T, U)[0])*dt
    k4 = F(X + k3, Fu(X + k2, Epsi, T, U)[0])*dt
    return (k1 + (k2 + k3)*2 + k4)/6

class EDOSolver():
    def __init__(self, X0, F, Fu, T, S0, U0, Epsi0):
        """
            - X0 : initial state vector
            - F  : EDO function, dX/dt = F(X, U) where U is the command
            - Fu : Command, U = Fu(X, T)
            - T  : Target function for the command
            - S0 : Initial state vector of the target
            - U0 : Initial command
            - Epsi0: initial integrated error
        """
        self._X0 = X0
        self._F = F
        self._Fu = Fu
        self._T = T
        self._X = self._X0.copy()
        self._t = 0
        self._S0 = S0
        self._S = copy.deepcopy(S0)
        self._U0 = U0
        self._U = U0.copy()
        self._Tvec = self._T(X0, S0, 0)[0]
        self._Epsi0 = Epsi0
        self._Epsi = self._Epsi0.copy()

        # For live computation:
        """self._Xs = []
        self._Us = []
        self._Ts = []"""


    def transient(self, tf, ti=0, dt=0.1, log=False):
        """
        Solve the transient response with the RK4 Method
            - tf : Final time of the simulation
            - ti : Initial time
            - dt : Time step size
            - log: If log are displayed
        """
        if (tf < 0):
            # Compute until the maximum time
            time = np.arange(ti, 1000, dt)
        else:
            time = np.arange(ti, tf, dt)
        ctime = []
        # Reset the state vector
        X = self._X0.copy()
        S = self._S0.copy()
        U = self._U0.copy()
        Epsi = self._Epsi0.copy()
        Xs = [X.copy()]
        Us = [self._U0.copy()]
        Ts = [self._T(X, S, 0)[0]]
        ts = [0]

        for i in range(len(time)):
            loopTime = tm.time()
            t = time[i]
            try:
                T, S = self._T(X, S, t)
            except ResetSimuException:
                self.reset()
            except SimuFinishedException:
                break
            
            try:
                X += RK4Step(X, self._F, self._Fu, T, Epsi, U, dt)
            except Exception as e:
                print(e)
                print("An error occured! Return partial results...")
                break

            U, dEpsi = self._Fu(X, Epsi, T, U)
            Epsi += dEpsi*dt
            Ts.append(T)
            Xs.append(X.copy())
            Us.append(U.copy())
            ts.append(t+dt)

            # Computation time
            ctime.append(tm.time()-loopTime)

            if (log):
                print("{:.1f}% ({:.1f}s / {:.1f}s)".format(100*i/len(time), t, tf))

        Xs = np.array(Xs)
        Us = np.array(Us)
        Ts = np.array(Ts)
        ts = np.array(ts)

        ctime = np.mean(np.array(ctime))
        print("Average computation time (ms): {:.2f}".format(1000*ctime))

        return ts, Xs, Us, Ts
    
    def solve_stationnary(self, T):
        """
        Find a stationnay solution, given:
            - A target dictionnary T

        We init the solution with:
            - Boat heading = target - drift ang
            - Boat speed = wind speed / 3
            - roll, pitch = 0
        """
        Xs = []
        Us = []
        Ts = []
        ts = []

        # Construct the initial vector
        target = T['target']
        wang   = T['wang']
        wspeed = T['wspeed']

        previousSpeed = wspeed/3

        if (getAngleDif(target, wang) > 0):
            # Set the drift
            driftAng = np.deg2rad(8)
        else:
            driftAng = -np.deg2rad(8)

        X = self._X0.copy()
        # Modify the heading
        X._x_ang   = np.array([0, 0, target + driftAng])
        # Modify the speed
        speedDir = np.array([np.cos(target), -np.sin(target), 0])
        speedv = speedDir*previousSpeed
        solver = X._x_speed.getSpeed().getSolver()
        X._x_speed.setSpeed(Vector(solver, speedv, Base.SEA))

        # Construct the initial command
        U = CommandVector(getOptimalSailAng(solver, X, np.deg2rad(15)), 0, wang, wspeed)

        pointSpeed   = X._x_speed
        angularSpeed = X._x_omega
        # Solve the equation, and wait for convergence of the speed
        for i in range(1000):
            
            # Save the angular and translation speed
            angularSpeed = X._x_omega.copy()
            pointSpeed   = X._x_speed.copy()

            speedv = X._x_speed.getSpeed().getNorm()
            # The steady vector have a speed vector of angle target
            X._x_speed.setSpeed(Vector(solver, speedv*speedDir, Base.SEA))
            # The steady vector have a angular speed equal to 0
            X._x_omega.setOmega(Vector(solver, np.zeros(3), Base.SEA))

            dX = self._F(X, self._Fu(X, self._Epsi0, T, U)[0])
            
            # Re-inject the angular and translation speed
            dX._x_ang = angularSpeed
            dX._x_pos = pointSpeed
            
            print("Ang =",X._x_ang)
            print("Pos =",X._x_pos)
            print("Rud =",U._u_rudder)

            # If the boat want to turn, we act with the rudder
            zmoment = dX._x_omega[2]

            rudder = U._u_rudder + zmoment*0.01
            U._u_rudder = np.clip(rudder, -np.deg2rad(30), np.deg2rad(30))

            Ts.append(T)
            Xs.append(X.copy())
            Us.append(U.copy())
            ts.append(i*0.01)

            X += dX*0.05

        Xs = np.array(Xs)
        Us = np.array(Us)
        Ts = np.array(Ts)
        ts = np.array(ts)

        return ts, Xs, Us, Ts

    
    def step(self, dt):
        """
        Perform a step in the RK4 method
        """
        self._Tvec, self._S = self._T(self._X, self._S, self._t)
        self._U, dEpsi = self._Fu(self._X, self._Epsi, self._Tvec, self._U)
        self._Epsi += dEpsi*dt
        self._X += RK4Step(self._X, self._F, self._Fu, self._Tvec, self._Epsi, self._U, dt)
        self._t += dt
        # Update the traces:
        """self._Xs.append(self._X.copy())
        self._Us.append(self._U.copy())
        self._Ts.append(self._t)"""

    def reset(self, resetTargetFct=True):
        """
        Reset the state of the EDO solver
        """
        self._X = self._X0.copy()
        if resetTargetFct:
            self._S = copy.deepcopy(self._S0)
        self._U = self._U0.copy()
        self._t = 0
        self._Epsi = self._Epsi0.copy()
        #print(self._X.toJSON())

    def setX0(self, X0):
        """
        Set the initial default state vector
        """
        self._X0 = X0.copy()

    def setX(self, X):
        """
        Overide the value of the state vector
        """
        self._X = X.copy()

    def resetS(self):
        """
        Reset the state vector of the target function
        """
        self._S = copy.deepcopy(self._S0)

    def getState(self):
        """
        Return the state of the system
        """
        return self._X
    
    def getCommand(self):
        """
        Return the command vector of the system
        """
        return self._U
    
    def getTarget(self):
        """
        Return the target of the system
        """
        return self._Tvec
    
    def getS(self):
        """
        Return the state vector of the target function
        """
        return self._S
    
    def getEpsi(self):
        """
        Return the integrated error
        """
        return self._Epsi
    
    def getTime(self):
        """
        Return the simulation time
        """
        return self._t
    
    def setTime(self, t):
        """
        Set the simulation time
        """
        self._t = t
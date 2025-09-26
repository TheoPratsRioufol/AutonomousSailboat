
import numpy as np

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Force import *
from utils.Utils import *

class StateVector():
    # State vector of the boat
    def __init__(self, pos0, speed0, ang0, omega0):
        if not isinstance(pos0, np.ndarray):
            raise TypeError()
        
        if not isinstance(speed0, PointSpeed):
            raise TypeError()
        
        if not isinstance(ang0, np.ndarray):
            raise TypeError()
        
        if not isinstance(omega0, AngularSpeed):
            raise TypeError()
        
        # Prefix 'x' for state vector
        self._x_pos   = pos0
        self._x_speed = speed0
        self._x_ang   = ang0
        self._x_omega = omega0

    def getParamNames(self):
        """
        Return the list of all the parameter's names
        """
        return ['pos', 'speed', 'ang', 'omega']

    def get(self, name):
        """
        Return the value (array) of a parameter of name equal to name
        """
        if (name == 'pos'):
            return self._x_pos
        if (name == 'speed'):
            return self._x_speed.getSpeed().valueIn(Base.SEA)
        if (name == 'ang'):
            return self._x_ang
        if (name == 'omega'):
            return self._x_omega.getOmega().valueIn(Base.SEA)
        
    def getTrueHeading(self):
        """
        Return the true heading of the boat
        """
        speed = self._x_speed.getSpeed().valueIn(Base.SEA)
        return - np.arctan2(speed[1], speed[0])
    
    def getBoatHeading(self):
        """
        Return the boat's heading
        """
        return self._x_ang[2]
    
    def getBoatRoll(self):
        """
        Return the boat's roll
        """
        return self._x_ang[0]
    
    def getBoatPitch(self):
        """
        Return the boat's pitch
        """
        return self._x_ang[1]
    
    def getBoatSpeedNorm(self):
        """
        Return the norm of the boat speed
        """
        return np.linalg.norm(self._x_speed.getSpeed().valueIn(Base.SEA))
        
    def info(self):
        """
        Return the essential info of the state vector
        """
        speed = self._x_speed
        return {
            'Speed       (knt)': speed.getSpeedNorm()/0.51444444,
            'TrueHeading (deg)': np.rad2deg(speed.getHeading()),
            'DriftAngle  (deg)': np.rad2deg(np.abs(speed.getHeading() - self._x_ang[2])),
            'Roll        (deg)': np.rad2deg(self._x_ang[0]),
            'Pitch       (deg)': np.rad2deg(self._x_ang[1]),
        }

    def toJSON(self):
        """
        Return a JSON dic of this object
        """
        return {'type':'StateVector',
                'pos':self._x_pos.tolist(),
                'speed':self._x_speed.toJSON(),
                'ang':self._x_ang.tolist(),
                'omega':self._x_omega.toJSON()}
    
    def fromJSON(dic, solver):
        """
        Return a JSON object from the dictionary
        """
        if (dic['type'] != 'StateVector'):
            return None
        
        return StateVector(np.array(dic['pos']),
                           PointSpeed.fromJSON(dic['speed'], solver),
                           np.array(dic['ang']),
                           AngularSpeed.fromJSON(dic['omega'], solver))

    def copy(self):
        return StateVector(self._x_pos.copy(),
                           self._x_speed.copy(),
                           self._x_ang.copy(),
                           self._x_omega.copy())
    
    def __add__(self, b):
        """
        Add Two State vector
        """
        if isinstance(b, StateVector):
            return StateVector(self._x_pos + b._x_pos,
                               self._x_speed + b._x_speed,
                               self._x_ang + b._x_ang,
                               self._x_omega + b._x_omega)
        
        elif isinstance(b, StateVectorDerivative):
            solver = self._x_omega.getOmega().getSolver()
            newSpeed = self._x_speed.copy()
            newOmegaSpeed = self._x_omega.copy()
            newOmegaSpeed._speed += Vector(solver, b._rotAccel, Base.SEA)
            newSpeed._speed += b._accel.getAccel()
            return StateVector(self._x_pos + b._speed.getSpeed().valueIn(Base.SEA),
                               newSpeed,
                               self._x_ang + b._omega.getOmega().valueIn(Base.SEA),
                               newOmegaSpeed)

        else:
            raise TypeError()
        

    def __mul__(self, b):
        """
        Multiply a State vector by a scalar
        """ 
        if not (isinstance(b, int) or isinstance(b, float)):
            raise TypeError()
        
        return StateVector(self._x_pos*b,
                           self._x_speed*b,
                           self._x_ang*b,
                           self._x_omega*b)
    
    def __sub__(self, b):
        """
        Substract Two State vector
        """
        return self.__add__(b.__mul__(-1))

    def __truediv__(self, b):
        """
        Divide a State vector by a scalar
        """
        return self.__mul__(1/b)
    
class StateVectorDerivative():
    """
    Derivative of a state vector
    """
    def __init__(self, speed0, accel0, omega0, rotAccel0):
        if not isinstance(speed0, PointSpeed):
            raise TypeError()
        
        if not isinstance(accel0, PointAcceleration):
            raise TypeError()
        
        if not isinstance(omega0, AngularSpeed):
            raise TypeError()
        
        if not isinstance(rotAccel0, np.ndarray):
            raise TypeError()
        
        self._speed = speed0
        self._accel = accel0
        self._omega = omega0
        self._rotAccel = rotAccel0

    def __mul__(self, b):
        """
        Multiply a State vector Derivative by a scalar
        """ 
        if not (isinstance(b, int) or isinstance(b, float)):
            raise TypeError()
        
        return StateVectorDerivative(self._speed*b,
                                     self._accel*b,
                                     self._omega*b,
                                     self._rotAccel*b)
    
    def __truediv__(self, b):
        """
        Divide a PointSpeed by a scalar
        """
        return self.__mul__(1/b)
    
    def __add__(self, b):
        """
        Add Two StateVectorDerivative object
        """
        if isinstance(b, StateVectorDerivative):
            return StateVectorDerivative(self._speed + b._speed,
                                         self._accel + b._accel,
                                         self._omega + b._omega,
                                         self._rotAccel + b._rotAccel)

        else:
            TypeError()
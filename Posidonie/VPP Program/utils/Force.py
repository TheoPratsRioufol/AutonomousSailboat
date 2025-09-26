from enum import Enum
import numpy as np

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

#from backend.solver.Solver import *

class Referential(Enum):
    BOAT   = "ref-boat"
    SEA    = "ref-sea"
    WIND   = "ref-wind"
    SAIL   = "ref-sail"
    RUDDER = "ref-rudder"

class Base(Enum):
    BOAT   = "base-boat"
    SEA    = "base-sea"
    SAIL   = "base-sail"
    RUDDER = "base-rudder"

def refFromName(name):
    """
    Return a referential from a name
    """
    for ref in Referential:
        if (ref.value == name):
            return ref
    raise Exception("No referential found with name: " + str(name))

def baseFromName(name):
    """
    Return a base from a name
    """
    for base in Base:
        if (base.value == name):
            return base
    raise Exception("No base found with name: " + str(name))

def getBaseFomRef(ref):
    """
    Return the basis of the referential ref
    """
    if (ref == Referential.BOAT):
        return Base.BOAT
    if (ref == Referential.SEA):
        return Base.SEA
    if (ref == Referential.WIND):
        return Base.SEA
    if (ref == Referential.SAIL):
        return Base.SAIL
    if (ref == Referential.RUDDER):
        return Base.RUDDER
    
    raise Exception('Unknow referential: '+ref.value)

def getRefFromBase(base):
    """
    Return the referential of the basis base
    """
    if (base == Base.BOAT):
        return Referential.BOAT
    if (base == Base.SEA):
        return Referential.SEA
    if (base == Base.WIND):
        return Referential.SEA
    if (base == Base.SAIL):
        return Referential.SAIL
    if (base == Base.RUDDER):
        return Referential.RUDDER
    
    raise Exception('Unknow base: '+base.value)

def getPassageMatrix(solver, from_, to):
    if (from_ == Base.BOAT) and (to == Base.SEA):
        return solver.getBoatToSeaMatrix()
    
    if (from_ == Base.SEA) and (to == Base.BOAT):
        return solver.getSeaToBoatMatrix()
    
    if (from_ == Base.SAIL):
        if (to != Base.BOAT):
            return np.matmul(getPassageMatrix(solver, Base.BOAT, to), solver.getSailToBoatMatrix())
        else:
            return solver.getSailToBoatMatrix()
        
    if (to == Base.SAIL):
        if (from_ == Base.BOAT):
            return solver.getBoatToSailMatrix()
        elif (from_ == Base.SEA):
            return solver.getSeaToSailMatrix()
        else:
            return np.matmul(solver.getBoatToSailMatrix(), getPassageMatrix(solver, from_, Base.BOAT))
        
    if (from_ == Base.RUDDER):
        if (to != Base.BOAT):
            return np.matmul(getPassageMatrix(solver, Base.BOAT, to), solver.getRudderToBoatMatrix())
        else:
            return solver.getRudderToBoatMatrix()
        
    if (to == Base.RUDDER):
        if (from_ != Base.BOAT):
            return np.matmul(solver.getBoatToRudderMatrix(), getPassageMatrix(solver, from_, Base.BOAT))
        else:
            return solver.getBoatToRudderMatrix()
        
    raise Exception("Unknow basis passage matrix: "+from_.value+", "+to.value)
    
def getPointInBoat(solver, point):
    """
    Return a point expressed in the referential from_ into the boat referential
    """
    from_ = point.getRef()
    pt = point.getPt()

    if (from_ == Referential.SAIL):
        # sail is in rotation around <pos> of the sail
        M = solver.getSailToBoatMatrix()
        ptRot = solver.getSail().getRotationPoint()
        #return Point(solver, np.matmul(M, pt - ptRot) + ptRot, Referential.BOAT)
        return Point(solver, M.dot(pt - ptRot) + ptRot, Referential.BOAT)
    
    if (from_ == Referential.RUDDER):
        # rudder is in rotation around <pos> of the sail
        M = solver.getRudderToBoatMatrix()
        ptRot = solver.getRudder().getRotationPoint()
        #return Point(solver, np.matmul(M, pt - ptRot) + ptRot, Referential.BOAT)
        return Point(solver, M.dot(pt - ptRot) + ptRot, Referential.BOAT)
    
    if (from_ == Referential.SEA):
        # translation and rotation
        # Frist, translation
        ptTran = pt - solver.getBoatPos()
        # Then, rotation around the origin (of the boat)
        M = solver.getSeaToBoatMatrix()
        #return Point(solver, np.matmul(M, ptTran), Referential.BOAT)
        return Point(solver, M.dot(ptTran), Referential.BOAT)
    
    if (from_ == Referential.BOAT):
        return Point(solver, pt, Referential.BOAT)
    
    raise Exception("Unable to express pt in ref: "+str(from_.value)+" to the BOAT ref.")


def getPointBoatTo(solver, point, to):
    """
    Express a point belonging to the BOAT to the "to" referential
    """
    pt = point.getPt()

    if (to == Referential.SAIL):
        # sail is in rotation around <pos> of the sail
        M = solver.getBoatToSailMatrix()
        ptRot = solver.getSail().getRotationPoint()
        #return Point(solver, np.matmul(M, pt - ptRot) + ptRot, Referential.SAIL)
        return Point(solver, M.dot(pt - ptRot) + ptRot, Referential.SAIL)
    
    if (to == Referential.RUDDER):
        # sail is in rotation around <pos> of the sail
        M = solver.getBoatToRudderMatrix()
        ptRot = solver.getRudder().getRotationPoint()
        #return Point(solver, np.matmul(M, pt - ptRot) + ptRot, Referential.RUDDER)
        return Point(solver, M.dot(pt - ptRot) + ptRot, Referential.RUDDER)
    
    if (to == Referential.SEA):
        # SEA -> BOAT: translation and rotation
        # Frist, inverse rotation
        M = solver.getBoatToSeaMatrix()
        ptRotated = np.matmul(M, pt)
        # The, inverse translation
        ptRotated += solver.getBoatPos()
        return Point(solver, ptRotated, Referential.SEA)
    
    if (to == Referential.BOAT):
        return point
    
    raise Exception("Unable to express pt (from the BOAT ref) to ref: "+str(to.value))


def getPointInRef(solver, pt, to):
    """
    Change a point to the referential to
    """
    # On transfère de 'from_' to 'boat' puis de 'boat' to 'to'
    return getPointBoatTo(solver, getPointInBoat(solver, pt), to)


class Point():
    def __init__(self, solver, pt, ref):
        """
        Point class:
            - pt : coordinates
            - ref: referential of the point
        """
        self._pt = np.array(pt)
        self._ref = ref
        self._solver = solver

        if not isinstance(pt, np.ndarray):
            raise TypeError("<pt> field of Point should be a numpy array")
        
        #if not isinstance(solver, Solver):
        #    raise TypeError("<solver> field of Point should be a Solver object")

        if not isinstance(ref, Referential):
            raise TypeError("<ref> field of Point should be a Referential object")
        
    def getPt(self):
        return self._pt
    
    def getRef(self):
        return self._ref

    def inRef(self, ref):
        """
        Return this point expressed in another referential
        """
        if (ref != self._ref):
            return getPointInRef(self._solver, self, ref)
        return self
        
    def valueIn(self, ref):
        """
        Return a numpy array of this point in the referential ref
        """
        return self.inRef(ref)._pt
        
    def __add__(self, b):
        """
        Add Two point
        """
        if not isinstance(b, Point):
            raise TypeError()
        
        return Point(self._solver, self._pt + b.valueIn(self._ref), self._ref)

    def __sub__(self, b):
        """
        Substract Two points
        """
        return self.__add__(b.__mul__(-1))

    def __mul__(self, b):
        """
        Multiply a Point by a scalar
        """ 
        if not (isinstance(b, int) or isinstance(b, float)):
            raise TypeError()
        
        return Point(self._solver, self._pt * b, self._ref)
    
    def __truediv__(self, b):
        """
        Divide a Point by a scalar
        """
        return self.__mul__(1/b)

    def vectorial(self, b):
        """
        Compute the vectorial product
        """
        if isinstance(b, Vector):
            array = b.valueIn(getBaseFomRef(self._ref))
        elif isinstance(b, Point):
            array = b.valueIn(self._ref)
        else:
            raise TypeError("Type is: '"+str(type(b))+ "', Vector or Point expected")
        
        # We always work with len=3 array
        #res1 = np.cross(self._pt, array)
        res = np.array([self._pt[1]*array[2] - self._pt[2]*array[1],
                        self._pt[2]*array[0] - self._pt[0]*array[2],
                        self._pt[0]*array[1] - self._pt[1]*array[0]])
        
        # Always return a vector object
        return Vector(self._solver, res, getBaseFomRef(self._ref))
    
    def __eq__(self, value):
        if not isinstance(value, Point):
            return False
        
        if (self._pt.all() == value.valueIn(self._ref).all()):
            return True
        return False

    def __str__(self):
        return str(self._pt) + " (ref: " + self._ref.value + ")"
    
    def copy(self):
        """
        Return a copy of this object
        """
        return Point(self._solver, self._pt.copy(), self._ref)
    
    def toJSON(self):
        """
        JSON Serialize this object
        """
        return {'type':'Point',
                'pt':self._pt.tolist(),
                'ref':self._ref.value}
    
    def fromJSON(dic, solver):
        """
        JSON De-Serialize this object
        """
        if (dic['type'] != 'Point'):
            return None
        return Point(solver, np.array(dic['pt']), refFromName(dic['ref']))


class Vector():
    def __init__(self, solver, vec, base):
        """
        Vecetor class:
            - solver : solver object (used to find the passage matrix)
            - vec    : coordinates
            - ref    : base of the vector
        """
        self._vec = np.array(vec)
        self._base = base
        self._solver = solver

        if not isinstance(base, Base):
            raise TypeError("<base> field of Vector should be a Base object")
        
        #if not isinstance(solver, Solver):
        #    raise TypeError("<solver> field of Vector should be a Solver object")
        
    def getSolver(self):
        return self._solver

    def inBase(self, base):
        """
        Return this vector expressed in another base
        """
        if (base != self._base):
            M = getPassageMatrix(self._solver, self._base, base)
            newvec = M.dot(self._vec)
            return Vector(self._solver, newvec, base)
        
        return self
        
    def valueIn(self, base):
        """
        Return a numpy array of this vector in the base <base>
        """
        return self.inBase(base)._vec
    
    def __add__(self, b):
        """
        Add Two vector
        """
        if not isinstance(b, Vector):
            raise TypeError()
        
        return Vector(self._solver, self._vec + b.valueIn(self._base), self._base)
    
    def __sub__(self, b):
        """
        Substract Two Vector
        """
        return self.__add__(b.__mul__(-1))

    def __mul__(self, b):
        """
        Multiply a Vector by a scalar / array element wise
        """ 
        if (isinstance(b, int) or isinstance(b, float)):
            res = self._vec * b
        elif isinstance(b, np.ndarray):
            res = self._vec * b
        else:
            raise TypeError("Impossible to multiply with: "+str(type(b)))
        
        return Vector(self._solver, res, self._base)
    
    def __truediv__(self, b):
        """
        Divide a Vector by a scalar
        """
        return self.__mul__(1/b)
    
    def __str__(self):
        return str(self._vec) + " (base: " + self._base.value + ")"
    
    def getNorm(self):
        return np.linalg.norm(self._vec)
    
    def copy(self):
        """
        Return a copy of this object
        """
        return Vector(self._solver, self._vec.copy(), self._base)
    
    def toJSON(self):
        """
        JSON Serialize this object
        """
        return {'type':'Vector',
                'vec':self._vec.tolist(),
                'base':self._base.value}
    
    def fromJSON(dic, solver):
        """
        JSON De-Serialize this object
        """
        if (dic['type'] != 'Vector'):
            return None
        return Vector(solver, np.array(dic['vec']), baseFromName(dic['base']))
    
    def vectorial(self, b):
        """
        Compute the vectorial product
        """
        if isinstance(b, Vector):
            array = b.valueIn(self._base)
        elif isinstance(b, Point):
            array = b.valueIn(getRefFromBase(self._base))
        else:
            raise TypeError("Type is: '"+str(type(b))+ "', Vector or Point expected")
        
        # We always work with len=3 array
        #res1 = np.cross(self._pt, array)
        res = np.array([self._vec[1]*array[2] - self._vec[2]*array[1],
                        self._vec[2]*array[0] - self._vec[0]*array[2],
                        self._vec[0]*array[1] - self._vec[1]*array[0]])
        
        # Always return a vector object
        return Vector(self._solver, res, self._base)



class Moment():
    def __init__(self, vec):
        """
        Moment class
        """
        if not isinstance(vec, Vector):
            raise TypeError("<vec> field of Moment should be a Vector object")
        self._vec = vec

    def fromForceAndOM(force, O=None):
        """
        Return the Moment object of a force <force> relative to a point O
        Moment = OM^F
        """
        if not isinstance(force, Force):
            raise TypeError()
        if (O != None) and (not isinstance(O, Point)):
            raise TypeError()
        
        if (O != None):
            return Moment((force.getPt()-O).vectorial(force.getVec()))
        
        return Moment(force.getPt().vectorial(force.getVec()))
    
    def getVec(self):
        return self._vec

    def __add__(self, b):
        """
        Add Two Moment
        """
        if not isinstance(b, Moment):
            raise TypeError()
        
        return Moment(self._vec + b._vec)
    
    def __mul__(self, b):
        """
        Multiply a Moment by a scalar
        """
        return Moment(self._vec * b)
    
    def __sub__(self, b):
        """
        Substract Two Moments
        """
        return self.__add__(b.__mul__(-1))
    
    def valueIn(self, base):
        return self._vec.valueIn(base)
    
    def __str__(self):
        return self._vec.__str__() + " at O"


class Force():
    def __init__(self, force, pointOrMoment):
        """
        Force class:
            - force: vector of the force
            - point: point of application
        """
        self._force = force
        self._point = None

        if not isinstance(force, Vector):
            raise TypeError("<force> field of Force should be a Vector object")
        
        if isinstance(pointOrMoment, Point):
            self._point = pointOrMoment
            # Compute the moment at the orign of the BOAT referential
            self._originMoment = Moment.fromForceAndOM(self)
        elif isinstance(pointOrMoment, Moment):
            self._originMoment = pointOrMoment
        else:
            raise TypeError("<pointOrMoment> field of Force should be a Point or a Moment object")

    def getPt(self):
        """
        Return the application point
        """
        return self._point
    
    def getVec(self):
        """
        Return the vector object
        """
        return self._force
    
    def getOriginMoment(self):
        return self._originMoment
        
    def __add__(self, b):
        """
        Add Two Force together
        """
        if not isinstance(b, Force):
            raise TypeError()
        
        # we add the force and the moment
        return Force(self.getVec() + b.getVec(), self.getOriginMoment() + b.getOriginMoment())
    
    def __str__(self):
        return "Force: " + self._force.__str__() + "\nMoment: " + self._originMoment.__str__()


class Matrix():
    def __init__(self, solver, matrix, base):
        """
        Class Matrix:
            save a matrix specified in a specific base
        """
        if not isinstance(base, Base):
            raise TypeError("<base> field of Matrix should be a Base object")
        
        #if not isinstance(matrix, np.array):
        #    raise TypeError("<matrix> field of Matrix should be a np.array object")
        
        self._matrix = matrix
        self._base = base
        self._solver = solver

    def getSolver(self):
        """
        Return the solver of this object
        """
        return self._solver

    def inBase(self, base):
        """
        Express this matrix in another base
        """
        if (base == self._base):
            return self
        raise Exception("Not implemented inBase Matrix")

    def valueIn(self, base):
        """
        Return the value of this matrix for a base <base>
        """
        return self.inBase(base)._matrix

    def getBase(self):
        """
        Return the base of the matrix
        """
        return self._base
    
    def __str__(self):
        """
        String representation
        """
        return str(self._matrix) + f" [{self._base.value}]"
    
    def __add__(self, b):
        """
        Add Two Matrix together
        """
        if not isinstance(b, Matrix):
            raise TypeError()
        
        return Matrix(self._solver, self._matrix + b.valueIn(self._base), self._base)


class Inertia():
    def __init__(self, matrix, cdg, m):
        """
        Class Inertia: used to store the inertia matrix of a component
        All the matrix are stored at the origin point
        The input inertia shoud be computed at the center of gravity cdg
        <m> is the mass of the object
        """
        if not isinstance(cdg, Point):
            raise TypeError("<cdg> field of Inertia should be a Point object")
        
        if not isinstance(matrix, Matrix):
            raise TypeError("<matrix> field of Inertia should be a Matrix object")
        
        if matrix.getBase() != Base.BOAT:
            raise Exception("The matrix must be expressed in the boat basis")

        self._matrix = self.huygens(matrix, m, cdg)

    def getMatrix(self):
        return self._matrix
    
    def huygens(self, matrix, m, cdg):
        """
        Shift the inertial matrix <matrix> expressed at the center of
        gravity of an object, in the base BOAT
        to the origin to the BOAT referential using
        the Huygens formula
        <m> is the mass of the object
        """
        og = cdg.valueIn(Referential.BOAT)
        xg, yg, zg = og[0], og[1], og[2]
        IOG = np.array([[yg**2+zg**2, -xg*yg, -xg*zg],
                        [-xg*yg, xg**2+zg**2, -yg*zg],
                        [-xg*zg, -yg*zg, xg**2+yg**2]])
        
        matrixValue = matrix.valueIn(Base.BOAT) + m*IOG
        return Matrix(matrix._solver, matrixValue, Base.BOAT)
    
    def __add__(self, b):
        """
        Add Two Inertia together
        """
        if not isinstance(b, Inertia):
            raise TypeError()
        
        cdg = Point(self._matrix.getSolver(), np.zeros(3), Referential.BOAT)
        return Inertia(self._matrix + b._matrix, cdg, 0)
    
    def __str__(self):
        """
        String representation
        """
        return self._matrix.__str__() + " @O"
    


class AngularSpeed():
    def __init__(self, speed, R1, R2):
        """
        The class AngularSpeed save an angular speed between two referential
        """
        if not isinstance(speed, Vector):
            raise TypeError("<speed> field of AngularSpeed should be a Vector object")
        
        if not isinstance(R1, Referential):
            raise TypeError("<R1> field of AngularSpeed should be a Referential object")
        
        if not isinstance(R2, Referential):
            raise TypeError("<R2> field of AngularSpeed should be a Referential object")
        
        self._speed = speed
        self._R1 = R1
        self._R2 = R2

    def getR1(self):
        return self._R1
    
    def getR2(self):
        return self._R2
    
    def getOmega(self):
        return self._speed
    
    def setOmega(self, speed):
        self._speed = speed
    
    def __add__(self, b):
        """
        Add Two AngularSpeed
        """
        if not isinstance(b, AngularSpeed):
            raise TypeError()
        
        if (self._R1 != b._R1) or (self._R2 != b._R2):
            raise Exception("Uncompatible referentials")
        
        return AngularSpeed(self._speed + b._speed, self._R1, self._R2)
    
    def __mul__(self, b):
        """
        Multiply a AngularSpeed by a scalar
        """ 
        if not (isinstance(b, int) or isinstance(b, float)):
            raise TypeError()
        
        return AngularSpeed(self._speed*b, self._R1, self._R2)
    
    def __truediv__(self, b):
        """
        Divide a AngularSpeed by a scalar
        """
        return self.__mul__(1/b)
    
    def copy(self):
        """
        Return a copy of this object
        """
        return AngularSpeed(self._speed.copy(), self._R1, self._R2)
    
    def toJSON(self):
        """
        JSON Serialize this object
        """
        return {'type':'AngularSpeed',
                'speed':self._speed.toJSON(),
                'R1':self._R1.value,
                'R2':self._R2.value}
    
    def fromJSON(dic, solver):
        """
        JSON De-Serialize this object
        """
        if (dic['type'] != 'AngularSpeed'):
            return None
        
        return AngularSpeed(Vector.fromJSON(dic['speed'], solver),
                            refFromName(dic['R1']),
                            refFromName(dic['R2']))
    

class PointSpeed():
    def __init__(self, point, speed, R1, R2):
        """
        The class PointSpeed save the speed of a point between two referential
        """
        if not isinstance(speed, Vector):
            raise TypeError("<speed> field of PointSpeed should be a Vector object")
        
        if not isinstance(point, Point):
            raise TypeError("<point> field of PointSpeed should be a Vector object")
        
        if not isinstance(R1, Referential):
            raise TypeError("<R1> field of PointSpeed should be a Referential object")
        
        if not isinstance(R2, Referential):
            raise TypeError("<R2> field of PointSpeed should be a Referential object")
        
        self._point = point
        self._speed = speed
        self._R1 = R1
        self._R2 = R2

    def getR1(self):
        return self._R1
    
    def getR2(self):
        return self._R2
    
    def getPt(self):
        return self._point
    
    def getSpeed(self):
        return self._speed
    
    def setSpeed(self, speed):
        self._speed = speed
    
    def getVelocity(solver, A, R1, R2):
        """
        Return V(A in R1 / R2)
        """
        # V(A, R1/R2) = V(A, R1/R0) + V(A, R0/R2)
        #             = V(A, R1/R0) - V(A, R2/R0)
        V_A_R1_R0 = solver.getVelocityField(R1).at(A).getSpeed()
        V_A_R2_R0 = solver.getVelocityField(R2).at(A).getSpeed()

        return PointSpeed(A, V_A_R1_R0-V_A_R2_R0, R1, R2)
    
    def __add__(self, b):
        """
        Add Two Points Speed
        """
        if not isinstance(b, PointSpeed):
            raise TypeError()
        
        if (self._point != b._point):
            raise Exception("Uncompatible points")
        if (self._R1 != b._R1):
            raise Exception("Uncompatible R1 referential")
        if (self._R2 != b._R2):
            raise Exception("Uncompatible R2 referential")
        
        return PointSpeed(self._point, self._speed + b._speed, self._R1, self._R2)

    def __mul__(self, b):
        """
        Multiply a PointSpeed by a scalar
        """ 
        if not (isinstance(b, int) or isinstance(b, float)):
            raise TypeError()
        
        return PointSpeed(self._point, self._speed*b, self._R1, self._R2)
    
    def __truediv__(self, b):
        """
        Divide a PointSpeed by a scalar
        """
        return self.__mul__(1/b)
    
    def copy(self):
        """
        Return a copy of this object
        """
        return PointSpeed(self._point.copy(), self._speed.copy(), self._R1, self._R2)

    def toJSON(self):
        """
        JSON Serialize this object
        """
        return {'type':'PointSpeed',
                'pt':self._point.toJSON(),
                'speed':self._speed.toJSON(),
                'R1':self._R1.value,
                'R2':self._R2.value}
    
    def fromJSON(dic, solver):
        """
        JSON De-Serialize this object
        """
        if (dic['type'] != 'PointSpeed'):
            return None
        return PointSpeed(Point.fromJSON(dic['pt'], solver),
                          Vector.fromJSON(dic['speed'], solver),
                          refFromName(dic['R1']),
                          refFromName(dic['R2']))
    
    def __str__(self):
        return "V(" + str(self._point) + "\nin " + self._R1.value + "/" + self._R2.value + ") =\n" +str(self._speed)
    
    def getHeading(self):
        """
        Return the heading of the speed vector in the (x,y) plane, in sea base
        """
        speed = self._speed.valueIn(Base.SEA)
        return -np.arctan2(speed[1], speed[0])
    
    def getSpeedNorm(self):
        """
        Return the norm of the speed
        """
        return self._speed.getNorm()
    
class PointAcceleration():
    def __init__(self, acceleration, point, R1, R2):
        """
        The class PointAcceleration save the acceleration of a point
        """

        if not isinstance(acceleration, Vector):
            raise TypeError("<acceleration> field of PointAcceleration should be a Vector object")
        
        if not isinstance(point, Point):
            raise TypeError("<point> field of PointAcceleration should be a Point object")
        
        if not isinstance(R1, Referential):
            raise TypeError("<R1> field of PointAcceleration should be a Referential object")
        
        if not isinstance(R2, Referential):
            raise TypeError("<R2> field of PointAcceleration should be a Referential object")
        
        self._accel = acceleration
        self._pt = point
        self._R1 = R1
        self._R2 = R2

    def getPt(self):
        return self._pt
    
    def getR1(self):
        return self._R1
    
    def getR2(self):
        return self._R2
    
    def getAccel(self):
        return self._accel
    
    def __mul__(self, b):
        """
        Multiply a Point Acceleration by a scalar
        """ 
        if not (isinstance(b, int) or isinstance(b, float)):
            raise TypeError()
        
        return PointAcceleration(self._accel*b, self._pt, self._R1, self._R2)
    
    def __truediv__(self, b):
        """
        Divide a PointSpeed by a scalar
        """
        return self.__mul__(1/b)
    
    def __add__(self, b):
        """
        Add Two Points Acceleration
        """
        if not isinstance(b, PointAcceleration):
            raise TypeError()
        
        if (self._pt != b._pt):
            raise Exception("Uncompatible points")
        if (self._R1 != b._R1):
            raise Exception("Uncompatible R1 referential")
        if (self._R2 != b._R2):
            raise Exception("Uncompatible R2 referential")
        
        return PointAcceleration(self._accel + b._accel, 
                                 self._pt, self._R1, self._R2)

class AccelerationField():
    def __init__(self, acceleration, rotSpeed):
        """
        The class AccelerationField save the field of acceleration
        """
        
        if not isinstance(rotSpeed, AngularSpeed):
            raise TypeError("<rotSpeed> field of AccelerationField should be a AngularSpeed object")
        
        if not isinstance(acceleration, PointAcceleration):
            raise TypeError("<acceleration> field of AccelerationField should be a PointAcceleration object")
        
        # The acceleration is: R1 / R2
        # The angular speed  : R1'/ R2'
        # The speed is       : R1*/ R2*
        # Verify the compatibility

        if (rotSpeed.getR1() != acceleration.getR1()) or (rotSpeed.getR2() != acceleration.getR2()):
            raise Exception("The acceleration and angular speed must have the same referential R1 and R2")

        self._omega = rotSpeed
        self._accel = acceleration

    def at(self, B):
        """
        Return the acceleration at point B
        """
        A = self._accel.getPt()
        accel  = self._accel.getAccel()
        accel += self._omega.getOmega().vectorial(B - A)
        accel += self._omega.getOmega().vectorial(self._omega.getOmega().vectorial(B - A))

        return PointAcceleration(accel, B, self._omega.getR1(), self._omega.getR2())

class VelocityField():

    def __init__(self, pointSpeed, angSpeed):
        """
        The class VelocityField is used to copmpute the velocity of a point
        A in a referential, relative to the sea:
            - V(A in R / Sea)

            angSpeed   : Angular Speed
            pointSpeed : Speed of a point
        """
        if not isinstance(pointSpeed, PointSpeed):
            raise TypeError("<pointSpeed> field of VelocityField should be a PointSpeed object")
        
        if not isinstance(angSpeed, AngularSpeed):
            raise TypeError("<angSpeed> field of VelocityField should be a AngularSpeed object")
        
        self._R = angSpeed.getR1()
        self.updateAngSpeed(angSpeed)
        self.updatePointSpeed(pointSpeed)

    def updateAngSpeed(self, omega):
        """
        Update the angular velocity Omega(R / Sea)
        """
        if not isinstance(omega, AngularSpeed):
            raise TypeError()
        
        if (omega.getR2() != Referential.SEA):
            raise Exception('The angular velocity should be between R and Sea')
        
        if (omega.getR1() != self._R):
            raise Exception('The angular velocity should be between R and Sea. R field and R omega different!')
        
        self._omega = omega

    def updatePointSpeed(self, pointSpeed):
        """
        Update the translation velocity of a point A : V(A in R / Sea)
        """
        if not isinstance(pointSpeed, PointSpeed):
            raise TypeError()
        
        if (pointSpeed.getR2() != Referential.SEA):
            raise Exception('The point speed should be between R and Sea')
        
        if (pointSpeed.getR1() != self._R):
            raise Exception('The point speed should be between R and Sea. R field and R point speed different!')

        self._pointSpeed = pointSpeed 

    def at(self, B):
        """
        Return - V(B in R / Sea) (point speed object)
        """
        # Varinion : 
        # V(B, R/R0) = V(A, R/R0) + (A - B)^Omega(R/R0)
        speed = self._pointSpeed.getSpeed() - (self._pointSpeed.getPt() - B).vectorial(self._omega.getOmega())
        return PointSpeed(B, speed, self._R, Referential.SEA)
    
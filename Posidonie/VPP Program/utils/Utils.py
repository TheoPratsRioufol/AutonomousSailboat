import tkinter as tk
from tkinter import ttk
from enum import Enum
import numpy as np
from PIL import ImageColor
import json

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *
from utils.Force import *
from backend.solver.StateVector import *
from backend.solver.CommandVector import *

class RunStopButton(ttk.Button):
    def __init__(self, master, app, command=None):
        super().__init__(master, command=self.toggle)
        self.command = command
        self.app = app
        self.set(False)

    def toggle(self):
        """
        Switch between play/stop
        """
        self.set(not self.get())

    def get(self):
        """
        Get the state of the button
        """
        return self.play
    
    def set(self, state):
        """
        Set the state of the button
        """
        self.play = state
        if (self.play):
            self.config(image=self.app.getAssets().get('stop'))
        else:
            self.config(image=self.app.getAssets().get('run'))
        if (self.command != None):
            self.command(self.play)

class TkValid(ttk.Label):
    """
    Display if a result is valid or not (need to be updated)
    """
    def __init__(self, master, app, background=''):
        super().__init__(master, background=background)
        self.app = app
        self.set(True)

    def get(self):
        """
        Get the state of the validity
        """
        return self.state
    
    def set(self, state):
        """
        Set the state of the validity
        """
        self.state = state
        if (self.state):
            self.config(image=self.app.getAssets().get('valid'))
        else:
            self.config(image=self.app.getAssets().get('invalid'))


class Units(Enum):
    m = 'm'
    m2 = 'm^2'
    mps = 'm/s'
    knt = 'knt'
    deg = 'deg'
    rad = 'rad'
    kg = 'kg'
    none = '-'
    percent = "%"
    s = 's'
    gmm2 = 'g*mm^2'
    kgm2 = 'kg*m^2'

    def fromName(name):
        """
        Return a unit with .value equal to 'name'
        """
        for e in Units:
            if (e.value == name):
                return e
        raise Exception("Unknow unit: " + str(name))

    def toSI(value, unit):
        """
        Return the value of 'value' in the SI system
        """
        if (unit in [Units.none, Units.m, Units.rad, Units.mps, Units.kg, Units.s, Units.kgm2]):
            return value
        if (unit == Units.deg):
            return np.deg2rad(value)
        if (unit == Units.gmm2):
            return value*1e-9
        if (unit == Units.knt):
            return noeud2ms(value)
        if (unit == Units.percent):
            return value/100
        raise Exception("Unhandeled unit to SI conversion for: " + str(unit))

    def toUnit(value, baseunit, finalunit):
        """
        Change value unit
        """
        if (baseunit == finalunit):
            return value
        
        if (baseunit == Units.gmm2) and ((finalunit == Units.kgm2)):
            return value*1e-9
        
        if (finalunit == Units.gmm2) and ((baseunit == Units.kgm2)):
            return value*1e9

        raise Exception(f"Unhandeled unit conversion: {baseunit.value} -> {finalunit.value}")


class Dir(Enum):
    X = "x"
    Y = "y"
    Z = "z"

class SimWarning(Enum):
    OUTOFBOUND = "OUTOFBOUND"

def npArrayToRGB(array):
    rgb = (int(array[0]), int(array[1]), int(array[2]))
    return "#%02x%02x%02x" % rgb

def hexTonpArray(hex):
    return np.array(ImageColor.getcolor(hex, "RGB"))

def gradientColor(col1, col2, i):
    col1 = hexTonpArray(col1)
    col2 = hexTonpArray(col2)
    color = col1 + (col2 - col1)*i
    return npArrayToRGB(color)

def noeud2ms(noeud):
    return noeud*0.51444444

def ms2noeud(ms):
    if (type(ms) == list):
        return list(np.array(ms)/0.51444444)
    return ms/0.51444444

def getANullForce(solver):
    """
    Return a force = 0
    """
    return Force(Vector(solver, np.zeros(3).copy(), Base.BOAT),
                               Point(solver, np.zeros(3).copy(), Referential.BOAT))

def getMatRot(dir, ang):
    """
    Return the rotation matrix associed to an angle and dir
    """
    if (isinstance(dir, Dir)):
        if (dir == Dir.X):
            M = np.array([[1, 0.0, 0],
                        [0, np.cos(ang), np.sin(ang)],
                        [0, -np.sin(ang), np.cos(ang)]])
        elif (dir == Dir.Y):
            M = np.array([[np.cos(ang), 0.0, -np.sin(ang)],
                        [0, 1, 0],
                        [np.sin(ang), 0, np.cos(ang)]])
        else:
            M = np.array([[np.cos(ang), np.sin(ang), 0],
                        [-np.sin(ang), np.cos(ang), 0],
                        [0, 0.0, 1]])
    else:
        # Rotation around a vector
        ux, uy, uz = dir/np.linalg.norm(dir)
        c = np.cos(ang)
        s = np.sin(ang)
        M = np.array([[(1-c)*ux**2+c,ux*uy*(1-c)-uz*s,ux*uz*(1-c)+uy*s],
                      [ux*uy*(1-c)+uz*s,(1-c)*uy**2+c,uy*uz*(1-c)-ux*s],
                      [ux*uz*(1-c)-uy*s,uy*uz*(1-c)+ux*s,(1-c)*uz**2+c]])
    return M

def rotation(pts, xr, dir, ang):
    """
    Apply a rotation to the list <pts> of axis (<xr>, <dir>) 
    of angle <ang>
    """

    M = getMatRot(dir, ang)
    out = []
    for pt in pts:
        out.append(np.matmul(M, pt-xr)+xr)
    return out

def rotationM(pts, M):
    """
    Apply a rotation to the list <pts> with matrix M
    """
    out = []
    for pt in pts:
        out.append(np.matmul(M, pt))
    return out

def translation(pts, vec):
    """
    Apply a translation of the list <pts> by vec
    """
    out = []
    for pt in pts:
        out.append(pt + vec)
    return out


def projection(pts, camera, screenSize):
    """
    Return the projection of a set of 3D point on the 2D place (x,z)
    <camera> is the object that represents the camera:
        - .getPos()  : position in 3D space
        - .getRotationMatrix()  : 3 angles rotation matrix
        - .zoom : the zoom factor
    Warning: inverse the y axis!
    And cast to int
    """
    out = []
    for pt in pts:
        # First apply the rotations of the camera
        pt = np.matmul(camera.getRotationMatrix(), pt - camera.getPos()) + camera.getPos()
        
        # The point of pos=camera.pos must be at the center of the screen
        offx = screenSize[0]/2 - camera.pos[0]*camera.zoom
        offy = screenSize[1]/2 + camera.pos[2]*camera.zoom

        # Then the zoom & projection
        out.append([int(pt[0]*camera.zoom+offx), 
                    int(-pt[2]*camera.zoom+offy)])

    return out

def projectionOnlyRotation(pts, camera):
    """
    Return the projection of a set of 3D point on the 2D place (x,z)
    <camera> is the object that represents the camera:
        - .pos  : position in 3D space
        - .ang  : 3 angles in the 3D space
        - .zoom : the zoom factor
    Warning: inverse the y axis!
    And cast to int
    ONLY  EXECUTE THE ROTATION AROUND (0,0,0)!
    """
    out = []
    for pt in pts:
        # Apply the rotations of the camera
        pt = np.matmul(camera.getRotationMatrix(), pt)

        out.append([int(pt[0]), 
                    int(-pt[2])])

    return out

def computeDistToCam(poly, camera):
    """
    Compute the distance between a polygon and the camera
    """
    # First, compute the geometric barycentre
    center = np.zeros(3)
    for pt in poly:
        center += pt
    center /= len(poly)
    # Then, project it and compute the distance (Y) coord
    center = np.matmul(camera.getRotationMatrix(), center)
    # Distance = y coordinate
    return center[1]

def computeLightCam(poly, camera):
    """
    Compute the amount of light on a polygon relative to camera
    """
    if (len(poly) < 2):
        return 0
    # take two lines and apply the vectorial product
    origin = np.matmul(camera.getRotationMatrix(), poly[0])
    line1 = origin - np.matmul(camera.getRotationMatrix(), poly[1])
    for i in range(1,len(poly)):
        line2 = origin - np.matmul(camera.getRotationMatrix(), poly[i])
        cross = np.cross(line1, line2)
        if (cross.any() != np.zeros(3).any()):
            cross = cross/np.linalg.norm(cross)
            return cross[1] # scalar product with the camera direction
    return 0

def polysRotation(polys, ang):
    """
    Apply a 3D rotation to a list of polygon
    """

    Myaw = getMatRot(Dir.Z, ang[2])
    Mroll = getMatRot(Dir.X, ang[0])
    Mpitch = getMatRot(Dir.Y, ang[1])

    for i in range(len(polys)):
        # Apply rotation in order
        # yaw - roll - pitch
        polys[i] = rotationM(polys[i], Myaw)
        polys[i] = rotationM(polys[i], Mroll)
        polys[i] = rotationM(polys[i], Mpitch)
    return polys

def polysMatRotation(polys, M):
    """
    Apply a 3D matricial rotation to a list of polygon
    """
    for i in range(len(polys)):
        polys[i] = rotationM(polys[i], M)
    return polys

def pointRotation(pt, ang):
    """
    Apply a 3D rotation to a point
    """

    Myaw = getMatRot(Dir.Z, ang[2])
    Mroll = getMatRot(Dir.X, ang[0])
    Mpitch = getMatRot(Dir.Y, ang[1])

    # Apply rotation in order
    # yaw - roll - pitch
    pt = np.matmul(Myaw, pt)
    pt = np.matmul(Mroll, pt)
    pt = np.matmul(Mpitch, pt)
    return pt

def pointMatRotation(pt, M):
    """
    Apply a 3D rotation to a point
    """
    return M.dot(pt)

def polysTranslation(polys, vec):
    """
    Apply a 3D translation to a list of polygon
    """
    for i in range(len(polys)):
        polys[i] = translation(polys[i], vec)
    return polys

def pointTranslation(pt, vec):
    """
    Apply a 3D translation to a point
    """
    return pt + vec
        

def interpolYYXXV(Y1, Y2, X1, X2, x):
    """
    Interpolate linearly the function f in x, where:
        f(X1) = Y1
        f(X2) = Y2
    """
    if X1 == X2:
        return Y1
    if (x >= X2):
        return Y2
    if (x <= X1):
        return Y1
    return (Y2 - Y1)*(x - X1)/(X2 - X1) + Y1

def interpolDXXV(dic, V1, V2, V):
    """
    Interpolate linearly V between V1 and V2 for values dic[V1], dic[V2]
    """
    return interpolYYXXV(dic[V1], dic[V2], V1, V2, V)

def getFraming(dic, value):
    """
    Return the previous and following key that closly frame value
    """
    orderedKeys = sorted(dic.keys())
    if (value <= orderedKeys[0]):
        return orderedKeys[0], orderedKeys[1]
    idx = len(orderedKeys)-1
    for i in range(1,len(orderedKeys)):
        if (orderedKeys[i-1] < value) and (orderedKeys[i] >= value):
            idx = i
            break
    return orderedKeys[idx-1], orderedKeys[idx]

def getCloserIdx(t, ts):
    """
    Return the index idx with minimize the distance between t and ts[idx]
    ts should be ordered in growing values
    """
    if (ts[1] < ts[0]):
        raise Exception("ts should be growing")
    
    for i in range(len(ts)):
        if (ts[i] > t):
            if (i > 0) and (np.abs(t - ts[i-1]) < np.abs(t - ts[i])):
                return i-1
            return i
    return len(ts)-1

def getBeforeAfterIdx(t, ts):
    """
    Return the two index idx such ts[i1] <= t < ts[i2]
    ts should be growing
    """
    if (ts[1] < ts[0]):
        raise Exception("ts should be growing")
    
    for i in range(len(ts)-1):
        if (ts[i] > t):
            return i, i+1
    return len(ts)-2, len(ts)-1

def getIn2pi(ang):
    """
    Return ang in the interval [0, 2pi[
    """
    n = int(ang/(2*np.pi))
    return ang - 360*n

def getInPi(ang):
    """
    Return ang in the interval [-pi, pi[
    """
    ang = getIn2pi(ang)
    if (ang > np.pi):
        return ang - 2*np.pi
    return ang

def getAngleDif(ang1, ang2):
    """
    Return the angle difference between ang1 and ang2
    """
    # First, put between -pi, pi
    ang1 = getInPi(ang1)
    ang2 = getInPi(ang2)

    # Then return the difference in -pi, pi
    return getInPi(ang2-ang1)


def expandPlotLim(lim, scale):
    m, M = lim
    if (m > 0):
        m = m*(1-scale)
    else:
        m = m*(1+scale)
    M = M*(1+scale)
    return [m, M]


def hashFct(str):
    """
    Repetable hash value
    """
    hash = 0
    for ch in str:
        hash = (hash*281 ^ ord(ch)*997) & 0xFFFFFFFF
    return hash


def saveSimuFile(path, ts, Xs, Us, Ts, solver):
    """
    Save the result of a simulation into a simulation file
    """
    with open(path, 'w') as f:
        f.write(json.dumps({'ts':ts.tolist(),
                         'Xs':[X.toJSON() for X in Xs],
                         'Us':[U.toJSON() for U in Us],
                         'Ts':Ts.tolist(),
                         'globalGeom':serializeGeom(solver.getGlobalGeom())}))

def loadSimuFile(path, solver):
    """
    Load a simulation file and output three arrays:
        - ts: time
        - Xs: state vectors
        - Us: command vectors
    """
    with open(path, 'r') as f:
        data = json.load(f)

    # Update the geometry of the boat
    solver.updateGlobalGeom(deserializeGeom(data['globalGeom']))

    ts = data['ts']
    Xs = []
    Us = []
    Ts = data['Ts']

    for i in range(len(ts)):
        Xs.append(StateVector.fromJSON(data['Xs'][i], solver))
        Us.append(CommandVector.fromJSON(data['Us'][i]))

        progress = 100*(i+1)/len(ts)
        if (int(progress)%5 == 0) and (int(100*i/len(ts))%5 != 0):
            print("{:.1f}%".format(progress))

    return np.array(ts), np.array(Xs), np.array(Us), np.array(Ts)


def floatifyVar(var):
    try:
        return float(var.get())
    except:
        return 0
    
def getOptimalSailAng(solver, X, i):
    """
    Return the optimal sail angle
        - solver: Solver object
        - X     : state vector of the boat
        - i     : target wind incidence
    """
    # Get the apparent wind direction
    rwindvel = PointSpeed.getVelocity(solver, solver.getBoatOrigin(), Referential.BOAT, Referential.WIND)
    windAng = -rwindvel.getHeading()
    boatAng = -X._x_ang[2]

    apparentWindAng = getAngleDif(windAng, boatAng)

    # Compute the Optimal angle between the sail and wind
    if (np.abs(apparentWindAng) < np.deg2rad(180)):
        # Go upwing
        if (apparentWindAng < 0):
            optimalSailAngle = apparentWindAng + i
        else:
            optimalSailAngle = apparentWindAng - i
    else:
        # Go downwind (45 deg cone)
        if (apparentWindAng < 0):
            optimalSailAngle = apparentWindAng + np.deg2rad(90)
        else:
            optimalSailAngle = apparentWindAng - np.deg2rad(90)

    return optimalSailAngle
    

def getPlotColor(n):
    """
    Giving a number, return a color
    """
    colors = "rbcgyk"
    return colors[n%len(colors)]

def copyPyplotAxe(axFrom, axTo):
    for trace in axFrom.get_lines():
        axTo.plot(trace.get_xdata(), 
             trace.get_ydata(), 
             trace.get_linestyle(),
             color=trace.get_color(),
             label=trace.get_label())
    axTo.autoscale()
    axTo.legend()

"""
    ==========================================
            UTILS USING THE GEOM DIC
    ==========================================
"""

def serializeGeom(geom):
    """
    Serialize a geometry dictionnary
    """
    # Need to serialise the <Units> object
    save = {}
    for elm in geom:
        save[elm] = {}
        for param in geom[elm]:
            save[elm][param] = {'value':geom[elm][param]['value'],
                                'unit':geom[elm][param]['unit'].value,
                                'info':geom[elm][param]['info']}
    return save
    
def deserializeGeom(save):
    """
    De-Serialize a geometry dictionnary
    """
    # Need to recreate the <Units> objects
    geom = {}
    for elm in save:
        geom[elm] = {}
        for param in save[elm]:
            geom[elm][param] = {'value':save[elm][param]['value'],
                                'unit':Units.fromName(save[elm][param]['unit']),
                                'info':save[elm][param]['info']}
    return geom

"""
    PHYSICAL FUNCTION
"""

def getRectangularPrismInertia(a, b, c, M):
    """Return the inertia matrix of a rectangular prism of 
    homogenous mass `M`

        `a` size in the axis x 
        `b` size in the axis y 
        `c` size in the axis z 
    """
    return np.array([[b**2+c**2, 0, 0],
                     [0, a**2+c**2, 0],
                     [0, 0, a**2+b**2]])*M/12


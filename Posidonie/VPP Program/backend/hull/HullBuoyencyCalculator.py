import meshlib.mrmeshpy as mr
import meshlib as mm
import matplotlib.pyplot as plt
import json

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *
from frontend.simu.Boat import *
from backend.hull.InputModel import *
from utils.Utils import *
from utils.Force import *

"""
This file compute the buoyency force on the hull for
various angles and displacement
"""

def getCdgFromSTL(stlpath):
    """
    Return the postion of the CdG of a STL file
    """
    cdg = mr.loadMesh(stlpath).findCenterFromFaces()
    return np.array([cdg.x, cdg.y, cdg.z])
    

def loadHull(stlpath, modelLength, centerOfRotation, cdg, longiDirection, topBottomDirection):
    """
    Load the hull and orient it corectly. Args:
        - stlpath         : path of the model
        - modelLength     : length of the boat
        - centerOfRotation: position of the center of rotation
        - cdg             : Center of gravity
        - longiDirection  : longi Direction of the hull
        - topBottomDirection: top bottom Direction of the hull
    """
    # First, open and scale to the proper length
    mesh = mr.loadMesh(stlpath)
    #print("Loading",modelName)

    # find the minimal bow that hold the hull
    bounding_box = mesh.computeBoundingBox()

    # extract the max dimention of this box
    sizeX = bounding_box.max.x - bounding_box.min.x
    sizeY = bounding_box.max.y - bounding_box.min.y
    sizeZ = bounding_box.max.z- bounding_box.min.z

    size = max(sizeX, sizeY, sizeZ)
    # scale it properly
    mesh.transform(mr.AffineXf3f.linear(mr.Matrix3f.scale(modelLength/size)))
    # Scale the position of the cg and cr too
    _cdg = cdg.copy()*modelLength/size
    _centerOfRotation = centerOfRotation.copy()*modelLength/size

    # Then put the center of rotation at the origin (0,0,0)
    dx, dy, dz = -_centerOfRotation
    translation = mr.AffineXf3f.translation(mr.Vector3f(dx, dy, dz))
    mesh.transform(translation)
    # Translate the cg and cr too
    _cdg += np.array([dx, dy, dz])
    _centerOfRotation += np.array([dx, dy, dz])

    # Then rotate it in the correct orientation
    # we want to transform longiDirection into "+X"
    axis, ang = findRotation(longiDirection, "+X")
    topBotAxis = topBottomDirection

    if (axis != None):
        # rotation
        longiRot = mr.AffineXf3f.linear(mr.Matrix3f.rotation(axis2vec[axis], ang))
        mesh.transform(longiRot)
        # Rotate the cg too
        _cdg, _centerOfRotation = rotation([_cdg, _centerOfRotation], np.zeros(3), axis2dir[axis], ang)

        # Look what topBottomDirection had became:
        newTopBottomDirection = rotation([getArrayFromSignedAxis(topBottomDirection)], np.zeros(3), axis2dir[axis], ang)[0]
    
        # finally, put the top of the hull toward +Z
        topBotAxis = getSignedAxisFromArray(newTopBottomDirection)

    axis, ang = findRotation(topBotAxis, "+Z")

    if (axis != None):
        # rotation
        topBotRot = mr.AffineXf3f.linear(mr.Matrix3f.rotation(axis2vec[axis], ang))
        mesh.transform(topBotRot)
        # Rotate the cg too
        _cdg, _centerOfRotation = rotation([_cdg, _centerOfRotation], np.zeros(3), axis2dir[axis], ang)

    # The hull is now in the good orientation

    # find the minimal box that hold the hull
    bounding_box = mesh.computeBoundingBox()

    #mr.saveMesh(mesh,PATH_3DMODEL_FOLDER + "ok_" + modelName)
    # done! Return the mesh, and other information about the boat
    dim = np.array([bounding_box.max.x-bounding_box.min.x,
                    bounding_box.max.y-bounding_box.min.y,
                    bounding_box.max.z-bounding_box.min.z])
    
    #print(modelName,"loaded!")
    return {'mesh':mesh, 
            'cdg':_cdg, 
            'sizeFactor':size,
            'cor':_centerOfRotation,
            'box':{'max':np.array([bounding_box.max.x, bounding_box.max.y, bounding_box.max.z]),
                   'min':np.array([bounding_box.min.x, bounding_box.min.y, bounding_box.min.z])}, 
            }


def findRotation(axe1, axe2):
    """
    Find a rotation (axis and angle) that put the vector
    axe1 to axe2.

    Note: axe(i) = "[+-][XYZ]"
    """
   
    axe1_pos = axe1[0] == '+'
    axe2_pos = axe2[0] == '+'

    axe1_vec = axe1[1]
    axe2_vec = axe2[1]

    if (axe1_vec == axe2_vec):
        # same axes, verify if same dimention
        if (axe1_pos == axe2_pos):
            # nothing to do
            return axe1_vec, 0
        else:
            # rotation of 180 in a different axis
            if (axe1_vec == "X"):
                return "Y", np.deg2rad(180)
            else:
                return "X", np.deg2rad(180)
    else:
        # different axes, need to found the one perpendicular
        axes = set(['X', 'Y', 'Z'])
        axes.discard(axe1_vec)
        axes.discard(axe2_vec)
        # Only one left since axe1_vec != axe2_vec
        thirdAxis = axes.pop()
        # Then need to find the angle. Need to verify if
        # axe1_vec, axe2_vec is in the direct way
        directWay = 'XYZX'
        ang = np.deg2rad(90) # we suppose it's direct
        for i in range(len(directWay)-1):
            if (directWay[i] == axe1_vec):
                if (directWay[i+1] != axe2_vec):
                    # indirect
                    ang = - ang
                break
        # if the pos are diffrenent, need to change the way
        if (axe1_pos != axe2_pos):
            ang = -ang
        # Finally, apply the rotation
        return thirdAxis, ang

axis2vec = {'X':mr.Vector3f(1, 0, 0),
            'Y':mr.Vector3f(0, 1, 0),
            'Z':mr.Vector3f(0, 0, 1)}

axis2array = {'X':np.array([1, 0, 0]),
              'Y':np.array([0, 1, 0]),
              'Z':np.array([0, 0, 1])}

axis2dir = {'X':Dir.X,
            'Y':Dir.Y,
            'Z':Dir.Z}

def getArrayFromSignedAxis(axis):
    arr = axis2array[axis[1]]
    if (axis[0] == "-"):
        arr = -arr
    return arr

def getSignedAxisFromArray(arr):
    if (arr[0] != 0):
        if (arr[0] > 0):
            return '+X'
        return "-X"
    if (arr[1] != 0):
        if (arr[1] > 0):
            return '+Y'
        return "-Y"
    if (arr[2] > 0):
        return '+Z'
    return '-Z'


def getWaterPlane(z):
    """
    Return a box for z<0, used to compute the displaced volume
    """
    sizeVector = mr.Vector3f(2*WATER_SIZE, 2*WATER_SIZE, WATER_SIZE+z)
    baseVector = mr.Vector3f(-WATER_SIZE, -WATER_SIZE, -WATER_SIZE)
    return mr.makeCube(sizeVector, baseVector)


class HullMesh():
    def __init__(self, argdic):
        loadedHull = loadHull(argdic['stlpath'], 
                              argdic['length'], 
                              np.array(argdic['originalCdg']),
                              np.array(argdic['originalCdg']), 
                              argdic['longiDir'], 
                              argdic['topBotDir'])
        self.argdic = argdic
        self.mesh = loadedHull['mesh']
        self.box = loadedHull['box']
        self.trueCdg = loadedHull['cdg']
        self.sizeFactor = loadedHull['sizeFactor']
        self.trueCenterOfRotation = loadedHull['cor']

    def computeVolume(self, mesh, z):
        """
        Compute the displaced volume of the mesh <mesh>
        for an altitude <z> (z>0 -> higher in the air) at an angle <ang> and the 
        center of buoyency
        Note: we have to compense the centerOfRotation z-postion
        """
        water = getWaterPlane(z)
        intersection = mr.boolean(mesh, water, mr.BooleanOperation.Intersection)
        
        #deduce the volume and center of buoyency
        volume = intersection.mesh.volume()
        buoyencyCenter = intersection.mesh.findCenterFromFaces()
        
        #mr.saveMesh(intersection.mesh,PATH_3DMODEL_FOLDER + "ok_" + modelName)
        return volume, buoyencyCenter

    def buildDisplacementFile(self, path, callback=None, abordFct=None):
        """
        Build a file with numerous orientation and altitude of the boat
        in order to be lineraly interpoled, and give a general formula
        for the displacement and center of buoyency
        Note: - yaw not useful (sea has a revolution symetry)
              - Symetry on the (oxz) plane : only consider roll>0

        Args:
            (intern) argsDic: A dictionnary with the following information:
                - maxRoll : The maximum roll modeled
                - maxPitch: The maximum pitch modeled
                - minPitch: The minimum pitch modeled
                - NRoll   : the number of point for the roll model
                - NPitch  : the number of point for the pitch model
                - NZ      : the number of point for the z model
            
            - path    : The path to write the model file
            - callback: function to plot the progress (signature: callback(p, n), 0<=p<=1, n number total of point)
            - abordFct: function to know if the computation must be aborted
        """
        argsDic = self.argdic
        maxRoll  = argsDic['roll']['max']
        maxPitch = argsDic['pitch']['max']
        minPitch = argsDic['pitch']['min']
        NRoll    = argsDic['roll']['N']
        NPitch   = argsDic['pitch']['N']
        NZ       = argsDic['Z']['N']
        out = {}
        print("Creating the displacement model")
        print(f"maxRoll={maxRoll}\nmaxPitch={maxPitch}\nminPitch={minPitch}\nNRoll={NRoll}\nNPitch={NPitch}\nNZ={NZ}")

        loop = 0
        Ntot = NRoll*NPitch*NZ

        for roll in np.linspace(0, maxRoll, NRoll):
            out[roll] = {}
            
            for pitch in np.linspace(minPitch, maxPitch, NPitch):
                out[roll][pitch] = {}
                # Rotate a new mesh
                newHull = loadHull( argsDic['stlpath'], 
                                    argsDic['length'], 
                                    np.array(argsDic['originalCdg']),
                                    np.array(argsDic['originalCdg']), 
                                    argsDic['longiDir'], 
                                    argsDic['topBotDir'])
                newMesh = newHull['mesh']

                # First, Roll
                rollRot = mr.AffineXf3f.linear(mr.Matrix3f.rotation(axis2vec['X'], roll))
                newMesh.transform(rollRot)

                # Then, pitch
                pitchRot = mr.AffineXf3f.linear(mr.Matrix3f.rotation(axis2vec['Y'], pitch))
                newMesh.transform(pitchRot)

                # Finaly translations from the minamal and maximal height of the bounding box
                bounding_box = newMesh.computeBoundingBox()
                minz = bounding_box.min.z
                maxz = bounding_box.max.z

                for z in np.linspace(minz, maxz, NZ):
                    vol, cdc = self.computeVolume(newMesh, z)

                    # The cdc is given in the sea referential.
                    # We move it back to the boat referential
                    cdc = np.array([cdc.x, cdc.y, cdc.z])
                    cdc = pointRotation(cdc, np.array([roll, pitch, 0]))

                    out[roll][pitch][z] = {'vol':vol, 'cdc':list(cdc)}

                    loop += 1
                    print("{}/{} ({:.2f}%)".format(loop, Ntot, 100*loop/Ntot))
                    if callback != None:
                        callback(loop/Ntot, Ntot)

                    if abordFct != None:
                        if abordFct():
                            # Abort
                            return
                        
        print("Computation done, saving...")

        # Get fully imerged volume and cdc
        imergedVol, imergedCdc = self.computeVolume(loadHull(argsDic['stlpath'], 
                                    argsDic['length'], 
                                    np.array(argsDic['originalCdg']),
                                    np.array(argsDic['originalCdg']), 
                                    argsDic['longiDir'], 
                                    argsDic['topBotDir'])['mesh'], self.box['max'][2])

        # Save the model
        model = {'info':argsDic,
                 'buoyencyMod':out,
                 'box':{'max':list(self.box['max']),
                        'min':list(self.box['min'])},
                 'cdg':self.trueCdg.tolist(),
                 'cor':self.trueCenterOfRotation.tolist(),
                 'imergedVol':imergedVol,
                 'imergedCdc':list(imergedCdc),
                 'mass':argsDic['mass'],
                 'sizeFactor':self.sizeFactor,
                 'inertiaMatrix':argsDic['inertia']}
        
        with open(path, 'w') as f:
            f.write(json.dumps(model, indent=4))


class HullBuoyencyCalculator():

    def __init__(self, solver):
        #self.loadModelFile()
        self._solver = solver
        self.mass = 0
        self.inertiaMatrix = np.eye(3)
        self.imergedVol = 0
        self.model = {}

    def getForce(self):
        """
        Return the buoyency force exerced on the hull
        The center of buoyency is exprimed in the boat referential
        The force is exprimed in the sea referential
        """
        ang = self._solver.getBoatAng()
        z = -self._solver.getBoatPos()[2]
        # z is the sea level, thus opposed to the boat altitude

        roll = ang[ROLL_AXIS]
        pitch = -ang[PITCH_AXIS]

        # Verify the asked point is within the model
        if (np.abs(roll) > BUOYENCY_MOD_MAX_ROLL):
            if (roll > 0):
                roll = BUOYENCY_MOD_MAX_ROLL
            else:
                roll = -BUOYENCY_MOD_MAX_ROLL
            print("[WARNING] - roll out of bound")
            self._solver.warning(SimWarning.OUTOFBOUND)

        if (pitch > BUOYENCY_MOD_MAX_PITCH):
            pitch = BUOYENCY_MOD_MAX_ROLL
            print("[WARNING] - pitch above bounds")
            self._solver.warning(SimWarning.OUTOFBOUND)

        if (pitch < BUOYENCY_MOD_MIN_PITCH):
            pitch = BUOYENCY_MOD_MIN_PITCH
            print("[WARNING] - pitch under bounds")
            self._solver.warning(SimWarning.OUTOFBOUND)

        """
        We use an LINEAR interpalation between the 3D input:
        roll, pitch, z
        """
        value = self.getValueAt(roll, pitch, z)

        vol = value[0]
        cdc = value[1:]

        return Force(Vector(self._solver, -vol*PHY_G_VECTOR*PHY_RHO_SWATER, Base.SEA), 
                     Point(self._solver, cdc, Referential.BOAT))
    
    def getValueAt(self, aroll, pitch, z):
        """
        We use an LINEAR interpalation between the 3D input:
        roll, pitch, z
        """
        roll = np.abs(aroll)
        # get the model inferor and superior roll
        previousRoll,followingRoll = getFraming(self.model, roll)

        # do the same for the pitchs
        PPPitch, PFPitch = getFraming(self.model[previousRoll], pitch)
        FPPitch, FFPitch = getFraming(self.model[followingRoll], pitch)

        # do the same for Z
        PPPZ, PPFZ = getFraming(self.model[previousRoll][PPPitch], z)
        PFPZ, PFFZ = getFraming(self.model[previousRoll][PFPitch], z)
        FPPZ, FPFZ = getFraming(self.model[followingRoll][FPPitch], z)
        FFPZ, FFFZ = getFraming(self.model[followingRoll][FFPitch], z)

        # interpolate everthing
        PP = interpolDXXV(self.model[previousRoll][PPPitch], PPPZ, PPFZ, z)
        PF = interpolDXXV(self.model[previousRoll][PFPitch], PFPZ, PFFZ, z)
        FP = interpolDXXV(self.model[followingRoll][FPPitch], FPPZ, FPFZ, z)
        FF = interpolDXXV(self.model[followingRoll][FFPitch], FFPZ, FFFZ, z)

        P = interpolYYXXV(PP, PF, PPPitch, PFPitch, pitch)
        F = interpolYYXXV(FP, FF, FPPitch, FFPitch, pitch)

        value = interpolYYXXV(P, F, previousRoll, followingRoll, roll)
        
        # If the roll was negative, inverse the y coordinate
        if (aroll > 0):
            value[2] = -value[2]

        return value

    def loadModelFile(self, path):
        # load the raw model
        try:
            with open(path, 'r') as f:
                fileModel = json.load(f)
        except Exception as e:
            print(e)
            print("[ERROR] - Impossible to load buoyency model file")
            self.model = {}
            self.cdg = np.zeros(3)
            self.imergedVol = 0
            self.box = {'max':np.ones(3),
                        'min':np.zeros(3)}
            return

        # Then convert the keys as float
        rawModel = fileModel['buoyencyMod']
        self.box = {'max':np.array(fileModel['box']['max']),
                    'min':np.array(fileModel['box']['min'])}
        self.cdg = np.array(fileModel['cdg'])
        self.mass = fileModel['mass']
        self.imergedVol = fileModel['imergedVol']
        self.inertiaMatrix = np.array(fileModel['inertiaMatrix'])
        self.info = fileModel['info']
        self.sizeFactor = fileModel['sizeFactor']
        self.model = {}
        for roll in rawModel:
            self.model[float(roll)] = {}
            for pitch in rawModel[roll]:
                self.model[float(roll)][float(pitch)] = {}
                for z in rawModel[roll][pitch]:
                    fvalue = rawModel[roll][pitch][z]
                    self.model[float(roll)][float(pitch)][float(z)] = np.array([fvalue['vol']]+fvalue['cdc'])


    def getCdG(self):
        """
        Return the position of the center of gravity
        """
        return Point(self._solver, self.cdg, Referential.BOAT)
    
    def getBox(self):
        """
        Return the dimention of the hull
        """
        return self.box
    
    def getWeight(self):
        """
        Return the mass of the hull
        """
        return self.mass
    
    def getImergedVolume(self):
        return self.imergedVol
    
    def getInertiaMatrix(self):
        """
        Return the inertia matrix expressed at the center of gravity
        """
        return Matrix(self._solver, self.inertiaMatrix, Base.BOAT)

    def getRollLimit(self):
        """
        Return the maximum and minimum roll considered in the model:
        (min, max)
        """
        if (len(self.model) == 0):
            return None, None
        return min(self.model.keys()), max(self.model.keys())
    
    def getPitchLimit(self):
        """
        Return the maximum and minimum pitch considered in the model:
        (min, max)
        """
        if (len(self.model) == 0):
            return None, None
        r = list(self.model.keys())[0]
        return min(self.model[r].keys()), max(self.model[r].keys())
    
    def getZLimit(self, roll, pitch):
        """
        Return the maximum and minimum z considered in the model:
        for a given roll and pitch (min, max)
        """
        if (len(self.model) == 0):
            return None, None
        croll = getFraming(self.model, roll)[0]
        cpitch = getFraming(self.model[croll], pitch)[0]

        return min(self.model[croll][cpitch].keys()), max(self.model[croll][cpitch].keys())

    def coordinateToSTL(self, pos):
        """
        Return the array pos in the boat's STL original coordinate system
        """
        newpos = pos.copy()

        # First, undo the rotation
        axis, ang = findRotation(self.info['topBotDir'], "+Z")
        M = getMatRot(axis2dir[axis], -ang)
        newpos = pointMatRotation(newpos, np.linalg.inv(M)) # inverse the rotation

        # Then the rotation
        axis, ang = findRotation(self.info['longiDir'], "+X")
        M = getMatRot(axis2dir[axis], -ang)
        newpos = pointMatRotation(newpos, np.linalg.inv(M)) # inverse the rotation

        # Continue with the size
        newpos = newpos*self.sizeFactor/self.info['length']

        # Finish with the translation
        translation = np.array(self.info['originalCor'])
        newpos += translation

        return newpos
    

"""if __name__ == "__main__":
    hullMesh = HullMesh()
    hullMesh.buildDisplacementFile()"""


if __name__ == "__main__":
    path = r"C:/Users/alber/Desktop/Posidonie/Conception/Coque/HullV5.stl"
    mesh = loadHull(path,
             1.871,
             np.array([0.254,0.0,85.52]),
             np.array([0.254,0.0,85.52]),
             '+X',
             '-Z')['mesh']
    
    mesh = mr.loadMesh(path)

    bounding_box = mesh.computeBoundingBox()

    # extract the max dimention of this box
    sizeX = bounding_box.max.x - bounding_box.min.x
    sizeY = bounding_box.max.y - bounding_box.min.y
    sizeZ = bounding_box.max.z- bounding_box.min.z

    size = max(sizeX, sizeY, sizeZ)
    # scale it properly
    mesh.transform(mr.AffineXf3f.linear(mr.Matrix3f.scale(1.871/size)))

    bounding_box = mesh.computeBoundingBox()

    # extract the max dimention of this box
    sizeX = bounding_box.max.x - bounding_box.min.x
    sizeY = bounding_box.max.y - bounding_box.min.y
    sizeZ = bounding_box.max.z - bounding_box.min.z
    
    print("Raw file volume: ",mesh.volume()*1e3,"L")
    print("Box volume =",sizeX*sizeY*sizeZ*1e3,"L")
    print("Model size=",sizeX,sizeY,sizeZ)
    
    mr.saveMesh(mesh,PATH_3DMODEL_FOLDER + "moved.stl")
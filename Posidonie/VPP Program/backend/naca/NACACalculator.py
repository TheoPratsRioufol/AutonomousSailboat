
from enum import Enum
import csv
import matplotlib.pyplot as plt
import time

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Force import *
from utils.Utils import *
from config.Config import *
from backend.naca.NACAInput import *

class Fluids(Enum):
    AIR = "air"
    WATER = "water"

    def fromName(name):
        """
        Return a fluid type from it name
        """
        for e in Fluids:
            if (e.value == name):
                return e
            
        raise Exception("Unknow fluid type: "+str(name))

    def getRho(fluid):
        """
        Return the volumic mass of a fluid
        """
        if (fluid == Fluids.WATER):
            return PHY_RHO_WATER
        
        if (fluid == Fluids.AIR):
            return PHY_RHO_AIR
        
        raise Exception(f"Unknow rho value for {str(fluid)}")

class NACACalculator():
    def __init__(self, solver):
        """
        This class is used to compute a fluid force of a NACA profile
        """
        self._solver = solver
        self.load()

    def getFluidForce(self, linepf, lengthpf, fluidType, refComp, refFluid, profileType):
        """
        Compute the fuild force exerced by a fluid of type <fluidType> on a section
        All the origin point of the NACA section (i.e. point with the same position 
        as in COMSOL), are given by the parametric function linepf:
            x = linepf(t),   for 0 <= t <= 1
        The length of those section are givent by the paremetric function lengthpf:
            lentth = lengthpf(t), for 0 <= t <= 1

        refComp    : referential of the wing
        refFluid   : referential of the fluid

        """
        if not isinstance(fluidType, Fluids):
            raise TypeError("Field <fluidType> of NACACalculator must be a Fluid object")
        
        if not isinstance(refFluid, Referential):
            raise TypeError("Field <refFluid> of NACACalculator must be a Referential object")
        
        if not isinstance(refComp, Referential):
            raise TypeError("Field <refComp> of NACACalculator must be a Referential object")
        

        zeroVector = Vector(self._solver, np.zeros(3), getBaseFomRef(refComp))
        fluidForce = Force(zeroVector, Moment(zeroVector))

        # perform the integration
        lastPoint = linepf(0)
        for t in np.linspace(0, 1, 10):

            # compute the section's center coordinate
            center = linepf(t)

            # compute the section height:
            dz = np.abs((center-lastPoint).valueIn(refComp)[2])

            # compute the elementary fluid force and sum it
            fluidForce += self.getElementaryFluidForce(center, lengthpf(t), dz, refComp, refFluid, fluidType, profileType)

            lastPoint = center

        return fluidForce


    def getElementaryFluidForce(self, pos, length, dz, refComp, refFluid, fluidType, profileType):
        """
        Return an elementary force exerced on a section located in <pos>
        <refComp> : referential of the wing
        <refFluid>: referential of the fluid
        """
        #zeroVector = Vector(self._solver, np.zeros(3), Base.BOAT)
        #return Force(zeroVector, Moment(zeroVector))

        # compute the fluid velocity
        # we assume that the component velocity is equal to the boat velocity!

        # compute   V (pos, refComp/refFluid)
        fluidV = -PointSpeed.getVelocity(self._solver, pos, refComp, refFluid).getSpeed().valueIn(getBaseFomRef(refComp))
        #fluidV = np.array([1.0,0,0])
        speed = np.linalg.norm(fluidV)
        # The fluid speed only correspond to the xy plane
        fspeed = np.sqrt(fluidV[0]**2 + fluidV[1]**2)

        # Compute the volumic mass of the fluid
        rhoFluid = Fluids.getRho(fluidType)
        
        if speed == 0:
            zeroVector = Vector(self._solver, np.zeros(3), Base.BOAT)
            return Force(zeroVector, Moment(zeroVector))

        # Normalise the speed vector to get the incidence
        fdir = fluidV/speed
        cosang = fdir[0]
        sinang = -fdir[1]
        ang = np.arctan2(sinang, cosang)
        
        # Transform ang into an incidence angle
        ang = np.pi + ang
        if ang > np.pi:
            ang -= 2*np.pi

        # get the simulated force for the same fluid incidence angle
        
        Fx, Fy, Mz = self.interpolateNACA(ang, profileType)
        #Fx, Fy = 0, 0

        # The fluid force is:
        # f = 0.5*rhof*length*dz*[Cx,Cy]*u^2
        Fx = Fx*length*dz*rhoFluid*fspeed**2
        Fy = Fy*length*dz*rhoFluid*fspeed**2
        Mz = Mz*length*dz*rhoFluid*fspeed**2
        
        return Force(Vector(self._solver, np.array([Fx, Fy, Mz]), getBaseFomRef(refComp)),
                     pos)


    def interpolateNACA(self, ang, profileType):
        """
        Return the simulated Fx, Fy, Mz value for a given <ang> and <profileType>
        Thus, Fx, Fy and Mz are in the profile's basis
        Note: Fx is normalised in term of width an length and volumic mass, and the speed term must be added
        """
        if (profileType not in self.model):
            raise Exception("Unknow profile type: "+str(profileType)+" for NACA model")
        
        if (ang < -np.pi) or (ang > np.pi):
            raise Exception('i out of bound!')
        
        # symetry on the incidence angle
        angAbs = np.abs(ang)

        # OUTDATED - get the model inferor and superior ang
        # Pang ,Fang = getFraming(self.model[fluidType.value], angAbs)
        # Fxym = interpolDXXV(self.model[fluidType.value], Pang ,Fang, angAbs)

        Fxym = [self.polyModel[profileType]['x'](angAbs),
                self.polyModel[profileType]['y'](angAbs),
                self.polyModel[profileType]['z'](angAbs)]

        # Return Fx, Fy and Mz
        if (ang < 0):
            return Fxym[0], -Fxym[1], -Fxym[2]
        else:
            return Fxym[0], Fxym[1], Fxym[2]
        

    def load(self):
        """
        Load the NACA model file
        """
        self.model = {}
        rhoSimuled = Fluids.getRho(Fluids.fromName(fluidSimuled))

        for profileType in profileTypesSimuled:
            self.model[profileType] = {}
            
            with open(PATH_NACA_SIMU_FOLDER + modelPrefix + profileType + ".csv", newline='') as csvfile:
                csvReader = csv.reader(csvfile, delimiter=',')
                for row in csvReader:
                    if (len(row[0]) > 0) and (row[0][0] == '%'):
                        continue
                    Fxyz = np.array([float(r) for r in row[1:]])
                    # Remove the simulation speed, section length, and fluid volumic mass
                    Fxyz = Fxyz/(slength*rhoSimuled*speedSimu**2)

                    FxFluid = Fxyz[0]
                    FyFluid = Fxyz[1]
                    MzFluid = Fxyz[2]

                    # The incidence angle must be in rad
                    i = np.deg2rad(float(row[0]))
                    
                    # Then we move Fx, Fy back to the wing referential
                    FxWing = FxFluid*np.cos(i) + FyFluid*np.sin(i)
                    FyWing = FyFluid*np.cos(i) - FxFluid*np.sin(i)

                    # save
                    self.model[profileType][i] = np.array([FxWing, FyWing, MzFluid])
            
            # then, create the poly interpolation for fast computing
            self.polyModel = {}
            for profile in self.model:
                angles = sorted(self.model[profile].keys())
                FWing = np.array([self.model[profile][a] for a in angles])
                self.polyModel[profile] = {'x':np.poly1d(np.polyfit(angles, FWing[:,0], 15)),
                                         'y':np.poly1d(np.polyfit(angles, FWing[:,1], 15)),
                                         'z':np.poly1d(np.polyfit(angles, FWing[:,2], 10))}

    def getModelFileAngs(self, fluid):
        """
        Return the angles computed in the model file (COMSOL)
        """
        return np.array(sorted(self.model[fluid].keys()))
    
    def getProfiles(self):
        """
        Return the list of the Profiles Models availables
        """
        return list(self.model.keys())
    
    def getModelDic(self):
        """
        Return the model dictionnary
        """
        return self.model

    def plot(self):
        """
        Plot the loaded model
        """
        i = 0
        for fluid in profileTypesSimuled:
            angles = np.array(sorted(self.model[fluid].keys()))
            plt.subplot(1, 2, 2*i+1)
            plt.plot(angles, [self.model[fluid][ang][0] for ang in angles], 'r', label='FxWing for '+fluid)
            plt.plot(angles, [self.model[fluid][ang][1] for ang in angles], 'b', label='FyWing for '+fluid)
            plt.plot(angles, self.polyModel[fluid]['x'](angles), 'r*')
            plt.plot(angles, self.polyModel[fluid]['y'](angles), 'b*')

            plt.subplot(1, 2, 2*i+2)
            plt.plot(angles, [self.model[fluid][ang][2] for ang in angles], 'r', label='Mz for '+fluid)
            plt.plot(angles, self.polyModel[fluid]['z'](angles), 'r*')
            
            i += 1
            
        plt.legend()
        plt.show()

        """for fluid in fluidTypeSimuled:
            angles = sorted(self.model[fluid].keys())
            plt.plot(angles, [self.model[fluid][ang][0]/self.model[fluid][ang][1] for ang in angles], label='Fx/Fy for '+fluid)
        plt.legend()
        plt.show()
        """   

if __name__ == "__main__":
    NACACalculator(None).plot()
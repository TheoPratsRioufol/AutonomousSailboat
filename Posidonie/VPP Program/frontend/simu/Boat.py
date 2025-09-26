
import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from backend.solver.Solver import *
from config.Config import *
from utils.Force import *
from backend.solver.EDOSolver import *

def addVectorVisual(scene, name, vec, point, color=COLOR_VECTOR, dispName=True):
    """
    Add a vector display to the scene starting at the point point
    """
    scene[name] = {'vec':vec.valueIn(Base.SEA), 
                   'point':point.valueIn(Referential.BOAT),
                   'color':color}
    if dispName:
        scene[name]['text'] = name

def addMomentVisual(scene, name, moment, point, color=COLOR_MOMENT, dispName=False):
    """
    Add a vector display to the scene starting at the point point
    """
    mv = moment.getVec().valueIn(Base.SEA)
    mv = Vector(moment.getVec()._solver, np.array([0, 0, mv[-1]]), Base.SEA)
    scene[name] = {'vec':mv.valueIn(Base.SEA), 
                   'point':point.valueIn(Referential.BOAT),
                   'color':color}
    if dispName:
        scene[name]['text'] = name

def addSpeedVisual(scene, name, rspeed, point):
    """
    Add a speed vector display to the scene starting at the point point
    """
    scene[name] = {'vec':rspeed.getSpeed().valueIn(Base.SEA)*10, 
                   'point':point.valueIn(Referential.BOAT),
                   'text':name,
                   'color':COLOR_SPEED}

class Boat():
    def __init__(self):
        self.solver = Solver()
        self.edoSolver = EDOSolver(self.solver.getX0(),
                                   self.solver.F,
                                   self.solver.getNavigator().Fu,
                                   self.solver.getNavigator().T,
                                   self.solver.getNavigator().getS0(),
                                   self.solver.getNavigator().getU0(),
                                   self.solver.getNavigator().getEpsi0())
        
    def getSolver(self):
        return self.solver
    
    def getEDOSolver(self):
        return self.edoSolver

    def getScene(self, displayPolicy=set()):
        """
        Return the scene (= set of components) that composes the boat
        Dictionary of objects with keys:
            polygons : the polygons that form the object
            color: the color of the object

        displayPolicy is used to know which element should be displayed or not
        """
        scene = {}

        # First, DRAW THE COMPONENTS
        # load them in the boat referential
        if ('boat' in displayPolicy):
            scene['hull']  = {'polygons':self.solver.getHull().getPolygons(), 'color':COLOR_HULL, 'fill':True, 'hitbox':True}
            scene['drift'] = {'polygons':self.solver.getDrift().getPolygons(), 'color':COLOR_DRIFT, 'fill':True, 'hitbox':True}
            scene['sail']  = {'polygons':self.solver.getSail().getPolygons(), 'color':COLOR_SAIL, 'fill':True, 'hitbox':True}
            scene['rudder']  = {'polygons':self.solver.getRudder().getPolygons(), 'color':COLOR_RUDDER, 'fill':True, 'hitbox':True}

        if ('integral' in displayPolicy):
            scene['integd']  = {'polygons':[[self.solver.getDrift().linepf(t).valueIn(Referential.BOAT) for t in np.linspace(0, 1, 20)]], 'color':COLOR_INTEG, 'fill':False}
            scene['integs']  = {'polygons':[[self.solver.getSail().linepf(t).valueIn(Referential.BOAT) for t in np.linspace(0, 1, 20)]], 'color':COLOR_INTEG, 'fill':False}

            def getPolygonFromIntegTrace(linepf, lengthpf):
                # Return a polygon representing the outline of the integration
                lefts = []
                rigths = []
                for t in np.linspace(0, 1, 10):
                    center = linepf(t).valueIn(Referential.BOAT)
                    width = lengthpf(t)
                    lefts.append(center - np.array([width*(1-0.25), 0, 0]))
                    rigths.append(center + np.array([width*0.25, 0, 0]))
                return [lefts + rigths]

            scene['integdoutline']  = {'polygons':getPolygonFromIntegTrace(self.solver.getDrift().linepf, self.solver.getDrift().lengthpf), 'color':COLOR_INTEG_OUTLINE, 'fill':False}
            
            
        # Then, DRAW THE PHYSICS
        if ('geom' in displayPolicy):
            cdg = self.solver.getBoatCdg()
            scene['cdg'] = {'point':cdg.valueIn(Referential.BOAT), 'color':COLOR_CDG, 'text':'cdg'}

            cdg = self.solver.getDrift().getCdg()
            scene['cdgd'] = {'point':cdg.valueIn(Referential.BOAT), 'color':COLOR_CDG, 'text':'cdgd'}

            cdc = self.solver.getHull().getBuoyencyForce().getPt()
            scene['cdc'] = {'point':cdc.valueIn(Referential.BOAT), 'color':COLOR_CDC, 'text':'cdc'}


        if ('force' in displayPolicy):
            #try:
            if True:
                addVectorVisual(scene, 'aero', self.solver.getSail().getAerodynamicForce().getVec(),
                                self.solver.getSail().getDefaultCenterOfEffort(), dispName=False)
                
                addVectorVisual(scene, 'dhydro', self.solver.getDrift().getHydrodynamicForce().getVec(),
                                self.solver.getDrift().getDefaultCenterOfEffort(), dispName=False)
                
                addVectorVisual(scene, 'bulb', self.solver.getDrift().getBulbForce().getVec(),
                                self.solver.getDrift().getCdgLest(), dispName=False)
                
                addVectorVisual(scene, 'rhydro', self.solver.getRudder().getRudderHydrodynamicForce().getVec(),
                                self.solver.getRudder().getCdgRudder(), dispName=False)
                
                addVectorVisual(scene, 'prhydro', self.solver.getRudder().getProtectRudderHydrodynamicForce().getVec(),
                                self.solver.getRudder().getCdgProtectRudder(), dispName=False)
                
                addVectorVisual(scene, 'accel', self.solver.getBoatAccel().getAccel()*self.solver.getBoatMass(),
                            self.solver.getHull().getCdg(), color='green', dispName=False)
                
                addMomentVisual(scene, 'maero', self.solver.getSail().getAerodynamicForce().getOriginMoment(),
                                self.solver.getSail().getCdg(), dispName=False)
                
                addMomentVisual(scene, 'mdhydro', self.solver.getDrift().getHydrodynamicForce().getOriginMoment(),
                                self.solver.getDrift().getCdg(), dispName=False)
                
                addMomentVisual(scene, 'mrhydro', self.solver.getRudder().getHydrodynamicForce().getOriginMoment(),
                                self.solver.getRudder().getCdg(), dispName=False)
                
                addMomentVisual(scene, 'mbuoy', self.solver.getHull().getBuoyencyForce().getOriginMoment()+self.solver.getHull().getGravityForce().getOriginMoment(),
                                self.solver.getHull().getCdg(), dispName=False)
                
                
        
        if ('speed' in displayPolicy):
            # display rspeed wind/sail
            vpos = self.solver.getSail().getCdg()
            addSpeedVisual(scene, 'v(wind/sail)', PointSpeed.getVelocity(self.solver, vpos, Referential.SAIL, Referential.WIND), vpos)
            
            # display rspeed rudder/sea
            vpos = self.solver.getRudder().getCdg()
            addSpeedVisual(scene, 'v(rudder/sea)', PointSpeed.getVelocity(self.solver, vpos, Referential.RUDDER, Referential.SEA), vpos)

            # display rspeed boat/sea
            vpos = self.solver.getBoatCdg()
            addSpeedVisual(scene, 'v(boat/sea)', PointSpeed.getVelocity(self.solver, vpos, Referential.BOAT, Referential.SEA), vpos)
            addSpeedVisual(scene, 'boatSpeed', self.solver.getState()._x_speed, self.solver.getBoatCdg())

            fieldspeed = PointSpeed.getVelocity(self.solver, vpos, Referential.BOAT, Referential.SEA)
            solverspeed = self.solver.getState()._x_speed
            

        # put all component in the sea referential
        for comp in scene:
            if ('polygons' in scene[comp]):
                scene[comp]['polygons'] = polysMatRotation(scene[comp]['polygons'], self.solver.getBoatToSeaMatrix())
                scene[comp]['polygons'] = polysTranslation(scene[comp]['polygons'], self.solver.getBoatPos())
            
            for type in ['point']:
                if (type in scene[comp]):
                    scene[comp][type] = pointMatRotation(scene[comp][type], self.solver.getBoatToSeaMatrix())
                    scene[comp][type] = pointTranslation(scene[comp][type], self.solver.getBoatPos())
               
        # Then, DRAW THE ENVIRONEMENT
        if ('env' in displayPolicy):
            # add the grid
            scene['waterGrid']  = {'polygons':self.getWaterGrid(), 'color':COLOR_GRID}
            
            # add water
            scene['water']  = {'polygons':self.getWaterPlane(), 'color':COLOR_WATER}
            # Translate the water with the boat
            scene['water']['polygons'] = polysTranslation(scene['water']['polygons'], self.solver.getBoatPos())
            
            scene.update(self.getBasisScene())
            scene['wind'] = {'polygons':self.getArrowPolygons(self.solver.getWind().getAngle()), 'color':COLOR_WIND}
            # The wind follow the boat
            scene['wind']['polygons'] = polysTranslation(scene['wind']['polygons'], self.solver.getBoatPos())

            scene['xdir'] = {'polygons':self.getArrowPolygons(0, r=0, way=True), 'color':COLOR_WIND}
            scene['xdir']['polygons'] = polysTranslation(scene['xdir']['polygons'], self.solver.getBoatPos())

        # Finally, draw the trajectory
        #if ('traj' in displayPolicy):
        #    scene['traj']  = {'polygons':[self.solver.getBoatTrajectory()+[self.solver.getBoatPos()]], 'color':COLOR_TRAJ, 'width':2}
        
        return scene
    

    def getWaterPlane(self):
        """
        Return the polygons associated with a water plane
        """
        lw = WATER_PLANE_W
        return [[[-lw,-lw,0], 
                 [-lw,lw,0], 
                  [lw,lw,0], 
                  [lw,-lw,0], 
                  [-lw,-lw,0]]]
    
    def getWaterGrid(self):
        """
        Return the polygons associated with a water grid
        """
        # Get the boat position
        x, y, z = self.solver.getBoatPos()
        gw = WATER_GIRD_W     # Grid width
        lw = WATER_PLANE_W    # Grid boundaries
        # Create patern
        out = []
        for k in range(int(2*lw/gw)):
            i = (k+1)*gw - lw
            yl = i + int(y/gw)*gw
            xl = i + int(x/gw)*gw
            out.append([[-lw+x, yl, 0],[lw+x, yl, 0]]) # add line
            out.append([[xl, -lw+y, 0],[xl, lw+y, 0]]) # add column

        return out

    def getBasisScene(self):
        """
        Return a scene with the ground basis
        """
        s = 0.5 # size of the basis
        ori = np.zeros(3) # origin of the basis
        axisWidth = 2
        scene = {'x':{'polygons':[[ori, ori+np.array([s, 0, 0])]], 'color':"red", 'width':axisWidth},
                 'y':{'polygons':[[ori, ori+np.array([0, s, 0])]], 'color':"green", 'width':axisWidth},
                 'z':{'polygons':[[ori, ori+np.array([0, 0, s])]], 'color':"blue", 'width':axisWidth}}
        return scene
    

    def getArrowPolygons(self, ang, r=1.5, way=0):
        """
        Return the polygons of a 3D 2D arrow of angle <ang>,
        At a radius <r> of the origin, in the way <way> (0 or !=0).
        Note: <ang> = 0, then the arrow is n the x axis
        """
        la = 0.8   # length of the arrow
        wa = 0.2 # width of the arrow

        poly = []
        poly.append([0,0,0])
        poly.append([-la/3,-wa/2,0])
        poly.append([-la/3,-wa/2+wa/3,0])
        poly.append([-la,-wa/2+wa/3,0])
        poly.append([-la,wa/2-wa/3,0])
        poly.append([-la/3,wa/2-wa/3,0])
        poly.append([-la/3,wa/2,0])
        poly.append([0,0,0])

        # shift by r and transform to numpy array, and update the way
        for i in range(len(poly)):
            if (way == 0):
                poly[i] = -np.array(poly[i]) + r*np.array([1, 0, 0])
            else:
                poly[i] = np.array(poly[i]) + (r + la)*np.array([1, 0, 0])
                ang = -ang

        # Then apply the rotation
        poly = rotation(poly, np.zeros(3), Dir.Z, ang)

        return [poly]
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.widgets import Slider
import matplotlib
from enum import Enum

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *
from backend.naca.NACACalculator import *


class StabilityPlot(Enum):
    ROLL = "Roll"
    PITCH = "Pitch"
    DISPLACEMENT = "Displacement"
    CDC = "Center of buoyency"

    def fromName(name):
        for e in StabilityPlot:
            if e.value == name:
                return e
        raise Exception('Unknow Stability plot: ' + str(name))

class StabilityEditor(tk.Frame):
    
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app

        mainPane = tk.Frame(self)

        ctrPane = tk.Frame(mainPane)
        self.stabPlot = ttk.Combobox(ctrPane, values=([e.value for e in StabilityPlot]), state="readonly")
        self.stabPlot.set(StabilityPlot.ROLL.value)
        self.stabPlot.pack(side=tk.LEFT)
        ttk.Button(ctrPane, text="Plot", command=self.redraw).pack(side=tk.LEFT)
        ctrPane.pack(side=tk.TOP, fill=tk.X)

        figPane = tk.Frame(mainPane)

        # Create a figure
        fig = plt.figure(figsize=(4, 3.5), dpi=100)
        self.ax = fig.add_subplot(1,1,1)
        self.ax2 = self.ax.twinx()
        
        fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(fig, master=figPane)
        NavigationToolbar2Tk(self.canvas, figPane).update() 
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw()

        figPane.pack(fill=tk.BOTH, expand=True)
        mainPane.pack(fill=tk.BOTH, expand=True)

    def redraw(self, ax=None, ax2=None):
        """
        Redraw the stability curve
        """
        hbc = self.app.getBoat().getSolver().getHull().getHullBuoyencyCalculator()
        rm, rM = hbc.getRollLimit()
        if rm == None:
            # Empty model
            return
        
        if (ax == None):
            ax = self.ax
            try:
                self.ax2.remove()
            except:
                pass
            self.ax2 = self.ax.twinx()
            ax2 = self.ax2
            ax.cla()

        try:
            plot = StabilityPlot.fromName(self.stabPlot.get())
            if (plot == StabilityPlot.DISPLACEMENT):
                self.drawDisplacementCurve(ax, ax2)
            elif (plot == StabilityPlot.ROLL):
                self.ax2.remove()
                self.drawStabilityCurve(ax, rollAxis=True)
            elif (plot == StabilityPlot.PITCH):
                self.ax2.remove()
                self.drawStabilityCurve(ax, rollAxis=False)
            elif (plot == StabilityPlot.CDC):
                self.ax2.remove()
                self.drawCdcCurve(ax)
            else:
                raise Exception ("Unknow stability plot: "+str(plot))
        except Exception as e:
            print(e)

    def drawCdcCurve(self, ax):
        """
        Draw the postion of the cdc as a function of the boat attitude
        """
        hbc = self.app.getBoat().getSolver().getHull().getHullBuoyencyCalculator()
        pitch = 0
        z = 0
        rm, rM = hbc.getRollLimit()
        rollhd = np.linspace(rm, rM, 100)
        data = np.array([hbc.getValueAt(roll, pitch, z) for roll in rollhd])

        ax.plot(np.rad2deg(rollhd), data[:, 1], 'r', label=r'$CdC.x$')
        ax.plot(np.rad2deg(rollhd), data[:, 2], 'b', label=r'$CdC.y$')
        ax.plot(np.rad2deg(rollhd), data[:, 3], 'g', label=r'$CdC.z$')
        ax.legend()

        ax.set_title("Position of the Buoyency Center with the roll")
        ax.set_ylabel('Position of the Buoyency Center [m]')
        ax.set_xlabel('Roll [deg]')

        # Actualize the canvas
        self.canvas.draw()
        

    def drawDisplacementCurve(self, ax, ax2):
        """
        Draw the displacement curve
        """
        hbc = self.app.getBoat().getSolver().getHull().getHullBuoyencyCalculator()
        # We draw the displacement curve for roll=pitch=0
        roll, pitch = 0, 0
        zm, zM = hbc.getZLimit(roll, pitch)
        zhd = np.linspace(zm, zM, 100)
        displacements = np.array([hbc.getValueAt(roll, pitch, z) for z in zhd])
        
        ax.plot(zhd, displacements[:, 0]*1e3, '--', label=r'$\nabla$')
        ax2.plot(zhd, displacements[:, 1], 'r', label=r'$CdC.x$')
        ax2.plot(zhd, displacements[:, 2], 'b', label=r'$CdC.y$')
        ax2.plot(zhd, displacements[:, 3], 'g', label=r'$CdC.z$')

        ax.set_title(r"Displacement curve for $\theta=0, \psi=0$")
        ax.set_ylabel('Displacement [L]')
        ax2.set_ylabel('Position of the Buoyency Center [m]')
        ax2.tick_params(axis='y', labelcolor='g')
        ax.set_xlabel('Level of the water relative to the boat $z$ [m]')
        ax.legend(loc='upper left')
        ax2.legend(loc='center right')

        # Actualize the canvas
        self.canvas.draw()
            

    def drawStabilityCurve(self, ax, rollAxis=True):
        """
        Draw the stability curve
        """

        Nminor = 3
        """
        Find the limit of the model:
            - The roll is ploted within  [rm, rM]
            - The pitch is ploted within [pm, pM]
        """
        hbc = self.app.getBoat().getSolver().getHull().getHullBuoyencyCalculator()
        rm, rM = hbc.getRollLimit()
        pm, pM = hbc.getPitchLimit()

        if (rollAxis == True):
            # If we studdy the roll axis,
            # We vary roll precisely and pitch roughly
            majorAxis = np.linspace(rm, rM, 40) # roll
            minorAxis = np.linspace(pm, pM, Nminor)  # pitch
            angChr = r'\psi' # The minor symbol is pitch, thus psi
        else:
            majorAxis = np.linspace(pm, pM, 40)
            minorAxis = np.linspace(rm, rM, Nminor)
            angChr = r'\theta'
        
        # Get cdg, cdc and their force
        cdg = self.app.getBoat().getSolver().getBoatCdg().valueIn(Referential.BOAT)
        mass = self.app.getBoat().getSolver().getBoatMass()
        loop = 0

        def computeRotation(arr, roll, pitch):
            """
            Compute the position of arr (belongin to the boat basis)
            into the sea basis
            """
            z1 = np.cos(roll)*arr[2] + np.sin(roll)*arr[1]
            y1 = np.cos(roll)*arr[1] - np.sin(roll)*arr[2]
            z2 = np.cos(pitch)*z1 + np.sin(pitch)*arr[0]
            x2 = np.cos(pitch)*arr[0] - np.sin(pitch)*z1
            return np.array([x2, y1, z2])
        
        def findEqZ(roll, pitch):
            """
            Return the equilibrium altitude for a given roll, pitch
            """
            zm, zM = hbc.getZLimit(roll, pitch)
            for z in np.linspace(zm, zM, 100):
                val = hbc.getValueAt(roll, pitch, z)
                if (val[0]*PHY_RHO_SWATER > mass):
                    # Equilibrium
                    return z
            return zm

        for minorAng in list(minorAxis) + [0]:
            Mgravs = []
            Mtots = []
            Mbuoys = []
            for majorAng in majorAxis:
                if (rollAxis == True):
                    # Major ang is roll
                    roll = majorAng
                    pitch = minorAng
                else:
                    # Minor ang is pitch
                    roll = minorAng
                    pitch = majorAng

                # Start by finding the z coordinate at equilibrium
                zeq = findEqZ(roll, -pitch)
                # Load the cdc at this position
                val = hbc.getValueAt(roll, -pitch, zeq)
                vol = val[0]
                cdc = val[1:]
                #print("Eq volume is:",vol*1e3,"vs mass:",mass)
                
                """
                Get the x, y position of the cdc and cdg at: 
                roll=roll, pitch=pitch
                """

                # Compute the postion of the center of gravity after rotation
                cdgRotated = computeRotation(cdg, -roll, pitch)
                # Compute the postion of the center of buoyency after rotation
                cdcRotated = computeRotation(cdc, -roll, pitch)

                # Compute the moment
                if rollAxis:
                    # We are interested about the moment arround the x axis
                    # i.e the y coordinate
                    Mgrav = -cdgRotated[1]*PHY_G_NORM*mass
                    Mbuoy = cdcRotated[1]*vol*PHY_RHO_SWATER*PHY_G_NORM
                else:
                    # We are interested about the moment arround the y axis
                    # i.e the x coordinate with -1 ! Indirect order
                    Mgrav = cdgRotated[0]*PHY_G_NORM*mass
                    Mbuoy = -cdcRotated[0]*vol*PHY_RHO_SWATER*PHY_G_NORM

                Mgravs.append(Mgrav)
                Mtots.append(Mgrav + Mbuoy)
                Mbuoys.append(Mbuoy)

            print("Minor VALUE=",minorAng)
            if (loop == len(minorAxis)):
                # plot the buoy moment for minorAng = 0
                ax.plot(np.rad2deg(majorAxis), Mbuoys, 'g-.', label='$M_{buoy}, '+angChr+'='+'{:.1f}deg$'.format(np.rad2deg(minorAng)))
                # plot the gravity moment for minorAng = 0
                ax.plot(np.rad2deg(majorAxis), Mgravs, 'r-.', label='$M_{grav}, '+angChr+'='+'{:.1f}deg$'.format(np.rad2deg(minorAng)))
            else:
                # plot
                ax.plot(np.rad2deg(majorAxis), Mtots, getPlotColor(loop), label='$M_{tot}, '+angChr+'='+'{:.1f}deg$'.format(np.rad2deg(minorAng)))
            
            loop += 1

        # Test for pitch = 2deg
        pitch0 = np.deg2rad(2.657)
        roll0 = 0
        zeq = findEqZ(roll0, -pitch0)
        val = hbc.getValueAt(roll0, -pitch0, zeq)
        cdc = val[1:]
        cdgRotated = computeRotation(cdg, -roll0, pitch0)
        cdcRotated = computeRotation(cdc, -roll0, pitch0)
        print("Boat mass:",mass)
        print("scdg in boat:",cdg[0])
        print("For Test pitch:\nxcdc=",cdcRotated[0],"\nxCdg=",cdgRotated[0],'\nz=',zeq)

        # plot the gravity moment


        # Then get the maximum moment of the sail
        nc = self.app.getBoat().getSolver().getNACACalculator()
        sailSurface = self.app.getBoat().getSolver().getSail().getSurface()
        sailCof = self.app.getBoat().getSolver().getSail().getDefaultCenterOfEffort().valueIn(Referential.BOAT)
        sailProfile = self.app.getBoat().getSolver().getSail().getGeomP('profile')
        fspeed = noeud2ms(20)
        Mmax = 0
        MmaxI = 0
        print("Sail surface=",sailSurface)
        for i in np.linspace(0, np.pi):
            relforce = nc.interpolateNACA(i, sailProfile)
            norm = np.sqrt(relforce[0]**2 + relforce[1]**2)
            norm = norm*sailSurface*PHY_RHO_AIR*fspeed**2
            # We take the worst case scenario where the reaction force of the sail is perpendicular to the boat
            # i.e, in the direction y
            if (rollAxis == True):
                # Roll axis considered: moment around x is the interest
                # thus, (moment around x) M = y*Fz - z*Fy with Fz=0
                M = -sailCof[2]*norm
            else:
                # pitch  (moment around y), M = z*Fx - y*Fz with Fz=0
                M = sailCof[2]*norm
            if (abs(M) > Mmax):
                Mmax = abs(M)
                MmaxI = i

        ax.plot([np.rad2deg(min(majorAxis)), np.rad2deg(max(majorAxis))], [Mmax, Mmax], 'k-', label='$M_{sail,max}, i='+'{:.1f}deg, w={:.1f}knt$'.format(np.rad2deg(MmaxI), ms2noeud(fspeed)))


        # Then add the maximum moment of the drift (drift angle < 25)
        driftSurface = self.app.getBoat().getSolver().getDrift().getSurface()
        driftCof = self.app.getBoat().getSolver().getDrift().getDefaultCenterOfEffort().valueIn(Referential.BOAT)
        driftProfile = self.app.getBoat().getSolver().getDrift().getGeomP('profile')
        fspeed = noeud2ms(10)
        Mmax = 0
        MmaxI = 0
        
        for drift in np.linspace(0, np.deg2rad(20)):
            relforce = nc.interpolateNACA(drift, driftProfile)
            # Only take in consideration the force in the y direction
            norm = np.abs(relforce[1])
            norm = norm*driftSurface*PHY_RHO_SWATER*fspeed**2
            # We take the worst case scenario where the reaction force of the sail is perpendicular to the boat
            M = norm*np.abs(driftCof[2])
            if (M > Mmax):
                Mmax = M
                MmaxI = drift

        ax.plot([np.rad2deg(min(majorAxis)), np.rad2deg(max(majorAxis))], [Mmax, Mmax], 'k-.', label='$M_{drift,max}, i='+'{:.1f}deg, w={:.1f}knt$'.format(np.rad2deg(MmaxI), ms2noeud(fspeed)))



        # Draw info
        ax.set_title('Stability Curve')
        ax.set_ylabel('Righting Moment [Nm]')
        if rollAxis:
            ax.set_xlabel('Roll [deg]')
        else:
            ax.set_xlabel('Pitch [deg]')
        ax.legend()
        ax.grid()

        # Actualize the canvas
        self.canvas.draw()
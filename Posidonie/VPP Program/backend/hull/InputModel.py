import numpy as np
import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from config.Config import *

"""
This file is used to recap all the proprety of the incoming 3D model of the hull
"""

"""
First we need to orient corectly the boat.
In the correct referential, the axis are the following:
    - Hull longitunaly among the X axis, toward positives values
    - Z axis toward the up, from the bottom of the boat to the top
"""

# Define the orientation of the orginal model
# in format "[+-][XYZ]"
longiDirection     = "-Y"
topBottomDirection = "+Z"

# Define the size of the original model
modelLength        = 2

# Define the name of the original model
modelName          = PATH_3DMODEL_FOLDER + "RowingBoat.stl"


# Then, position of the center of gravity in the model's referential
cdg = np.array([0, 100, -30])

# Inertia matrix of the boat
inertiaMatrix = np.array([[10,0,0],
                         [0,10,0],
                         [0,0,10]])

# Mass of the hull
mass = 10

# position of the rotation axis in the model's referential
centerOfRotation = cdg.copy()
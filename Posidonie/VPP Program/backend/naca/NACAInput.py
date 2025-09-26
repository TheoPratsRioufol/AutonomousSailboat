
import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

# SPECIFY THE SIMULATION PARAMETERS

# profiles available
profileTypesSimuled = ['NACA0015']

# Model name
modelPrefix = "wing"

# Simuled fluid speed [m/s]
speedSimu = 5

# Section length [m]
slength = 0.4

# Distance from the front of the profil to the center of rotation
sfr = 0.135112

# Fluid simuled
fluidSimuled = 'water'
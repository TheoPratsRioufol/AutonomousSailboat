import numpy as np


FOLDER_SEP_CHR = '\\'

APP_WIDTH  = 600
APP_HEIGHT = 800

APP_LOGO = 'sailboat'
MAIN_WINDOW_TITLE = "Posidonie Physics"

VIEWVER_WIDTH  = 800
VIEWVER_HEIGHT = 800

SPLASH_WIN_W = 300
SPLASH_WIN_H = int(SPLASH_WIN_W*396/800)

currentPath = __file__
currentFolder = currentPath[:currentPath.rfind(FOLDER_SEP_CHR)]
projectFolder = currentFolder[:currentFolder.rfind(FOLDER_SEP_CHR)+1]
currentFolder += FOLDER_SEP_CHR

ZOOM_SPEED = 0.3
ZOOM_ROT_SPEED = 5

ICON_SIZE = 18
PATH_RES_ICON = projectFolder + "utils" + FOLDER_SEP_CHR + "res" + FOLDER_SEP_CHR
PATH_3DMODEL_FOLDER     = projectFolder + "backend" + FOLDER_SEP_CHR + "hull" + FOLDER_SEP_CHR
PATH_BUOYENCY_MODEL_DIR = PATH_3DMODEL_FOLDER
PATH_BUOYENCY_MODEL     = PATH_BUOYENCY_MODEL_DIR + "buoyencyModel.buoyModel"
PATH_NACA_SIMU_FOLDER   = projectFolder + "backend" + FOLDER_SEP_CHR + "naca" + FOLDER_SEP_CHR
DEFAULT_SAVE_PATH       = projectFolder + "config" + FOLDER_SEP_CHR + "save.json"
BUOYENCY_MOD_FILETYPE   = "buoyModel"
PATH_REPORT             = projectFolder + "reports" + FOLDER_SEP_CHR + "report"
PATH_REPORT_IMGFOLDER   = projectFolder + "reports" + FOLDER_SEP_CHR + "img" + FOLDER_SEP_CHR
PATH_GHOSTSCRIPT        = r'C:\Program Files\gs\gs10.05.1\bin\gswin64c'
PATH_EXPORT_STL         = projectFolder + "export" + FOLDER_SEP_CHR + "goodOrientation.stl"
PATH_CONFIG_FILE        = projectFolder + "config" + FOLDER_SEP_CHR + "dynamicConfig.json"


COLOR_BG_BOAT_VIEWVER    = "#FFFBE6"

LIGTH = False

if LIGTH:
    COLOR_BG_VIEWVER_TOP     = "#FEFEFE"
    COLOR_BG_VIEWVER_BOTTOM  = "#E2E5ED"
else:
    COLOR_BG_VIEWVER_BOTTOM  = "#a4c2f7"
    COLOR_BG_VIEWVER_TOP     = "#FEFEFE"


BASE_3DRENDERER_SIZE = 30

PINK_DARK = "#AD217E"

COLOR_BOX_BORDER_FOCUSED = "#425982"
COLOR_BOX_BORDER = "#a4c2f7"
COLOR_BOX        = "#e8edfc"
COLOR_BOX_DARK   = "#428dff"

COLOR_HULL  = PINK_DARK #"#AB274F"
COLOR_DRIFT = "#FFBF00"
COLOR_SAIL  = "#0000B3" # "#7CB9E8"
COLOR_RUDDER= "purple"
COLOR_WATER = "#0048BA"
COLOR_GRID  = "#C0C0C0"
COLOR_WIND  = "#AB274F"
COLOR_CDC   = "#00308F" 
COLOR_CDG   = "#00308F" 
COLOR_TRAJ  = "red"
COLOR_INTEG = "magenta"
COLOR_INTEG_OUTLINE = "cyan"

COLOR_MOMENT = "#FF19B3"
COLOR_VECTOR = 'blue'
COLOR_SPEED  = COLOR_BOX_DARK

COLOR_PLAYBACK = "#808080"

LIGHT_TEXT_COLOR = "#6a9ef8"
SELECTING_COLOR = "#cad9fc"

RENDERER_POINT_HSIZE = 2

ELM_DISPLAYABLE = ['boat', 'env', 'speed', 'force', 'sinfo', 'geom', 'integral']
REPORT_DISPLAY_POLICY = {'boat':True,
                         'env':False,
                         'speed':False,
                         'force':False,
                         'sinfo':False,
                         'integral':False,
                         'geom':True}

DEFAULT_DISPLAY_POLICIY = {'boat':True,
                         'env':True,
                         'speed':False,
                         'force':True,
                         'sinfo':True,
                         'integral':False,
                         'geom':True}

DEFAULT_DYNAMIC_CONFIG = {'lastFile':""}

PAD_MAX_ZOOM = 0.1 # percentage

WATER_GIRD_W  = 0.8
WATER_PLANE_W = 6*WATER_GIRD_W

PHY_G_NORM = 9.81
PHY_G_VECTOR = np.array([0, 0, -PHY_G_NORM]) # gravity vector

"""Materials"""
PHY_RHO_ALU = 2700.0 # Masse volumique matériaux ALU
PHY_RHO_SWATER = 1024.0 # Masse volumique eau salé
PHY_RHO_WATER  = 1000.0 # Masse volumique eau
PHY_RHO_LEAD = 11350.0 # Masse volumique matériaux PLOMB
PHY_RHO_AIR = 1.2 # Masse volumique matériaux AIR
PHY_RHO_CTP = 510 # Masse volumique cpt 5mm, 10mm, 15mm [kg/m3]
PHY_RHO_EXOTICWOOD = 950 # Masse volumique bois exotique [kg/m3]
PHY_RHO_FIBERGLASS = 2540 # Masse volumique fibre de verre + résine [kg/m3]
PHY_RHO_PUR = 40 # Masse volumique Polyuréthane [kg/m3] (mousse isolante dure)
PHY_RHOL_MAST = 2.2/4 # Masse linéique mat carbone [kg/m]

PHY_RHO_DRIFT = (5*PHY_RHO_FIBERGLASS + 20*PHY_RHO_EXOTICWOOD)/25  # Masse volumique dérive
PHY_RHO_KEEL  = PHY_RHO_LEAD # Masse volumique matériaux lest de quille

PHY_RHO_RUDDER         = (4*PHY_RHO_FIBERGLASS + 15*PHY_RHO_EXOTICWOOD)/29 # Masse volumique matériaux rudder (10mm de bois)
PHY_RHO_PROTECT_RUDDER = PHY_RHO_RUDDER # Masse volumique matériaux protège safran

# Voile : 2*coque de 2mm de résine + mousse de 8cm au centre + mat plance à voile (PHY_RHOL_MAST)
PHY_RHOS_SAIL = 0.082*(80*PHY_RHO_PUR + 4*PHY_RHO_FIBERGLASS)/84 # Masse surfacique de la voile

DEFAULT_DT_SIM = 0.05


ROLL_AXIS  = 0 # rotation d'axe x
PITCH_AXIS = 1 # rotation d'axe y

WATER_SIZE = 5 #size of the water plane when computing the buoyency

BUOYENCY_MOD_MAX_ROLL  = np.deg2rad(180)  # Maximum roll considered for the boat
BUOYENCY_MOD_MAX_PITCH = np.deg2rad(60)   # Maximum pitch considered for the boat
BUOYENCY_MOD_MIN_PITCH = np.deg2rad(-60)  # Minimal pitch considered for the boat

BUOYENCY_MOD_N_ROLL  = 30  # Number of point for the roll model of the boat
BUOYENCY_MOD_N_PITCH = 16  # Number of point for the pitch model of the boat
BUOYENCY_MOD_N_Z     = 10  # Number of point for the z model of the boat


TITLE_FONT = ("consolas", 10, "bold")
INFO_FONT  = ("consolas", 12, "bold")
COMPASS_FONT = "consolas 10"
COMPASS_FONT_BIG = "consolas 14"
BOX_BORDER_COLOR = "#7CB9E8"

CRITICAL_ROLL = np.deg2rad(35)  # Critical roll for dynamic stability analysis
CRITICAL_PITCH = np.deg2rad(10) # Critical pitch for dynamic stability analysis


N_HEADINGS_MX = 4
HEADINGS_MX = np.linspace(150, 40, N_HEADINGS_MX)
COMPLETE_MX_CHARACTERISTIC_FORMS = {}

for i in range(len(HEADINGS_MX)):
    COMPLETE_MX_CHARACTERISTIC_FORMS[i] = {
                                            'wmin':8,
                                            'wmax':45,
                                            'tslope':90,
                                            'anglecv':3,
                                            'cvTime':3,
                                            'heading':HEADINGS_MX[i],
                                            'N':30,
                                        }
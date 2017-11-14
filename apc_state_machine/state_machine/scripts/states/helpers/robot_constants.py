"""
Constants for the robot and its movement
"""

# Sucker
SUCKER_MAX_GLOBAL_Z = 1.162
SUCKER_MIN_GLOBAL_Z_WITH_GRIPPER = 0.62
SUCKER_LIFT_HEIGHT = 0.65
SUCKER_MIN_GLOBAL_Y = 0.105
SUCKER_REALSENSE_Y_OFFSET = 0.185
SUCKER_MAX_Y_STRAIGHT = 1.034
SUCKER_MIN_Y_STRAIGHT = 0.115
SUCKER_MAX_GLOBAL_X = 0.975
SUCKER_MIN_GLOBAL_X = 0.005

# Gripper
GRIPPER_MAX_GLOBAL_Z = 1.203
GRIPPER_MIN_GLOBAL_Z = 0.81
GRIPPER_LIFT_HEIGHT_45 = 0.65
GRIPPER_LIFT_HEIGHT_STRAIGHT = 0.79

#Realsense
TOOL_CHANGE_REALSENSE_GLOBAL_Z = 0.55
SAFE_REALSENSE_GLOBAL_Z = 0.55
TOOL_CHANGE_SAFE_Y = 0.7  # Y at which you won't hit the top frame.
TOOL_CHANGE_DANGEROUS_Y = 0.59  # Y at witch it's possible to hit the frame.
TOOL_CHANGE_X_MAX = 0.77  # X_MAX when frame is safe when dangerous y.
TOOL_CHANGE_X_MIN = 0.15  # X_MIN when frame is safe when dangerous y.
REALSENSE_MAX_GLOBAL_X = 0.975
REALSENSE_MIN_GLOBAL_X = 0.005
REALSENSE_MIN_GLOBAL_Y = -0.065

# Global positions for the sucker_endpoint to fit inside the tote.
SUCKER_TOTE_BOUNDS = {
    'x_min': 0.275,
    'x_max': 0.695,
    'y_min': 0.760,
    'y_max': 1.015,
    'z_max': 1.160
}
GRIPPER_TOTE_BOUNDS = {
    'x_min': 0.295,
    'x_max': 0.680,
    'y_min': 0.785,
    'y_max': 0.995,
    'z_max': 1.195
}
TOTE_CENTRE = ((SUCKER_TOTE_BOUNDS['x_min'] + SUCKER_TOTE_BOUNDS['x_max'])/2,
               (SUCKER_TOTE_BOUNDS['y_min'] + SUCKER_TOTE_BOUNDS['y_max'])/2)

SUCKER_STORAGE_A_BOUNDS = {
    'x_min': 0.625,
    'x_max': 0.940,
    'y_min': 0.125,
    'y_max': 0.625,
    'z_max': 1.160
}
GRIPPER_STORAGE_A_BOUNDS = {
    'x_min': 0.645,
    'x_max': 0.910,
    'y_min': 0.135,
    'y_max': 0.610,
    'z_max': 1.203
}
STORAGE_A_CENTRE = ((SUCKER_STORAGE_A_BOUNDS['x_min'] + SUCKER_STORAGE_A_BOUNDS['x_max'])/2,
               (SUCKER_STORAGE_A_BOUNDS['y_min'] + SUCKER_STORAGE_A_BOUNDS['y_max'])/2)

SUCKER_STORAGE_B_BOUNDS = {
    'x_min': 0.210,
    'x_max': 0.505,
    'y_min': 0.125,
    'y_max': 0.625,
    'z_max': 1.160
}
GRIPPER_STORAGE_B_BOUNDS = {
    'x_min': 0.225,
    'x_max': 0.480,
    'y_min': 0.130,
    'y_max': 0.610,
    'z_max': 1.203
}
STORAGE_B_CENTRE = ((SUCKER_STORAGE_B_BOUNDS['x_min'] + SUCKER_STORAGE_B_BOUNDS['x_max'])/2,
               (SUCKER_STORAGE_B_BOUNDS['y_min'] + SUCKER_STORAGE_B_BOUNDS['y_max'])/2)

POSITION_LIMITS = {
    'stow_tote': {
        'sucker': SUCKER_TOTE_BOUNDS,
        'gripper': GRIPPER_TOTE_BOUNDS,
        'centre': TOTE_CENTRE
    },
    'storage_A': {
        'sucker': SUCKER_STORAGE_A_BOUNDS,
        'gripper': GRIPPER_STORAGE_A_BOUNDS,
        'centre': STORAGE_A_CENTRE
    },
    'storage_B': {
        'sucker': SUCKER_STORAGE_B_BOUNDS,
        'gripper': GRIPPER_STORAGE_B_BOUNDS,
        'centre': STORAGE_B_CENTRE
    },
}

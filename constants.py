import numpy as np
from enum import Enum

BALANCE = 0
TRANSITION = 1

# all equilibrium states of double pendulum
STATE_DICT = {
        "00" : (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "01" : (0.0, 0.0, np.pi, 0.0, 0.0, 0.0),
        "10" : (0.0, np.pi, np.pi, 0.0, 0.0, 0.0),
        "11" : (0.0, np.pi, 0.0, 0.0, 0.0, 0.0),
}

# Enum specifying the indices in TRANSCRIPTION_PARAMS
class TranscriptionParams(Enum):
    TARGET_TIME = 0
    NUM_TIME_STEPS = 1
    INPUT_COST = 2
    SHOULDER_VELOCITY_COST = 3
    ELBOW_VELOCITY_COST = 4
    ACTUATION_CONSTRAINT = 5
    TIME_CUTOFF = 6
    CONSTRAINT_IDXS = 7
    FINISH_POSE = 8
    KEY_FRAME_FILE = 9

# params used in direct transcription trajectory optimisation
# (each pose in tuple explained in enum above ^)
TRANSCRIPTION_PARAMS = {
        "00_01" : (1.0, 350, 0.5, 5.5, 0.5, 7.5, 0.8, [1, 2, 3, 4, 5], 0.1, None),
        "00_10" : (4.0, 300, 1.0, 1.5, 1.5, 50.0, 1.95, None, None, None),
        "00_11" : (4.0, 250, 1.0, 0.5, 0.5, 5.0, 1.8, None, None, None),
        "01_00" : (1.0, 300, 1.5, 1.5, 1.5, 7.5, None, [1, 2, 3, 4, 5], 0.2, None),
        "01_10" : (2.0, 200, 1.0, 1.5, 1.5, 10.0, 1.3, [1, 2, 4, 5], None, None),
        "01_11" : (0.8, 300, 0.5, 0.5, 5.5, 50.0, 0.797, [1, 2, 3, 4], 0.2, None),
        "10_00" : (1.0, 300, 1.0, 1.5, 1.5, 10.0, 0.99, [1, 2, 3, 4, 5], 0.2, None),
        "10_01" : (1.0, 200, 1.0, 1.5, 1.5, 10.0, 0.85, [0, 1, 2, 4, 5], None, None),
        "10_11" : (1.0, 300, 1.0, 1.5, 1.5, 10.0, None, [1, 2, 3, 4, 5], 0.2, None), # This one working, but ugly
        "11_00" : (1.5, 200, 1.0, 1.5, 1.5, 10.0, 1.49, [1, 2, 3, 4, 5], 0.2, None),
        "11_01" : (0.8, 300, 0.5, 1.5, 5.5, 50.0, None, [1, 2, 3, 4, 5], 0.2, None),
        "11_10" : (0.8, 300, 0.5, 0.5, 5.5, 50.0, 0.797, [1, 2, 3, 4], 0.2, None),
}

LQR_FILE = "./gains/lqrs.pkl"
TVLQR_FILE = "./gains/tvlqrs.pkl"

import numpy as np
from enum import Enum

DOWN_BALANCE = 0
SWING_UP = 1
UP_BALANCE = 2
SWING_DOWN = 3

STATE_DICT = {
        "00" : (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "01" : (0.0, 0.0, np.pi, 0.0, 0.0, 0.0),
        "10" : (0.0, np.pi, np.pi, 0.0, 0.0, 0.0),
        "11" : (0.0, np.pi, 0.0, 0.0, 0.0, 0.0),
}

class TranscriptionParams(Enum):
    TARGET_TIME = 0
    NUM_TIME_STEPS = 1
    INPUT_COST = 2
    SHOULDER_VELOCITY_COST = 3
    ELBOW_VELOCITY_COST = 4
    ACTUATION_CONSTRAINT = 5
    TIME_CUTOFF = 6
    CONSTRAINT_IDXS = 7
    KEY_FRAME_FILE = 8

TRANSCRIPTION_PARAMS = {
        "00_01" : (),
        "00_10" : (4.0, 300, 1.0, 1.5, 1.5, 50.0, 1.95, None, None),
        "00_11" : (4.0, 250, 1.0, 0.5, 0.5, 5.0, 1.8, None, None),
        "01_00" : (1.0, 200, 1.0, 1.5, 1.5, 10.0, 0.99, None),
        "01_10" : (),
        "01_11" : (),
        "10_00" : (),       # should be free
        "10_01" : (),
        "10_11" : (),
        "11_00" : (),       # should be free
        "11_01" : (),
        "11_10" : (),
}


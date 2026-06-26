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
    KEY_FRAME_FILE = 7

TRANSCRIPTION_PARAMS = {
        "00_11" : (4.0, 250, 1.0, 0.5, 0.5, 5.0, 1.8, None),
        "00_10" : (3.5, 500, 1.0, 0.5, 0.5, 2.5, 2.65, "./keyframes/00_10_2.npz"),
}


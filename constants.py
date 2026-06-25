import numpy as np

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



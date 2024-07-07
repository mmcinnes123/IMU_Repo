

import opensim as osim
import numpy as np
from os.path import join
import matplotlib.pyplot as plt

bool_arr = [False, True, False, False, False,
 False, False, False, False, False, False]

print(np.where(bool_arr)[0])
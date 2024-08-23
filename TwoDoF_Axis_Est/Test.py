
from os.path import join
import pandas as pd
import os
import numpy as np

from helpers_2DoF import plot_FE_estimates

# Read all_data and change the column headers


all_FE_axes_est = np.array([
    [ 0.57200827,  0.51329661, -0.63979147],
    [ 0.57200827,  0.51329661, -0.63979147],
    [-0.40833391, -0.59275877,  0.69419051],
    [ 0.40833391,  0.59275877, -0.69419051],
    [ 0.40833391,  0.59275877, -0.69419051],
    [-0.57200827, -0.51329661,  0.63979147],
    [-0.40833391, -0.59275877,  0.69419051],
    [ 0.57200827,  0.51329661, -0.63979147],
    [ 0.06661604, -0.45267729,  0.88918253]
])

FE_axis_in_humerus = np.array([0 , 1, 0])
plot_FE_estimates(all_FE_axes_est, FE_axis_in_humerus)
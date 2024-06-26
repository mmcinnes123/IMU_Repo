import numpy as np
import itertools

def axisToThetaPhi(j, var):
    if var == 1:
        theta = np.arccos(j[2])
        phi = np.arctan2(j[1], j[0])
    elif var == 2:
        theta = np.arccos(j[0])
        phi = np.arctan2(j[1], j[2])
    else:
        raise ValueError('invalid axis var')
    return theta, phi

x = slice(0, 5)

y = [1, 2, 3, 4, 5, 6]

print(y[x])
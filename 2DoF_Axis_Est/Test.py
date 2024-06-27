import numpy as np
import itertools

def inner1d(a, b):  # avoid deprecation, cf. https://stackoverflow.com/a/15622926
    return np.einsum('ij,ij->i', np.atleast_2d(a), np.atleast_2d(b))

a = inner1d(np.array([0, 0, 1]).T, [0.09554825, 0.00132716, 0.99542392])

print(a)
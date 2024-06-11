from abc import ABC, abstractmethod

import numpy as np


class AbstractObjectiveFunction(ABC):
    """
    Abstract base class that implements an objective function (e.g. calculates error and jacobi matrix) and handles
    tasks related to the estimated parameter vector (switch representation, provide initial values, ...)

    N samples, P parameters
    """
    def __init__(self, init):
        self._x = np.array(init, float)  # parameter vector
        self._cache = {}  # cache to avoid duplicate calculations
        self._data = None
        # Specifies indices of the parameter vector that should be updated by the optimizer.
        # This can be used to store parameters that are not part of the regular optimization (e.g. which axis
        # representation is currently used) or to only optimize specific parameters.
        self.updateIndices = slice(None, None, None)

    def clearCache(self):
        self._cache = {}

    def setData(self, data):
        self._data = data
        self.clearCache()

    def getX(self):
        return self._x.copy()

    @abstractmethod
    def unpackX(self):
        pass

    def setX(self, x):
        assert x.shape == self._x.shape
        self._x = x.copy()
        self.clearCache()

    def addToX(self, deltaX):
        self._x[self.updateIndices] = self._x[self.updateIndices] + deltaX
        self.clearCache()

    def checkAndSwitchParametrization(self):
        pass

    @abstractmethod
    def err(self):
        pass

    def jac(self):
        return self.errAndJac()[1]

    def errAndJac(self):
        return self.err(), None

    def errAndApproxJac(self):
        """Calculate error (N) and finite difference approximation of Jacobi matrix (NxP)."""
        if 'approxJac' in self._cache and 'err' in self._cache:
            return self._cache['err'], self._cache['approxJac']

        eps = np.sqrt(np.finfo(float).eps)
        err = self.err()
        x_orig = self._x.copy()
        jac = np.zeros((len(err), len(x_orig)))
        for i in np.arange(len(x_orig))[self.updateIndices]:
            x_copy = x_orig.copy()
            x_copy[i] += eps
            self.setX(x_copy)
            jac[:, i] = (self.err() - err)/eps
        self.setX(x_orig)
        self._cache['approxJac'] = jac
        return err, jac

    def approxJac(self):
        """Calculate finite difference approximation of Jacobi matrix (NxP)."""
        return self.errAndApproxJac()[1]

    def costAndGradient(self):
        if 'grad' in self._cache and 'cost' in self._cache:
            return self._cache['cost'], self._cache['grad']

        err, jac = self.errAndJac()
        cost = np.mean(err**2)
        grad = 2*np.dot(err, jac)/len(err)
        self._cache['cost'] = cost
        self._cache['grad'] = grad
        return cost, grad

    def cost(self):
        if 'cost' in self._cache:
            return self._cache['cost']
        cost = np.mean(self.err()**2)
        self._cache['cost'] = cost
        return cost

    def costGradient(self):
        return self.costAndGradient()[1]

    def checkJac(self):
        """Returns the maximum absolute and relative difference between the analytic Jacobi matrix and a
        numeric approximation."""
        jac1 = self.jac()[:, self.updateIndices]
        jac2 = self.approxJac()[:, self.updateIndices]
        # print(np.max(np.abs(jac1 - jac2), axis=0))
        absDiff = np.max(np.abs(jac1 - jac2))
        relDiff = np.max(np.abs(jac1 - jac2)/np.maximum(np.finfo(float).eps, np.maximum(np.abs(jac1), np.abs(jac2))))
        return absDiff, relDiff

    @staticmethod
    @abstractmethod
    def getInitVals(variant='default', seed=None):
        pass

    @abstractmethod
    def calculateParameterDistance(self, other, strict=False):
        """Calculates some distance measure between the parameter vector of this object and the parameter vector
        of another objective function object which can be used to determine if the solutions are similar."""
        pass


class AbstractSolver(ABC):
    def __init__(self, objFn):
        self.objFn = objFn
        self.switch = True
        self.stepCounter = 0

    def setData(self, data):
        self.objFn.setData(data)

    def getState(self):
        return np.array([self.stepCounter], np.float)

    @abstractmethod
    def step(self):
        pass

    def steps(self, N, fullOutput=False):
        if fullOutput:
            costSteps = np.zeros((N+1,))
            xSteps = np.zeros((N + 1, len(self.objFn.getX())))
            costSteps[0] = self.objFn.cost()
            xSteps[0] = self.objFn.getX()
            for i in range(1, N+1):
                self.step()
                costSteps[i] = self.objFn.cost()
                xSteps[i] = self.objFn.getX()
            return costSteps, xSteps
        else:
            for _ in range(N):
                self.step()


class GNSolver(AbstractSolver):
    """ Gauss-Newton solver """
    def __init__(self, objFn, stepSize=1):
        super().__init__(objFn)
        self.stepSize = stepSize

    def step(self):
        if self.switch:
            self.objFn.checkAndSwitchParametrization()
        # print(self.objFn.checkJac())
        err, jac = self.objFn.errAndJac()
        jac = jac[:, self.objFn.updateIndices]
        # try:
        deltaX = -self.stepSize*np.linalg.solve(np.dot(jac.T, jac), np.dot(jac.T, err))
        # except np.linalg.LinAlgError as e:
        #     print('LinAlgError, using leastsq:', e)
        #     deltaX = -self.stepSize*np.linalg.lstsq(np.dot(jac.T, jac), np.dot(jac.T, err), rcond=None)[0]
        self.objFn.addToX(deltaX)
        self.stepCounter += 1
        return deltaX

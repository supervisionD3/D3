import numpy as np
np.set_printoptions(suppress=True)

from scipy import stats

class DeviationDetector:
    def __init__(self):

        # nominal model
        self.A = 0.0
        self.B = 3.0959
        self.C = 1.0

        # parameter deviation
        self.F = 1.0
        self.omega = 0.01
        self.Q = 0.02955

        # uncertainty compensation
        self.gamma = 0.17184
        self.W = 0.50961
        self.V = 2.0e-16

        # safety region quantification
        self.beta = self.C*(1.0-self.A)*self.B
        self.pole = 0.6
        self.region = [0.0, 1.0/(1-self.pole)*self.beta]
        self.probThreshold = 0.9973  # 3sigma

        # state mean and variance
        self.deltaStateMean = 0.0
        self.deltaStateVar = 0.5

        # B mean and variance
        self.B_k = self.B
        self.P = 0.5

        # results
        self.cdf = 0.0
        self.alarm = 0

        # horizon list
        self.list_u = []
        self.list_a = []
        self.list_y = []

    def detector(self, u, a, y):

        if len(self.list_u)<3:
            self.list_u.append(u)
            self.list_a.append(a)
            self.list_y.append(y)
        else:
            self.list_u.append(u)
            self.list_a.append(a)
            self.list_y.append(y)

            delta_y_2 = self.list_y[1] - self.list_y[0]
            delta_u_2 = self.list_u[1] - self.list_u[0]
            delta_a_1 = self.list_a[2] - self.list_a[1]
            self.stateEstimation(delta_y_2, delta_u_2, delta_a_1)

            delta_u_1 = self.list_u[2] - self.list_u[1]
            if abs(delta_u_1)>0:
                delta_a = self.list_a[3] - self.list_a[2]
                delta_y = self.list_y[3] - self.list_y[2]
                self.parameterEstimation(delta_u_1, delta_a, delta_y)
                self.probDetection()

            # update old data
            self.list_u.remove(self.list_u[0])
            self.list_a.remove(self.list_a[0])
            self.list_y.remove(self.list_y[0])

        return self.alarm

    def stateEstimation(self, delta_y_2, delta_u_2, delta_a_1):

        # update
        R_k = self.C*self.deltaStateVar*self.C + self.V
        K = self.deltaStateVar*self.C*(1.0/R_k)
        error = delta_y_2 - self.C*self.deltaStateMean

        self.deltaStateMean = self.deltaStateMean + K*error
        self.deltaStateVar = (1.0 - K*self.C)*self.deltaStateVar

        # predict
        self.deltaStateMean = self.A*self.deltaStateMean + self.B*delta_u_2 + self.gamma*delta_a_1
        self.deltaStateVar = self.A*self.deltaStateVar*self.A + self.W

    def parameterEstimation(self, delta_u_1, delta_a, delta_y):

        self.H = self.C*delta_u_1

        # predict
        self.B_k = self.F*self.B_k + self.omega*(self.beta-self.B_k)
        self.P = self.F*self.P*self.F + self.omega*self.P*self.omega + self.Q

        # update
        R_k = (self.C*self.A)*self.deltaStateVar*(self.C*self.A) + self.H*self.P*self.H + self.C*self.W*self.C + self.V
        K = self.P*self.H*(1.0/R_k)
        error = delta_y - (self.C*self.A*self.deltaStateMean + self.H*self.B_k + self.C*self.gamma*delta_a)

        self.B_k = self.B_k + K*error
        self.P = (1-K*self.H)*self.P

    def probDetection(self):

        # cumulative distribution function
        loc = self.B_k
        scale = np.sqrt(self.P)
        self.cdf = stats.norm.cdf(self.region[1], loc, scale) - stats.norm.cdf(self.region[0], loc, scale)

        self.alarm = 0
        if self.cdf < self.probThreshold:
            self.alarm = 1

    def getStateMean(self):
        return self.deltaStateMean

    def getStateCovariance(self):
        return self.deltaStateVar

    def getBMean(self):
        return self.B_k

    def getBCovariance(self):
        return self.P

    def getCDF(self):
        return self.cdf
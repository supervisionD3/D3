import numpy as np
np.set_printoptions(suppress=True)

from scipy import stats

class DeviationDetector:
    def __init__(self):

        # nominal model
        self.A = np.zeros((3,3))

        self.B = np.array([[0.42592, -0.37037, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.37037, -0.37037, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.37037, -0.037036]])

        self.C = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])

        # parameter deviation
        self.F = np.eye(6)
        self.omega = 0.001*np.eye(6)
        self.Q = np.array([[2.0e-07, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 2.0e-07, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 2.0e-07, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 2.0e-07, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 2.0e-07, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 2.0e-07]])

        # uncertainty compensation
        self.W = np.array([[2.0e-24, 0.0, 0.0],
                           [0.0, 2.0e-24, 0.0],
                           [0.0, 0.0, 2.0e-24]])

        self.V = np.array([[0.00080, 0.0, 0.0],
                           [0.0, 0.00080, 0.0],
                           [0.0, 0.0, 0.00080]])

        # safety region quantification
        self.percent = [[0.35, 0.35], [0.35, 0.35],
                        [0.35, 0.35], [0.35, 0.35],
                        [0.35, 0.35], [3.50, 3.50]]

        self.region = np.array([[self.B[0][0]-self.percent[0][0]*abs(self.B[0][0]), self.B[0][0]+self.percent[0][1]*abs(self.B[0][0])],
                                [self.B[0][1]-self.percent[1][0]*abs(self.B[0][1]), self.B[0][1]+self.percent[1][1]*abs(self.B[0][1])],
                                [self.B[1][2]-self.percent[2][0]*abs(self.B[1][2]), self.B[1][2]+self.percent[2][1]*abs(self.B[1][2])],
                                [self.B[1][3]-self.percent[3][0]*abs(self.B[1][3]), self.B[1][3]+self.percent[3][1]*abs(self.B[1][3])],
                                [self.B[2][4]-self.percent[4][0]*abs(self.B[2][4]), self.B[2][4]+self.percent[4][1]*abs(self.B[2][4])],
                                [self.B[2][5]-self.percent[5][0]*abs(self.B[2][5]), self.B[2][5]+self.percent[5][1]*abs(self.B[2][5])]])

        self.probThreshold = 0.999999998  # 6sigma

        # state observer
        self.stateMean = np.zeros((3,3))
        self.stateVar = np.mean(0.00080)*np.eye(3)

        # state mean and variance
        self.B_k = self.B.T
        self.P = 2.0e-07*np.eye(6)

        # results
        self.cdf = np.zeros(6)
        self.alarm = np.zeros(6)

        # horizon list
        self.list_u = []
        self.list_y = []

    def detector(self, u, y):

        if len(self.list_u)<2:
            self.list_u.append(u)
            self.list_y.append(y)
        else:
            self.list_u.append(u)
            self.list_y.append(y)

            # update state
            y_2 = self.list_y[0]
            u_2 = self.list_u[0]

            if u_2[0][0]>0 or u_2[1][0]>0 or \
               u_2[2][1]>0 or u_2[3][1]>0 or \
               u_2[4][2]>0 or u_2[5][2]>0:
                self.stateEstimation(y_2, u_2)

            u_1 = self.list_u[1]
            y = self.list_y[2]

            # passive detector
            if u_1[0][0]>0 or u_1[1][0]>0 or \
               u_1[2][1]>0 or u_1[3][1]>0 or \
               u_1[4][2]>0 or u_1[5][2]>0:

                self.parameterEstimation(u_1, y)
                self.probDetection()

            # active detector
            ## Tank101
            if u_1[0][0]==0 and u_1[1][0]==0:
                self.alarm[0] = self.alarm[1] = self.activeDetection(y[0][0])

            ## Tank301
            if u_1[2][1]==0 and u_1[3][1]==0:
                self.alarm[2] = self.alarm[3] = self.activeDetection(y[1][1])

            ## Tank401
            if u_1[4][2]==0 and u_1[5][2]==0:
                self.alarm[4] = self.alarm[5] = self.activeDetection(y[2][2])

            # Update old data
            self.list_u.remove(self.list_u[0])
            self.list_y.remove(self.list_y[0])

        return self.alarm

    def stateEstimation(self, y_2, u_2):

        # update
        R_k = np.dot(np.dot(self.C,self.stateVar),self.C) + self.V
        K = np.dot(np.dot(self.stateVar,self.C.T),np.linalg.inv(R_k))
        error = y_2 - np.dot(self.C,self.stateVar)

        self.stateMean = self.stateMean + np.dot(K,error)
        self.stateVar = np.dot((np.eye(len(self.stateVar))-np.dot(K,self.C)), self.stateVar)

        # predict
        self.stateMean = np.dot(self.A,self.stateMean) + np.dot(self.B,u_2)
        self.stateVar = np.dot(np.dot(self.A,self.stateVar),self.A)

    def parameterEstimation(self, u_1, y):

        H = np.dot(self.C,u_1.T)

        # predict
        self.B_k = np.dot(self.F,self.B_k) + np.dot(self.omega,(self.B.T-self.B_k))
        for i in range(0, np.size(self.P,0)):
            if abs(u_1[i][int(i/2)])>0.0:
                self.P[i,i] = self.F[i,i]*self.P[i,i]*self.F[i,i] + self.omega[i,i]*self.P[i,i]*self.omega[i,i] + self.Q[i,i]

        # update
        R_k =  np.dot(np.dot(np.dot(self.C,self.A), self.stateVar), np.dot(self.C,self.A).T) + np.dot(np.dot(H,self.P),H.T) + self.V
        K = np.dot(np.dot(self.P,H.T),np.linalg.inv(R_k))
        error = y - (np.dot(np.dot(self.C,self.A),self.stateMean) + np.dot(H,self.B_k))

        self.B_k = self.B_k + np.dot(K,error)
        self.P = np.dot((np.eye(len(self.P))-np.dot(K,H)), self.P)

    def probDetection(self):

        # cumulative distribution function
        # tank101
        loc = self.B_k[0][0]
        scale = np.sqrt(self.P[0][0])
        self.cdf[0] = abs(stats.norm.cdf(self.region[0][1],loc,scale) - stats.norm.cdf(self.region[0][0],loc,scale))

        loc = self.B_k[1][0]
        scale = np.sqrt(self.P[1][1])
        self.cdf[1] = stats.norm.cdf(self.region[1][1],loc,scale) - stats.norm.cdf(self.region[1][0],loc,scale)

        # tank301
        loc = self.B_k[2][1]
        scale = np.sqrt(self.P[2][2])
        self.cdf[2] = stats.norm.cdf(self.region[2][1],loc,scale) - stats.norm.cdf(self.region[2][0],loc,scale)

        loc = self.B_k[3][1]
        scale = np.sqrt(self.P[3][3])
        self.cdf[3] = stats.norm.cdf(self.region[3][1],loc,scale) - stats.norm.cdf(self.region[3][0],loc,scale)

        # tank401
        loc = self.B_k[4][2]
        scale = np.sqrt(self.P[4][4])
        self.cdf[4] = stats.norm.cdf(self.region[4][1], loc, scale) - stats.norm.cdf(self.region[4][0], loc, scale)

        loc = self.B_k[5][2]
        scale = np.sqrt(self.P[5][5])
        self.cdf[5] = stats.norm.cdf(self.region[5][1], loc, scale) - stats.norm.cdf(self.region[5][0], loc, scale)

        # Alarm
        size = np.size(self.P,0)
        for i in range(0, size):
            if self.cdf[i] < self.probThreshold:
                self.alarm[i] = 1

                if self.alarm[i]==1:
                    print("passive alarm:", loc,scale)

    def activeDetection(self, z):
        # H_0: prob < 1.0-self.probThreshold
        # H_1: prob >= 1.0-self.probThreshold

        loc = 0.0
        scale = np.sqrt(self.V[0][0])

        if z<=loc:
            cdf = stats.norm.cdf(z,loc,scale)
        else:
            cdf = 1.0-stats.norm.cdf(z,loc,scale)

        active_alarm = 0
        if cdf < 1.0-self.probThreshold:
            active_alarm = 1

        if active_alarm==1:
            print("scale:{}".format(6*scale))
            print("cdf:{}".format(cdf))
            print("ative alarm", z)

        return active_alarm

    def getStateMean(self):
        return self.stateMean

    def getStateCovariance(self):
        return self.stateVar

    def getBMean(self):
        return self.B_k

    def getBCovariance(self):
        return self.P

    def getCDF(self):
        return self.cdf
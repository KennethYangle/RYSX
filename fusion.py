import numpy as np

class KF:
    def __init__(self, F, H, dt, Q, R):
        self.F = F
        self.H = H
        self.dt = dt
        self.Q = Q
        self.R = R

        self.n_state = self.F.shape[0]
        self.n_obser = len(H)
        self.initial()

    def initial(self):
        self.x_k_k = np.zeros(self.n_state)
        self.P_k_k = np.tile(np.identity(self.n_state), (self.n_obser,1,1))
        self.P_k_kp = np.tile(np.zeros((self.n_state,self.n_state)), (self.n_obser,1,1))
        return self.x_k_k

    def prediction(self):
        self.x_k_kp = self.F.dot(self.x_k_k)
        for s in range(self.n_obser):
            self.P_k_kp[s] = self.F.dot(self.P_k_k[s]) + self.Q

    def measurement(self, z):
        for k, v in z.items():
            S_k = self.R[k] + self.H[k].dot(self.P_k_kp[k]).dot(self.H[k].T)
            K_k = self.P_k_kp[k].dot(self.H[k].T).dot( np.linalg.inv(S_k) )
            self.P_k_k[k] = self.P_k_kp[k] - K_k.dot(self.H[k]).dot(self.P_k_kp[k])

            nu = np.array(v) - self.H.dot(self.x_k_kp)
            self.x_k_k = self.x_k_kp + K_k.dot(nu)

    def update(self, z):
        self.prediction()
        self.measurement(z)
        return self.x_k_k
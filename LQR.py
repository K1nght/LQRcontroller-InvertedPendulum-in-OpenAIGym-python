import numpy as np 

class LQR:
    def __init__(self, x_n, u_n, F_t, f_t, C_t, c_t):
        self.x_n = x_n 
        self.u_n = u_n 
        assert F_t.shape == (x_n, x_n+u_n)
        assert f_t.shape == (x_n, 1)
        assert C_t.shape == (x_n+u_n, x_n+u_n)
        assert c_t.shape == (x_n+u_n, 1)
        self.F_t = F_t 
        self.f_t = f_t 
        self.C_t = C_t 
        self.c_t = c_t 

        self.C_xtxt = C_t[:x_n, :x_n]
        self.C_xtut = C_t[:x_n, x_n:]
        self.C_utxt = C_t[x_n:, :x_n]
        self.C_utut = C_t[x_n:, x_n:]

        self.c_xt = c_t[:x_n, :]
        self.c_ut = c_t[x_n:, :]

    def __step(self, x_t, u_t):
        return self.F_t@np.concatenate([x_t, u_t]) + self.f_t
    
    def __call__(self, x_0, T):
        V_t = np.zeros((self.x_n, self.x_n))
        v_t = np.zeros((self.x_n, 1))
        K = [np.zeros((self.u_n, self.x_n)) for  _ in range(T)]
        k = [np.zeros((self.u_n, 1)) for _ in range(T)]
        for i in range(T-1, -1, -1):
            Q_t = self.C_t + self.F_t.T@V_t@self.F_t 
            q_t = self.c_t + self.F_t.T@V_t@self.f_t + self.F_t.T@v_t 
            Q_xtxt = Q_t[:self.x_n, :self.x_n]
            Q_xtut = Q_t[:self.x_n, self.x_n:]
            Q_utxt = Q_t[self.x_n:, :self.x_n]
            Q_utut = Q_t[self.x_n:, self.x_n:]
            q_xt = q_t[:self.x_n, :]
            q_ut = q_t[self.x_n:, :]
            K[i] = -np.linalg.inv(Q_utut)@Q_utxt 
            k[i] = -np.linalg.inv(Q_utut)@q_ut 
            V_t = Q_xtxt + Q_xtut@K[i] + K[i].T@Q_utxt + K[i].T@Q_utut@K[i]
            v_t = q_xt + Q_xtut@k[i] + K[i].T@q_ut + K[i].T@Q_utut@k[i]
        x_t = x_0.copy()
        u = [np.zeros((self.u_n, 1)) for _ in range(T)]
        for i in range(T):
            u[i] = K[i]@x_t + k[i]
            x_t = self.__step(x_t, u[i])
        return u 
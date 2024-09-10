import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
import cvxpy as cp

class System:
    def __init__(self, A, B, C, Q_v, Q_w, **kwargs):
        self.A = A
        self.B = B
        self.C = C

        self.Q_v = Q_v
        self.Q_w = Q_w

        self.Q_v_inv = np.linalg.inv(Q_v)
        self.Q_w_inv = np.linalg.inv(Q_w)

        self.n_states = np.shape(A)[0]
        self.n_inputs = np.shape(B)[1]
        self.n_outputs = np.shape(C)[0]

        # Handle additional keyword arguments
        for key, value in kwargs.items():
            setattr(self, key, value)

    def c2d(self, ts):
        """
        C2D computes a discrete-time model of a system (A_c,B_c) with sample time ts.
        The function returns matrices Ad, Bd of the discrete-time system.
        """
        A = self.A
        B = self.B

        n = self.n_states
        nb = self.n_inputs

        s = np.concatenate([A,B], axis=1)
        s = np.concatenate([s, np.zeros((nb, n+nb))], axis=0)
        S = la.expm(s*ts)
        Ad = S[0:n,0:n]
        Bd = S[0:n,n:n+nb+1]
        sysd = System(A=Ad, B=Bd, C=self.C, Q_v=self.Q_v, Q_w=self.Q_w, ts=ts)
        return sysd

class Data:
    def __init__(self,y,u):
        self.y = y
        self.u = u

class QRegularization:
    def __init__(self, Q_u, Q_x, c_u, c_x):
        self.Q_u = Q_u
        self.Q_x = Q_x
        self.c_u = c_u
        self.c_x = c_x

class QConstraints:
    def __init__(self, A_u, A_x, b_u, b_x):
        self.A_u = A_u
        self.A_x = A_x
        self.b_u = b_u
        self.b_x = b_x

class Problem:
    def __init__(self, sys: System, data: Data, reg: QRegularization, constr: QConstraints, x0, u0, HORIZON_LENGTH):
        self.sys = sys
        self.data = data
        self.reg = reg
        self.constr = constr
        self.HORIZON_LENGTH = HORIZON_LENGTH
        # variables
        self.x, self.u = self.create_variables()
        # initial state and input estimate
        self.x0 = x0
        self.u0 = u0
        # create problem
        self.problem = self.update_problem()

    def create_variables(self):
        x = cp.Variable((self.sys.n_states, self.HORIZON_LENGTH), complex=False)
        u = cp.Variable((self.sys.n_inputs, self.HORIZON_LENGTH), complex=False)
        return x, u
    
    def update_problem(self):
        # create objective function
        obj = 0
        #obj = cp.quad_form( self.x, self.reg.Q_x ) + self.reg.c_x.T @ self.x
        #obj = cp.quad_form( self.u, self.reg.Q_u ) + self.reg.c_u.T @ self.u
        for k in range(self.HORIZON_LENGTH):
            if k == 0:
                obj += cp.quad_form( self.data.y[:,k] - self.sys.C@self.x[:,k], self.sys.Q_v_inv ) + cp.quad_form( self.sys.A@self.x0 + self.sys.B@self.u[:,k] - self.x[:,k] , self.sys.Q_w_inv ) 
            else:
                obj += cp.quad_form( self.data.y[:,k] - self.sys.C@self.x[:,k], self.sys.Q_v_inv ) + cp.quad_form( self.sys.A@self.x[:,k-1] + self.sys.B@self.u[:,k] - self.x[:,k] , self.sys.Q_w_inv )
        objective = cp.Minimize(obj)

        constraints = []
        #constraints.append( self.constr.A_u @ self.u + self.constr.b_u.T @ self.u )
        #constraints.append( constraints = self.constr.A_x @ self.x + self.constr.b_x.T @ self.x )
        
        # create problem
        return cp.Problem(objective, constraints)

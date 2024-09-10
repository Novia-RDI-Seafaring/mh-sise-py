import cvxpy as cp
import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt

class LTI:
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
        sysd = LTI(A=Ad, B=Bd, C=self.C, Q_v=self.Q_v, Q_w=self.Q_w, ts=ts)
        return sysd

class Data:
    def __init__(self,y,u):
        self.y = y
        self.u = u

class Problem:
    def __init__(self, lti: LTI, data: Data, regul, constr, x0, u0, HORIZON_LENGTH):
        self.lti = lti
        # regularization term
        self.regul = regul
        # coinstraints
        self.constr = constr

        self.HORIZON_LENGTH = HORIZON_LENGTH

        # variables
        self.x, self.u = self.create_variables()

        # initial state and input estimate
        self.x0 = x0
        self.u0 = u0

    def create_variables(self):
        x = cp.Variable((self.lti.n_states, self.HORIZON_LENGTH), complex=False)
        u = cp.Variable((self.lti.n_inputs, self.HORIZON_LENGTH), complex=False)
        return x, u
    
    def create_objective(self):
        obj = self.regul
        for k in range(self.HORIZON_LENGTH):
            if k == 0:
                obj += cp.quad_form( y[k] - self.lti.C@self.x[:,k], self.lti.Q_v_inv ) + cp.quad_form( self.lti.A@self.x0 + self.lti.B@self.u[:,k] - x[:,k] , self.lti.Q_w_inv ) 
            else:
                obj += cp.quad_form( y[k] - self.lti.C@self.x[:,k], self.lti.Q_v_inv ) + cp.quad_form( self.lti.A@self.x[:,k-1] + self.lti.B@self.u[:,k] - self.x[:,k] , self.lti.Q_w_inv )

        return cp.Minimize(obj)
    
### EXAMPLE ###
inertia = [1E-2, 1]
damping = [.01, .01]
stiffness = [1000, 1000]
friction = [0.1]

A = np.array([
    [-(friction[0]+damping[0]/inertia[0]), damping[0]/inertia[0], -stiffness[0]/inertia[0], stiffness[0]/inertia[0]],
    [damping[0]/inertia[1], -(damping[0]-damping[1])/inertia[1], stiffness[0]/inertia[1], -(stiffness[0]+stiffness[1])/inertia[1]],
    [1, 0, 0, 0],
    [0, 1, 0, 0]
    ])
B = np.array([
    [1/inertia[0], 0],
    [0, -1/inertia[1]],
    [0, 0],
    [0, 0]
])
C = np.array([
    [0, 0, stiffness[0], -stiffness[1]],
    [1, 0, 0, 0]
])
Q_v = 0.01*np.eye(2)
Q_w = 0.01*np.eye(4)

# create system model
lti = LTI(A=A, B=B, C=C, Q_v=Q_v, Q_w=Q_w)
dlti = lti.c2d(ts=0.001)

# create data
HORIZON_LENGTH = 1000
u = 1*np.ones((dlti.n_inputs, HORIZON_LENGTH))
u[0,:] = np.sin( 2* np.pi * 10 * np.arange(HORIZON_LENGTH) * dlti.ts )
y = np.zeros((dlti.n_outputs, HORIZON_LENGTH))
x = np.zeros((dlti.n_states, HORIZON_LENGTH))
for k in range(HORIZON_LENGTH-1):
    x[:,k+1] = dlti.A @ x[:,k] + dlti.B @ u[:,k]
    y[:,k] = dlti.C @ x[:,k]

data = Data(y=y, u=u)

# create problem
prob = Problem(lti=dlti, data=data, regul=0, constr=[], x0=np.zeros((4,1)), u0=np.zeros((2,1)), HORIZON_LENGTH=HORIZON_LENGTH)
obj = prob.create_objective()

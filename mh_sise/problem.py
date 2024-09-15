
import cvxpy as cp
import time
from cvxpygen import cpg
import pickle

class Problem:
    def __init__(self, n, m, p, HORIZON_LENGTH):
        # Initialize parameters
        self.n = n
        self.m = m
        self.p = p
        self.HORIZON_LENGTH = HORIZON_LENGTH
        
        # Define parameters, variables, objective, and constraints
        self.create_parameters()
        self.create_variables()
        self.create_objective()
        self.create_constraints()
        self.create_problem()
    
    def create_parameters(self):
        # Define the parameters
        self.Q_v_inv_sqrt = cp.Parameter(self.p, name='Q_v_inv_sqrt')
        self.Q_w_inv_sqrt = cp.Parameter((self.n, self.n), name='Q_w_inv_sqrt')
        self.Q_u_inv_sqrt = cp.Parameter(self.m, name='Q_u_inv_sqrt')
        self.y = cp.Parameter((self.p, self.HORIZON_LENGTH), name='y')
        self.x0 = cp.Parameter(self.n, name='x0')
        self.u0 = cp.Parameter(self.n, name='u0')
        self.A = cp.Parameter((self.n, self.n), name='A')
        self.B = cp.Parameter((self.n, self.m), name='B')
        self.C = cp.Parameter((self.p, self.n), name='C')
   
    def create_variables(self):
        # Define the variables
        self.X = cp.Variable((self.n, self.HORIZON_LENGTH), name='X')
        self.U = cp.Variable((self.m, self.HORIZON_LENGTH - 1), name='U')
        self.V = cp.Variable((self.p, self.HORIZON_LENGTH), name='V')
        self.W = cp.Variable((self.n, self.HORIZON_LENGTH), name='W')

    def create_objective(self):
        # Define the objective function based on variables and parameters
        self.objective = cp.Minimize(
            cp.sum_squares(self.Q_w_inv_sqrt @ self.W) +
            cp.sum_squares(self.Q_v_inv_sqrt @ self.V) +
            cp.sum_squares(self.Q_u_inv_sqrt @ self.U)
        )
    
    def create_constraints(self):
        # Define the constraints
        self.constraints = [
            self.W[:, 0] == self.X[:, 0] - self.x0,
            self.W[:, 1:] == self.X[:, 1:self.HORIZON_LENGTH] - self.A @ self.X[:, 0:self.HORIZON_LENGTH - 1] - self.B @ self.U,
            self.V[:, :self.HORIZON_LENGTH] == self.y[:, :self.HORIZON_LENGTH] - self.C @ self.X[:, :self.HORIZON_LENGTH],
            cp.abs(self.U) <= 1
        ]
    
    def create_problem(self):
        # Create the optimization problem
        self.problem = cp.Problem(self.objective, self.constraints)

    def assign_parameter_values(self, **kwargs):
    # Loop through the keyword arguments and set values for matching parameters
        for param_name, param_value in kwargs.items():
            if hasattr(self, param_name):
                getattr(self, param_name).value = param_value
            else:
                raise AttributeError(f"Parameter '{param_name}' does not exist in the Problem class.")
        self.update_problem()
    
    def get_variable_values(self):
        # Retrieve the values of all decision variables in the problem's variable dictionary
        solution_values = {}
        
        # Loop through the variable dictionary and get values
        for var_name, var in self.problem.var_dict.items():
            solution_values[var_name] = var.value
        
        return solution_values

    def solve(self):
        # Solve the optimization problem
        t0 = time.time()
        solution = self.problem.solve(eps_abs=1e-3, eps_rel=1e-3, max_iter=4000, polish=False, ignore_dpp=True) #why faster with ignore_dpp=True?
        t1 = time.time()

        print('\nCVXPY\nSolve time: %.3f ms' % (1000 * (t1 - t0)))
        print('Objective function value: %.6f\n' % solution)

    def generate_code(self, pth, fnm):
        # compiles code with cvxgen
        cpg.generate_code(self.problem, code_dir=f'{pth}{fnm}')

class CProblem:
    def __init__(self, pth):
        self.pth = pth

        self.get_problem()

    def get_problem(self):
        with open(self.pth + 'problem.pickle', 'rb') as f:
            self.problem = pickle.load(f)
        print(f'Problem loaded from path: {self.pth + 'problem.pickle'}')

    def assign_parameter_values(self, **kwargs):       
        # Loop through the keyword arguments and set values for matching parameters
        for param_name, param_value in kwargs.items():
            if param_name in self.problem.param_dict:
                param = self.problem.param_dict[param_name]
                param.value = param_value
            else:
                raise AttributeError(f"Parameter '{param_name}' does not exist in the Problem's parameter dictionary.")
            
    def get_parameter_values(self):
        # Retrieve the values of all parameters in the problem's parameter dictionary
        param_values = {}
        
        # Loop through the parameter dictionary and get values
        for param_name, param in self.problem.param_dict.items():
            param_values[param_name] = param.value
        
        return param_values
    
    def get_variable_values(self):
        # Retrieve the values of all variables in the problem's variable dictionary
        variable_values = {}
        
        # Loop through the variable dictionary and get values
        for var_name, var in self.problem.var_dict.items():
            variable_values[var_name] = var.value
        
        return variable_values
    
    def solve(self, cpg_solve):
        # Solve the optimization problem
        self.problem.register_solve('CPG', cpg_solve)
        t0 = time.time()
        val = self.problem.solve(method='CPG')
        t1 = time.time()

        print('\nCVXPYgen\nSolve time: %.3f ms' % (1000 * (t1 - t0)))
        print('Objective function value: %.6f\n' % val)
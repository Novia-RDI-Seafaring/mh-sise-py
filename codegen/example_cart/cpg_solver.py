
"""
Auto-generated by CVXPYgen on September 15, 2024 at 22:02:05.
Content: Custom solve method for CVXPY interface.
"""

import time
import warnings
import numpy as np
from cvxpy.reductions import Solution
from cvxpy.problems.problem import SolverStats
from codegen.example_cart import cpg_module


standard_settings_names = {"max_iters": "maxit"}


def cpg_solve(prob, updated_params=None, **kwargs):

    # set flags for updated parameters
    upd = cpg_module.cpg_updated()
    if updated_params is None:
        updated_params = ["Q_w_inv_sqrt", "Q_v_inv_sqrt", "Q_u_inv_sqrt", "x0", "A", "B", "y", "C"]
    for p in updated_params:
        try:
            setattr(upd, p, True)
        except AttributeError:
            raise AttributeError(f"{p} is not a parameter.")

    # set solver settings
    cpg_module.set_solver_default_settings()
    for key, value in kwargs.items():
        try:
            eval(f'cpg_module.set_solver_{standard_settings_names.get(key, key)}(value)')
        except AttributeError:
            raise AttributeError(f'Solver setting "{key}" not available.')

    # set parameter values
    par = cpg_module.cpg_params()
    param_dict = prob.param_dict
    par.Q_w_inv_sqrt = list(param_dict["Q_w_inv_sqrt"].value.flatten(order="F"))
    par.Q_v_inv_sqrt = param_dict["Q_v_inv_sqrt"].value
    par.Q_u_inv_sqrt = param_dict["Q_u_inv_sqrt"].value
    par.x0 = list(param_dict["x0"].value.flatten(order="F"))
    par.A = list(param_dict["A"].value.flatten(order="F"))
    par.B = list(param_dict["B"].value.flatten(order="F"))
    par.y = list(param_dict["y"].value.flatten(order="F"))
    par.C = list(param_dict["C"].value.flatten(order="F"))

    # solve
    t0 = time.time()
    res = cpg_module.solve(upd, par)
    t1 = time.time()

    # store solution in problem object
    prob._clear_solution()
    prob.var_dict['W'].save_value(np.array(res.cpg_prim.W).reshape((2, 100), order='F'))
    prob.var_dict['V'].save_value(np.array(res.cpg_prim.V).reshape((1, 100), order='F'))
    prob.var_dict['U'].save_value(np.array(res.cpg_prim.U).reshape((1, 99), order='F'))
    prob.var_dict['X'].save_value(np.array(res.cpg_prim.X).reshape((2, 100), order='F'))
    prob.constraints[0].save_dual_value(np.array(res.cpg_dual.d0).reshape(2))
    prob.constraints[1].save_dual_value(np.array(res.cpg_dual.d1).reshape((2, 99), order='F'))
    prob.constraints[2].save_dual_value(np.array(res.cpg_dual.d2).reshape((1, 100), order='F'))
    prob.constraints[3].save_dual_value(np.array(res.cpg_dual.d3).reshape((1, 99), order='F'))

    # store additional solver information in problem object
    prob._status = res.cpg_info.status
    if abs(res.cpg_info.obj_val) == 1e30:
        prob._value = np.sign(res.cpg_info.obj_val) * np.inf
    else:
        prob._value = res.cpg_info.obj_val
    primal_vars = {var.id: var.value for var in prob.variables()}
    dual_vars = {c.id: c.dual_value for c in prob.constraints}
    solver_specific_stats = {'obj_val': res.cpg_info.obj_val,
                             'status': prob._status,
                             'iter': res.cpg_info.iter,
                             'pri_res': res.cpg_info.pri_res,
                             'dua_res': res.cpg_info.dua_res,
                             'time': res.cpg_info.time}
    attr = {'solve_time': t1 - t0, 'solver_specific_stats': solver_specific_stats, 'num_iters': res.cpg_info.iter}
    prob._solution = Solution(prob.status, prob.value, primal_vars, dual_vars, attr)
    results_dict = {'solver_specific_stats': solver_specific_stats,
                    'num_iters': res.cpg_info.iter,
                    'solve_time': t1 - t0}
    prob._solver_stats = SolverStats(results_dict, 'OSQP')

    return prob.value


/*
Auto-generated by CVXPYgen on September 11, 2024 at 20:51:33.
Content: Function declarations.
*/

#include "cpg_workspace.h"

// Update user-defined parameter values
extern void cpg_update_Q_w_inv_sqrt(cpg_int idx, cpg_float val);
extern void cpg_update_Q_v_inv_sqrt(cpg_int idx, cpg_float val);
extern void cpg_update_Q_u_inv_sqrt(cpg_int idx, cpg_float val);
extern void cpg_update_x0(cpg_int idx, cpg_float val);
extern void cpg_update_A(cpg_int idx, cpg_float val);
extern void cpg_update_B(cpg_int idx, cpg_float val);
extern void cpg_update_y(cpg_int idx, cpg_float val);
extern void cpg_update_C(cpg_int idx, cpg_float val);

// Map user-defined to canonical parameters
extern void cpg_canonicalize_A();
extern void cpg_canonicalize_l();
extern void cpg_canonicalize_u();

// Retrieve solver information
extern void cpg_retrieve_info();

// Solve via canonicalization, canonical solve, retrieval
extern void cpg_solve();

// Update solver settings
extern void cpg_set_solver_default_settings();
extern void cpg_set_solver_rho(cpg_float rho_new);
extern void cpg_set_solver_max_iter(cpg_int max_iter_new);
extern void cpg_set_solver_eps_abs(cpg_float eps_abs_new);
extern void cpg_set_solver_eps_rel(cpg_float eps_rel_new);
extern void cpg_set_solver_eps_prim_inf(cpg_float eps_prim_inf_new);
extern void cpg_set_solver_eps_dual_inf(cpg_float eps_dual_inf_new);
extern void cpg_set_solver_alpha(cpg_float alpha_new);
extern void cpg_set_solver_scaled_termination(cpg_int scaled_termination_new);
extern void cpg_set_solver_check_termination(cpg_int check_termination_new);
extern void cpg_set_solver_warm_start(cpg_int warm_start_new);

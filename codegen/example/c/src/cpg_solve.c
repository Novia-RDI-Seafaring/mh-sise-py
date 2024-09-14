
/*
Auto-generated by CVXPYgen on September 14, 2024 at 09:41:42.
Content: Function definitions.
*/

#include "cpg_solve.h"
#include "cpg_workspace.h"

static cpg_int i;
static cpg_int j;

// Update user-defined parameters
void cpg_update_Q_w_inv_sqrt(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+0] = val;
  Canon_Outdated.A = 1;
}

void cpg_update_Q_v_inv_sqrt(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+9] = val;
  Canon_Outdated.A = 1;
}

void cpg_update_Q_u_inv_sqrt(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+13] = val;
  Canon_Outdated.A = 1;
}

void cpg_update_x0(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+17] = val;
  Canon_Outdated.l = 1;
  Canon_Outdated.u = 1;
}

void cpg_update_A(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+20] = val;
  Canon_Outdated.A = 1;
}

void cpg_update_B(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+29] = val;
  Canon_Outdated.A = 1;
}

void cpg_update_y(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+35] = val;
  Canon_Outdated.l = 1;
  Canon_Outdated.u = 1;
}

void cpg_update_C(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+235] = val;
  Canon_Outdated.A = 1;
}

// Map user-defined to canonical parameters
void cpg_canonicalize_A(){
  for(i=0; i<6269; i++){
    Canon_Params.A->x[i] = 0;
    for(j=canon_A_map.p[i]; j<canon_A_map.p[i+1]; j++){
      Canon_Params.A->x[i] += canon_A_map.x[j]*cpg_params_vec[canon_A_map.i[j]];
    }
  }
}

void cpg_canonicalize_l(){
  for(i=0; i<1198; i++){
    Canon_Params.l[i] = 0;
    for(j=canon_l_map.p[i]; j<canon_l_map.p[i+1]; j++){
      Canon_Params.l[i] += canon_l_map.x[j]*cpg_params_vec[canon_l_map.i[j]];
    }
  }
}

void cpg_canonicalize_u(){
  for(i=0; i<1792; i++){
    Canon_Params.u[i] = 0;
    for(j=canon_u_map.p[i]; j<canon_u_map.p[i+1]; j++){
      Canon_Params.u[i] += canon_u_map.x[j]*cpg_params_vec[canon_u_map.i[j]];
    }
  }
}

// Retrieve solver info
void cpg_retrieve_info(){
  CPG_Info.obj_val = (workspace.info->obj_val);
  CPG_Info.iter = workspace.info->iter;
  CPG_Info.status = workspace.info->status;
  CPG_Info.pri_res = workspace.info->pri_res;
  CPG_Info.dua_res = workspace.info->dua_res;
}

// Solve via canonicalization, canonical solve, retrieval
void cpg_solve(){
  // Canonicalize if necessary
  if (Canon_Outdated.A) {
    cpg_canonicalize_A();
  }
  if (Canon_Outdated.l) {
    cpg_canonicalize_l();
  }
  if (Canon_Outdated.u) {
    cpg_canonicalize_u();
  }
  if ( Canon_Outdated.l&&Canon_Outdated.u) {
    osqp_update_bounds(&workspace, Canon_Params.l, Canon_Params.u);
  } else {
    if ( Canon_Outdated.l) {
      osqp_update_lower_bound(&workspace, Canon_Params.l);
    }
    if ( Canon_Outdated.u) {
      osqp_update_upper_bound(&workspace, Canon_Params.u);
    }
  }
  if ( Canon_Outdated.A) {
    osqp_update_A(&workspace, Canon_Params.A->x, 0, 0);
  }
  // Solve with OSQP
  osqp_solve(&workspace);
  // Retrieve results
  cpg_retrieve_info();
  // Reset flags for outdated canonical parameters
  Canon_Outdated.A = 0;
  Canon_Outdated.l = 0;
  Canon_Outdated.u = 0;
}

// Update solver settings
void cpg_set_solver_default_settings(){
  osqp_set_default_settings(&settings);
}

void cpg_set_solver_rho(cpg_float rho_new){
  osqp_update_rho(&workspace, rho_new);
}

void cpg_set_solver_max_iter(cpg_int max_iter_new){
  osqp_update_max_iter(&workspace, max_iter_new);
}

void cpg_set_solver_eps_abs(cpg_float eps_abs_new){
  osqp_update_eps_abs(&workspace, eps_abs_new);
}

void cpg_set_solver_eps_rel(cpg_float eps_rel_new){
  osqp_update_eps_rel(&workspace, eps_rel_new);
}

void cpg_set_solver_eps_prim_inf(cpg_float eps_prim_inf_new){
  osqp_update_eps_prim_inf(&workspace, eps_prim_inf_new);
}

void cpg_set_solver_eps_dual_inf(cpg_float eps_dual_inf_new){
  osqp_update_eps_dual_inf(&workspace, eps_dual_inf_new);
}

void cpg_set_solver_alpha(cpg_float alpha_new){
  osqp_update_alpha(&workspace, alpha_new);
}

void cpg_set_solver_scaled_termination(cpg_int scaled_termination_new){
  osqp_update_scaled_termination(&workspace, scaled_termination_new);
}

void cpg_set_solver_check_termination(cpg_int check_termination_new){
  osqp_update_check_termination(&workspace, check_termination_new);
}

void cpg_set_solver_warm_start(cpg_int warm_start_new){
  osqp_update_warm_start(&workspace, warm_start_new);
}

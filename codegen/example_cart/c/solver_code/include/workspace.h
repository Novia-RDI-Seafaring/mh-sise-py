#ifndef WORKSPACE_H
#define WORKSPACE_H

/*
 * This file was autogenerated by OSQP-Python on September 15, 2024 at 22:01:54.
 * 
 * This file contains the prototypes for all the workspace variables needed
 * by OSQP. The actual data is contained inside workspace.c.
 */

#include "types.h"
#include "qdldl_interface.h"

// Data structure prototypes
extern csc Pdata;
extern csc Adata;
extern c_float qdata[1097];
extern c_float ldata[996];
extern c_float udata[996];
extern OSQPData data;

// Settings structure prototype
extern OSQPSettings settings;

// Scaling structure prototypes
extern c_float Dscaling[1097];
extern c_float Dinvscaling[1097];
extern c_float Escaling[996];
extern c_float Einvscaling[996];
extern OSQPScaling scaling;

// Prototypes for linsys_solver structure
extern csc linsys_solver_L;
extern c_float linsys_solver_Dinv[2093];
extern c_int linsys_solver_P[2093];
extern c_float linsys_solver_bp[2093];
extern c_float linsys_solver_sol[2093];
extern c_float linsys_solver_rho_inv_vec[996];
extern c_int linsys_solver_Pdiag_idx[399];
extern csc linsys_solver_KKT;
extern c_int linsys_solver_PtoKKT[399];
extern c_int linsys_solver_AtoKKT[2787];
extern c_int linsys_solver_rhotoKKT[996];
extern QDLDL_float linsys_solver_D[2093];
extern QDLDL_int linsys_solver_etree[2093];
extern QDLDL_int linsys_solver_Lnz[2093];
extern QDLDL_int   linsys_solver_iwork[6279];
extern QDLDL_bool  linsys_solver_bwork[2093];
extern QDLDL_float linsys_solver_fwork[2093];
extern qdldl_solver linsys_solver;

// Prototypes for solution
extern c_float xsolution[1097];
extern c_float ysolution[996];

extern OSQPSolution solution;

// Prototype for info structure
extern OSQPInfo info;

// Prototypes for the workspace
extern c_float work_rho_vec[996];
extern c_float work_rho_inv_vec[996];
extern c_int work_constr_type[996];
extern c_float work_x[1097];
extern c_float work_y[996];
extern c_float work_z[996];
extern c_float work_xz_tilde[2093];
extern c_float work_x_prev[1097];
extern c_float work_z_prev[996];
extern c_float work_Ax[996];
extern c_float work_Px[1097];
extern c_float work_Aty[1097];
extern c_float work_delta_y[996];
extern c_float work_Atdelta_y[1097];
extern c_float work_delta_x[1097];
extern c_float work_Pdelta_x[1097];
extern c_float work_Adelta_x[996];
extern c_float work_D_temp[1097];
extern c_float work_D_temp_A[1097];
extern c_float work_E_temp[996];

extern OSQPWorkspace workspace;

#endif // ifndef WORKSPACE_H

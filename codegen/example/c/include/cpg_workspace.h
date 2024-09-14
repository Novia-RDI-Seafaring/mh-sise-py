
/*
Auto-generated by CVXPYgen on September 14, 2024 at 09:41:42.
Content: Type definitions and variable declarations.
*/

#include "osqp.h"
#include "types.h"
#include "workspace.h"

#ifndef CPG_TYPES_H
# define CPG_TYPES_H

typedef c_float cpg_float;
typedef c_int cpg_int;

// Compressed sparse column matrix
typedef struct {
  cpg_int      *p;
  cpg_int      *i;
  cpg_float    *x;
} cpg_csc;

// Canonical parameters
typedef struct {
  cpg_csc      *P;         // Canonical parameter P
  cpg_float    *q;         // Canonical parameter q
  cpg_float    d;          // Canonical parameter d
  cpg_csc      *A;         // Canonical parameter A
  cpg_float    *l;         // Canonical parameter l
  cpg_float    *u;         // Canonical parameter u
} Canon_Params_t;

// Flags indicating outdated canonical parameters
typedef struct {
  int        P;            // Bool, if canonical parameter P outdated
  int        q;            // Bool, if canonical parameter q outdated
  int        d;            // Bool, if canonical parameter d outdated
  int        A;            // Bool, if canonical parameter A outdated
  int        l;            // Bool, if canonical parameter l outdated
  int        u;            // Bool, if canonical parameter u outdated
} Canon_Outdated_t;

// Primal solution
typedef struct {
  cpg_float    *W;         // Your variable W
  cpg_float    *V;         // Your variable V
  cpg_float    *U;         // Your variable U
  cpg_float    *X;         // Your variable X
} CPG_Prim_t;

// Dual solution
typedef struct {
  cpg_float    *d0;        // Your dual variable for constraint d0
  cpg_float    *d1;        // Your dual variable for constraint d1
  cpg_float    *d2;        // Your dual variable for constraint d2
  cpg_float    *d3;        // Your dual variable for constraint d3
} CPG_Dual_t;

// Solver information
typedef struct {
  cpg_float    obj_val;    // Objective function value
  cpg_int      iter;       // Number of iterations
  char         *status;    // Solver status
  cpg_float    pri_res;    // Primal residual
  cpg_float    dua_res;    // Dual residual
} CPG_Info_t;

// Solution and solver information
typedef struct {
  CPG_Prim_t *prim;        // Primal solution
  CPG_Dual_t *dual;        // Dual solution
  CPG_Info_t *info;        // Solver info
} CPG_Result_t;

// Solver settings
typedef struct {
  cpg_float  rho;
  cpg_int    max_iter;
  cpg_float  eps_abs;
  cpg_float  eps_rel;
  cpg_float  eps_prim_inf;
  cpg_float  eps_dual_inf;
  cpg_float  alpha;
  cpg_int    scaled_termination;
  cpg_int    check_termination;
  cpg_int    warm_start;
} Canon_Settings_t;

#endif // ifndef CPG_TYPES_H

// Vector containing flattened user-defined parameters
extern cpg_float cpg_params_vec[242];

// Sparse mappings from user-defined to canonical parameters
extern cpg_csc canon_A_map;
extern cpg_csc canon_l_map;
extern cpg_csc canon_u_map;

// Canonical parameters
extern cpg_csc canon_P;
extern cpg_float canon_q[1894];
extern cpg_csc canon_A;
extern cpg_float canon_l[1792];
extern cpg_float canon_u[1792];

// Struct containing canonical parameters
extern Canon_Params_t Canon_Params;

// Struct containing flags for outdated canonical parameters
extern Canon_Outdated_t Canon_Outdated;

// Struct containing primal solution
extern CPG_Prim_t CPG_Prim;

// Struct containing dual solution
extern CPG_Dual_t CPG_Dual;

// Struct containing solver info
extern CPG_Info_t CPG_Info;

// Struct containing solution and info
extern CPG_Result_t CPG_Result;

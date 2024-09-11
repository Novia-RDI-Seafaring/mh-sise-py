
/*
Auto-generated by CVXPYgen on September 11, 2024 at 20:51:33.
Content: Example program for updating parameters, solving, and inspecting the result.
*/

#include <stdio.h>
#include "cpg_workspace.h"
#include "cpg_solve.h"

static int i;

int main(int argc, char *argv[]){

  // Update first entry of every user-defined parameter
  cpg_update_Q_w_inv_sqrt(0, 10.00000000000000000000);
  cpg_update_Q_v_inv_sqrt(0, 10.00000000000000000000);
  cpg_update_Q_u_inv_sqrt(0, 1.00000000000000000000);
  cpg_update_x0(0, 0.00000000000000000000);
  cpg_update_A(0, -0.00000000001233969416);
  cpg_update_B(0, 10.00004786590369221244);
  cpg_update_y(0, -0.06272387568574601391);
  cpg_update_C(0, 1.00000000000000000000);

  // Solve the problem instance
  cpg_solve();

  // Print objective function value
  printf("obj = %f\n", CPG_Result.info->obj_val);

  // Print primal solution
  for(i=0; i<300; i++) {
    printf("W[%d] = %f\n", i, CPG_Result.prim->W[i]);
  }
  for(i=0; i<200; i++) {
    printf("V[%d] = %f\n", i, CPG_Result.prim->V[i]);
  }
  for(i=0; i<198; i++) {
    printf("U[%d] = %f\n", i, CPG_Result.prim->U[i]);
  }
  for(i=0; i<300; i++) {
    printf("X[%d] = %f\n", i, CPG_Result.prim->X[i]);
  }

  // Print dual solution
  for(i=0; i<3; i++) {
    printf("d0[%d] = %f\n", i, CPG_Result.dual->d0[i]);
  }
  for(i=0; i<297; i++) {
    printf("d1[%d] = %f\n", i, CPG_Result.dual->d1[i]);
  }
  for(i=0; i<200; i++) {
    printf("d2[%d] = %f\n", i, CPG_Result.dual->d2[i]);
  }
  for(i=0; i<198; i++) {
    printf("d3[%d] = %f\n", i, CPG_Result.dual->d3[i]);
  }

  return 0;

}

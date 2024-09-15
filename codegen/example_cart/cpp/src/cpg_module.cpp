
/*
Auto-generated by CVXPYgen on September 15, 2024 at 22:02:05.
Content: Python binding with pybind11.
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ctime>
#include "cpg_module.hpp"

extern "C" {
    #include "include/cpg_workspace.h"
    #include "include/cpg_solve.h"
}

namespace py = pybind11;

static int i;

CPG_Result_cpp_t solve_cpp(struct CPG_Updated_cpp_t& CPG_Updated_cpp, struct CPG_Params_cpp_t& CPG_Params_cpp){

    // Pass changed user-defined parameter values to the solver
    if (CPG_Updated_cpp.Q_w_inv_sqrt) {
        for(i=0; i<4; i++) {
            cpg_update_Q_w_inv_sqrt(i, CPG_Params_cpp.Q_w_inv_sqrt[i]);
        }
    }
    if (CPG_Updated_cpp.Q_v_inv_sqrt) {
        cpg_update_Q_v_inv_sqrt(CPG_Params_cpp.Q_v_inv_sqrt);
    }
    if (CPG_Updated_cpp.Q_u_inv_sqrt) {
        cpg_update_Q_u_inv_sqrt(CPG_Params_cpp.Q_u_inv_sqrt);
    }
    if (CPG_Updated_cpp.x0) {
        for(i=0; i<2; i++) {
            cpg_update_x0(i, CPG_Params_cpp.x0[i]);
        }
    }
    if (CPG_Updated_cpp.A) {
        for(i=0; i<4; i++) {
            cpg_update_A(i, CPG_Params_cpp.A[i]);
        }
    }
    if (CPG_Updated_cpp.B) {
        for(i=0; i<2; i++) {
            cpg_update_B(i, CPG_Params_cpp.B[i]);
        }
    }
    if (CPG_Updated_cpp.y) {
        for(i=0; i<100; i++) {
            cpg_update_y(i, CPG_Params_cpp.y[i]);
        }
    }
    if (CPG_Updated_cpp.C) {
        for(i=0; i<2; i++) {
            cpg_update_C(i, CPG_Params_cpp.C[i]);
        }
    }

    // Solve
    std::clock_t ASA_start = std::clock();
    cpg_solve();
    std::clock_t ASA_end = std::clock();

    // Arrange and return results
    CPG_Prim_cpp_t CPG_Prim_cpp {};
    for(i=0; i<200; i++) {
        CPG_Prim_cpp.W[i] = CPG_Prim.W[i];
    }
    for(i=0; i<100; i++) {
        CPG_Prim_cpp.V[i] = CPG_Prim.V[i];
    }
    for(i=0; i<99; i++) {
        CPG_Prim_cpp.U[i] = CPG_Prim.U[i];
    }
    for(i=0; i<200; i++) {
        CPG_Prim_cpp.X[i] = CPG_Prim.X[i];
    }
    CPG_Dual_cpp_t CPG_Dual_cpp {};
    for(i=0; i<2; i++) {
        CPG_Dual_cpp.d0[i] = CPG_Dual.d0[i];
    }
    for(i=0; i<198; i++) {
        CPG_Dual_cpp.d1[i] = CPG_Dual.d1[i];
    }
    for(i=0; i<100; i++) {
        CPG_Dual_cpp.d2[i] = CPG_Dual.d2[i];
    }
    for(i=0; i<99; i++) {
        CPG_Dual_cpp.d3[i] = CPG_Dual.d3[i];
    }
    CPG_Info_cpp_t CPG_Info_cpp {};
    CPG_Info_cpp.obj_val = CPG_Info.obj_val;
    CPG_Info_cpp.iter = CPG_Info.iter;
    CPG_Info_cpp.status = CPG_Info.status;
    CPG_Info_cpp.pri_res = CPG_Info.pri_res;
    CPG_Info_cpp.dua_res = CPG_Info.dua_res;
    CPG_Info_cpp.time = 1.0 * (ASA_end - ASA_start) / CLOCKS_PER_SEC;
    CPG_Result_cpp_t CPG_Result_cpp {};
    CPG_Result_cpp.prim = CPG_Prim_cpp;
    CPG_Result_cpp.dual = CPG_Dual_cpp;
    CPG_Result_cpp.info = CPG_Info_cpp;
    return CPG_Result_cpp;

}

PYBIND11_MODULE(cpg_module, m) {

    py::class_<CPG_Params_cpp_t>(m, "cpg_params")
            .def(py::init<>())
            .def_readwrite("Q_w_inv_sqrt", &CPG_Params_cpp_t::Q_w_inv_sqrt)
            .def_readwrite("Q_v_inv_sqrt", &CPG_Params_cpp_t::Q_v_inv_sqrt)
            .def_readwrite("Q_u_inv_sqrt", &CPG_Params_cpp_t::Q_u_inv_sqrt)
            .def_readwrite("x0", &CPG_Params_cpp_t::x0)
            .def_readwrite("A", &CPG_Params_cpp_t::A)
            .def_readwrite("B", &CPG_Params_cpp_t::B)
            .def_readwrite("y", &CPG_Params_cpp_t::y)
            .def_readwrite("C", &CPG_Params_cpp_t::C)
            ;

    py::class_<CPG_Updated_cpp_t>(m, "cpg_updated")
            .def(py::init<>())
            .def_readwrite("Q_w_inv_sqrt", &CPG_Updated_cpp_t::Q_w_inv_sqrt)
            .def_readwrite("Q_v_inv_sqrt", &CPG_Updated_cpp_t::Q_v_inv_sqrt)
            .def_readwrite("Q_u_inv_sqrt", &CPG_Updated_cpp_t::Q_u_inv_sqrt)
            .def_readwrite("x0", &CPG_Updated_cpp_t::x0)
            .def_readwrite("A", &CPG_Updated_cpp_t::A)
            .def_readwrite("B", &CPG_Updated_cpp_t::B)
            .def_readwrite("y", &CPG_Updated_cpp_t::y)
            .def_readwrite("C", &CPG_Updated_cpp_t::C)
            ;

    py::class_<CPG_Prim_cpp_t>(m, "cpg_prim")
            .def(py::init<>())
            .def_readwrite("W", &CPG_Prim_cpp_t::W)
            .def_readwrite("V", &CPG_Prim_cpp_t::V)
            .def_readwrite("U", &CPG_Prim_cpp_t::U)
            .def_readwrite("X", &CPG_Prim_cpp_t::X)
            ;

    py::class_<CPG_Dual_cpp_t>(m, "cpg_dual")
            .def(py::init<>())
            .def_readwrite("d0", &CPG_Dual_cpp_t::d0)
            .def_readwrite("d1", &CPG_Dual_cpp_t::d1)
            .def_readwrite("d2", &CPG_Dual_cpp_t::d2)
            .def_readwrite("d3", &CPG_Dual_cpp_t::d3)
            ;

    py::class_<CPG_Info_cpp_t>(m, "cpg_info")
            .def(py::init<>())
            .def_readwrite("obj_val", &CPG_Info_cpp_t::obj_val)
            .def_readwrite("iter", &CPG_Info_cpp_t::iter)
            .def_readwrite("status", &CPG_Info_cpp_t::status)
            .def_readwrite("pri_res", &CPG_Info_cpp_t::pri_res)
            .def_readwrite("dua_res", &CPG_Info_cpp_t::dua_res)
            .def_readwrite("time", &CPG_Info_cpp_t::time)
            ;

    py::class_<CPG_Result_cpp_t>(m, "cpg_result")
            .def(py::init<>())
            .def_readwrite("cpg_prim", &CPG_Result_cpp_t::prim)
            .def_readwrite("cpg_dual", &CPG_Result_cpp_t::dual)
            .def_readwrite("cpg_info", &CPG_Result_cpp_t::info)
            ;

    m.def("solve", &solve_cpp);

    m.def("set_solver_default_settings", &cpg_set_solver_default_settings);
    m.def("set_solver_rho", &cpg_set_solver_rho);
    m.def("set_solver_max_iter", &cpg_set_solver_max_iter);
    m.def("set_solver_eps_abs", &cpg_set_solver_eps_abs);
    m.def("set_solver_eps_rel", &cpg_set_solver_eps_rel);
    m.def("set_solver_eps_prim_inf", &cpg_set_solver_eps_prim_inf);
    m.def("set_solver_eps_dual_inf", &cpg_set_solver_eps_dual_inf);
    m.def("set_solver_alpha", &cpg_set_solver_alpha);
    m.def("set_solver_scaled_termination", &cpg_set_solver_scaled_termination);
    m.def("set_solver_check_termination", &cpg_set_solver_check_termination);
    m.def("set_solver_warm_start", &cpg_set_solver_warm_start);

}

import os
from mh_sise.latex_utils import save_table


# Example usage
values_dict = {
    'PROBLEM_FORMULATION_NAME': 'ExampleProblem',
    'n_states': 4,
    'HORIZON_LENGTH': 20,
    'time_cvx_s': 100,
    'time_cvx_m': 200,
    'time_cvx_l': 300,
    'pars_s': 10,
    'pars_m': 20,
    'pars_l': 30,
    'vars_s': 50,
    'vars_m': 60,
    'vars_l': 70,
    'tvars_s': 80,
    'tvars_m': 90,
    'tvars_l': 100,
    'teq_s': 110,
    'teq_m': 120,
    'teq_l': 130,
    'tieq_s': 140,
    'tieq_m': 150,
    'tieq_l': 160,
    'code_size_s': 170,
    'code_size_m': 180,
    'code_size_k': 190,
    'gen_time_s': 200,
    'gen_time_m': 210,
    'gen_time_k': 220,
    'comp_time_s': 230,
    'comp_time_m': 240,
    'comp_time_k': 250,
    'binary_size_s': 260,
    'binary_size_m': 270,
    'binary_size_l': 280,
    'time_cvxgen_s': 290,
    'time_cvxgen_m': 300,
    'time_cvxgen_l': 310,
    'maxit_s': 320,
    'maxit_m': 330,
    'maxit_l': 340,
}

template_path = 'templates/table_results_template.txt'

# Call the function with the path to the template and the values dictionary
save_table(template_path=template_path, values_dict=values_dict, output_dir='results', output_filename='filled_table.tex')
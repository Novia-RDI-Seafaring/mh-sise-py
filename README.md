# Moving-Horizon Simultaneous Input-and-State Estimation (MH-SISE)

This repository implements **Moving-Horizon Simultaneous Input-and-State Estimation (MH-SISE)** using the Python packages `cvxpy` and `cvxpygen`. The MH-SISE algorithm estimates both inputs and states of a dynamic system over a moving horizon, enabling real-time state estimation with optimal accuracy. 

## Method

We consider a discrete-time linear time-invariant (LTI) system of the form:

$$
\begin{align*}
x_{k+1} &= Ax_k + Bu_k + w_k \\
y_k &= Cx_k + v_k
\end{align*}
$$

Where:
- $x_k \in \mathbb{R}^n$ represents the system state at time step $k$,
- $u_k \in \mathbb{R}^m$ is the control input at time step $k$,
- $y_k \in \mathbb{R}^p$ is a vector of measured outputs at time step $k$,
- $w_k \in \mathbb{R}^n$ and $v_k \in \mathbb{R}^p$ are process and measurement noise, respectively,
- $A \in \mathbb{R}^{n \times n}$, $B \in \mathbb{R}^{n \times m}$, and $C \in \mathbb{R}^{p \times n}$ are system matrices describing the dynamics and observation model.

To estimate both the states $x_k$ and the inputs $u_k$ over a moving horizon of length $N$, the package solves the following quadratic constrainted optimization problem:

$$
\begin{aligned}
    &\underset{x, u}{\mathrm{minimize}} \quad u^T R u + \sum_{n=k-N}^{k} v_n^T Q_v^{-1} v_n + \sum_{n=k-N}^{k} w_n^T Q_w^{-1} w_n \\
    &\text{subject to} \\
    &x_{n} = A x_{n-1} + B u_{n-1} + w_n, &n = k-N, \dots, k \\
    &y_n = C x_n + v_n, &n = k-N, \dots, k \\
    &x_0 = \hat{x}_{k-N-1|k-N-1} \\
    &u \in \mathcal{C},
\end{aligned}
$$

where:
- $Q_v \in \mathbb{R}^{p \times p}$, $Q_w \in \mathbb{R}^{n \times n} arecovariande matrices of measurement noise $v_n$, process noise $w_n$, and inputs respectively.
- $R \in \mathbb{R}^{Nm}$ is a design matrix used to app degularization on $u$.
- $\mathcal{C}$ is a convex set.
- $\hat{x}_{k-N-1}$ is an estimatie of the initial state.

This formulation leads to a convex optimization problem, which is solved using the `cvxpy` optimization framework, and code generation for efficient evaluation is handled by `cvxpygen`.

## Installation
### Python version
This project is developed ``Python 3.12.4`` and has not been tested for other versions.

### Install MH-SISE
Clone repository and install `mh_sise`  in **editable** mode using:

```bash
pip install -e .
```

This will install the package and allow you to modify the code without needing to reinstall it. Now you can import `mh_sise` in your Python projects:

```python
from mh_sise import model_utils
```
### Requirementes
To run the code in this project, you'll need to install the following Python packages:

- `numpy` for numerical operations.
- `cvxpy` for convex optimization.
- `cvxpygen` for code ceneration.
- `scipy` for linear algebra and control-related oprations.
- `matplotlib` for plotting and visualization.

Install the dependencies with:

```bash
pip install -r requirements.txt
```


## Example
Define a generic MH-SISE problem for a system with $m$ inputs, $n$ states, and $p$ outputs. Estimation happens on a moving horizon with length $N$.

### Create the optimization problem
```python
from mh_sise.problem import Problem

p = n_outputs
m = n_inputs
n = n_states

problem = Problem(n, m, p, N)
```

#### Assign parameter values
Parameter values are assigned as follows. It is also possible to assign values to a selection of the parameters.

```python
problem.assign_parameter_values(
    Q_v_inv_sqrt = Q_v_inv_sqrt,
    Q_w_inv_sqrt = Q_w_inv_sqrt,
    Q_u_inv_sqrt = Q_u_inv_sqrt,
    A = A,
    B = B,
    C = C,
    x0 = x0,
    y = y
)
```


#### Solve the problem with CVXPY
```python
problem.solve()
```
### Generate and use C-solver code
Here we will step through how to generate C code using `CVXPYGEN` por a problem created as above, load the C solver into Python and solve it. 

#### Generate C code with CVXPYGEN
```python
problem.generate_code(pth, name)
```
where `pth` is the parth to where to save the solver, and `name` is the name you want to give the solver.

#### Load C-solver code in Python
```python
from mh_sise.problem import CProblem
from codegen.<<pth_to_solver>>.cpg_solver import cpg_solve

cproblem = CProblem(pth=pth_to_solver)
```

#### Assign parameter values
Parameter values are assigned to the  C solver as follows. It is also possible to assign values to a selection of the parameters.

```python
cproblem.assign_parameter_values(
    Q_v_inv_sqrt = Q_v_inv_sqrt,
    Q_w_inv_sqrt = Q_w_inv_sqrt,
    Q_u_inv_sqrt = Q_u_inv_sqrt,
    A = A,
    B = B,
    C = C,
    x0 = x0,
    y = y
)
```
#### Solve the problem with the C-solver
```python
cproblem.solve(cpg_solve)
```
---
A complete example is provided in:
- MH-SISE Example, [example.ipynb](https://github.com/Novia-RDI-Seafaring/mh-sise-py/blob/main/examples/example.ipynb)

## Main contributors
- **Mikael Manngård**, (Novia UAS). Contributed with the MH-SISE formulation.
    - CRediT: *Conceptualization*, *Methodology*, *Software*, *Formal analysis*, *Supervision*. 
- **Dimitrios Bouzoulas** (Novia UAS). Contributed with the code-generation work with `cvxpygen`.
    - CRediT: *Software*, *Validation*.
- **Urho Hakonen** (Aalto University). This work is a continuation and implementation of ([Hakonen, 2023](https://www.finna.fi/Record/aaltodoc.123456789_123180?sid=3456825094&lng=en-gb)).
    - CRediT: *Methodology*, *Validation*. 
- **Jan Kronqvist** (KTH).
    - CRediT: *Supervision*. 

## Acknowledgements
This work was done as part of the Business Finland funded project [Virtual Sea Trial](https://virtualseatrial.fi).
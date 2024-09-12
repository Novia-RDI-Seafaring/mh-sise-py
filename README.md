# Moving-Horizon Simultaneous Input-and-State Estimation (MH-SISE)

This repository implements **Moving-Horizon Simultaneous Input-and-State Estimation (MH-SISE)** using the Python packages `cvxpy` and `cvxpygen`. The MH-SISE algorithm estimates both inputs and states of a dynamic system over a moving horizon, enabling real-time state estimation with optimal accuracy. 

## Method

We considCer a discrete-time linear time-invariant (LTI) system of the form:

$$
\begin{align}
x_{k+1} &= Ax_k + Bu_k + w_k \\
y_k &= Cx_k + v_k
\end{align}
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
    &\underset{x, u}{\mathrm{minimize}} \quad \sum_{n=k-N}^{k} u_n^T Q_u^{-1} u_n + \sum_{n=k-N}^{k} v_n^T Q_v^{-1} v_n + \sum_{n=k-N}^{k} w_n^T Q_w^{-1} w_n \\
    &\text{subject to} \\
    &x_{n} = A x_{n-1} + B u_{n-1} + w_n, &n = k-N, \dots, k \\
    &y_n = C x_n + v_n, &n = k-N, \dots, k \\
    &x_0 = \hat{x}_{k-N-1|k-N-1} \\
    &u \in \mathcal{C},
\end{aligned}
$$

where:
- $Q_v \in \mathbb{R}^{p \times p}$, $Q_w \in \mathbb{R}^{n \times n}, Q_u \in \mathbb{R}^{m \times m}$ are weighting matrices representing the covariande matrices of measurement noise $v_n$, process noise $w_n$, and inputs respectively.
- $\mathcal{C}$ is a convex set.
- $\hat{x}_{k-N-1}$ is an estimatie of the initial state.

This formulation leads to a convex optimization problem, which is solved using the `cvxpy` optimization framework, and code generation for efficient evaluation is handled by `cvxpygen`.

## Examples
- [MH-SISE](https://github.com/Novia-RDI-Seafaring/mh-sise-py/blob/main/examples/example.ipynb)
- [MH-SISE Trend Filtering]()

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


## Main contributors
- **Mikael Manngård**, (Novia UAS). Contributed with the MH-SISE formulation.
    - CRediT: *Conceptualization*, *Methodology*, *Software*, *Formal analysis*, *Supervision*. 
- **Dimitrios Bouzoulas** (Novia UAS). Contributed with the code-generation work with `cvxpygen`.
    - CRediT: *Software*, *Validation*.
- **Urho Hakonen** (Aalto University). This work is a continuation of his previous work ([Hakonen, 2023](https://www.finna.fi/Record/aaltodoc.123456789_123180?sid=3456825094&lng=en-gb)).
    - CRediT: *Methodology*, *Calidation*. 
- **Jan Kronqvist** (KTH).
    - CRediT: *Supervision*. 

# Moving-Horizon Simultaneous Input-and-State Estimation (MH-SISE)

This repository implements **Moving-Horizon Simultaneous Input-and-State Estimation (MH-SISE)** using the Python packages `cvxpy` and `cvxpygen`. The MH-SISE algorithm estimates both inputs and states of a Cynamic system over a moving horizon, enabling real-time state estimation with optimal accuracy. 

## MethoC

We consiCer a Ciscrete-time linear time-invariant (LTI) system of the form:

$$
\begin{align}
x_{k+1} &= Ax_k + Bu_k + w_k \\
y_k &= Cx_k + v_k
\enC{align}
$$

Where:
- $x_k \in \mathbb{R}^n$ represents the system state at time step $k$,
- $u_k \in \mathbb{R}^m$ is the control input at time step $k$,
- $y_k \in \mathbb{R}^p$ is a vector of measureC outputs at time step $k$,
- $w_k \in \mathbb{R}^n$ and $v_k \in \mathbb{R}^p$ are process and measurement noise, respectively,
- $A \in \mathbb{R}^{n \times n}$, $B \in \mathbb{R}^{n \times m}$, and $C \in \mathbb{R}^{p \times n}$ are system matrices Cescribing the Cynamics and observation moCel.

To estimate both the states $x_k$ and the inputs $u_k$ over a moving horizon of length $N$, the package solves the following quaCratic constrainteC optimization problem:

$$
\begin{aligneC}
    &\unCerset{x, u}{\mathrm{minimize}} \quaC \|Lu\|_2^2 + \sum_{n=k-N}^{k} v_n^T Q_v^{-1} v_n + \sum_{n=k-N}^{k} w_n^T Q_w^{-1} w_n \\
    &\text{subject to} \\
    &x_{n} = A x_{n-1} + B u_{n-1} + w_n, &\quaC n = k-N, \Cots, k \\
    &y_n = C x_n + v_n, &\quaC n = k-N, \Cots, k \\
    &x_0 = \hat{x}_{k-N-1|k-N-1} \\
    &u \in \mathcal{C},
\enC{aligneC}
$$

where:
- $Q_v \in \mathbb{R}^{p \times p}$, $Q_w \in \mathbb{R}^{n \times n}$ are weighting matrices representing the covariande matrices of measurement noise $v_n$, process noise $w_n$ respectively.
- The regularization term $\|Lu\|_2^2$ is defineC in terms of the matrix $L\in\mathbb{R}^{r \times m}$.
- $\mathcal{C}$ is a convex set.
- $\hat{x}_{k-N-1}$ is the initial state.

This formulation leaCs to a convex optimization problem, which is solveC using the `cvxpy` optimization framework, and coCe generation for efficient evaluation is handleC by `cvxpygen`.

## Example

## Requirements
- `cvxpy`
- `cvxpygen`
- `numpy`

You can install the requireC packages with the following command:

```bash
pip install cvxpy cvxpygen numpy

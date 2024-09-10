# Moving-Horizon Simultaneous Input-and-State Estimation (MH-SISE)

This repository implements **Moving-Horizon Simultaneous Input-and-State Estimation (MH-SISE)** using the Python packages `cvxpy` and `cvxpygen`. The MH-SISE algorithm estimates both inputs and states of a dynamic system over a moving horizon, enabling real-time state estimation with optimal accuracy. 

## Method

We consider a discrete-time linear time-invariant (LTI) system of the form:

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
    &\underset{x, u}{\mathrm{minimize}} \quad L(u) + \sum_{n=k-N}^{k} v_n^T Q_v^{-1} v_n + \sum_{n=k-N}^{k} w_n^T Q_w^{-1} w_n \\
    &\text{subject to} \\
    &x_{n} = A x_{n-1} + B u_{n-1} + w_n, &\quad n = k-N, \dots, k \\
    &y_n = C x_n + v_n, &\quad n = k-N, \dots, k \\
    &G u \leq b
\end{aligned}
$$

Where:
- $Q_v \in \mathbb{R}^{p \times p}$, $Q_w \in \mathbb{R}^{n \times n}$ are weighting matrices representing the covariance matrices of measurement noise $v_n$, process noise $w_n$ respectively.
- $L(u)$ is a quadratic regularization term with respect to $u$ and sohuld be on the form on the form $L(u) = u^TQ_u u^T + c^Tu$, where $Q_u \in \mathbb{R}^{m \times m}$ is a symmetric positive semidefinite matrix (ensuring convexity) and $c \in \mathbb{R}^m$ is a vector.
- $G \in \mathbb{R}^{r \times n}$.
- $c \in \mathbb{R}^{r}$
- $\mathcal{C}$ is a convex set.

This formulation leads to a convex optimization problem, which is solved using the `cvxpy` optimization framework, and code generation for efficient evaluation is handled by `cvxpygen`.

## Example

## Requirements
- `cvxpy`
- `cvxpygen`
- `numpy`

You can install the required packages with the following command:

```bash
pip install cvxpy cvxpygen numpy

# Comprehensive Sliding Mode Control (SMC) Knowledge Base

A structured, in-depth knowledge base on Sliding Mode Control, covering fundamentals, controller design, parameter tuning, and stability analysis. Curated from high-quality textbooks, academic papers, and practical implementations.

**Target Audience:** Graduate students, researchers, and control engineers.

---

## 📖 Table of Contents

1. [Fundamentals of Control Theory](#1-fundamentals-of-control-theory)
2. [Sliding Mode Controller Design](#2-sliding-mode-controller-design)
3. [Controller Parameter Tuning](#3-controller-parameter-tuning)
4. [Stability Analysis](#4-stability-analysis)
5. [Practical Implementation in MATLAB/Simulink](#5-practical-implementation-in-matlabsimulink)
6. [Advanced Topics](#6-advanced-topics)
7. [Resources & References](#7-resources--references)

---

## 1. Fundamentals of Control Theory

### 1.1 What is Sliding Mode Control?

Sliding Mode Control (SMC) is a **nonlinear control technique** known for its **robustness**. Its core idea is to force the system's state trajectory onto a pre-defined **sliding surface** (or manifold) in the state space and maintain it on that surface thereafter.

- **Sliding Surface (`s(x) = 0`)**: A hyperplane representing desired system dynamics (e.g., `s = λe + ė`). Once on the surface, the system's behavior is governed by the surface equation and becomes insensitive to certain uncertainties and disturbances.
- **Reaching Phase**: The trajectory from the initial state to the sliding surface.
- **Sliding Mode**: The motion *on* the sliding surface towards the equilibrium.

### 1.2 Prerequisite Concepts

- **State-Space Representation**: `˙x = Ax + Bu` (linear) or `˙x = f(x, u)` (nonlinear).
- **Lyapunov Stability**: A system is stable if there exists a positive definite function `V(x)` (like energy) whose derivative `˙V(x)` is negative semi-definite. This is the primary tool for proving SMC stability.
- **Robustness**: A controller's ability to maintain performance despite model uncertainties and external disturbances.

---

## 2. Sliding Mode Controller Design

### 2.1 Design Procedure

#### Step 1: Define the Sliding Surface
The surface is a function of the tracking error. For a second-order system:
s = λe + ė

text
- `e = x - x_d`: Tracking error.
- `λ > 0`: A design parameter that determines the convergence rate on the surface.

For a system with state vector `x`, the surface is often `s = Cᵀx`, where `C` is the sliding coefficient matrix.

#### Step 2: Derive the Control Law
The control law has two components:
u = u_eq + u_sw

text
- **Equivalent Control (`u_eq`)**: The control input that would maintain the system on the sliding surface (`˙s = 0`) in the absence of uncertainties. It is found by solving `˙s = 0` for `u`.
- **Switching Control (`u_sw`)**: A discontinuous term that drives the system to the surface and rejects disturbances. The most basic form is `u_sw = -K * sign(s)`, where `K > 0`.

### 2.2 Example for a Second-Order System

Consider a system: `m¨x = u + d(t)`, where `|d(t)| < D` is a bounded disturbance.

1.  **Define State Variables**: `x₁ = x`, `x₂ = ˙x`. State-space: `˙x₁ = x₂`, `˙x₂ = (1/m)u + d(t)`.
2.  **Define Sliding Surface**: `s = λe + ė`, with `e = x₁ - x_d`. Assuming `x_d` is constant, `˙e = x₂`. So, `s = λ(x₁ - x_d) + x₂`.
3.  **Derive Control Law**:
    - Find `˙s`: `˙s = λx₂ + (1/m)u + d(t)`.
    - Design `u` to enforce `˙s = -η sign(s)` (a common reaching law):
        `u = -m(λx₂ + η sign(s))`
    - To fully account for the disturbance, the gain must be large enough: `u = -m(λx₂ + (η + D) sign(s))`.

---

## 3. Controller Parameter Tuning

### 3.1 The Chattering Problem
The discontinuous `sign(s)` function causes **chattering**—high-frequency, finite-amplitude oscillations around the sliding surface. This is undesirable as it can excite unmodeled dynamics and damage actuators.

### 3.2 Mitigation: The Boundary Layer Method
Replace the discontinuous `sign(s)` function with a continuous approximation inside a "boundary layer" `|s| < Φ`.

- **Saturation Function**:
text
          { sign(s),  if |s/Φ| > 1
sat(s/Φ) = {
{ s/Φ, if |s/Φ| ≤ 1

text
The control law becomes: `u_sw = -K * sat(s/Φ)`.

- **Tuning Parameters**:
- **Switching Gain (`K`)**: Must be large enough to overcome the total disturbance. A higher `K` provides more robustness but increases control effort and can lead to chattering.
- **Boundary Layer Thickness (`Φ`)**: A larger `Φ` reduces chattering but introduces a **steady-state error** within the layer. A smaller `Φ` improves accuracy but may not eliminate chattering. Tuning is a trade-off between precision and smoothness.

### 3.3 Tuning Procedure
1.  Start with a small `Φ` and a conservative `K` based on the estimated disturbance bounds.
2.  Simulate the system with disturbances and uncertainties.
3.  If chattering is excessive, increase `Φ` slightly.
4.  If the controller is not robust enough (diverges from the surface), increase `K`.
5.  Iterate until a satisfactory balance between performance and robustness is achieved.

---

## 4. Stability Analysis

### 4.1 Lyapunov's Direct Method
Stability of the SMC is proven using a Lyapunov function candidate.

1.  **Choose a Lyapunov Function**: The most common choice is `V = (1/2)s²`. This is always positive definite in `s`.
2.  **Analyze its Derivative**:
  ```
  ˙V = s * ˙s
  ```
3.  **Ensure Negative Definiteness**: The control law must be designed to guarantee `˙V < 0` for `s ≠ 0`. This is the **reachability condition**.
  - For the example in 2.2: `˙V = s * ( -η sign(s) + d(t) ) = -η |s| + s*d(t) ≤ -η |s| + D|s| = -|s|(η - D)`.
  - Therefore, if we choose `η > D`, then `˙V < 0` for `s ≠ 0`. This proves the state will reach the surface `s=0` in finite time and remain there.

### 4.2 Reachability Conditions
The condition `˙V < 0` can be decomposed into:
s * ˙s < 0

text
This means the state velocity always points towards the sliding surface, "reaching" it from any initial condition.

---

## 5. Practical Implementation in MATLAB/Simulink

MathWorks provides dedicated blocks for implementing SMC, simplifying the design process.

### 5.1 Sliding Mode Controller (Reaching Law) Block
- **Use Case**: General nonlinear systems of the form `˙x = f(x) + g(x)u`.
- **Your Responsibility**: Design the sliding surface `C` (i.e., the matrix `C` in `s = Cᵀx`).
- **Block's Role**: Computes the control law `u` based on the reaching law you specify.

### 5.2 Linear Sliding Mode Controller (State Feedback) Block
- **Use Case**: Uncertain linear systems.
- **Major Advantage**: Can **automatically design** the sliding surface `S` (in `s = Sx`) using:
    - **Pole Placement**: Assign desired eigenvalues for the sliding mode dynamics.
    - **Quadratic Minimization**: Minimize a cost function `J = ∫ xᵀQx dt`, similar to LQR design.

### 5.3 Key Configuration Parameters
- **Reaching Law**: Defines how the system converges to the sliding surface.
    - **Constant Rate**: `˙s = -η sign(s)`. Simple, but can cause chattering.
    - **Exponential**: `˙s = -η sign(s) - K s`. Faster convergence.
    - **Power Rate**: `˙s = -η |s|^α sign(s)`. Fast when far, soft when near, reducing chattering.
- **Boundary Layer**: As discussed in Section 3.2, you can choose `sign`, `sat`, `tanh`, or `relay` functions.

---

## 6. Advanced Topics

- **Higher-Order Sliding Modes (HOSM)**: Acts on higher-order derivatives of `s` (e.g., `s = ˙s = 0`). The **Super-Twisting Algorithm (STA)** is a popular 2nd-order SMC that eliminates chattering without a boundary layer and provides even higher accuracy.
- **Adaptive SMC**: The gain `K` or boundary layer `Φ` is adjusted online to adapt to changing uncertainty bounds, improving performance without overly conservative designs.
- **Disturbance Observer-Based SMC**: Uses a separate observer to estimate and cancel disturbances in the `u_eq` term, allowing for smaller switching gains and reduced chattering.

---

## 7. Resources & References

### 7.1 Primary GitHub Repositories
- **[ALEX-SVKIN/Sliding-mode-control](https://github.com/ALEX-SVKIN/Sliding-mode-control)**: The core resource for this knowledge base. Contains detailed theory, derivations, and MATLAB/Simulink examples.
- **[s-Hua/Awesome-Sliding-Mode-Control](https://github.com/s-Hua/Awesome-Sliding-Mode-Control)**: A curated list of books, papers, and code for deeper exploration.

### 7.2 Recommended Textbooks & Papers
- **Utkin, V., "Sliding Mode Control in Electro-Mechanical Systems"** (Classic Textbook)
- **Slotine, J.-J. E., and Li, W., "Applied Nonlinear Control"** (Excellent Chapter on SMC)
- **MathWorks Documentation: "Sliding Mode Control"** (For practical implementation details)

### 7.3 Citation
When using this knowledge base, please cite the primary sources listed above.

---

*This knowledge base was curated for educational purposes in the field of robust control systems.*

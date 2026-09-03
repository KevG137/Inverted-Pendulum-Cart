# State-Space Control & Digital Twin Simulation of an Inverted Pendulum-Cart System
[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/KevG137/Inverted-Pendulum-Cart/HEAD?urlpath=lab%2Ftree%2Fnotebooks%2FInverted_Pendulum_and_Cart.ipynb)

**Author:** Kevin Goguen  
**Project Focus:** State-Space Modeling, LQR Synthesis, Dynamic Simulation & Parameter Robustness  

---

## Project Overview & Engineering Specifications

### Objective
This project implements a continuous state-space modeling framework and an optimal control loop to actively stabilize an inherently unstable, non-linear inverted pendulum-cart system. 

### Industrial Relevance
The state-space architectures and optimization methodologies executed here serve as the baseline tracking framework for modern multi-variable industrial applications, including:
* **Robotics:** Dynamic balancing loops for bipedal and humanoid mobility platforms.
* **Material Handling:** Active anti-sway crane control systems in automated shipping yards.

### Core Technology Stack
* **Languages & Core Libraries:** Python (`numpy`, `scipy`, `control`)
* **Simulation Visualizations:** `ipycanvas`, `matplotlib`
* **Control Systems Architecture:** Continuous State-Space Modeling, Linear-Quadratic Regulator (LQR) Optimal Control Synthesis
* **Numerical Methods:** Self-implemented 4th-Order Runge-Kutta (RK4) integration engine

---

## System Modeling & Control Loop Design

### 1. State-Space Formulation
The system monitors linear track displacement ($x$), linear cart velocity ($\dot{x}$), pendulum angular deviation from vertical ($\theta$), and angular velocity ($\dot{\theta}$):

$$
\mathbf{x} = \begin{bmatrix} \theta & \dot{\theta} & x & \dot{x} \end{bmatrix}^T
$$

The highly coupled, non-linear system equations of motion derived via Lagrangian mechanics are linearized about the unstable upper equilibrium point ($\theta \approx 0, \dot{\theta} \approx 0$) to construct the continuous state-space matrices ($A$ and $B$):

$$
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
$$

$$
\begin{bmatrix}
\dot{\theta} \\
\ddot{\theta} \\
\dot{x} \\
\ddot{x}
\end{bmatrix}
\approx
\begin{bmatrix}
0 & 1 & 0 & 0 \\
\left(1+\frac{m}{M}\right)\frac{g}{l} & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
-\frac{m}{M}g & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
\theta \\
\dot{\theta} \\
x \\
\dot{x}
\end{bmatrix}
+
\begin{bmatrix}
0 \\
-\frac{1}{Ml} \\
0 \\
\frac{1}{M}
\end{bmatrix}
F
$$

### 2. Optimal Feedback Synthesis (LQR)
To track system deviations and minimize structural error, an optimal feedback gain matrix $K$ is derived via negative state feedback ($\mathbf{u} = -K\mathbf{x}$). The gain minimizes the infinite-horizon continuous cost function $J$:

$$
J = \int_{0}^{\infty} \left( \mathbf{x}^T Q \mathbf{x} + \mathbf{u}^T R \mathbf{u} \right) dt
$$

The unique positive-definite solution matrix $S$ is solved via the continuous **Algebraic Riccati Equation (ARE)** using the `python-control` library:

$$
S A + A^T S - S B R^{-1} B^T S + Q = 0 \implies K = R^{-1} B^T S
$$

### 3. Controller Tuning & Physical Constraints
* **Design Constraint:** The cart must remain within a physical rail safety boundary of **$\pm 1.0\text{ meter}$** during tracking and disturbance rejection.
* **Tuning Matrix Rationale:** To satisfy this constraint, the state penalty matrix $Q$ is heavily biased on the linear displacement diagonal element ($Q_{33} = 50$). This penalizes spatial travel tightly while balancing standard actuator effort penalties ($R = 1$):

$$
Q = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 50 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}, \quad R = 1
$$

---

## Non-Linear Plant Simulation (Digital Twin)

Because analytical solutions to the true coupled system equations are non-existent, this project implements a **4th-Order Runge-Kutta (RK4) numerical integration engine** to step the physical plant forward in time ($h = \Delta t$). 

This architecture constructs an authentic "Digital Twin" pipeline: the optimal LQR controller—designed using a *simplified, linear approximation*—is forced to regulate the *true, non-linear, coupled* physical equations of motion.

$$
\mathbf{x}(t + h) \approx \mathbf{x}(t) + \frac{h}{6} \left( \mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4 \right)
$$

---

## Deep Dive: Theoretical Background & Mathematical Derivations

### Non-Linear Plant Physics
Applying Lagrangian mechanics to the cart-pendulum system yields the following exact non-linear equations of motion:

$$
\ddot{\theta} = \frac{g \sin\theta - \cos\theta \left( \frac{F + m l \dot{\theta}^2 \sin\theta}{M + m} \right)}{l \left( 1 - \frac{m \cos^2\theta}{M + m} \right)}
$$

$$
\ddot{x} = \frac{F + m l \dot{\theta}^2 \sin\theta - m l \ddot{\theta} \cos\theta}{M + m}
$$

Assuming small angle approximations ($\theta \approx 0, \dot{\theta} \approx 0$) for linearization yields:

$$
\ddot{\theta} \approx \left( 1 + \frac{m}{M} \right) \frac{g}{l} \theta - \frac{F}{M l}
$$

$$
\ddot{x} \approx -\frac{m}{M} g \theta + \frac{F}{M}
$$

### Derivation of the Matrix State Equations
By mapping the linearized scalar differentials back to state vector definitions, the system dynamics are isolated into explicit matrix form:

$$
\dot{\mathbf{x}} =
\begin{bmatrix}
\dot{\theta} \\
\ddot{\theta} \\
\dot{x} \\
\ddot{x}
\end{bmatrix}
\approx
\begin{bmatrix}
\dot{\theta} \\
\left( 1 + \frac{m}{M} \right) \frac{g}{l} \theta - \frac{F}{M l} \\
\dot{x} \\
-\frac{m}{M} g \theta + \frac{F}{M}
\end{bmatrix}
= A\mathbf{x} + B\mathbf{u}
$$

### Cost Optimization via Bellman's Principle of Optimality
Assuming the optimal cost-to-go function $J^*$ is quadratic in state ($J^* = \mathbf{x}^T S \mathbf{x}$), Bellman's tracking partials are minimized with respect to control vector $\mathbf{u}$:

$$
\frac{\partial}{\partial \mathbf{u}} \left[ \mathbf{x}^T Q \mathbf{x} + \mathbf{u}^T R \mathbf{u} + \frac{\partial J^*}{\partial \mathbf{x}} \left( A\mathbf{x} + B\mathbf{u} - \dot{\mathbf{x}} \right) \right] = 0
$$

$$
\frac{\partial J^*}{\partial \mathbf{x}} = 2\mathbf{x}^T S \implies 2\mathbf{u}^T R + 2\mathbf{x}^T S B = 0 \implies \mathbf{u} = -R^{-1} B^T S \mathbf{x}
$$

### RK4 Derivative Vector Stepping Function
The time-evolution function $\frac{\partial \mathbf{x}}{\partial t} \equiv \mathbf{f}$ updates intermediate vector fields $\mathbf{k}_n$ across step width $h$:

$$
\mathbf{k}_1 = \mathbf{f}(\theta_0, \dot{\theta}_0, x_0, \dot{x}_0)
$$

$$
\mathbf{k}_2 = \mathbf{f}\left( \theta_0 + \frac{h}{2}k_{1,\theta}, \dot{\theta}_0 + \frac{h}{2}k_{1,\dot{\theta}}, x_0 + \frac{h}{2}k_{1,x}, \dot{x}_0 + \frac{h}{2}k_{1,\dot{x}} \right)
$$

$$
\mathbf{k}_3 = \mathbf{f}\left( \theta_0 + \frac{h}{2}k_{2,\theta}, \dot{\theta}_0 + \frac{h}{2}k_{2,\dot{\theta}}, x_0 + \frac{h}{2}k_{2,x}, \dot{x}_0 + \frac{h}{2}k_{2,\dot{x}} \right)
$$

$$
\mathbf{k}_4 = \mathbf{f}\left( \theta_0 + h k_{3,\theta}, \dot{\theta}_0 + h k_{3,\dot{\theta}}, x_0 + h k_{3,x}, \dot{x}_0 + h k_{3,\dot{x}} \right)
$$

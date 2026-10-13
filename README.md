# ROV State Observability & Observer Design

## Project Overview
This repository provides a mathematical framework for analyzing the observability of a Remotely Operated Vehicle (ROV) and designing a continuous-time Luenberger observer. The system estimates the full 6-dimensional state vector of the ROV using only partial measurements (position and orientation) while converging to the true state under specific damping and natural frequency constraints.

## Mathematical Architecture

### System and Measurement Model
The ROV's dynamics are modeled as a linear time-invariant (LTI) system. Since only the spatial positions and yaw angle $(x, y, \psi)$ are measured, the measurement matrix $C \in \mathbb{R}^{3 \times 6}$ extracts only the first three states:
$$\dot{X} = A X + B U$$
$$Y = C X$$

### Luenberger Observer Dynamics
To estimate the unmeasured velocities $(v_x, v_y, r)$, a state estimator (Observer) is designed. The estimated state $\hat{X}$ evolves according to the real system dynamics, corrected by an error term weighted by the observer gain matrix $K_o \in \mathbb{R}^{6 \times 3}$:
$$\dot{\hat{X}} = A \hat{X} + B U + K_o (Y - \hat{Y})$$
Where $\hat{Y} = C \hat{X}$ is the estimated measurement.

### Pole Placement (Gain Calculation)
The observer gain matrix $K_o$ is determined analytically by forcing the error dynamics to match a desired second-order characteristic equation:
$$s^2 + 2\xi\omega_n s + \omega_n^2$$
For this implementation, the system is tuned for critical damping $(\xi = 1)$ and a natural frequency of $\omega_n = 1 \text{ rad/s}$ to ensure rapid and oscillation-free convergence of the estimation error.

## Simulation & Performance Metrics

The simulation evaluates the system over a 25-second trajectory driven by minimum energy control inputs calculated via the Controllability Gramian.

**Key Results:**
*   **State Convergence:** The observer successfully drives the estimation error to near-zero, validating the calculated $K_o$ gains.
*   **Final Positioning Error:** $0.00183$ meters.
*   **Final Estimation Error Norm:** $8.6257 \times 10^{-10}$.

*- State Evolution*
<img width="1902" height="968" alt="Fig1" src="https://github.com/user-attachments/assets/a758e92d-f00e-4368-a478-adcb323faa58" />

*- State Estimation Error*
<img width="1902" height="968" alt="Fig2" src="https://github.com/user-attachments/assets/ac77ef6e-e46f-41e0-91c6-f30a33f7000a" />


*- Control Inputs*
<img width="700" height="525" alt="Fig3" src="https://github.com/user-attachments/assets/9437a3d2-c54b-48b5-a094-be5bcfb7cb60" />


*- 2D Trajectory*
<img width="700" height="525" alt="Fig4" src="https://github.com/user-attachments/assets/24901a2d-97e9-46a7-bc2c-bff445968cbf" />


## How to Run

**Prerequisites:**
- MATLAB (Symbolic Math Toolbox required for Gramian integration)

**Execution:**
1. Clone the repository.
2. Open `lab3_Sol.m` in MATLAB.
3. Run the script to execute the observer simulation and generate the performance plots.

---
*Developed by Hussein Elnaggar.*

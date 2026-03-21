# 6DOF Quadrotor Controller: INDI + SMC + Differential Flatness
**MATLAB / Simulink Implementation**

---

## Overview

Hierarchical flight controller for aggressive 6DOF quadrotor trajectory tracking.
Designed to achieve robustness against model uncertainty and external disturbances
during high-speed, large-angle maneuvers.

Developed as an independent undergraduate research project at Northeastern University
(Advisor: Prof. Rifat Sipahi, 2024–2025).

---

## Controller Architecture

```
Reference Trajectory (pos, vel, acc, jerk, snap)
              ↓
   [ PD + Acc Feedforward ]    error → acceleration command
              ↓
   [ INDI Outer Loop ]         thrust vector + desired attitude
              ↓
   [ SMC Inner Loop ]          roll / pitch / yaw torques
     + Disturbance Observer    (each axis independently)
     + DF Feedforward          (jerk/snap → angular rate & acceleration)
              ↓
   [ Motor Mixer ]             individual rotor commands
```

**Design rationale:**
- **INDI** implicitly compensates for external forces and model errors
  using measured acceleration — no explicit disturbance model required
- **SMC** guarantees attitude tracking robustness with Lyapunov-proven stability
- **DF feedforward** preemptively reduces tracking error during aggressive maneuvers
  by feeding desired angular rates and accelerations from jerk/snap

---

## How to Use

### Requirements
- MATLAB (tested on R2024a)
- Simulink
- Symbolic Math Toolbox (for trajectory generation)

### Workflow Overview

```
1. Parameter.m          → Initialize gains and physical parameters
2. traj_generator.m     → Generate C4 reference trajectory function files
3. INDI_POS_SMC_Att.slx → Configure solver/timestep, run simulation
4. (Simulink Scope)     → Real-time visualization during simulation
5. excel_export.m       → Export simulation data to Excel
6. ref_traj_plotter.m   → Visualize reference trajectory from Excel
7. result_plotter.m     → Compare actual vs reference (position, velocity, acceleration)
```

---

### Step 1 — Initialize Parameters

Run `Parameter.m` to load all gains and physical parameters into the
MATLAB workspace before opening the Simulink model.

```matlab
run('Parameter.m')
```

Key parameters defined here:

| Variable | Value | Description |
|----------|-------|-------------|
| `m` | 0.8 | Mass [kg] |
| `l` | 0.165 | Arm length [m] |
| `Jx`, `Jy`, `Jz` | 0.005, 0.005, 0.009 | Moments of inertia [kg·m²] |
| `g` | 9.8067 | Gravitational acceleration [m/s²] |
| `K1` | 0.01 | Linear drag coefficient [N/(m/s)] |
| `K2` | 0 | Quadratic drag coefficient (ignored) |
| `Tmax` | 4 | Maximum thrust per rotor [N] |
| `Qmax` | 0.05 | Maximum reaction torque per rotor [N·m] |
| `Pmax`, `Pmin` | 2000, 1000 | PWM command range [μs] |
| `pos_gain` | diag([4.14, 4.14, 3.105]) | Position error gain Kp [1/s²] |
| `vel_gain` | diag([2.34, 2.34, 1.77]) | Velocity error gain Kv [1/s] |
| `acc_gain` | diag([0.5, 0.5, 0.3]) | Acceleration error gain Ka (dimensionless) |
| `a3`, `k5`, `k6` | 32.764, 0.0388, 0.0219 | SMC roll/pitch: pole, switching gain, damping gain |
| `a4`, `k7`, `k8` | 13.82, 0.3147, 0.1244 | SMC yaw: pole, switching gain, damping gain |

### Step 2 — Define Reference Trajectory

Open `traj_generator.m` and define your trajectory symbolically.

```matlab
syms t real
s = t * pi / 20;

x   = 0.5 * cos(s);
y   = 0.5 * sin(s) * cos(s);
z   = 3 - 2 * cos(s);
psi = t * 0;   % Must be a function of t to avoid dimension mismatch
               % Use t*0 for constant yaw = 0
```

Any **C⁴ (four-times continuously differentiable)** trajectory
expressible as a symbolic function of `t` can be used.
The trajectory must be differentiable up to the 4th order (snap),
as the DF feedforward requires jerk and snap to compute
desired angular rates and accelerations for the SMC inner loop.

The script automatically computes derivatives up to snap (4th order) and
saves them as MATLAB function files (`ref_pos.m`, `ref_vel.m`, `ref_jerk.m`, `ref_snap.m`).

> **Note:** Piecewise-defined trajectories require careful handling at
> discontinuous points, where higher-order derivatives may not exist.

Run the script:
```matlab
run('traj_generator.m')
```

### Step 3 — Run Simulation

Open and run the Simulink model:

```matlab
open('INDI_POS_SMC_Att.slx')
```

Configure in Simulink before running:
- **Solver**: fixed-step (default: `ode4`)
- **Step size**: desired step size (default: `0.001` s)
- **Stop time**: desired simulation duration

Run the simulation. Use **Scope** blocks for real-time monitoring.

### Step 4 — Export Data

Run `excel_export.m` after simulation to save results:

```matlab
run('excel_export.m')
```

Exports position, velocity, and acceleration
(actual + reference) to `simulation_result_data.xlsx`.

### Step 5 — Visualize Results

**Reference trajectory only:**
```matlab
run('ref_traj_plotter.m')
```

**Actual vs Reference comparison:**
```matlab
run('result_plotter.m')
```

---

## Modeling Assumptions

| Assumption | Description |
|------------|-------------|
| Ideal actuators | Motor dynamics and delays are not modeled. Rotor thrust and torque respond instantaneously to commands. |
| Rigid body | Quadrotor is treated as a rigid body. Structural flexibility is ignored. |
| No aerodynamic drag | Blade flapping and rotor drag effects are excluded (K₁ = 0.01, K₂ = 0 in this model). |
| Known parameters | Mass, inertia, and motor constants are assumed to be exactly known. |
| No sensor noise | Ideal state measurement is assumed. No IMU noise or filtering. |

> These assumptions simplify the initial controller design and allow
> focus on the core control architecture. Motor dynamics, sensor noise,
> and aerodynamic effects would need to be incorporated for
> real hardware deployment.

---

## Key Technical Details

### INDI (Outer Loop)
Uses incremental control based on measured acceleration.
External forces are implicitly compensated without explicit modeling.

```
F_des = F_meas + m · (a_cmd − a_meas)
```

### SMC Attitude Controller (Inner Loop)
Lyapunov-proven stability for each axis independently.
Saturation function replaces sign function to suppress chattering.

```
s = λe + ė
τ = k·sat(s/ε) + feedforward terms
V̇ = s·ṡ ≤ 0  (proven for roll, pitch, yaw)
```

### Differential Flatness Feedforward
Desired angular rates and accelerations are derived analytically
from jerk and snap of the reference trajectory.
These are fed as feedforward terms into the SMC to proactively
reduce attitude tracking error.

### Disturbance Observer
First-order observer on each rotational axis:
```
d_obs = z_in + k_ob · ω
ż_in  = −k_ob · d_obs − k_ob · f(τ, ω)
τ_new = τ_orig + d_obs · (J / control_coefficient)
```

---

## Repository Structure

```
├── INDI_POS_SMC_Att.slx            # Main Simulink model
├── Parameter.m                     # Parameters and gains
├── traj_generator.m                # Trajectory symbolic definition
├── ref_pos.m / ref_vel.m / ...     # Auto-generated trajectory functions
├── excel_export.m                  # Export simulation data to Excel
├── ref_traj_plotter.m              # Visualize reference trajectory
├── result_plotter.m                # Actual vs reference visualization
├── results/
│   ├── tracking_result.png
│   └── attitude_response.png
├── docs/
│   └── controller_Draft.pdf       # Full mathematical derivation
└── README.md
```

---

## Technical Report

Full mathematical derivation in [`docs/controller_Draft.pdf`](docs/controller_Draft.pdf), including:
- Complete 6DOF rigid body dynamics
- INDI control law and stability analysis
- Lyapunov stability proof for roll, pitch, and yaw SMC
- Differential Flatness feedforward derivation
- Disturbance observer design

---

## Quadrotor Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Mass (m) | 0.8 | kg |
| Arm length (l) | 0.165 | m |
| Jx | 0.005 | kg·m² |
| Jy | 0.005 | kg·m² |
| Jz | 0.009 | kg·m² |
| Tmax | 4 | N |
| Qmax | 0.05 | N·m |

---

## References

[1] Tal, E., & Karaman, S. — *Accurate tracking of aggressive quadrotor trajectories using incremental nonlinear dynamic inversion and differential flatness* (2020)

[2] Mellinger, D., & Kumar, V. — *Minimum snap trajectory generation and control for quadrotors* (2011)

[3] Wang, X., van Kampen, E., Chu, Q., & Lu, P. — *Incremental sliding-mode fault-tolerant flight control* (2019)

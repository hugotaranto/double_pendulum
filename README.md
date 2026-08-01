# Sliding Double Pendulum Swing-Up and Stabilisation

Trajectory optimisation and optimal control of an underactuated sliding double pendulum using **Drake**, **Direct Transcription**, **LQR**, and **Time-Varying LQR (TVLQR)**.

---

## Demo

https://github.com/user-attachments/assets/a059fd63-37a4-4ad3-8d1f-88dfa237b028

---

## Overview

This project demonstrates autonomous control of a sliding double pendulum capable of moving between multiple stable configurations.

The controller combines:

- **Direct Transcription** to generate dynamically feasible swing-up trajectories
- **Finite Horizon TVLQR** to robustly track each trajectory
- **Infinite Horizon LQR** to stabilise each equilibrium configuration
- A high-level state machine that automatically switches between controllers

The system is simulated using the **Drake MultibodyPlant** physics engine with real-time visualisation through **Meshcat**.

---

## Features

- Dynamic simulation using Drake
- Optimal swing-up trajectory generation
- Automatic computation of LQR gains
- Finite-horizon TVLQR tracking
- Interactive Meshcat interface
- Multiple equilibrium transitions
- Saved controller gains for fast startup

---

## State Transitions

The controller supports transitions between the four equilibrium configurations.

| State | Description |
|--------|-------------|
| 00 | Both links hanging |
| 01 | First down, second up |
| 10 | First up, second down |
| 11 | Both links upright |

For every transition (12 transitions):

1. Direct Transcription computes an optimal trajectory.
2. TVLQR tracks the trajectory.
3. Once sufficiently close to the goal, the controller switches to the corresponding infinite-horizon LQR controller.

---

## Running

Start the simulation

```bash
python pendulum.py
```

The application launches a Meshcat visualiser where you can command transitions between equilibrium states.

---

## Methods

### Direct Transcription

Trajectory optimisation is formulated as a nonlinear optimisation problem using Drake's Direct Transcription framework.

The optimisation includes

- initial and terminal state constraints
- slider travel limits
- quadratic input cost
- joint velocity penalties

---

### Infinite Horizon LQR

An LQR controller is computed around each equilibrium point.

The controller is responsible for maintaining stability once the pendulum reaches its target configuration.

---

### Time-Varying LQR

Each optimised trajectory is converted into a finite-horizon TVLQR controller.

This provides significantly better robustness to modelling error and disturbances during swing-up than open-loop execution.

---

## Technologies

- Python
- Drake (pydrake)
- Meshcat
- NumPy

---

## Future Improvements

- Hardware implementation
- Automatic trajectory generation for arbitrary goals
- MPC-based controller
- Disturbance rejection experiments
- Energy-based swing-up comparison
- RL-based controller

---

## References

- Russ Tedrake — *Underactuated Robotics*
- Drake: Model-Based Design and Verification for Robotics

# Fixed-Wing UAV MATLAB Simulation

> 6-DOF fixed-wing UAV dynamics simulation in MATLAB based on the **Aerosonde** platform.
> Implements full aerodynamic models (longitudinal + lateral-directional), motor-propeller physics,
> and real-time 3D animation — built from first principles following Beard & McLain (2012).

---

## Repository Structure

```
fixedwing-matlab-simulation/
│
├── main.m                       # Entry point — run this
├── init_params.m                # Aerosonde UAV parameters (Beard & McLain Appendix E)
├── fixedwing_dynamics.m         # 6-DOF equations of motion (Newton-Euler)
├── fixedwing_system_wrapper.m   # ODE45 wrapper — connects control + dynamics
├── get_control_inputs.m         # Motor physics model + PD stabilization controller
├── body2world_xyz.m             # ZYX Euler rotation matrix (body → world)
├── animate.m                    # Real-time 3D UAV animation
└── plot_results.m               # Post-simulation results visualization
```

---

## Simulation Overview

The simulation models a full 12-state rigid-body aircraft:

| State Group | Variables | Description |
|---|---|---|
| Position | `pn, pe, pd` | NED position [m] |
| Body velocity | `u, v, w` | Body frame velocity [m/s] |
| Euler angles | `φ, θ, ψ` | Roll / Pitch / Yaw [rad] |
| Angular rates | `p, q, r` | Body angular rates [rad/s] |

**Control inputs:** Thrust `Tp` (from motor physics), aileron `δa`, elevator `δe`, rudder `δr`

---

## Aircraft Parameters (Aerosonde UAV)

| Parameter | Value |
|---|---|
| Mass | 11.0 kg |
| Wing span | 2.90 m |
| Wing area | 0.55 m² |
| Coordinate system | NED (North-East-Down) |
| Euler convention | ZYX (Yaw → Pitch → Roll) |

---

## Aerodynamic Model

### Longitudinal
$$C_L = C_{L_0} + C_{L_\alpha}\alpha + C_{L_q}\frac{c}{2V_a}q + C_{L_{\delta_e}}\delta_e$$
$$C_D = C_{D_0} + C_{D_\alpha}\alpha + C_{D_q}\frac{c}{2V_a}q + C_{D_{\delta_e}}|\delta_e|$$

**Stall model:** transitions from linear to flat-plate model at `α₀ = 0.47 rad (~27°)`

### Lateral-Directional
$$C_Y,\ C_l,\ C_n = f(\beta,\ p,\ r,\ \delta_a,\ \delta_r)$$

### Propulsion (Motor-Propeller Physics)
Thrust is computed from throttle via advance ratio `J` and quadratic coefficient fits `CT`, `CQ` (Beard & McLain Appendix E, Eq. 4.14).

---

## Visualization

### Real-Time 3D Animation (`animate.m`)
- Fuselage, wings, horizontal/vertical tail rendered in body frame
- Rotating propeller at nose
- Green heading arrow (body x-axis)
- Dotted flight trace

### Post-Simulation Plot (`plot_results.m`)
5-panel left + 3D trajectory right:

| Panel | Content |
|---|---|
| 1 | Altitude vs. time |
| 2 | Airspeed `Va` + Angle of attack `α` |
| 3 | Euler angles `φ / θ / ψ` |
| 4 | Control surfaces `δa / δe` |
| 5 | Thrust `Tp` |
| Right | 3D trajectory (altitude color-coded) |

---

## How to Run

```matlab
% In MATLAB command window:
main
```

Initial conditions (set in `main.m`):
- Altitude: 100 m
- Forward speed: 25 m/s
- Level flight (φ = θ = ψ = 0)

To change the flight scenario, edit the time-segmented commands in `get_control_inputs.m`.

---

## Requirements

- MATLAB R2020a or later
- No additional toolboxes required

---

## References

- Beard, R. W., & McLain, T. W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press.
- Förster, J. (2015). *System identification of the Crazyflie 2.0 nano quadrocopter*. ETH Zurich.
- Lecture 09: Fixed-wing Part 3, POSTECH MECH701A-01

---

## Author

**Lim Minseok** (임민석)  
M.S. Student, Mechanical Engineering, POSTECH  
[minseoklim@postech.ac.kr](mailto:minseoklim@postech.ac.kr)

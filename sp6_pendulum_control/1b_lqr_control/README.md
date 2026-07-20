# sp6 · Pendulum Control — 1b: LQR/LQI Balance Control

**Quanser reference:** [sp6 Application Guide — LQR/LQI Balance Control](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp6_pendulum_control/1b_lqr_control/)

---

## Objective

Catch and balance the Furuta pendulum at the upright equilibrium using LQR state feedback, with integral action (LQI) to eliminate steady-state arm tracking error under physical friction. Validate against setpoint step commands.

---

## Concept review

### Linearisation at the upright equilibrium

Linearising the pendulum equations of motion at the **upright** equilibrium (α = 0 upright, in the convention established in sp5):

```
ẋ = A·x + B·v_m
y = C·x
```

State vector: `x = [θ, α, θ̇, α̇]ᵀ`. Input: `v_m` (motor voltage, V).

At the upright equilibrium the gravity term has the **opposite sign** to the downward linearisation in sp5.2: it now acts as a destabilising force (inverted pendulum), so the (4,2) entry of A is positive.

The A and B matrices are computed by `modeling/lqr_design.py`. Physical parameters come from `modeling/constants.py`.

### LQR gain design

The gain vector K minimises the infinite-horizon cost:

```
J = ∫ (xᵀQx + uᵀRu) dt
```

Solved via the continuous-time algebraic Riccati equation (CARE). The default weights are:

| Weight | Default | Physical meaning |
|---|---|---|
| q_theta | 2.0 | Penalty on arm displacement |
| q_alpha | 35.0 | Penalty on pendulum tilt (high: critical for balance) |
| q_theta_dot | 0.1 | Penalty on arm velocity |
| q_alpha_dot | 0.1 | Penalty on pendulum angular rate |
| r | 1.0 | Penalty on control effort |

### Integral augmentation (LQI)

Pure LQR leaves a steady-state offset in θ under friction deadband. Adding an integrator on the arm tracking error:

```
ẋ_i = θ_d − θ
```

Augments the state to 5D and adds a 5th gain `k_i`. The integrator accumulator is clamped to `±2 rad` to prevent windup and reset to zero whenever the pendulum leaves the catch window.

### Catch logic

The controller is silenced (`v_m = 0`) whenever `|α| > 10°`. This prevents the motor from fighting the manual lift, avoids windup, and ensures the controller only runs near the regime where the linearisation is valid.

### Factory gains (reference baseline)

Quanser's worked example provides a fixed gain vector for the upright balance without tracking:

```
K_factory = [−1.2247, 24.9044, −0.6877, 3.1321]
```

These are available as "Factory" mode (`[g]` toggle) for direct comparison against the CARE-solved gains.

---

## Files in this folder

| File | Purpose |
|---|---|
| `lqr_balance_control.py` | Real-time LQR/LQI control loop, catch logic, gain tuning, telemetry |
| `lqr_run_log.csv` | Written on exit — telemetry from the active-control periods |
| `README.md` | This file |

---

## How to run

```bash
python sp6_pendulum_control/1b_lqr_control/lqr_balance_control.py
```

Select device: `0` (virtual twin) or `1` (physical hardware). The controller starts silenced. Raise the pendulum to within 10° of upright — manually (hardware) or via Q-Labs "Lift Pendulum" (virtual) — to engage balance control.

Press `[g]` to switch between Factory and Custom LQI gains. Press `[t]` to toggle ±20° setpoint tracking. Press `[q]` to exit; telemetry writes to `lqr_run_log.csv`.

---

## Status

| Item | Status |
|---|---|
| Virtual twin — balance | ✓ Validated: holds indefinitely, tracks setpoints |
| Virtual twin — LQI vs LQR | ✓ LQI eliminates steady-state arm offset |
| Physical hardware — balance | ⚠ Known gap: degrades after ~5 s (see below) |
| Physical hardware — setpoint tracking | Not yet tested |

### Known hardware gap: balance degrades after ~5 s

On physical hardware the balance hold currently degrades after approximately 5 seconds. Two contributing factors have been identified:

1. **GIL-induced timing jitter.** Python's Global Interpreter Lock causes the scope refresh thread to stall the control thread periodically, introducing 16–20 ms pauses. During these pauses the motor receives a zero-order hold on the last voltage command, producing voltage chatter at the natural frequency of the closed-loop system.

2. **Friction deadband.** The motor and cable drag create a deadband of approximately ±0.15 V. Below this threshold the motor does not respond, causing the integral term to wind up. LQI partially compensates, but the windup/release cycle introduces oscillation.

**Mitigations under investigation:**
- Separating the scope refresh into a fully independent process (not a thread) to remove GIL contention.
- Adding a friction-aware deadband compensator in the voltage output path.
- Stall detection: the disc application guide describes a subsystem that halts the motor if voltage exceeds ±5 V for >20 s — this is not yet implemented here and becomes important for longer RL training runs.

This gap is documented here, not papered over. Do not treat the hardware balance as solved.

---

## Checkpoint

1. **Catch engagement.** Confirm the motor stays silent until `|α| < 10°`.
2. **Upright balance (virtual).** Confirm pendulum holds within ±1° and arm settles to 0° within 3 s of catching.
3. **Setpoint tracking (virtual).** With `[t]` enabled, confirm the arm follows the ±20° square wave with settling time < 1.5 s and transient pendulum deflection < 8°.
4. **Factory vs. LQI comparison.** Record the settling time and steady-state arm error for both gain modes. Does the Factory gain show a residual arm offset on the virtual twin? (It should not — friction is lower in simulation.)
5. **Hardware.** How long does balance hold before degrading? Note the elapsed time and the last voltage command before loss of control.

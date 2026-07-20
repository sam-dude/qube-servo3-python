# sp6 · Pendulum Control — 2: Swing-Up Control

**Quanser reference:** [sp6 Application Guide — Swing-Up Control](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp6_pendulum_control/2_swing_up_control/)

---

## Objective

Drive the Furuta pendulum from the downward resting position to the upright equilibrium using energy-shaping control, without any direct reference trajectory. The controller injects or removes energy from the pendulum until its total mechanical energy matches a target value E_r corresponding to the upright unstable equilibrium.

---

## Concept review

### Energy-shaping principle

Define the pendulum's total mechanical energy relative to the hanging rest position:

```
E = E_k + E_p  =  (1/2)·J_p·α̇²  +  m_p·g·l·(1 + cos α)
```

At the upright equilibrium (α = 0, α̇ = 0): `E_upright = m_p·g·l·L_p/2`. This is the target energy E_r.

The energy error `ΔE = E − E_r` drives the control law. The pivot acceleration command u is chosen to make ΔE → 0.

### Control laws

Three modes are implemented:

**Smooth (continuous):**
```
u = k_e · (E − E_r) · α̇ · cos α
```
Injects energy when the pendulum is moving upward and has too little energy; removes it when it has too much. Smooth voltage profile.

**Saturated proportional (`sat_prop`, default):**
```
u = k_e · (E − E_r) · sign(α̇ · cos α)
```
From Åström & Furuta (1996), Eq. 8. Note: the Quanser lab manual contains a typo — the sign argument should be `α̇·cos α`, not `α·cos α`. The correct form is implemented here.

**Bang-bang:**
```
u = u_max · sign(k_e · (E − E_r) · sign(α̇ · cos α))
```
Full-amplitude switching. Fastest energy injection; highest mechanical stress on the arm joints.

### Voltage mapping

The acceleration command u (m/s²) is converted to motor voltage via the arm dynamics:

```
v_m = (R_m · m_r · L_r / k_t) · u
```

With the Quanser parameters this gives: `v_m ≈ 1.497 · u`.

### Centering spring

A proportional centering term `−k_p·θ` (default k_p = 6 V/rad) is subtracted from the voltage to keep the arm centred and away from the ±120° physical stops. It can be toggled off for investigation but is recommended on during normal operation.

### Kick-start

If the pendulum is hanging at perfect rest (`|α̇| < 0.1 rad/s`, `cos α ≈ −1`), the energy-shaping law produces zero command. A one-shot directional kick (`u = ±1`) is applied to break symmetry.

---

## Files in this folder

| File | Purpose |
|---|---|
| `swing_up_control.py` | Energy-shaping control loop, three modes, interactive tuning |
| `README.md` | This file |

---

## How to run

```bash
python sp6_pendulum_control/2_swing_up_control/swing_up_control.py
```

The script runs on the virtual twin (hardware=0). Let the pendulum hang at rest; it will auto-kick and begin pumping energy. Watch the energy trace approach E_r. If the pendulum consistently overshoots and doesn't settle near upright, reduce E_r by 1–2 mJ with `[j]`.

To chain swing-up into balance: run this script until the pendulum is consistently reaching near-upright, then switch to `lqr_balance_control.py` for the catch. A combined script (swing-up → automatic LQR catch handoff) is the next planned milestone.

---

## Status

| Item | Status |
|---|---|
| Virtual twin — sat_prop mode | ✓ Validated: consistent swing-up to near-upright |
| Virtual twin — smooth mode | ✓ Validated |
| Virtual twin — bang-bang mode | ✓ Validated (faster but more voltage chatter) |
| Physical hardware | Not yet tested |
| Combined swing-up + balance handoff | Not yet implemented — open milestone |

---

## Checkpoint

1. With `sat_prop` mode and default k_e = 50, does the pendulum reliably reach near-upright within ~10 s?
2. Increase k_e to 100 and 200. Does it swing up faster? Does it overshoot and require a lower E_r to stabilise near the top?
3. Toggle to `smooth` mode at the same k_e. Is the voltage profile visibly smoother on the scope? Does it take longer to reach upright?
4. What happens if you disable the centering spring (`[c]`) during swing-up? At what arm angle does it hit the physical stop?
5. What is the minimum E_r (in mJ) at which the pendulum reaches the upright equilibrium with `sat_prop` mode? How does this compare to the theoretical value `m_p·g·(L_p/2)`?

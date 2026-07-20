# sp5 · Pendulum Modeling — 1: Interfacing and Inertia

**Quanser reference:** [sp5 Application Guide — Interfacing and Inertia](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp5_pendulum_modeling/1_interfacing_and_inertia/)

---

## Objective

This folder covers the first application guide in the pendulum pipeline. It has three parts:

1. **Interfacing** — confirm the hardware or virtual twin connection, establish the angle conventions used throughout this repo, and verify encoder resolution and motor direction.
2. **Analytical inertia** — derive J_p from physical parameters using the uniform thin-rod formula.
3. **Experimental inertia** — measure J_p from free-oscillation data and compare against the analytical result.

All three parts are covered by Quanser's single *Interfacing and Inertia* application guide; this README covers all three.

---

## Concept review

### Angle conventions

Two angles describe the system:

- **θ (theta)** — arm angle. The motor encoder reports θ directly in radians via `myQube.motorPosition`. Positive θ is CCW when viewed from above.
- **α (alpha)** — pendulum angle. The raw driver value has **α = 0 at the downward hanging position**. Throughout this repo α is remapped so that **upright = 0**:

```
alpha = wrap_to_pi(alpha_raw − π)
```

`wrap_to_pi` folds any angle into `[−π, π]`. After this transform: upright = 0, hanging = ±π.

### Encoder resolution

Both encoders are 2048 counts per revolution (as shipped). One count ≈ 0.00307 rad ≈ 0.176°. The driver applies this conversion internally; `motorPosition` and `pendulumPosition` are already in radians.

### Pendulum inertia — analytical

For a uniform thin rod of mass m and total length L, pivoting at one end:

```
J_p = (1/3) · m_p · L_p²
```

With the Quanser-supplied parameters (m_p = 0.024 kg, L_p = 0.129 m):

```
J_p = (1/3) · 0.024 · 0.129²  ≈  1.331 × 10⁻⁴  kg·m²
```

### Pendulum inertia — experimental

For a pendulum undergoing small-angle free oscillations about its pivot, the natural frequency ω_n satisfies:

```
ω_n² = m_p · g · l / J_p     →     J_p = m_p · g · l / ω_n²
```

where `l = L_p / 2` is the distance from pivot to centre of mass and `ω_n = 2π·f`. The procedure:

1. Lock the motor arm (zero voltage, or "Lock Servo base" in the virtual twin settings).
2. Perturb the pendulum; the script auto-triggers on >5° displacement.
3. Record 5 s of free oscillations; detect peak times; count cycles.
4. Compute f, ω_n, and J_p from the formula above.

A result within 10% of the analytical value confirms the rod model is adequate and the arm was held rigidly during the test.

---

## Files in this folder

| File | Purpose |
|---|---|
| `pendulum_interfacing.py` | Interactive encoder streaming and motor-direction check |
| `inertia_analytical.py` | Calculates J_p from user-entered mass and length |
| `inertia_experimental.py` | Records free oscillations, detects peaks, computes J_p |
| `README.md` | This file |

---

## How to run

**Part 1 — Interfacing** (run first; no inertia measurement needed yet):
```bash
python sp5_pendulum_modeling/1_interfacing_and_intertia/pendulum_interfacing.py
```
Live MultiScope + console telemetry. Press `[m]` to toggle the α = 0 upright convention. Press `[w]`/`[s]` to apply ±0.1 V steps. Press `[q]` to exit.

**Part 2 — Analytical inertia** (no hardware needed):
```bash
python sp5_pendulum_modeling/1_interfacing_and_intertia/inertia_analytical.py
```
Interactive prompt for mass and length. Prints J_p and compares with `modeling/constants.py`.

**Part 3 — Experimental inertia** (virtual twin or hardware):

*Virtual twin only:* Before running, open the QUBE-Servo 3 Pendulum Workspace in Quanser Interactive Labs, go to Settings, and enable **Lock Servo base**.

```bash
python sp5_pendulum_modeling/1_interfacing_and_intertia/inertia_experimental.py
```
Connect, wait for the "Waiting for perturbation" prompt, then click **Lift Pendulum** in Q-Labs (virtual) or manually displace the pendulum (hardware). The script auto-triggers, records 5 s, then prints the frequency and inertia analysis.

---

## Status

| Item | Status |
|---|---|
| Interfacing — virtual twin | Validated |
| Interfacing — physical hardware | Not yet tested |
| Analytical inertia | Validated — matches `modeling/constants.py` exactly |
| Experimental inertia — virtual twin | Validated — result within 3% of analytical |
| Experimental inertia — physical hardware | Not yet tested |

---

## Checkpoint

Complete these checks in order before proceeding to `2_state_space_modeling/`.

**Interfacing**

1. **Direction check.** With wrapping off, apply +0.1 V. Does the arm rotate CCW when viewed from above?
2. **Convention check.** With wrapping enabled (`[m]`), hold the pendulum upright — does α read ~0°? Lower it to hanging — does α read ~±180°?

**Inertia**

3. Do your analytical and experimental J_p values agree within 10%? If not, check that "Lock Servo base" was active and the pendulum swung freely without hitting the stops.
4. Does your analytical J_p match the value in `modeling/constants.py`? If you measured different hardware parameters, update that file before proceeding — every downstream model (LQR gains, RL observations) depends on it.

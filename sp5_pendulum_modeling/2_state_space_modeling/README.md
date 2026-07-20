# sp5 · Pendulum Modeling — 2: State-Space Modeling

**Quanser reference:** [sp5 Application Guide — State-Space Modeling](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp5_pendulum_modeling/2_state_space_modeling/)

---

## Objective

Derive the linearised state-space representation of the Furuta pendulum from first principles, verify the matrix entries analytically, then validate the model against hardware (or virtual twin) by overlaying simulated and real responses to the same input.

---

## Concept review

### System and state vector

The QUBE-Servo 3 pendulum is a two-DOF underactuated system. One input (motor voltage), four states:

```
x = [θ, α, θ̇, α̇]ᵀ
u = v_m  (motor voltage, V)
y = [θ, α]ᵀ
```

`θ` is the arm angle (motor encoder, CCW positive). `α` is the pendulum angle, with the convention from `1_interfacing_and_inertia`: **α = 0 at upright, α = ±π at hanging**.

### Linearisation point

This folder linearises around the **downward equilibrium** (α = 0 hanging), which is the stable operating point for model identification. The LQR controller in sp6 uses a separate linearisation around the upward equilibrium — same matrix structure, sign change on the gravity term.

### Equations of motion

Applying Lagrangian mechanics to the coupled rigid-body system, then linearising:

```
(1)  J_r·θ̈ + m_p·l·r·α̈  =  τ − b_r·θ̇
(2)  J_p·α̈ + m_p·l·r·θ̈ + m_p·g·l·α  =  −b_p·α̇
```

Substituting `τ = (k_t/R_m)·(v_m − k_m·θ̇)` defines the effective arm damping `D_arm = b_r + k_t·k_m/R_m`.

### Mass matrix and inversion

Writing in matrix form `M·q̈ + D·q̇ + G·q = B_v·v_m`, where:

```
M = [[ J_r,       m_p·l·r ],     D = [[ D_arm,  0    ],
     [ m_p·l·r,   J_p     ]]          [   0,    b_p  ]]

G = [[  0,          0      ],     B_v = [ k_t/R_m,  0 ]ᵀ
     [  0,    m_p·g·l     ]]
```

Let `Δ = J_r·J_p − (m_p·l·r)²` (always > 0 for physically realisable parameters). Then:

```
A = [ 0₂  I₂  ]      B = [      0₂      ]
    [ −M⁻¹G   −M⁻¹D ]      [ M⁻¹·B_v   ]
```

### Damping identification (b_r, b_p)

The damping coefficients `b_r` and `b_p` are not directly measurable; they are identified by matching the simulated model to hardware. `state_space_model.py` lets you tune them interactively against a live square-wave excitation. The procedure:

1. Run with default values. Watch arm angle tracking — if the model decays too fast, `b_r` is too high.
2. Press `[w]` / `[s]` to adjust `b_r` until arm traces overlap.
3. Press `[e]` / `[d]` to adjust `b_p` until pendulum traces overlap.
4. Record the converged values and update `modeling/constants.py`.

**Known limitation:** the linear model matches well at low voltage (±1 V square wave, small excursions). At higher amplitudes the nonlinear coupling terms become significant and the model diverges. This is expected — the linearisation is only valid near the operating point.

---

## Files in this folder

| File | Purpose |
|---|---|
| `symbolic_derivation.py` | Prints/derives A and B symbolically via SymPy (or analytically if SymPy absent) |
| `state_space_model.py` | Live model-vs-hardware validation with real-time damping tuning |
| `README.md` | This file |

---

## How to run

**Step 1 — Inspect the derivation** (no hardware needed, SymPy optional):
```bash
python sp5_pendulum_modeling/2_state_space_modeling/symbolic_derivation.py
```

**Step 2 — Run the live validation:**
```bash
python sp5_pendulum_modeling/2_state_space_modeling/state_space_model.py
```

The script opens a 2×2 MultiScope window. Real and model traces are overlaid on the same axes. Tune damping with `[w]`/`[s]`/`[e]`/`[d]`, reset with `[r]`, quit with `[q]`.

---

## Status

| Item | Status |
|---|---|
| Symbolic derivation (SymPy) | Validated — A and B match analytical closed form |
| Model validation, virtual twin | Validated — good tracking at ±1 V, small angles |
| Model validation, physical hardware | Not yet tested |
| High-amplitude regime | Known gap — model diverges above ~±2 V or large α excursions |

---

## Checkpoint

After running both scripts, you should be able to answer:

1. What are your converged `b_r` and `b_p` values? Do they differ from the defaults in `modeling/constants.py`? If so, update the file and note the excitation amplitude used.
2. At what voltage amplitude does the model-vs-hardware error become clearly visible? (This is the practical linearisation validity limit for this hardware setup.)
3. The A matrix has a zero in position (3,1). Why? (Hint: at the downward equilibrium, does arm angle θ appear in the linearised acceleration equations?)

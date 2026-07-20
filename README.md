# Qube-Servo 3 in Python

Python control stack for the Quanser Qube-Servo 3, built to run without a MATLAB/Simulink license.

## Purpose

Quanser's Qube-Servo 3 curriculum is written for MATLAB/Simulink. This repository reimplements the same control theory in Python, so the hardware remains operable without a MATLAB license and integrates directly with the Python-based tools standard in robotics and reinforcement learning research (NumPy, SciPy, PyTorch, Stable-Baselines3).

This is a translation, not a reinterpretation. Where Quanser specifies a method, model, or parameter, this repository follows it. The contribution here is the execution layer.

## Scope

The long-term aim is a complete Python port of Quanser's seven-pipeline curriculum. Current development is scoped to the pendulum path.

| Pipeline | Sub-pipeline | Status |
|---|---|---|
| sp0. Instrumentation | Interfacing, Filtering | Not independently validated — see sp5 pendulum interfacing, which exercises the same read/write API |
| sp1. Modeling | — | Not started |
| sp2. Analysis | — | Not started |
| sp3. Fundamental Control | — | Not started |
| sp5. Pendulum Modeling | Interfacing & Inertia | Done |
| sp5. Pendulum Modeling | State-Space Modeling | Done |
| sp6. Pendulum Control | 1a. Balance Control | Done (implemented as a mode of 1b, see `sp6_pendulum_control/1a_balance_control/README.md`) |
| sp6. Pendulum Control | 1b. LQR/LQI Control | Done |
| sp6. Pendulum Control | 2. Swing-Up Control | Done |

> **Note on sp6 status:** balance control holds indefinitely in simulation. On physical hardware, balance currently degrades after approximately 5 seconds. This is a known, documented gap; see `sp6_pendulum_control/1b_lqr_control/README.md` for details. It is treated as an open problem, not a resolved one.

The immediate objective is a full swing-up-to-balance sequence, validated on hardware, that recovers from large disturbances. This serves as the baseline for subsequent research (residual learning, RL comparisons).

## Reference material

This repository does not include Quanser's PDFs. Each lab folder links directly to the corresponding document in Quanser's official repository. Consult the linked document before reading or modifying that lab's code.

Curriculum overview:
- [Curriculum Pipelines](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/curriculum_pipelines.pdf)

Document types you will encounter, and their role:
- **Application guide** — control theory, equations, and parameters for the lab.
- **Recommended assessment** — homework/quiz questions to test understanding.
- **Lab procedure** — step-by-step instructions (digital twin or hardware variant).

Supplementary reference sheets:
- [PID Control](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/supplementary_material/PID%20Control.pdf)
- [Pendulum Equations](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/supplementary_material/Pendulum%20Equations.pdf)
- [Swing-up Pendulum Energy Control](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/supplementary_material/Swing-up%20Pendulum%20Energy%20Control.pdf)

## Repo structure

```
sp0_instrumentation/              Pipeline 0: hardware read/write, safety, filtering
  README.md                       Gap note: no standalone sp0 lab was built

sp5_pendulum_modeling/            Pipeline 5: pendulum EOM, linearization, model validation
  1_interfacing_and_intertia/
    pendulum_interfacing.py       Encoder streaming & motor-direction check
    inertia_analytical.py         J_p from physical parameters (thin-rod formula)
    inertia_experimental.py       J_p from free-oscillation peak detection
    README.md
  2_state_space_modeling/
    symbolic_derivation.py        Derives A & B symbolically (SymPy) or analytically
    state_space_model.py          Live model-vs-hardware validation, damping tuning
    README.md

sp6_pendulum_control/             Pipeline 6: swing-up, LQR/LQI balance, disturbance tests
  1a_balance_control/
    README.md                     Pointer — implementation lives in 1b
  1b_lqr_control/
    lqr_balance_control.py        LQR/LQI catch-and-balance with setpoint tracking
    README.md
  2_swing_up_control/
    swing_up_control.py           Energy-shaping swing-up (three control modes)
    README.md
```

Control logic lives in tested Python modules. Each pipeline folder contains its own README that links to the relevant Quanser documents and describes what has and hasn't been validated on hardware.

## Installation

**1. Quanser software (hardware drivers, virtual devices, or both)**

Follow Quanser's own setup guide: [PC Setup](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/docs/pc_setup.md). This repository does not duplicate that process. Confirm `step_1_check_requirements` passes before continuing.

**2. Make the Quanser Python libraries importable**

After Quanser's setup, their Python packages (`pal`, `hal`, etc.) live in a local folder, not on Python's default search path. Create a virtual environment for this repo, then add a `.pth` file so the environment can find them:

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows
```

Create `.venv\Lib\site-packages\quanser.pth` containing a single line: the full path to your Quanser Python libraries folder (typically `Documents\Quanser\0_libraries\python`).

Confirm it worked:
```bash
python -c "from pal.products.qube import QubeServo3"
```
If this fails, the `.pth` file path is wrong or Quanser's setup step 2 did not complete.

**3. Install this repo's dependencies**

```bash
pip install -r requirements.txt
```

## Getting started

1. Complete Installation above. Confirm both the Quanser import check and `pip install` succeeded.
2. Read the [Curriculum Pipelines](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/curriculum_pipelines.pdf) overview.
3. Run the pendulum interfacing script to confirm the hardware or virtual twin connection works:
   ```bash
   python sp5_pendulum_modeling/1_interfacing_and_intertia/pendulum_interfacing.py
   ```
4. If that runs and streams live encoder data, proceed to `sp5_pendulum_modeling/README.md` and work through the pipelines in order: sp5 → sp6.

Note: this repo did not build a standalone sp0 disc lab. See `sp0_instrumentation/README.md` for details on that gap.

## Safety

This drives a physical motor. Every script that writes voltage must go through the safety clamp defined in `sp0_instrumentation/safety.py`. Confirm the kill switch (zero voltage command) works before running anything else.

## Contributing

Contributions toward the unimplemented pipelines (sp1, sp2, sp3) are welcome, as is hardware validation work on sp6 (the balance control degradation noted above is an open problem).

Before opening a pull request:

1. Read the relevant Quanser document, linked from the target pipeline's README, before writing any code.
2. Follow the folder and naming conventions established in `sp5_pendulum_modeling/` and `sp6_pendulum_control/`.
3. Keep control logic in tested modules. Keep analysis and plotting in notebooks that import from those modules, not the other way around.
4. State which Quanser document and section your implementation follows, in the module docstring or the pull request description.
5. If your contribution touches hardware behaviour, report what was tested on the physical device versus simulation only. Do not claim hardware validation without it.

Open an issue before starting on sp0, sp1, sp2, or sp3 to avoid duplicated work.

## Acknowledgments

This repository implements control theory and lab procedures developed by Quanser for the Qube-Servo 3. All theoretical content, equations, and lab methodology originate from Quanser's official curriculum. This repository is an independent Python implementation and is not affiliated with or endorsed by Quanser.
# sp6 · Pendulum Control — 1a: Balance Control

**Quanser reference:** [sp6 Application Guide — Balance Control](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp6_pendulum_control/1a_balance_control/)

---

This folder does not contain a separate implementation.

Quanser's sp6 curriculum splits balance control into two sub-pipelines:

- **1a** — basic LQR balance (no integral action, fixed gains from the application guide)
- **1b** — LQR/LQI with integral action, interactive weight tuning, and setpoint tracking

This repository implements both in a single script located at [`sp6_pendulum_control/1b_lqr_control/lqr_balance_control.py`](../1b_lqr_control/lqr_balance_control.py). The 1a use case (fixed Factory gains, no tracking) is accessible via the `[g]` toggle in that script, which switches to the hardcoded Quanser gain vector `K = [−1.2247, 24.9044, −0.6877, 3.1321]`.

**Why merged?** Building a separate 1a script that is a strict subset of 1b would have produced dead code. The toggle approach preserves the ability to reproduce the 1a baseline exactly while keeping the codebase DRY.

See [`1b_lqr_control/README.md`](../1b_lqr_control/README.md) for the full concept review, status, and checkpoint questions.

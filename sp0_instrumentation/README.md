# sp0 · Instrumentation

**Quanser reference:** [sp0 Application Guide](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp0_instrumentation/)

---

This repository did not build a standalone sp0 lab.

Quanser's sp0 curriculum covers generic disc instrumentation: confirming motor direction under positive voltage, reading encoder counts-to-degrees, and verifying loop timing — without any pendulum attachment. No such script exists here, and none was intentionally skipped; the decision was to move directly to the pendulum path (sp5) rather than first characterising the bare disc.

The closest equivalent in this repo is [`sp5_pendulum_modeling/1_interfacing_and_intertia/pendulum_interfacing.py`](../sp5_pendulum_modeling/1_interfacing_and_intertia/pendulum_interfacing.py), which exercises the same underlying `QubeServo3` read/write API and establishes the same encoder-resolution and motor-direction checks, but with `pendulum=1` and the α = 0-upright convention already applied.

**Gap:** no generic disc-only encoder/motor check has been performed in this repo. If you need to validate the bare disc configuration (e.g. for sp1–sp3 work on the disc plant), that script does not yet exist.

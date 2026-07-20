"""State-Space Model Validation for the Furuta Pendulum.

Drives the QUBE-Servo 3 with a 1 V, 1 Hz square wave and overlays the
real-time hardware response against a simulated linear state-space model,
both plotted on the same MultiScope axes.

The model is linearised around the downward equilibrium (alpha = 0 hanging).
Damping coefficients b_r and b_p can be tuned in real time via the keyboard
so the simulated trace can be matched to the hardware response by eye.

State vector: x = [theta, alpha, theta_dot, alpha_dot]^T
Input:        u = v_m (motor voltage, V)

Controls:
  [w] / [s]   : Increase / decrease arm damping b_r (+10% / -10%)
  [e] / [d]   : Increase / decrease pendulum damping b_p (+10% / -10%)
  [r]         : Reset both damping coefficients to defaults
  [q] / [Esc] : Quit

Reference:
  Quanser sp5 Application Guide — State-Space Modeling
  https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp5_pendulum_modeling/2_state_space_modeling/
"""

from __future__ import annotations

import sys
import time
import msvcrt
import numpy as np
from threading import Thread
from pathlib import Path

try:
    from pal.products.qube import QubeServo3
    from pal.utilities.scope import MultiScope
except ModuleNotFoundError:
    print("[-] Error: Quanser PAL libraries not found.")
    print("    Activate the project virtual environment before running:")
    print("      .venv\\Scripts\\Activate.ps1")
    print("    See README.md > 'Installation' for setup instructions.")
    sys.exit(1)

# Physical constants — import from shared module, fall back to defaults
_project_root = str(Path(__file__).resolve().parents[2])
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)
try:
    from modeling.constants import (
        J_r, J_p, m_p, m_r, L_r, L_p, l_p, g,
        k_m, k_t, R_m, b_r as DEFAULT_BR, b_p as DEFAULT_BP
    )
except ImportError:
    L_r = 0.085
    m_r = 0.095
    J_r = (1.0 / 3.0) * m_r * L_r**2
    L_p = 0.129
    l_p = L_p / 2.0
    m_p = 0.024
    J_p = (1.0 / 3.0) * m_p * L_p**2
    g = 9.81
    k_t = 0.0422
    k_m = 0.0422
    R_m = 7.5
    DEFAULT_BR = 1.5e-3
    DEFAULT_BP = 5.0e-5


def compute_matrices(br: float, bp: float) -> tuple[np.ndarray, np.ndarray]:
    """Compute continuous-time A and B for the downward equilibrium.

    The coupling term sign follows the linearisation at alpha=0 (hanging):
    gravity acts as a restoring force, so the (4,2) element of A is negative
    (alpha_dot_dot decreases as alpha increases from hanging rest).
    """
    coupling = -m_p * L_r * l_p  # off-diagonal mass matrix entry

    M = np.array([
        [J_r + m_p * L_r**2, coupling],
        [coupling,            J_p],
    ])

    G = np.array([
        [0.0,               0.0],
        [0.0, m_p * g * l_p],
    ])

    D_arm = br + (k_m**2 / R_m)  # effective arm damping (includes back-EMF)
    D_mat = np.array([
        [D_arm, 0.0],
        [0.0,   bp],
    ])

    B_tau = np.array([k_t / R_m, 0.0])

    Minv = np.linalg.inv(M)

    A = np.zeros((4, 4))
    A[0, 2] = 1.0
    A[1, 3] = 1.0
    A[2:4, 0:2] = -Minv @ G
    A[2:4, 2:4] = -Minv @ D_mat

    B = np.zeros((4, 1))
    B[2:4, 0] = Minv @ B_tau

    return A, B


def main() -> None:
    print("==================================================")
    print("   sp5.2: State-Space Model Validation            ")
    print("==================================================")
    print("[*] Linearising around downward equilibrium (alpha = 0)")
    print("[*] Driving with 1 V, 1 Hz square wave")
    print("--------------------------------------------------")
    print("Controls:")
    print("  [w] / [s]   : b_r  +10% / -10%")
    print("  [e] / [d]   : b_p  +10% / -10%")
    print("  [r]         : Reset damping to defaults")
    print("  [q] / [Esc] : Quit")
    print("--------------------------------------------------")
    input("Press Enter to begin validation...")

    multiScope = MultiScope(rows=2, cols=2, title="sp5.2: State-Space Model Validation")

    multiScope.addAxis(row=0, col=0, timeWindow=5)
    multiScope.axes[0].attachSignal(name='Arm Angle — Real (deg)')
    multiScope.axes[0].attachSignal(name='Arm Angle — Model (deg)')

    multiScope.addAxis(row=0, col=1, timeWindow=5)
    multiScope.axes[1].attachSignal(name='Pendulum Angle — Real (deg)')
    multiScope.axes[1].attachSignal(name='Pendulum Angle — Model (deg)')

    multiScope.addAxis(row=1, col=0, timeWindow=5)
    multiScope.axes[2].attachSignal(name='Square Wave Input (V)')

    multiScope.addAxis(row=1, col=1, timeWindow=5)
    multiScope.axes[3].attachSignal(name='Arm Velocity — Real (deg/s)')
    multiScope.axes[3].attachSignal(name='Arm Velocity — Model (deg/s)')

    global running, b_r, b_p
    running = True
    b_r = DEFAULT_BR
    b_p = DEFAULT_BP

    def control_loop():
        global running, b_r, b_p

        x_ss = np.zeros((4, 1))   # simulated state
        count = 0
        count_max = 500 / 50      # 50 Hz scope update
        dt = 0.002                 # 500 Hz loop

        last_br, last_bp = -1.0, -1.0
        Ad, Bd = None, None

        with QubeServo3(hardware=0, pendulum=1) as myQube:
            start_time = time.time()
            time_last_print = 0.0

            while running:
                myQube.read_outputs()

                # Recompute discretised matrices only when damping changes
                if b_r != last_br or b_p != last_bp:
                    A, B = compute_matrices(b_r, b_p)
                    C_out = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
                    D_out = np.zeros((2, 1))
                    from scipy.signal import cont2discrete
                    Ad, Bd, _, _, _ = cont2discrete((A, B, C_out, D_out), dt, method='zoh')
                    last_br, last_bp = b_r, b_p

                current_time = time.time() - start_time

                # 1 V, 1 Hz square wave
                voltage = 1.0 if (current_time % 1.0) < 0.5 else -1.0
                myQube.write_voltage(voltage)

                # Step the simulated model
                x_ss = Ad @ x_ss + Bd * voltage

                # Read real hardware states
                theta_real  = float(myQube.motorPosition.item()    if hasattr(myQube.motorPosition,    'item') else myQube.motorPosition)
                alpha_real  = float(myQube.pendulumPosition.item() if hasattr(myQube.pendulumPosition, 'item') else myQube.pendulumPosition)
                # Note: motorSpeed sign flip + /10 matches empirical virtual-twin output scaling
                theta_dot_real = -float(myQube.motorSpeed.item()   if hasattr(myQube.motorSpeed,       'item') else myQube.motorSpeed) / 10.0

                # Model states
                theta_model    = float(x_ss[0, 0])
                alpha_model    = float(x_ss[1, 0])
                theta_dot_model = float(x_ss[2, 0])

                # Degrees for scope
                theta_real_deg     = np.rad2deg(theta_real)
                theta_model_deg    = np.rad2deg(theta_model)
                alpha_real_deg     = np.rad2deg(alpha_real)
                alpha_model_deg    = np.rad2deg(alpha_model)
                theta_dot_real_deg  = np.rad2deg(theta_dot_real)
                theta_dot_model_deg = np.rad2deg(theta_dot_model)

                if current_time - time_last_print >= 0.2:
                    sys.stdout.write(
                        f"\r[Damp] b_r: {b_r*1000:.2f} mN·m·s | b_p: {b_p*1000:.3f} mN·m·s | "
                        f"Arm Err: {abs(theta_real_deg - theta_model_deg):4.2f}° | "
                        f"Pend Err: {abs(alpha_real_deg - alpha_model_deg):4.2f}°"
                    )
                    sys.stdout.flush()
                    time_last_print = current_time

                count += 1
                if count >= count_max:
                    multiScope.axes[0].sample(current_time, [theta_real_deg, theta_model_deg])
                    multiScope.axes[1].sample(current_time, [alpha_real_deg, alpha_model_deg])
                    multiScope.axes[2].sample(current_time, [voltage])
                    multiScope.axes[3].sample(current_time, [theta_dot_real_deg, theta_dot_model_deg])
                    count = 0

                time.sleep(0.0001)

    thread = Thread(target=control_loop)
    thread.daemon = True
    thread.start()

    try:
        while running and thread.is_alive():
            multiScope.refresh()
            if msvcrt.kbhit():
                ch = msvcrt.getch().lower()
                if ch == b'w':
                    b_r = min(1.0, b_r * 1.1)
                elif ch == b's':
                    b_r = max(1e-6, b_r * 0.9)
                elif ch == b'e':
                    b_p = min(1.0, b_p * 1.1)
                elif ch == b'd':
                    b_p = max(1e-6, b_p * 0.9)
                elif ch == b'r':
                    b_r, b_p = DEFAULT_BR, DEFAULT_BP
                    print("\n[Reset] Damping coefficients reset to defaults.")
                elif ch in (b'q', b'\x1b'):
                    running = False
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n\n[!] Interrupted by user.")
    finally:
        running = False
        thread.join(timeout=1.0)
        print("\n[+] Shut down cleanly. Voltage reset to 0.0 V.")


if __name__ == "__main__":
    main()

"""Energy-Shaping Swing-Up Control for the Furuta Pendulum.

Drives the pendulum from the downward resting position to the upright
equilibrium using an energy-shaping control law. Three control modes are
available and can be toggled interactively:

  smooth    — u = k_e · (E − E_r) · α̇ · cos(α)
              Continuous energy-injection. Smoother voltage profile.

  sat_prop  — u = k_e · (E − E_r) · sign(α̇ · cos(α))         [default]
              Saturated proportional law (Åström & Furuta 1996, Eq. 8).
              Note: the lab manual contains a typo writing α instead of α̇
              in the sign argument; the correct form is used here.

  bang_bang — u = u_max · sign(k_e · (E − E_r) · sign(α̇ · cos(α)))
              Full-amplitude switching. Fastest energy injection; highest
              mechanical stress.

The pivot acceleration command u (m/s²) is converted to motor voltage by:

    v_m = (R_m · m_r · L_r / k_t) · u

A centering spring term (`−k_p · θ`) is added to keep the arm away from the
physical stops at ±120°. It can be toggled off for open-loop investigation.

If the pendulum is hanging at rest (|α̇| < 0.1 rad/s, cos α ≈ −1), a
one-shot directional kick is applied to break the unstable symmetric
equilibrium.

Controls:
  [w] / [Up Arrow]   : Increase k_e (+10)
  [s] / [Down Arrow] : Decrease k_e (−10)
  [u] / [j]          : Increase / decrease E_r (+1 mJ / −1 mJ)
  [r]                : Reset k_e and E_r to defaults
  [c]                : Toggle centering spring
  [t]                : Cycle control mode (smooth → sat_prop → bang_bang)
  [q] / [Esc]        : Quit

Reference:
  Quanser sp6 Application Guide — Swing-Up Control
  https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp6_pendulum_control/2_swing_up_control/

  Åström & Furuta (1996): "Swinging up a pendulum by energy control."
  Automatica, 36(2):287–295.
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
    from modeling.constants import m_p, L_p, l_p, J_p, g, m_r, L_r, k_t, R_m
except ImportError:
    m_p  = 0.024
    L_p  = 0.129
    l_p  = L_p / 2.0
    J_p  = (1.0 / 3.0) * m_p * L_p**2
    g    = 9.81
    m_r  = 0.095
    L_r  = 0.085
    k_t  = 0.0422
    R_m  = 7.5


def wrap_to_pi(angle: float) -> float:
    """Wrap angle in radians to [−π, π]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def main() -> None:
    # Voltage-to-acceleration scale factor
    u_to_v = (R_m * m_r * L_r) / k_t

    print("==================================================")
    print("   sp6.2: Energy-Shaping Swing-Up Control         ")
    print("==================================================")
    print(f"[*] v_m = ({R_m} × {m_r} × {L_r} / {k_t}) × u  =  {u_to_v:.4f} × u")
    print("--------------------------------------------------")
    print("Controls:")
    print("  [w] / [Up]   : k_e  +10")
    print("  [s] / [Down] : k_e  −10")
    print("  [u] / [j]    : E_r  +1 mJ / −1 mJ")
    print("  [r]          : Reset k_e and E_r to defaults")
    print("  [c]          : Toggle centering spring")
    print("  [t]          : Cycle control mode")
    print("  [q] / [Esc]  : Quit")
    print("--------------------------------------------------")
    input("Press Enter to start the swing-up loop...")

    multiScope = MultiScope(rows=2, cols=2, title="sp6.2: Energy-Shaping Swing-Up")

    multiScope.addAxis(row=0, col=0, timeWindow=5)
    multiScope.axes[0].attachSignal(name='Pendulum Angle (deg)')
    multiScope.axes[0].attachSignal(name='Arm Angle (deg)')

    multiScope.addAxis(row=0, col=1, timeWindow=5)
    multiScope.axes[1].attachSignal(name='Total Energy (mJ)')
    multiScope.axes[1].attachSignal(name='Target Energy (mJ)')

    multiScope.addAxis(row=1, col=0, timeWindow=5)
    multiScope.axes[2].attachSignal(name='Applied Voltage (V)')

    multiScope.addAxis(row=1, col=1, timeWindow=5)
    multiScope.axes[3].attachSignal(name='Pivot Acceleration u (m/s²)')

    global running, k_e, centering_enabled, control_mode, E_ref
    running = True
    k_e = 50.0
    centering_enabled = True
    control_mode = "sat_prop"
    E_ref = 0.010   # 10 mJ starting target (lab manual step 15)

    def control_loop() -> None:
        global running, k_e, centering_enabled, control_mode, E_ref

        count = 0
        count_max = 500 / 50   # 50 Hz scope update
        u_max = 6.0             # m/s² saturation limit

        with QubeServo3(hardware=0, pendulum=1) as myQube:
            start_time = time.time()
            time_last_print = 0.0

            while running:
                myQube.read_outputs()

                theta     = float(myQube.motorPosition.item()    if hasattr(myQube.motorPosition,    'item') else myQube.motorPosition)
                alpha_raw = float(myQube.pendulumPosition.item() if hasattr(myQube.pendulumPosition, 'item') else myQube.pendulumPosition)
                theta_dot_raw = float(myQube.motorSpeed.item()   if hasattr(myQube.motorSpeed,       'item') else myQube.motorSpeed)
                alpha_dot_raw = float(myQube.pendulumSpeed.item() if hasattr(myQube.pendulumSpeed,   'item') else myQube.pendulumSpeed)

                # Correct virtual-twin speed scaling
                theta_dot = -theta_dot_raw / 10.0
                alpha_dot = alpha_dot_raw  / 10.0

                # α = 0 upright convention
                alpha = wrap_to_pi(alpha_raw - np.pi)

                # Energy relative to hanging rest (E_p = 0 at hanging)
                E_k = 0.5 * J_p * alpha_dot**2
                E_p = m_p * g * l_p * (1.0 + np.cos(alpha))
                E_total = E_k + E_p

                # Kick-start if hanging at rest (symmetric equilibrium)
                if abs(alpha_dot) < 0.1 and abs(np.cos(alpha) + 1.0) < 0.05:
                    u_cmd = 1.0 if theta >= 0.0 else -1.0
                else:
                    dE = E_total - E_ref
                    sign_arg = alpha_dot * np.cos(alpha)

                    if control_mode == "smooth":
                        u_cmd = k_e * dE * alpha_dot * np.cos(alpha)
                    elif control_mode == "sat_prop":
                        u_cmd = k_e * dE * np.sign(sign_arg)
                    else:  # bang_bang
                        u_cmd = u_max * np.sign(k_e * dE * np.sign(sign_arg))

                u_sat = float(np.clip(u_cmd, -u_max, u_max))
                voltage = u_to_v * u_sat

                if centering_enabled:
                    voltage -= 6.0 * theta   # proportional centering spring

                final_voltage = float(np.clip(voltage, -6.0, 6.0))
                myQube.write_voltage(final_voltage)

                current_time = time.time() - start_time

                if current_time - time_last_print >= 0.2:
                    sys.stdout.write(
                        f"\r[{control_mode.upper():8s}] k_e:{k_e:5.1f} | "
                        f"E:{float(E_total)*1000:5.1f} mJ (target:{float(E_ref)*1000:4.1f} mJ) | "
                        f"u:{u_sat:+.2f} m/s² | V:{final_voltage:+.2f}V"
                    )
                    sys.stdout.flush()
                    time_last_print = current_time

                count += 1
                if count >= count_max:
                    multiScope.axes[0].sample(current_time, [float(np.rad2deg(alpha)), float(np.rad2deg(theta))])
                    multiScope.axes[1].sample(current_time, [float(E_total) * 1000.0, float(E_ref) * 1000.0])
                    multiScope.axes[2].sample(current_time, [final_voltage])
                    multiScope.axes[3].sample(current_time, [u_sat])
                    count = 0

                time.sleep(0.0001)

    thread = Thread(target=control_loop)
    thread.daemon = True
    thread.start()

    try:
        while running and thread.is_alive():
            multiScope.refresh()

            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x00', b'\xe0'):
                    ch2 = msvcrt.getch()
                    if ch2 == b'H':    # Up Arrow
                        k_e = min(1000.0, k_e + 10.0)
                        print(f"\n[Tune] k_e: {k_e:.1f}")
                    elif ch2 == b'P':  # Down Arrow
                        k_e = max(0.0, k_e - 10.0)
                        print(f"\n[Tune] k_e: {k_e:.1f}")
                else:
                    ch = ch.lower()
                    if ch == b'w':
                        k_e = min(1000.0, k_e + 10.0)
                        print(f"\n[Tune] k_e: {k_e:.1f}")
                    elif ch == b's':
                        k_e = max(0.0, k_e - 10.0)
                        print(f"\n[Tune] k_e: {k_e:.1f}")
                    elif ch == b'u':
                        E_ref = min(0.1, E_ref + 0.001)
                        print(f"\n[Tune] E_ref: {E_ref*1000:.1f} mJ")
                    elif ch == b'j':
                        E_ref = max(0.001, E_ref - 0.001)
                        print(f"\n[Tune] E_ref: {E_ref*1000:.1f} mJ")
                    elif ch == b'r':
                        k_e, E_ref = 50.0, 0.010
                        print(f"\n[Reset] k_e={k_e:.1f}, E_ref={E_ref*1000:.1f} mJ")
                    elif ch == b'c':
                        centering_enabled = not centering_enabled
                        print(f"\n[Toggle] Centering spring: {'ON' if centering_enabled else 'OFF'}")
                    elif ch == b't':
                        modes = ["smooth", "sat_prop", "bang_bang"]
                        control_mode = modes[(modes.index(control_mode) + 1) % 3]
                        print(f"\n[Toggle] Mode: {control_mode.upper()}")
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

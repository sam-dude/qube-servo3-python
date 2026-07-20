"""LQR/LQI Balance Control and Setpoint Tracking for the Furuta Pendulum.

Implements an LQR (or LQI with integral action) controller that catches and
balances the pendulum in the upright position, with optional arm-angle setpoint
tracking. Two gain modes are available:

  - Factory:  fixed gains [-1.2247, 24.9044, -0.6877, 3.1321] from Quanser's
              worked example. No integral term.
  - Custom LQI: gains solved live via dare (continuous-time ARE) from the
                Q and R weights set interactively below.

The catch window is |α| < 10°. Outside this window the motor is silenced and
the integral accumulator is reset to prevent windup.

Telemetry is buffered in memory and written to lqr_run_log.csv on exit.

Controls:
  [w] / [s]   : Tune arm weight q_theta       (+10% / −10%)
  [e] / [d]   : Tune pendulum weight q_alpha   (+10% / −10%)
  [x] / [c]   : Tune input weight r            (+10% / −10%)
  [u] / [j]   : Tune integral weight q_integral (+10% / −10%)
  [t]         : Toggle setpoint tracking (±20° arm square wave, 6 s period)
  [g]         : Toggle gain mode (Factory ↔ Custom LQI)
  [r]         : Reset all weights to defaults
  [q] / [Esc] : Quit and save telemetry

Reference:
  Quanser sp6 Application Guide — LQR/LQI Balance Control
  https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp6_pendulum_control/1b_lqr_control/
"""

from __future__ import annotations

import sys
import time
import msvcrt
import math
import signal
import numpy as np
from threading import Thread
from pathlib import Path

try:
    from pal.products.qube import QubeServo3
    from pal.utilities.math import ddt_filter
    from pal.utilities.scope import MultiScope
except ModuleNotFoundError:
    print("[-] Error: Quanser PAL libraries not found.")
    print("    Activate the project virtual environment before running:")
    print("      .venv\\Scripts\\Activate.ps1")
    print("    See README.md > 'Installation' for setup instructions.")
    sys.exit(1)

# LQR/LQI design module — must be importable from project root
_project_root = str(Path(__file__).resolve().parents[2])
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)
try:
    from modeling.lqr_design import design_lqi
except ImportError as exc:
    print(f"[-] Error: Could not import modeling.lqr_design: {exc}")
    sys.exit(1)


def main() -> None:
    print("==================================================")
    print("   sp6.1b: LQR/LQI Balance Control               ")
    print("==================================================")
    print("[*] Catch window: |α| < 10° (upright)")
    print("--------------------------------------------------")

    hw_input = input("Device: [0] Virtual Twin (default)  [1] Physical QUBE-Servo 3: ").strip()
    target_hardware = 1 if hw_input == "1" else 0

    print("--------------------------------------------------")
    if target_hardware == 1:
        print("[*] Physical hardware selected.")
        print("1. Set up the QUBE-Servo 3.")
        print("2. Manually lift the pendulum to the upright position.")
    else:
        print("[*] Virtual twin selected.")
        print("1. Open QUBE-Servo 3 Pendulum Workspace in Q-Labs.")
        print("2. Click 'Lift Pendulum' to engage the catch window.")

    print("3. The LQI controller catches and holds automatically.")
    print("4. Press [t] to enable arm tracking; [g] to switch gain mode.")
    print("--------------------------------------------------")
    print("Controls:")
    print("  [w] / [s]   : q_theta   +10% / −10%")
    print("  [e] / [d]   : q_alpha   +10% / −10%")
    print("  [x] / [c]   : r         +10% / −10%")
    print("  [u] / [j]   : q_integral +10% / −10%")
    print("  [t]         : Toggle setpoint tracking")
    print("  [g]         : Toggle Factory / Custom LQI gains")
    print("  [r]         : Reset weights to defaults")
    print("  [q] / [Esc] : Quit + save telemetry")
    print("--------------------------------------------------")
    input("Press Enter to start the balance loop...")

    multiScope = MultiScope(rows=2, cols=2, title="sp6.1b: LQR/LQI Balance Control")

    multiScope.addAxis(row=0, col=0, timeWindow=5)
    multiScope.axes[0].attachSignal(name='Arm Angle (deg)')
    multiScope.axes[0].attachSignal(name='Arm Target (deg)')

    multiScope.addAxis(row=0, col=1, timeWindow=5)
    multiScope.axes[1].attachSignal(name='Pendulum Angle (deg)')

    multiScope.addAxis(row=1, col=0, timeWindow=5)
    multiScope.axes[2].attachSignal(name='Applied Voltage (V)')

    multiScope.addAxis(row=1, col=1, timeWindow=5)
    multiScope.axes[3].attachSignal(name='k_theta (×1)')
    multiScope.axes[3].attachSignal(name='k_alpha (×0.1)')

    global running, q_theta, q_alpha, r_weight, q_integral
    global tracking_enabled, gain_mode, log_buffer
    running = True
    q_theta = 2.0
    q_alpha = 35.0
    r_weight = 1.0
    q_integral = 0.5
    tracking_enabled = False
    gain_mode = 0        # 0 = Factory, 1 = Custom LQI
    log_buffer = []

    def sig_handler(*args):
        global running
        running = False
    signal.signal(signal.SIGINT, sig_handler)

    def control_loop(hw_type: int) -> None:
        global running, q_theta, q_alpha, r_weight, q_integral
        global tracking_enabled, gain_mode, log_buffer

        frequency = 500.0
        state_theta_dot = np.array([0.0, 0.0])
        state_alpha_dot = np.array([0.0, 0.0])

        theta_integral = 0.0
        theta_target_offset = 0.0
        count = 0
        count_max = frequency / 50.0   # 50 Hz scope update

        last_q_theta, last_q_alpha = -1.0, -1.0
        last_r, last_q_integral      = -1.0, -1.0
        K_custom = np.zeros(5)

        with QubeServo3(hardware=hw_type, pendulum=1, frequency=int(frequency)) as myQube:
            start_time = time.perf_counter()
            time_last_print = 0.0

            while running:
                myQube.read_outputs()

                # Recompute LQI gains only when weights change
                if (q_theta != last_q_theta or q_alpha != last_q_alpha or
                        r_weight != last_r or q_integral != last_q_integral):
                    try:
                        K_raw, _ = design_lqi(
                            q_theta=q_theta,
                            q_alpha=q_alpha,
                            q_theta_dot=0.1,
                            q_alpha_dot=0.1,
                            q_integral=q_integral,
                            r_val=r_weight,
                        )
                        # Sign convention: Quanser uses −α and −α̇ in state vector
                        K_custom = np.array([K_raw[0], -K_raw[1], K_raw[2], -K_raw[3], K_raw[4]])
                        last_q_theta, last_q_alpha = q_theta, q_alpha
                        last_r, last_q_integral    = r_weight, q_integral
                    except Exception:
                        pass

                if gain_mode == 0:
                    K_active = np.array([-1.2247, 24.9044, -0.6877, 3.1321])
                    mode_str = "Factory"
                else:
                    K_active = K_custom
                    mode_str = "Custom "

                current_time = time.perf_counter() - start_time

                # Read sensor states
                theta   = float(myQube.motorPosition.item()    if hasattr(myQube.motorPosition,    'item') else myQube.motorPosition) * -1.0
                alpha_f = float(myQube.pendulumPosition.item() if hasattr(myQube.pendulumPosition, 'item') else myQube.pendulumPosition)
                alpha   = np.mod(alpha_f, 2.0 * np.pi) - np.pi
                alpha_deg = abs(math.degrees(alpha))

                # Filtered velocities
                theta_dot, state_theta_dot = ddt_filter(theta,   state_theta_dot, 50,  1.0 / frequency)
                alpha_dot, state_alpha_dot = ddt_filter(alpha_f, state_alpha_dot, 100, 1.0 / frequency)

                # Setpoint
                if tracking_enabled:
                    command_deg = 20.0 if (current_time % 6.0) < 3.0 else -20.0
                else:
                    command_deg = 0.0

                # Catch logic: silence motor outside catch window
                if alpha_deg > 10.0:
                    voltage = 0.0
                    theta_integral = 0.0
                    theta_target_offset = theta
                    theta_ref = theta
                else:
                    theta_target_offset *= 0.999
                    theta_ref = (command_deg * np.pi / 180.0) + theta_target_offset

                    theta_error = theta_ref - theta
                    theta_integral += theta_error * (1.0 / frequency)
                    theta_integral = np.clip(theta_integral, -2.0, 2.0)

                states = np.array([theta_ref - theta, -alpha, -theta_dot, -alpha_dot])

                if alpha_deg > 10.0:
                    voltage = 0.0
                else:
                    if len(K_active) >= 5:
                        voltage = -np.dot(K_active[:4], states) + K_active[4] * theta_integral
                    else:
                        # Factory LQR: ad-hoc integral gain
                        voltage = -np.dot(K_active, states) + 0.5 * theta_integral

                final_voltage = float(np.clip(voltage, -6.0, 6.0))
                myQube.write_voltage(final_voltage)

                # Buffer telemetry when controller is active
                if alpha_deg <= 10.0:
                    log_buffer.append((
                        current_time, theta, alpha,
                        theta_dot, alpha_dot, final_voltage, gain_mode,
                    ))

                if current_time - time_last_print >= 0.2:
                    k_i = K_active[4] if len(K_active) >= 5 else 0.5
                    sys.stdout.write(
                        f"\r[{mode_str}] q_th:{q_theta:5.2f} q_al:{q_alpha:5.2f} "
                        f"q_in:{q_integral:5.2f} r:{r_weight:5.2f} | "
                        f"K:[{K_active[0]:+.2f},{K_active[1]:+.2f},{K_active[2]:+.2f},"
                        f"{K_active[3]:+.2f},{k_i:+.2f}] | V:{final_voltage:+.2f}V"
                    )
                    sys.stdout.flush()
                    time_last_print = current_time

                count += 1
                if count >= count_max:
                    multiScope.axes[0].sample(current_time, [float(np.rad2deg(-theta)), float(command_deg)])
                    multiScope.axes[1].sample(current_time, [float(np.rad2deg(alpha))])
                    multiScope.axes[2].sample(current_time, [final_voltage])
                    multiScope.axes[3].sample(current_time, [float(K_active[0]), float(K_active[1] * 0.1)])
                    count = 0

    thread = Thread(target=control_loop, args=(target_hardware,))
    thread.daemon = True
    thread.start()

    try:
        last_refresh = time.perf_counter()
        while running and thread.is_alive():
            now = time.perf_counter()
            if now - last_refresh >= 0.033:   # ~30 Hz scope refresh
                multiScope.refresh()
                last_refresh = now

            if msvcrt.kbhit():
                ch = msvcrt.getch().lower()
                if ch == b'w':
                    q_theta = min(100.0, q_theta * 1.1)
                elif ch == b's':
                    q_theta = max(0.1, q_theta * 0.9)
                elif ch == b'e':
                    q_alpha = min(1000.0, q_alpha * 1.1)
                elif ch == b'd':
                    q_alpha = max(1.0, q_alpha * 0.9)
                elif ch == b'x':
                    r_weight = min(10.0, r_weight * 1.1)
                elif ch == b'c':
                    r_weight = max(0.05, r_weight * 0.9)
                elif ch == b'u':
                    q_integral = min(50.0, q_integral * 1.1)
                elif ch == b'j':
                    q_integral = max(0.01, q_integral * 0.9)
                elif ch == b't':
                    tracking_enabled = not tracking_enabled
                    print(f"\n[Toggle] Setpoint tracking: {'ON' if tracking_enabled else 'OFF'}")
                elif ch == b'g':
                    gain_mode = 1 - gain_mode
                    print(f"\n[Toggle] Gain mode: {'Custom LQI' if gain_mode else 'Factory'}")
                elif ch == b'r':
                    q_theta, q_alpha, r_weight, q_integral = 2.0, 35.0, 1.0, 0.5
                    print("\n[Reset] Weights reset to defaults.")
                elif ch in (b'q', b'\x1b'):
                    running = False

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\n[!] Interrupted by user.")
    finally:
        running = False
        thread.join(timeout=1.0)
        print("\n[+] Shut down cleanly. Voltage reset to 0.0 V.")

        if log_buffer:
            import csv
            log_path = Path(__file__).parent / "lqr_run_log.csv"
            print(f"[*] Saving {len(log_buffer)} telemetry rows to '{log_path}'...")
            try:
                with open(log_path, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        'time_s', 'theta_rad', 'alpha_rad',
                        'theta_dot_rads', 'alpha_dot_rads', 'voltage_V', 'gain_mode',
                    ])
                    writer.writerows(log_buffer)
                print("[+] Telemetry saved.")
            except Exception as exc:
                print(f"[-] Error saving telemetry: {exc}")


if __name__ == "__main__":
    main()

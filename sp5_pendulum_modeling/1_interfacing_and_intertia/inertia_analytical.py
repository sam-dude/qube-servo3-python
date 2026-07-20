"""Analytical Calculation of Pendulum Moment of Inertia.

Models the pendulum rod as a uniform thin rod pivoting at one end and applies
the standard formula to compute J_p analytically from physical parameters.

Formula:
    J_p = (1/3) * m_p * L_p²

The result is compared against the value in modeling/constants.py (if importable)
to confirm the constants file matches the user's measured hardware parameters.

Reference:
  Quanser sp5 Application Guide — Interfacing and Inertia
  https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp5_pendulum_modeling/1_interfacing_and_inertia/
"""

from __future__ import annotations

import sys
import numpy as np
from pathlib import Path


def calculate_inertia(mass: float, length: float) -> float:
    """Moment of inertia of a uniform thin rod pivoting at one end.

    Args:
        mass:   Rod mass in kg.
        length: Rod total length in m.

    Returns:
        J_p in kg·m².
    """
    return (1.0 / 3.0) * mass * (length ** 2)


def main() -> None:
    print("==================================================")
    print("  sp5.1a: Pendulum Inertia — Analytical           ")
    print("==================================================")
    print("Model: uniform thin rod, pivot at one end.")
    print("  J_p = (1/3) * m_p * L_p²")
    print("--------------------------------------------------")

    # Mass
    print("\nStep 1: Enter the pendulum mass.")
    print("  Default (QUBE-Servo 3 User Manual): 24 g  (0.024 kg)")
    try:
        raw = input("  Pendulum mass in kg [default 0.024]: ").strip()
        mass = float(raw) if raw else 0.024
    except ValueError:
        print("  [-] Invalid input. Using 0.024 kg.")
        mass = 0.024

    # Length
    print("\nStep 2: Enter the pendulum length.")
    print("  Default (QUBE-Servo 3 User Manual): 12.9 cm  (0.129 m)")
    try:
        raw = input("  Pendulum length in m [default 0.129]: ").strip()
        length = float(raw) if raw else 0.129
    except ValueError:
        print("  [-] Invalid input. Using 0.129 m.")
        length = 0.129

    com = length / 2.0
    J_p = calculate_inertia(mass, length)

    print("\n--------------------------------------------------")
    print("  Calculation Summary")
    print("--------------------------------------------------")
    print(f"  m_p          : {mass:.4f} kg  ({mass * 1000:.1f} g)")
    print(f"  L_p          : {length:.4f} m   ({length * 100:.1f} cm)")
    print(f"  l = L_p/2   : {com:.4f} m   ({com * 100:.1f} cm)")
    print(f"\n  J_p = (1/3) * {mass:.4f} * ({length:.4f})²")
    print(f"      = (1/3) * {mass:.4f} * {length**2:.6f}")
    print(f"      = {J_p:.8f} kg·m²  ({J_p * 1e6:.3f} μkg·m²)")

    # Compare with shared constants
    print("\n--------------------------------------------------")
    print("  Comparison with modeling/constants.py")
    print("--------------------------------------------------")
    _repo_root = str(Path(__file__).resolve().parents[3])
    if _repo_root not in sys.path:
        sys.path.insert(0, _repo_root)
    try:
        from modeling.constants import J_p as system_J_p
        diff = abs(J_p - system_J_p)
        print(f"  System J_p   : {system_J_p:.8f} kg·m²")
        print(f"  Your J_p     : {J_p:.8f} kg·m²")
        print(f"  Difference   : {diff:.2e} kg·m²")
        if diff < 1e-9:
            print("  [+] Exact match with repository defaults.")
        else:
            print("  [!] Parameters differ from repository defaults.")
            print("      If you measured different hardware, update modeling/constants.py.")
    except Exception as exc:
        print(f"  [-] Could not import system constants: {exc}")

    print("\nPress Enter to exit.")
    input()


if __name__ == "__main__":
    main()

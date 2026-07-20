"""State-Space Symbolic Derivation for the Furuta Pendulum.

Derives the linearised A and B matrices for the QUBE-Servo 3 pendulum system
symbolically, starting from the coupled equations of motion.

Two paths:
  - If SymPy is installed: solves the coupled EOM system symbolically and
    differentiates to extract A and B entries automatically.
  - If SymPy is absent: prints the pre-derived analytical steps and the
    closed-form matrix expressions.

The linearisation is around the downward equilibrium (alpha = 0, hanging).
The upward-balance linearisation (used by the LQR controller) uses the same
structural form with a sign change on the gravity term; see state_space_model.py.

State vector: x = [theta, alpha, theta_dot, alpha_dot]^T
Input:        u = v_m  (motor voltage, V)
Output:       y = [theta, alpha]^T

Equations of motion linearised at downward equilibrium:
  (1)  J_r * dd_theta + m_p*l*r * dd_alpha = tau - b_r * d_theta
  (2)  J_p * dd_alpha + m_p*l*r * dd_theta + m_p*g*l * alpha = -b_p * d_alpha

where tau = (k_t/R_m)*(v_m - k_m*d_theta), giving effective arm damping
  D_arm = b_r + k_t*k_m/R_m

Reference:
  Quanser sp5 Application Guide — State-Space Modeling
  https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp5_pendulum_modeling/2_state_space_modeling/
"""

from __future__ import annotations

import sys


def print_derivation_steps() -> None:
    """Print pre-derived analytical derivation steps (no SymPy required)."""
    print("--------------------------------------------------")
    print("          Mathematical Derivation Steps           ")
    print("--------------------------------------------------")
    print("The linear equations of motion at downward equilibrium are:")
    print("  (1) J_r * dd_theta + m_p*l*r * dd_alpha = tau - b_r * d_theta")
    print("  (2) J_p * dd_alpha + m_p*l*r * dd_theta + m_p*g*l * alpha = -b_p * d_alpha")
    print("\nWhere:")
    print("  dd_theta, dd_alpha : Angular accelerations")
    print("  d_theta,  d_alpha  : Angular velocities")
    print("  theta,    alpha    : Angular positions")
    print("  tau                : Motor torque")
    print("  r                  : Arm length (L_r)")
    print("  l                  : Pendulum COM distance (L_p / 2)")
    print("  J_r                : Total arm inertia")
    print("  J_p                : Pendulum inertia about its pivot")

    print("\nStep 1: Express torque tau in terms of motor voltage v_m.")
    print("  tau = (k_t/R_m) * (v_m - k_m * d_theta)")
    print("  Substituting into (1):")
    print("  (1') J_r * dd_theta + m_p*l*r * dd_alpha = (k_t/R_m)*v_m - D_arm * d_theta")
    print("  where D_arm = b_r + k_t*k_m/R_m  (effective arm damping)")

    print("\nStep 2: Write in matrix form: M*q_dd + D*q_d + G*q = B_vm*v_m")
    print("  M = [[ J_r,       m_p*l*r ],")
    print("       [ m_p*l*r,   J_p     ]]")
    print("\n  D = [[ D_arm,  0    ],")
    print("       [   0,    b_p  ]]")
    print("\n  G = [[  0,         0      ],")
    print("       [  0,    m_p*g*l     ]]")
    print("\n  B_vm = [ k_t/R_m,  0 ]^T")

    print("\nStep 3: Solve for accelerations. Let det = J_r*J_p - (m_p*l*r)^2")
    print("  M_inv = (1/det) * [[ J_p,      -m_p*l*r ],")
    print("                     [ -m_p*l*r,   J_r    ]]")
    print("  q_dd = M_inv * (-D*q_d - G*q + B_vm*v_m)")

    print("\nStep 4: State vector x = [theta, alpha, d_theta, d_alpha]^T")
    print("  dx/dt = A*x + B*v_m")
    print("\n  A rows 1-2: identity coupling (theta_dot -> theta, alpha_dot -> alpha)")
    print("  A row 3: [ 0,  (m_p^2*g*l^2*r)/det,  -J_p*D_arm/det,  (m_p*l*r*b_p)/det ]")
    print("  A row 4: [ 0,  -(m_p*g*l*J_r)/det,   (m_p*l*r*D_arm)/det,  -J_r*b_p/det ]")
    print("\n  B = [ 0,  0,  (J_p*k_t/R_m)/det,  -(m_p*l*r*k_t/R_m)/det ]^T")

    print("\nStep 5: Output matrix (positions only)")
    print("  C = [[ 1, 0, 0, 0 ],")
    print("       [ 0, 1, 0, 0 ]]")
    print("  D = [[ 0 ], [ 0 ]]")
    print("--------------------------------------------------")


def run_symbolic() -> None:
    """Derive A and B symbolically via SymPy, or fall back to printed steps."""
    try:
        import sympy as sp
        print("[+] SymPy detected. Performing symbolic derivation...")

        # Symbols
        J_r, J_p, m_p, l, r, g, b_r, b_p, k_t, R_m, k_m, v_m = sp.symbols(
            'J_r J_p m_p l r g b_r b_p k_t R_m k_m v_m'
        )
        theta, alpha, d_theta, d_alpha = sp.symbols('theta alpha d_theta d_alpha')
        dd_theta, dd_alpha = sp.symbols('dd_theta dd_alpha')

        D_arm = b_r + k_t * k_m / R_m

        # Coupled EOM
        eq1 = sp.Equality(
            J_r * dd_theta + m_p * l * r * dd_alpha,
            -D_arm * d_theta + (k_t / R_m) * v_m
        )
        eq2 = sp.Equality(
            J_p * dd_alpha + m_p * l * r * dd_theta,
            -b_p * d_alpha - m_p * g * l * alpha
        )

        sol = sp.solve((eq1, eq2), (dd_theta, dd_alpha))
        dd_theta_expr = sp.simplify(sol[dd_theta])
        dd_alpha_expr = sp.simplify(sol[dd_alpha])

        print("\nDerived acceleration expressions:")
        print("  dd_theta =")
        sp.pprint(dd_theta_expr)
        print("\n  dd_alpha =")
        sp.pprint(dd_alpha_expr)

        # Build A and B by differentiation
        A_sym = sp.Matrix([
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [dd_theta_expr.diff(theta), dd_theta_expr.diff(alpha),
             dd_theta_expr.diff(d_theta), dd_theta_expr.diff(d_alpha)],
            [dd_alpha_expr.diff(theta), dd_alpha_expr.diff(alpha),
             dd_alpha_expr.diff(d_theta), dd_alpha_expr.diff(d_alpha)],
        ])
        B_sym = sp.Matrix([
            [0],
            [0],
            [dd_theta_expr.diff(v_m)],
            [dd_alpha_expr.diff(v_m)],
        ])

        print("\n=== SYMBOLIC A MATRIX ===")
        sp.pprint(A_sym)
        print("\n=== SYMBOLIC B MATRIX ===")
        sp.pprint(B_sym)

    except ImportError:
        print("[-] SymPy not installed. Showing pre-derived analytical steps.")
        print_derivation_steps()


def main() -> None:
    print("==================================================")
    print("   sp5.2: State-Space Symbolic Derivation         ")
    print("==================================================")
    run_symbolic()
    print("\nPress Enter to exit.")
    input()


if __name__ == "__main__":
    main()

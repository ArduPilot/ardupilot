#!/usr/bin/env python3
"""
LQR Gain Calculator for ArduSub State Feedback Control

This tool calculates optimal state feedback gains using Linear Quadratic Regulator (LQR)
theory for ArduSub's state feedback control system. It supports:
- Rate loop (3 states: roll, pitch, yaw rates)
- Attitude loop (6 states: angles + rates)
- Position loop (12 states: position, velocity, angles, rates)

Usage:
    # Rate loop gains
    python3 lqr_gain_calculator.py --loop rate --Ixx 0.15 --Iyy 0.15 --Izz 0.25 \
                                   --Dx 0.5 --Dy 0.5 --Dz 0.3 \
                                   --Q 10,10,10 --R 1,1,1

    # Attitude loop gains
    python3 lqr_gain_calculator.py --loop attitude --Ixx 0.15 --Iyy 0.15 --Izz 0.25 \
                                   --Dx 0.5 --Dy 0.5 --Dz 0.3 \
                                   --Q 100,100,100,10,10,10 --R 1,1,1

    # Position loop gains
    python3 lqr_gain_calculator.py --loop position --mass 10.5 --Ixx 0.15 --Iyy 0.15 --Izz 0.25 \
                                   --Dx 0.5 --Dy 0.5 --Dz 0.3 --Dtx 5.0 --Dty 5.0 --Dtz 8.0 \
                                   --Q 10,10,10,1,1,1,100,100,100,10,10,10 --R 0.1,1,1,1

Author: ArduPilot Development Team
License: GPLv3
"""

import argparse
import numpy as np
from scipy import linalg
import sys

def parse_vector(s, expected_length, name):
    """Parse comma-separated string into numpy array"""
    try:
        values = [float(x) for x in s.split(',')]
        if len(values) != expected_length:
            raise ValueError(f"{name} must have {expected_length} values, got {len(values)}")
        return np.array(values)
    except ValueError as e:
        print(f"Error parsing {name}: {e}", file=sys.stderr)
        sys.exit(1)

def build_rate_loop_system(Ixx, Iyy, Izz, Dx, Dy, Dz, dt):
    """
    Build state-space matrices for rate loop (3 states: p, q, r)

    Continuous dynamics:
        ẋ = Ax + Bu
        A = diag([-Dx/Ixx, -Dy/Iyy, -Dz/Izz])  (damping)
        B = diag([1/Ixx, 1/Iyy, 1/Izz])         (control effectiveness)

    Discretize using zero-order hold
    """
    # Continuous system
    A_c = np.diag([-Dx/Ixx, -Dy/Iyy, -Dz/Izz])
    B_c = np.diag([1.0/Ixx, 1.0/Iyy, 1.0/Izz])

    # Discretize using matrix exponential (zero-order hold)
    n = A_c.shape[0]
    m = B_c.shape[1]

    # Build augmented matrix for discretization
    # M = [A  B]
    #     [0  0]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A_c * dt
    M[:n, n:] = B_c * dt

    # Compute matrix exponential
    expM = linalg.expm(M)

    # Extract discretized matrices
    A_d = expM[:n, :n]
    B_d = expM[:n, n:]

    return A_d, B_d

def build_attitude_loop_system(Ixx, Iyy, Izz, Dx, Dy, Dz, dt):
    """
    Build state-space matrices for attitude loop (6 states: φ, θ, ψ, p, q, r)

    Continuous dynamics (small angle approximation):
        A = [[0, 0, 0,  1,  0,  0],     # φ̇ = p
             [0, 0, 0,  0,  1,  0],     # θ̇ = q
             [0, 0, 0,  0,  0,  1],     # ψ̇ = r
             [0, 0, 0, -Dx/Ixx, 0,  0], # ṗ (damping)
             [0, 0, 0,  0, -Dy/Iyy, 0], # q̇
             [0, 0, 0,  0,  0, -Dz/Izz]]# ṙ

        B = [[0,      0,      0     ],
             [0,      0,      0     ],
             [0,      0,      0     ],
             [1/Ixx,  0,      0     ],
             [0,      1/Iyy,  0     ],
             [0,      0,      1/Izz]]
    """
    A_c = np.array([
        [0, 0, 0,  1,      0,      0     ],
        [0, 0, 0,  0,      1,      0     ],
        [0, 0, 0,  0,      0,      1     ],
        [0, 0, 0, -Dx/Ixx, 0,      0     ],
        [0, 0, 0,  0,     -Dy/Iyy, 0     ],
        [0, 0, 0,  0,      0,     -Dz/Izz]
    ])

    B_c = np.array([
        [0,      0,      0     ],
        [0,      0,      0     ],
        [0,      0,      0     ],
        [1/Ixx,  0,      0     ],
        [0,      1/Iyy,  0     ],
        [0,      0,      1/Izz]
    ])

    # Discretize
    n = A_c.shape[0]
    m = B_c.shape[1]

    M = np.zeros((n + m, n + m))
    M[:n, :n] = A_c * dt
    M[:n, n:] = B_c * dt

    expM = linalg.expm(M)

    A_d = expM[:n, :n]
    B_d = expM[:n, n:]

    return A_d, B_d

def build_position_loop_system(mass, Ixx, Iyy, Izz, Dx, Dy, Dz, Dtx, Dty, Dtz, dt):
    """
    Build state-space matrices for position loop (12 states)
    States: x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r

    Simplified dynamics with tilt-to-translate coupling:
    - Translational: ẍ = -Dtx*vx/mass + g*θ, ÿ = -Dty*vy/mass - g*φ, z̈ = -Dtz*vz/mass + Tz/mass
    - Rotational: same as attitude loop

    Control: u = [Tz, τ_roll, τ_pitch, τ_yaw]
    """
    g = 9.81  # gravity

    A_c = np.zeros((12, 12))

    # Position derivatives
    A_c[0, 3] = 1.0  # ẋ = vx
    A_c[1, 4] = 1.0  # ẏ = vy
    A_c[2, 5] = 1.0  # ż = vz

    # Velocity derivatives (damping + gravity coupling)
    A_c[3, 3] = -Dtx/mass      # v̇x damping
    A_c[3, 7] = g               # v̇x = g*θ (pitch angle)
    A_c[4, 4] = -Dty/mass      # v̇y damping
    A_c[4, 6] = -g              # v̇y = -g*φ (roll angle)
    A_c[5, 5] = -Dtz/mass      # v̇z damping

    # Attitude dynamics (same as attitude loop)
    A_c[6, 9] = 1.0             # φ̇ = p
    A_c[7, 10] = 1.0            # θ̇ = q
    A_c[8, 11] = 1.0            # ψ̇ = r
    A_c[9, 9] = -Dx/Ixx        # ṗ damping
    A_c[10, 10] = -Dy/Iyy      # q̇ damping
    A_c[11, 11] = -Dz/Izz      # ṙ damping

    # Control matrix
    B_c = np.zeros((12, 4))
    B_c[5, 0] = 1.0/mass       # Vertical thrust
    B_c[9, 1] = 1.0/Ixx        # Roll torque
    B_c[10, 2] = 1.0/Iyy       # Pitch torque
    B_c[11, 3] = 1.0/Izz       # Yaw torque

    # Discretize
    n = A_c.shape[0]
    m = B_c.shape[1]

    M = np.zeros((n + m, n + m))
    M[:n, :n] = A_c * dt
    M[:n, n:] = B_c * dt

    expM = linalg.expm(M)

    A_d = expM[:n, :n]
    B_d = expM[:n, n:]

    return A_d, B_d

def solve_dare_lqr(A, B, Q, R):
    """
    Solve Discrete Algebraic Riccati Equation (DARE) and compute LQR gain

    DARE: P = A'*P*A - (A'*P*B)*inv(R + B'*P*B)*(B'*P*A) + Q
    Gain: K = inv(R + B'*P*B)*(B'*P*A)

    Returns: K (gain matrix), P (solution to DARE)
    """
    try:
        # Solve DARE using scipy
        P = linalg.solve_discrete_are(A, B, Q, R)

        # Compute LQR gain
        K = linalg.solve(R + B.T @ P @ B, B.T @ P @ A)

        return K, P
    except Exception as e:
        print(f"Error solving DARE: {e}", file=sys.stderr)
        print("This may indicate:")
        print("  - System is not controllable")
        print("  - Q matrix is not positive semi-definite")
        print("  - R matrix is not positive definite")
        sys.exit(1)

def check_controllability(A, B):
    """Check if system (A, B) is controllable"""
    n = A.shape[0]

    # Build controllability matrix: [B, AB, A²B, ..., A^(n-1)B]
    C = B
    for i in range(1, n):
        C = np.hstack([C, np.linalg.matrix_power(A, i) @ B])

    rank = np.linalg.matrix_rank(C)
    return rank == n, rank

def generate_param_commands(K, loop_type):
    """Generate ArduPilot parameter set commands"""
    commands = []

    if loop_type == 'rate':
        # 3×3 gain matrix
        param_names = [
            'SF_R_K1', 'SF_R_K2', 'SF_R_K3',  # Row 0: roll control
            'SF_R_K4', 'SF_R_K5', 'SF_R_K6',  # Row 1: pitch control
            'SF_R_K7', 'SF_R_K8', 'SF_R_K9'   # Row 2: yaw control
        ]
        K_flat = K.flatten()

    elif loop_type == 'attitude':
        # 3×6 gain matrix
        param_names = [
            'SF_A_K1', 'SF_A_K2', 'SF_A_K3', 'SF_A_K4', 'SF_A_K5', 'SF_A_K6',    # Row 0
            'SF_A_K7', 'SF_A_K8', 'SF_A_K9', 'SF_A_K10', 'SF_A_K11', 'SF_A_K12', # Row 1
            'SF_A_K13', 'SF_A_K14', 'SF_A_K15', 'SF_A_K16', 'SF_A_K17', 'SF_A_K18' # Row 2
        ]
        K_flat = K.flatten()

    elif loop_type == 'position':
        # 4×12 gain matrix
        param_names = []
        for i in range(1, 49):  # 48 parameters
            param_names.append(f'SF_P_K{i}')
        K_flat = K.flatten()

    for name, value in zip(param_names, K_flat):
        commands.append(f"param set {name} {value:.6f}")

    return commands

def main():
    parser = argparse.ArgumentParser(
        description='Calculate LQR gains for ArduSub state feedback control',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--loop', required=True, choices=['rate', 'attitude', 'position'],
                       help='Control loop to calculate gains for')

    # Vehicle parameters
    parser.add_argument('--Ixx', type=float, required=True, help='Roll moment of inertia (kg·m²)')
    parser.add_argument('--Iyy', type=float, required=True, help='Pitch moment of inertia (kg·m²)')
    parser.add_argument('--Izz', type=float, required=True, help='Yaw moment of inertia (kg·m²)')
    parser.add_argument('--Dx', type=float, required=True, help='Roll damping (Nm/(rad/s))')
    parser.add_argument('--Dy', type=float, required=True, help='Pitch damping (Nm/(rad/s))')
    parser.add_argument('--Dz', type=float, required=True, help='Yaw damping (Nm/(rad/s))')

    # Position loop only
    parser.add_argument('--mass', type=float, help='Vehicle mass (kg) - position loop only')
    parser.add_argument('--Dtx', type=float, help='X-axis translational damping (N/(m/s)) - position loop only')
    parser.add_argument('--Dty', type=float, help='Y-axis translational damping (N/(m/s)) - position loop only')
    parser.add_argument('--Dtz', type=float, help='Z-axis translational damping (N/(m/s)) - position loop only')

    # LQR weights
    parser.add_argument('--Q', required=True,
                       help='State weight matrix (comma-separated diagonal elements)')
    parser.add_argument('--R', required=True,
                       help='Control weight matrix (comma-separated diagonal elements)')

    # Discretization
    parser.add_argument('--dt', type=float, default=0.0025,
                       help='Sample time (seconds), default=0.0025 (400Hz)')

    # Output
    parser.add_argument('--output', '-o', help='Output file for parameter commands (default: stdout)')

    args = parser.parse_args()

    # Validate position loop parameters
    if args.loop == 'position':
        if args.mass is None or args.Dtx is None or args.Dty is None or args.Dtz is None:
            parser.error("Position loop requires --mass, --Dtx, --Dty, --Dtz")

    # Parse weight matrices
    if args.loop == 'rate':
        Q_diag = parse_vector(args.Q, 3, 'Q')
        R_diag = parse_vector(args.R, 3, 'R')
    elif args.loop == 'attitude':
        Q_diag = parse_vector(args.Q, 6, 'Q')
        R_diag = parse_vector(args.R, 3, 'R')
    elif args.loop == 'position':
        Q_diag = parse_vector(args.Q, 12, 'Q')
        R_diag = parse_vector(args.R, 4, 'R')

    Q = np.diag(Q_diag)
    R = np.diag(R_diag)

    # Build system matrices
    print(f"Building {args.loop} loop system model...", file=sys.stderr)
    if args.loop == 'rate':
        A, B = build_rate_loop_system(args.Ixx, args.Iyy, args.Izz, args.Dx, args.Dy, args.Dz, args.dt)
    elif args.loop == 'attitude':
        A, B = build_attitude_loop_system(args.Ixx, args.Iyy, args.Izz, args.Dx, args.Dy, args.Dz, args.dt)
    elif args.loop == 'position':
        A, B = build_position_loop_system(args.mass, args.Ixx, args.Iyy, args.Izz,
                                         args.Dx, args.Dy, args.Dz,
                                         args.Dtx, args.Dty, args.Dtz, args.dt)

    # Check controllability
    print("Checking controllability...", file=sys.stderr)
    is_controllable, rank = check_controllability(A, B)
    if not is_controllable:
        print(f"WARNING: System is not controllable (rank={rank}, need {A.shape[0]})", file=sys.stderr)
        print("This may result in poor performance or instability.", file=sys.stderr)
    else:
        print("System is controllable ✓", file=sys.stderr)

    # Solve LQR
    print("Solving Discrete Algebraic Riccati Equation...", file=sys.stderr)
    K, P = solve_dare_lqr(A, B, Q, R)

    # Display results
    print("\n=== LQR Gain Matrix K ===", file=sys.stderr)
    print(K, file=sys.stderr)
    print(f"\nGain matrix shape: {K.shape}", file=sys.stderr)
    print(f"Max gain: {np.max(np.abs(K)):.6f}", file=sys.stderr)
    print(f"Min gain: {np.min(np.abs(K[K != 0])):.6f}", file=sys.stderr)

    # Generate parameter commands
    print("\n=== ArduPilot Parameter Commands ===", file=sys.stderr)
    commands = generate_param_commands(K, args.loop)

    # Output
    output_lines = []
    output_lines.append("# LQR Gain Parameters")
    output_lines.append(f"# Loop: {args.loop}")
    output_lines.append(f"# Vehicle: Ixx={args.Ixx}, Iyy={args.Iyy}, Izz={args.Izz}")
    output_lines.append(f"# Damping: Dx={args.Dx}, Dy={args.Dy}, Dz={args.Dz}")
    if args.loop == 'position':
        output_lines.append(f"# Mass={args.mass}, Dtx={args.Dtx}, Dty={args.Dty}, Dtz={args.Dtz}")
    output_lines.append(f"# Q weights: {args.Q}")
    output_lines.append(f"# R weights: {args.R}")
    output_lines.append(f"# Sample time: {args.dt}s")
    output_lines.append("")
    output_lines.extend(commands)

    if args.output:
        with open(args.output, 'w') as f:
            f.write('\n'.join(output_lines))
        print(f"\nParameter commands written to: {args.output}", file=sys.stderr)
    else:
        print()  # blank line
        for line in output_lines:
            print(line)

if __name__ == '__main__':
    main()

import numpy as np
import tinympc
import os
import shutil

# --- 1. Define Drone Model (Simplified Linearized Quadrotor) ---
# This is a VERY simplified double-integrator model for testing integration.
# Real world usage requires system identification or accurate physics modeling.
# States: [x, y, z, phi, theta, psi, u, v, w, p, q, r] (12 states)
# Inputs: [T1, T2, T3, T4] mapped to [Throttle, Roll, Pitch, Yaw] (4 inputs)

# For initial integration, we will use a decoupled model where inputs directly affect accelerations.
# dt = 0.01 (100Hz MPC loop, though PID is 8kHz, MPC runs slower usually)
dt = 0.02 # 50Hz

nx = 12
nu = 4
N = 10 # Horizon length

# Identity A matrix (discrete time double integrator roughly)
A = np.eye(nx)
# Position updates from velocity
A[0, 6] = dt
A[1, 7] = dt
A[2, 8] = dt
# Angle updates from angular rate
A[3, 9] = dt
A[4, 10] = dt
A[5, 11] = dt

B = np.zeros((nx, nu))
# Inputs affect velocities/rates directly (simplified)
# Throttle (z-accel)
B[8, 0] = dt * 10.0 # arbitrary gain
# Roll command (roll-accel)
B[9, 1] = dt * 20.0
# Pitch command
B[10, 2] = dt * 20.0
# Yaw command
B[11, 3] = dt * 10.0

# Q and R matrices (Costs)
Q = np.eye(nx) * 10
Q[0:3, 0:3] *= 10 # Higher cost on position error
R = np.eye(nu) * 0.1 # Lower cost on control effort

# State Constraints (Infinity for now for unconstrained)
x_min = -np.inf * np.ones(nx)
x_max = np.inf * np.ones(nx)

# Input Constraints (Normalized 0-1 or similar)
u_min = -1.0 * np.ones(nu)
u_max = 1.0 * np.ones(nu)

# --- 2. Setup TinyMPC ---
prob = tinympc.TinyMPC()
prob.setup(A, B, N, Q, R, x_min, x_max, u_min, u_max)

# --- 3. Generate Code ---
# Target Directory in Betaflight source
output_dir = os.path.join("src", "main", "flight", "tinympc", "generated_code")

if os.path.exists(output_dir):
    shutil.rmtree(output_dir)
os.makedirs(output_dir, exist_ok=True)

print(f"Generating code to {output_dir}...")
prob.codegen(output_dir)
print("Done!")

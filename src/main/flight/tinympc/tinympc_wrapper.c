#include "flight/tinympc/tinympc_wrapper.h"
#include "common/maths.h"

// Placeholder for generated solver state
typedef struct {
  float x[12]; // State vector
  float u[4];  // Control input
} tinympc_workspace_t;

static tinympc_workspace_t workspace;
static bool initialized = false;

void tinympcInit(void) {
  // partial setup of solver workspace
  // logic to load specific weights would go here
  initialized = true;
}

void tinympcReset(void) {
  for (int i = 0; i < 12; i++)
    workspace.x[i] = 0.0f;
  for (int i = 0; i < 4; i++)
    workspace.u[i] = 0.0f;
}

#include "generated_code/tinympc/tiny_api.h"

void tinympcUpdate(float dt, float *gyroRates, float *attitude,
                   float *setpointRates, float *controlOutput) {
  (void)dt;
  (void)setpointRates;

  if (!initialized) {
    tinympcInit();
  }

  // 1. Map State to TinyMPC
  // Zero out position/velocity for this simple test
  for (int i = 0; i < 12; i++)
    tiny_data_solver.x0[i] = 0;

  // Fill rates [p, q, r] -> indices 9, 10, 11
  // gyroRates is [roll, pitch, yaw]
  tiny_data_solver.x0[9] = gyroRates[FD_ROLL];
  tiny_data_solver.x0[10] = gyroRates[FD_PITCH];
  tiny_data_solver.x0[11] = gyroRates[FD_YAW];

  // Fill attitude [phi, theta, psi] -> indices 3, 4, 5
  // Note: BF attitude indices might differ from standard frame, ensuring
  // standard roll/pitch/yaw order here
  tiny_data_solver.x0[3] = attitude[FD_ROLL];
  tiny_data_solver.x0[4] = attitude[FD_PITCH];
  tiny_data_solver.x0[5] = attitude[FD_YAW];

  // 2. Solve
  tiny_solve();

  // 3. Map Output: u -> [Thrust, Roll, Pitch, Yaw]
  // Ideally this maps to Torque/Thrust. For PID sum equivalence:
  // We assume u[1]=Roll, u[2]=Pitch, u[3]=Yaw
  controlOutput[FD_ROLL] = tiny_data_solver.u[1];
  controlOutput[FD_PITCH] = tiny_data_solver.u[2];
  controlOutput[FD_YAW] = tiny_data_solver.u[3];
}

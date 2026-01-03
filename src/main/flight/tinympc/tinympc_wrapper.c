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

void tinympcUpdate(float dt, float *gyroRates, float *attitude,
                   float *setpointRates, float *controlOutput) {
  (void)dt;
  (void)attitude;

  if (!initialized) {
    tinympcInit();
  }

  // 1. State Estimation / Map BF state to TinyMPC state (x)
  // x = [pos, vel, orientation, rates] (example)
  // For ACRO we might just care about rates + attitude error

  // MOCK SOLVER: Simple P-controller behavior to prove connection
  // In real implementation, this calls tiny_solve()

  // Roll
  float errorRoll = setpointRates[FD_ROLL] - gyroRates[FD_ROLL];
  controlOutput[FD_ROLL] = errorRoll * 0.5f; // Random P-gain mock

  // Pitch
  float errorPitch = setpointRates[FD_PITCH] - gyroRates[FD_PITCH];
  controlOutput[FD_PITCH] = errorPitch * 0.5f;

  // Yaw
  float errorYaw = setpointRates[FD_YAW] - gyroRates[FD_YAW];
  controlOutput[FD_YAW] = errorYaw * 1.5f;

  // Throttle pass-through or Z-accel handling would happen here
}

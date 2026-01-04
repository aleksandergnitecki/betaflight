#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TARGET_BOARD_IDENTIFIER "TEST"

extern "C" {
#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "config/config.h"
#include "config/config_reset.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "flight/geometric/geometric_control.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "flight/position.h"
#include "io/gps.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "platform.h"
#include "rx/rx.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

// MOCKS
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

acc_t acc;
gyro_t gyro;
attitudeEulerAngles_t attitude;
rxRuntimeState_t rxRuntimeState = {};
}

pidProfile_t *pidProfile;

// Minimal test runner
void assert_true(bool condition, const char *msg) {
  if (!condition) {
    printf("FAIL: %s\n", msg);
    exit(1);
  } else {
    printf("PASS: %s\n", msg);
  }
}

void test_mpc_switching() {
  printf("Running test_mpc_switching...\n");

  printf("Resetting PG...\n");
  pgResetAll();

  printf("Getting mutable profile...\n");
  pidProfile = pidProfilesMutable(0); // Use profile 0
  if (!pidProfile) {
    printf("FATAL: pidProfile is NULL\n");
    exit(1);
  }
  printf("pidProfile @ %p\n", pidProfile);

  printf("Clearing runtime...\n");
  memset(&pidRuntime, 0, sizeof(pidRuntime));

  // Initialize some globals expected by pidInit
  gyro.targetLooptime = 125; // 8kHz

  printf("Initializing PID...\n");
  pidInit(pidProfile);

  // 1. Test Disabled
  printf("Setting geometric_enabled=0\n");
  pidProfile->geometric_enabled = 0;
  // Need to initialize PID to set up default state logic if any
  // For switching test, we primarily care about pidController logic branch

  printf("Setting PID data...\n");
  pidData[FD_ROLL].P = 10.0f;
  pidData[FD_ROLL].Sum = 100.0f;

  printf("Enabling stabilization...\n");
  pidStabilisationState(PID_STABILISATION_ON);

  // Call controller
  // We need dt? pidController uses currentTestTime() in mocked one.
  // In real code it uses currentTimeUs.
  printf("Calling pidController (Time 0)...\n");
  pidController(pidProfile, 0);

  // If MPC disabled, PID logic should run.
  // Since we mocked inputs to 0, P/I/D might change but shouldn't be strictly
  // zeroed out by the "MPC active" block. Actually, with 0 inputs, PID might
  // result in 0. Let's set some error to ensure PID produces output
  printf("Setting gyro...\n");
  gyro.gyroADCf[FD_ROLL] = 100.0f;

  printf("Calling pidController (Time 1000)...\n");
  pidController(pidProfile, 1000);

  // Standard PID should have reacted to gyro error
  // check if P term is non-zero
  printf("Checking non-zero response...\n");
  if (pidData[FD_ROLL].P == 0.0f) {
    printf("WARN: P term is zero\n");
  }

  // 2. Enable Geometric Control
  printf("Setting geometric_enabled=1\n");
  pidProfile->geometric_enabled = 1;

  // Run controller
  printf("Calling pidController (Time 2000)...\n");
  pidController(pidProfile, 2000);

  // Geometric Controller default implementation:
  // u = -GEOMETRIC_KW * error_w
  // gyro=100, setpoint=0 (mocked) => error = 100
  // u = -0.3 * 100 = -30.0 for Roll/Pitch, -0.1 * 100 = -10 for Yaw
  // Check Roll (X)

  assert_true(pidData[FD_ROLL].P == 0.0f,
              "Geometric Enabled: P term should be cleared to 0");
  assert_true(pidData[FD_ROLL].I == 0.0f,
              "Geometric Enabled: I term should be cleared to 0");

  // Check Sum (output)
  // Expected: -30.0f
  printf("Geometric Output Roll: %f\n", pidData[FD_ROLL].Sum);
  assert_true(fabsf(pidData[FD_ROLL].Sum - (-30.0f)) < 0.01f,
              "Geometric Enabled: Sum should match expected output (-30.0)");
}

int main(void) {
  test_mpc_switching(); // Renaming function is optional but good practice
  printf("All tests passed.\n");
  return 0;
}

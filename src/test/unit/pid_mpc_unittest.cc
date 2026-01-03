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
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "flight/position.h"
#include "flight/tinympc/tinympc_wrapper.h"
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

// PG Mocks
// We can't easily rely on real PG macros without linker support for sections,
// likely simpler to just define the structs and point pointers to them if
// possible, or rely on PG_REGISTER if the linker script is used. Since we
// manually link, PG might fail if it relies on linker sections. However,
// usually PG_REGISTER just defines a variable.
PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG,
            0);
PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
PG_REGISTER(positionConfig_t, positionConfig, PG_SYSTEM_CONFIG, 4);

bool unitLaunchControlActive = false;
float simulatedMotorMixRange = 0.0f;
float simulatedSetpointRate[3] = {0, 0, 0};
bool simulatedThrottleRaised = false;
float simulatedRcDeflection[3] = {0, 0, 0};
float simulatedMaxRcDeflectionAbs = 0;
float simulatedMaxRate[3] = {670, 670, 670};
float simulatedPrevSetpointRate[3] = {0, 0, 0};
float simulatedMixerGetRcThrottle = 0;

float getMotorMixRange(void) { return simulatedMotorMixRange; }
float getSetpointRate(int axis) { return simulatedSetpointRate[axis]; }
bool wasThrottleRaised(void) { return simulatedThrottleRaised; }
float getRcDeflectionAbs(int axis) {
  return fabsf(simulatedRcDeflection[axis]);
}
float getMaxRcDeflectionAbs() { return fabsf(simulatedMaxRcDeflectionAbs); }
float mixerGetRcThrottle() { return fabsf(simulatedMixerGetRcThrottle); }
bool isBelowLandingAltitude(void) { return false; }
void systemBeep(bool) {}
bool gyroOverflowDetected(void) { return false; }
float getRcDeflection(int axis) { return simulatedRcDeflection[axis]; }
#define TARGET_BOARD_IDENTIFIER "TEST"
#include "platform.h"
float getRcDeflectionRaw(int axis) { return simulatedRcDeflection[axis]; }
float getRawSetpoint(int axis) {
  UNUSED(axis);
  return 0;
}
float getFeedforward(int axis) {
  return simulatedSetpointRate[axis] - simulatedPrevSetpointRate[axis];
}
void beeperConfirmationBeeps(uint8_t) {}
bool isLaunchControlActive(void) { return unitLaunchControlActive; }
void disarm(flightLogDisarmReason_e) {}
float getMaxRcRate(int axis) {
  UNUSED(axis);
  return simulatedMaxRate[axis];
}
void initRcProcessing(void) {}
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
  printf("Setting mpc_enabled=0\n");
  pidProfile->mpc_enabled = 0;
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

  // 2. Enable MPC
  printf("Setting mpc_enabled=1\n");
  pidProfile->mpc_enabled = 1;

  // Run controller
  printf("Calling pidController (Time 2000)...\n");
  pidController(pidProfile, 2000);

  // MPC wrapper mock (tinympc_wrapper.c) sets:
  // controlOutput[FD_ROLL] = errorRoll * 0.5f;
  // where errorRoll = setpoint - gyro
  // setpoint=0, gyro=100 => error=-100 => output=-50
  // And it clears P/I/D struct members.

  assert_true(pidData[FD_ROLL].P == 0.0f,
              "MPC Enabled: P term should be cleared to 0");
  assert_true(pidData[FD_ROLL].I == 0.0f,
              "MPC Enabled: I term should be cleared to 0");

  // Check Sum (output)
  // Mock wrapper: output = (0 - 100) * 0.5 = -50.
  printf("MPC Output Roll: %f\n", pidData[FD_ROLL].Sum);
  assert_true(fabsf(pidData[FD_ROLL].Sum - (-50.0f)) < 0.01f,
              "MPC Enabled: Sum should match mock solver output");
}

int main(void) {
  test_mpc_switching();
  printf("All tests passed.\n");
  return 0;
}

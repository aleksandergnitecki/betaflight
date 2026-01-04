#include <math.h>
#include <string.h>

#include "common/axis.h"
#include "common/maths.h"
#include "flight/geometric/geometric_control.h"

// Inertia Matrix (kg.m^2) - Tuned for generic 5" Racing Drone
// These are diagonal elements Jxx, Jyy, Jzz
#define INERTIA_J_X 0.003f
#define INERTIA_J_Y 0.003f
#define INERTIA_J_Z 0.005f

// Torque to Motor Mix Scaling Factor (Tunable)
// This converts N.m torque to Betaflight's arbitrary motor mix units
#define TORQUE_TO_MIX_SCALE 20000.0f

static bool initialized = false;

void geometricInit(void) { initialized = true; }

void geometricReset(void) {
  // Reset any internal state if needed
}

// Convert Euler to Rotation Matrix (Helper, currently unused for Rate Control)
static void __attribute__((unused))
eulerToRotationMatrix(float roll, float pitch, float yaw, float R[3][3]) {
  float cr = cosf(roll);
  float sr = sinf(roll);
  float cp = cosf(pitch);
  float sp = sinf(pitch);
  float cy = cosf(yaw);
  float sy = sinf(yaw);

  // Rz * Ry * Rx
  R[0][0] = cy * cp;
  R[0][1] = cy * sp * sr - sy * cr;
  R[0][2] = cy * sp * cr + sy * sr;

  R[1][0] = sy * cp;
  R[1][1] = sy * sp * sr + cy * cr;
  R[1][2] = sy * sp * cr - cy * sr;

  R[2][0] = -sp;
  R[2][1] = cp * sr;
  R[2][2] = cp * cr;
}

// Vee map (Helper)
static void __attribute__((unused)) veeMap(float S[3][3], float v[3]) {
  v[0] = S[2][1]; // x
  v[1] = S[0][2]; // y
  v[2] = S[1][0]; // z
}

void geometricUpdate(float dt, float *gyroRates, float *attitudeEuler,
                     float *setpointRates, float *controlOutput) {
  if (!initialized)
    geometricInit();
  (void)dt;
  (void)attitudeEuler; // Not used for Rate Control

  // 1. Angular Velocity (Omega) in rad/s
  // Betaflight gyro is typically in deg/s. We need rad/s for physics math.
  float omega[3];
  omega[FD_ROLL] = DEGREES_TO_RADIANS(gyroRates[FD_ROLL]);
  omega[FD_PITCH] = DEGREES_TO_RADIANS(gyroRates[FD_PITCH]);
  omega[FD_YAW] = DEGREES_TO_RADIANS(gyroRates[FD_YAW]);

  // 2. Setpoint Velocity (Omega_d) in rad/s
  float omega_d[3];
  omega_d[FD_ROLL] = DEGREES_TO_RADIANS(setpointRates[FD_ROLL]);
  omega_d[FD_PITCH] = DEGREES_TO_RADIANS(setpointRates[FD_PITCH]);
  omega_d[FD_YAW] = DEGREES_TO_RADIANS(setpointRates[FD_YAW]);

  // 3. Error (e_w = omega - omega_d)
  float e_w[3];
  e_w[FD_ROLL] = omega[FD_ROLL] - omega_d[FD_ROLL];
  e_w[FD_PITCH] = omega[FD_PITCH] - omega_d[FD_PITCH];
  e_w[FD_YAW] = omega[FD_YAW] - omega_d[FD_YAW];

  // 4. Gyroscopic Feedforward: w x J w
  // This term cancels the non-linear coupling of the drone's rotation.
  // Jw calculation
  float Jw[3];
  Jw[0] = INERTIA_J_X * omega[FD_ROLL];
  Jw[1] = INERTIA_J_Y * omega[FD_PITCH];
  Jw[2] = INERTIA_J_Z * omega[FD_YAW];

  // Cross Product: omega x Jw
  // cx = wy*Jz - wz*Jy
  // cy = wz*Jx - wx*Jz
  // cz = wx*Jy - wy*Jx
  // Note: omega indices [0]=ROLL, [1]=PITCH, [2]=YAW
  float w_x_Jw[3];
  w_x_Jw[0] = omega[FD_PITCH] * Jw[2] - omega[FD_YAW] * Jw[1];
  w_x_Jw[1] = omega[FD_YAW] * Jw[0] - omega[FD_ROLL] * Jw[2];
  w_x_Jw[2] = omega[FD_ROLL] * Jw[1] - omega[FD_PITCH] * Jw[0];

  // 5. Control Law: u = -K_w * e_w + (w x J w)
  // Note: The output 'u' here is Torque (N.m).
  // We convert Torque -> MotorMix units.

  float u_torque[3];
  u_torque[0] = -GEOMETRIC_KW_X * e_w[0] + w_x_Jw[0];
  u_torque[1] = -GEOMETRIC_KW_Y * e_w[1] + w_x_Jw[1];
  u_torque[2] = -GEOMETRIC_KW_Z * e_w[2] + w_x_Jw[2];

  // Output to Betaflight (Mixer Units)
  controlOutput[FD_ROLL] = u_torque[0] * TORQUE_TO_MIX_SCALE;
  controlOutput[FD_PITCH] = u_torque[1] * TORQUE_TO_MIX_SCALE;
  controlOutput[FD_YAW] = u_torque[2] * TORQUE_TO_MIX_SCALE;

  // Safety Clamping
  for (int i = 0; i < 3; i++) {
    if (controlOutput[i] > 2000.0f)
      controlOutput[i] = 2000.0f;
    if (controlOutput[i] < -2000.0f)
      controlOutput[i] = -2000.0f;
  }
}

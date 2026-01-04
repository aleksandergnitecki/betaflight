#pragma once

#include "platform.h"
#include <stdbool.h>
#include <stdint.h>

// Geometric Control Gains (Placeholder defaults)
// In a real system these would be in pidProfile
#define GEOMETRIC_KR_X 2.0f
#define GEOMETRIC_KR_Y 2.0f
#define GEOMETRIC_KR_Z 1.0f

#define GEOMETRIC_KW_X 0.3f
#define GEOMETRIC_KW_Y 0.3f
#define GEOMETRIC_KW_Z 0.1f

void geometricInit(void);
void geometricReset(void);
void geometricUpdate(float dt, float *gyroRates, float *attitudeEuler,
                     float *setpointRates, float *controlOutput);

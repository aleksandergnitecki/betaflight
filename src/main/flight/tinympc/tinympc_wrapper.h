#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"

// Initialize TinyMPC solver
void tinympcInit(void);

// Reset internal state (e.g. after disarm)
void tinympcReset(void);

// Main update step
// Takes current state (roll, pitch, yaw rates, angles) and references
// Returns motor control inputs (mixed or raw moments depending on
// implementation)
void tinympcUpdate(float dt, float *gyroRates, float *attitude,
                   float *setpointRates, float *controlOutput);

#ifndef TINY_API_H
#define TINY_API_H

#include "glob_opts.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  tiny_type x0[12]; // State: x, y, z, phi, theta, psi, u, v, w, p, q, r
  tiny_type u[4];   // Output: Thrust, Roll, Pitch, Yaw
} TinySolverData;

extern TinySolverData tiny_data_solver;

void tiny_solve(void);

#ifdef __cplusplus
}
#endif

#endif // TINY_API_H

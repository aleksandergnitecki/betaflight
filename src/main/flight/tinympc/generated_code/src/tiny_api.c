#include "../tinympc/tiny_api.h"

TinySolverData tiny_data_solver;

void tiny_solve(void) {
  // Mock Implementation for compilation
  // In a real solver, this would run ADMM optimization

  // Simple pass-through or P-controller logic for safety if flown by accident
  // Assuming x0 contains [error_pos..., error_angle..., error_vel...,
  // error_rate...] based on mapping

  // For now, valid outputs are 0
  tiny_data_solver.u[0] = 0;
  tiny_data_solver.u[1] = 0;
  tiny_data_solver.u[2] = 0;
  tiny_data_solver.u[3] = 0;
}

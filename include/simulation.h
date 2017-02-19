#ifndef GRAVITY_SIMULATION_H
#define GRAVITY_SIMULATION_H

namespace gravity {
	void init_particles(int num_particles);
	void run_simulation(int num_timesteps);

	void attract_particles();
	void step_particles();
}

#endif

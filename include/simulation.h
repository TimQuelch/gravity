/**
 * \file simulation.h
 * \brief Functions to run n-body simulation
 * \author Tim Quelch
 */

#ifndef GRAVITY_SIMULATION_H
#define GRAVITY_SIMULATION_H

namespace gravity {
	/**
	 * \brief Initialise particles to random positions
	 * \param num_particles The number of particles
	 */
	void init_particles(int num_particles);

	/**
	 * \brief Run the simulation
	 * \param num_timesteps Number of timesteps to run
	 */
	void run_simulation(int num_timesteps);

	/**
	 * \brief Attract all particles to each other
	 */
	void attract_particles();

	/**
	 * \brief Collide nearby particles
	 * Particles whose radii overlap will be collided, merging into a single particle. One of the
	 * particles in a collision has it's mass and velocity updated to conserve momentum, and the
	 * other is removed from the simulation
	 */
	void collide_particles();

	/**
	 * \brief Step all particles according to their velocity vector
	 */
	void step_particles();
}

#endif

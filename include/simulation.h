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
	 * \param numParticles The number of particles
	 */
	void initParticles(int numParticles);

	/**
	 * \brief Run the simulation
	 * \param numTimesteps Number of timesteps to run
	 */
	void runSimulation(int numTimesteps);

	/**
	 * \brief Attract all particles to each other
	 */
	void attractParticles();

	/**
	 * \brief Collide nearby particles
	 * Particles whose radii overlap will be collided, merging into a single particle. One of the
	 * particles in a collision has it's mass and velocity updated to conserve momentum, and the
	 * other is removed from the simulation
	 */
	void collideParticles();

	/**
	 * \brief Step all particles according to their velocity vector
	 */
	void stepParticles();
}

#endif

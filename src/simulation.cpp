#include "simulation.h"
#include "particle.h"
#include <chrono>
#include <functional>
#include <random>
#include <stdexcept>
#include <vector>

namespace gravity {
	namespace {
		std::vector<Particle> particles;
	}

	void initParticles(int numParticles) {
		if (numParticles <= 0) {
			throw std::invalid_argument{"Number of particles must be greater than 0"};
		}

		std::default_random_engine rng(std::chrono::system_clock::now().time_since_epoch().count());
		auto pos = [&rng]() { return std::uniform_real_distribution<float>(-100, 100)(rng); };
		auto vel = [&rng]() { return std::uniform_real_distribution<float>(-0.2, 0.2)(rng); };

		particles.clear();
		particles.reserve(numParticles);
		for (int i = 0; i < numParticles; i++) {
			Vec3 position{pos(), pos(), pos()};
			Vec3 velocity{vel(), vel(), vel()};
			particles.push_back(Particle{position, velocity, 1});
		}
	}

	void runSimulation(int numTimesteps) {
		if (numTimesteps <= 0) {
			throw std::invalid_argument{"Number of timesteps must be greater than 0"};
		}

		for (int i = 0; i < numTimesteps; i++) {
			collideParticles();
			attractParticles();
			stepParticles();
		}
	}

	void attractParticles() {
		for (Particle& p : particles) {
			for (const Particle& other : particles) {
				if (&p != &other) {
					p.attract(other);
				}
			}
		}
	}

	void collideParticles() {
		// Each time there is a collision, all particles must be checked again for collisions with
		// the new particle
		auto one = particles.begin();
		while (one != particles.end()) {
			auto two = particles.begin();
			while (two != particles.end()) {
				if (one != two && Particle::checkCollision(*one, *two)) {
					// Collide particles, merging two into one
					*one = Particle::collide(*one, *two);
					// Remove two (it is now merged with one
					particles.erase(two);
					// Reset iterators at the beginning, as they are both invalid now
					// And new particle may be colliding with other particles
					one = particles.begin();
					two = particles.begin();
				} else {
					two++; // If no collision, increment two normally
				}
			}
			one++;
		}
	}

	void stepParticles() {
		for (Particle& p : particles) {
			p.step();
		}
	}
}

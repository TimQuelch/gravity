#include "simulation.h"
#include "particle.h"
#include <cassert>
#include <chrono>
#include <functional>
#include <random>
#include <vector>

namespace gravity {
	namespace {
		std::vector<particle> particles;
	}

	void init_particles(int num_particles) {
		assert(num_particles > 0);

		std::default_random_engine rng(std::chrono::system_clock::now().time_since_epoch().count());
		auto pos = std::bind(std::uniform_real_distribution<float>(-150, 150), rng);
		auto vel = std::bind(std::uniform_real_distribution<float>(-10, 10), rng);

		particles.clear();
		particles.reserve(num_particles);
		for (int i = 0; i < num_particles; i++) {
			vec3 position{pos(), pos(), pos()};
			vec3 velocity{vel(), vel(), vel()};
			particles.push_back(particle{position, velocity, 1});
		}
	}

	void run_simulation(int num_timesteps) {
		assert(num_timesteps > 0);

		for (int i = 0; i < num_timesteps; i++) {
			collide_particles();
			attract_particles();
			step_particles();
		}
	}

	void attract_particles() {
		for (particle& p : particles) {
			for (const particle& other : particles) {
				if (&p != &other) {
					p.attract(other);
				}
			}
		}
	}

	void collide_particles() {
		// Each time there is a collision, all particles must be checked again for collisions with
		// the new particle
		auto one = particles.begin();
		while (one != particles.end()) {
			auto two = particles.begin();
			while (two != particles.end()) {
				if (one != two && one->check_collision(*two)) {
					// Collide particles, merging two into one
					one->collide(*two);
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

	void step_particles() {
		for (particle& p : particles) {
			p.step();
		}
	}
}


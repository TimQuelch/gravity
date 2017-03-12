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

	void step_particles() {
		for (particle& p : particles) {
			p.step();
		}
	}
}


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
			particles.push_back(particle(pos(), pos(), pos(), vel(), vel(), vel(), 1));
		}
	}

	void run_simulation(int num_timesteps) {
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


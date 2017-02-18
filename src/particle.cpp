#include "particle.h"
#include <cassert>
#include <cmath>
#include <tuple>

namespace gravity {
	void particle::set_x(float x) { _x = x; }

	void particle::set_y(float y) { _y = y; }

	void particle::set_vx(float vx) { _vx = vx; }

	void particle::set_vy(float vy) { _vy = vy; }

	void particle::set_pos(float x, float y) {
		_x = x;
		_y = y;
	}

	void particle::set_pos(std::pair<float, float> pos) { std::tie(_x, _y) = pos; }

	void particle::set_velocity(float vx, float vy) {
		_vx = vx;
		_vy = vy;
	}

	void particle::set_velocity(std::pair<float, float> velocity) { std::tie(_vx, _vy) = velocity; }

	void particle::set_mass(float mass) {
		assert(mass > 0);
		_mass = mass;
	}

	void particle::step() {
		_x += _vx;
		_y += _vy;
	}

	void particle::attract(const particle& other) {
		const float x_dist = _x - other.get_x();
		const float y_dist = _y - other.get_y();
		const float radius_sq = x_dist * x_dist + y_dist * y_dist;
		const float radius = std::sqrt(radius_sq);

		const float force = GRAVITATIONAL_CONSTANT * _mass * other.get_mass() / radius_sq;
		const float force_x = force * (x_dist / radius);
		const float force_y = force * (y_dist / radius);

		accelerate(force_x, force_y);
	}

	void particle::accelerate(float force_x, float force_y) {
		_vx += force_x / _mass;
		_vx += force_y / _mass;
	}
}


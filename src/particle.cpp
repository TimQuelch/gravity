#include "particle.h"
#include <cassert>
#include <cmath>

namespace gravity {
	void particle::set_x(float x) { _x = x; }

	void particle::set_y(float y) { _y = y; }

	void particle::set_z(float z) { _z = z; }

	void particle::set_vx(float vx) { _vx = vx; }

	void particle::set_vy(float vy) { _vy = vy; }

	void particle::set_vz(float vz) { _vz = vz; }

	void particle::set_pos(float x, float y, float z) {
		_x = x;
		_y = y;
		_z = z;
	}

	void particle::set_pos(vec3 position) { std::tie(_x, _y, _z) = position; }

	void particle::set_vel(float vx, float vy, float vz) {
		_vx = vx;
		_vy = vy;
		_vz = vz;
	}

	void particle::set_vel(vec3 velocity) { std::tie(_vx, _vy, _vz) = velocity; }

	void particle::set_mass(float mass) {
		assert(mass > 0);
		_mass = mass;
	}

	void particle::step() {
		_x += _vx;
		_y += _vy;
		_z += _vz;
	}

	void particle::attract(const particle& other) {
		const float x_dist = _x - other.get_x();
		const float y_dist = _y - other.get_y();
		const float z_dist = _z - other.get_z();
		const float radius_sq = x_dist * x_dist + y_dist * y_dist + z_dist * z_dist;
		const float radius = std::sqrt(radius_sq);

		const float force = GRAVITATIONAL_CONSTANT * _mass * other.get_mass() / radius_sq;
		const float force_x = force * (x_dist / radius);
		const float force_y = force * (y_dist / radius);
		const float force_z = force * (z_dist / radius);

		accelerate(force_x, force_y, force_z);
	}

	void particle::accelerate(float force_x, float force_y, float force_z) {
		_vx += force_x / _mass;
		_vy += force_y / _mass;
		_vz += force_z / _mass;
	}

	void particle::accelerate(vec3 force) {
		_vx += std::get<0>(force) / _mass;
		_vy += std::get<1>(force) / _mass;
		_vz += std::get<2>(force) / _mass;
	}

	void particle::collide(const particle& other) {
		const vec3 this_momentum = get_momentum();
		const vec3 other_momentum = other.get_momentum();
		_mass += other._mass;
		_vx = (std::get<0>(this_momentum) + std::get<0>(other_momentum)) / _mass;
		_vy = (std::get<1>(this_momentum) + std::get<1>(other_momentum)) / _mass;
		_vz = (std::get<2>(this_momentum) + std::get<2>(other_momentum)) / _mass;
	}
}

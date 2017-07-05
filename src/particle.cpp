#include "particle.h"

namespace gravity {
	vec3& vec3::operator+=(const vec3& other) {
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}

	vec3& vec3::operator-=(const vec3& other) {
		x -= other.x;
		y -= other.y;
		z -= other.z;
		return *this;
	}

	void particle::step() { _pos += _vel; }

	void particle::attract(const particle& other) {
		const vec3 dist = other.pos() - _pos;
		const float radius = dist.magnitude();
		const float radius_cb = radius * radius * radius;
		const float accel_factor = gravitational_constant * other.mass() / radius_cb;

		_vel += dist * accel_factor;
	}

	particle particle::collide(const particle& one, const particle& two) {
		const float mass_one = one.mass();
		const float mass_two = two.mass();
		const float mass = mass_one + mass_two;
		const vec3 pos = (two.pos() - one.pos()) * (mass_two / mass);
		const vec3 vel = (one.momentum() + two.momentum()) * (1 / mass);
		return {pos, vel, mass};
	}

	bool particle::check_collision(const particle& one, const particle& two) {
		const float dist = (one.pos() - two.pos()).magnitude();
		return dist <= (one.radius() + two.radius());
	}
}

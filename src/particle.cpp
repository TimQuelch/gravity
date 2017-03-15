#include "particle.h"

namespace gravity {
	float vec3::magnitude() const { return std::sqrt(x * x + y * y + z * z); }

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

	vec3 operator+(const vec3& one, const vec3& two) { return vec3{one} += two; }

	vec3 operator-(const vec3& one, const vec3& two) { return vec3{one} -= two; }

	void particle::step() { _pos += _vel; }

	void particle::attract(const particle& other) {
		const vec3 dist = _pos - other.pos();
		const float radius = dist.magnitude();
		const float radius_cb = radius * radius * radius;
		const float accel_factor = gravitational_constant * other.mass() / radius_cb;

		_vel += dist.scale(accel_factor);
	}

	void particle::collide(const particle& other) {
		const vec3 this_momentum = momentum();
		const vec3 other_momentum = other.momentum();
		_mass += other._mass;
		_vel = (this_momentum + other_momentum).scale(1 / _mass);
		_radius = compute_radius();
	}

	bool particle::check_collision(const particle& other) {
		const float dist = (_pos - other.pos()).magnitude();
		return dist <= (radius() + other.radius());
	}
}

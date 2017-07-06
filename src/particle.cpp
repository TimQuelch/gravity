#include "particle.h"

namespace gravity {
	Vec3& Vec3::operator+=(const Vec3& other) {
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}

	Vec3& Vec3::operator-=(const Vec3& other) {
		x -= other.x;
		y -= other.y;
		z -= other.z;
		return *this;
	}

	void Particle::step() { pos_ += vel_; }

	void Particle::attract(const Particle& other) {
		const Vec3 dist = other.pos() - pos_;
		const float radius = dist.magnitude();
		const float radiusCubed = radius * radius * radius;
		const float accelFactor = gravitationalConstant * other.mass() / radiusCubed;

		vel_ += dist * accelFactor;
	}

	Particle Particle::collide(const Particle& one, const Particle& two) {
		const float mass_one = one.mass();
		const float mass_two = two.mass();
		const float mass = mass_one + mass_two;
		const Vec3 pos = (two.pos() - one.pos()) * (mass_two / mass);
		const Vec3 vel = (one.momentum() + two.momentum()) * (1 / mass);
		return {pos, vel, mass};
	}

	bool Particle::checkCollision(const Particle& one, const Particle& two) {
		const float dist = (one.pos() - two.pos()).magnitude();
		return dist <= (one.radius() + two.radius());
	}
}

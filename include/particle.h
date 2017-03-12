#ifndef GRAVITY_PARTICLE_H
#define GRAVITY_PARTICLE_H

#include <stdexcept>

namespace gravity {
	const float gravitational_constant = 6.674e-11;

	struct vec3 {
		float x{0};
		float y{0};
		float z{0};

		vec3() = default;
		vec3(float xval, float yval, float zval)
		    : x{xval}
		    , y{yval}
		    , z{zval} {}

		float magnitude() const;
		vec3 scale(float factor) const { return {factor * x, factor * y, factor * z}; }
		vec3& operator+=(const vec3& other);
		vec3& operator-=(const vec3& other);
	};
	vec3 operator+(const vec3& one, const vec3& two);
	vec3 operator-(const vec3& one, const vec3& two);

	class particle {
	public:
		particle() = default;
		particle(vec3 position, vec3 velocity, float mass)
		    : _pos{position}
		    , _vel{velocity}
		    , _mass{mass > 0 ? mass : throw std::invalid_argument{"Mass must be positive"}} {}

		vec3 pos() const { return _pos; }
		vec3 vel() const { return _vel; }
		vec3 momentum() const { return _vel.scale(_mass); }
		float mass() const { return _mass; }

		void step();
		void attract(const particle& other);
		void collide(const particle& other);

	private:
		vec3 _pos{0, 0, 0};
		vec3 _vel{0, 0, 0};
		float _mass{1};
	};
}

#endif

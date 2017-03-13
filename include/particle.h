/**
 * @file particle.h
 * @brief Particle and vector classes for n-body simulation
 * @author Tim Quelch
 */

#ifndef GRAVITY_PARTICLE_H
#define GRAVITY_PARTICLE_H

#include <stdexcept>

namespace gravity {
	/**
	 * @brief Gravitational constant. Gravitational forces are proportional to this constant
	 */
	const float gravitational_constant = 6.674e-11;

	/**
	 * @brief Simple 3D vector of floats
	 */
	struct vec3 {
		/**
		 * @brief x component of vector
		 */
		float x{0};
		/**
		 * @brief y component of vector
		 */
		float y{0};
		/**
		 * @brief z component of vector
		 */
		float z{0};

		/**
		 * @brief Create vector with all components as 0
		 */
		vec3() = default;

		/**
		 * @brief Create vector with initial values
		 *
		 * @param xval Initial x component
		 * @param yval Initial y component
		 * @param zval Initial z component
		 */
		vec3(float xval, float yval, float zval)
		    : x{xval}
		    , y{yval}
		    , z{zval} {}

		/**
		 * @brief Calculate the magnitude of the vector
		 *
		 * @return The magnitude of the vector
		 */
		float magnitude() const;

		/**
		 * @brief Scales the vector by a scalar
		 *
		 * @param factor The scalar to scale the vector by
		 *
		 * @return The scaled vector
		 */
		vec3 scale(float factor) const { return {factor * x, factor * y, factor * z}; }

		/**
		 * @brief Add vector to this vector
		 *
		 * @param other Another vector
		 *
		 * @return *this
		 */
		vec3& operator+=(const vec3& other);

		/**
		 * @brief Subtract vector from this vector
		 *
		 * @param other Another vector
		 *
		 * @return *this
		 */
		vec3& operator-=(const vec3& other);
	};

	/**
	 * @brief Add two vectors together
	 *
	 * @param one A vector
	 * @param two Another vector
	 *
	 * @return The sum of the two vectors
	 */
	vec3 operator+(const vec3& one, const vec3& two);

	/**
	 * @brief Subtract a vector from another
	 *
	 * @param one A vector
	 * @param two Another vector
	 *
	 * @return The difference of the two vectors
	 */
	vec3 operator-(const vec3& one, const vec3& two);

	/**
	 * @brief A single particle in a n-body simulation
	 *
	 * Particles have a position, velocity, and a mass. Momentum can be found from these properties.
	 * Particles atract each other through gravity, and can collide with other particles
	 */
	class particle {
	public:
		/**
		 * @brief Create particle with default values
		 *
		 * Position and velocity are 0. Mass is 1
		 */
		particle() = default;

		/**
		 * @brief Create particle with an initial position and velocity
		 *
		 * @param position The initial position of the particle
		 * @param velocity The initial velocity of the particle
		 * @param mass The mass of the particle. Must be positive
		 */
		particle(vec3 position, vec3 velocity, float mass)
		    : _pos{position}
		    , _vel{velocity}
		    , _mass{mass > 0 ? mass : throw std::invalid_argument{"Mass must be positive"}} {}

		/**
		 * @brief Gets the position of the particle
		 *
		 * @return The position of the particle
		 */
		vec3 pos() const { return _pos; }

		/**
		 * @brief Gets the velocity of the particle
		 *
		 * @return The velocity of the particle
		 */
		vec3 vel() const { return _vel; }

		/**
		 * @brief Gets the momentum of the particle. Momentum = mass * velocity
		 *
		 * @return The momentum of the particle
		 */
		vec3 momentum() const { return _vel.scale(_mass); }

		/**
		 * @brief Gets the mass of the particle
		 *
		 * @return The mass of the particle
		 */
		float mass() const { return _mass; }

		/**
		 * @brief Steps the particle in the direction of the velocity vector
		 */
		void step();

		/**
		 * @brief Attracts a particle to another particle by gravity
		 *
		 * This modifies the velocity of the particle, but not the position
		 *
		 * @param other Another particle
		 */
		void attract(const particle& other);

		/**
		 * @brief Collides two particles together
		 *
		 * Sets new mass and velocity vectors, conserving momentum
		 *
		 * It is assumed that the other particle is removed from the simulation
		 *
		 * @param other Another particle
		 */
		void collide(const particle& other);

	private:
		vec3 _pos{0, 0, 0};
		vec3 _vel{0, 0, 0};
		float _mass{1};
	};
}

#endif

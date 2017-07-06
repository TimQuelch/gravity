/**
 * \file particle.h
 * \brief Particle and vector classes for n-body simulation
 * \author Tim Quelch
 */

#ifndef GRAVITY_PARTICLE_H
#define GRAVITY_PARTICLE_H

#include <cmath>
#include <stdexcept>

namespace gravity {
	/**
	 * \brief Gravitational constant. Gravitational forces are proportional to this constant
	 */
	const float gravitationalConstant = 0.10;
	// const float gravitationalConstant = 6.674e-11;

	/**
	 * \brief Simple 3D vector of floats
	 */
	struct Vec3 {
		float x{0}; /**< \brief x component of vector */
		float y{0}; /**< \brief y component of vector */
		float z{0}; /**< \brief z component of vector */

		Vec3() = default; /**< \brief Create vector with all components as 0 */

		/**
		 * \brief Create vector with initial values
		 * \param x Initial x component
		 * \param y Initial y component
		 * \param z Initial z component
		 */
		Vec3(float x, float y, float z)
		    : x{x}
		    , y{y}
		    , z{z} {}

		/**
		 * \brief Calculate the magnitude of the vector
		 * \return The magnitude of the vector
		 */
		float magnitude() const { return std::sqrt(x * x + y * y + z * z); }

		/**
		 * \brief Calculate the unit vector of the vector
		 * \return The unit vector
		 */
		Vec3 direction() const { return *this * (1 / magnitude()); }

		/**
		 * \brief Add vector to this vector
		 * \param other Another vector
		 * \return *this
		 */
		Vec3& operator+=(const Vec3& other);

		/**
		 * \brief Add two vectors together
		 * \param one A vector
		 * \param two Another vector
		 * \return The sum of the two vectors
		 */
		friend Vec3 operator+(const Vec3& one, const Vec3& two) { return Vec3{one} += two; }

		/**
		 * \brief Subtract vector from this vector
		 * \param other Another vector
		 * \return *this
		 */
		Vec3& operator-=(const Vec3& other);

		/**
		 * \brief Subtract a vector from another
		 * \param one A vector
		 * \param two Another vector
		 * \return The difference of the two vectors
		 */
		friend Vec3 operator-(const Vec3& one, const Vec3& two) { return Vec3{one} -= two; }

		/**
		 * \brief Multiply the vector by a scalar
		 * \tparam A numeric scalar type
		 * \param scalar The scalar value
		 * \return *this
		 */
		template <typename Scalar>
		Vec3& operator*=(Scalar scalar) {
			x *= scalar;
			y *= scalar;
			z *= scalar;
			return *this;
		}

		/**
		 * \brief Multiply a vector by a scalar
		 * \tparam A numeric scalar type
		 * \param scalar The scalar value
		 * \param vec The vector
		 * \return The scaled vector
		 */
		template <typename Scalar>
		friend Vec3 operator*(Scalar scalar, const Vec3& vec) {
			return Vec3{vec} *= scalar;
		}

		/**
		 * \brief Multiply a vector by a scalar
		 * \tparam A numeric scalar type
		 * \param scalar The scalar value
		 * \param vec The vector
		 * \return The scaled vector
		 */
		template <typename Scalar>
		friend Vec3 operator*(const Vec3& vec, Scalar scalar) {
			return Vec3{vec} *= scalar;
		}
	};

	/**
	 * \brief A single particle in a n-body simulation
	 * Particles have a position, velocity, and a mass. Momentum can be found from these properties.
	 * Particles atract each other through gravity, and can collide with other particles
	 */
	class Particle {
	public:
		/**
		 * \brief Create particle with default values
		 * Position and velocity are 0. Mass is 1
		 */
		Particle() = default;

		/**
		 * \brief Create particle with an initial position and velocity
		 * \param position The initial position of the particle
		 * \param velocity The initial velocity of the particle
		 * \param mass The mass of the particle. Must be positive
		 */
		Particle(Vec3 position, Vec3 velocity, float mass)
		    : pos_{position}
		    , vel_{velocity}
		    , mass_{mass > 0 ? mass : throw std::invalid_argument{"Mass must be positive"}}
		    , radius_{computeRadius()} {}

		/**
		 * \brief Gets the position of the particle
		 * \return The position of the particle
		 */
		Vec3 pos() const { return pos_; }

		/**
		 * \brief Gets the velocity of the particle
		 * \return The velocity of the particle
		 */
		Vec3 vel() const { return vel_; }

		/**
		 * \brief Gets the momentum of the particle. Momentum = mass * velocity
		 * \return The momentum of the particle
		 */
		Vec3 momentum() const { return vel_ * mass_; }

		/**
		 * \brief Gets the mass of the particle
		 * \return The mass of the particle
		 */
		float mass() const { return mass_; }

		/**
		 * \brief Gets the radius of the particle
		 * The radius of the particle depends on the mass. Radius should be updated each time the
		 * mass of the particle changes.
		 * \return The radius of the particle
		 */
		float radius() const { return radius_; }

		/**
		 * \brief Steps the particle in the direction of the velocity vector
		 */
		void step();

		/**
		 * \brief Attracts a particle to another particle by gravity.
		 * This modifies the velocity of the particle, but not the position
		 * \param other Another particle
		 */
		void attract(const Particle& other);

		/**
		 * \brief Merge two particles together when they collide.
		 * New particle is created conserving momentum.
		 * \param one A particle
		 * \param two Another particle
		 * \return The particle that is the result of the collision
		 */
		static Particle collide(const Particle& one, const Particle& two);

		/**
		 * \brief Checks whether two particles are colliding
		 * \param one A particle
		 * \param two Another particle
		 * \return True if the particles' radii are overlapping
		 */
		static bool checkCollision(const Particle& one, const Particle& two);

	private:
		Vec3 pos_{0, 0, 0};
		Vec3 vel_{0, 0, 0};
		float mass_{1};
		float radius_{computeRadius()};

		/**
		 * \brief Compute the radius of the particle
		 * This should be used to update the radius of the particle any time the mass is changed.
		 * The value is derived from the volume of a sphere, V = 4pi/3 r^3. It is assumed that
		 * density is normalised.
		 */
		float computeRadius() const { return std::cbrt((3 * mass_) / (4 * M_PI)); }
	};
}

#endif

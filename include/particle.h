#ifndef GRAVITY_PARTICLE_H
#define GRAVITY_PARTICLE_H

#include <tuple>

namespace gravity {
	const float GRAVITATIONAL_CONSTANT = 6.674e-11;

	typedef std::tuple<float, float, float> vec3;

	class particle {
	public:
		particle()
		    : _x(0)
		    , _y(0)
		    , _z(0)
		    , _vx(0)
		    , _vy(0)
		    , _vz(0)
		    , _mass(1) {}
		particle(float x, float y, float z, float mass)
		    : _x(x)
		    , _y(y)
		    , _z(z)
		    , _vx(0)
		    , _vy(0)
		    , _vz(0)
		    , _mass(mass > 0 ? mass : 1) {}
		particle(float x, float y, float z, float vx, float vy, float vz, float mass)
		    : _x(x)
		    , _y(y)
		    , _z(z)
		    , _vx(vx)
		    , _vy(vy)
		    , _vz(vz)
		    , _mass(mass > 0 ? mass : 1) {}
		particle(vec3 position, vec3 velocity, float mass)
		    : _x(std::get<0>(position))
		    , _y(std::get<1>(position))
		    , _z(std::get<2>(position))
		    , _vx(std::get<0>(velocity))
		    , _vy(std::get<1>(velocity))
		    , _vz(std::get<2>(velocity))
		    , _mass(mass > 0 ? mass : 1) {}

		float get_x() const { return _x; }
		float get_y() const { return _y; }
		float get_z() const { return _z; }
		float get_vx() const { return _vx; }
		float get_vy() const { return _vy; }
		float get_vz() const { return _vz; }
		float get_mass() const { return _mass; }
		vec3 get_pos() const { return std::make_tuple(_x, _y, _z); }
		vec3 get_vel() const { return std::make_tuple(_vx, _vy, _vz); }

		void set_x(float x);
		void set_y(float y);
		void set_z(float z);
		void set_vx(float vx);
		void set_vy(float vy);
		void set_vz(float vz);
		void set_pos(float x, float y, float z);
		void set_pos(vec3 position);
		void set_vel(float vx, float vy, float vz);
		void set_vel(vec3 velocity);
		void set_mass(float mass);

		void step();
		void attract(const particle& other);
		void accelerate(float force_x, float force_y, float force_z);
		void accelerate(vec3 force);

	private:
		float _x;
		float _y;
		float _z;
		float _vx;
		float _vy;
		float _vz;
		float _mass;
	};
}

#endif

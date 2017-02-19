#ifndef GRAVITY_PARTICLE_H
#define GRAVITY_PARTICLE_H

#include <utility>

namespace gravity {
	const float GRAVITATIONAL_CONSTANT = 6.674e-11;

	class particle {
	public:
		particle()
		    : _x(0)
		    , _y(0)
		    , _vx(0)
		    , _vy(0)
		    , _mass(1) {}
		particle(float x, float y, float mass)
		    : _x(x)
		    , _y(y)
		    , _vx(0)
		    , _vy(0)
		    , _mass(mass > 0 ? mass : 1) {}
		particle(float x, float y, float vx, float vy, float mass)
		    : _x(x)
		    , _y(y)
		    , _vx(vx)
		    , _vy(vy)
		    , _mass(mass > 0 ? mass : 1) {}

		float get_x() const { return _x; }
		float get_y() const { return _y; }
		float get_vx() const { return _vx; }
		float get_vy() const { return _vy; }
		float get_mass() const { return _mass; }
		std::pair<float, float> get_pos() const { return std::make_pair(_x, _y); }
		std::pair<float, float> get_velocity() const { return std::make_pair(_vx, _vy); }

		void set_x(float x);
		void set_y(float y);
		void set_vx(float vx);
		void set_vy(float vy);
		void set_pos(float x, float y);
		void set_pos(std::pair<float, float> pos);
		void set_velocity(float vx, float vy);
		void set_velocity(std::pair<float, float> velocity);
		void set_mass(float mass);

		void step();
		void attract(const particle& other);
		void accelerate(float force_x, float force_y);

	private:
		float _x;
		float _y;
		float _vx;
		float _vy;
		float _mass;
	};
}

#endif

#include "simulation.h"

int main() {
	gravity::init_particles(1000);
	gravity::run_simulation(10000);
	return 0;
}

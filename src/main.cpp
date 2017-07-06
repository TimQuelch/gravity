#include "simulation.h"

int main() {
	gravity::initParticles(1000);
	gravity::runSimulation(10000);
	return 0;
}

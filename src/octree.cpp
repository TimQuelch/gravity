#include "octree.h"

#include <algorithm>

namespace gravity {
	Octree::Domain::Domain(Vec3 v1, Vec3 v2)
	    : min_{std::min(v1.x, v2.x), std::min(v1.y, v2.y), std::min(v1.z, v2.z)}
	    , max_{std::max(v1.x, v2.x), std::max(v1.y, v2.y), std::max(v1.z, v2.z)} {}

	bool Octree::Domain::isInDomain(Vec3 pos) const {
		const bool inX = pos.x >= min_.x && pos.x < max_.x;
		const bool inY = pos.y >= min_.y && pos.y < max_.y;
		const bool inZ = pos.z >= min_.z && pos.z < max_.z;
		return inX && inY && inZ;
	}

	int Octree::Domain::getOctantIndex(Vec3 pos) const {
		const Vec3 mid = min_ + ((max_ - min_) * 0.5);
		if (pos.z >= mid.z) {
			if (pos.y >= mid.y) {
				if (pos.x >= mid.x) {
					return 0;
				} else {
					return 1;
				}
			} else {
				if (pos.x >= mid.x) {
					return 2;
				} else {
					return 3;
				}
			}
		} else {
			if (pos.y >= mid.y) {
				if (pos.x >= mid.x) {
					return 4;
				} else {
					return 5;
				}
			} else {
				if (pos.x >= mid.x) {
					return 6;
				} else {
					return 7;
				}
			}
		}
	}

	Octree::Domain Octree::Domain::getOctantDomain(int octantIndex) const {
		if (octantIndex >= 8 || octantIndex < 0) {
			throw std::invalid_argument("Invalid octant index");
		}
		const Vec3 span = max_ - min_;
		const Vec3 mid = min_ + (span * 0.5);
		Vec3 outerCorner;
		switch (octantIndex) {
		case 0:
			outerCorner = max_;
			break;
		case 1:
			outerCorner = max_ - Vec3{span.x, 0, 0};
			break;
		case 2:
			outerCorner = max_ - Vec3{0, span.y, 0};
			break;
		case 3:
			outerCorner = min_ + Vec3{0, 0, span.z};
			break;
		case 4:
			outerCorner = max_ - Vec3{0, 0, span.z};
			break;
		case 5:
			outerCorner = min_ + Vec3{0, span.y, 0};
			break;
		case 6:
			outerCorner = min_ - Vec3{span.x, 0, 0};
			break;
		case 7:
			outerCorner = min_;
			break;
		}
		return {mid, outerCorner};
	}

	Octree::Octree(const ParticleList& particles, Domain domain)
	    : root_{std::make_shared<Node>(particles, domain)} {
		if (particles.empty()) {
			throw std::invalid_argument("Must be at least one particle in Octree");
		}
	}

	Octree::Node::Node(const ParticleList& particles, Domain domain)
	    : domain_{domain}
	    , particles_{particles}
	    , children_{} {
		if (particles.empty()) {
			throw std::invalid_argument("Node must contain at least one Particle");
		}
		if (particles.size() == 1) {
			// Set mass and centerOfMass directly if there is only one particle
			mass_ = (*(particles.begin()))->mass();
			centerOfMass_ = (*(particles.begin()))->pos();
		} else {
			// Build the child nodes and compute the mass and center of mass
			children_ = buildChildren(particles, domain);
			mass_ = computeMass(children_);
			centerOfMass_ = computeCenterOfMass(children_);
		}
	}

	Octree::Node::Node(const ParticlePtr& particle, Domain domain)
	    : domain_{domain}
	    , particles_{}
	    , children_{} {
		particles_.push_back(particle);
		mass_ = particle->mass();
		centerOfMass_ = particle->pos();
	}

	void Octree::Node::addParticle(const ParticlePtr& particle) {
		if (!domain_.isInDomain(particle->pos())) {
			throw std::invalid_argument("Particle is not in the Node's Domain");
		}
		if (contains(particle)) {
			throw std::invalid_argument("Particle is already held by the Node");
		}
		// Add particle to the list of particles in the Node
		particles_.push_back(particle);

		// If there are no child Nodes, build them.
		// This happens when the Node was previously an external Node
		if (children_.empty()) {
			children_ = buildChildren(particles_, domain_);
		} else {
			// If any of the current children has an appropriate domain, add the particle to it
			bool added = false;
			for (auto child : children_) {
				if (child->domain_.isInDomain(particle->pos())) {
					child->addParticle(particle);
					added = true;
					break;
				}
			}
			// If not, create a new child Node with the particle
			if (!added) {
				Domain newDomain = domain_.getOctantDomain(domain_.getOctantIndex(particle->pos()));
				auto newChild = std::make_shared<Node>(particle, newDomain);
				children_.push_back(newChild);
			}
		}
		// Recompute mass and center of mass
		mass_ = computeMass(children_);
		centerOfMass_ = computeCenterOfMass(children_);
	}

	void Octree::Node::removeParticle(const ParticlePtr& particle) {
		if (!contains(particle)) {
			throw std::invalid_argument("Particle is not held in the node");
		}
		// Remove particle from the list of particles
		particles_.erase(std::find(particles_.begin(), particles_.end(), particle));
		// Remove the particle from the relevant child
		for (auto child : children_) {
			if (child->contains(particle)) {
				child->removeParticle(particle);
				break;
			}
		}
	}

	bool Octree::Node::contains(const ParticlePtr& particle) const {
		for (const auto& p : particles_) {
			if (p == particle) {
				return true;
			}
		}
		return false;
	}

	float Octree::Node::computeMass(const NodeList& nodes) {
		float sum{0};
		for (const auto& node : nodes) {
			sum += node->mass();
		}
		return sum;
	}

	Vec3 Octree::Node::computeCenterOfMass(const NodeList& nodes) {
		float sumMass = 0;
		Vec3 sumProduct{0, 0, 0};
		for (const auto& node : nodes) {
			sumProduct += node->mass() * node->centerOfMass();
			sumMass += node->mass();
		}
		return sumProduct * (1 / sumMass);
	}

	Octree::NodeList Octree::Node::buildChildren(const ParticleList& particles, Domain domain) {
		if (particles.size() < 2) {
			throw std::invalid_argument("Must be at least two particles in list");
		}
		// Create an array of particle lists for the octants
		std::array<ParticleList, 8> octants{};
		for (const auto& particle : particles) {
			octants[domain.getOctantIndex(particle->pos())].push_back(particle);
		}
		// Create Nodes for each of the octants
		NodeList children{};
		for (int i = 0; i < octants.size(); i++) {
			if (!octants[i].empty()) {
				children.push_back(std::make_shared<Node>(octants[i], domain.getOctantDomain(i)));
			}
		}
		return children;
	}
}

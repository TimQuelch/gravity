#include "octree.h"

#include <algorithm>
#include <stack>

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

	Octree::Octree(const Octree& other)
	    : root_{std::make_shared<Node>(*other.root_)} {}

	Octree::Octree(Octree&& other)
	    : root_{std::move(other.root_)} {}

	Octree& Octree::operator=(Octree other) {
		std::swap(*this, other);
		return *this;
	}

	Octree::Octree(const ParticleList& particles, Domain domain)
	    : root_{std::make_shared<Node>(particles, domain)} {
		if (particles.empty()) {
			throw std::invalid_argument("Must be at least one particle in Octree");
		}
	}

	Octree::Node::Node(const Node& other)
	    : centerOfMass_{other.centerOfMass_}
	    , mass_{other.mass_}
	    , domain_{other.domain_} {
		// Copy the list of Particles from the old Node
		for (ParticlePtr old : other.particles_) {
			particles_.push_back(std::make_shared<Particle>(*old));
		}
		// Build the child Nodes
		children_ = buildChildren(particles_, domain_);
	}

	Octree::Node::Node(Node&& other)
	    : centerOfMass_{other.centerOfMass_}
	    , mass_{other.mass_}
	    , domain_{other.domain_}
	    , particles_{std::move(other.particles_)}
	    , children_{std::move(other.children_)} {}

	Octree::Node& Octree::Node::operator=(Node other) {
		std::swap(*this, other);
		return *this;
	}

	Octree::Node::Node(const ParticleList& particles, Domain domain)
	    : domain_{domain}
	    , particles_{particles}
	    , children_{buildChildren(particles_, domain_)} {
		if (particles.empty()) {
			throw std::invalid_argument("Node must contain at least one Particle");
		}
		if (isExteriorNode()) {
			updateNodeValues();
		} else {
			mass_ = computeMass(children_);
			centerOfMass_ = computeCenterOfMass(children_);
		}
	}

	Octree::Node::Node(ParticlePtr particle, Domain domain)
	    : domain_{domain} {
		particles_.push_back(particle);
		updateNodeValues();
	}

	void Octree::Node::addParticle(ParticlePtr particle) {
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
		if (isExteriorNode()) {
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

	void Octree::Node::removeParticle(ParticlePtr particle) {
		if (!contains(particle)) {
			throw std::invalid_argument("Particle is not held in the node");
		}
		// Remove particle from the list of particles
		particles_.erase(std::find(particles_.begin(), particles_.end(), particle));
		// Remove the particle from the relevant child
		for (auto it = children_.begin(); it != children_.end(); ++it) {
			NodePtr child{*it};
			if (child->contains(particle)) {
				if (child->isExteriorNode()) {
					children_.erase(it);
				} else {
					child->removeParticle(particle);
				}
				break;
			}
		}
	}

	bool Octree::Node::contains(ParticlePtr particle) const {
		for (const auto& p : particles_) {
			if (p == particle) {
				return true;
			}
		}
		return false;
	}

	void Octree::Node::rebalanceNode(std::stack<NodePtr> history) {
		if (history.top().get() != this) {
			throw std::invalid_argument{"Top value of the history stack should be *this"};
		}
		if (isExteriorNode()) {
			// If it is an exterior node, check if the particle is within the Domain bounds
			ParticlePtr particle = *particles_.begin();
			if (!domain_.isInDomain(particle->pos())) {
				// Remove the particle from the current node
				removeParticle(particle);
				while (!history.empty()) {
					// Traverse up the history until the particle is in the Domain
					history.pop();
					if (history.top()->domain_.isInDomain(particle->pos())) {
						// Add the particle to that Node
						history.top()->addParticle(particle);
					}
				}
			}
		} else {
			// Recursively rebalance all child Nodes
			for (NodePtr child : children_) {
				std::stack<NodePtr> newHistory{history};
				newHistory.push(child);
				child->rebalanceNode(newHistory);
			}
		}
	}

	void Octree::Node::updateNodeValues() {
		if (isExteriorNode()) {
			ParticlePtr particle = *particles_.begin();
			mass_ = particle->mass();
			centerOfMass_ = particle->pos();
		} else {
			for (auto child : children_) {
				child->updateNodeValues();
			}
			mass_ = computeMass(children_);
			centerOfMass() = computeCenterOfMass(children_);
		}
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
		if (particles.empty()) {
			throw std::invalid_argument{"Must be at least one particle in list"};
		}
		if (particles.size() == 1) {
			return {};
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

	bool Octree::Node::isExteriorNode() const { return children_.empty(); }

	void Octree::rebalanceTree() {
		std::stack<NodePtr> history{};
		history.push(root_);
		root_->rebalanceNode(history);
	}
}

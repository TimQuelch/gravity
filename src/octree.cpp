#include "octree.h"

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

	Octree::Octree(const std::list<std::shared_ptr<Particle>>& particles, Domain domain)
	    : root_{particles, domain} {
		if (particles.empty()) {
			throw std::invalid_argument("Must be at least one particle in Octree");
		}
	}

	Octree::Node::Node(const std::list<std::shared_ptr<Particle>>& particles, Domain domain)
	    : domain_{domain}
	    , particles_{particles}
	    , children_{} {
		if (particles.empty()) {
			throw std::invalid_argument("Node must contain at least one Particle");
		}
		if (particles.size() == 1) {
			mass_ = (*(particles.begin()))->mass();
			centerOfMass_ = (*(particles.begin()))->pos();
		} else {
			std::array<std::list<std::shared_ptr<Particle>>, 8> octants{};
			for (std::shared_ptr<Particle> particle : particles) {
				octants[domain.getOctantIndex(particle->pos())].push_back(particle);
			}
			for (int i = 0; i < octants.size(); i++) {
				if (!octants[i].empty()) {
					children_.push_back(Node{particles, domain.getOctantDomain(i)});
				}
			}
			mass_ = computeMass(children_);
			centerOfMass_ = computeCenterOfMass(children_);
		}
	}

	float Octree::Node::computeMass(const std::list<Node>& nodes) {
		float sum{0};
		for (const Node& node : nodes) {
			sum += node.mass();
		}
		return sum;
	}

	Vec3 Octree::Node::computeCenterOfMass(const std::list<Node>& nodes) {
		float sumMass = 0;
		Vec3 sumProduct{0, 0, 0};
		for (const Node& node : nodes) {
			sumProduct += node.mass() * node.centerOfMass();
			sumMass += node.mass();
		}
		return sumProduct * (1 / sumMass);
	}
}

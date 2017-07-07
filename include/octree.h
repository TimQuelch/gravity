/// \file octree.h
/// Octree structure to hold particles
/// \author Tim Quelch

#ifndef GRAVITY_OCTREE_H
#define GRAVITY_OCTREE_H

#include "particle.h"
#include <list>
#include <memory>

namespace gravity {
	/// Class to hold an Octree of particles
	class Octree {
	public:
		/// A class to hold the 3D bounds of a region of space
		class Domain {
		public:
			/// Construct Domain with default values
			Domain() = default;

			/// Construct Domain with two vectors. The domain is constructed from the extremes of
			/// these vectors
			/// \param v1 The first vector
			/// \param v2 The second vector
			Domain(Vec3 v1, Vec3 v2);

			/// Tests whether a position is in the Domain
			/// \param pos The position to test
			/// \return Whether the position is in the Domain
			bool isInDomain(Vec3 pos) const;

			/// Get the index of the octant that a position is in. Octants are integers from 0-7
			/// \param pos The position to categorise
			/// \return The index of the octant the position belongs to
			int getOctant(Vec3 pos) const;

			/// Get the Domain a octant in the domain
			/// \param octantIndex The index of the octant
			/// \return The Domain of the octant
			/// \throw std::invalid_argument If octantIndex is not an integer from 0-7
			Domain getSubdomain(int octantIndex) const;

		private:
			Vec3 min_{0, 0, 0}; ///< The minimum extremes of the Domain
			Vec3 max_{0, 0, 0}; ///< The maximum extremes of the Domain
		};

		/// Construct an Octree for a given set of particles and given domain
		/// \param particles A list of particles
		/// \param domain The domain of the Octree
		/// \throw std::invalid_argument If list of particles is empty
		Octree(const std::list<Particle>& particles, Domain domain);

		/// Get the Domain of the Octree
		/// \return The Domain of the Octree
		Domain domain() const;

	private:
		/// A node in the Octree. Has a mass, center of mass, and a domain
		class Node {
		public:
			/// Construct a Node with default values
			Node() = default;

			/// Construct a Node with a given center of mass, mass and domain
			/// \param centerOfMass The center of mass of the node
			/// \param mass The mass of the node
			/// \param domain The domain of the node
			Node(Vec3 centerOfMass, float mass, Domain domain)
			    : centerOfMass_{centerOfMass}
			    , mass_{mass}
			    , domain_{domain} {}

			/// Get the center of mass of the node
			/// \return The center of mass
			Vec3 centerOfMass() const { return centerOfMass_; }

			/// Get the mass of the node
			/// \return The mass
			float mass() const { return mass_; }

		protected:
			Vec3 centerOfMass_{0, 0, 0}; ///< The center of mass of the node
			float mass_{1};              ///< The mass of the node
			Domain domain_{};            ///< The domain of the node
		};

		/// An internal node in the Octree. This node represents a group of nodes underneath it
		class InternalNode : public Node {
		public:
			/// Construct an InternalNode with default values
			InternalNode() = default;

			/// Construct an InternalNode from a set of particles and given domain
			/// \param particles A list of particles
			/// \param domain The domain of the node
			InternalNode(const std::list<Particle>& particles, Domain domain);

		private:
			/// Compute the total mass of a given list of Nodes. Should be used to set the mass
			/// of the Node on construction and update
			/// \param nodes The nodes to calculate the mass of. Is usually subnodes_
			/// \return The total mass of the nodes
			static float computeMass(const std::list<Node>& nodes);

			/// Compute the center of mass of as given list of Nodes. Should be used to set the
			/// center of mass of the Node on construciton and update.
			/// \param nodes The nodes to calculate the center of mass of. Is usually subnodes_
			/// \return The center of mass of the nodes
			static Vec3 computeCenterOfMass(const std::list<Node>& nodes);

			std::list<Node> subnodes_; ///< The subnodes of this node in the Octree
		};

		/// An external node in the Octree. This node represents a single particle in the Octree. It
		/// has no child nodes
		class ExternalNode : public Node {
		public:
			/// Construct an ExternalNode with default values
			ExternalNode() = default;

			/// Construct an ExternalNode with a given Particle and domain. The mass and center of
			/// mass are set to the Particle's mass and position respectively
			/// \param particle The particle this Node should represent
			/// \param domain The domain of the Node
			ExternalNode(const Particle& particle, Domain domain)
			    : Node{particle.pos(), particle.mass(), domain}
			    , particle{particle} {}

			Particle particle; ///< The particle this Node represents
		};

		std::unique_ptr<Node> root_; ///< The root node of the Octree
	};
}

#endif

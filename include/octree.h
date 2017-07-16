/// \file octree.h
/// Octree structure to hold particles
/// \author Tim Quelch

#ifndef GRAVITY_OCTREE_H
#define GRAVITY_OCTREE_H

#include "particle.h"
#include <list>
#include <memory>
#include <stack>
#include <stdexcept>

namespace gravity {
	/// Class to hold an Octree of particles
	class Octree {
	public:
		/// Shorthand for Particle smart pointer
		using ParticlePtr = std::shared_ptr<Particle>;
		/// Shorthand for a list of Particles
		using ParticleList = std::list<ParticlePtr>;

		/// A class to hold the 3D bounds of a region of space
		class Domain {
		public:
			/// Construct Domain with default values
			Domain() = default;

			/// Construct Domain with two vectors. The Domain is constructed from the extremes of
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
			int getOctantIndex(Vec3 pos) const;

			/// Get the Domain a octant in the Domain
			/// \param octantIndex The index of the octant
			/// \return The Domain of the octant
			/// \throw std::invalid_argument If octantIndex is not an integer from 0-7
			Domain getOctantDomain(int octantIndex) const;

		private:
			Vec3 min_{0, 0, 0}; ///< The minimum extremes of the Domain
			Vec3 max_{0, 0, 0}; ///< The maximum extremes of the Domain
		};

		/// Construct an Octree for a given set of particles and given Domain
		/// \param particles A list of particles
		/// \param domain The Domain of the Octree
		/// \throw std::invalid_argument If list of particles is empty
		Octree(const ParticleList& particles, Domain domain);

		/// Get the Domain of the Octree
		/// \return The Domain of the Octree
		Domain domain() const;

	private:
		class Node;
		/// Shorthand for Node smart pointer
		using NodePtr = std::shared_ptr<Node>;
		/// Shorthand for a list of Nodes
		using NodeList = std::list<NodePtr>;

		/// A Node in the Octree. Has a mass, center of mass, and a Domain
		class Node {
		public:
			/// Construct a Node with default values
			Node() = default;

			/// Construct a Node for a given set of Particles and Domain
			/// \param particles A list of pointers to particles
			/// \param domain The Domain of the Node
			/// \throw std::invalid_argument If list of particles is empty
			Node(const ParticleList& particles, Domain domain);

			/// Construct a Node for a given Particle and Domain
			/// \param particle A Particle
			/// \param domain Domain of the Node
			Node(ParticlePtr particle, Domain domain);

			/// Get the center of mass of the Node
			/// \return The center of mass
			Vec3 centerOfMass() const { return centerOfMass_; }

			/// Get the mass of the Node
			/// \return The mass
			float mass() const { return mass_; }

			/// Add a particle to the Node
			/// \param particle A pointer to a particle
			/// \throw std::invalid_argument If the particle is not within the Node Domain, or the
			/// particle is already held by the Node
			void addParticle(ParticlePtr particle);

			/// Remove a particle from the Node
			/// \param particle A pointer to a particle
			/// \throw std::invalid_argument If the particle is not held by the Node
			void removeParticle(ParticlePtr particle);

			/// Check whether the Node contains a particle
			/// \param particle A Particle
			/// \return Whether the particle is in the Node or not
			bool contains(ParticlePtr particle) const;

			/// Recursively rebalance Node
			/// \param history Parents of the Nodes. Top value in history should be a
			/// std::shared_ptr<Node> of `this`
			/// \throw std::invalid_argument If the top value of history is not `this`
			void rebalanceNode(std::stack<NodePtr> history);

			/// Recursively update the mass and center of mass of the Node
			void updateNodeValues();

		private:
			/// Compute the total mass of a given list of Nodes. Should be used to set the mass
			/// of the Node on construction and update
			/// \param nodes The Nodes to calculate the mass of. Is usually children_
			/// \return The total mass of the Nodes
			static float computeMass(const NodeList& nodes);

			/// Compute the center of mass of as given list of Nodes. Should be used to set the
			/// center of mass of the Node on construciton and update.
			/// \param nodes The Nodes to calculate the center of mass of. Is usually children_
			/// \return The center of mass of the Nodes
			static Vec3 computeCenterOfMass(const NodeList& nodes);

			/// Build a list of child nodes from a given list of Particles. If the list of Particles
			/// contains a single Particle, an empty list is returned
			/// \param particles A list of pointers to particles
			/// \param domain The Domain of the parent node
			/// \return A list of child nodes
			/// \throw std::invalid_argument If the list of particles is empty
			static NodeList buildChildren(const ParticleList& particles, Domain domain);

			/// Checks if the Node represents a single particle
			/// \return bool True if Node is exterior
			bool isExteriorNode() const;

			Vec3 centerOfMass_{0, 0, 0}; ///< Center of mass of the Node
			float mass_{1};              ///< Mass of the Node
			Domain domain_{};            ///< Domain of the Node
			ParticleList particles_{};   ///< Particles represented by the Node
			NodeList children_{};        ///< Child Nodes
		};

		/// Move particles which have left their Node's Domain into the correct Node
		void rebalanceTree();

		NodePtr root_; ///< The root Node of the Octree
	};
}

#endif

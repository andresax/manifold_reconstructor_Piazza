/*
 * Delaunay3DCellInfo.h
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#ifndef DELAUNAY3DCELLINFO_H_
#define DELAUNAY3DCELLINFO_H_

#define NO_HEURISTIC_K
#define HEURISTIC_K 5
//#define HEURISTIC_K 1

#include <Eigen/Core>
#include <set>
#include <iostream>
#include <vector>
#include <glm.hpp>
#include "FSConstraint.h"

struct RayPath;

class Delaunay3DCellInfo {
public:

	// Constructors (it must be default-constructable)
	Delaunay3DCellInfo();
	Delaunay3DCellInfo(const Delaunay3DCellInfo & ref);

	virtual ~Delaunay3DCellInfo();

	// Getters
	int getVoteCount() const;
	float getVoteCountProb() const;
	float getVoteCountProbFromIntersections();

	bool isBoundary() const {
		return boundary;
	}
	bool iskeptManifold() const {
		return keepManifold;
	}

	bool isTemporaryInside() const {
		return temporary_Inside_;
	}
	bool isToBeTested() const {
		return toBeTested_;
	}
	bool isShrinked() const {
		return shrinked_;
	}
	bool isGalpha() const {
		return Galpha_;
	}
	const std::set<FSConstraint, FSConstraint::LtFSConstraint> & getIntersections() const {
		return m_setIntersections;
	}
//	const std::vector<RayReconstruction*> & getLw1() const {
//		return Lw1_;
//	}
//	const std::vector<RayReconstruction*> & getLw2() const {
//		return Lw2_;
//	}
//	const std::vector<RayReconstruction*> & getLw3() const {
//		return Lw3_;
//	}

	bool isNew() const {
		return m_bNew;
	}

	// Setters
	void setVoteCount(const int voteCount);
	void setVoteCountProb(const float voteCountProb);

	void setBoundary(bool value) {
		boundary = value;
	}
	void setKeptManifold(bool value) {
		keepManifold = value;
	}
	void setShrinked(bool value) {
		shrinked_ = value;
	}
	void setGalpha(bool value) {
		Galpha_ = value;
	}
	void setTemporaryInside(bool value) {
		temporary_Inside_ = value;
	}
	void setToBeTested(bool value) {
		toBeTested_ = value;
	}
	void setIntersections(const std::set<FSConstraint, FSConstraint::LtFSConstraint> & ref) {
		m_setIntersections = ref;
	}
//	void setLw1(const std::vector<RayReconstruction*> & ref) {
//		Lw1_ = ref;
//	}
//	void setLw2(const std::vector<RayReconstruction*> & ref) {
//		Lw2_ = ref;
//	}
//	void setLw3(const std::vector<RayReconstruction*> & ref) {
//		Lw3_ = ref;
//	}

	void setIdxManifold(bool value, int i) {
		idxFacetNotManifold[i] = value;
	}

	bool getIdxManifold(int i) {
		return idxFacetNotManifold[i];
	}

	// Public Methods
	void incrementVoteCount();
	void incrementVoteCount(int num);
	void incrementVoteCountProb(float incr);

	void decrementVoteCount();
	void decrementVoteCount(int num);
	void decrementVoteCountProb(float incr);

	bool isKeptByVoteCount(const int nVoteThresh) const;
	bool isKeptByVoteCountProb(const float nVoteThresh) const;

	void setWeights(float w_1, float w_2, float w_3);

	void addPath(RayPath* p);
	void removePath(RayPath* p);
	std::set<RayPath*> getPaths();

	void printIntersections();

	void addItersectionWeightedW1(RayReconstruction* r);
	void addItersectionWeightedW2(RayReconstruction* r);
	void addItersectionWeightedW3(RayReconstruction* r);
	/* template<class T>
	 void addIntersection(int camIndex, int featureIndex, const vector<T> & vecVertexHandles, const vector<Matrix> & vecCamCenters) {
	 m_setIntersections.insert(m_setIntersections.end(), FSConstraint(camIndex, featureIndex));
	 }
	 template<class T>
	 void addIntersection(int camIndex, int featureIndex, float vote, const vector<T> & vecVertexHandles, const vector<Matrix> & vecCamCenters) {
	 m_setIntersections.insert(m_setIntersections.end(), FSConstraint(camIndex, featureIndex,vote));
	 }*/
#ifdef NO_HEURISTIC_K
	template<class T>
	void addIntersection(int camIndex, int featureIndex, float vote, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		m_setIntersections.insert(m_setIntersections.end(), FSConstraint(camIndex, featureIndex, vote));
	}

	template<class T>
	void addIntersection(int camIndex, int featureIndex, float vote, int vertexidx, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		m_setIntersections.insert(m_setIntersections.end(), FSConstraint(camIndex, featureIndex, vertexidx, vote));
	}

	template<class T>
	void addIntersection(int camIndex, int featureIndex, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		m_setIntersections.insert(m_setIntersections.end(), FSConstraint(camIndex, featureIndex));
	}
#else
	template<class T>
	void addIntersection(int camIndex, int featureIndex, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		FSConstraint incoming(camIndex, featureIndex);
		if ((int) m_setIntersections.size() < HEURISTIC_K) {
			// The constraint set is not full, so insert the incoming free-space constraint and update nearest neighbor info.
			std::set<FSConstraint, FSConstraint::LtFSConstraint>::iterator it, itIncoming;
			itIncoming = m_setIntersections.insert(m_setIntersections.end(), incoming);
			for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
				if (it == itIncoming)
				continue;
				// Asymmetric metric:
				float curDist = distFSConstraint(*itIncoming, *it, vecVertexHandles, vecCamCenters);
				float curDist2 = distFSConstraint(*it, *itIncoming, vecVertexHandles, vecCamCenters);
				// Update incoming
				if (curDist < itIncoming->fNearestNeighborDist)
				itIncoming->setNearestNeighbor(it, curDist);
				// Update *it:
				if (curDist2 < it->fNearestNeighborDist)
				it->setNearestNeighbor(itIncoming, curDist2);
			}
		} else {
			// The constraint set is full, so apply the spatial cover heuristic to determine whether or not to insert the incoming free-space constraint

			// Quick return / rejection on the case that m_nMaxConstraintsKept == 1.
			if (HEURISTIC_K == 1)
			return;

			float minDist = std::numeric_limits<float>::infinity();
			std::set<FSConstraint, FSConstraint::LtFSConstraint>::iterator it, it2, itEject;
			for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
				float curDist = distFSConstraint(incoming, *it, vecVertexHandles, vecCamCenters);
				if (curDist < it->fNearestNeighborDist)
				break; // REJECT
				float curDist2 = distFSConstraint(*it, incoming, vecVertexHandles, vecCamCenters);
				if (curDist2 < it->fNearestNeighborDist)
				break;// REJECT
				// Update incoming
				if (curDist < incoming.fNearestNeighborDist)
				incoming.setNearestNeighbor(it, curDist);
				// Update minDist & itEject
				if (it->fNearestNeighborDist < minDist) {
					minDist = it->fNearestNeighborDist;
					itEject = it;
				}
			}

			if (it == m_setIntersections.end()) {
				// No rejection, so insert incoming and evict itEject.

				// For an asymmetric metric, incoming might have its nearest neighbor ejected.  If so, compute the 2nd nearest neighbor
				if (incoming.pNearestNeighbor == itEject) {
					incoming.fNearestNeighborDist = std::numeric_limits<float>::infinity();
					for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
						if (it2 == itEject)
						continue;
						float curDist = distFSConstraint(incoming, *it2, vecVertexHandles, vecCamCenters);
						if (curDist < incoming.fNearestNeighborDist)
						incoming.setNearestNeighbor(it2, curDist);
					}
				}

				// Recompute nearest neighbors that previously pointed to itEject.
				for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
					if (it2->pNearestNeighbor == itEject) { // implicity "continue;"'s if it2 == itEject
						// Recompute the nearest neighbor for it2:
						it2->resetNearestNeighborDist();
						for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
							if (it == itEject || it == it2)
							continue;
							float curDist = distFSConstraint(*it2, *it, vecVertexHandles, vecCamCenters);
							if (curDist < it2->fNearestNeighborDist)
							it2->setNearestNeighbor(it, curDist);
						}
					}
				}

				// Finally erase itEject and insert incoming
				m_setIntersections.erase(itEject);
				m_setIntersections.insert(m_setIntersections.end(), incoming);
			}
		}
	}

	template<class T>
	void addIntersection(int camIndex, int featureIndex, float vote, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		FSConstraint incoming(camIndex, featureIndex, vote);
		if ((int) m_setIntersections.size() < HEURISTIC_K) {
			// The constraint set is not full, so insert the incoming free-space constraint and update nearest neighbor info.
			std::set<FSConstraint, FSConstraint::LtFSConstraint>::iterator it, itIncoming;
			itIncoming = m_setIntersections.insert(m_setIntersections.end(), incoming);
			for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
				if (it == itIncoming)
				continue;
				// Asymmetric metric:
				float curDist = distFSConstraint(*itIncoming, *it, vecVertexHandles, vecCamCenters);
				float curDist2 = distFSConstraint(*it, *itIncoming, vecVertexHandles, vecCamCenters);
				// Update incoming
				if (curDist < itIncoming->fNearestNeighborDist)
				itIncoming->setNearestNeighbor(it, curDist);
				// Update *it:
				if (curDist2 < it->fNearestNeighborDist)
				it->setNearestNeighbor(itIncoming, curDist2);
			}
		} else {
			// The constraint set is full, so apply the spatial cover heuristic to determine whether or not to insert the incoming free-space constraint

			// Quick return / rejection on the case that m_nMaxConstraintsKept == 1.
			if (HEURISTIC_K == 1)
			return;

			float minDist = std::numeric_limits<float>::infinity();
			std::set<FSConstraint, FSConstraint::LtFSConstraint>::iterator it, it2, itEject;
			for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
				float curDist = distFSConstraint(incoming, *it, vecVertexHandles, vecCamCenters);
				if (curDist < it->fNearestNeighborDist)
				break; // REJECT
				float curDist2 = distFSConstraint(*it, incoming, vecVertexHandles, vecCamCenters);
				if (curDist2 < it->fNearestNeighborDist)
				break;// REJECT
				// Update incoming
				if (curDist < incoming.fNearestNeighborDist)
				incoming.setNearestNeighbor(it, curDist);
				// Update minDist & itEject
				if (it->fNearestNeighborDist < minDist) {
					minDist = it->fNearestNeighborDist;
					itEject = it;
				}
			}

			if (it == m_setIntersections.end()) {
				// No rejection, so insert incoming and evict itEject.

				// For an asymmetric metric, incoming might have its nearest neighbor ejected.  If so, compute the 2nd nearest neighbor
				if (incoming.pNearestNeighbor == itEject) {
					incoming.fNearestNeighborDist = std::numeric_limits<float>::infinity();
					for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
						if (it2 == itEject)
						continue;
						float curDist = distFSConstraint(incoming, *it2, vecVertexHandles, vecCamCenters);
						if (curDist < incoming.fNearestNeighborDist)
						incoming.setNearestNeighbor(it2, curDist);
					}
				}

				// Recompute nearest neighbors that previously pointed to itEject.
				for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
					if (it2->pNearestNeighbor == itEject) { // implicity "continue;"'s if it2 == itEject
						// Recompute the nearest neighbor for it2:
						it2->resetNearestNeighborDist();
						for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
							if (it == itEject || it == it2)
							continue;
							float curDist = distFSConstraint(*it2, *it, vecVertexHandles, vecCamCenters);
							if (curDist < it2->fNearestNeighborDist)
							it2->setNearestNeighbor(it, curDist);
						}
					}
				}

				// Finally erase itEject and insert incoming
				m_setIntersections.erase(itEject);
				m_setIntersections.insert(m_setIntersections.end(), incoming);
			}
		}
	}
#endif
#ifdef NO_HEURISTIC_K
	template<class T>
	void removeIntersection(int camIndex, int featureIndex, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		m_setIntersections.erase(FSConstraint(camIndex, featureIndex));
	}

	template<class T>
	void removeIntersection(int camIndex, int featureIndex, float vote, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		m_setIntersections.erase(FSConstraint(camIndex, featureIndex, vote));
	}
#else
	template<class T>
	void removeIntersection(int camIndex, int featureIndex, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		if ((int) m_setIntersections.size() <= 1) {
			// No nearest neighbor info needs to be updated
			m_setIntersections.erase(FSConstraint(camIndex, featureIndex));
		} else {
			// The nearest neighbor info needs to be updated
			std::set<FSConstraint, FSConstraint::LtFSConstraint>::iterator it, it2, itEject;

			itEject = m_setIntersections.find(FSConstraint(camIndex, featureIndex));
			if (itEject == m_setIntersections.end())
			return;// wasn't in the set to begin with

			for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
				if (it == itEject)
				continue;
				if (it->pNearestNeighbor == itEject) {
					// Then recompute the nearest neighbor for it:
					it->resetNearestNeighborDist();
					for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
						if (it2 == itEject || it2 == it)
						continue;
						float curDist = distFSConstraint(*it, *it2, vecVertexHandles, vecCamCenters);
						if (curDist < it->fNearestNeighborDist)
						it->setNearestNeighbor(it2, curDist);
					}
				}
			}

			// Finally, erase it.
			m_setIntersections.erase(itEject);
		}
	}
	template<class T>
	void removeIntersection(int camIndex, int featureIndex, float vote, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		if ((int) m_setIntersections.size() <= 1) {
			// No nearest neighbor info needs to be updated
			m_setIntersections.erase(FSConstraint(camIndex, featureIndex, vote));
		} else {
			// The nearest neighbor info needs to be updated
			std::set<FSConstraint, FSConstraint::LtFSConstraint>::iterator it, it2, itEject;

			itEject = m_setIntersections.find(FSConstraint(camIndex, featureIndex, vote));
			if (itEject == m_setIntersections.end())
			return;// wasn't in the set to begin with

			for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
				if (it == itEject)
				continue;
				if (it->pNearestNeighbor == itEject) {
					// Then recompute the nearest neighbor for it:
					it->resetNearestNeighborDist();
					for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
						if (it2 == itEject || it2 == it)
						continue;
						float curDist = distFSConstraint(*it, *it2, vecVertexHandles, vecCamCenters);
						if (curDist < it->fNearestNeighborDist)
						it->setNearestNeighbor(it2, curDist);
					}
				}
			}

			// Finally, erase it.
			m_setIntersections.erase(itEject);
		}
	}
#endif

	int getNumIntersection() {
		return m_setIntersections.size();
	}

	/*  template<class T>
	 void removeIntersection(int camIndex, int featureIndex, const vector<T> & vecVertexHandles, const vector<Matrix> & vecCamCenters) {
	 m_setIntersections.erase(FSConstraint(camIndex, featureIndex));
	 }

	 template<class T>
	 void removeIntersection(int camIndex, int featureIndex, float vote, const vector<T> & vecVertexHandles,
	 const vector<Matrix> & vecCamCenters) {
	 m_setIntersections.erase(FSConstraint(camIndex, featureIndex, vote));
	 }*/

	template<class T>
	float distFSConstraint(const FSConstraint & x, const FSConstraint & y, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		return distFSConstraintTriangleAreaAaron(x, y, vecVertexHandles, vecCamCenters);
	}
	void clearIntersections() {
		m_setIntersections.clear();
	}
	void markOld() {
		m_bNew = false;
	}
	void markNew() {
		m_bNew = true;
	}

	// Operators (It must be assignable)
	Delaunay3DCellInfo & operator=(const Delaunay3DCellInfo & rhs);

private:
	// Private Methods
	template<class T>
	float distFSConstraintTriangleAreaAaron(const FSConstraint & x, const FSConstraint & y, const std::vector<T> & vecVertexHandles, const std::vector<glm::vec3> & vecCamCenters) {
		// Asymmetric distance heuristic.
		// Sum of two triangle areas, use the base segment PQ as constraint x, and the two points from y as R1 and R2.
		// Note: For efficiency, to avoid unnecessary division by 2 and square-roots, use the sum of twice-the-areas squared = squared area of parallelograms.
		const Eigen::Vector3f & P = Eigen::Vector3f(vecCamCenters[x.first].x, vecCamCenters[x.first].y, vecCamCenters[x.first].z);
		Eigen::Vector3f Q;
		Q(0) = vecVertexHandles[x.second].position.x();
		Q(1) = vecVertexHandles[x.second].position.y();
		Q(2) = vecVertexHandles[x.second].position.z();
		const Eigen::Vector3f & R1 = Eigen::Vector3f(vecCamCenters[y.first].x, vecCamCenters[y.first].y, vecCamCenters[y.first].z);
		Eigen::Vector3f R2(3, 1);
		R2(0) = vecVertexHandles[y.second].position.x();
		R2(1) = vecVertexHandles[y.second].position.y();
		R2(2) = vecVertexHandles[y.second].position.z();

		// Vector distances
		Eigen::Vector3f PQ(Q - P);
		Eigen::Vector3f PR1(R1 - P);
		Eigen::Vector3f PR2(R2 - P);

		// Sum of squared areas of parallelograms
		Eigen::Vector3f PQxPR1(PQ.cross(PR1));
		Eigen::Vector3f PQxPR2(PQ.cross(PR2));
		return PQxPR1.dot(PQxPR1) + PQxPR2.dot(PQxPR2);
	}

	// Private Members

	int Lw1_count_ = 0;

	float weightCache_ = 0.0;
	bool isCacheValid_ = false;

	// set of rays accounting for w_1, w_2, w_3
	std::set<RayPath*>* paths_;
	std::vector<RayReconstruction*>* Lw1_;
	std::vector<RayReconstruction*>* Lw2_;
	std::vector<RayReconstruction*>* Lw3_;
	float w_1_, w_2_, w_3_;

	int m_voteCount;
	float m_voteCountProb;
	bool boundary;
	bool keepManifold;
	bool toBeTested_;
	std::set<FSConstraint, FSConstraint::LtFSConstraint> m_setIntersections;

	bool m_bNew;
	bool idxFacetNotManifold[4];
	bool shrinked_;
	bool Galpha_;
	bool temporary_Inside_;
};

#endif /* DELAUNAY3DCELLINFO_H_ */

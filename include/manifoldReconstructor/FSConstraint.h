/*
 * FSConstraint.h
 *
 *  Created on: 17 Apr 2016
 *      Author: enrico
 */

#ifndef INCLUDE_MANIFOLDRECONSTRUCTOR_FSCONSTRAINT_H_
#define INCLUDE_MANIFOLDRECONSTRUCTOR_FSCONSTRAINT_H_

#include <limits>
#include <set>
#include <utility>

// Sorted-Set related struct

/*struct LtConstraint {
 bool operator()(const std::pair<int, int> x, const std::pair<int, int> y) const {
 if (x.first < y.first)
 return true;
 else
 return (x.first == y.first) && (x.second < y.second);
 }
 };*/

struct RayReconstruction {
	bool valid = true;
};

struct FSConstraint {

	int first;
	int second;
	int vertexIdx;
	float vote;

	struct LtFSConstraint {
		bool operator()(const FSConstraint x, const FSConstraint y) const {
			if (x.first < y.first) return true;
			else return (x.first == y.first) && (x.vertexIdx < y.vertexIdx);
		}
	};

	mutable std::set<FSConstraint, LtFSConstraint>::iterator pNearestNeighbor;
	mutable float fNearestNeighborDist;

	FSConstraint() {
		first = -1;
		second = -1;
		vote = -1;
		vertexIdx = second;
		fNearestNeighborDist = std::numeric_limits<float>::infinity();
	}
	FSConstraint(int camIndex, int featureIndex) {
		first = camIndex;
		second = featureIndex;
		vote = 1.0;
		vertexIdx = second;
		fNearestNeighborDist = std::numeric_limits<float>::infinity();
	}
	FSConstraint(int camIndex, int featureIndex, float voteVal) {
		first = camIndex;
		second = featureIndex;
		vertexIdx = second;
		vote = voteVal;
		fNearestNeighborDist = std::numeric_limits<float>::infinity();
	}
	FSConstraint(int camIndex, int featureIndex, int vertidx, float voteVal) {
		first = camIndex;
		second = featureIndex;
		vertexIdx = vertidx;
		vote = voteVal;
		fNearestNeighborDist = std::numeric_limits<float>::infinity();
	}
	FSConstraint(const FSConstraint & ref) {
		first = ref.first;
		second = ref.second;
		vote = ref.vote;
		vertexIdx = ref.vertexIdx;
		pNearestNeighbor = ref.pNearestNeighbor;
		fNearestNeighborDist = ref.fNearestNeighborDist;
	}
	FSConstraint & operator=(const FSConstraint & ref) {
		if (&ref != this) {
			first = ref.first;
			second = ref.second;
			vote = ref.vote;
			pNearestNeighbor = ref.pNearestNeighbor;
			fNearestNeighborDist = ref.fNearestNeighborDist;
		}
		return *this;
	}
	operator std::pair<int, int>() const {
		return std::make_pair(first, second);
	}

	// Trick c++ "const"-ness so that we can change the nearest neighbor information in the set from an iterator:
	void setNearestNeighbor(const std::set<FSConstraint, LtFSConstraint>::iterator itNearest, const float dist) const {
		pNearestNeighbor = itNearest;
		fNearestNeighborDist = dist;
	}
	void resetNearestNeighborDist() const {
		fNearestNeighborDist = std::numeric_limits<float>::infinity();
	}
};


#endif /* INCLUDE_MANIFOLDRECONSTRUCTOR_FSCONSTRAINT_H_ */

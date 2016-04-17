/*
 * Delaunay3DCellInfo.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <Delaunay3DCellInfo.h>

#define NO_HEURISTIC_K
#define HEURISTIC_K 5
//#define HEURISTIC_K 1

Delaunay3DCellInfo::Delaunay3DCellInfo() {
	m_voteCount = 0;
	m_voteCountProb = 0;
	m_bNew = true;
	boundary = false;
	keepManifold = false;
	toBeTested_ = false;
	shrinked_ = true;
	Galpha_ = false;
	temporary_Inside_ = false;

}
Delaunay3DCellInfo::Delaunay3DCellInfo(const Delaunay3DCellInfo & ref) {
	setVoteCount(ref.getVoteCount());
	setBoundary(ref.isBoundary());
	setIntersections(ref.getIntersections());
	if (!ref.isNew()) markOld();
}

Delaunay3DCellInfo::~Delaunay3DCellInfo() {
}

int Delaunay3DCellInfo::getVoteCount() const {
	return m_voteCount;
}

void Delaunay3DCellInfo::setVoteCount(const int voteCount) {
	m_voteCount = voteCount;
}

void Delaunay3DCellInfo::setVoteCountProb(const float voteCountProb) {
	m_voteCountProb = voteCountProb;
}

float Delaunay3DCellInfo::getVoteCountProb() const {
	return m_voteCountProb;
}

void Delaunay3DCellInfo::incrementVoteCount() {
	m_voteCount++;
}

void Delaunay3DCellInfo::incrementVoteCount(int num) {
	m_voteCount += num;
}

void Delaunay3DCellInfo::incrementVoteCountProb(float incr) {
	m_voteCountProb += incr;
}

void Delaunay3DCellInfo::decrementVoteCount() {
	if (m_voteCount > 0) m_voteCount--;
}
void Delaunay3DCellInfo::decrementVoteCount(int num) {
	if (m_voteCount - num > 0) m_voteCount -= num;
	else m_voteCount = 0;
}
void Delaunay3DCellInfo::decrementVoteCountProb(float incr) {
	if (m_voteCountProb > incr) m_voteCountProb -= incr;
}
bool Delaunay3DCellInfo::isKeptByVoteCount(const int nVoteThresh = 1) const {
	if (getVoteCount() < nVoteThresh) return true;
	return false;
}
bool Delaunay3DCellInfo::isKeptByVoteCountProb(const float nVoteThresh = 1.0) const {
	if (getVoteCountProb() < nVoteThresh) return true;
	return false;
}

// Operators (It must be assignable)
Delaunay3DCellInfo & Delaunay3DCellInfo::operator=(const Delaunay3DCellInfo & rhs) {
	if (this != &rhs) {
		setVoteCount(rhs.getVoteCount());
		setIntersections(rhs.getIntersections());
		if (!rhs.isNew()) markOld();
	}
	return *this;
}


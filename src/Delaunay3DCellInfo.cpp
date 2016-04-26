/*
 * Delaunay3DCellInfo.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <Delaunay3DCellInfo.h>
#include <iostream>

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
	w_1_ = 0.0;
	w_2_ = 0.0;
	w_3_ = 0.0;

	// Workaround to the bad bointers problem
	Lw1_ = new std::vector<RayReconstruction*>();
	Lw2_ = new std::vector<RayReconstruction*>();
	Lw3_ = new std::vector<RayReconstruction*>();

}
Delaunay3DCellInfo::Delaunay3DCellInfo(const Delaunay3DCellInfo & ref) {
	setVoteCount(ref.getVoteCount());
	setBoundary(ref.isBoundary());
	setIntersections(ref.getIntersections());
//	setLw1(ref.getLw1());
//	setLw2(ref.getLw2());
//	setLw3(ref.getLw3());
	if (!ref.isNew()) markOld();
}

Delaunay3DCellInfo::~Delaunay3DCellInfo() {
	delete Lw1_;
	delete Lw2_;
	delete Lw3_;
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

float Delaunay3DCellInfo::getVoteCountProbFromIntersections() {
	if (!isCacheValid_) {

		float sum = 0.0;

		for (auto r : *Lw1_)
			if (r->valid) sum += w_1_;

		for (auto r : *Lw2_)
			if (r->valid) sum += w_2_;

		for (auto r : *Lw2_)
			if (r->valid) sum += w_2_;

		weightCache_ = sum;
		isCacheValid_ = true;

	}

	return weightCache_;
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
//	if (m_voteCountProb > incr) m_voteCountProb -= incr;
	m_voteCountProb -= incr; // TODO uncomment?
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
//		setLw1(rhs.getLw1());
//		setLw2(rhs.getLw2());
//		setLw3(rhs.getLw3());
		if (!rhs.isNew()) markOld();
	}
	return *this;
}

void Delaunay3DCellInfo::setWeights(float w_1, float w_2, float w_3) {
	w_1_ = w_1;
	w_2_ = w_2;
	w_3_ = w_3;
}

void Delaunay3DCellInfo::printIntersections() {

	std::cout << "Delaunay3DCellInfo::printIntersections Lw1_: ";
	for (int i = 0; i < Lw1_->size(); i++) {
		RayReconstruction* r_ = Lw1_->at(i);
//		if (r_ == 0x1 || r_ == 0x1930000000b || r_ == 0x0 || r_ == 0x7fff00000001 || r_ == 0x7f800000 || r_ == 0x41 || r_ == 0x51 || r_ == 0x7fff00000000) {
//			std::cout << "Delaunay3DCellInfo::addItersectionWeightedW1 BAD pointer in Lw1_: " << std::endl;
//		}
		std::cout << r_ << " ";
	}
	std::cout << std::endl;

}

void Delaunay3DCellInfo::addItersectionWeightedW1(RayReconstruction* r) {
	isCacheValid_ = false;
	Lw1_count_++;
	Lw1_->push_back(r);

//	for (int i = 0; i < Lw1_->size(); i++) {
//		RayReconstruction* r_ = Lw1_->at(i);
//		if (r_ == 0x1 || r_ == 0x1930000000b || r_ == 0x0 || r_ == 0x7fff00000001 || r_ == 0x7f800000 || r_ == 0x41 || r_ == 0x51 || r_ == 0x7fff00000000) {
//			std::cout << "Delaunay3DCellInfo::addItersectionWeightedW1 BAD pointer in Lw1_: " << std::endl;
//		}
//	}

//	if (r == 0x1 || r == 0x1930000000b || r == 0x0 || r == 0x7fff00000001 || r == 0x7f800000 || r == 0x41 || r == 0x51 || r == 0x7fff00000000) {
//		std::cout << "Delaunay3DCellInfo::addItersectionWeightedW1 BAD Lw1_: ";
//		for (int i = 0; i < Lw1_.size(); i++) {
//			RayReconstruction* r_ = Lw1_[i];
//			std::cout << r_ << " ";
//		}
//		std::cout << std::endl;
//		return;
//	}

}
void Delaunay3DCellInfo::addItersectionWeightedW2(RayReconstruction* r) {
	isCacheValid_ = false;
//	if (r == 0x1 || r == 0x1930000000b || r == 0x0 || r == 0x7fff00000001 || r == 0x7f800000 || r == 0x41 || r == 0x51 || r == 0x7fff00000000) {
//		std::cout << "Delaunay3DCellInfo::addItersectionWeightedW2 Lw2_: ";
//		for (int i = 0; i < Lw2_->size(); i++) {
//			RayReconstruction* r_ = Lw2_->at(i);
//			std::cout << r_ << " ";
//		}
//		std::cout << std::endl;
//		return;
//	}
	Lw2_->push_back(r);
}
void Delaunay3DCellInfo::addItersectionWeightedW3(RayReconstruction* r) {
	isCacheValid_ = false;
	Lw3_->push_back(r);
}


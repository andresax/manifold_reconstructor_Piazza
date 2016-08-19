/*
 * ManifoldManager.cpp
 *
 *  Created on: 26/giu/2015
 *      Author: andrea
 */

#include <ManifoldManager.h>
#include <OutputCreator.h>
#include <utilities.hpp>
#include <sstream>
#include <iostream>

using std::cout;
using std::cerr;
using std::endl;

ManifoldManager::ManifoldManager(Delaunay3 &dt, bool inverseConic, float probabTh) :
		dt_(dt) {

	inverseConic_ = inverseConic;
	probabTh_ = probabTh;
	outputM_ = new OutputCreator(dt_);
}

ManifoldManager::~ManifoldManager() {
	boundaryCells_.clear();
}

void ManifoldManager::shrinkManifold2(std::set<PointD3> points, const float &maxPointToPointDistance, long currentEnclosingVersion) {
	bool fixedPoint = false;

	// Stats
	int countInEnclosingVolume = 0, countShrinked = 0, countTotal = 0, countCheckIfExplored = 0, countRepeatedVisits = 0, countIterations = 1;
	Chronometer chronoSorting, chronoInserting, chronoEnclosing0, chronoEnclosing, chronoTesting, chronoShrinking;

	// Populate the list with the boundary tetrahedra (? belonging to the carved space)
	std::vector<Delaunay3::Cell_handle> tetNotCarved;
	std::deque<Delaunay3::Cell_handle> tetsQueue;
	tetsQueue.insert(tetsQueue.begin(), boundaryCells_.begin(), boundaryCells_.end());
	chronoSorting.start();
	std::sort(tetsQueue.begin(), tetsQueue.end());
	chronoSorting.stop();

	while (!fixedPoint) {

		fixedPoint = true;

		// Shrink process
		while (!tetsQueue.empty()) {
			countTotal++;

			Delaunay3::Cell_handle currentTet = tetsQueue.front();
			tetsQueue.pop_front();

			bool isInEnclosingVolume = false;

			chronoEnclosing.start();
			if (currentTet->info().getEnclosingVersion() == currentEnclosingVersion) {
				isInEnclosingVolume = currentTet->info().isInEnclosingVolume();
			} else {
				for (int vertexId = 0; vertexId < 4; ++vertexId) {
					for (auto p : points) {
						if (utilities::distanceEucl(currentTet->vertex(vertexId)->point(), p) < maxPointToPointDistance) {
							isInEnclosingVolume = true;
							break;
						}
					}
					if (isInEnclosingVolume) break;
				}

				currentTet->info().setInEnclosingVolume(isInEnclosingVolume, currentEnclosingVersion);
			}
			chronoEnclosing.stop();

//			chronoEnclosing0.start();
//			bool isInEnclosingVolume = false;
//			for (int vertexId = 0; vertexId < 4; ++vertexId) {
//				for (auto p : points) {
//					if (utilities::distanceEucl(currentTet->vertex(vertexId)->point(), p) < maxPointToPointDistance) {
//						isInEnclosingVolume = true;
//						break;
//					}
//				}
//				if (isInEnclosingVolume) break;
//			}
//			chronoEnclosing0.stop();
//
//			if(isInEnclosingVolume_1 != isInEnclosingVolume) cerr << "ManifoldManager::shrinkManifold2:\t wrong enclosing \t iteration: " << countIterations << endl;

			if (isInEnclosingVolume) countInEnclosingVolume++;
			else continue;

			chronoTesting.start();
			if (singleTetTest(currentTet)) {
				chronoTesting.stop();

				if (!currentTet->info().iskeptManifold()) cerr << "ManifoldManager::shrinkManifold2:\t wrong shrink order\t iteration: " << countIterations << endl;

				countShrinked++;
				chronoShrinking.start();

				std::vector<Delaunay3::Cell_handle> newBoundaryTets;
				subTetAndUpdateBoundary(currentTet, newBoundaryTets);

				chronoShrinking.stop();

				for (auto neighbour : newBoundaryTets) {
					if (neighbour->info().iskeptManifold()) {
						countCheckIfExplored++;

						chronoInserting.start();
						std::deque<Delaunay3::Cell_handle>::iterator it;
						it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), neighbour, sortTetByIntersection());
						chronoInserting.stop();
						tetsQueue.insert(it, neighbour);
						fixedPoint = false;

					} else cerr << "ManifoldManager::shrinkManifold2:\t subTetAndUpdateBoundary return non inManifoldSet tet\t iteration: " << countIterations << endl;
				}

			} else {
				chronoTesting.stop();
				chronoInserting.start();
				std::vector<Delaunay3::Cell_handle>::iterator it;
				it = std::lower_bound(tetNotCarved.begin(), tetNotCarved.end(), currentTet, sortTetByIntersection());
				chronoInserting.stop();

				tetNotCarved.insert(it, currentTet);
			}
		}

		if (tetsQueue.empty() && !fixedPoint) {
			countIterations++;
			tetsQueue.insert(tetsQueue.begin(), tetNotCarved.begin(), tetNotCarved.end());
			tetNotCarved.clear();
		}
	}
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold2:\t\t\t\t sorting:\t\t\t\t" << chronoSorting.getSeconds() << "s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold2:\t\t\t\t inserting:\t\t\t\t" << chronoInserting.getSeconds() << "s" << endl;
//	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold2:\t\t\t\t enclosing0:\t\t\t\t" << chronoEnclosing0.getSeconds() << "s\t / \t" << countTotal << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold2:\t\t\t\t enclosing:\t\t\t\t" << chronoEnclosing.getSeconds() << "s\t / \t" << countTotal << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold2:\t\t\t\t test:\t\t\t\t\t" << chronoTesting.getSeconds() << "s\t / \t" << countInEnclosingVolume << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold2:\t\t\t\t shrink:\t\t\t\t" << chronoShrinking.getSeconds() << "s\t / \t" << countShrinked << endl;
	cout << "ManifoldManager::shrinkManifold2:\t\t\t\t number_of_finite_cells:\t\t" << dt_.number_of_finite_cells() << endl;
}

void ManifoldManager::shrinkSeveralAtOnce2(std::set<PointD3> points, const float &maxPointToPointDistance, long currentEnclosingVersion) {

	functionProfileChronometer_isRegular_.reset();
	functionProfileCounter_isRegular_ = 0;
	Chronometer chronoEnclosing0, chronoEnclosing, chronoAddAndCheckManifoldness;

	std::deque<Delaunay3::Cell_handle> tetsQueue;
	tetsQueue.insert(tetsQueue.begin(), boundaryCells_.begin(), boundaryCells_.end());

	while (!tetsQueue.empty()) {
		Delaunay3::Cell_handle currentTet = tetsQueue.back();
		tetsQueue.pop_back();

		bool isInEnclosingVolume = false;

		chronoEnclosing.start();

		if (currentTet->info().getEnclosingVersion() == currentEnclosingVersion) {
			isInEnclosingVolume = currentTet->info().isInEnclosingVolume();
		} else {
			for (int vertexId = 0; vertexId < 4; ++vertexId) {
				for (auto p : points) {
					if (utilities::distanceEucl(currentTet->vertex(vertexId)->point(), p) < maxPointToPointDistance) {
						isInEnclosingVolume = true;
						break;
					}
				}
				if (isInEnclosingVolume) break;
			}

			currentTet->info().setInEnclosingVolume(isInEnclosingVolume, currentEnclosingVersion);
		}
		chronoEnclosing.stop();

//		chronoEnclosing0.start();
//
//		bool isInEnclosingVolume = false;
//		for (int vertexId = 0; vertexId < 4; ++vertexId) {
//			for (auto p : points) {
//				if (utilities::distanceEucl(currentTet->vertex(vertexId)->point(), p) < maxPointToPointDistance) {
//					isInEnclosingVolume = true;
//					break;
//				}
//			}
//			if (isInEnclosingVolume) break;
//		}
//
//		chronoEnclosing0.stop();
//
//		if(isInEnclosingVolume_1 != isInEnclosingVolume) cerr << "ManifoldManager::shrinkSeveralAtOnce2:\t wrong enclosing" << endl;

		if (isInEnclosingVolume) {
			// TODO check on non boundary vertices first
			// TODO use vertices instead (when the test fail for all adjacent cells, the test is done many times on the same vertex)
			for (int curIdx = 0; curIdx < 4; ++curIdx) {
				if (!currentTet->info().isBoundary()) break;

				chronoAddAndCheckManifoldness.start();

				bool success = addAndCheckManifoldness(currentTet, curIdx);

				chronoAddAndCheckManifoldness.stop();

				if (success) break;
			}
		}
	}

//	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce2:\t\t\t\t enclosing0:\t\t\t\t" << chronoEnclosing0.getSeconds() << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce2:\t\t\t\t enclosing:\t\t\t\t" << chronoEnclosing.getSeconds() << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce2:\t\t\t\t addAndCheckManifoldness:\t\t" << chronoAddAndCheckManifoldness.getSeconds() << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce2:\t\t\t\t isRegular:\t\t\t\t" << functionProfileChronometer_isRegular_.getSeconds() << "s\t / \t" << functionProfileCounter_isRegular_ << endl;

}

void ManifoldManager::growSeveralAtOnce2() {
	functionProfileChronometer_isRegular_.reset();
	functionProfileCounter_isRegular_ = 0;
	Chronometer chronoQueueSelection, chronoRemoveAndCheckManifoldness;

	std::vector<Delaunay3::Cell_handle> tetsQueue;

	chronoQueueSelection.start();

	for (auto boundaryCell : boundaryCells_) {

		if (!dt_.is_cell(boundaryCell)) std::cerr << "ManifoldManager::growSeveralAtOnce2: dead cell in boundary" << std::endl;

		for (int neighbourId = 0; neighbourId < 4; ++neighbourId) {
			Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(neighbourId);

			if (!dt_.is_cell(neighbour)) std::cerr << "ManifoldManager::growSeveralAtOnce2: dead cell in boundary" << std::endl;

			if (boundaryCell->info().iskeptManifold() && !neighbour->info().iskeptManifold() && !neighbour->info().isToBeTested() && isFreespace(neighbour)) {

				std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), neighbour, sortTetByIntersection());
				tetsQueue.insert(it, neighbour);
				neighbour->info().setToBeTested(true);
			}
		}
	}

	chronoQueueSelection.stop();

	chronoRemoveAndCheckManifoldness.start();

	while (!tetsQueue.empty()) {
		Delaunay3::Cell_handle currentTet = tetsQueue.back();
		tetsQueue.pop_back();

		if (currentTet->info().isBoundary()) break;

		// TODO before or after aborting the cycle?
		currentTet->info().setToBeTested(false);

		for (int curIdx = 0; curIdx < 4; ++curIdx) {
			// TODO check on non boundary vertices first
			// TODO stop when success?
			bool success = removeAndCheckManifoldness(currentTet, curIdx);

			if (success) break;
		}
	}

	chronoRemoveAndCheckManifoldness.stop();

	cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce2:\t\t\t\t queue selection:\t\t\t" << chronoQueueSelection.getSeconds() << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce2:\t\t\t\t remove and check manifoldness:\t\t" << chronoRemoveAndCheckManifoldness.getSeconds() << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce2:\t\t\t\t isRegular:\t\t\t\t" << functionProfileChronometer_isRegular_.getSeconds() << "s\t / \t" << functionProfileCounter_isRegular_ << endl;

}

void ManifoldManager::regionGrowingBatch(PointD3 firstCamPosition) {

	for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
		itCell->info().setKeptManifold(false);
		for (int curV = 0; curV < 4; ++curV) {
			itCell->vertex(curV)->info().setUsed(0);
			itCell->vertex(curV)->info().setNotUsed(true);
		}
	}

	// Locate the idxCam-th cam and start from this one to grow the manifold
	Delaunay3::Cell_handle startingCell = dt_.locate(firstCamPosition);

	addTetAndUpdateBoundary(startingCell);

	regionGrowingProcedure();

}

void ManifoldManager::shrinkManifold(const PointD3 &camPosition, const float &maxPointToPointDistance, const float &maxPointToCamDistance) {
	bool fixedPoint = false;
	float r = maxPointToPointDistance + maxPointToCamDistance;

	// Populate the list with the boundary tetrahedra (? belonging to the carved space)
	std::vector<Delaunay3::Cell_handle> tetNotCarved;
//	std::set<Delaunay3::Cell_handle> shrinkedTets;
	std::deque<Delaunay3::Cell_handle> tetsQueue;
	tetsQueue.insert(tetsQueue.begin(), boundaryCells_.begin(), boundaryCells_.end());
	std::sort(tetsQueue.begin(), tetsQueue.end());

	// Stats
	int countInEnclosingVolume = 0, countShrinked = 0, countTotal = 0, countCheckIfExplored = 0, countRepeatedVisits = 0, countIterations = 1;

	while (!fixedPoint) {

		fixedPoint = true;

		//Shrink process
		while (!tetsQueue.empty()) {

			Delaunay3::Cell_handle currentTet = tetsQueue.front();
			tetsQueue.pop_front();

			countTotal++;

			bool isInEnclosingVolume = false;
			for (int vertexId = 0; vertexId < 4; ++vertexId)
				if (utilities::distanceEucl(currentTet->vertex(vertexId)->point(), camPosition) < r) {
					isInEnclosingVolume = true;
					break;
				}

			if (countIterations > 1 && !isInEnclosingVolume)
				cerr << "ManifoldManager::shrinkManifold:\t reinserted tet outside enclosing volume\t iteration: " << countIterations << endl;
//			if (shrinkedTets.count(currentTet)) cerr << "ManifoldManager::shrinkManifold:\t Tet visited multiple times\t iteration: " << countIterations << endl;
			if (isInEnclosingVolume) countInEnclosingVolume++;
			else continue;

			if (singleTetTest(currentTet)) {
				countShrinked++;

				if (!currentTet->info().iskeptManifold()) cerr << "ManifoldManager::shrinkManifold:\t wrong shrink order\t iteration: " << countIterations << endl;

				std::vector<Delaunay3::Cell_handle> newBoundaryTets;
				subTetAndUpdateBoundary(currentTet, newBoundaryTets);
//				shrinkedTets.insert(currentTet);

				for (auto neighbour : newBoundaryTets) {
					if (neighbour->info().iskeptManifold()) {
						countCheckIfExplored++;

//						if (true || !shrinkedTets.count(neighbour)) {
						std::deque<Delaunay3::Cell_handle>::iterator it;
						it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), neighbour, sortTetByIntersection());
						tetsQueue.insert(it, neighbour);
						fixedPoint = false;
//						}
					} else cerr << "ManifoldManager::shrinkManifold:\t subTetAndUpdateBoundary return non inManifoldSet tet\t iteration: " << countIterations << endl;
				}

			} else {

				std::vector<Delaunay3::Cell_handle>::iterator it;
				it = std::lower_bound(tetNotCarved.begin(), tetNotCarved.end(), currentTet, sortTetByIntersection());
				tetNotCarved.insert(it, currentTet);

			}

		}

		if (tetsQueue.empty() && !fixedPoint) {
			countIterations++;
			tetsQueue.insert(tetsQueue.begin(), tetNotCarved.begin(), tetNotCarved.end());
			tetNotCarved.clear();
		}
	}

	cout << "\t\t\t shrinked:\t\t" << countShrinked << "\tenclosed: " << countInEnclosingVolume << "\ttotal: " << countTotal << endl;
//	cout << "\t\t\t explored cells:\t\t" << shrinkedTets.size() << "\tchecked if already explored: " << countCheckIfExplored << endl;
	cout << "\t\t\t total triangulation cells:\t" << dt_.number_of_finite_cells() << endl;
	cout << "\t\t\t iterations untill fixed point:\t" << countIterations << endl;
}

void ManifoldManager::regionGrowingBatch(Delaunay3::Cell_handle &startingCell) {

	addTetAndUpdateBoundary(startingCell);

	regionGrowingProcedure();
}

void ManifoldManager::regionGrowing() {
	regionGrowingProcedure();
}

bool ManifoldManager::isInBoundary(Delaunay3::Cell_handle &cellToTest) {
	std::vector<int> toThrowAway;

	return isInBoundary(cellToTest, toThrowAway);
}

/*
 *	Returns true if cellToTest is kept manifold (?) and some neighbour of cellToTest isn't.
 *	If cellToTest is kept manifold, then neighNotManifold contains the indeces of the neighbours that aren't kept manifold.
 */
bool ManifoldManager::isInBoundary(Delaunay3::Cell_handle &cellToTest, std::vector<int> &neighNotManifold) {

	if (!cellToTest->info().iskeptManifold()) {
		return false;
	} else {
		bool neighNotManifoldFound = false;

		for (int curNeighId = 0; curNeighId < 4; ++curNeighId) {
			if (!cellToTest->neighbor(curNeighId)->info().iskeptManifold()) {
				neighNotManifold.push_back(curNeighId);

				neighNotManifoldFound = true;
			}
		}
		return neighNotManifoldFound;
	}
}

/*
 *	If cellToBeAdded->info().isBoundary() is false, then insert cellToBeAdded in boundaryCells_ (maintaining the order) and set cellToBeAdded->info().isBoundary() to true.
 */
bool ManifoldManager::insertInBoundary(Delaunay3::Cell_handle &cellToBeAdded) {

	if (!cellToBeAdded->info().isBoundary()) {
		cellToBeAdded->info().setBoundary(true);
		std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(boundaryCells_.begin(), boundaryCells_.end(), cellToBeAdded, sortTetByIntersection());
		if (it != boundaryCells_.end()) {
			if (*it != cellToBeAdded) {
				boundaryCells_.insert(it, cellToBeAdded);
			}
		} else {
			boundaryCells_.push_back(cellToBeAdded);
		}
		return true;
	} else {
		return false;
	}

}

/*
 *	If cellToBeRemoved->info().isBoundary() is set to true, removes cellToBeRemoved from boundaryCells_ and set cellToBeRemoved->info().isBoundary() to false,
 *	otherwise does nothing.
 */
bool ManifoldManager::removeFromBoundary(Delaunay3::Cell_handle &cellToBeRemoved) {
	if (!cellToBeRemoved->info().isBoundary()) {
		return false;
	} else {

		std::vector<Delaunay3::Cell_handle>::iterator it2;

		int num = 0;

		std::vector<Delaunay3::Cell_handle>::iterator itmin = std::lower_bound(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved, sortTetByIntersection());
		std::vector<Delaunay3::Cell_handle>::iterator itmax = std::upper_bound(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved, sortTetByIntersection());
		it2 = std::find(itmin, itmax, cellToBeRemoved);
		if ((*it2) == cellToBeRemoved) {
			cellToBeRemoved->info().setBoundary(false);
			boundaryCells_.erase(it2);
			++num;
		} else {
			it2 = std::find(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved);
			//while (it2 != boundaryCells_.end()) {
			cellToBeRemoved->info().setBoundary(false);
			boundaryCells_.erase(it2);
			++num;
			//it2 = std::find(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved);
			// }
		}

		if (num == 0) {
			std::cerr << "ManifoldManager::removeFromBoundary::notFound" << std::endl;
		} else if (num > 1) {
			std::cout << "ManifoldManager::removeFromBoundary::morethan1" << std::endl;
		}

		return true;
	}
}

void ManifoldManager::addTetAndUpdateBoundary(Delaunay3::Cell_handle &currentTet) {

//add the current tetrahedron in the manifold set and add it to the boundary if needed
	currentTet->info().setKeptManifold(true);
	currentTet->info().setShrinked(false);

	std::vector<int> notManifoldNeigh;
	if (isInBoundary(currentTet, notManifoldNeigh)) {
		insertInBoundary(currentTet);
	} else {
		if (currentTet->info().isBoundary()) {
			removeFromBoundary(currentTet);
		}
		currentTet->info().setBoundary(false);
	}

	if (notManifoldNeigh.size() == 3) {
		for (int curV = 0; curV < 4; ++curV) {
			if (find(notManifoldNeigh.begin(), notManifoldNeigh.end(), curV) == notManifoldNeigh.end()) {
				currentTet->vertex(curV)->info().setUsed(1);
			}
		}
	} else if (notManifoldNeigh.size() == 4) {
		for (int curV = 0; curV < 4; ++curV) {
			currentTet->vertex(notManifoldNeigh[curV])->info().setUsed(1);
		}
	}

	if (notManifoldNeigh.size() == 1) {  //when you fill a "hole"
		currentTet->vertex(notManifoldNeigh[0])->info().setNotUsed(false);
	} else if (notManifoldNeigh.size() == 0) {
		for (int curV = 0; curV < 4; ++curV) {
			currentTet->vertex(curV)->info().setNotUsed(false);
		}
	}

//Check if the neigh of the added tetrahedron still belongs to the boundary
	for (int curNeighId = 0; curNeighId < 4; ++curNeighId) {
		Delaunay3::Cell_handle currNeigh = currentTet->neighbor(curNeighId);
//if needed, remove the neighbor of currentTet from the boundary if no facet belongs  to the manifold
//note that isBoundary function returns the state flag of the tetrahedron, while
//isInBoundary() check for the current real state of the tetrahedron inside the triangulation after
// the new tetrahedron has been added
		if (currNeigh->info().isBoundary()) {
			//We nested this if since it is more efficient to test firstly
			// if the boundary state flag of the tetrahedron is set true
			if (!isInBoundary(currNeigh)) {
				removeFromBoundary(currNeigh);
			}
		} else {
			if (isInBoundary(currNeigh)) {
				insertInBoundary(currNeigh);
			}
		}
	}
}

/*
 * 	Remove the current tetrahedron from the manifold set and from the boundary set,
 * 	and add or remove its neighbours from the boundary set accordingly.
 */
void ManifoldManager::subTetAndUpdateBoundary(Delaunay3::Cell_handle &currentTet, std::vector<Delaunay3::Cell_handle>& newBoundaryTets) {
	std::vector<int> notManifoldNeigh;
	isInBoundary(currentTet, notManifoldNeigh);

	removeFromBoundary(currentTet);

	currentTet->info().setKeptManifold(false);

	if (notManifoldNeigh.size() == 3) {
		for (int curV = 0; curV < 4; ++curV) {
			if (find(notManifoldNeigh.begin(), notManifoldNeigh.end(), curV) == notManifoldNeigh.end()) {
				currentTet->vertex(curV)->info().setUsed(0);
			}
		}
	} else if (notManifoldNeigh.size() == 4) {
		for (int curV = 0; curV < 4; ++curV) {
			currentTet->vertex(notManifoldNeigh[curV])->info().setUsed(0);
		}
	}

	if (notManifoldNeigh.size() == 1) {      //when you fill a "hole"
		currentTet->vertex(notManifoldNeigh[0])->info().setNotUsed(true);
	} else if (notManifoldNeigh.size() == 4) {
		for (int curV = 0; curV < 4; ++curV) {
			currentTet->vertex(notManifoldNeigh[curV])->info().setNotUsed(true);
		}
	}
	currentTet->info().setShrinked(true);

// Check for each neighbour whether it should belong to the boundary set
	for (int curNeighId = 0; curNeighId < 4; ++curNeighId) {
		Delaunay3::Cell_handle currNeigh = currentTet->neighbor(curNeighId);
// ((
//if needed, add the neighbor of currentTet from the boundary if at least one facet belongs  to the manifold
//note that isBoundary function returns the state flag of the tetrahedron, while
//isInBoundary() check for the current real state of the tetrahedron inside the triangulation after
// the new tetrahedron has been added
// ))

// If the neighbour wasn't in the boundary set before, it is now on the only condition of being in the manifold set,
// in fact the remaining condition is satisfied. (Namely, it has at least a neighbour outside the manifold set, that is currentTet).
		if (!currNeigh->info().isBoundary()) {

			// ((
			//We nested this "if" since it is more efficient to test firstly
			// if the boundary state flag of the tetrahedron is set to false
			// ))

			if (currNeigh->info().iskeptManifold()) {
				insertInBoundary(currNeigh);
				newBoundaryTets.push_back(currNeigh);
			}
		} else {
			if (!currNeigh->info().iskeptManifold()) {
				cerr << "cell such that iskeptManifold() == false && isBoundary() == true" << endl;
				removeFromBoundary(currNeigh);
			}
		}
	}
}

void ManifoldManager::regionGrowingProcedure() {

	bool somethingIsChanged = false;
	bool singleTetGrowingMode = true;

//populate the list with the boundary tetrahedra not belonging to the carved space
	std::vector<Delaunay3::Cell_handle> tetsQueue;
	for (auto itBound = boundaryCells_.begin(); itBound != boundaryCells_.end(); itBound++) {
		for (int curIdx = 0; curIdx < 4; ++curIdx) {
			Delaunay3::Cell_handle curTet = (*itBound)->neighbor(curIdx);
			if (!dt_.is_cell((*itBound)) || !dt_.is_cell(curTet)) {
				outputM_->writeBoundaryOFF("temp.off", boundaryCells_);

				std::cerr << "Something's wrong" << std::endl;

				boundaryCells_.erase(itBound);
			} else {
				if (!curTet->info().iskeptManifold() && !curTet->info().isToBeTested() && isFreespace(curTet)) {

					std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), curTet, sortTetByIntersection());
					tetsQueue.insert(it, curTet);

					curTet->info().setToBeTested(true);
				}
			}
		}
	}

	if (tetsQueue.size() == 0) {
		std::cerr << "ManifoldManager::regionGrowingProcedure(): tetsQueue.size() == 0" << std::endl << std::endl;
		return;
	}

	Delaunay3::Cell_handle currentTet = tetsQueue.back();

	int curIter = 0;
//Growing process
	somethingIsChanged = true;
	int iC = 0;
	OutputCreator oc(dt_);
	std::stringstream s;
	std::vector<Delaunay3::Cell_handle> tetNotCarved;
	while (somethingIsChanged) {
		somethingIsChanged = false;
		while (!tetsQueue.empty()) {

			//Single-tet-at-once
			Delaunay3::Cell_handle currentTet = tetsQueue.back();
			tetsQueue.pop_back();

			currentTet->info().setToBeTested(false);

//      if (!currentTet->info().iskeptManifold() && isFreespace(currentTet)) {
			//if (true) {

			/* if (singleTetTest(currentTet)!=additionTest(currentTet)) {
			 additionTest(currentTet);
			 singleTetTest(currentTet);
			 }*/
			if (singleTetTest(currentTet) == true) {
				//   if (additionTest(currentTet) == true) {

				addTetAndUpdateBoundary(currentTet);
				/*
				 for (int curV = 0; curV < 4; ++curV) {

				 Delaunay3::Vertex_handle v = currentTet->vertex(curV);
				 if (!isRegular(v)) {
				 std::cout << utilities::printTet(currentTet) << std::endl;
				 std::cout << "1NAAAAAAA" << std::endl;
				 }
				 }*/

				//add the adjacent tets to the queue
				for (int curNeig = 0; curNeig < 4; ++curNeig) {

					Delaunay3::Cell_handle neigh = currentTet->neighbor(curNeig);

					if (!currentTet->neighbor(curNeig)->info().iskeptManifold() && isFreespace(neigh)) {
						if (find(tetsQueue.begin(), tetsQueue.end(), currentTet->neighbor(curNeig)) == tetsQueue.end()) {

							//keep the list ordered
							std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), neigh, sortTetByIntersection());
							tetsQueue.insert(it, neigh);

						}

					}
				}

			} else {
				currentTet->info().setKeptManifold(true);
				/* bool numReg = 0;
				 for (int curV = 0; curV < 4; ++curV) {
				 Delaunay3::Vertex_handle v = currentTet->vertex(curV);

				 if (isRegular(v))
				 numReg++;
				 }

				 if (numReg == 4)
				 std::cout << "2NAAAAAAA" << std::endl;*/

				currentTet->info().setKeptManifold(false);
				std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetNotCarved.begin(), tetNotCarved.end(), currentTet, sortTetByIntersection());
				tetNotCarved.insert(it, currentTet);
			}
			//}

			curIter++;
		}

		tetsQueue.insert(tetsQueue.begin(), tetNotCarved.begin(), tetNotCarved.end());

		tetNotCarved.clear();

	}
	tetsQueue.clear();

	/* fileOut_ << "Bound ";
	 printBound();
	 fileOut_ << std::endl;*/
}

void ManifoldManager::growSeveralAtOnce() {
	std::vector<Delaunay3::Cell_handle> tetsQueue;
	for (auto itBound : boundaryCells_) {
		for (int curIdx = 0; curIdx < 4; ++curIdx) {
			Delaunay3::Cell_handle curTet = itBound->neighbor(curIdx);
			if (!dt_.is_cell(itBound) || !dt_.is_cell(curTet)) {
				std::cerr << "Something's wrong" << std::endl;
			}
			if (itBound->info().iskeptManifold() && !curTet->info().iskeptManifold() && !curTet->info().isToBeTested() && isFreespace(curTet)) {

				std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), curTet, sortTetByIntersection());
				tetsQueue.insert(it, curTet);
				curTet->info().setToBeTested(true);
			}
		}
	}

	while (!tetsQueue.empty()) {
		Delaunay3::Cell_handle currentTet = tetsQueue.back();
		tetsQueue.pop_back();
		currentTet->info().setToBeTested(false);

		bool id[4] = { true, true, true, true };
		for (int curIdx = 0; curIdx < 4; ++curIdx) {
			if (id[curIdx] == true) {

				removeAndCheckManifoldness(currentTet, curIdx);
			}
		}
	}
}

void ManifoldManager::shrinkSeveralAtOnce(const PointD3 &camPosition, const float &maxPointToPointDistance, const float &maxPointToCamDistance) {

	std::deque<Delaunay3::Cell_handle> tetsQueue;
	tetsQueue.insert(tetsQueue.begin(), boundaryCells_.begin(), boundaryCells_.end());

	while (!tetsQueue.empty()) {
		Delaunay3::Cell_handle currentTet = tetsQueue.back();
		tetsQueue.pop_back();
//		float maxValue = 0.0;
//		int idXMax;
//		bool found = false;

		float distance;
		bool isInEnclosingVolume = false;

		for (int curVertex = 0; curVertex < 4; ++curVertex) {
			PointD3 curPt = currentTet->vertex(curVertex)->point();
			distance = utilities::distanceEucl(curPt, camPosition);
			if (distance < maxPointToPointDistance + maxPointToCamDistance) {
				isInEnclosingVolume = true;
			}
		}

//		bool id[4] = { false, false, false, false };
//		for (int curIdx = 0; curIdx < 4; ++curIdx) {
//			Delaunay3::Cell_handle curTetNeigh = currentTet->neighbor(curIdx);
//			if (isFreespace(curTetNeigh) != isFreespace(currentTet)) {
//				for (int curIdxI = 0; curIdxI < 4; ++curIdxI) {
//					if (curIdxI != curIdx) {
//						id[curIdxI] = true;
//					}
//				}
//			}
//		}
		if (isInEnclosingVolume) {
			//bool id[4] = { true, true, true, true };
			for (int curIdx = 0; curIdx < 4; ++curIdx) {
				//if (id[curIdx] == true) {
				addAndCheckManifoldness(currentTet, curIdx);
				//}
			}
		}
	}
}

bool ManifoldManager::removeAndCheckManifoldness(Delaunay3::Cell_handle &currentTet, int curIdx) {
	std::vector<Delaunay3::Cell_handle> incidentCells, cellsModified;
	std::vector<Delaunay3::Vertex_handle> incidentV;
	Delaunay3::Vertex_handle curV = currentTet->vertex(curIdx);
	dt_.incident_cells(curV, std::back_inserter(incidentCells));
	dt_.adjacent_vertices(curV, std::back_inserter(incidentV));

	bool testFailed = false;

	for (auto ic : incidentCells) {
		if (isFreespace(ic)) {
			if (!ic->info().iskeptManifold()) {
				cellsModified.push_back(ic);
				ic->info().setKeptManifold(true);
			}
		} else {
//			foundNotFreespace = true;
//			cerr << "ManifoldManager::removeAndCheckManifoldness:\t trying to grow several cells over freespace boundary" << endl;
		}
	}

//	if (foundNotFreespace) {
//		for (auto ic : cellsModified) {
//			ic->info().setKeptManifold(false);
//		}
//		return false;
//	}

	for (auto iv : incidentV) {
		if (!isRegularProfiled(iv)) {
			testFailed = true;
			break;
		}
	}

	if (!testFailed && !isRegularProfiled(curV)) {
		testFailed = true;
	}

	if (testFailed) {

		for (auto ic : cellsModified)
			ic->info().setKeptManifold(false);

		return false;

	} else {

		for (auto ic : cellsModified)
			ic->info().setKeptManifold(false);

		for (auto ic : cellsModified) {
			addTetAndUpdateBoundary(ic);
		}

	}

	return true;
}

/*
 * 	?
 */
bool ManifoldManager::addAndCheckManifoldness(Delaunay3::Cell_handle &currentTet, int curIdx) {

	std::vector<Delaunay3::Cell_handle> incidentCells, cellsModified;
	std::vector<Delaunay3::Vertex_handle> incidentV;
	Delaunay3::Vertex_handle curV = currentTet->vertex(curIdx);
	dt_.incident_cells(curV, std::back_inserter(incidentCells));
	dt_.adjacent_vertices(curV, std::back_inserter(incidentV));

	for (auto ic : incidentCells) {
		if (ic->info().iskeptManifold()) {
			cellsModified.push_back(ic);
		}
		ic->info().setKeptManifold(false);
	}

	bool testFailed = false;

	for (auto iv : incidentV) {
//		if (!isRegular(iv)) {
		if (!isRegularProfiled(iv)) {
			testFailed = true;
			break;
		}
	}

//	if (!testFailed && !isRegular(curV)) {
	if (!testFailed && !isRegularProfiled(curV)) {
		testFailed = true;
	}

	if (testFailed) {

		for (auto ic : cellsModified)
			ic->info().setKeptManifold(true);

		return false;

	} else {

		for (auto ic : cellsModified)
			ic->info().setKeptManifold(true);

		for (auto ic : cellsModified) {
			std::vector<Delaunay3::Cell_handle> newBoundaryTets;
			subTetAndUpdateBoundary(ic, newBoundaryTets);
		}

	}

	return true;
}

bool ManifoldManager::checkManifoldness(Delaunay3::Cell_handle &cellToTest1, int idxNeigh) {
	Delaunay3::Vertex_handle v = cellToTest1->vertex(idxNeigh);
	Delaunay3::Cell_handle curTetNeigh = cellToTest1->neighbor(idxNeigh);

	if (!isRegular(v)) return false;

	for (int curNei = 0; curNei < 4; ++curNei) {
		Delaunay3::Vertex_handle vC = curTetNeigh->vertex(curNei);

		if (!isRegular(vC)) return false;
	}
	return true;
}

bool ManifoldManager::isRegularProfiled(Delaunay3::Vertex_handle &v) {
	functionProfileCounter_isRegular_++;

	functionProfileChronometer_isRegular_.start();

	bool r = isRegular(v);

	functionProfileChronometer_isRegular_.stop();

	return r;
}

bool ManifoldManager::isRegular(Delaunay3::Vertex_handle &v) {

	std::vector<Delaunay3::Cell_handle> incidentCells;
	std::vector<Delaunay3::Vertex_handle> incidentV;
	dt_.incident_cells(v, std::back_inserter(incidentCells));
	dt_.adjacent_vertices(v, std::back_inserter(incidentV));

	std::vector<Delaunay3::Vertex_handle> pathVert;
	std::vector<Delaunay3::Edge> pathEdge;
	std::vector<Delaunay3::Edge> pathEdgeUnique;
	/*look for mesh edges concurring in each vertex (incidentV) incident to v*/
	for (auto incV : incidentV) {
		/*Find one tetrahedron with edge v->incV*/
		auto c = incidentCells.begin();
		bool found = false;
		Delaunay3::Edge curEdge;
		Delaunay3::Cell_handle startingCell;

		while (c != incidentCells.end() && !found) {
			if ((*c)->has_vertex(incV)) {
				Delaunay3::Edge e((*c), (*c)->index(incV), (*c)->index(v));
				curEdge = e;
				found = true;
				startingCell = (*c);
			}
			c++;
		}

		if (found) {
			Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(curEdge, startingCell);
			Delaunay3::Cell_handle lastCell = cellCirc;
			bool lastManif = cellCirc->info().iskeptManifold();
			/*look for edges of the mesh*/
			do {
				cellCirc++;
				if (lastManif == !cellCirc->info().iskeptManifold()) {
					int curIdx;
					for (int curV = 0; curV < 4; ++curV) {/*look for the vertex along the mesh elements of the edge starting from incV*/
						Delaunay3::Vertex_handle curVonLast = cellCirc->vertex(curV);
						if (curV != cellCirc->index(v) && curV != cellCirc->index(incV) && lastCell->has_vertex(curVonLast)) {
							curIdx = curV;
						}
					}

					/*store the edge of the mesh*/
					pathEdge.push_back(Delaunay3::Edge(cellCirc, cellCirc->index(incV), curIdx));

				}
				lastManif = cellCirc->info().iskeptManifold();
				lastCell = cellCirc;
			} while (cellCirc != startingCell);
		}
	}

	std::vector<bool> testE(pathEdge.size(), true);

	for (int idxExt = 0; idxExt < pathEdge.size(); ++idxExt) {

		if (testE[idxExt]) {
			Delaunay3::Vertex_handle vE1 = pathEdge[idxExt].first->vertex(pathEdge[idxExt].second);
			Delaunay3::Vertex_handle vE2 = pathEdge[idxExt].first->vertex(pathEdge[idxExt].third);

			pathVert.push_back(vE1);
			pathVert.push_back(vE2);
			pathEdgeUnique.push_back(pathEdge[idxExt]);
			//Remove duplicates
			for (int idxInt = idxExt + 1; idxInt < pathEdge.size(); ++idxInt) {
				Delaunay3::Vertex_handle vI1 = pathEdge[idxInt].first->vertex(pathEdge[idxInt].second);
				Delaunay3::Vertex_handle vI2 = pathEdge[idxInt].first->vertex(pathEdge[idxInt].third);

				if ((vI1 == vE1 && vI2 == vE2) || (vI1 == vE2 && vI2 == vE1)) {
					testE[idxInt] = false;
				}
			}

		}
	}

	if (pathEdgeUnique.empty()) {
		return true;
	}
	std::vector<bool> yetVisited(pathEdgeUnique.size(), false);

	int curEdgeIdx = 0;

	Delaunay3::Vertex_handle initV = pathEdgeUnique[0].first->vertex(pathEdgeUnique[0].second);
	Delaunay3::Vertex_handle curV = pathEdgeUnique[0].first->vertex(pathEdgeUnique[0].second);

//yetVisited[curEdgeIdx] = true;

	do {
		int countConcurr = 0;
		int testEdgeOK = 0;

		for (int testEdge = 0; testEdge < pathEdgeUnique.size(); testEdge++) {
			if (curV == pathEdgeUnique[testEdge].first->vertex(pathEdgeUnique[testEdge].second) || curV == pathEdgeUnique[testEdge].first->vertex(pathEdgeUnique[testEdge].third)) {

				countConcurr++;

				if (yetVisited[testEdge] == false) {
					testEdgeOK = testEdge;
				}
			}
		}

		if (countConcurr != 2) {
			return false;
		}

// std::cout << "Step2:(" << curV->point().x() << ", " << curV->point().y() << "," << curV->point().z() << ") " << std::endl;
		if (curV == pathEdgeUnique[testEdgeOK].first->vertex(pathEdgeUnique[testEdgeOK].second)) {

			curV = pathEdgeUnique[testEdgeOK].first->vertex(pathEdgeUnique[testEdgeOK].third);

		} else if (curV == pathEdgeUnique[testEdgeOK].first->vertex(pathEdgeUnique[testEdgeOK].third)) {

			curV = pathEdgeUnique[testEdgeOK].first->vertex(pathEdgeUnique[testEdgeOK].second);
		} else {

			//std::cerr << "isRegular SOMETHING WRONG" << std::endl;
		}
//std::cout << "Step3:(" << curV->point().x() << ", " << curV->point().y() << "," << curV->point().z() << ") " << std::endl;

		yetVisited[testEdgeOK] = true;

	} while (curV != initV);

	for (auto v : yetVisited) {
		if (!v) {
			return false;
		}
	}
	return true;
}

bool ManifoldManager::isFreespace(Delaunay3::Cell_handle &cell) {
	bool value;
	if (!inverseConic_) {
		value = !cell->info().isKeptByVoteCount(probabTh_);
	} else {
		value = !cell->info().isKeptByVoteCountProb(probabTh_);      // && cell->info().getVoteCount()>=1.0;
	}
	return value;
}

bool ManifoldManager::additionTest(Delaunay3::Cell_handle &cell) {

	bool additionManifold;

	int numV = 0;
	int numFound = 0;

	for (int curVertexId = 0; curVertexId < 4; ++curVertexId) {

		if (cell->vertex(curVertexId)->info().isUsed() > 0) {
			numFound++;
		}

	}
	numV = numFound;
	int numE = 0;
	for (int curEdgeId1 = 0; curEdgeId1 < 4; ++curEdgeId1) {
		for (int curEdgeId2 = curEdgeId1 + 1; curEdgeId2 < 4; ++curEdgeId2) {
			bool intersectionFound = false;
			Delaunay3::Edge curEdge(cell, curEdgeId1, curEdgeId2);

			Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(curEdge, cell);
			Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(curEdge, cell);

			do {
				if (cellCirc->info().iskeptManifold()) {
					intersectionFound = true;
				}
				cellCirc++;
			} while (cellCirc != cellCircInit);

			if (intersectionFound) {
				numE++;
			}
		}
	}

	int numF = 0;
	for (int curNeighId = 0; curNeighId < 4; ++curNeighId) {
		bool intersectionFound = false;

		if (cell->neighbor(curNeighId)->info().iskeptManifold()) {
			intersectionFound = true;
		}
		if (intersectionFound) {
			numF++;
		}
	}

	if ((numV == 0 && numE == 0 && numF == 0) || (numV == 3 && numE == 3 && numF == 1) || (numV == 4 && numE == 5 && numF == 2) || (numV == 4 && numE == 6 && numF == 3) || (numV == 4 && numE == 6 && numF == 4)) {
		additionManifold = true;
	} else {
		additionManifold = false;
	}
	return additionManifold;

}

bool ManifoldManager::singleTetTest(Delaunay3::Cell_handle &cell) {
	bool iskeptManif = cell->info().iskeptManifold();

	/*COUNT NUM VERTICES in the intersection between tet cell and the current manifold*/
	int numV = 0;
	for (int curVertexId = 0; curVertexId < 4; ++curVertexId) {
		std::vector<Delaunay3::Cell_handle> incidentCells;
		dt_.incident_cells(cell->vertex(curVertexId), std::back_inserter(incidentCells));
		auto it = incidentCells.begin();
		bool found = false;
		while (it != incidentCells.end() && found == false) {
			if ((*it)->info().iskeptManifold() != iskeptManif) {
				found = true;
			}
			it++;
		}

		if (found) {
			numV++;
		}
	}

	/*COUNT NUM Edges in the intersection between tet cell and the current manifold*/
	int numE = 0;
	for (int curEdgeId1 = 0; curEdgeId1 < 4; ++curEdgeId1) {
		for (int curEdgeId2 = curEdgeId1 + 1; curEdgeId2 < 4; ++curEdgeId2) {
			bool intersectionFound = false;
			Delaunay3::Edge curEdge(cell, curEdgeId1, curEdgeId2);

			Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(curEdge, cell);
			Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(curEdge, cell);

			do {
				if (cellCirc->info().iskeptManifold() != iskeptManif) {
					intersectionFound = true;
				}
				cellCirc++;
			} while (cellCirc != cellCircInit && intersectionFound == false);

			if (intersectionFound) {
				numE++;
			}
		}
	}

	/*COUNT NUM Facets in the intersection between tet cell and the current manifold*/
	int numF = 0;
	for (int curNeighId = 0; curNeighId < 4; ++curNeighId) {
		bool intersectionFound = false;

		if (cell->neighbor(curNeighId)->info().iskeptManifold() != iskeptManif) {
			numF++;
		}
	}

	if ((numV == 0 && numE == 0 && numF == 0) || (numV == 3 && numE == 3 && numF == 1) || (numV == 4 && numE == 5 && numF == 2) || (numV == 4 && numE == 6 && numF == 3) || (numV == 4 && numE == 6 && numF == 4)) {
		return true;
	} else {
		return false;
	}

}
bool ManifoldManager::subtractionTest(Delaunay3::Cell_handle &i) {

	bool subtractionManifold;

	int numV = 0;
	int numFound = 0;
	bool iskeptManif = i->info().iskeptManifold();

	for (int curVertexId = 0; curVertexId < 4; ++curVertexId) {

		if (i->vertex(curVertexId)->info().isNotUsed()) {
			numFound++;
		}
	}
	numV = numFound;

	int numE = 0;
	for (int curEdgeId1 = 0; curEdgeId1 < 4; ++curEdgeId1) {
		for (int curEdgeId2 = curEdgeId1 + 1; curEdgeId2 < 4; ++curEdgeId2) {
			bool intersectionFound = false;
			Delaunay3::Edge curEdge(i, curEdgeId1, curEdgeId2);

			Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(curEdge, i);
			Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(curEdge, i);

			do {
				if (cellCirc->info().iskeptManifold() != iskeptManif) {
					intersectionFound = true;
				}
				cellCirc++;
			} while (cellCirc != cellCircInit && intersectionFound == false);

			if (intersectionFound) {
				numE++;
			}
		}
	}

	/*COUNT NUM Facets in the intersection between tet cell and the current manifold*/
	int numF = 0;
	for (int curNeighId = 0; curNeighId < 4; ++curNeighId) {
		bool intersectionFound = false;

		if (i->neighbor(curNeighId)->info().iskeptManifold() != iskeptManif) {
			numF++;
		}
	}
	if ((numV == 0 && numE == 0 && numF == 0) || (numV == 3 && numE == 3 && numF == 1) || (numV == 4 && numE == 5 && numF == 2) || (numV == 4 && numE == 6 && numF == 3) || (numV == 4 && numE == 6 && numF == 4)) {
		subtractionManifold = true;
	} else {
		subtractionManifold = false;
	}
	return subtractionManifold;

}

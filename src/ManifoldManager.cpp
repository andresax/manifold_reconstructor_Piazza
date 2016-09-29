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

ManifoldManager::ManifoldManager(Delaunay3& dt, bool inverseConic, float probabTh, ManifoldReconstructionConfig conf) :
		dt_(dt) {

	conf_ = conf;
	inverseConic_ = inverseConic;
	probabTh_ = probabTh;
	outputM_ = new OutputCreator(dt_);
}

ManifoldManager::~ManifoldManager() {
//	boundaryCells_.clear();
	boundaryCellsSpatialMap_.clear();
}

void ManifoldManager::shrinkManifold3(const std::set<PointD3>& points, const float& maxPointToPointDistance, long currentEnclosingVersion) {
	// Stats
//	int countQueueInitCells, countBoundaryInitCells;
	int countInEnclosingVolume = 0, countShrinked = 0, countTotal = 0, countIterations = 1;
	int countSuccessfulEnclosingVersionCache = 0, countSuccessfulLastEnclosingPointCache = 0;
	Chronometer chronoQueueInit, chronoQueueInserting, chronoQueuePopping, chronoInserting, chronoEnclosing, chronoLastEnclosingPointCache, chronoTesting, chronoShrinking;

	float maxPointToPointDistanceSquared = std::pow(maxPointToPointDistance, 2);

	bool fixedPoint = false;
	std::vector<Delaunay3::Cell_handle> tetNotCarved;
	std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> tetsQueue;

	// Populate the list with the boundary tetrahedra
	chronoQueueInit.start();

	std::set<index3> mapIndices;
	for (auto p : points) {
		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(p.x() / conf_.steinerGridStepLength);
		int j = std::floor(p.y() / conf_.steinerGridStepLength);
		int k = std::floor(p.z() / conf_.steinerGridStepLength);

		for (int i_ : std::vector<int> { -1, 0, 1 })
			for (int j_ : std::vector<int> { -1, 0, 1 })
				for (int k_ : std::vector<int> { -1, 0, 1 })
					mapIndices.insert(index3(i + i_, j + j_, k + k_));
	}

	for (index3 mapIndex : mapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		tetsQueue.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

//	countBoundaryInitCells = boundaryCells_.size();
//	countQueueInitCells = tetsQueue.size();

//	std::sort(tetsQueue.begin(), tetsQueue.end());
	chronoQueueInit.stop();

	while (!fixedPoint) {

		fixedPoint = true;

		// Shrink process
		while (!tetsQueue.empty()) {
			countTotal++;

//			Delaunay3::Cell_handle currentTet = tetsQueue.front();
//			tetsQueue.pop_front();
			chronoQueuePopping.start();
			Delaunay3::Cell_handle currentTet = *tetsQueue.begin();
			tetsQueue.erase(tetsQueue.begin());
			chronoQueuePopping.stop();

			bool isInEnclosingVolume = false;

			chronoEnclosing.start();
			if (currentTet->info().getEnclosingVersion() == currentEnclosingVersion) {
				countSuccessfulEnclosingVersionCache++;
				isInEnclosingVolume = currentTet->info().isInEnclosingVolume();
			} else {
				chronoLastEnclosingPointCache.start();
				if (currentTet->info().hasLastEnclosingPoint() && points.count(currentTet->info().getLastEnclosingPoint())) {
					countSuccessfulLastEnclosingPointCache++;
					isInEnclosingVolume = true;
					chronoLastEnclosingPointCache.stop();
				} else {
					chronoLastEnclosingPointCache.stop();

					for (int vertexId = 0; vertexId < 4; ++vertexId) {
						for (auto p : points) {
							if (utilities::distanceEuclSquared(currentTet->vertex(vertexId)->point(), p) < maxPointToPointDistanceSquared) {
								isInEnclosingVolume = true;
								currentTet->info().setLastEnclosingPoint(p);
								break;
							}
						}
						if (isInEnclosingVolume) break;
					}
				}

				currentTet->info().setInEnclosingVolume(isInEnclosingVolume, currentEnclosingVersion);
			}
			chronoEnclosing.stop();

			if (isInEnclosingVolume) countInEnclosingVolume++;
			else continue;

			chronoTesting.start();
			if (singleTetTest2(currentTet)) {
				chronoTesting.stop();

				if (!currentTet->info().iskeptManifold()) cerr << "ManifoldManager::shrinkManifold3:\t wrong shrink order\t iteration: " << countIterations << endl;

				countShrinked++;

				chronoShrinking.start();
				std::vector<Delaunay3::Cell_handle> newBoundaryTets;
				subTetAndUpdateBoundary2(currentTet, newBoundaryTets);
				chronoShrinking.stop();

				for (auto neighbour : newBoundaryTets) {
					if (neighbour->info().iskeptManifold()) {
						chronoQueueInserting.start();
//						std::deque<Delaunay3::Cell_handle>::iterator it;
//						it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), neighbour, sortTetByIntersection());
//						tetsQueue.insert(it, neighbour);
						tetsQueue.insert(neighbour);
						chronoQueueInserting.stop();
						fixedPoint = false;

					} else cerr << "ManifoldManager::shrinkManifold3:\t subTetAndUpdateBoundary return non inManifoldSet tet\t iteration: " << countIterations << endl;
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
//			tetsQueue.insert(tetsQueue.begin(), tetNotCarved.begin(), tetNotCarved.end());
			tetsQueue.insert(tetNotCarved.begin(), tetNotCarved.end());
			tetNotCarved.clear();
		}
	}
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t lastEnclosingPointCache:\t\t" << chronoLastEnclosingPointCache.getMicroseconds() << " µs" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t queue inserting:\t\t\t" << chronoQueueInserting.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t queue popping:\t\t\t\t" << chronoQueuePopping.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t inserting:\t\t\t\t" << chronoInserting.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t enclosing:\t\t\t\t" << chronoEnclosing.getSeconds() << " s\t / \t" << countTotal << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t test:\t\t\t\t\t" << chronoTesting.getSeconds() << " s\t / \t" << countInEnclosingVolume << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t shrink:\t\t\t\t" << chronoShrinking.getSeconds() << " s\t / \t" << countShrinked << endl;
//	cout << "ManifoldManager::shrinkManifold3:\t\t\t\t countQueueInitCells:\t\t\t\t\t" << countQueueInitCells << "\t / \t" << countBoundaryInitCells << endl;
	cout << "ManifoldManager::shrinkManifold3:\t\t\t\t countSuccessfulEnclosingVersionCache:\t\t\t" << countSuccessfulEnclosingVersionCache << "\t / \t" << countTotal << endl;
	cout << "ManifoldManager::shrinkManifold3:\t\t\t\t countSuccessfulLastEnclosingPointCache:\t\t" << countSuccessfulLastEnclosingPointCache << "\t / \t" << countTotal - countSuccessfulEnclosingVersionCache << endl;
	cout << "ManifoldManager::shrinkManifold3:\t\t\t\t number_of_finite_cells:\t\t" << dt_.number_of_finite_cells() << endl;

}

void ManifoldManager::shrinkSeveralAtOnce3(const std::set<PointD3>& points, const float &maxPointToPointDistance, long currentEnclosingVersion) {
	functionProfileChronometer_isRegular_.reset();
	functionProfileCounter_isRegular_ = 0;
	Chronometer chronoQueueInit, chronoQueuePopping, chronoEnclosing, chronoAddAndCheckManifoldness;

	float maxPointToPointDistanceSquared = std::pow(maxPointToPointDistance, 2);

//	std::deque<Delaunay3::Cell_handle> tetsQueue;
//	tetsQueue.insert(tetsQueue.begin(), boundaryCells_.begin(), boundaryCells_.end());

	std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> tetsQueue;

	// Populate the list with the boundary tetrahedra
	chronoQueueInit.start();

	std::set<index3> mapIndices;
	for (auto p : points) {
		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(p.x() / conf_.steinerGridStepLength);
		int j = std::floor(p.y() / conf_.steinerGridStepLength);
		int k = std::floor(p.z() / conf_.steinerGridStepLength);

		for (int i_ : std::vector<int> { -1, 0, 1 })
			for (int j_ : std::vector<int> { -1, 0, 1 })
				for (int k_ : std::vector<int> { -1, 0, 1 })
					mapIndices.insert(index3(i + i_, j + j_, k + k_));
	}

	for (index3 mapIndex : mapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		tetsQueue.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

//	int countBoundaryInitCells = boundaryCells_.size();
//	int countQueueInitCells = tetsQueue.size();

	//	std::sort(tetsQueue.begin(), tetsQueue.end());
	chronoQueueInit.stop();

	while (!tetsQueue.empty()) {
//		Delaunay3::Cell_handle currentTet = tetsQueue.back();
//		tetsQueue.pop_back();

		chronoQueuePopping.start();
		Delaunay3::Cell_handle currentTet = *tetsQueue.begin();
		tetsQueue.erase(tetsQueue.begin());
		chronoQueuePopping.stop();

		bool isInEnclosingVolume = false;

		chronoEnclosing.start();

		if (currentTet->info().getEnclosingVersion() == currentEnclosingVersion) {
			isInEnclosingVolume = currentTet->info().isInEnclosingVolume();
		} else {
			for (int vertexId = 0; vertexId < 4; ++vertexId) {
				for (auto p : points) {
					if (utilities::distanceEuclSquared(currentTet->vertex(vertexId)->point(), p) < maxPointToPointDistanceSquared) {
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

				bool success = subSeveralAndCheckManifoldness2(currentTet, curIdx);

				chronoAddAndCheckManifoldness.stop();

				if (success) break;
			}
		}
	}

	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t queue popping:\t\t\t\t" << chronoQueuePopping.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t enclosing:\t\t\t\t" << chronoEnclosing.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t addAndCheckManifoldness:\t\t" << chronoAddAndCheckManifoldness.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t isRegular:\t\t\t\t" << functionProfileChronometer_isRegular_.getSeconds() << " s\t / \t" << functionProfileCounter_isRegular_ << endl;
//	cout << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t countQueueInitCells:\t\t\t\t\t" << countQueueInitCells << "\t / \t" << countBoundaryInitCells << endl;

}

void ManifoldManager::regionGrowingBatch3(Delaunay3::Cell_handle& startingCell, const std::set<PointD3>& points) {
	addTetAndUpdateBoundary2(startingCell);
	regionGrowingProcedure3(points);
}

void ManifoldManager::regionGrowing3(const std::set<PointD3>& points) {
	regionGrowingProcedure3(points);
}

void ManifoldManager::regionGrowingProcedure3(const std::set<PointD3>& points) {
	// Stats
//	int countQueueInitCells, countBoundaryInitCells;
	int countGrowned = 0, countTotal = 0, countIterations = 1;
	Chronometer chronoQueueInit, chronoQueueInserting, chronoQueuePopping, chronoInserting, chronoTesting, chronoGrowing;

	bool fixedPoint = false;

	std::vector<Delaunay3::Cell_handle> tetsQueue;
	std::vector<Delaunay3::Cell_handle> tetNotCarved;
	std::set<Delaunay3::Cell_handle> localBoundaryCellsUnion;

	chronoQueueInit.start();

	std::set<index3> mapIndices;
	for (auto p : points) {
		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(p.x() / conf_.steinerGridStepLength);
		int j = std::floor(p.y() / conf_.steinerGridStepLength);
		int k = std::floor(p.z() / conf_.steinerGridStepLength);

		for (int i_ : std::vector<int> { -2, -1, 0, 1, 2 })
			for (int j_ : std::vector<int> { -2, -1, 0, 1, 2 })
				for (int k_ : std::vector<int> { -2, -1, 0, 1, 2 })
					mapIndices.insert(index3(i + i_, j + j_, k + k_));
	}

	for (index3 mapIndex : mapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		localBoundaryCellsUnion.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

	for (auto boundaryCell : localBoundaryCellsUnion) {
		for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
			Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(neighbourIndex);

			if (!dt_.is_cell(boundaryCell) || !dt_.is_cell(neighbour)) {
				std::cerr << "ManifoldManager::regionGrowingProcedure3: \t\t dead cells found in boundary" << std::endl;
				continue;
			}

			if (!neighbour->info().iskeptManifold() && !neighbour->info().isToBeTested() && isFreespace(neighbour)) {
				std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), neighbour, sortTetByIntersection());
				tetsQueue.insert(it, neighbour);
				neighbour->info().setToBeTested(true);
			}
		}
	}
	chronoQueueInit.stop();

//	countBoundaryInitCells = boundaryCells_.size();
//	countQueueInitCells = tetsQueue.size();

//	if (tetsQueue.size() == 0) {
//		std::cerr << "ManifoldManager::regionGrowingProcedure3: \t\t tetsQueue is empty" << std::endl;
//	}

	while (!fixedPoint) {
		fixedPoint = true;

		while (!tetsQueue.empty()) {
			countTotal++;

			//Single-tet-at-once
			chronoQueuePopping.start();
			Delaunay3::Cell_handle currentTet = tetsQueue.back();
			tetsQueue.pop_back();
			chronoQueuePopping.stop();

			currentTet->info().setToBeTested(false);

			chronoTesting.start();
			if (singleTetTest2(currentTet)) {
				chronoTesting.stop();

				countGrowned++;

				chronoGrowing.start();
				addTetAndUpdateBoundary2(currentTet);
				chronoGrowing.stop();

				//add the adjacent tets to the queue
				for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
					Delaunay3::Cell_handle neighbour = currentTet->neighbor(neighbourIndex);

					if (!neighbour->info().iskeptManifold() && isFreespace(neighbour)) {
						// TODO use toBeTested instead of search
						chronoQueueInserting.start();
						if (find(tetsQueue.begin(), tetsQueue.end(), neighbour) == tetsQueue.end()) {
							std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), neighbour, sortTetByIntersection());
							tetsQueue.insert(it, neighbour);
						}
						chronoQueueInserting.stop();

					}
				}

			} else {
				chronoTesting.stop();

				currentTet->info().setKeptManifold(false); // TODO useful?

				chronoInserting.start();
				std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetNotCarved.begin(), tetNotCarved.end(), currentTet, sortTetByIntersection());
				tetNotCarved.insert(it, currentTet);
				chronoInserting.stop();
			}
		}

		tetsQueue.insert(tetsQueue.begin(), tetNotCarved.begin(), tetNotCarved.end());
		tetNotCarved.clear();

		countIterations++;
	}

	// TODO is only 1 iteration enough?
//	if (tetsQueue.size()) cerr << "ManifoldManager::regionGrowingProcedure3: \t\t exiting while queue not empty" << endl;
	tetsQueue.clear();

	cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t queue inserting:\t\t\t" << chronoQueueInserting.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t queue popping:\t\t\t\t" << chronoQueuePopping.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t inserting:\t\t\t\t" << chronoInserting.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t test:\t\t\t\t\t" << chronoTesting.getSeconds() << " s\t / \t" << countTotal << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t grow:\t\t\t\t\t" << chronoGrowing.getSeconds() << " s\t / \t" << countGrowned << endl;
//	cout << "ManifoldManager::regionGrowingProcedure3:\t\t\t countQueueInitCells:\t\t\t\t\t" << countQueueInitCells << "\t / \t" << countBoundaryInitCells << endl;
	cout << "ManifoldManager::regionGrowingProcedure3:\t\t\t number_of_finite_cells:\t\t" << dt_.number_of_finite_cells() << endl;

}

void ManifoldManager::growSeveralAtOnce3(const std::set<PointD3>& points) {
	functionProfileChronometer_isRegular_.reset();
	functionProfileCounter_isRegular_ = 0;
	Chronometer chronoQueueInit, chronoRemoveAndCheckManifoldness;
	int countTotal = 0, countSuccess = 0;

	std::vector<Delaunay3::Cell_handle> tetsQueue;
	std::set<Delaunay3::Cell_handle> localBoundaryCellsUnion;

	chronoQueueInit.start();

	std::set<index3> mapIndices;
	for (auto p : points) {
		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(p.x() / conf_.steinerGridStepLength);
		int j = std::floor(p.y() / conf_.steinerGridStepLength);
		int k = std::floor(p.z() / conf_.steinerGridStepLength);

		for (int i_ : std::vector<int> { -2, -1, 0, 1, 2 })
			for (int j_ : std::vector<int> { -2, -1, 0, 1, 2 })
				for (int k_ : std::vector<int> { -2, -1, 0, 1, 2 })
					mapIndices.insert(index3(i + i_, j + j_, k + k_));
	}

	for (index3 mapIndex : mapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		localBoundaryCellsUnion.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

	for (auto boundaryCell : localBoundaryCellsUnion) {

		if (!dt_.is_cell(boundaryCell)) std::cerr << "ManifoldManager::growSeveralAtOnce3: dead cell in boundary" << std::endl;

		for (int neighbourId = 0; neighbourId < 4; ++neighbourId) {
			Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(neighbourId);

			if (!dt_.is_cell(neighbour)) std::cerr << "ManifoldManager::growSeveralAtOnce3: dead cell in boundary" << std::endl;

			if (boundaryCell->info().iskeptManifold() && !neighbour->info().iskeptManifold() && !neighbour->info().isToBeTested() && isFreespace(neighbour)) {

				std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(tetsQueue.begin(), tetsQueue.end(), neighbour, sortTetByIntersection());
				tetsQueue.insert(it, neighbour);
				neighbour->info().setToBeTested(true);
			}
		}
	}

	chronoQueueInit.stop();

	chronoRemoveAndCheckManifoldness.start();

	while (!tetsQueue.empty()) {
		Delaunay3::Cell_handle currentTet = tetsQueue.back();
		tetsQueue.pop_back();

		countTotal++;

		if (currentTet->info().isBoundary()) break;

		// TODO before or after aborting the cycle?
		currentTet->info().setToBeTested(false);

		for (int curIdx = 0; curIdx < 4; ++curIdx) {
			// TODO check on non boundary vertices first
			// TODO stop when success?
			bool success = addSeveralAndCheckManifoldness2(currentTet, curIdx);

			if (success) countSuccess++;
			if (success) break;
		}
	}

	chronoRemoveAndCheckManifoldness.stop();

	cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce3:\t\t\t\t queue init:\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce3:\t\t\t\t remove and check manifoldness:\t\t" << chronoRemoveAndCheckManifoldness.getSeconds() << " s" << endl;
	cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce3:\t\t\t\t isRegular:\t\t\t\t" << functionProfileChronometer_isRegular_.getSeconds() << " s\t / \t" << functionProfileCounter_isRegular_ << endl;
	cout << "ManifoldManager::growSeveralAtOnce3:\t\t\t\t successfully growned:\t\t\t" << countSuccess << "\t / \t" << countTotal << endl;

}

bool ManifoldManager::addSeveralAndCheckManifoldness2(Delaunay3::Cell_handle& currentTet, int curIdx) {
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
		}
	}

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

		for (auto ic : cellsModified)
			addTetAndUpdateBoundary2(ic);

	}

	return true;
}

bool ManifoldManager::subSeveralAndCheckManifoldness2(Delaunay3::Cell_handle& currentTet, int curIdx) {

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
			subTetAndUpdateBoundary2(ic, newBoundaryTets);
		}

	}

	return true;
}

void ManifoldManager::addTetAndUpdateBoundary2(Delaunay3::Cell_handle& currentTet) {

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
void ManifoldManager::subTetAndUpdateBoundary2(Delaunay3::Cell_handle& currentTet, std::vector<Delaunay3::Cell_handle>& newBoundaryTets) {
	std::vector<int> notManifoldNeigh;
	isInBoundary(currentTet, notManifoldNeigh);

	removeFromBoundary(currentTet);

	currentTet->info().setKeptManifold(false);

	currentTet->info().setShrinked(true);

	// Check for each neighbour whether it should belong to the boundary set
	for (int curNeighId = 0; curNeighId < 4; ++curNeighId) {
		Delaunay3::Cell_handle currNeigh = currentTet->neighbor(curNeighId);

		// If the neighbour wasn't in the boundary set before, it is now on the only condition of being in the manifold set,
		// in fact the remaining condition is satisfied. (Namely, it has at least a neighbour outside the manifold set, that is currentTet).
		if (!currNeigh->info().isBoundary()) {

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

bool ManifoldManager::isInBoundary(Delaunay3::Cell_handle& cellToTest) {
	std::vector<int> toThrowAway;

	return isInBoundary(cellToTest, toThrowAway);
}

/*
 *	Returns true if cellToTest is kept manifold (?) and some neighbour of cellToTest isn't.
 *	If cellToTest is kept manifold, then neighNotManifold contains the indeces of the neighbours that aren't kept manifold.
 */
bool ManifoldManager::isInBoundary(Delaunay3::Cell_handle& cellToTest, std::vector<int>& neighNotManifold) {

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
bool ManifoldManager::insertInBoundary(Delaunay3::Cell_handle& cellToBeAdded) {
	chronoInsertInBoundary_.start();

	// TODO rewrite in a cleaner way
	if (!cellToBeAdded->info().isBoundary()) {
		cellToBeAdded->info().setBoundary(true);


//		std::vector<Delaunay3::Cell_handle>::iterator it = std::lower_bound(boundaryCells_.begin(), boundaryCells_.end(), cellToBeAdded, sortTetByIntersection());
//		if (it != boundaryCells_.end()) {
//			if (*it != cellToBeAdded) {
//				boundaryCells_.insert(it, cellToBeAdded);
//			}
//		} else {
//			boundaryCells_.push_back(cellToBeAdded);
//		}

		std::set<index3> mapIndices;
		// TODO is it possible that a tetrahedron intersects a grid cube but doesn't have any vertex within it? (shouldn't be a problem, since the adjacent cubes will be selected too)
		for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
			auto v = cellToBeAdded->vertex(vertexIndex);

			// The index of the grid's cube is the integer rounded down for each coordinate
			int i = std::floor(v->point().x() / conf_.steinerGridStepLength);
			int j = std::floor(v->point().y() / conf_.steinerGridStepLength);
			int k = std::floor(v->point().z() / conf_.steinerGridStepLength);
			mapIndices.insert(index3(i, j, k));
		}

		for (index3 mapIndex : mapIndices) {
			std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
			localBoundaryCells.insert(cellToBeAdded);

//			std::vector<Delaunay3::Cell_handle>::iterator itL = std::lower_bound(localBoundaryCells.begin(), localBoundaryCells.end(), cellToBeAdded, sortTetByIntersection());
//			if (itL != localBoundaryCells.end()) {
//				if (*itL != cellToBeAdded) {
//					localBoundaryCells.insert(itL, cellToBeAdded);
//				}
//			} else {
//				localBoundaryCells.push_back(cellToBeAdded);
//			}
		}

		chronoInsertInBoundary_.stop();
		return true;
	} else {
		chronoInsertInBoundary_.stop();
		return false;
	}

}

/*
 *	If cellToBeRemoved->info().isBoundary() is set to true, removes cellToBeRemoved from boundaryCells_ and set cellToBeRemoved->info().isBoundary() to false,
 *	otherwise does nothing.
 */
bool ManifoldManager::removeFromBoundary(Delaunay3::Cell_handle& cellToBeRemoved) {
	chronoRemoveFromBoundary_.start();
	if (!cellToBeRemoved->info().isBoundary()) {
		chronoRemoveFromBoundary_.stop();
		return false;
	} else {

//		std::vector<Delaunay3::Cell_handle>::iterator it2;

//		int num = 0;
//
//		std::vector<Delaunay3::Cell_handle>::iterator itmin = std::lower_bound(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved, sortTetByIntersection());
//		std::vector<Delaunay3::Cell_handle>::iterator itmax = std::upper_bound(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved, sortTetByIntersection());
//		it2 = std::find(itmin, itmax, cellToBeRemoved);
//		if ((*it2) == cellToBeRemoved) {
//			cellToBeRemoved->info().setBoundary(false);
//			boundaryCells_.erase(it2);
//			++num;
//		} else {
//			it2 = std::find(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved);
//			//while (it2 != boundaryCells_.end()) {
//			cellToBeRemoved->info().setBoundary(false);
//			boundaryCells_.erase(it2);
//			++num;
//			//it2 = std::find(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved);
//			// }
//		}
//
//		if (num == 0) {
//			std::cerr << "ManifoldManager::removeFromBoundary::notFound" << std::endl;
//		} else if (num > 1) {
//			std::cout << "ManifoldManager::removeFromBoundary::morethan1" << std::endl;
//		}

		cellToBeRemoved->info().setBoundary(false);

		std::set<index3> mapIndices;
		for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
			auto v = cellToBeRemoved->vertex(vertexIndex);

			// The index of the grid's cube is the integer rounded down for each coordinate
			int i = std::floor(v->point().x() / conf_.steinerGridStepLength);
			int j = std::floor(v->point().y() / conf_.steinerGridStepLength);
			int k = std::floor(v->point().z() / conf_.steinerGridStepLength);
			mapIndices.insert(index3(i, j, k));
		}

		for (index3 mapIndex : mapIndices) {
			std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
			localBoundaryCells.erase(cellToBeRemoved);
			cellToBeRemoved->info().setBoundary(false);

//			std::vector<Delaunay3::Cell_handle>::iterator itmin = std::lower_bound(localBoundaryCells.begin(), localBoundaryCells.end(), cellToBeRemoved, sortTetByIntersection());
//			std::vector<Delaunay3::Cell_handle>::iterator itmax = std::upper_bound(localBoundaryCells.begin(), localBoundaryCells.end(), cellToBeRemoved, sortTetByIntersection());
//			it2 = std::find(itmin, itmax, cellToBeRemoved);
//			if ((*it2) == cellToBeRemoved) {
//				cellToBeRemoved->info().setBoundary(false);
//				localBoundaryCells.erase(it2);
////				++num;
//			} else {
//				it2 = std::find(localBoundaryCells.begin(), localBoundaryCells.end(), cellToBeRemoved);
//				//while (it2 != boundaryCells_.end()) {
//				cellToBeRemoved->info().setBoundary(false);
//				localBoundaryCells.erase(it2);
////				++num;
//				//it2 = std::find(boundaryCells_.begin(), boundaryCells_.end(), cellToBeRemoved);
//				// }
//			}

		}

		chronoRemoveFromBoundary_.stop();
		return true;
	}
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

bool ManifoldManager::singleTetTest2(Delaunay3::Cell_handle& cell) {
	bool iskeptManif = cell->info().iskeptManifold();

	// Count the number of Facets in the intersection between cell and the current manifold
	int faceIndexI, faceIndexJ;
	int numF = 0;
	for (int faceIndex = 0; faceIndex < 4; ++faceIndex) {
		if (cell->neighbor(faceIndex)->info().iskeptManifold() != iskeptManif) {
			numF++;
			if (numF == 1) faceIndexI = faceIndex;
			if (numF == 2) faceIndexJ = faceIndex;
		}
	}

	// If numF == 0, then the test is true only if both numV and numE are 0.
	// If either numV or numE are greater than zero, the test is already determined and false.
	if (numF == 0) {

		// If even one vertex satisfies the condition, the test is false (numV > 0)
		for (int curVertexId = 0; curVertexId < 4; ++curVertexId) {
			std::vector<Delaunay3::Cell_handle> incidentCells;
			dt_.incident_cells(cell->vertex(curVertexId), std::back_inserter(incidentCells));

			for (auto c : incidentCells)
				if (c->info().iskeptManifold() != iskeptManif) return false; // shortcut

		}

		// If even one edge satisfies the condition, the test is false (numE > 0)
		for (int curEdgeId1 = 0; curEdgeId1 < 4; ++curEdgeId1) {
			for (int curEdgeId2 = curEdgeId1 + 1; curEdgeId2 < 4; ++curEdgeId2) {
				Delaunay3::Edge curEdge(cell, curEdgeId1, curEdgeId2);

				Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(curEdge, cell);
				Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(curEdge, cell);

				do {
					if (cellCirc->info().iskeptManifold() != iskeptManif) return false; // shortcut
					cellCirc++;
				} while (cellCirc != cellCircInit);

			}
		}

		return true;

	} else if (numF == 1) {

		// If numF == 1, then the test is true only if both numV and numE are 3,
		// but given that the condition is true for one and only one face (called face i),
		// then the vertex opposite to the face i (that is, vertex i), determines if the test is true.
		// In fact, the remaining vertices are incident to the cell adjacent to the face that already satisfied the condition,
		// hence the condition on those vertices is already satisfied.

		std::vector<Delaunay3::Cell_handle> incidentCells;
		dt_.incident_cells(cell->vertex(faceIndexI), std::back_inserter(incidentCells));

		for (auto c : incidentCells) {
			if (c->info().iskeptManifold() != iskeptManif) {
				return false;
			}
		}

		// If the condition is true for one and the only one face i,
		// then the condition must be true for all edges composing the face i and whether or not the condition is true for the vertex opposite to the face i (that is, vertex i),
		// it is redundant to check for the remaining edges, since the cells incident to the vertex i are also incident to them.
		// condition false for vertex i implies condition false for remaining edges (i, -)
		// condition true for vertex i implies the test is already determined (and false).

		return true;

	} else if (numF == 2) {

		// If two of the faces are adjacent the cells that make the condition true, then the condition is implied for all vertices and for all but one edge.
		// The only edge for which the condition needs to be tested is the one connecting the vertices opposite to the faces satisfing the condition.
		// Calling such faces i and j, said edge is the edge (i, j).

		Delaunay3::Edge edgeIJ(cell, faceIndexI, faceIndexJ);
		Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(edgeIJ, cell);
		Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(edgeIJ, cell);

		do {
			if (cellCirc->info().iskeptManifold() != iskeptManif) {
				return false;
			}
			cellCirc++;
		} while (cellCirc != cellCircInit);

		return true;

	} else if (numF >= 3) {

		// If all or even just three faces satisfy the condition, then it is implied for all vertices and all edges to satisfy the condition.

		return true;

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

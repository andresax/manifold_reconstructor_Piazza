/*
 * ManifoldMeshReconstructor.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <ManifoldMeshReconstructor.h>
#include <vector>
#include <utilities.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Logger.h>
#include <algorithm>
#include <math.h>

using std::cout;
using std::cerr;
using std::endl;
using std::pair;

ManifoldMeshReconstructor::ManifoldMeshReconstructor(ManifoldReconstructionConfig conf) {
	conf_ = conf;
	manifoldManager_ = new ManifoldManager(dt_, conf_.inverseConicEnabled, conf_.probOrVoteThreshold, conf_);
	outputM_ = new OutputCreator(dt_);

	stepX_ = stepY_ = stepZ_ = conf_.steinerGridStepLength;
	l_ = sqrt(stepX_ * stepX_ + stepY_ * stepY_ + stepZ_ * stepZ_);

	sgMinX_ = sgMinY_ = sgMinZ_ = sgMaxX_ = sgMaxY_ = sgMaxZ_ = 0.0;

	sgCurrentMinX_ = sgCurrentMinY_ = sgCurrentMinZ_ = 0.0;
	sgCurrentMaxX_ = sgCurrentMaxY_ = sgCurrentMaxZ_ = conf_.steinerGridStepLength;

	timeStatsFile_.open("output/stats/timeStats.csv");
	timeStatsFile_ << "cameras number, updateSteinerGrid, shrinkManifold, shrinkSingle, shrinkSeveral, Add new vertices, Move vertices, Move Cameras, rayUntracing, rayTracing, rayRetracing, growManifold, growManifoldSev, growManifold, Overall" << endl;
}

ManifoldMeshReconstructor::~ManifoldMeshReconstructor() {
	delete (manifoldManager_);
	delete (outputM_);
	timeStatsFile_.close();
}

void ManifoldMeshReconstructor::printWhatever() {

	int count_dead = 0, count_empty = 0;
	for (auto cell : freeSpaceTets_) {
		if (!dt_.is_cell(cell)) {
			count_dead++;
			continue;
		}
		if (cell->info().getIntersections().size() == 0) count_empty++;
	}
	//cout << "\td: " << count_dead << "\te: " << count_empty << "\t / " << freeSpaceTets_.size() << endl;
}

void ManifoldMeshReconstructor::setWeights(float w_1, float w_2, float w_3) {
	conf_.w_1 = w_1;
	conf_.w_2 = w_2;
	conf_.w_3 = w_3;
}

void ManifoldMeshReconstructor::clearLog() {
	fileOut_.close();
	fileOut_.open("ManifoldMeshReconstructor.log");
}

void ManifoldMeshReconstructor::addPoint(float x, float y, float z) {
	PointReconstruction t;
	t.idReconstruction = points_.size();
	t.position = PointD3(x, y, z);
	points_.push_back(t);
}

int ManifoldMeshReconstructor::addPointWhere(float x, float y, float z) {
	addPoint(x, y, z);
	return points_.size() - 1;
}

void ManifoldMeshReconstructor::movePoint(int idxPoint, float x, float y, float z) {
	points_[idxPoint].newPosition = PointD3(x, y, z);
	points_[idxPoint].toBeMoved = true;
	pointsMovedIdx_.push_back(idxPoint);

	//TODO check various conditions
//	updateSteinerGridTargetBounds(x, y, z);
}

void ManifoldMeshReconstructor::moveCamera(int idxCamera, float x, float y, float z) {
	cams_[idxCamera].newPosition = PointD3(x, y, z);
	cams_[idxCamera].toBeMoved = true;
	//camsPositions_[idxCamera] = cams_[idxCamera].newPosition; // TODO

	movedCamerasIdx_.push_back(idxCamera);
	updatedCamerasIdx_.insert(idxCamera);

	//TODO check various conditions
	updateSteinerGridTargetBounds(x, y, z);
}

//PointD3 ManifoldMeshReconstructor::movePointGetOld(int idxPoint, float x, float y, float z) {
//	points_[idxPoint].newPosition = PointD3(x, y, z);
//	pointsMovedIdx_.push_back(idxPoint);
//	return points_[idxPoint].position;
//}

void ManifoldMeshReconstructor::addCameraCenter(float x, float y, float z) {
	CamReconstruction t;
	t.idReconstruction = cams_.size();
	t.position = PointD3(x, y, z);
	glm::vec3 pos = glm::vec3(x, y, z);

	cams_.push_back(t);
	camsPositions_.push_back(pos);

	updatedCamerasIdx_.insert(t.idReconstruction);

}

void ManifoldMeshReconstructor::addVisibilityPair(int camIdx, int pointIdx) {
	CamReconstruction& c = cams_[camIdx];
	PointReconstruction& p = points_[pointIdx];

	c.visiblePoints.push_back(pointIdx);
	c.newVisiblePoints.push_back(pointIdx);
	p.viewingCams.push_back(camIdx);
	addRay(camIdx, pointIdx);

	if (utilities::distanceEucl(c.position, p.position) < conf_.maxDistanceCamFeature) {
		updateSteinerGridTargetBounds(c.position.x(), c.position.y(), c.position.z());
		updateSteinerGridTargetBounds(p.position.x(), p.position.y(), p.position.z());
	}
}

bool ManifoldMeshReconstructor::hasVisibilityPair(int camIdx, int pointIdx) {
	for (auto i : cams_[camIdx].visiblePoints)
		if (i == pointIdx) return true;
	return false;
}

RayPath* ManifoldMeshReconstructor::addRayPath(int cameraId, int pointId) {
	RayPath* r = new RayPath();

	r->cameraId = cameraId;
	r->pointId = pointId;

	const std::pair<int, int> k = std::pair<int, int>(cameraId, pointId);
	rayPaths_.insert(std::pair<const std::pair<int, int>, RayPath*>(k, r));

	camerasRayPaths_[cameraId].insert(r);
	pointsRayPaths_[pointId].insert(r);

	return r;
}
RayPath* ManifoldMeshReconstructor::getRayPath(int cameraId, int pointId) {
	if (!rayPaths_.count(pair<int, int>(cameraId, pointId))) {
		return addRayPath(cameraId, pointId);
	}

	const pair<int, int> k = pair<int, int>(cameraId, pointId);
	return rayPaths_.at(k);
}
std::set<RayPath*> ManifoldMeshReconstructor::getRayPathsFromCamera(int cameraId) {
	return camerasRayPaths_.at(cameraId);
}
std::set<RayPath*> ManifoldMeshReconstructor::getRayPathsFromPoint(int pointId) {
	return pointsRayPaths_.at(pointId);
}

void ManifoldMeshReconstructor::getDegree1Neighbours(std::set<Delaunay3::Cell_handle>& path, std::set<Delaunay3::Cell_handle>& d1Neighbours) {

	for (auto cell : path) {
		if (!dt_.is_cell(cell)) {
			cerr << "dead cell found in ray path" << endl;
			continue;
		}
		for (int facetIndex = 0; facetIndex < 4; ++facetIndex) {
			Delaunay3::Cell_handle nearCell = cell->neighbor(facetIndex);
			if (!path.count(nearCell)) d1Neighbours.insert(nearCell);
		}
	}
}
void ManifoldMeshReconstructor::getDegree2Neighbours(
		std::set<Delaunay3::Cell_handle>& path, std::set<Delaunay3::Cell_handle>& d1Neighbours, std::set<Delaunay3::Cell_handle>& d2Neighbours) {

	for (auto cell : d1Neighbours) {
		if (!dt_.is_cell(cell)) {
			cerr << "dead cell found in ray path" << endl;
			continue;
		}
		for (int facetIndex = 0; facetIndex < 4; ++facetIndex) {
			Delaunay3::Cell_handle nearCell = cell->neighbor(facetIndex);
			if (!path.count(nearCell) && !d1Neighbours.count(nearCell)) d2Neighbours.insert(nearCell);
		}
	}
}

void ManifoldMeshReconstructor::addRay(int cameraId, int pointId) {
	RayReconstruction* r = new RayReconstruction();

	r->cameraId = cameraId;
	r->pointId = pointId;

	const std::pair<int, int> k = std::pair<int, int>(cameraId, pointId);
	rays_.insert(std::pair<std::pair<int, int>, RayReconstruction*>(k, r));

	camerasRays_[cameraId].insert(r);
	pointsRays_[pointId].insert(r);

}
RayReconstruction* ManifoldMeshReconstructor::getRay(int cameraId, int pointId) {
	const std::pair<int, int> k = std::pair<int, int>(cameraId, pointId);
	return rays_.at(k);
}
std::set<RayReconstruction*> ManifoldMeshReconstructor::getRaysFromCamera(int cameraId) {
	return camerasRays_.at(cameraId);
}
std::set<RayReconstruction*> ManifoldMeshReconstructor::getRaysFromPoint(int pointId) {
	return pointsRays_.at(pointId);
}

void ManifoldMeshReconstructor::updateTriangulation() {

	timeStatsFile_ << endl << cams_.size() << ", ";

//	cout << "ManifoldMeshReconstructor::updateTriangulation: " << endl;
//	for(auto kv : manifoldManager_->getBoundaryCellsSpatialMap()) cout << "\t\t\t (" << kv.first.i << ", " << kv.first.j << ", " << kv.first.k << ")\t→\t" << kv.second.size() << " cells" << endl;

	if (dt_.number_of_vertices() == 0) {
		logger_.startEvent();
//		createSteinerPointGridAndBound();
		initSteinerPointGridAndBound();
		updateSteinerPointGridAndBound();
		logger_.endEventAndPrint("├ initSteinerGrid\t\t", true);
	} else {
		logger_.startEvent();
		updateSteinerPointGridAndBound();
		logger_.endEventAndPrint("├ updateSteinerGrid\t\t", true);
	}
	timeStatsFile_ << logger_.getLastDelta() << ", ";

	/* DEBUG BEGIN */

	std::set<PointD3> enclosingVolumePoints;
	if (conf_.update_points_position) for (auto pIndex : pointsMovedIdx_) {
		//		shrinkPoints.insert(points_[pIndex].position);
		enclosingVolumePoints.insert(points_[pIndex].newPosition);
	}
	for (auto cIndex : updatedCamerasIdx_)
		for (auto pIndex : cams_[cIndex].newVisiblePoints)
			if (points_[pIndex].new_ && utilities::distanceEucl(points_[pIndex].position, cams_[cIndex].position) < conf_.maxDistanceCamFeature)
				enclosingVolumePoints.insert(points_[pIndex].position);

	// This is used to cache the enclosing information in the cells, incrementing it invalidates the cached values and need to be done when the points on which the enclosing volume is base are changed
	currentEnclosingVersion_++;

	// Useful when updating points, maybe...
//	int countRepeatedShrinkPoints = 0;
//	if(lastShrinkPoints_.size()){
//		for( auto p : lastShrinkPoints_) if(shrinkPoints.count(p)) countRepeatedShrinkPoints++;
//		cout << "countRepeatedShrinkPoints:\t" << countRepeatedShrinkPoints << endl;
//	}
//	lastShrinkPoints_.clear();
//	lastShrinkPoints_.insert(shrinkPoints.begin(), shrinkPoints.end());

//	int countInBoundaryPoints = 0;
//	std::set<Delaunay3::Cell_handle> enclosingSet;
//	for (auto p : shrinkPoints) {
//		int li, lj;
//		Delaunay3::Locate_type lt;
//		Delaunay3::Cell_handle c = dt_.locate(p, lt, li, lj);
//		Delaunay3::Facet f;
//		std::vector<Delaunay3::Cell_handle> enclosingVector;
//		dt_.find_conflicts(p, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(enclosingVector));
//		for (auto cell : enclosingVector) {
//			if (cell->info().isBoundary()) {
//				countInBoundaryPoints++;
//				break;
//			}
//		}
//	}
//
//	cout << "Boundary destroing points:\t" << countInBoundaryPoints << " / " << shrinkPoints.size() << endl;

	timerShrinkTime_ = 0.0;
	timerShrinkSeveralTime_ = 0.0;
	if (enclosingVolumePoints.size()) {
		logger_.startEvent();
		shrinkManifold3(enclosingVolumePoints);
		logger_.endEventAndPrint("├ shrinkManifold\t\t", true);
		timeStatsFile_ << logger_.getLastDelta() << ", ";
	} else {
		logger_.startEvent();
		cout << "├ shrinkManifold\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	timeStatsFile_ << timerShrinkTime_ << ", " << timerShrinkSeveralTime_ << ", ";
	/* DEBUG END */

	/*
	 timerShrinkTime_ = 0.0;
	 timerShrinkSeveralTime_ = 0.0;
	 if (updatedCamerasIdx_.size()) {
	 logger_.startEvent();
	 for (auto updatedCameraIndex : updatedCamerasIdx_) { // TODO shrink for all cameras and then insert all point?
	 shrinkManifold(cams_[updatedCameraIndex].position, updatedCameraIndex);
	 }
	 logger_.endEventAndPrint("├ shrinkManifold\t\t", true);
	 timeStatsFile_ << logger_.getLastDelta() << ", ";
	 } else {
	 logger_.startEvent();
	 cout << "├ shrinkManifold\t\tSkipped" << endl;
	 timeStatsFile_ << 0.0 << ", ";
	 }

	 timeStatsFile_ << timerShrinkTime_ << ", " << timerShrinkSeveralTime_ << ", ";
	 */

	if (updatedCamerasIdx_.size()) {
		logger_.startEvent();
		for (auto updatedCameraIndex : updatedCamerasIdx_) {

			for (auto pIndex : cams_[updatedCameraIndex].newVisiblePoints) {
				PointReconstruction& newPoint = points_[pIndex];

				if (newPoint.new_) {
					// TODO move in insertVertex as in moveVertex
					if (utilities::distanceEucl(newPoint.position, cams_[updatedCameraIndex].position) < conf_.maxDistanceCamFeature) {

						vecDistanceWeight_.clear();

						/*	Try to insert the new point in the triangulation.
						 * 	When successful, a new vertex corresponding to the nwe point is created,
						 * 	some cells are removed from the triangulation and replaced by some others.
						 */
						if (insertVertex(newPoint)) {
							newPoint.vertexHandle->info().setLastCam(updatedCameraIndex);
							newPoint.vertexHandle->info().setFirstCam(updatedCameraIndex);
						}
					}

				} else {
//					if (raysToBeTraced_.count(pair<int, int>(updatedCameraIndex, newPoint.idReconstruction)) == 0) cerr << "raysToBeTraced_ count" << endl;
					raysToBeTraced_.insert(pair<int, int>(updatedCameraIndex, newPoint.idReconstruction)); //TODO useful?

					newPoint.vertexHandle->info().setLastCam(updatedCameraIndex);
					for (auto c : newPoint.viewingCams) {
						if (c <= updatedCameraIndex) newPoint.vertexHandle->info().addCam(updatedCameraIndex);
					}
				}
			}

		}
		updatedCamerasIdx_.clear();
		logger_.endEventAndPrint("├ Add new vertices\t\t", true);
		timeStatsFile_ << logger_.getLastDelta() << ", ";
	} else {
		cout << "├ Add new vertices\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	// Move the vertices for the moved points
	if (conf_.update_points_position && pointsMovedIdx_.size()) {
		logger_.startEvent();
		for (auto id : pointsMovedIdx_) {

			Segment s = Segment(points_[id].position, points_[id].newPosition);

			bool moved;
			moved = moveVertex(id);

			if (conf_.all_sort_of_output) if (moved) movedPointsSegments_.push_back(s);

		}
		pointsMovedIdx_.clear();
		logger_.endEventAndPrint("├ Move vertices\t\t\t", true);
		timeStatsFile_ << logger_.getLastDelta() << ", ";
	} else {
		cout << "├ Move vertices\t\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	if (conf_.all_sort_of_output) outputM_->writeRaysToOFF("output/moved_points/moved_points", std::vector<int> { }, movedPointsSegments_);

	// Update the constraints for the moved cameras
	if (movedCamerasIdx_.size()) {
		logger_.startEvent();
		for (int cameraIndex : movedCamerasIdx_) {
			moveCameraConstraints(cameraIndex);
		}
		movedCamerasIdx_.clear();
		logger_.endEventAndPrint("├ Move Cameras\t\t\t", true);
		timeStatsFile_ << logger_.getLastDelta() << ", ";
	} else {
		cout << "├ Move Cameras\t\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	rayTracingFromAllCam();

	// TODO erase duplicated cells or use a set
	freeSpaceTets_.erase(std::remove_if(freeSpaceTets_.begin(), freeSpaceTets_.end(), [&](Delaunay3::Cell_handle cell) {
		return !dt_.is_cell(cell) || cell->info().getIntersections().size() == 0;
	}), freeSpaceTets_.end());

//	printWhatever();
//	outputM_->writeAllVerticesToOFF("output/triangulation_vertices/all_vertices", std::vector<int> { });
	growManifold3(enclosingVolumePoints);

	iterationCounter_++;
}

void ManifoldMeshReconstructor::insertNewPointsFromCam(int camIdx, bool incremental) {
	;
}

void ManifoldMeshReconstructor::rayTracingFromAllCam() {
//	double vote_sum = 0.0;
//	for (auto cell = dt_.finite_cells_begin(); cell != dt_.finite_cells_end(); cell++) {
//		vote_sum += cell->info().getVoteCountProb();
//		//cout << cell->info().getVoteCountProb() << endl;
//	}
//	cout << "\tvote_sum: " << vote_sum << endl;

	if (raysToBeUntraced_.size()) {
		logger_.startEvent();
		for (auto cIndex_pIndex : raysToBeUntraced_) {
			rayUntracing(getRayPath(cIndex_pIndex.first, cIndex_pIndex.second));
		}
		raysToBeUntraced_.clear();
		logger_.endEventAndPrint("│ ├ rayUntracing\t\t", true);
		timeStatsFile_ << logger_.getLastDelta() << ", ";
	} else {
		cout << "│ ├ rayUntracing\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

//	vote_sum = 0.0;
//	for (auto cell = dt_.finite_cells_begin(); cell != dt_.finite_cells_end(); cell++) {
//		vote_sum += cell->info().getVoteCountProb();
//	}
//	cout << "\tvote_sum: " << vote_sum << endl;

	if (raysToBeTraced_.size()) {
		logger_.startEvent();
		for (auto cIndex_pIndex : raysToBeTraced_) {
			raysToBeRetraced_.erase(cIndex_pIndex);
			rayTracing2(cIndex_pIndex.first, cIndex_pIndex.second, false);
		}
		raysToBeTraced_.clear();
		logger_.endEventAndPrint("│ ├ rayTracing\t\t\t", true);
		timeStatsFile_ << logger_.getLastDelta() << ", ";
	} else {
		cout << "│ ├ rayTracing\t\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
//
//	vote_sum = 0.0;
//	for (auto cell = dt_.finite_cells_begin(); cell != dt_.finite_cells_end(); cell++) {
//		vote_sum += cell->info().getVoteCountProb();
//	}
//	cout << "\tvote_sum: " << vote_sum << endl;

	if (raysToBeRetraced_.size()) {
		logger_.startEvent();
		for (auto cIndex_pIndex : raysToBeRetraced_) {

			rayRetracing(cIndex_pIndex.first, cIndex_pIndex.second, newCells_);
		}
		raysToBeRetraced_.clear();

		for (auto cell : newCells_) { //TODO erase dead cells from new cells too (when cells are killed)
			if (dt_.is_cell(cell)) {
				cell->info().markOld();
			} else {
				; //cerr << "new cell was found dead :C" << endl;
			}
		}
		newCells_.clear();

		logger_.endEventAndPrint("│ ├ rayRetracing\t\t", true);
		timeStatsFile_ << logger_.getLastDelta() << ", ";
	} else {
		cout << "│ ├ rayRetracing\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

//	vote_sum = 0.0;
//	for (auto cell = dt_.finite_cells_begin(); cell != dt_.finite_cells_end(); cell++) {
//		vote_sum += cell->info().getVoteCountProb();
//	}
//	cout << "\tvote_sum: " << vote_sum << endl;

}

void ManifoldMeshReconstructor::rayTracingFromCam(int idxCam) {
	;
}
void ManifoldMeshReconstructor::rayTracing(int idxCam, int idxPoint, bool bOnlyMarkNew, bool incrementCount) {

	Delaunay3::Cell_handle tetPrev;
	Delaunay3::Cell_handle tetCur;
	PointD3 source = points_[idxPoint].position;
	PointD3 target = cams_[idxCam].position;
	Segment constraint = Segment(source, target);

	RayPath* rayPath = getRayPath(idxCam, idxPoint);

	//RayReconstruction* ray = getRay(idxCam, idxPoint);
	RayReconstruction* ray = NULL;

	if (!hasVisibilityPair(idxCam, idxPoint)) {
		std::cerr << "no visibility pair " << idxCam << ", " << idxPoint << std::endl;
		return;
	}

	/*******************************************************************************************
	 Look for the tetrahedron incident to Q intersected by the ray from camera O to point Q
	 ******************************************************************************************/
	Delaunay3::Locate_type lt;
	int li, lj;
	Vertex3D_handle pointHandle = points_[idxPoint].vertexHandle;
	bool firstExitFacetFound = false;

	//stores in the vector qCells the handle to the cells (=tetrahedra) incident to Q
	std::vector<Delaunay3::Cell_handle> qCells;
	dt_.incident_cells(pointHandle, std::back_inserter(qCells));

	if (qCells.size() == 0) {
		std::cerr << "ManifoldMeshReconstructor::rayTracing: incident cells to initial vertex not found for ray " << idxCam << ", " << idxPoint << std::endl;
	}
	if (pointHandle == NULL) {
		std::cerr << "ManifoldMeshReconstructor::rayTracing: vertexHandle is NULL; ray " << idxCam << ", " << idxPoint << std::endl;
	}

	// For each tetrahedron t, incident to the point p (t s.t. one of its four vertices is p)
	for (auto t : qCells) {

		// If the tetrahedron t contains the camera, the ray ends in t. Mark t and return
		if (dt_.side_of_cell(constraint.target(), t, lt, li, lj) != CGAL::ON_UNBOUNDED_SIDE) {
			if (!bOnlyMarkNew || t->info().isNew() || conf_.enableSuboptimalPolicy) {

				markTetraedron(t, idxCam, idxPoint, rayPath->path, ray, incrementCount);
			}
			return;
		}

		// Let facetIndex be the index (in t) of the facet f opposite to the point Q
		int facetIndex = t->index(pointHandle);
		if (CGAL::do_intersect(dt_.triangle(t, facetIndex), constraint)) {
			firstExitFacetFound = true;

			tetPrev = t;
			tetCur = t->neighbor(facetIndex); // t.actual = neighbour of t incident to facet f
			if (!bOnlyMarkNew || t->info().isNew() || conf_.enableSuboptimalPolicy) {
				markTetraedron(t, idxCam, idxPoint, rayPath->path, ray, incrementCount);
			}
			if (!bOnlyMarkNew || tetCur->info().isNew() || conf_.enableSuboptimalPolicy) {
				markTetraedron(tetCur, idxCam, idxPoint, rayPath->path, ray, incrementCount);
			}
			break;
		}
	}

	if (!firstExitFacetFound) {
		std::cerr << "firstExitFacet NOT FOUND for ray " << idxCam << ", " << idxPoint << "\tqCells.size(): " << qCells.size() << std::endl;
		return;
	}

	qCells.clear();

	/**********************************************
	 Follow the ray through the triangulation
	 *********************************************/
	int f, fOld;
	std::set<Delaunay3::Cell_handle> visitedTetrahedra;

	// While t.actual doesn't contain O
	while (cellTraversalExitTest(f, fOld, tetCur, tetPrev, visitedTetrahedra, constraint)) {

		if (conf_.inverseConicEnabled) {

			// Increment weight of the neighbors not traversed by the ray
			for (int curidFac = 0; curidFac < 4; ++curidFac) {
				if ((curidFac != f) && (curidFac != fOld)) {
					Delaunay3::Cell_handle nearCellNotIntersected = tetCur->neighbor(curidFac);
					if (incrementCount == true) {
						nearCellNotIntersected->info().incrementVoteCountProb(conf_.w_2);

						nearCellNotIntersected->info().addItersectionWeightedW2(ray);
						nearCellNotIntersected->info().setWeights(conf_.w_1, conf_.w_2, conf_.w_3);
					} else {
						nearCellNotIntersected->info().decrementVoteCountProb(conf_.w_2);
					}
					if (!conf_.enableSuboptimalPolicy) {
						if (incrementCount == true) {
							nearCellNotIntersected->info().addIntersection(idxCam, idxPoint, conf_.w_2, points_[idxPoint].idVertex, points_, camsPositions_);
						} else {
							nearCellNotIntersected->info().removeIntersection(idxCam, idxPoint, conf_.w_2, points_, camsPositions_);
						}
					}

					// Increment weight of the neighbors of the neighbors
					for (int curidFacNear = 0; curidFacNear < 4; ++curidFacNear) {
						if (curidFacNear != nearCellNotIntersected->index(tetCur)) {
							if (incrementCount == true) {
								nearCellNotIntersected->neighbor(curidFacNear)->info().incrementVoteCountProb(conf_.w_3);

								nearCellNotIntersected->neighbor(curidFacNear)->info().addItersectionWeightedW3(ray);
								nearCellNotIntersected->neighbor(curidFacNear)->info().setWeights(conf_.w_1, conf_.w_2, conf_.w_3);
							} else {
								nearCellNotIntersected->neighbor(curidFacNear)->info().decrementVoteCountProb(conf_.w_3);
							}
							if (!conf_.enableSuboptimalPolicy) {
								if (incrementCount == true) {
									nearCellNotIntersected->neighbor(curidFacNear)->info().addIntersection(
											idxCam, idxPoint, conf_.w_3, points_[idxPoint].idVertex, points_, camsPositions_);
								} else {
									nearCellNotIntersected->neighbor(curidFacNear)->info().removeIntersection(idxCam, idxPoint, conf_.w_3, points_, camsPositions_);
								}
							}
						}
					}
				}
			}
		}
		tetPrev = tetCur; // t.precedent = t.actual
		tetCur = tetCur->neighbor(f); // t.actual = neighbour of t.precedent(==t.actual) incident to facet f

		if (!bOnlyMarkNew || tetCur->info().isNew() || conf_.enableSuboptimalPolicy) {
			markTetraedron(tetCur, idxCam, idxPoint, rayPath->path, ray, incrementCount);
		}

	}

}

float ManifoldMeshReconstructor::weightFunction(RayPath* r) { // TODO acceptably wrong: position is not the original position when rays are moved
	float d = utilities::distanceEucl(points_[r->pointId].position, cams_[r->cameraId].position);
	float maxD = conf_.maxDistanceCamFeature;

	return 1.0;
//	return sqrt(1 - pow((d/maxD), 2));
}

void ManifoldMeshReconstructor::rayTracing2(int idxCam, int idxPoint, bool bOnlyMarkNew) {
	Delaunay3::Cell_handle tetPrev;
	Delaunay3::Cell_handle tetCur;
	PointD3 source = points_[idxPoint].position;
	PointD3 target = cams_[idxCam].position;
	Segment constraint = Segment(source, target);

	RayPath* rayPath = getRayPath(idxCam, idxPoint);

	//RayReconstruction* ray = getRay(idxCam, idxPoint);
	RayReconstruction* ray = NULL;

	if (!hasVisibilityPair(idxCam, idxPoint)) {
		std::cerr << "no visibility pair " << idxCam << ", " << idxPoint << std::endl;
		return;
	}

	/*******************************************************************************************
	 Look for the tetrahedron incident to Q intersected by the ray from camera O to point Q
	 ******************************************************************************************/
	Delaunay3::Locate_type lt;
	int li, lj;
	Vertex3D_handle vertexHandle = points_[idxPoint].vertexHandle;
	bool firstExitFacetFound = false;

	//stores in the vector qCells the handle to the cells (=tetrahedra) incident to Q
	std::vector<Delaunay3::Cell_handle> incidentCells;
	dt_.incident_cells(vertexHandle, std::back_inserter(incidentCells)); // TODO CGAL ERROR: precondition violation! infinite cell?

	if (incidentCells.size() == 0) {
		std::cerr << "ManifoldMeshReconstructor::rayTracing: incident cells to initial vertex not found for ray " << idxCam << ", " << idxPoint << std::endl;
	}
	if (vertexHandle == NULL) {
		std::cerr << "ManifoldMeshReconstructor::rayTracing: vertexHandle is NULL; ray " << idxCam << ", " << idxPoint << std::endl;
	}

	// For each tetrahedron t, incident to the point p (t s.t. one of its four vertices is p)
	for (auto t : incidentCells) {

		// If the tetrahedron t contains the camera, the ray ends in t. Mark t and return
		if (dt_.side_of_cell(constraint.target(), t, lt, li, lj) != CGAL::ON_UNBOUNDED_SIDE) {
			if (!bOnlyMarkNew || t->info().isNew() || conf_.enableSuboptimalPolicy) {

				markTetraedron(t, idxCam, idxPoint, rayPath->path, ray, true);
			}
			return;
		}

		// Let facetIndex be the index (in t) of the facet f opposite to the point Q
		int facetIndex = t->index(vertexHandle);
		if (CGAL::do_intersect(dt_.triangle(t, facetIndex), constraint)) {
			firstExitFacetFound = true;

			tetPrev = t;
			tetCur = t->neighbor(facetIndex); // t.actual = neighbour of t incident to facet f
			if (!bOnlyMarkNew || t->info().isNew() || conf_.enableSuboptimalPolicy) {
				markTetraedron(t, idxCam, idxPoint, rayPath->path, ray, true);
			}
			if (!bOnlyMarkNew || tetCur->info().isNew() || conf_.enableSuboptimalPolicy) {
				markTetraedron(tetCur, idxCam, idxPoint, rayPath->path, ray, true);
			}
			break;
		}
	}

	if (!firstExitFacetFound) {
		std::cerr << "firstExitFacet NOT FOUND for ray " << idxCam << ", " << idxPoint << "\tincidentCells.size(): " << incidentCells.size() << std::endl;
		return;
	}

	incidentCells.clear();

	/**********************************************
	 Follow the ray through the triangulation
	 *********************************************/
	int f, fOld;
	std::set<Delaunay3::Cell_handle> visitedTetrahedra;

	// While t.actual doesn't contain O
	while (cellTraversalExitTest(f, fOld, tetCur, tetPrev, visitedTetrahedra, constraint)) {
		tetPrev = tetCur;
		tetCur = tetCur->neighbor(f);

		if (!bOnlyMarkNew || tetCur->info().isNew() || conf_.enableSuboptimalPolicy) {
			markTetraedron(tetCur, idxCam, idxPoint, rayPath->path, ray, true);
		}
	}

	if (conf_.inverseConicEnabled) {
		// set of degree-1 and degree-2 neighbours for the path
		std::set<Delaunay3::Cell_handle> d1Neighbours;
		std::set<Delaunay3::Cell_handle> d2Neighbours;
		getDegree1Neighbours(rayPath->path, d1Neighbours);
		getDegree2Neighbours(rayPath->path, d1Neighbours, d2Neighbours);

		// Increment weight of the neighbors
		for (auto d1Neighbour : d1Neighbours) {
			d1Neighbour->info().incrementVoteCountProb(conf_.w_2 * weightFunction(rayPath));
			if (!conf_.enableSuboptimalPolicy)
				d1Neighbour->info().addIntersection(idxCam, idxPoint, conf_.w_2 * weightFunction(rayPath), points_[idxPoint].idVertex, points_, camsPositions_);
//			d1Neighbour->info().addItersectionWeightedW2(ray);
//			d1Neighbour->info().setWeights(conf_.w_1, conf_.w_2, conf_.w_3);
		}

		// Increment weight of the neighbors of the neighbors
		for (auto d2Neighbour : d2Neighbours) {
			d2Neighbour->info().incrementVoteCountProb(conf_.w_3 * weightFunction(rayPath));
			if (!conf_.enableSuboptimalPolicy)
				d2Neighbour->info().addIntersection(idxCam, idxPoint, conf_.w_3 * weightFunction(rayPath), points_[idxPoint].idVertex, points_, camsPositions_);
//			d2Neighbour->info().addItersectionWeightedW3(ray);
//			d2Neighbour->info().setWeights(conf_.w_1, conf_.w_2, conf_.w_3);
		}
	}
}

void ManifoldMeshReconstructor::rayUntracing(RayPath* rayPath) {
	int idxCam = rayPath->cameraId;
	int idxPoint = rayPath->pointId;

	RayReconstruction* ray = NULL;
//	RayReconstruction* ray = getRay(idxCam, idxPoint);
//	ray->valid = false;

	// for all the cells in the ray's path, do the opposite of rayTracing
	for (auto cell : rayPath->path) {

		// The path is littered with dead cells, just skip them (the path is going to be cleared anyway)
		if (!dt_.is_cell(cell)) {
			cerr << "dead cell found on ray path " << idxCam << ", " << idxPoint << endl;
			continue;
		}

		cell->info().decrementVoteCount(1);
		cell->info().decrementVoteCountProb(conf_.w_1 * weightFunction(rayPath));
		if (!conf_.enableSuboptimalPolicy) cell->info().removeIntersection(idxCam, idxPoint, points_, camsPositions_);

	}

	//cout << rayPath->path.size() << endl;

	if (conf_.inverseConicEnabled) {
		// set of degree-1 and degree-2 neighbours for the path
		std::set<Delaunay3::Cell_handle> d1Neighbours;
		std::set<Delaunay3::Cell_handle> d2Neighbours;
		getDegree1Neighbours(rayPath->path, d1Neighbours);
		getDegree2Neighbours(rayPath->path, d1Neighbours, d2Neighbours);

		// Increment weight of the neighbors
		for (auto d1Neighbour : d1Neighbours) {
			d1Neighbour->info().decrementVoteCountProb(conf_.w_2 * weightFunction(rayPath));
			if (!conf_.enableSuboptimalPolicy) d1Neighbour->info().removeIntersection(idxCam, idxPoint, conf_.w_2 * weightFunction(rayPath), points_, camsPositions_);
		}

		// Increment weight of the neighbors of the neighbors
		for (auto d2Neighbour : d2Neighbours) {
			d2Neighbour->info().decrementVoteCountProb(conf_.w_3 * weightFunction(rayPath));
			if (!conf_.enableSuboptimalPolicy) d2Neighbour->info().removeIntersection(idxCam, idxPoint, conf_.w_3 * weightFunction(rayPath), points_, camsPositions_);
		}
	}

	// Remove all cells from the path (rayTracing will add them back)
	rayPath->path.clear();
}

void ManifoldMeshReconstructor::rayRetracing(int idxCam, int idxPoint, std::set<Delaunay3::Cell_handle>& newCells) {
	// TODO
	rayTracing2(idxCam, idxPoint, true);

}

bool ManifoldMeshReconstructor::cellTraversalExitTest(
		int & exitFacet, int & entryFacet, const Delaunay3::Cell_handle& tetCur, const Delaunay3::Cell_handle& tetPrev, std::set<Delaunay3::Cell_handle>& visitedTetrahedra,
		const Segment& constraint) {

	// For tetCur, find the facet index of the facet adjacent to tetPrev (the entry facet's index in the current tetrahedron)
	for (int facetIndex = 0; facetIndex < 4; ++facetIndex) {
		if (tetCur->neighbor(facetIndex) == tetPrev) entryFacet = facetIndex;
	}

	// From the current tetrahedron, find the facets that intersects with the ray, excluding the entry facet
	bool candidateFacetFound = false;
	for (int facetIndex = 0; facetIndex < 4; facetIndex++) {
		if (facetIndex != entryFacet && CGAL::do_intersect(dt_.triangle(tetCur, facetIndex), constraint)) {

			/** WORKAROUND
			 * 	If a candidate facet was already found then the next tetrahedron to be visited is not uniqe.
			 * 	This means that the sequence of visited tetrahedra could enter a loop.
			 * 	Workaround: end rayTracing
			 *
			 * 	Possible solution to the bug: use visitedTetrahedra to select the next tetrahedron in order to visit all tetrahedra that intersects with the ray
			 */
			if (candidateFacetFound) {
//				std::cerr << "ManifoldMeshReconstructor::cellTraversalExitTest: rayTracing stopped prematurely to avoid a possible infinite loop" << std::endl;
				return false;
			}

			exitFacet = facetIndex;
			candidateFacetFound = true;
		}
	}

	/** WORKAROUND (redundant)
	 * 	The sequence of visited tetrahedra entered a (probably infinite) loop.
	 * 	Workaround: end rayTracing
	 *
	 * 	Possible solution to the bug: use visitedTetrahedra to select the next tetrahedron in order to visit all tetrahedra that intersects with the ray
	 */
//	if(visitedTetrahedra.count(tetCur)){
//		std::cerr << "ManifoldMeshReconstructor::cellTraversalExitTest: infinite loop detected. rayTracing stopped prematurely" << std::endl;
//		return false;
//	}
//	visitedTetrahedra.insert(tetPrev);
//	visitedTetrahedra.insert(tetCur);
	if (candidateFacetFound) return true;
	else return false;
}

void ManifoldMeshReconstructor::markTetraedron(
		Delaunay3::Cell_handle & cell, const int camIndex, const int featureIndex, std::set<Delaunay3::Cell_handle>& path, RayReconstruction* ray, bool incrementCount) {

	if (incrementCount) {
		path.insert(cell);

		cell->info().incrementVoteCount(1);
		cell->info().incrementVoteCountProb(conf_.w_1 * weightFunction(getRayPath(camIndex, featureIndex)));

//		cell->info().incrementVoteCount(1.0); // why it's incremented twice??
//		cell->info().addItersectionWeightedW1(ray);
//		cell->info().setWeights(conf_.w_1, conf_.w_2, conf_.w_3);

		if (!conf_.enableSuboptimalPolicy) {
			cell->info().addIntersection(
					camIndex, featureIndex, conf_.w_1 * weightFunction(getRayPath(camIndex, featureIndex)), points_[featureIndex].idVertex, points_, camsPositions_);
		}

		freeSpaceTets_.push_back(cell);

	} else {
		cell->info().decrementVoteCount(1);
		cell->info().decrementVoteCountProb(conf_.w_1 * weightFunction(getRayPath(camIndex, featureIndex)));
//		ray->valid = false;

		if (!conf_.enableSuboptimalPolicy) {
			cell->info().removeIntersection(camIndex, featureIndex, points_, camsPositions_);
		}
	}
}

void ManifoldMeshReconstructor::growManifold3(std::set<PointD3> points) {
	if (!freeSpaceTets_.size()) {
		cerr << "freeSpaceTets_ is empty; Can't grow" << endl;
		return;
	}

	if (conf_.all_sort_of_output) saveBoundary(1, 0);

	// TODO move the initialisation to updateTriangulation
	logger_.startEvent();
	if (manifoldManager_->getBoundarySize() == 0) {
		// If the boundary is still empty, initialise all the cells' informations and start growing from the cell with highest vote
		std::sort(freeSpaceTets_.begin(), freeSpaceTets_.end(), sortTetByIntersection());
		for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
			itCell->info().setKeptManifold(false);
			for (int curV = 0; curV < 4; ++curV) {
				itCell->vertex(curV)->info().setUsed(0);
				itCell->vertex(curV)->info().setNotUsed(true);
			}
		}
		Delaunay3::Cell_handle startingCell = freeSpaceTets_[freeSpaceTets_.size() - 1];
		manifoldManager_->regionGrowingBatch3(startingCell, points);
	} else {
		manifoldManager_->regionGrowing3(points);
	}
	logger_.endEventAndPrint("│ ├ growManifold\t\t", true);
	timeStatsFile_ << logger_.getLastDelta() << ", ";

	if (conf_.all_sort_of_output) saveBoundary(1, 1);

	logger_.startEvent();
	manifoldManager_->growSeveralAtOnce3(points);
	logger_.endEventAndPrint("│ ├ growManifoldSev\t\t", true);
	timeStatsFile_ << logger_.getLastDelta() << ", ";

	if (conf_.all_sort_of_output) saveBoundary(1, 2);

	logger_.startEvent();
	manifoldManager_->regionGrowing3(points);
	logger_.endEventAndPrint("│ ├ growManifold\t\t", true);
	timeStatsFile_ << logger_.getLastDelta() << ", ";

	if (conf_.all_sort_of_output) saveBoundary(1, 3);
}

void ManifoldMeshReconstructor::growManifold() {
	if (manifoldManager_->getBoundarySize() == 0) {
		std::sort(freeSpaceTets_.begin(), freeSpaceTets_.end(), sortTetByIntersection());
		for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
			itCell->info().setKeptManifold(false);
			for (int curV = 0; curV < 4; ++curV) {
				itCell->vertex(curV)->info().setUsed(0);
				itCell->vertex(curV)->info().setNotUsed(true);
			}
		}
		if (freeSpaceTets_.size()) {
			Delaunay3::Cell_handle startingCell = freeSpaceTets_[freeSpaceTets_.size() - 1];
			manifoldManager_->regionGrowingBatch(startingCell);
		} else {
			cerr << "freeSpaceTets_ is empty; Can't grow" << endl;

		}
	} else {
		manifoldManager_->regionGrowing();
	}

}

void ManifoldMeshReconstructor::growManifold(int camIdx) {
	if (manifoldManager_->getBoundarySize() == 0) {
		std::sort(freeSpaceTets_.begin(), freeSpaceTets_.end(), sortTetByIntersection());
		for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
			itCell->info().setKeptManifold(false);
			for (int curV = 0; curV < 4; ++curV) {
				itCell->vertex(curV)->info().setUsed(0);
				itCell->vertex(curV)->info().setNotUsed(true);
			}

		}

		Delaunay3::Cell_handle startingCell = dt_.locate(cams_[camIdx].position);

		manifoldManager_->regionGrowingBatch(startingCell);
	} else {
		manifoldManager_->regionGrowing();
	}

}

void ManifoldMeshReconstructor::growManifoldSev() {
	manifoldManager_->growSeveralAtOnce2();
}

void ManifoldMeshReconstructor::saveManifold(const std::string filename) {
	outputM_->writeOFF(filename);

	if (conf_.all_sort_of_output) {

		std::vector<Segment> rays;
		for (auto rayPath : rayPaths_) {

			PointD3 source = points_[rayPath.second->pointId].position;
			PointD3 target = cams_[rayPath.second->cameraId].position;
			Segment constraint = Segment(source, target);

			rays.push_back(constraint);
		}
		outputM_->writeRaysToOFF("output/all_rays/rays", std::vector<int> { iterationCounter_ }, rays);
	}
}

void ManifoldMeshReconstructor::saveBoundary(int i, int j) {

//	outputM_->writeBoundaryOFF(filename, manifoldManager_->getBoundaryCells());

	std::vector<Delaunay3::Cell_handle> b = manifoldManager_->getBoundaryCells();
	if (b.size()) outputM_->writeTetrahedraToOFF("output/boundary/boundary", std::vector<int> { iterationCounter_, i, j }, b);
}

void ManifoldMeshReconstructor::saveOldManifold(const std::string filename, int idx) {
	outputM_->writeOFF(filename, idx);
}

void ManifoldMeshReconstructor::saveOldManifold(const std::string filename, std::vector<int> idx) {

	outputM_->writeOFF(filename, idx);
}

void ManifoldMeshReconstructor::saveFreespace(const std::string filename) {
	outputM_->writeFreespaceOFF(filename);
}

void ManifoldMeshReconstructor::shrinkManifold3(std::set<PointD3> points) {

	if (conf_.all_sort_of_output) saveBoundary(0, 0);

	logger_.startEvent();
	manifoldManager_->shrinkManifold3(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 1);

	logger_.startEvent();
	manifoldManager_->shrinkSeveralAtOnce3(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrinkSeveral\t\t", true);
	timerShrinkSeveralTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 2);

	logger_.startEvent();
	manifoldManager_->shrinkManifold3(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 3);

	logger_.startEvent();
	manifoldManager_->shrinkSeveralAtOnce3(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrinkSeveral\t\t", true);
	timerShrinkSeveralTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 4);

	logger_.startEvent();
	manifoldManager_->shrinkManifold3(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 5);

}

void ManifoldMeshReconstructor::shrinkManifold2(std::set<PointD3> points) {

	if (conf_.all_sort_of_output) saveBoundary(0, 0);

	logger_.startEvent();
	manifoldManager_->shrinkManifold2(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 1);

	logger_.startEvent();
	manifoldManager_->shrinkSeveralAtOnce2(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrinkSeveral\t\t", true);
	timerShrinkSeveralTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 2);

	logger_.startEvent();
	manifoldManager_->shrinkManifold2(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 3);

	logger_.startEvent();
	manifoldManager_->shrinkSeveralAtOnce2(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrinkSeveral\t\t", true);
	timerShrinkSeveralTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 4);

	logger_.startEvent();
	manifoldManager_->shrinkManifold2(points, l_, currentEnclosingVersion_);
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(0, 5);

	currentEnclosingVersion_++;
}

void ManifoldMeshReconstructor::shrinkManifold(const PointD3 &camCenter, int updatedCameraIndex) {
	if (conf_.all_sort_of_output) saveBoundary(updatedCameraIndex, 0);

	logger_.startEvent();
	manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);
//  saveManifold("tempa2.off");
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(updatedCameraIndex, 1);

	logger_.startEvent();
	manifoldManager_->shrinkSeveralAtOnce(camCenter, l_, conf_.maxDistanceCamFeature);
//  saveManifold("tempb2.off");
	logger_.endEventAndPrint("│ ├ shrinkSeveral\t\t", true);
	timerShrinkSeveralTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(updatedCameraIndex, 2);

	logger_.startEvent();
	manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);
//  saveManifold("tempc2.off");
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(updatedCameraIndex, 3);

	logger_.startEvent();
	manifoldManager_->shrinkSeveralAtOnce(camCenter, l_, conf_.maxDistanceCamFeature);
//  saveManifold("tempb2.off");
	logger_.endEventAndPrint("│ ├ shrinkSeveral\t\t", true);
	timerShrinkSeveralTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(updatedCameraIndex, 4);

	logger_.startEvent();
	manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);
	logger_.endEventAndPrint("│ ├ shrink\t\t\t", true);
	timerShrinkTime_ += logger_.getLastDelta();

	if (conf_.all_sort_of_output) saveBoundary(updatedCameraIndex, 5);

}

void ManifoldMeshReconstructor::initSteinerPointGridAndBound() {
	std::vector<PointD3> newPoints;

	for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
		for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(PointD3(x, y, z));

	dt_.insert(newPoints.begin(), newPoints.end());
}

void ManifoldMeshReconstructor::updateSteinerPointGridAndBound() {
	std::vector<PointD3> newPoints;

	// Prolong the grid on the positive x semi axis
	while (sgCurrentMaxX_ < sgMaxX_ + stepX_) {
		float x = sgCurrentMaxX_ + stepX_;
		for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(PointD3(x, y, z));
		sgCurrentMaxX_ += stepX_;
	}

	// Prolong the grid on the negative x semi axis
	while (sgCurrentMinX_ > sgMinX_ - stepX_) {
		float x = sgCurrentMinX_ - stepX_;
		for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(PointD3(x, y, z));
		sgCurrentMinX_ -= stepX_;
	}

	// Prolong the grid on the positive y semi axis
	while (sgCurrentMaxY_ < sgMaxY_ + stepY_) {
		float y = sgCurrentMaxY_ + stepY_;
		for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(PointD3(x, y, z));
		sgCurrentMaxY_ += stepY_;
	}

	// Prolong the grid on the negative y semi axis
	while (sgCurrentMinY_ > sgMinY_ - stepY_) {
		float y = sgCurrentMinY_ - stepY_;
		for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(PointD3(x, y, z));
		sgCurrentMinY_ -= stepY_;
	}

	// Prolong the grid on the positive z semi axis
	while (sgCurrentMaxZ_ < sgMaxZ_ + stepZ_) {
		float z = sgCurrentMaxZ_ + stepZ_;
		for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
			for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
				newPoints.push_back(PointD3(x, y, z));
		sgCurrentMaxZ_ += stepZ_;
	}

	// Prolong the grid on the negative z semi axis
	while (sgCurrentMinZ_ > sgMinZ_ - stepZ_) {
		float z = sgCurrentMinZ_ - stepZ_;
		for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
			for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
				newPoints.push_back(PointD3(x, y, z));
		sgCurrentMinZ_ -= stepZ_;
	}

	dt_.insert(newPoints.begin(), newPoints.end());

//	cout << "\tManifoldMeshReconstructor::updateSteinerPointGridAndBound: New bounds: " << endl << "\t\t" << sgCurrentMinX_ << "\t←\tx\t→\t" << sgCurrentMaxX_ << endl << "\t\t" << sgCurrentMinY_ << "\t←\ty\t→\t" << sgCurrentMaxY_ << endl << "\t\t" << sgCurrentMinZ_ << "\t←\tz\t→\t" << sgCurrentMaxZ_ << endl;
}

void ManifoldMeshReconstructor::updateSteinerGridTargetBounds(float x, float y, float z) {
	if (sgMinX_ > x) sgMinX_ = x;
	if (sgMaxX_ < x) sgMaxX_ = x;

	if (sgMinY_ > y) sgMinY_ = y;
	if (sgMaxY_ < y) sgMaxY_ = y;

	if (sgMinZ_ > z) sgMinZ_ = z;
	if (sgMaxZ_ < z) sgMaxZ_ = z;
}

void ManifoldMeshReconstructor::createSteinerPointGridAndBound() {
	std::vector<PointD3> vecPoint;

	stepX_ = stepY_ = stepZ_ = conf_.steinerGridStepLength;

	float inX = -conf_.steinerGridSideLength / 2;
	float finX = conf_.steinerGridSideLength / 2;

	float inY = -conf_.steinerGridSideLength / 2;
	float finY = conf_.steinerGridSideLength / 2;

	float inZ = -conf_.steinerGridSideLength / 2;
	float finZ = conf_.steinerGridSideLength / 2;

//	stepX_ = 40.000;
//	stepY_ = 40.000;
//	stepZ_ = 40.000;
//
//	float inX = -400;
//	float finX = 400;
//	float inY = -400;
//	float finY = 400;
//	float inZ = -400;
//	float finZ = 400;

//  float inX = -300;
//  float finX = 300;
//  float inY = -300;
//  float finY = 300;
//  float inZ = -300;
//  float finZ = 300;

	float x = inX;
	do {
		float y = inY;
		do {
			float z = inZ;
			do {
				vecPoint.push_back(PointD3(x, y, z));
				z = z + stepZ_;
			} while (z <= finZ);
			y = y + stepY_;
		} while (y <= finY);
		x = x + stepX_;
	} while (x <= finX);

	dt_.insert(vecPoint.begin(), vecPoint.end());

	l_ = sqrt(stepX_ * stepX_ + stepY_ * stepY_ + stepZ_ * stepZ_);
}

bool ManifoldMeshReconstructor::insertVertex(PointReconstruction& point) {

	// If the point is marked to be moved but wasn't inserted, use the new position
	if (point.new_ && point.toBeMoved) {
		point.toBeMoved = false;
		point.position = point.newPosition;
	}

	// Locate the new point
	Delaunay3::Locate_type lt;
	int li, lj;
	Delaunay3::Cell_handle c = dt_.locate(point.position, lt, li, lj);

	if (!point.new_) {
		std::cerr << "ManifoldMeshReconstructor::insertNewPoint: point is not marked new!" << std::endl;
	}

	// If there is already a vertex in the new point's position, do not insert it
	if (lt == Delaunay3::VERTEX) {
		return false;
	}

	std::set<pair<int, int>> raysToBeRetraced;

	// Insert in removedCells the cells that conflict with the new point Q, and a facet on the boundary of this hole in f.
	// These cells will be erased by insert_in_hole
	std::vector<Delaunay3::Cell_handle> removedCells;
	Delaunay3::Facet f;
	dt_.find_conflicts(point.position, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(removedCells));

	for (auto cell : removedCells)
		if (cell->info().isBoundary()) cerr << "ManifoldMeshReconstructor::insertNewPoint: destroing boundary cells" << std::endl;

	if (!conf_.enableSuboptimalPolicy) {
		for (auto removedCell : removedCells) {
			for (auto constraint : removedCell->info().getIntersections())
				raysToBeRetraced.insert(pair<int, int>(constraint.first, constraint.second));
		}
	} else {
		for (auto removedCell : removedCells) {
			PointD3 temp = CGAL::barycenter(
					removedCell->vertex(0)->point(), 1.0, removedCell->vertex(1)->point(), 1.0, removedCell->vertex(2)->point(), 1.0, removedCell->vertex(3)->point(), 1.0);
			vecDistanceWeight_.push_back(DistanceWeight(temp, (float) removedCell->info().getVoteCountProb()));
		}
	}

	// Schedule a reyRetracing for all the rays that intersected the removed cells
	for (auto ray : raysToBeRetraced) {
		raysToBeRetraced_.insert(pair<int, int>(ray.first, ray.second));
		//rayRetracing(ray.first, ray.second, newCellsFromHole);

		// Remove dead cells from paths
		for (auto removedCell : removedCells) {
			getRayPath(ray.first, ray.second)->path.erase(removedCell);
		}
	}
//
//	//TODO remove
//	cout << "(pre ) " << endl;
//	printWhatever();

	// Creates a new vertex by starring a hole. Delete all the cells describing the hole vecConflictCells, creates a new vertex hndlQ, and for each facet on the boundary of the hole f, creates a new cell with hndlQ as vertex.
	Vertex3D_handle hndlQ = dt_.insert_in_hole(point.position, removedCells.begin(), removedCells.end(), f.first, f.second);
//
//	//TODO remove
//	cout << "(post) " << endl;
//	printWhatever();

	// Set of the cells that were created to fill the hole, used by rayRetracing to restore the rays
	std::vector<Delaunay3::Cell_handle> newCellsFromHole;
	dt_.incident_cells(hndlQ, std::inserter(newCellsFromHole, newCellsFromHole.begin()));
	newCells_.insert(newCellsFromHole.begin(), newCellsFromHole.end());

	// Add the new vertex handle hndlQ to vecVertexHandles
	vecVertexHandles_.push_back(hndlQ);
	point.idVertex = vecVertexHandles_.size() - 1;
	point.vertexHandle = hndlQ;
	point.new_ = false;

	if (!conf_.enableSuboptimalPolicy) {
		// Schedule rayTracing for all rays between the point and the cameras viewing it
		for (auto cameraIndex : point.viewingCams)
			raysToBeTraced_.insert(pair<int, int>(cameraIndex, point.idReconstruction));
	}

	if (conf_.enableSuboptimalPolicy) { // why's this?
		updateDistanceAndWeights(newCellsFromHole, vecDistanceWeight_);
	}

	return true;

}

void ManifoldMeshReconstructor::updateDistanceAndWeights(std::vector<Delaunay3::Cell_handle> &cellsToBeUpdated, const std::vector<DistanceWeight> &vecDistanceWeight) {
	for (auto itNewCell : cellsToBeUpdated) {
		PointD3 curCenter = CGAL::barycenter(
				itNewCell->vertex(0)->point(), 1.0, itNewCell->vertex(1)->point(), 1.0, itNewCell->vertex(2)->point(), 1.0, itNewCell->vertex(3)->point(), 1.0);
		float voteCount = 0;

		if (conf_.suboptimalMethod == 0) {
			//mindist policy
			float minDist = 10000000000000.0;
			for (auto itWeight : vecDistanceWeight) {
				float dist = utilities::distanceEucl(curCenter.x(), itWeight.first.x(), curCenter.y(), itWeight.first.y(), curCenter.z(), itWeight.first.z());
				if (minDist > dist) {
					minDist = dist;
					voteCount = itWeight.second;
				}
			}
		} else if (conf_.suboptimalMethod == 1) {
			//Mean policy
			for (auto itWeight : vecDistanceWeight) {
				voteCount = voteCount + itWeight.second;
			}
			voteCount = voteCount / vecDistanceWeight.size();

		} else if (conf_.suboptimalMethod == 2) {
			//Weighted mean policy
			float totDist = 0;
			for (auto itWeight : vecDistanceWeight) {
				float dist = utilities::distanceEucl(curCenter.x(), itWeight.first.x(), curCenter.y(), itWeight.first.y(), curCenter.z(), itWeight.first.z());

				voteCount = voteCount + (1 / dist) * itWeight.second;
				totDist = totDist + (1 / dist);
			}
			voteCount = voteCount / totDist;

		} else {
			std::cerr << "updateDistanceAndWeight: you choose a wrong policy num" << std::endl;
		}

		itNewCell->info().incrementVoteCountProb(voteCount); // t.n++
		itNewCell->info().markOld();
	}
}

int ManifoldMeshReconstructor::moveVertex_WHeuristic(int idxPoint, int idxCam) { //TODO use buffered point's position (newPosition, etc), etc

	std::set<Delaunay3::Cell_handle> setNewCells;
	Delaunay3::Vertex_handle hndlQ = points_[idxPoint].vertexHandle;

	PointD3 initialPosition = hndlQ->point();
	PointD3 pd3NewPoint = points_[idxPoint].position;
	PointD3 camPosition = cams_[idxCam].position;

	if (utilities::distanceEucl(pd3NewPoint, camPosition) < conf_.maxDistanceCamFeature) {

		/********************** Step 1: find the cells incident to the vertex to be removed*****************/
		std::vector<Delaunay3::Cell_handle> setIncidentCells;
		dt_.incident_cells(hndlQ, std::back_inserter(setIncidentCells));

		//store their weights
		std::vector<std::pair<PointD3, float> > vecDistanceWeight, vecDistanceWeight2;
		for (auto it : setIncidentCells) {
			PointD3 temp = CGAL::barycenter(it->vertex(0)->point(), 1.0, it->vertex(1)->point(), 1.0, it->vertex(2)->point(), 1.0, it->vertex(3)->point(), 1.0);
			vecDistanceWeight.push_back(std::pair<PointD3, float>(temp, (float) it->info().getVoteCountProb()));
		}

		/**********************Step 2: raytracing to update the weights before the  point removal****************/
		for (int curCam : points_[idxPoint].viewingCams) {
			Segment QO = Segment(hndlQ->point(), cams_[curCam].position);
			rayTracing(curCam, idxPoint, false, false);
		}

		/************Step 3: remove the point and update the new cells weights according to the information on the
		 * weights collected in the previous step****************/
		// stored previously
		std::vector<Delaunay3::Cell_handle> newCells;
		dt_.remove_and_give_new_cells(hndlQ, std::back_inserter(newCells));

		updateDistanceAndWeights(newCells, vecDistanceWeight);

		/***********Step 4: Locate the point and remove conflicting tetrahedra****************/
		Delaunay3::Locate_type lt;
		int li, lj;
		Delaunay3::Cell_handle c = dt_.locate(pd3NewPoint, lt, li, lj);
		if (lt == Delaunay3::VERTEX) {
			std::cerr << "Error in FreespaceDelaunayManifold::moveVertex(): Attempted to move a vertex to an already existing vertex location" << std::endl;
			return false;
		}

		// Get the cells that conflict in a vector vecConflictCells, and a facet on the boundary of this hole in f.
		std::vector<Delaunay3::Cell_handle> vecConflictCells;
		Delaunay3::Facet f;
		dt_.find_conflicts(pd3NewPoint, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

		for (auto it : vecConflictCells) {
			PointD3 temp = CGAL::barycenter(it->vertex(0)->point(), 1.0, it->vertex(1)->point(), 1.0, it->vertex(2)->point(), 1.0, it->vertex(3)->point(), 1.0);
			vecDistanceWeight2.push_back(std::pair<PointD3, float>(temp, (float) it->info().getVoteCountProb()));
		}

		/**********Step 5: Add the new (moved) tets****************/
		hndlQ = dt_.insert_in_hole(pd3NewPoint, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
		points_[idxPoint].vertexHandle = hndlQ;

		/**********Step 6 update the weights of the new tetrahedra****************/
		updateDistanceAndWeights(newCells, vecDistanceWeight2);

		/**********Step 7 raytracing to update the weights after point insertion****************/
		for (int curCam : points_[idxPoint].viewingCams) {
			Segment QO = Segment(hndlQ->point(), cams_[curCam].position);
			rayTracing(curCam, idxPoint, false, true);
		}

		return true;
	} else {
		return false;
	}
}

int ManifoldMeshReconstructor::moveVertex(int idxPoint) {
	if (!points_[idxPoint].toBeMoved) {
		return false;
	}

	Delaunay3::Vertex_handle hndlQ = points_[idxPoint].vertexHandle;

	if ((points_[idxPoint].new_ && hndlQ != NULL) || (!points_[idxPoint].new_ && hndlQ == NULL)) {
		std::cerr << "ManifoldMeshReconstructor::moveVertex: point " << idxPoint << " new xnor (hndlQ == NULL)" << std::endl;
	}

	// If the point isn't in the triangulation, do nothing
	if (hndlQ == NULL) {
		return 0;
	}

	if (hndlQ->point() != points_[idxPoint].position) {
		std::cerr << "ManifoldMeshReconstructor::moveVertex: inconsistent position between vertex handle and point position for point " << idxPoint << std::endl;
	}

	PointD3 initialPosition = hndlQ->point();
	PointD3 newPosition = points_[idxPoint].newPosition;

	bool canMove = true;
	for (int cIndex : points_[idxPoint].viewingCams) {
		CamReconstruction c = cams_[cIndex];
		if (utilities::distanceEucl(c.position, newPosition) > conf_.maxDistanceCamFeature) {
			canMove = false;
			break;
		}
	}

	/* 	Let Br be the ball centered on the camera and r the parameter maxDistanceCamFeature.
	 * 	All the points must be in B if they are added to the triangulation to ensure that manifoldness is preserved.
	 *
	 * 	TODO manage all the cases where the point p is moved:
	 * 		· from inside B to inside B (as it is done)
	 * 		· from inside B to outside B (only remove from triangulation)
	 * 		· from outside B to inside B (insert)
	 * 		· from outside B to outside B (do nothing)
	 */
	if (canMove) {

		points_[idxPoint].position = points_[idxPoint].newPosition;
		points_[idxPoint].toBeMoved = false;

		// Set of rays <cameraIndex, pointIndex> intersecting the hole that need to be retraced
		std::set<pair<int, int>> raysToBeRetraced;

		// Step 0
		// Undo rayTracing for all cells on the rayPaths concerning the point and schedule the rayTracing on those rays
		for (auto rayPath : getRayPathsFromPoint(idxPoint)) {
			//rayUntracing(rayPath);
			raysToBeUntraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

			// rayTracing will be computed again when possible
			raysToBeTraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

//			// Remove all cells from the path (rayTracing will add them back)
//			rayPath->path.clear();
		}

		std::set<Delaunay3::Cell_handle> deadCells;

		// Step 1
		// The incident cells will be removed when the vertex is removed from the triangulation
		std::set<Delaunay3::Cell_handle> setIncidentCells;
		dt_.incident_cells(hndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));
		deadCells.insert(setIncidentCells.begin(), setIncidentCells.end());

		// Schedule retracing for all rays that intersect the cells that will be removed
		for (auto itCell : setIncidentCells) {
			for (auto intersection : itCell->info().getIntersections())
				raysToBeRetraced.insert(pair<int, int>(intersection.first, intersection.second));
		}

		for (auto cell : deadCells)
			if (cell->info().isBoundary()) cerr << "ManifoldMeshReconstructor::moveVertex: destroing boundary cells" << std::endl;

		// Step 2
		// Remove the vertex from the triangulation
		dt_.remove(hndlQ);

		// Step 3
		// Locate the point
		Delaunay3::Locate_type lt;
		int li, lj;
		Delaunay3::Cell_handle c = dt_.locate(newPosition, lt, li, lj);
		if (lt == Delaunay3::VERTEX) {
			cerr << "Error in FreespaceDelaunayAlgorithm::moveVertex(): Attempted to move a vertex to an already existing vertex location" << endl;
			return false;
		}

		// Get the cells in conflict with the new vertex, and a facet on the boundary of this hole in f.
		// These cells will also be removed from the triangulation when the new vertex is inserted
		std::vector<Delaunay3::Cell_handle> vecConflictCells;
		Delaunay3::Facet f;
		dt_.find_conflicts(newPosition, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));
		deadCells.insert(vecConflictCells.begin(), vecConflictCells.end());

		// Schedule retracing for all rays that intersect the cells that will be removed (again)
		for (auto it : vecConflictCells) {
			for (auto intersection : it->info().getIntersections())
				raysToBeRetraced.insert(pair<int, int>(intersection.first, intersection.second));
		}

		for (auto cell : vecConflictCells)
			if (cell->info().isBoundary()) cerr << "ManifoldMeshReconstructor::moveVertex: destroing boundary cells" << std::endl;

		// Step 4
		// Fill the hole by inserting the new vertex
		hndlQ = dt_.insert_in_hole(newPosition, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
		points_[idxPoint].vertexHandle = hndlQ;

		// Vector of the cells that were created to fill the hole
		std::vector<Delaunay3::Cell_handle> newCellsFromHole;
		dt_.incident_cells(hndlQ, std::inserter(newCellsFromHole, newCellsFromHole.begin()));
		newCells_.insert(newCellsFromHole.begin(), newCellsFromHole.end());

		// Step 8
		// Schedule retracing all rays that intersected removed cells
		for (auto ray : raysToBeRetraced) {
			raysToBeRetraced_.insert(pair<int, int>(ray.first, ray.second));

			// Remove the dead cells from paths
			for (auto deadCell : deadCells) {
				getRayPath(ray.first, ray.second)->path.erase(deadCell);
			}

			//rayRetracing(ray.first, ray.second, newCells_);
		}

		return true;
	} else {
		//cout << "moveVertex refused" << endl;
		return false;
	}
	return 1;

}

void ManifoldMeshReconstructor::moveCameraConstraints(int idxCam) {

// The set of all the constraints that
	SetConstraints setUnionedConstraints;

	CamReconstruction& camera = cams_[idxCam];
	PointD3 camPosition = camera.position;
	PointD3 newCamPosition = camera.newPosition;

	if (!camera.toBeMoved) return;

	/* 	Let Br be the ball centered on the camera and r the parameter maxDistanceCamFeature.
	 * 	All the points must be in B if they are added to the triangulation to ensure that manifoldness is preserved.
	 *
	 * 	TODO manage all the cases where the point p is moved:
	 * 		· from inside B to inside B (as it is done)
	 * 		· from inside B to outside B (only remove from triangulation)
	 * 		· from outside B to inside B (insert)
	 * 		· from outside B to outside B (do nothing)
	 */

	bool canMove = true;
	for (int pIndex : camera.visiblePoints) {
		PointReconstruction p = points_[pIndex];
		if (utilities::distanceEucl(p.position, camPosition) > conf_.maxDistanceCamFeature) {
			canMove = false;
			break;
		}
	}

	if (canMove) {
		camera.position = camera.newPosition;
		camera.toBeMoved = false;

		//idxPointsForRayTracing_.clear();

		// Step 6
		// Iterate over all the intersections i with i.second == idxPoint and remove them (and decrement their vote)
//		for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
//
//			// Linear search:
//			for (auto itDelete = itCell->info().getIntersections().begin(); itDelete != itCell->info().getIntersections().end();) {
//				if (itDelete->first == idxCam) {
//					// invalidates iterator, so careful about incrementing it:
//					std::set<FSConstraint, FSConstraint::LtFSConstraint>::const_iterator itNext = itDelete;
//					itNext++;
//					itCell->info().removeIntersection(itDelete->first, itDelete->second, itDelete->vote, points_, camsPositions_);
//					itCell->info().decrementVoteCount(1.0);
//					itCell->info().decrementVoteCountProb(itDelete->vote);
//					itDelete = itNext;
//
//				} else itDelete++;
//			}
//		}

		for (auto rayPath : getRayPathsFromCamera(idxCam)) {

			//rayUntracing(rayPath);
			raysToBeUntraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

			// Step 7: rayTracing will be computed again when possible
			raysToBeTraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

			// remove all cells from the path (rayTracing will add them back anyway)
			rayPath->path.clear();
		}

		// Step 7
//		for (int pIndex : camera.visiblePoints) {
//			//idxPointsForRayTracing_.push_back(pIndex);
//			raysToBeTraced_.insert(pair<int, int>(idxCam, pIndex));
//		}

	} else {
		cout << "moveCameraConstraints refused" << endl;
	}

}


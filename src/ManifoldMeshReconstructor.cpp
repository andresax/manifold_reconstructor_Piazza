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

	timeStatsFile_.open("/home/enrico/gamesh_stats/timeStats.csv");
	timeStatsFile_ << "cameras number, updateSteinerGrid, shrinkManifold, shrinkSingle, shrinkSeveral, Add new vertices, Remove vertices, Move vertices, Move Cameras, rayUntracing, rayTracing, rayRetracing, rayRemoving, growManifold, growManifoldSev, growManifold, InsertInBoundary, RemoveFromBoundary, addedPoints, movedPoints, Overall" << endl;
}

ManifoldMeshReconstructor::~ManifoldMeshReconstructor() {
	delete (manifoldManager_);
	delete (outputM_);
	timeStatsFile_.close();
}

void ManifoldMeshReconstructor::addPoint(float x, float y, float z) {
	PointReconstruction t;
	t.idReconstruction = points_.size();
	t.position = PointD3(x, y, z);
	points_.push_back(t);
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
//	addRay(camIdx, pointIdx);

	if (utilities::distanceEucl(c.position, p.position) < conf_.maxDistanceCamFeature) {
		updateSteinerGridTargetBounds(c.position.x(), c.position.y(), c.position.z());
		updateSteinerGridTargetBounds(p.position.x(), p.position.y(), p.position.z());
	}
}

void ManifoldMeshReconstructor::removeVisibilityPair(RayPath* r) {
	std::vector<int>& cvp = cams_[r->cameraId].visiblePoints;
	cvp.erase(std::remove(cvp.begin(), cvp.end(), r->pointId), cvp.end());

	std::vector<int>& cnvp = cams_[r->cameraId].newVisiblePoints;
	cnvp.erase(std::remove(cnvp.begin(), cnvp.end(), r->pointId), cnvp.end());

	std::vector<int>& pvc = points_[r->pointId].viewingCams;
	pvc.erase(std::remove(pvc.begin(), pvc.end(), r->cameraId), pvc.end());

	if (cvp.size() == 0) cout << "Orphan camera " << r->cameraId << endl; //TODO remove
	if (pvc.size() == 0) {
//		cout << "Orphan point  " << r->pointId << endl;
		pointsToBeRemovedIdx_.push_back(r->pointId);
	}
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
void ManifoldMeshReconstructor::eraseRayPath(RayPath* r) {
	const std::pair<int, int> k = std::pair<int, int>(r->cameraId, r->pointId);
	rayPaths_.erase(pair<int, int>(k));

	camerasRayPaths_[r->cameraId].erase(r);
	pointsRayPaths_[r->pointId].erase(r);

	delete r;
}
std::set<RayPath*> ManifoldMeshReconstructor::getRayPathsFromCamera(int cameraId) {
	return camerasRayPaths_.at(cameraId);
}
std::set<RayPath*> ManifoldMeshReconstructor::getRayPathsFromPoint(int pointId) {
	return pointsRayPaths_.at(pointId);
}

void ManifoldMeshReconstructor::updateTriangulation() {

	timeStatsFile_ << endl << cams_.size() << ", ";

	rt2_ChronoUseless_.reset();
	rt2_ChronoFirstCell_.reset();
	rt2_ChronoCellTraversing_.reset();
	rt2_ChronoNeighboursD1Selection_.reset();
	rt2_ChronoNeighboursD2Selection_.reset();
	rt2_ChronoNeighboursD1WeightUpdate_.reset();
	rt2_ChronoNeighboursD2WeightUpdate_.reset();

	manifoldManager_->chronoInsertInBoundary_.reset();
	manifoldManager_->chronoRemoveFromBoundary_.reset();

	rt2_CountNeighboursD1WeightUpdate_ = 0;
	rt2_CountNeighboursD2WeightUpdate_ = 0;

	rt2_SuccessfulCachedIndices = 0;
	rt2_TriedCachedIndices = 0;

	Chronometer chronoCheck, chronoEverything;
	chronoCheck.start();
	chronoCheck.stop();
	if (conf_.time_stats_output) cout << "ManifoldMeshReconstructor::updateTriangulation: \t\t min chrono time\t\t" << chronoCheck.getNanoseconds() << " ns" << endl;

	int addedPointsStat = 0, movedPointsStat = 0, updatedCamerasStat = updatedCamerasIdx_.size();

	/*
	 *  Steiner grid updating
	 */

	if (dt_.number_of_vertices() == 0) {
		chronoEverything.reset();
		chronoEverything.start();

		initSteinerPointGridAndBound();
		updateSteinerPointGridAndBound();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "├ initSteinerGrid\t\t" << chronoEverything.getSeconds() << endl;
	} else {
		chronoEverything.reset();
		chronoEverything.start();

		updateSteinerPointGridAndBound();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "├ updateSteinerGrid\t\t" << chronoEverything.getSeconds() << endl;
	}
	timeStatsFile_ << chronoEverything.getSeconds() << ", ";

	/*
	 *  Enclosing
	 */

	// TODO also include the points that will be removed
	std::set<PointD3> enclosingVolumePoints;
	if (conf_.update_points_position) for (auto pIndex : pointsMovedIdx_) {
		//		shrinkPoints.insert(points_[pIndex].position);
		enclosingVolumePoints.insert(points_[pIndex].newPosition);
	}
	for (auto cIndex : updatedCamerasIdx_)
		for (auto pIndex : cams_[cIndex].newVisiblePoints)
			if (points_[pIndex].notTriangulated && utilities::distanceEucl(points_[pIndex].position, cams_[cIndex].position) < conf_.maxDistanceCamFeature)
				enclosingVolumePoints.insert(points_[pIndex].position);

	// This is used to cache the enclosing information in the cells, incrementing it invalidates the cached values and needs to be done when the points on which the enclosing volume is base are changed
	currentEnclosingVersion_++;

	// TODO: See if useful when updating points
//	int countRepeatedShrinkPoints = 0;
//	if(lastShrinkPoints_.size()){
//		for( auto p : lastShrinkPoints_) if(shrinkPoints.count(p)) countRepeatedShrinkPoints++;
//		cout << "countRepeatedShrinkPoints:\t" << countRepeatedShrinkPoints << endl;
//	}
//	lastShrinkPoints_.clear();
//	lastShrinkPoints_.insert(shrinkPoints.begin(), shrinkPoints.end());

	/*
	 *  Shrinking
	 */

	timerShrinkTime_ = 0.0;
	timerShrinkSeveralTime_ = 0.0;
	if (enclosingVolumePoints.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		shrinkManifold3(enclosingVolumePoints);

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "├ shrinkManifold\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "├ shrinkManifold\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
	timeStatsFile_ << timerShrinkTime_ << ", " << timerShrinkSeveralTime_ << ", ";

	/*
	 *  Ray removing
	 */

	int verticesRemovedCount = pointsToBeRemovedIdx_.size(), raysRemovedAndInvalidatedCount = 0, raysInvaldatedCount = 0, raysRemovedCount = 0, raysCandidateToBeRemovedCount = raysCandidateToBeRemoved_.size();

	if (raysCandidateToBeRemoved_.size()) {

//		std::vector<Segment> outputRays;
//		for (auto cIndex_pIndex : raysCandidateToBeRemoved_) {
//			RayPath* r = getRayPath(cIndex_pIndex.first, cIndex_pIndex.second);
//			if (r->mistrustVote > conf_.rayRemovalThreshold) outputRays.push_back(Segment(cams_[r->cameraId].position, points_[r->pointId].position));
//		}
//		outputM_->writeRaysToOFF("output/erasedRays/", std::vector<int> { iterationCounter_ }, outputRays);

		chronoEverything.reset();
		chronoEverything.start();
		for (auto cIndex_pIndex : raysCandidateToBeRemoved_) {
			RayPath* r = getRayPath(cIndex_pIndex.first, cIndex_pIndex.second);

			if (r->mistrustVote > conf_.rayRemovalThreshold) {
				raysRemovedCount++;
				if(raysNotValid_.count(cIndex_pIndex)) raysRemovedAndInvalidatedCount++;
				removeRay(r);
			}
		}
		raysCandidateToBeRemoved_.clear();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "│ ├ rayRemoving\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "│ ├ rayRemoving\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
	cout << "ManifoldMeshReconstructor::rayTracingFromAllCam:\t rays removed:\t " << raysRemovedCount << "\t/\t" << raysCandidateToBeRemovedCount << endl;

	if (raysNotValid_.size()) {

//		std::vector<Segment> outputRays;
//		for (auto cIndex_pIndex : raysCandidateToBeRemoved_) {
//			RayPath* r = getRayPath(cIndex_pIndex.first, cIndex_pIndex.second);
//			if (r->mistrustVote > conf_.rayRemovalThreshold) outputRays.push_back(Segment(cams_[r->cameraId].position, points_[r->pointId].position));
//		}
//		outputM_->writeRaysToOFF("output/erasedRays/", std::vector<int> { iterationCounter_ }, outputRays);

		chronoEverything.reset();
		chronoEverything.start();
		for (auto cIndex_pIndex : raysNotValid_) {
			raysInvaldatedCount++;
			//removeRay(cIndex_pIndex.first, cIndex_pIndex.second);
		}
		raysNotValid_.clear();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "│ ├ rayInvalidating\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "│ ├ rayInvalidating\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
	cout << "ManifoldMeshReconstructor::rayTracingFromAllCam:\t rays invalidated:\t " << raysInvaldatedCount << endl;

	cout << "ManifoldMeshReconstructor::rayTracingFromAllCam:\t rays removed AND invalidated:\t " << raysRemovedAndInvalidatedCount << endl;



	/*
	 *  Vertex removing
	 */

	if (pointsToBeRemovedIdx_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (auto id : pointsToBeRemovedIdx_) {
		//	cout << "removing vertex " << id << endl;
			removeVertex(id);
		}
		pointsToBeRemovedIdx_.clear();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "├ Remove vertices\t\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "├ Remove vertices\t\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
	cout << "ManifoldMeshReconstructor::rayTracingFromAllCam:\t vertices removed:\t " << verticesRemovedCount << endl;

	/*
	 *  Vertex inserting
	 */

	if (updatedCamerasIdx_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (auto updatedCameraIndex : updatedCamerasIdx_) {

			for (auto pIndex : cams_[updatedCameraIndex].newVisiblePoints) {
				PointReconstruction& point = points_[pIndex];

				if (point.notTriangulated) {

					if (utilities::distanceEucl(point.position, cams_[updatedCameraIndex].position) < conf_.maxDistanceCamFeature) {

						/*	Try to insert the new point in the triangulation.
						 * 	When successful, a new vertex corresponding to the nwe point is created,
						 * 	some cells are removed from the triangulation and replaced by some others.
						 */
						if (insertVertex(point)) {
							addedPointsStat++;
							point.vertexHandle->info().setLastCam(updatedCameraIndex);
						}
					}

				} else {
					raysToBeTraced_.insert(pair<int, int>(updatedCameraIndex, point.idReconstruction));
					point.vertexHandle->info().setLastCam(updatedCameraIndex);
					for (auto c : point.viewingCams) {
						if (c <= updatedCameraIndex) point.vertexHandle->info().addCam(updatedCameraIndex);
					}

				}
			}

			cams_[updatedCameraIndex].newVisiblePoints.clear(); // TODO test

		}
		updatedCamerasIdx_.clear();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "├ Add new vertices\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "├ Add new vertices\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	/*
	 *  Vertex moving
	 */

	if (conf_.update_points_position && pointsMovedIdx_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (auto id : pointsMovedIdx_) {

			Segment s = Segment(points_[id].position, points_[id].newPosition);

			bool moved;
			moved = moveVertex(id);

			if (moved) movedPointsStat++;

			if (conf_.all_sort_of_output) if (moved) movedPointsSegments_.push_back(s);

		}
		pointsMovedIdx_.clear();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "├ Move vertices\t\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "├ Move vertices\t\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	if (conf_.all_sort_of_output) outputM_->writeRaysToOFF("output/moved_points/moved_points", std::vector<int> { }, movedPointsSegments_);

	/*
	 *  Camera moving
	 */

	if (movedCamerasIdx_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (int cameraIndex : movedCamerasIdx_) {
			moveCameraConstraints(cameraIndex);
		}
		movedCamerasIdx_.clear();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "├ Move Cameras\t\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "├ Move Cameras\t\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	/*
	 *  Ray untracing
	 *  Ray tracing
	 *  Ray retracing
	 */

	raysToBeChecked_.insert(raysToBeTraced_.begin(), raysToBeTraced_.end());

	rayTracingFromAllCam();

	/*
	 *  Grow
	 */

	growManifold3(enclosingVolumePoints);

	/*
	 *  Ray checking
	 */
	for(auto cIndex_pIndex : raysToBeChecked_){
		RayPath* r = getRayPath(cIndex_pIndex.first, cIndex_pIndex.second);
		for (Delaunay3::Cell_handle c : r->path) {
			if(!c->info().iskeptManifold()){
				r->valid = false;
				raysNotValid_.insert(cIndex_pIndex);
			}

		}
	}
	raysToBeChecked_.clear();


	timeStatsFile_ << manifoldManager_->chronoInsertInBoundary_.getSeconds() << ", ";
	timeStatsFile_ << manifoldManager_->chronoRemoveFromBoundary_.getSeconds() << ", ";

	timeStatsFile_ << (float) addedPointsStat / 100 / updatedCamerasStat << ", ";
	timeStatsFile_ << (float) movedPointsStat / 100 / updatedCamerasStat << ", ";

	if (conf_.time_stats_output) {
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t mm \t updated cameras:\t\t\t" << updatedCamerasStat << endl;
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t mm \t added points:\t\t\t" << addedPointsStat << endl;
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t mm \t moved points:\t\t\t" << movedPointsStat << endl;

		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t mm \t insertInBoundary_:\t\t\t" << manifoldManager_->chronoInsertInBoundary_.getSeconds() << " s" << endl;
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t mm \t removeFromBoundary_:\t\t\t" << manifoldManager_->chronoRemoveFromBoundary_.getSeconds() << " s" << endl;

		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t rt2\t useless:\t\t\t" << rt2_ChronoUseless_.getSeconds() << " s" << endl;
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t rt2\t first cell:\t\t\t" << rt2_ChronoFirstCell_.getSeconds() << " s" << endl;
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t rt2\t cell traversing:\t\t" << rt2_ChronoCellTraversing_.getSeconds() << " s" << endl;
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t rt2\t index cache hit ratio:\t\t" << (double) rt2_SuccessfulCachedIndices / (double) rt2_TriedCachedIndices << endl;
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t rt2\t neighbours weight update:\t\t" << rt2_ChronoNeighboursD1WeightUpdate_.getSeconds() << " s\t / \t" << rt2_CountNeighboursD1WeightUpdate_ << endl;
		cout << "ManifoldMeshReconstructor::updateTriangulation:\t\t\t rt2\t neighbours int insert/remove:\t\t" << rt2_ChronoNeighboursD2WeightUpdate_.getSeconds() << " s\t / \t" << rt2_CountNeighboursD2WeightUpdate_ << endl;
		cout << endl;
	}

	iterationCounter_++;
}

void ManifoldMeshReconstructor::rayTracingFromAllCam() {
	Chronometer chronoEverything;

	if (raysToBeUntraced_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (auto cIndex_pIndex : raysToBeUntraced_) {
			rayUntracing2(getRayPath(cIndex_pIndex.first, cIndex_pIndex.second));
		}
		raysToBeUntraced_.clear();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "│ ├ rayUntracing\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "│ ├ rayUntracing\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	if (raysToBeTraced_.size()) {
		chronoEverything.reset();
		chronoEverything.start();
		for (auto cIndex_pIndex : raysToBeTraced_) {
			raysToBeRetraced_.erase(cIndex_pIndex);
			rayTracing4(cIndex_pIndex.first, cIndex_pIndex.second);
		}
		raysToBeTraced_.clear();

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "│ ├ rayTracing\t\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "│ ├ rayTracing\t\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	if (raysToBeRetraced_.size()) {
		chronoEverything.reset();
		chronoEverything.start();
		for (auto cIndex_pIndex : raysToBeRetraced_) {
			rayRetracing4(cIndex_pIndex.first, cIndex_pIndex.second);
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

		chronoEverything.stop();
		if (conf_.time_stats_output) cout << "│ ├ rayRetracing\t\t" << chronoEverything.getSeconds() << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.time_stats_output) cout << "│ ├ rayRetracing\t\tSkipped" << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
//
//	int raysRemovedCount = 0, raysCandidateToBeRemovedCount = raysCandidateToBeRemoved_.size();
//
//	if (raysCandidateToBeRemoved_.size()) {
//
////		std::vector<Segment> outputRays;
////		for (auto cIndex_pIndex : raysCandidateToBeRemoved_) {
////			RayPath* r = getRayPath(cIndex_pIndex.first, cIndex_pIndex.second);
////			if (r->mistrustVote > conf_.rayRemovalThreshold) outputRays.push_back(Segment(cams_[r->cameraId].position, points_[r->pointId].position));
////		}
////		outputM_->writeRaysToOFF("output/erasedRays/", std::vector<int> { iterationCounter_ }, outputRays);
//
//		chronoEverything.reset();
//		chronoEverything.start();
//		for (auto cIndex_pIndex : raysCandidateToBeRemoved_) {
//			RayPath* r = getRayPath(cIndex_pIndex.first, cIndex_pIndex.second);
//
//			if (r->mistrustVote > conf_.rayRemovalThreshold) {
//				raysRemovedCount++;
//				removeRay(r);
//			}
//		}
//		raysCandidateToBeRemoved_.clear();
//
//		chronoEverything.stop();
//		if (conf_.time_stats_output) cout << "│ ├ rayRemoving\t\t" << chronoEverything.getSeconds() << endl;
//		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
//	} else {
//		if (conf_.time_stats_output) cout << "│ ├ rayRemoving\t\tSkipped" << endl;
//		timeStatsFile_ << 0.0 << ", ";
//	}
//	cout << "ManifoldMeshReconstructor::rayTracingFromAllCam:\t rays removed:\t " << raysRemovedCount << "\t/\t" << raysCandidateToBeRemovedCount << endl;

}

void ManifoldMeshReconstructor::rayUntracing2(RayPath* rayPath) {
	int cameraIndex = rayPath->cameraId;
	int pointIndex = rayPath->pointId;

	// Remove all the dead cells from the path
	rayPath->path.erase(std::remove_if(rayPath->path.begin(), rayPath->path.end(), [&](Delaunay3::Cell_handle cell) {
		return !dt_.is_cell(cell);
	}), rayPath->path.end());

	// For all the cells in the ray's path, do the opposite of rayTracing
	for (auto c : rayPath->path) {
		unmarkCell(c, cameraIndex, pointIndex);
	}

	// Remove all cells from the path (rayTracing will add them back)
	rayPath->path.clear();

}

void ManifoldMeshReconstructor::rayRetracing4(int cameraIndex, int pointIndex) {
	rayTracing4(cameraIndex, pointIndex, true);
}

void ManifoldMeshReconstructor::rayTracing4(int cameraIndex, int pointIndex, bool retrace) {
	std::vector<Delaunay3::Cell_handle> incidentCells;
	Delaunay3::Cell_handle previousCell, currentCell, targetCell;
	Vertex3D_handle vertexHandle = points_[pointIndex].vertexHandle;
	Delaunay3::Locate_type lt;
	int li, lj;

	PointD3 source = points_[pointIndex].position;
	PointD3 target = cams_[cameraIndex].position;
	Segment constraint = Segment(source, target);

	bool firstExitFacetFound = false;
	long int iterationCount = 0;

	RayPath* rayPath = getRayPath(cameraIndex, pointIndex);
	if (!retrace && rayPath->path.size())
		cerr << "ManifoldMeshReconstructor::rayTracing: ray path not empty before rayTracing for ray " << cameraIndex << ", " << pointIndex << endl;
	rayPath->path.clear();

	if (vertexHandle == NULL) {
		cerr << "ManifoldMeshReconstructor::rayTracing: ignoring ray because vertex not in triangulation; ray " << cameraIndex << ", " << pointIndex << endl;
		return;
	}

	rt2_ChronoFirstCell_.start();

	// incidentCells contains the cells incident to the point's vertex
	dt_.incident_cells(vertexHandle, std::back_inserter(incidentCells));

	if (incidentCells.size() == 0) std::cerr << "ManifoldMeshReconstructor::rayTracing: no incident cells found for ray " << cameraIndex << ", " << pointIndex << std::endl;
	if (vertexHandle == NULL) std::cerr << "ManifoldMeshReconstructor::rayTracing: vertexHandle is NULL for ray " << cameraIndex << ", " << pointIndex << std::endl;

	// Locate the target cell
	targetCell = dt_.locate(target, lt, li, lj);

	if (lt != Delaunay3::Locate_type::CELL) {
//		cerr << "ManifoldMeshReconstructor::rayTracing: (WORKAROUND) moving camera position because overlapping a vertex for ray " << cameraIndex << ", " << pointIndex << endl;
		target = PointD3(target.x() + 0.0001 * conf_.steinerGridStepLength, target.y() + 0.0001 * conf_.steinerGridStepLength, target.z() + 0.0001 * conf_.steinerGridStepLength);
		constraint = Segment(source, target);

		targetCell = dt_.locate(target, lt, li, lj);

		if (lt != Delaunay3::Locate_type::CELL) {
			cerr << "ManifoldMeshReconstructor::rayTracing: (WORKAROUND) moving camera position because overlapping a vertex; didn't work, aborting rayTracing for ray " << cameraIndex << ", " << pointIndex << endl;
			return;
		}
	}

	// If one of the incident cells also contains the camera, the ray ends in that cell. Mark it and return
	for (auto i : incidentCells) {
		if (i == targetCell) {
			markCell(i, cameraIndex, pointIndex, rayPath->path, retrace);
			if (!retrace) markRemovalCandidateRays(vertexHandle, i, incidentCells); // TODO also if retracing?
			return;
		}
	}

	// Otherwise there has to be a neighbour cell that intersects with the ray and it is the neighbour opposite to the point's vertex
	for (auto i : incidentCells) {

		// facetIndex is the index of the facet opposite to the point's vertex in the incident cell i
		int facetIndex = i->index(vertexHandle);

		// If the facet (i, facetIndex) intersects with the ray, the corresponding neighbour is the next cell,
		// then mark the first cell and initialise the first two cells (previous and current)
		if (CGAL::do_intersect(dt_.triangle(i, facetIndex), constraint)) {
			firstExitFacetFound = true;

			markCell(i, cameraIndex, pointIndex, rayPath->path, retrace);
			if (!retrace) markRemovalCandidateRays(vertexHandle, i, incidentCells); // TODO also if retracing?

			previousCell = i;
			currentCell = i->neighbor(facetIndex);

			break;
		}
	}
	rt2_ChronoFirstCell_.stop();

	if (!firstExitFacetFound) {
		std::cerr << "firstExitFacet NOT FOUND for ray " << cameraIndex << ", " << pointIndex << "\tincidentCells.size(): " << incidentCells.size() << std::endl;
		return;
	}

	rt2_ChronoCellTraversing_.start();

	// Follow the ray through the triangulation and mark the cells
	do {
		iterationCount++;
		markCell(currentCell, cameraIndex, pointIndex, rayPath->path, retrace);
	} while (nextCellOnRay(currentCell, previousCell, targetCell, constraint) && iterationCount < 10000); // TODO no magic

	if (iterationCount >= 10000) cerr << "ManifoldMeshReconstructor::rayTracing: max iterations exceeded for ray " << cameraIndex << ", " << pointIndex << endl;

	rt2_ChronoCellTraversing_.stop();

}

// TODO how much useful is this, again? (wrt retracing all ray)
void ManifoldMeshReconstructor::perHoleRayRetracing(std::set<Delaunay3::Cell_handle>& newCells) {
	// The boundary is the set of faces between the old cells and the new cells.
	// All new cells have at least one face in the boundary.
	// All deleted cells also had at least one face in the boundary.

	// Step 1.
	// newCell is intersected by all rays that intersected some of its boundary face.
	// For each of these intersections update the weight of newCell

	// Step 2.
	// It's possible that a newCell

}

bool ManifoldMeshReconstructor::nextCellOnRay(
		Delaunay3::Cell_handle& currentCell, Delaunay3::Cell_handle& previousCell, const Delaunay3::Cell_handle& targetCell, const Segment& constraint) {

	if (currentCell == targetCell) return false;

	int cachedFacetIndex = currentCell->info().getRayTracingLastFacetIndex();

	if (cachedFacetIndex > -1) {
		Delaunay3::Cell_handle candidateNextCell = currentCell->neighbor(cachedFacetIndex);

		rt2_TriedCachedIndices++;

		// Note: dt_.triangle(currentCell, facetIndex) is the facet common to currentCell and candidateNextCell
		if (candidateNextCell != previousCell && CGAL::do_intersect(dt_.triangle(currentCell, cachedFacetIndex), constraint)) {

			previousCell = currentCell;
			currentCell = candidateNextCell;

			rt2_SuccessfulCachedIndices++;

			return true;
		}
	}

	// From the current tetrahedron, find the facets that intersects with the ray, excluding the entry facet
	for (int facetIndex = 0; facetIndex < 4; facetIndex++) {

		// Since the cached facet index has already been checked, don't try again
		if (cachedFacetIndex == facetIndex) continue;

		Delaunay3::Cell_handle candidateNextCell = currentCell->neighbor(facetIndex);

		// Note: dt_.triangle(currentCell, facetIndex) is the facet common to currentCell and candidateNextCell
		if (candidateNextCell != previousCell && CGAL::do_intersect(dt_.triangle(currentCell, facetIndex), constraint)) {

			currentCell->info().setRayTracingLastFacetIndex(facetIndex);

			previousCell = currentCell;
			currentCell = candidateNextCell;

			return true;
		}
	}

	cerr << "ManifoldMeshReconstructor::nextCellOnRay: no next cell found" << endl;

	return false;
}

void ManifoldMeshReconstructor::markCell(Delaunay3::Cell_handle& c, const int cameraIndex, const int pointIndex, std::vector<Delaunay3::Cell_handle>& path, bool onlyMarkNewCells) {

	// TODO in case of perHoleRayRetracing?
	path.push_back(c);

	if (!onlyMarkNewCells || c->info().isNew()) {

		rt2_ChronoNeighboursD1WeightUpdate_.start();
		c->info().incrementVoteCount(1);
		c->info().incrementVoteCountProb(conf_.w_1);
		rt2_ChronoNeighboursD1WeightUpdate_.stop();

		rt2_ChronoNeighboursD2WeightUpdate_.start();
		if (!conf_.enableSuboptimalPolicy) c->info().addIntersection(cameraIndex, pointIndex, conf_.w_1);
		rt2_ChronoNeighboursD2WeightUpdate_.stop();

		rt2_ChronoNeighboursD1WeightUpdate_.start();
		for (int in1 = 0; in1 < 4; in1++) {
			Delaunay3::Cell_handle n1 = c->neighbor(in1);

			if (!onlyMarkNewCells || c->info().isNew()) {
				n1->info().incrementVoteCount(1);
				n1->info().incrementVoteCountProb(conf_.w_2);
			}

			for (int in2 = 0; in2 < 4; in2++) {
				Delaunay3::Cell_handle n2 = n1->neighbor(in2);

				if (!onlyMarkNewCells || c->info().isNew()) {
					n2->info().incrementVoteCount(1);
					n2->info().incrementVoteCountProb(conf_.w_3);
				}

			}
		}

		rt2_ChronoNeighboursD1WeightUpdate_.stop();

	}
}

void ManifoldMeshReconstructor::unmarkCell(Delaunay3::Cell_handle& c, const int cameraIndex, const int pointIndex) {

	rt2_ChronoNeighboursD1WeightUpdate_.start();
	c->info().decrementVoteCount(1);
	c->info().decrementVoteCountProb(conf_.w_1);
	rt2_ChronoNeighboursD1WeightUpdate_.stop();

	rt2_ChronoNeighboursD2WeightUpdate_.start();
	if (!conf_.enableSuboptimalPolicy) c->info().removeIntersection(cameraIndex, pointIndex);
	rt2_ChronoNeighboursD2WeightUpdate_.stop();

	rt2_ChronoNeighboursD1WeightUpdate_.start();
	for (int in1 = 0; in1 < 4; in1++) {
		Delaunay3::Cell_handle n1 = c->neighbor(in1);

		n1->info().decrementVoteCount(1);
		n1->info().decrementVoteCountProb(conf_.w_2);

		for (int in2 = 0; in2 < 4; in2++) {
			Delaunay3::Cell_handle n2 = n1->neighbor(in2);

			n2->info().decrementVoteCount(1);
			n2->info().decrementVoteCountProb(conf_.w_3);
		}
	}
	rt2_ChronoNeighboursD1WeightUpdate_.stop();
}

void ManifoldMeshReconstructor::markRemovalCandidateRays(Vertex3D_handle& v, Delaunay3::Cell_handle& c, std::vector<Delaunay3::Cell_handle>& incidentCells) {
	std::vector<Delaunay3::Cell_handle> neighbours;
	for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++)
		neighbours.push_back(c->neighbor(neighbourIndex));

	// Select cells i that are incident to vertex v and opposed to cell c (not neighbours),
	// and for each of them increment mistrust vote on intersecting rays and add them to raysCandidateToBeRemoved_.
	for (auto i : incidentCells) {
		if (i != c && i != neighbours[0] && i != neighbours[1] && i != neighbours[2] && i != neighbours[3]) {
			int intersectionsCount = i->info().getIntersections().size();
			for (auto intersection : i->info().getIntersections()) {
				getRayPath(intersection.first, intersection.second)->mistrustVote += conf_.w_m/intersectionsCount;
//				cout << getRayPath(intersection.first, intersection.second)->mistrustVote << endl;
				raysCandidateToBeRemoved_.insert(pair<int, int>(intersection.first, intersection.second));
			}
		}
	}

	// TODO can take into accout the freeVote of cell i or the number of rays intersecting the cell (to avoid mass removing)?
}

void ManifoldMeshReconstructor::removeRay(RayPath* r) {

	// Untrace the ray
	rayUntracing2(r);

	// Remove the visibility pair
	removeVisibilityPair(r);

	// Remove it from the paths
	eraseRayPath(r);
}

void ManifoldMeshReconstructor::growManifold3(std::set<PointD3> points) {
//	if (!freeSpaceTets_.size()) {
//		cerr << "freeSpaceTets_ is empty; Can't grow" << endl;
//		return;
//	}
	Chronometer chronoEverything;

	if (conf_.all_sort_of_output) saveBoundary(1, 0);

	// TODO move the initialisation to updateTriangulation
	chronoEverything.reset();
	chronoEverything.start();
	if (manifoldManager_->getBoundarySize() == 0) {
		double max = 0.0;
		Delaunay3::Cell_handle startingCell;

		// If the boundary is still empty, initialise all the cells' informations and start growing from the cell with highest vote
		//TODO from every cell conaining cameras; otherwise disconnected spaces wouldn't all be grown
		for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
//			itCell->info().setKeptManifold(false);
//			for (int curV = 0; curV < 4; ++curV) {
//				itCell->vertex(curV)->info().setUsed(0);
//				itCell->vertex(curV)->info().setNotUsed(true);
//			}

			if (itCell->info().getVoteCountProb() > max) {
				max = itCell->info().getVoteCountProb();
				startingCell = itCell;
			}
		}

		manifoldManager_->regionGrowingBatch3(startingCell, points);
	} else {
		manifoldManager_->regionGrowing3(points);
	}

	chronoEverything.stop();
	if (conf_.time_stats_output) cout << "│ ├ growManifold\t\t" << chronoEverything.getSeconds() << endl;
	timeStatsFile_ << chronoEverything.getSeconds() << ", ";

	if (conf_.all_sort_of_output) saveBoundary(1, 1);

	chronoEverything.reset();
	chronoEverything.start();
	manifoldManager_->growSeveralAtOnce3(points);
	chronoEverything.stop();
	if (conf_.time_stats_output) cout << "│ ├ growManifoldSev\t\t" << chronoEverything.getSeconds() << endl;
	timeStatsFile_ << chronoEverything.getSeconds() << ", ";

	if (conf_.all_sort_of_output) saveBoundary(1, 2);

	chronoEverything.reset();
	chronoEverything.start();
	manifoldManager_->regionGrowing3(points);
	chronoEverything.stop();
	if (conf_.time_stats_output) cout << "│ ├ growManifold\t\t" << chronoEverything.getSeconds() << endl;
	timeStatsFile_ << chronoEverything.getSeconds() << ", ";

	if (conf_.all_sort_of_output) saveBoundary(1, 3);
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

	std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> b = manifoldManager_->getBoundaryCells();
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
	Chronometer chronoEverything;

	if (conf_.all_sort_of_output) saveBoundary(0, 0);

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkManifold3(points, l_, currentEnclosingVersion_);

	chronoEverything.stop();
	if (conf_.time_stats_output) cout << "│ ├ shrink\t\t\t" << chronoEverything.getSeconds() << endl;
	timerShrinkTime_ += chronoEverything.getSeconds();

	if (conf_.all_sort_of_output) saveBoundary(0, 1);

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkSeveralAtOnce3(points, l_, currentEnclosingVersion_);

	chronoEverything.stop();
	if (conf_.time_stats_output) cout << "│ ├ shrinkSeveral\t\t" << chronoEverything.getSeconds() << endl;
	timerShrinkSeveralTime_ += chronoEverything.getSeconds();

	if (conf_.all_sort_of_output) saveBoundary(0, 2);

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkManifold3(points, l_, currentEnclosingVersion_);

	chronoEverything.stop();
	if (conf_.time_stats_output) cout << "│ ├ shrink\t\t\t" << chronoEverything.getSeconds() << endl;
	timerShrinkTime_ += chronoEverything.getSeconds();

	if (conf_.all_sort_of_output) saveBoundary(0, 3);

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkSeveralAtOnce3(points, l_, currentEnclosingVersion_);

	if (conf_.time_stats_output) cout << "│ ├ shrinkSeveral\t\t" << chronoEverything.getSeconds() << endl;
	timerShrinkSeveralTime_ += chronoEverything.getSeconds();

	if (conf_.all_sort_of_output) saveBoundary(0, 4);

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkManifold3(points, l_, currentEnclosingVersion_);

	chronoEverything.stop();
	if (conf_.time_stats_output) cout << "│ ├ shrink\t\t\t" << chronoEverything.getSeconds() << endl;
	timerShrinkTime_ += chronoEverything.getSeconds();

	if (conf_.all_sort_of_output) saveBoundary(0, 5);

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

bool ManifoldMeshReconstructor::insertVertex(PointReconstruction& point) {

	// If the point is marked to be moved but wasn't inserted, use the new position
	if (point.notTriangulated && point.toBeMoved) {
		point.toBeMoved = false;
		point.position = point.newPosition;
	}

	// Locate the new point
	Delaunay3::Locate_type lt;
	int li, lj;
	Delaunay3::Cell_handle c = dt_.locate(point.position, lt, li, lj);

	if (!point.notTriangulated) {
		std::cerr << "ManifoldMeshReconstructor::insertVertex: point is already marked as triangulated!" << std::endl;
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
		if (cell->info().isBoundary()) cerr << "ManifoldMeshReconstructor::insertVertex: destroying boundary cells" << std::endl;

	if (!conf_.enableSuboptimalPolicy) {
		for (auto removedCell : removedCells) {
			for (auto constraint : removedCell->info().getIntersections())
				raysToBeRetraced.insert(pair<int, int>(constraint.first, constraint.second));
		}
	} else {

//		for (auto removedCell : removedCells) {
//			PointD3 temp = CGAL::barycenter(
//					removedCell->vertex(0)->point(), 1.0, removedCell->vertex(1)->point(), 1.0, removedCell->vertex(2)->point(), 1.0, removedCell->vertex(3)->point(), 1.0);
//			vecDistanceWeight_.push_back(DistanceWeight(temp, (float) removedCell->info().getVoteCountProb()));
//		}
		cerr << "ManifoldMeshReconstructor::insertVertex:\tenableSuboptimalPolicy is depracated in this version" << endl;
	}

	// Schedule a reyRetracing for all the rays that intersected the removed cells
	for (auto ray : raysToBeRetraced) {
		raysToBeRetraced_.insert(pair<int, int>(ray.first, ray.second));
		//rayRetracing(ray.first, ray.second, newCellsFromHole);

		std::vector<Delaunay3::Cell_handle>& path = getRayPath(ray.first, ray.second)->path;

		// rayRetracing will manage the dead cells
		// TODO removed to make the path a vector
//		// TODO this is probably slower than something else.
//		// Remove dead cells from paths
//		for (auto removedCell : removedCells) {
//			path.erase(std::remove_if(path.begin(), path.end(), [&](Delaunay3::Cell_handle cell) {
//				return cell == removedCell;
//			}), path.end());
//		}
	}

	// Creates a new vertex by starring a hole. Delete all the cells describing the hole vecConflictCells, creates a new vertex hndlQ, and for each facet on the boundary of the hole f, creates a new cell with hndlQ as vertex.
	Vertex3D_handle vertexHandle = dt_.insert_in_hole(point.position, removedCells.begin(), removedCells.end(), f.first, f.second);

	// Set of the cells that were created to fill the hole, used by rayRetracing to restore the rays
	std::vector<Delaunay3::Cell_handle> newCellsFromHole;
	dt_.incident_cells(vertexHandle, std::inserter(newCellsFromHole, newCellsFromHole.begin()));
	newCells_.insert(newCellsFromHole.begin(), newCellsFromHole.end());

	// Add the new vertex handle hndlQ to vecVertexHandles
	vecVertexHandles_.push_back(vertexHandle);
	point.idVertex = vecVertexHandles_.size() - 1;
	point.vertexHandle = vertexHandle;
	point.notTriangulated = false;

	if (!conf_.enableSuboptimalPolicy) {
		// Schedule rayTracing for all rays between the point and the cameras viewing it
		for (auto cameraIndex : point.viewingCams)
			raysToBeTraced_.insert(pair<int, int>(cameraIndex, point.idReconstruction));
	}

	if (conf_.enableSuboptimalPolicy) { // why's this?
//		updateDistanceAndWeights(newCellsFromHole, vecDistanceWeight_);
		cerr << "ManifoldMeshReconstructor::insertVertex:\tenableSuboptimalPolicy is depracated in this version" << endl;
	}

	return true;

}

int ManifoldMeshReconstructor::moveVertex(int idxPoint) {
	PointReconstruction& point = points_[idxPoint];

	if (!point.toBeMoved) {
		return false;
	}

	Delaunay3::Vertex_handle vertexHandle = point.vertexHandle;

	if ((point.notTriangulated && vertexHandle != NULL) || (!point.notTriangulated && vertexHandle == NULL)) {
		std::cerr << "ManifoldMeshReconstructor::moveVertex: point " << idxPoint << " new xnor (hndlQ == NULL)" << std::endl;
	}

	// If the point isn't in the triangulation, do nothing
	if (vertexHandle == NULL) {
		return 0;
	}

	if (vertexHandle->point() != point.position) {
		std::cerr << "ManifoldMeshReconstructor::moveVertex: inconsistent position between vertex handle and point position for point " << idxPoint << std::endl;
	}

	PointD3 initialPosition = vertexHandle->point();
	PointD3 newPosition = point.newPosition;

	bool canMove = true;
	for (int cIndex : point.viewingCams) {
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

		point.position = point.newPosition;
		point.toBeMoved = false;

		// Set of rays <cameraIndex, pointIndex> intersecting the hole that needs to be retraced
		std::set<pair<int, int>> raysToBeRetraced;

		// Step 0
		// Undo rayTracing for all cells on the rayPaths concerning the point and schedule the rayTracing on those rays
		for (auto rayPath : getRayPathsFromPoint(idxPoint)) {
			//rayUntracing(rayPath);
			raysToBeUntraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

			// rayTracing will be computed again when possible
			raysToBeTraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

		}

		std::set<Delaunay3::Cell_handle> deadCells;

		// Step 1
		// The incident cells will be removed when the vertex is removed from the triangulation
		std::set<Delaunay3::Cell_handle> setIncidentCells;
		dt_.incident_cells(vertexHandle, std::inserter(setIncidentCells, setIncidentCells.begin()));
		deadCells.insert(setIncidentCells.begin(), setIncidentCells.end());

		// Schedule retracing for all rays that intersect the cells that will be removed
		for (auto itCell : setIncidentCells) {
			for (auto intersection : itCell->info().getIntersections())
				raysToBeRetraced.insert(pair<int, int>(intersection.first, intersection.second));
		}

		for (auto cell : deadCells)
			if (cell->info().isBoundary()) cerr << "ManifoldMeshReconstructor::moveVertex: destroying boundary cells" << std::endl;

		// Step 2
		// Remove the vertex from the triangulation
		dt_.remove(vertexHandle);

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
			if (cell->info().isBoundary()) cerr << "ManifoldMeshReconstructor::moveVertex: destroying boundary cells" << std::endl;

		// Step 4
		// Fill the hole by inserting the new vertex
		vertexHandle = dt_.insert_in_hole(newPosition, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
		point.vertexHandle = vertexHandle;

		// Vector of the cells that were created to fill the hole
		std::vector<Delaunay3::Cell_handle> newCellsFromHole;
		dt_.incident_cells(vertexHandle, std::inserter(newCellsFromHole, newCellsFromHole.begin()));
		newCells_.insert(newCellsFromHole.begin(), newCellsFromHole.end());

		// Step 8
		// Schedule retracing all rays that intersected removed cells
		for (auto ray : raysToBeRetraced) {
			raysToBeRetraced_.insert(pair<int, int>(ray.first, ray.second));

			// TODO removed to make the path a vector
			// Remove the dead cells from paths
//			for (auto deadCell : deadCells) {
//				getRayPath(ray.first, ray.second)->path.erase(deadCell);
//			}

			//rayRetracing(ray.first, ray.second, newCells_);
		}

		return true;
	} else {
		//cout << "moveVertex refused" << endl;
		return false;
	}
	return 1;

}

void ManifoldMeshReconstructor::removeVertex(int idxPoint) {
	PointReconstruction& point = points_[idxPoint];
	Delaunay3::Vertex_handle vertexHandle = point.vertexHandle;

	if ((point.notTriangulated && vertexHandle != NULL) || (!point.notTriangulated && vertexHandle == NULL)) {
		std::cerr << "ManifoldMeshReconstructor::removeVertex: point " << idxPoint << " new xnor (vertexHandle == NULL)" << std::endl;
	}

	// If the point isn't in the triangulation, do nothing
	if (vertexHandle == NULL || point.notTriangulated) {
		std::cerr << "ManifoldMeshReconstructor::removeVertex: trying remove a vertex not in triangulation; point " << idxPoint << std::endl;
		return;
	}

	if (vertexHandle->point() != point.position) {
		std::cerr << "ManifoldMeshReconstructor::removeVertex: inconsistent position between vertex handle and point position for point " << idxPoint << std::endl;
	}

	// Set of rays <cameraIndex, pointIndex> intersecting the hole that needs to be retraced
	std::set<pair<int, int>> raysToBeRetraced;
	std::set<Delaunay3::Cell_handle> deadCells;

	// Step 1
	// The incident cells will be removed when the vertex is removed from the triangulation
	std::set<Delaunay3::Cell_handle> setIncidentCells;
	dt_.incident_cells(vertexHandle, std::inserter(setIncidentCells, setIncidentCells.begin()));
	deadCells.insert(setIncidentCells.begin(), setIncidentCells.end());

	// Schedule retracing for all rays that intersect the cells that will be removed
	for (auto itCell : setIncidentCells)
		for (auto intersection : itCell->info().getIntersections())
			raysToBeRetraced.insert(pair<int, int>(intersection.first, intersection.second));

	for (auto cell : deadCells)
		if (cell->info().isBoundary()) cerr << "ManifoldMeshReconstructor::removeVertex: destroying boundary cells; vertex " << idxPoint << std::endl;

	// Step 2
	// Remove the vertex from the triangulation
	dt_.remove(vertexHandle);

	point.notTriangulated = true;
	point.vertexHandle = NULL;
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


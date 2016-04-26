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

ManifoldMeshReconstructor::ManifoldMeshReconstructor(ManifoldReconstructionConfig conf) {
	conf_ = conf;
	manifoldManager_ = new ManifoldManager(dt_, conf_.inverseConicEnabled, conf_.probOrVoteThreshold);
	outputM_ = new OutputCreator(dt_);
	//fileOut_.open("ManifoldMeshReconstructor.log");
	l_ = 0;
	stepX_ = stepY_ = stepZ_ = -1;
}

ManifoldMeshReconstructor::~ManifoldMeshReconstructor() {
	delete (manifoldManager_);
	delete (outputM_);

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
	t.position = PointD3(x, y, z);
	points_.push_back(t);
}

void ManifoldMeshReconstructor::movePoint(int idxPoint, float x, float y, float z) {
//	points_[idxPoint].position = PointD3(x, y, z);
	points_[idxPoint].newPosition = PointD3(x, y, z);
	points_[idxPoint].toBeMoved = true;
	pointsMovedIdx_.push_back(idxPoint);
}

PointD3 ManifoldMeshReconstructor::movePointGetOld(int idxPoint, float x, float y, float z) {
//	PointD3 old = points_[idxPoint].position;
//	points_[idxPoint].position = PointD3(x, y, z);
//	pointsMovedIdx_.push_back(idxPoint);
//	return old;

	points_[idxPoint].newPosition = PointD3(x, y, z);
	pointsMovedIdx_.push_back(idxPoint);
	return points_[idxPoint].position;
}

int ManifoldMeshReconstructor::addPointWhere(float x, float y, float z) {
	PointReconstruction t;
	t.position = PointD3(x, y, z);
	points_.push_back(t);

	return points_.size() - 1;
}
void ManifoldMeshReconstructor::addCameraCenter(float x, float y, float z) {
	CamReconstruction t;
	t.position = PointD3(x, y, z);
	glm::vec3 pos = glm::vec3(x, y, z);

	cams_.push_back(t);
	camsPositions_.push_back(pos);
}

void ManifoldMeshReconstructor::addVisibilityPair(int camIdx, int pointIdx) {
	cams_[camIdx].visiblePoints.push_back(pointIdx);
	cams_[camIdx].newVisiblePoints.push_back(pointIdx);
	points_[pointIdx].viewingCams.push_back(camIdx);
	addRay(camIdx, pointIdx);
}

bool ManifoldMeshReconstructor::hasVisibilityPair(int camIdx, int pointIdx) {
	for (auto i : cams_[camIdx].visiblePoints)
		if (i == pointIdx) return true;
	return false;
}

void ManifoldMeshReconstructor::addRay(int cameraId, int pointId) {
	RayReconstruction* v = new RayReconstruction();

	v->cameraId = cameraId;
	v->pointId = pointId;

	const std::pair<int, int> k = std::pair<int, int>(cameraId, pointId);
	rays_.insert(std::pair<std::pair<int, int>, RayReconstruction*>(k, v));
	// TODO
	/*
	 * if(no camerasRays_[cameraId]) new vector; get vector; push_back
	 const std::pair<const int, RayReconstruction*> p_c = std::pair<cameraId, v>;
	 camerasRays_.insert(p_c);

	 const std::pair<const int, RayReconstruction*> p_p = std::pair<pointId, v>;
	 pointsRays_.insert(p_p);
	 */
}

RayReconstruction* ManifoldMeshReconstructor::getRay(int cameraId, int pointId) {
	const std::pair<int, int> k = std::pair<int, int>(cameraId, pointId);
	return rays_.at(k);
}
//std::vector<RayReconstruction*> ManifoldMeshReconstructor::getRaysFromCamera(int cameraId) {
//	return camerasRays_.at(cameraId);
//}
//std::vector<RayReconstruction*> ManifoldMeshReconstructor::getRaysFromPoint(int pointId) {
//	return pointsRays_.at(pointId);
//}

void ManifoldMeshReconstructor::insertNewPointsFromCam(int camIdx, bool incremental) {
	if (dt_.number_of_vertices() == 0) {
		logger_.startEvent();
		createSteinerPointGridAndBound();
		logger_.endEventAndPrint("│ ├ createSteinerPointGridAndBound\t\t", true);
	}
	if (incremental) {
		logger_.startEvent();
		shrinkManifold(cams_[camIdx].position);
		logger_.endEventAndPrint("│ ├ shrinkManifold\t\t\t\t", true);
	}

	curConstraints_.clear();

	logger_.startEvent();
	int count = 0;
	for (auto id : pointsMovedIdx_) {
		if (moveVertex(id, camIdx)) {
			count++;
		}
	}
	pointsMovedIdx_.clear();
	logger_.endEventAndPrint("│ ├ Move vertices\t\t\t\t", true);

	logger_.startEvent();
	int oldNumVertices = (int) vecVertexHandles_.size();
	idxPointsForRayTracing_.clear();
	std::vector<int> insertedPointsIndex;

	for (auto newVisiblePointIndex : cams_[camIdx].newVisiblePoints) {

		if (points_[newVisiblePointIndex].new_) {

			if (utilities::distanceEucl(points_[newVisiblePointIndex].position, cams_[camIdx].position) < conf_.maxDistanceCamFeature) {

				vecDistanceWeight_.clear();

				/*	Try to insert the new point in the triangulation.
				 * 	When successful, a new vertex corresponding to the nwe point is created, some cells are removed from the triangulation and replaced by some others.
				 * 	The constraints that were intersecting the removed cells are added to curConstraints_
				 */
				if (insertNewPoint(points_[newVisiblePointIndex])) {
					idxPointsForRayTracing_.push_back(newVisiblePointIndex);
					points_[newVisiblePointIndex].new_ = false;
					insertedPointsIndex.push_back(newVisiblePointIndex);

					points_[newVisiblePointIndex].vertexHandle->info().setLastCam(camIdx);
					points_[newVisiblePointIndex].vertexHandle->info().setFirstCam(camIdx);

					if (conf_.enableSuboptimalPolicy) {
						std::vector<Delaunay3::Cell_handle> newCells;
						dt_.incident_cells(points_[newVisiblePointIndex].vertexHandle, std::inserter(newCells, newCells.begin()));
						updateDistanceAndWeights(newCells, vecDistanceWeight_);
					}

				}
			}

		} else {
			idxPointsForRayTracing_.push_back(newVisiblePointIndex);

			points_[newVisiblePointIndex].vertexHandle->info().setLastCam(camIdx);
			for (auto c : points_[newVisiblePointIndex].viewingCams) {

				if (c <= camIdx) {
					points_[newVisiblePointIndex].vertexHandle->info().addCam(camIdx);
				}
			}
		}

	}

	if (!conf_.enableSuboptimalPolicy) {

		// Fill setNewCells with all the cells incident to the new points.
		std::set<Delaunay3::Cell_handle> setNewCells;
		for (auto i : insertedPointsIndex) {
			dt_.incident_cells(points_[i].vertexHandle, std::inserter(setNewCells, setNewCells.begin()));
		}

		// Call rayTracing on all the constraints (rays) that were intersecting the removed cells, only mark new cells
		for (auto itConstraint : curConstraints_) {
			rayTracing(itConstraint.first, itConstraint.second, true);
		}

		// Mark as old the new cells //TODO why not marking as old the cells in rayTracing, and getting setNewCells out of insertNewPoint()
		for (std::set<Delaunay3::Cell_handle>::iterator itCell = setNewCells.begin(); itCell != setNewCells.end(); itCell++) {
			(*itCell)->info().markOld();
		}
	}
	logger_.endEventAndPrint("│ ├ Add new vertices\t\t\t\t", true);

}

void ManifoldMeshReconstructor::printIdxPointsForRayTracing() { //TODO remove
	for (auto p : idxPointsForRayTracing_) {
		std::cout << p << " ";
	}
	std::cout << std::endl << std::endl << std::endl;
}

void ManifoldMeshReconstructor::rayTracingFromCam(int idxCam) {
	logger_.startEvent();
	for (auto visPt : idxPointsForRayTracing_) {
		rayTracing(idxCam, visPt, false, true);
	}
	logger_.endEventAndPrint("│ ├ rayTracingFromCam\t\t\t\t", true);
}

void ManifoldMeshReconstructor::rayTracing(int idxCam, int idxPoint, bool bOnlyMarkNew, bool incrementCount) {

	Delaunay3::Cell_handle tetPrev;
	Delaunay3::Cell_handle tetCur;
	PointD3 source = points_[idxPoint].position;
	PointD3 target = cams_[idxCam].position;
	Segment constraint = Segment(source, target);

	std::vector<Delaunay3::Cell_handle> markedTetrahedra;

	RayReconstruction* ray = getRay(idxCam, idxPoint);

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

				markTetraedron(t, idxCam, idxPoint, markedTetrahedra, ray, incrementCount);
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
				markTetraedron(t, idxCam, idxPoint, markedTetrahedra, ray, incrementCount);
			}
			if (!bOnlyMarkNew || tetCur->info().isNew() || conf_.enableSuboptimalPolicy) {
				markTetraedron(tetCur, idxCam, idxPoint, markedTetrahedra, ray, incrementCount);
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

//						ray->valid = false;
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

//								ray->valid = false;
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
			markTetraedron(tetCur, idxCam, idxPoint, markedTetrahedra, ray, incrementCount);
		}

	}

	//if (!bOnlyMarkNew && (idxPoint % 10) == 0) outputM_->writeTetrahedraAndRayToOFF("output/ray_tracing/ray", idxCam, idxPoint, markedTetrahedra, constraint);

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
				std::cerr << "ManifoldMeshReconstructor::cellTraversalExitTest: rayTracing stopped prematurely to avoid a possible infinite loop" << std::endl;
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
		Delaunay3::Cell_handle & cell, const int camIndex, const int featureIndex, std::vector<Delaunay3::Cell_handle>& markedTetrahedra, RayReconstruction* ray,
		bool incrementCount) {

	if (incrementCount) {
		cell->info().incrementVoteCount(1); // t.n++

		freeSpaceTets_.push_back(cell);

		cell->info().incrementVoteCountProb(conf_.w_1);
		cell->info().incrementVoteCount(1.0);
		cell->info().addItersectionWeightedW1(ray);
		cell->info().setWeights(conf_.w_1, conf_.w_2, conf_.w_3);

		if (!conf_.enableSuboptimalPolicy) {
			cell->info().addIntersection(camIndex, featureIndex, conf_.w_1, points_[featureIndex].idVertex, points_, camsPositions_);
		}
	} else {
		cell->info().decrementVoteCount(1.0); // t.n++
		cell->info().decrementVoteCountProb(conf_.w_1); // t.n++

//		ray->valid = false;

		if (!conf_.enableSuboptimalPolicy) {
			cell->info().removeIntersection(camIndex, featureIndex, points_, camsPositions_);
		}
	}

//	markedTetrahedra.push_back(cell);
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
		Delaunay3::Cell_handle startingCell = freeSpaceTets_[freeSpaceTets_.size() - 1];
		manifoldManager_->regionGrowingBatch(startingCell);
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
	manifoldManager_->growSeveralAtOnce();

}

void ManifoldMeshReconstructor::saveManifold(const std::string filename) {
	outputM_->writeOFF(filename);
}

void ManifoldMeshReconstructor::saveBoundary(const std::string filename) {

	outputM_->writeBoundaryOFF(filename, manifoldManager_->getBoundaryCells());
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

void ManifoldMeshReconstructor::shrinkManifold(const PointD3 &camCenter) {

	manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);
//  saveBoundary("tempa.off");
//  saveManifold("tempa2.off");

	manifoldManager_->shrinkSeveralAtOnce();
//  saveBoundary("tempb.off");
//  saveManifold("tempb2.off");

	manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);
//  saveBoundary("tempc.off");
//  saveManifold("tempc2.off");

	manifoldManager_->shrinkSeveralAtOnce();
//  saveBoundary("tempb.off");
//  saveManifold("tempb2.off");

	manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);

}

void ManifoldMeshReconstructor::createSteinerPointGridAndBound() {
	std::vector<PointD3> vecPoint;
//temple
//  stepX_ = 1.000;
//  stepY_ = 1.000;
//  stepZ_ = 1.000;
//
//  float inX = -5;
//  float finX = 5;
//  float inY = -5;
//  float finY = 5;
//  float inZ = -5;
//  float finZ = 5;

//dino
//  stepX_ = 0.100;
//  stepY_ = 0.100;
//  stepZ_ = 0.100;
//
//  float inX = -2;
//  float finX = 2;
//  float inY = -1;
//  float finY = 1;
//  float inZ = -2;
//  float finZ = 2;

///castle - kitti
	stepX_ = 40.000;
	stepY_ = 40.000;
	stepZ_ = 40.000;

	float inX = -400;
	float finX = 400;
	float inY = -400;
	float finY = 400;
	float inZ = -400;
	float finZ = 400;

//  float inX = -300;
//  float finX = 300;
//  float inY = -300;
//  float finY = 300;
//  float inZ = -300;
//  float finZ = 300;

//
//  stepX_ = 550.0;
//  stepY_ = 550.0;
//  stepZ_ = 550.0;
//
//  float inX = -5000;
//  float finX = 5000;
//  float inY = -5000;
//  float finY = 5000;
//  float inZ = -5000;
//  float finZ = 5000;

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

bool ManifoldMeshReconstructor::insertNewPoint(PointReconstruction &point) {

	// if the point is marked to be moved but wasn't inserted, use the new position
	if(point.new_ && point.toBeMoved){
		point.toBeMoved = false;
		point.position = point.newPosition;
	}

	// Locate the new point
	Delaunay3::Locate_type lt;
	int li, lj;
	Delaunay3::Cell_handle c = dt_.locate(point.position, lt, li, lj);

	if(!point.new_) std::cerr << "ManifoldMeshReconstructor::insertNewPoint: point is not marked new!" << std::endl;

	// If there is already a vertex in the new point's position, do not insert it
	if (lt == Delaunay3::VERTEX) {
		return false;
	}

	// Insert in vecConflictCells the cells that conflict with the new point Q, and a facet on the boundary of this hole in f.
	std::vector<Delaunay3::Cell_handle> vecConflictCells;
	Delaunay3::Facet f;
	dt_.find_conflicts(point.position, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

	if (!conf_.enableSuboptimalPolicy) {
		// Add to curConstraints_ the constraints that intersect the conflicting cells
		for (auto it : vecConflictCells) {
			curConstraints_.insert(it->info().getIntersections().begin(), it->info().getIntersections().end());
		}
	} else {
		for (auto it : vecConflictCells) {
			PointD3 temp = CGAL::barycenter(it->vertex(0)->point(), 1.0, it->vertex(1)->point(), 1.0, it->vertex(2)->point(), 1.0, it->vertex(3)->point(), 1.0);
			vecDistanceWeight_.push_back(DistanceWeight(temp, (float) it->info().getVoteCountProb()));
		}
	}

	// Creates a new vertex by starring a hole. Delete all the cells (resp. facets) describing the hole vecConflictCells, creates a new vertex hndlQ, and for each facet (resp. edge) on the boundary of the hole f, creates a new cell (resp. facet) with hndlQ as vertex.
	Vertex3D_handle hndlQ = dt_.insert_in_hole(point.position, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);

	// Add the new vertex handle hndlQ to vecVertexHandles
	vecVertexHandles_.push_back(hndlQ);
	point.idVertex = vecVertexHandles_.size() - 1;
	point.vertexHandle = hndlQ;
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

int ManifoldMeshReconstructor::moveVertex_WHeuristic(int idxPoint, int idxCam) { //TODO use buffered point's position (newPosition, etc)

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

int ManifoldMeshReconstructor::moveVertex(int idxPoint, int idxCam) {

	std::set<Delaunay3::Cell_handle> setNewCells;
	Delaunay3::Vertex_handle hndlQ = points_[idxPoint].vertexHandle;
	SetConstraints setUnionedConstraints;

	if((points_[idxPoint].new_ && hndlQ != NULL) || (!points_[idxPoint].new_ && hndlQ == NULL)){
		std::cerr << "ManifoldMeshReconstructor::moveVertex: point "<< idxPoint <<" new xnor (hndlQ == NULL)" << std::endl;
	}

	// If the point isn't in the triangulation, do nothing
	if (hndlQ == NULL) {
		//std::cout << "ManifoldMeshReconstructor::moveVertex: null vertex handle for point " << idxPoint << std::endl;
		return 0; // point hasn't been inserted yet
	}

	if (hndlQ->point() != points_[idxPoint].position) {
		std::cerr << "ManifoldMeshReconstructor::moveVertex: inconsistent position between vertex handle and point position for point " << idxPoint << std::endl;
	}

	PointD3 initialPosition = hndlQ->point();
	PointD3 newPoint = points_[idxPoint].newPosition;
	PointD3 camPosition = cams_[idxCam].position;

	/* 	Let Br be the ball centered on the camera and r the parameter maxDistanceCamFeature.
	 * 	All the points must be in B if they are added to the triangulation to ensure that manifoldness is preserved.
	 *
	 * 	TODO manage all the cases where the point p is moved:
	 * 		· from inside B to inside B (as it is done)
	 * 		· from inside B to outside B (only remove from triangulation)
	 * 		· from outside B to inside B (insert)
	 * 		· from outside B to outside B (do nothing)
	 */
	if (utilities::distanceEucl(newPoint, camPosition) < conf_.maxDistanceCamFeature) {

		points_[idxPoint].position = points_[idxPoint].newPosition;
		points_[idxPoint].toBeMoved = false;

		//*********** Step 1: find the cells incident to the vertex to be removed*****************/
		std::set<Delaunay3::Cell_handle> setIncidentCells;
		dt_.incident_cells(hndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));

		for (auto itCell : setIncidentCells) {
			setUnionedConstraints.insert(itCell->info().getIntersections().begin(), itCell->info().getIntersections().end());
		}

		// Step 2:
		dt_.remove(hndlQ);

		// Step 3:
		// Locate the point
		Delaunay3::Locate_type lt;
		int li, lj;
		Delaunay3::Cell_handle c = dt_.locate(newPoint, lt, li, lj);
		if (lt == Delaunay3::VERTEX) {
			std::cerr << "Error in FreespaceDelaunayAlgorithm::moveVertex(): Attempted to move a vertex to an already existing vertex location" << std::endl;
			return false;
		}

		// Get the cells that conflict in a vector vecConflictCells, and a facet on the boundary of this hole in f.
		std::vector<Delaunay3::Cell_handle> vecConflictCells;
		Delaunay3::Facet f;
		dt_.find_conflicts(newPoint, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

		// Get the partitioned unioned constraint sets of all the cells in vecConflictCells.
		for (auto it : vecConflictCells) {
			setUnionedConstraints.insert(it->info().getIntersections().begin(), it->info().getIntersections().end());
		}

		// Step 4
		hndlQ = dt_.insert_in_hole(newPoint, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
		points_[idxPoint].vertexHandle = hndlQ;

		// Step 6
		for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
			if (itCell->info().isNew()) setNewCells.insert(itCell);
			// Linear search:
			for (auto itDelete = itCell->info().getIntersections().begin(); itDelete != itCell->info().getIntersections().end();) {
				if (itDelete->second == idxPoint) {
					// invalidates iterator, so careful about incrementing it:
					std::set<FSConstraint, FSConstraint::LtFSConstraint>::const_iterator itNext = itDelete;
					itNext++;
					itCell->info().removeIntersection(itDelete->first, itDelete->second, itDelete->vote, points_, camsPositions_);
					itCell->info().decrementVoteCount(1.0);
					itCell->info().decrementVoteCountProb(itDelete->vote);
					itDelete = itNext;

				} else itDelete++;
			}
		}

		// Step 7
		for (auto itConstraint : setUnionedConstraints) {
			if (itConstraint.second == idxPoint) {
				rayTracing(itConstraint.first, itConstraint.second, false);
			} else {
				rayTracing(itConstraint.first, itConstraint.second, true);
			}
		}

		for (auto itCell : setNewCells)
			itCell->info().markOld();

		return true;
	} else {
		return false;
	}
	return 1;

}

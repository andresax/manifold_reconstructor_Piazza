/*
 * ManifoldMeshReconstructor.h
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#ifndef MANIFOLDMESHRECONSTRUCTOR_H_
#define MANIFOLDMESHRECONSTRUCTOR_H_

#include <types_reconstructor.hpp>
#include <types_config.hpp>
#include <Logger.h>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <ManifoldManager.h>
#include <OutputCreator.h>
#include <fstream>
#include <iostream>
#include <Ray.hpp>
#include "FSConstraint.h"
#include <Chronometer.h>

typedef std::set<FSConstraint, FSConstraint::LtFSConstraint> SetConstraints;

/**
 * This class provides the API to manage the 2-manifold creation as explained in the
 * paper:
 *
 * Andrea Romanoni, Matteo Matteucci. Incremental Urban Manifold Reconstruction from a Sparse Edge-Points Cloud.
 * IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2015.
 *
 * The class builds a 3D Delaunay Triangulation, and keeps the information of the visibility updated incrementally.
 * In the same time it provides the functions to incrementally estimate a manifold mesh out of the triangulation.
 *
 * The moving point related functions (as explained in the paper: A. Romanoni, M. Matteucci.
 * Efficient moving point handling for incremental 3D manifold reconstruction.
 * International Conference on Image Analysis and Processing (ICIAP) 2015.) have not been tested in this library yet
 *
 *
 *
 */
class ManifoldMeshReconstructor {
public:
	ManifoldMeshReconstructor(ManifoldReconstructionConfig conf);
	virtual ~ManifoldMeshReconstructor();

//	void printWhatever();

//	void printIdxPointsForRayTracing();

	/*Functions to add informations about the points, the cameras and visibility
	 * Important Note: whenever you add here the points, they are not added automatically
	 * to the Delaunay Triangulation. to actually add these points you need to call the
	 * insertNewPointsFromCam function which manages conveniently the manifold update
	 *
	 **/
	void addPoint(float x, float y, float z);
//	int addPointWhere(float x, float y, float z);
	void addCameraCenter(float x, float y, float z);
	void addVisibilityPair(int camIdx, int pointIdx);
//	bool hasVisibilityPair(int camIdx, int pointIdx);
	void movePoint(int idxPoint, float x, float y, float z);
//	PointD3 movePointGetOld(int idxPoint, float x, float y, float z);
	void moveCamera(int idxCamera, float x, float y, float z);

	//incremental reconstruction

	void updateTriangulation();

	/**
	 * Insert all the points visible from camera idxCam. Before calling this
	 * function you have to add the camera, the points and the proper visibility information.
	 * If needed this function shrinks the manifold before point addition
	 */
//	void insertNewPointsFromCam(int idxCam, bool incremental = true);
	/**
	 * Perform ray tracing to update the visibility information, of the tetrahedra,inducted by visibility
	 * rays starting from camera idxCam*/
//	void rayTracingFromCam(int idxCam);
	/**
	 * Estimate the manifold mesh bootstrapping from the most visible tetrahedron of the boundary if the manifold is initialized, otherwise
	 * it boostraps from the most visible tetrahedron among all*/
//	void growManifold();
	/**
	 * Same as growManifold() but if the manifold is not initialized
	 * it boostraps from the tetrahedron containing camera idxCam*/
//	void growManifold(int idxCam);
	/**
	 * Same as growManifold() but grows the mesh with the general manifold test, i.e., less efficient than growManifold() but
	 * manages the genus change. It is usually called after a call to growManifold().
	 */
//	void growManifoldSev();

//	void clearLog();

	/*Saves the current manifold in OFF file format*/
	void saveManifold(const std::string filename);
	/*Saves the current boundary in OFF file format*/
	void saveBoundary(int i, int j);
	/*Saves the old manifold with points up to cam  idx in OFF file format*/
	void saveOldManifold(const std::string filename, int idx);
	/*Saves the submap manifold specified by the vector idx in OFF file format*/
	void saveOldManifold(const std::string filename, std::vector<int> idx);
	/*Saves in OFF file format the boundary mesh between tetrahedra labelled as free space and those labelled as matter */
	void saveFreespace(const std::string filename);

//	void setWeights(float w_1, float w_2, float w_3);

	std::ofstream timeStatsFile_;

private:
//	void shrinkManifold(const PointD3 &camCenter, int updatedCameraIndex);
//	void shrinkManifold2(std::set<PointD3> points);
	void shrinkManifold3(std::set<PointD3> points);

	void growManifold3(std::set<PointD3> points);

	void initSteinerPointGridAndBound();
	void updateSteinerPointGridAndBound();
	void updateSteinerGridTargetBounds(float x, float y, float z);

	/*create the initial grid of steiner points to avoid the infinite tetrahedra issue, while growing the mesh.
	 * The parameters inside these function are also useful to avoid the creation of too big tetrahedra wich may cause
	 * visual artifacts in the final mesh*/
//	void createSteinerPointGridAndBound();
	/*add new point to the Delaunay Triangulation*/
	bool insertVertex(PointReconstruction &points);
	/*mark a tetrahedron with a visibility rays, it updates the information stored in the tetrahedron*/
	void markCell(Delaunay3::Cell_handle& c, const int cameraIndex, const int pointIndex, std::vector<Delaunay3::Cell_handle>& path, bool onlyMarkNewCells);
	void unmarkCell(Delaunay3::Cell_handle& c, const int cameraIndex, const int pointIndex);
	void markRemovalCandidateRays(Vertex3D_handle& v, Delaunay3::Cell_handle& c, std::vector<Delaunay3::Cell_handle>& incidentCells);
	void removeRay(RayPath* r);
	void removeVisibilityPair(RayPath* r);

	//	void markTetraedron2(
//			Delaunay3::Cell_handle& cell, const int camIndex, const int featureIndex, std::vector<Delaunay3::Cell_handle>& path, bool incrementCount, bool onlyMarkNewCells);
//	void markTetraedron(
//			Delaunay3::Cell_handle& cell, const int camIndex, const int featureIndex, std::vector<Delaunay3::Cell_handle>& path, RayReconstruction* ray,
//			bool incrementCount = true);

	void rayTracingFromAllCam();

	/*Traces the ray from a camera to a point and update the weights, i.e., the visibility infromation, soterd in the traversed tetrahedra
	 * and in their neighbor. Here is implemented the Inverse Cone Heuristic (ICH)*/
//	void rayTracing(int idxCam, int idxPoint, bool bOnlyMarkNew = false, bool incrementCount = true);

//	void rayTracing2(int idxCam, int idxPoint, bool bOnlyMarkNew = false);

//	void rayTracing3(int idxCam, int idxPoint);

	void rayTracing4(int idxCam, int idxPoint, bool retrace = false);

	/* Inverse operation of rayTracing
	 * Unlike rayTracing that need to explore the terahedra from neighbour to neighbour, the ray path is known
	 */
//	void rayUntracing(RayPath* path);

	void rayUntracing2(RayPath* path);

	/* Complementary operation of rayTracing(idxCam, idxPoint, bOnlyMarkNew = true)
	 * Unlike rayTracing that need to explore the terahedra from neighbour to neighbour and performs its operations only on new cells,
	 * the set of new cells is known and rayRetracing start to trace the ray only in these cells
	 */
	void rayRetracing4(int idxCam, int idxPoint);
//	void rayRetracing3(int idxCam, int idxPoint);
//	void rayRetracing2(int idxCam, int idxPoint, std::set<Delaunay3::Cell_handle>& newCells);
//	void rayRetracing(int idxCam, int idxPoint, std::set<Delaunay3::Cell_handle>& newCells);

	void perHoleRayRetracing(std::set<Delaunay3::Cell_handle>& newCells);

//	/*Update the weights of the cellsToBeUpdates according to the values in the vecDistanceWeights. This implements the suboptimal policy*/
//	void updateDistanceAndWeights(std::vector<Delaunay3::Cell_handle> &cellsToBeUpdated, const std::vector<DistanceWeight> &vecDistanceWeight);

//	/*not tested yet*/
//	int moveVertex_WHeuristic(int idxPoint, int idxCam);
	/*not tested yet*/
	int moveVertex(int idxPoint);

	void removeVertex(int idxPoint);

	void moveCameraConstraints(int idxCam);
	/**test if the segment traverse two tetrahedra in facet f*/
//	bool cellTraversalExitTest(
//			int & f, int & fOld, const Delaunay3::Cell_handle& tetCur, const Delaunay3::Cell_handle &tetPrev, std::set<Delaunay3::Cell_handle>& visitedTetrahedra,
//			const Segment & constraint);

	bool nextCellOnRay(
			Delaunay3::Cell_handle& currentCell, Delaunay3::Cell_handle& previousCell, const Delaunay3::Cell_handle& targetCell,
			const Segment& constraint);

//	void addRay(int cameraId, int pointId);
//	RayReconstruction* getRay(int cameraId, int pointId);
//	std::set<RayReconstruction*> getRaysFromCamera(int cameraId);
//	std::set<RayReconstruction*> getRaysFromPoint(int pointId);

	RayPath* addRayPath(int cameraId, int pointId);
	RayPath* getRayPath(int cameraId, int pointId);
	void eraseRayPath(RayPath* r);
	std::set<RayPath*> getRayPathsFromCamera(int cameraId);
	std::set<RayPath*> getRayPathsFromPoint(int pointId);

//	float weightFunction(RayPath* r);

//	void getDegree1Neighbours(std::set<Delaunay3::Cell_handle>& path, std::set<Delaunay3::Cell_handle>& d1Neighbours);
//	void getDegree2Neighbours(std::set<Delaunay3::Cell_handle>& path, std::set<Delaunay3::Cell_handle>& d1Neighbours, std::set<Delaunay3::Cell_handle>& d2Neighbours);
//	void getDegree1And2Neighbours(
//			std::vector<Delaunay3::Cell_handle>& path, std::vector<Delaunay3::Cell_handle>& d1Neighbours, std::vector<Delaunay3::Cell_handle>& d2Neighbours, bool onlyMarkNewCells);

	int iterationCounter_ = 0;

	Delaunay3 dt_;
//	std::vector<Delaunay3::Cell_handle> freeSpaceTets_;
	std::set<Delaunay3::Cell_handle> newCells_;

	std::vector<CamReconstruction> cams_;
	std::vector<PointReconstruction> points_;
	std::vector<Vertex3D_handle> vecVertexHandles_;
	std::vector<glm::vec3> camsPositions_;
//	std::vector<int> curPointsVisible_;

//	std::vector<int> idxPointsForRayTracing_;
//	std::map<std::pair<int, int>, RayReconstruction*> rays_;
//	std::map<int, std::set<RayReconstruction*>> camerasRays_;
//	std::map<int, std::set<RayReconstruction*>> pointsRays_;

	std::map<std::pair<int, int>, RayPath*> rayPaths_;
	std::map<int, std::set<RayPath*>> camerasRayPaths_;
	std::map<int, std::set<RayPath*>> pointsRayPaths_;

//	std::set<FSConstraint, FSConstraint::LtFSConstraint> curConstraints_;
//	std::vector<DistanceWeight> vecDistanceWeight_;

	/*	raysToBeTraced_ contains all the rays <cameraIndex, pointIndex> that need to be traced by rayTracing.
	 *	See rayTracing
	 */
	std::set<std::pair<int, int>> raysToBeTraced_;

	/*	raysToBeRetraced_ contains all the rays <cameraIndex, pointIndex> that need to be retraced by rayRetracing.
	 * 	A ray is retraced by rayRetracing if it were intersecting some cells that were removed from the triangulation.
	 *	See rayRetracing
	 */
	std::set<std::pair<int, int>> raysToBeRetraced_;

	std::set<std::pair<int, int>> raysToBeUntraced_;

	std::set<std::pair<int, int>> raysCandidateToBeRemoved_;

	std::vector<int> pointsToBeRemovedIdx_;

	std::set<int> updatedCamerasIdx_;
	std::vector<int> pointsMovedIdx_;
	std::vector<int> movedCamerasIdx_;

	float l_, stepX_, stepY_, stepZ_;

	float sgMinX_, sgMaxX_, sgMinY_, sgMaxY_, sgMinZ_, sgMaxZ_;
	float sgCurrentMinX_, sgCurrentMaxX_, sgCurrentMinY_, sgCurrentMaxY_, sgCurrentMinZ_, sgCurrentMaxZ_;

	ManifoldReconstructionConfig conf_;
//	utilities::Logger logger_;
	ManifoldManager * manifoldManager_;
	OutputCreator *outputM_;
	std::ofstream fileOut_;

	long currentEnclosingVersion_ = 0;

	float timerShrinkTime_ = 0.0;
	float timerShrinkSeveralTime_ = 0.0;

	long rt2_CountNeighboursD1WeightUpdate_ = 0, rt2_CountNeighboursD2WeightUpdate_ = 0, rt2_SuccessfulCachedIndices = 0, rt2_TriedCachedIndices = 0;
	Chronometer rt2_ChronoUseless_, rt2_ChronoFirstCell_, rt2_ChronoCellTraversing_;
	Chronometer rt2_ChronoNeighboursD1Selection_, rt2_ChronoNeighboursD2Selection_, rt2_ChronoNeighboursD1WeightUpdate_, rt2_ChronoNeighboursD2WeightUpdate_;

	std::vector<Segment> movedPointsSegments_;

//	std::set<PointD3> lastShrinkPoints_;

};

#endif /* MANIFOLDMESHRECONSTRUCTOR_H_ */

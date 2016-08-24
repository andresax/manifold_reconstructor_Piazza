
#ifndef MANIFOLDMANAGER_H_
#define MANIFOLDMANAGER_H_

//#include <Mesh.h>
#include <tuple>
#include <fstream>
#include <iostream>
#include <types_reconstructor.hpp>
#include <types_config.hpp>
#include <OutputCreator.h>
#include <Logger.h>
#include <Chronometer.h>
/**
 * This class provides the basic tools to manage the actual manifold creation,
 * such as the region growing procedure, the manifoldness tests and the update of the
 * tetrahedra-based boundary
 *
 * */
class ManifoldManager {
public:
	ManifoldManager(Delaunay3& dt, bool inverseConic_, float probabTh_, ManifoldReconstructionConfig conf);
	virtual ~ManifoldManager();

	/*Tell on which delaunay triangulation the class instatiation will apply the grow/shrink algorithms*/
	void setDt(Delaunay3& dt) {
		dt_ = dt;
	}

	void setInverseConic(bool inverseConic) {
		inverseConic_ = inverseConic;
	}

	size_t getBoundarySize() {
		return boundaryCells_.size();
	}

	void shrinkManifold3(const std::set<PointD3>& points, const float& maxPointToPointDistance, long currentEnclosingVersion);
	void shrinkSeveralAtOnce3(const std::set<PointD3>& points, const float& maxPointToPointDistance, long currentEnclosingVersion);

	void shrinkManifold2(std::set<PointD3> points, const float& maxPointToPointDistance, long currentEnclosingVersion);
	void shrinkSeveralAtOnce2(std::set<PointD3> points, const float& maxPointToPointDistance, long currentEnclosingVersion);

	void regionGrowingBatch3(Delaunay3::Cell_handle& startingCell, const std::set<PointD3>& points);
	void regionGrowing3(const std::set<PointD3>& points);

	void growSeveralAtOnce3(const std::set<PointD3>& points);
	void growSeveralAtOnce2();

	/*Grow the manifold from the startingcell */
	void regionGrowingBatch(Delaunay3::Cell_handle& startingCell);

	/*Grow the manifold from the cell including the point in position firstCamPosition*/
	void regionGrowingBatch(PointD3 firstCamPosition);

	/* Grow the manifold  one tet-at-once incrementally, bootstrapping from the current boundary inside and outside the manifold */
	void regionGrowing();

	/*Grow the manifold several-tet-at-one in order to handle the genus change It bootstraps from the boundary between inside and outside the manifold*/
	void growSeveralAtOnce();

	/*shrink the manifold such that all the space inside the sphere with center in camPosition
	 and ray maxPointToPointDistance+maxPointToCamDistance is matter. In this way, our are
	 the able to add points seen from the cam in cam position
	 to the triangulation and retriangulate the space without breaking the delauna property*/
	void shrinkManifold(const PointD3 &camPosition, const float &maxPointToPointDistance, const float &maxPointToCamDistance);

	/*shrink the manifold several-tet-at-once in order to handle the genus change*/
	void shrinkSeveralAtOnce(const PointD3& camPosition, const float& maxPointToPointDistance, const float& maxPointToCamDistance);

	const std::vector<Delaunay3::Cell_handle>& getBoundaryCells() const {
		return boundaryCells_;
	}

	const std::map<index3, std::vector<Delaunay3::Cell_handle>>& getBoundaryCellsSpatialMap() const {
		return boundaryCellsSpatialMap_;
	}

private:

	void regionGrowingProcedure3(const std::set<PointD3>& points);
	void regionGrowingProcedure();

	/******************************************************/
	/**************Manifold check functions****************/
	/******************************************************/
	bool additionTest(Delaunay3::Cell_handle& i);
	bool subtractionTest(Delaunay3::Cell_handle& i);
	bool singleTetTest2(Delaunay3::Cell_handle& i);
	bool singleTetTest(Delaunay3::Cell_handle& i);
	bool checkManifoldness(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
	bool addSeveralAndCheckManifoldness2(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
	bool removeAndCheckManifoldness(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
	bool subSeveralAndCheckManifoldness2(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
	bool addAndCheckManifoldness(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
	bool isRegular(Delaunay3::Vertex_handle& v);
	bool isRegularProfiled(Delaunay3::Vertex_handle& v);

	/******************************************************/
	/************Boundary update functions*****************/
	/******************************************************/

	bool isInBoundary(Delaunay3::Cell_handle& cellToTest);
	bool isInBoundary(Delaunay3::Cell_handle& cellToTest, std::vector<int>& neighNotManifold);
	bool insertInBoundary(Delaunay3::Cell_handle& cellToTest);
	bool removeFromBoundary(Delaunay3::Cell_handle& cellToBeRemoved);
	void addTetAndUpdateBoundary2(Delaunay3::Cell_handle& cell);
	void addTetAndUpdateBoundary(Delaunay3::Cell_handle& cell);
	void subTetAndUpdateBoundary2(Delaunay3::Cell_handle& currentTet, std::vector<Delaunay3::Cell_handle>& newBoundaryTets);
	void subTetAndUpdateBoundary(Delaunay3::Cell_handle& currentTet, std::vector<Delaunay3::Cell_handle>& newBoundaryTets);

	bool isFreespace(Delaunay3::Cell_handle& cell);

	std::vector<Delaunay3::Cell_handle> boundaryCells_;
	std::map<index3, std::vector<Delaunay3::Cell_handle>> boundaryCellsSpatialMap_;


	Delaunay3& dt_;
	OutputCreator *outputM_;
	bool inverseConic_;
	float probabTh_;
	ManifoldReconstructionConfig conf_;

	std::ofstream fileOut_;

	utilities::Logger logger_;

	Chronometer functionProfileChronometer_isRegular_;
	long functionProfileCounter_isRegular_ = 0;
};

#endif /* MANIFOLDMANAGER_H_ */

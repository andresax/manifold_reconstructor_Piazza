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

	size_t getBoundarySize() {
		long long int bSize = 0;
		for (auto i_lbc : boundaryCellsSpatialMap_) {
			bSize += i_lbc.second.size();
		}

		return bSize;
	}

	/*shrink the manifold such that all the space inside the sphere with center in camPosition
	 and ray maxPointToPointDistance+maxPointToCamDistance is matter. In this way, our are
	 the able to add points seen from the cam in cam position
	 to the triangulation and retriangulate the space without breaking the delauna property*/
	void shrinkManifold3(const std::set<PointD3>& points, const float& maxPointToPointDistance, long currentEnclosingVersion);

	/*shrink the manifold several-tet-at-once in order to handle the genus change*/
	void shrinkSeveralAtOnce3(const std::set<PointD3>& points, const float& maxPointToPointDistance, long currentEnclosingVersion);

	/*Grow the manifold from the startingcell */
	void regionGrowingBatch3(Delaunay3::Cell_handle& startingCell, const std::set<PointD3>& points);

	/* Grow the manifold  one tet-at-once incrementally, bootstrapping from the current boundary inside and outside the manifold */
	void regionGrowing3(const std::set<PointD3>& points);

	/*Grow the manifold several-tet-at-one in order to handle the genus change It bootstraps from the boundary between inside and outside the manifold*/
	void growSeveralAtOnce3(const std::set<PointD3>& points);

	const std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> getBoundaryCells() const {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> boundaryCells_;

		for (auto i_lbc : boundaryCellsSpatialMap_) {
			boundaryCells_.insert(i_lbc.second.begin(), i_lbc.second.end());
		}

		return boundaryCells_;
	}

	const std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>>& getBoundaryCellsSpatialMap() const {
		return boundaryCellsSpatialMap_;
	}

	Chronometer chronoInsertInBoundary_, chronoRemoveFromBoundary_;

private:

	void regionGrowingProcedure3(const std::set<PointD3>& points);

	/******************************************************/
	/**************Manifold check functions****************/
	/******************************************************/
//	bool additionTest(Delaunay3::Cell_handle& i);
//	bool subtractionTest(Delaunay3::Cell_handle& i);
	bool singleTetTest2(Delaunay3::Cell_handle& i);
//	bool checkManifoldness(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
	bool addSeveralAndCheckManifoldness2(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
	bool subSeveralAndCheckManifoldness2(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
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
	void subTetAndUpdateBoundary2(Delaunay3::Cell_handle& currentTet, std::vector<Delaunay3::Cell_handle>& newBoundaryTets);

	bool isFreespace(Delaunay3::Cell_handle& cell);

//	std::vector<Delaunay3::Cell_handle> boundaryCells_;

// TODO unordered map?
	std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>> boundaryCellsSpatialMap_;

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

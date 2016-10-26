/*
 * OutputManager.h
 *
 *	TODO
 */

#ifndef OUTPUTMANAGER_H_
#define OUTPUTMANAGER_H_

#include <types_config.hpp>
#include <types_reconstructor.hpp>
#include <string>
#include <array>
#include <map>
#include <set>

class OutputManager {
public:

	OutputManager(
			std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>>& boundaryCellsSpatialMap, ManifoldReconstructionConfig conf);
	virtual ~OutputManager();

	void publishROSMesh();
	void writeMeshToOff(const std::string filename);

private:

//	std::array<int, 3> facetToTriangle(int facetIndex);
	std::array<Delaunay3::Vertex_handle, 3> faceIndexToVertices(Delaunay3::Cell_handle c, int faceIndex);

	ManifoldReconstructionConfig& conf_;
	std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>>& boundaryCellsSpatialMap_;
};

#endif /* OUTPUTMANAGER_H_ */

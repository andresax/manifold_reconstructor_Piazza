/*
 * OutputCreator.cpp
 *
 *	TODO
 */

#include <OutputManager.h>
#include <Chronometer.h>
#include <fstream>
#include <iostream>
#include <map>

OutputManager::OutputManager(std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>>& boundaryCellsSpatialMap, ManifoldReconstructionConfig conf) :
		boundaryCellsSpatialMap_(boundaryCellsSpatialMap), conf_(conf) {

}

OutputManager::~OutputManager() {
}

void OutputManager::publishROSMesh() {
	std::vector<PointD3> points;
	std::vector<index3> triangles;

	std::map<Delaunay3::Vertex_handle, int> vertexHandleToIndex;
	std::vector<Delaunay3::Vertex_handle> vertexHandles;

	int facetToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
	{ 0, 2, 3 },	// facetIndex : 1
	{ 3, 1, 0 },	// facetIndex : 2
	{ 0, 1, 2 } 	// facetIndex : 3
	};

	// Initialize points and tris as empty:
	if (!points.empty()) points.clear();
	if (!triangles.empty()) triangles.clear();

	// Populate the list of points, the list of vertex handles, and
	// the associative maps from vertex handle to vertex index).
	for (auto i_lbc : boundaryCellsSpatialMap_) { // For each spatially mapped container

		for (auto c : i_lbc.second) { // For each boundary cell in the container

			for (int faceIndex = 0; faceIndex < 4; faceIndex++) { // For each face in the cell

				// If the face is a boundary face (between the boundary cell and a non manifold cell)
				if (!c->info().iskeptManifold()) {

					std::array<Delaunay3::Vertex_handle, 3> triangleVertices = faceIndexToVertices(c, faceIndex);

					// Add the face's vertices to the vertices list (if they aren't already in it)
					for (auto v : triangleVertices) {
						if (!vertexHandleToIndex.count(v)) {
							points.push_back(v->point());
							vertexHandles.push_back(v);
							vertexHandleToIndex[v] = points.size() - 1;
						}
					}

					// Add the face's triangle to the triangles list
					triangles.push_back(index3(vertexHandleToIndex[triangleVertices[0]], vertexHandleToIndex[triangleVertices[1]], vertexHandleToIndex[triangleVertices[2]]));

				}
			}
		}
	}

}

void OutputManager::writeMeshToOff(const std::string filename) {
	Chronometer chrono1, chrono2;
	chrono1.start();

	std::ofstream outfile;
	std::vector<PointD3> points;
	std::set<index3> triangles;
	std::map<Delaunay3::Vertex_handle, int> vertexHandleToIndex;
	std::vector<Delaunay3::Vertex_handle> vertexHandles;

	int facetToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
	{ 0, 2, 3 },	// facetIndex : 1
	{ 3, 1, 0 },	// facetIndex : 2
	{ 0, 1, 2 } 	// facetIndex : 3
	};

	outfile.open(filename.c_str());
	if (!outfile.is_open()) {
		std::cerr << "Unable to open file: " << filename << std::endl;
		return;
	}

	std::cout << "boundaryCellsSpatialMap_.size()\t\t " << boundaryCellsSpatialMap_.size() << std::endl;

	// Populate the list of points, the list of vertex handles, and
	// the associative maps from vertex handle to vertex index).
	for (auto i_lbc : boundaryCellsSpatialMap_) { // For each spatially mapped container

		for (auto c : i_lbc.second) { // For each boundary cell in the container

			for (int faceIndex = 0; faceIndex < 4; faceIndex++) { // For each face in the cell

				// If the face is a boundary face (between the boundary cell and a non manifold cell)
				if (!c->neighbor(faceIndex)->info().iskeptManifold()) {

					std::array<Delaunay3::Vertex_handle, 3> triangleVertices = faceIndexToVertices(c, faceIndex);

//					bool containsSteinerPoint = false;
//					for(auto v : triangleVertices) if(v->info().getPointId() == -1) containsSteinerPoint = true;
//					if(containsSteinerPoint) continue;

					// Add the face's vertices to the vertices list (if they aren't already in it)
					for (auto v : triangleVertices) {
						if (!vertexHandleToIndex.count(v)) {
							points.push_back(v->point());
							vertexHandles.push_back(v);
							vertexHandleToIndex[v] = points.size() - 1;
						}
					}

					// Add the face's triangle to the triangles list
					triangles.insert(index3(vertexHandleToIndex[triangleVertices[0]], vertexHandleToIndex[triangleVertices[1]], vertexHandleToIndex[triangleVertices[2]]));

				}
			}
		}
	}

	chrono1.stop();
	chrono2.start();

	outfile << "OFF" << std::endl << points.size() << " " << triangles.size() << " 0" << std::endl;

	for (auto p : points)
		outfile << static_cast<float>(p.x()) << " " << static_cast<float>(p.y()) << " " << static_cast<float>(p.z()) << " " << std::endl;

	for (auto t : triangles)
		outfile << "3 " << t.i << " " << t.j << " " << t.k << std::endl;

	outfile.close();

	chrono2.stop();
	std::cout << "writeMeshToOff collect:\t\t" << chrono1.getSeconds() << " s" << std::endl;
	std::cout << "writeMeshToOff write  :\t\t" << chrono2.getSeconds() << " s" << std::endl;

}

//
//std::array<int, 3> OutputManager::facetToTriangle(int facetIndex) {
//	int vertices[4];
//
//	if (facetIndex == 0) {
//		// Vertex handle order: 3 2 1
//		vertices[0] = 3;
//		vertices[1] = 2;
//		vertices[2] = 1;
//	} else if (facetIndex == 1) {
//		// Vertex handle order: 0 2 3
//		vertices[0] = 0;
//		vertices[1] = 2;
//		vertices[2] = 3;
//	} else if (facetIndex == 2) {
//		// Vertex handle order: 3 1 0
//		vertices[0] = 3;
//		vertices[1] = 1;
//		vertices[2] = 0;
//	} else { // f->second == 3
//		// Vertex handle order: 0 1 2
//		vertices[0] = 0;
//		vertices[1] = 1;
//		vertices[2] = 2;
//	}
//
//	return vertices;
//}

std::array<Delaunay3::Vertex_handle, 3> OutputManager::faceIndexToVertices(Delaunay3::Cell_handle c, int faceIndex) {
	std::array<Delaunay3::Vertex_handle, 3> vertices;

	int faceToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
	{ 0, 2, 3 },	// faceIndex : 1
	{ 3, 1, 0 },	// faceIndex : 2
	{ 0, 1, 2 } 	// faceIndex : 3
	};

	for (int i = 0; i < 3; i++)
		vertices[i] = c->vertex(faceToTriangleMatrix[faceIndex][i]);

	return vertices;
}


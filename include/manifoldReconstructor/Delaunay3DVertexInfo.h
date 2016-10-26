/*
 * Delaunay3DVertexInfo.h
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#ifndef DELAUNAY3DVERTEXINFO_H_
#define DELAUNAY3DVERTEXINFO_H_

#include <vector>


class Delaunay3DVertexInfo {

public:
	Delaunay3DVertexInfo();
	virtual ~Delaunay3DVertexInfo();

	int getPointId() const {
		return pointId_;
	}

	void setPointId(int pointId) {
		pointId_ = pointId;
	}

	int getLastCam() const {
		return lastCam_;
	}

	void setLastCam(int lastCam) {
		lastCam_ = lastCam;
	}

	void addCam(int idx) {
		listViewingCam_.push_back(idx);
	}

	const std::vector<int>& getListViewingCam() const {
		return listViewingCam_;
	}

//	int getFirstCam() const {
//		return firstCam_;
//	}

//	void setFirstCam(int firstCam) {
//		firstCam_ = firstCam;
//	}

private:
//	int used_;
//	bool notUsed_;
	int lastCam_ = -1;
	int pointId_ = -1;
//	int firstCam_;
	std::vector<int> listViewingCam_;
};

#endif /* DELAUNAY3DVERTEXINFO_H_ */

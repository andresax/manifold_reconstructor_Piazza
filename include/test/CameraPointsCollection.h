/*
 * CameraPointsCollection.h
 *
 *  Created on: 25 Mar 2016
 *      Author: enrico
 */

#ifndef INCLUDE_TEST_CAMERAPOINTSCOLLECTION_H_
#define INCLUDE_TEST_CAMERAPOINTSCOLLECTION_H_

#include <types_reconstructor.hpp>
#include <map>
#include <SfMData.h>

class CameraPointsCollection {
public:
	CameraPointsCollection();
	virtual ~CameraPointsCollection();

	int hasCamera(long unsigned int);
	void addCamera(CameraType*);
	CameraType* getCamera(long unsigned int);
	int numCameras();

	int hasPoint(long unsigned int);
	void addPoint(PointType*);
	PointType* getPoint(long unsigned int);
	int numPoints();

	void addVisibility(long unsigned int, long unsigned int);
	void addVisibility(CameraType* camera, PointType* point);

	void clear();

	const std::map<unsigned long int, CameraType*>& getCameras() const {
		return cameras_;
	}

	const std::map<unsigned long int, PointType*>& getPoints() const {
		return points_;
	}

	void computeSfmData() {
//		SfMData d;

//		int maxPointId = 508441;//(points_.end()--)->first;
//
//		sfm_data_.numCameras_ = (cameras_.end()--)->first;
//		sfm_data_.numPoints_ = maxPointId;

		sfm_data_.numCameras_ = cameras_.size();
		sfm_data_.numPoints_ = points_.size();

		sfm_data_.camerasList_.assign(sfm_data_.numCameras_, CameraType());
		sfm_data_.points_.assign(sfm_data_.numPoints_, glm::vec3());
		sfm_data_.pointsVisibleFromCamN_.assign(sfm_data_.numCameras_, std::vector<int>());
		sfm_data_.camViewingPointN_.assign(sfm_data_.numPoints_, std::vector<int>());

//		std::cout << "points_.size(): " << sfm_data_.points_.size() << std::endl;

		int nextCameraIdReconstruction = 0;
		int nextPointIdReconstruction = 0;

		for (auto i_p : points_) {

			i_p.second->idReconstruction = nextPointIdReconstruction++;
			sfm_data_.points_[i_p.second->idReconstruction] = i_p.second->position;

//			std::cout << "i_p.first: " << i_p.first << "\t i_p.second->idReconstruction: " << i_p.second->idReconstruction << "\t sfm_data_.points_.size(): " << sfm_data_.points_.size() << std::endl;
		}

		for (auto i_c : cameras_) {
//			CameraType c;
//
//			c.center = i_c.second->center;
//			c.idCam = i_c.second->idCam;

			i_c.second->idReconstruction = nextCameraIdReconstruction++;

			sfm_data_.camerasList_[i_c.second->idReconstruction] = *(i_c.second);

			for (auto p : i_c.second->visiblePointsT)
				sfm_data_.pointsVisibleFromCamN_[i_c.second->idReconstruction].push_back(p->idReconstruction);
		}

		for (auto i_p : points_) {
//			std::cout << "i_p.first: " << i_p.first << "\t sfm_data_.camViewingPointN_.size(): " << sfm_data_.camViewingPointN_.size() << "\t sfm_data_.points_.size(): " << sfm_data_.points_.size() << std::endl;
//			std::cout << "\t\t " << i_p.second->viewingCams.size() << "\tc->idCam: ";

			for (auto c : i_p.second->viewingCams) {
//				std::cout << c->idCam << " ";

				sfm_data_.camViewingPointN_[i_p.second->idReconstruction].push_back(c->idReconstruction);
			}
//			std::cout << std::endl;
		}

//		return d;
//	    return sfm_data_;
	}

	const SfMData& getSfmData() const {
		return sfm_data_;
	}

private:
	std::map<long unsigned int, PointType*> points_;
	std::map<long unsigned int, CameraType*> cameras_;

	SfMData sfm_data_;

// std::vector<std::vector<int> > camViewingPointN_;
// std::vector<std::vector<int> > pointsVisibleFromCamN_;
// std::vector<std::vector<glm::vec2> > point2DoncamViewingPoint_;
};

#endif /* INCLUDE_TEST_CAMERAPOINTSCOLLECTION_H_ */

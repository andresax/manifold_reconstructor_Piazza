/*
 * ReconstructFromSLAMData.h
 *
 *  Created on: 27 mar 2016
 *      Author: enrico
 */

#ifndef RECONSTRUCTFROMSFMDATA_H_
#define RECONSTRUCTFROMSFMDATA_H_

#include <types_config.hpp>
#include <ManifoldMeshReconstructor.h>
#include <OutputManager.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <SfMData.h>
//#include <CameraPointsCollection.h>

class ReconstructFromSLAMData {
public:
	ReconstructFromSLAMData(ManifoldReconstructionConfig& manifConf);
	virtual ~ReconstructFromSLAMData();

	void addCamera(CameraType* newCamera);
	void updateManifold();
	void saveManifold(std::string namePrefix, std::string nameSuffix);

	void overwriteFocalY(float f);

	void setExpectedTotalIterationsNumber(int n);
	void insertStatValue(float v);

	OutputManager* getOutputManager() {
		return manifRec_->getOutputManager();
	}

	// Iteration counter. Incremented every time a camera is added
	int iterationCount;

private:
	//void outlierFiltering(std::vector<bool>& inliers);
	int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint);

	int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &jacobian, cv::Mat &hessian);

	ManifoldReconstructionConfig& manifConf_;
	ManifoldMeshReconstructor* manifRec_;
	utilities::Logger logger_;

	std::set<CameraType*> rayTracingSet_;
	std::set<CameraType*> insertNewPointsFromCamSet_;

	// Camera's and point's index used in ManifoldMeshReconstructor must be set incrementally every time the camera or point is effectivly added to ManifoldMeshReconstructor
	int cameraNextId, pointNextId;

	// Flag representing whether the manifold was updated since the last time it was saved. Set to true by updateManifold, to false by saveManifold
	bool manifoldUpdatedSinceSave_;

	int expectedTotalIterationsNumber_ = 0;
};

#endif /* RECONSTRUCTFROMSLAMDATA_H_ */

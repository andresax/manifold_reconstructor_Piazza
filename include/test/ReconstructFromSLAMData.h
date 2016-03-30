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
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <SfMData.h>
//#include <CameraPointsCollection.h>


class ReconstructFromSLAMData {
public:
  ReconstructFromSLAMData(const CameraPointsCollection& cp_data, ManifoldReconstructionConfig& manifConf);
  virtual ~ReconstructFromSLAMData();
  void increment(CameraType* newCamera);
  void saveManifold();
  void overwriteFocalY(float f);

  int iterationCount;

private:
  //void outlierFiltering(std::vector<bool>& inliers);
  int GaussNewton(const std::vector<cv::Mat> &cameras,
      const std::vector<cv::Point2f> &points,cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint);

  int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint,
      cv::Mat &jacobian, cv::Mat &hessian);


  CameraPointsCollection cp_data_;
  ManifoldReconstructionConfig manifConf_;
  ManifoldMeshReconstructor* manifRec_;
  utilities::Logger logger_;

  int cameraNextId, pointNextId;
};

#endif /* RECONSTRUCTFROMSLAMDATA_H_ */

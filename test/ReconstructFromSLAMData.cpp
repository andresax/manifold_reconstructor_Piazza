/*
 * ReconstructFromSLAMData.cpp
 *
 *  Created on: 27 mar 2016
 *      Author: enrico
 */

#include <CameraPointsCollection.h>
#include <ReconstructFromSLAMData.h>
#include <utilities.hpp>

ReconstructFromSLAMData::ReconstructFromSLAMData(const CameraPointsCollection& cp_data, ManifoldReconstructionConfig& manifConf) {
  cp_data_ = cp_data;
  manifConf_ = manifConf;
  manifRec_ = new ManifoldMeshReconstructor(manifConf_);
  utilities::Logger logger_;

  cameraNextId = 0; pointNextId = 0;

  iterationCount = 0;

}

ReconstructFromSLAMData::~ReconstructFromSLAMData() {
}

void ReconstructFromSLAMData::increment(CameraType* newCamera) {

  std::cout << "ReconstructFromSLAMData::increment iteration " << iterationCount << std::endl;

  //std::cout << "start outlier filtering" << std::endl;
  //std::vector<bool> inliers;
  //outlierFiltering(inliers); // comment this; EP

  // Add the new camera to ManifoldMeshReconstructor
  //std::cout << "add newCamera to manifRec_" << std::endl;
  glm::vec3 center = newCamera->center;
  manifRec_->addCameraCenter(center.x, center.y, center.z);

  // generate the ManifoldMeshReconstructor's index if the camera hasn't got one already
  if(newCamera->idReconstruction<0) newCamera->idReconstruction = cameraNextId++;

  std::cout << "ADD cam " << newCamera->idCam << " (" <<  newCamera->idReconstruction << ")"
   //       << ": " << center.x << ", " << center.y << ", " << center.z
          << std::endl;


  // Add the points associated to the new camera to ManifoldMeshReconstructor
  int countIgnoredPoints = 0, countUpdatedPoints = 0, countAddedPoints = 0;
  for (auto const &p : newCamera->visiblePointsT) {
    // Only consider points that were observed in many frames //TODO magic doesn't exist
    if(p->getNunmberObservation() < 2){ //TODO use selection heuristic
      //std::cout << "IGNORE point \t" << p->idPoint << "\t (recId:" <<  p->idReconstruction << "),\tobs: " << p->getNunmberObservation() << std::endl;
      countIgnoredPoints++;
      continue;
    }

    glm::vec3 position = p->position;

    if(p->idReconstruction<0){

      // generate the ManifoldMeshReconstructor's index if the point hasn't got one already
      p->idReconstruction = pointNextId++;

      // If the point didn't have this index then it is a new point for sure
      manifRec_->addPoint(position.x, position.y, position.z);
      countAddedPoints ++;

//      std::cout << "ADD    point \t" << p->idPoint << "\t (recId:" <<  p->idReconstruction << "),\tobs: " << p->getNunmberObservation()
//         // << ": " << position.x << ", " << position.y << ", " << position.z
//          << std::endl;

    }else{

      // The point was already added to ManifoldMeshReconstructor, so it is only updated
      manifRec_->movePoint(p->idReconstruction, position.x, position.y, position.z);
      countUpdatedPoints++;
//      std::cout << "UPDATE point \t" << p->idPoint << "\t (recId:" <<  p->idReconstruction << "),\tobs: " << p->getNunmberObservation()
//         // << ": " << position.x << ", " << position.y << ", " << position.z
//          << std::endl;
    }

  }

  std::cout
      <<   "A:   " << countAddedPoints
      << "\tU: " << countUpdatedPoints
      << "\tI: " << countIgnoredPoints
      << std::endl;

  // Add visibility pairs to manifRec_
  //std::cout << "add visibility pairs to manifRec_" << std::endl;
  for(auto & p : newCamera->visiblePointsT){

    //TODO inliers
    //TODO also add visibility with old cameras?
    if(newCamera->idReconstruction>=0 && p->idReconstruction>=0){
      manifRec_->addVisibilityPair(newCamera->idReconstruction, p->idReconstruction);

      //std::cout << "ADD Visibility Pair       : cam " << newCamera->idCam << "(recId:" <<  newCamera->idReconstruction << "), point " << p->idPoint << " (recId:" <<  p->idReconstruction << ")" << std::endl;
    }else{
      std::cout << "ADD Visibility Pair FAILED: cam " << newCamera->idCam << "(recId:" <<  newCamera->idReconstruction << "), point " << p->idPoint << " (recId:" <<  p->idReconstruction << ")" << std::endl;
    }

  }


  if(iterationCount > 4){

    //TODO actually manage the case of newCamera has too few or no points associated

    // Tell ManifoldMeshReconstructor to shrink the manifold
    //std::cout << "insertNewPointsFromCam( " << newCamera->idReconstruction << ", true )" << std::endl;
    logger_.startEvent();
    manifRec_->insertNewPointsFromCam(newCamera->idReconstruction, true);
    logger_.endEventAndPrint("insertNewPointsFromCam\t\t\t\t", true);

    // Tell ManifoldMeshReconstructor to do its stuff
    //std::cout << "rayTracingFromCam( " << newCamera->idReconstruction << " )" << std::endl;
    logger_.startEvent();
    manifRec_->rayTracingFromCam(newCamera->idReconstruction);
    logger_.endEventAndPrint("rayTracingFromCam\t\t\t\t", true);

    // Tell ManifoldMeshReconstructor to grow back the manifold
    logger_.startEvent();
    std::cout << "growManifold" << std::endl;
    manifRec_->growManifold();
    logger_.endEventAndPrint("growManifold\t\t\t\t\t", true);

    logger_.startEvent();
    std::cout << "growManifoldSev" << std::endl;
    manifRec_->growManifoldSev();
    logger_.endEventAndPrint("growManifoldSev\t\t\t\t\t", true);

    logger_.startEvent();
    std::cout << "growManifold" << std::endl;
    manifRec_->growManifold();
    logger_.endEventAndPrint("growManifold\t\t\t\t\t", true);
  }
  iterationCount++;
}


void ReconstructFromSLAMData::saveManifold(std::string namePrefix, std::string nameSuffix){
  logger_.startEvent();
  std::ostringstream nameManifold; nameManifold << namePrefix << "manifold_" << nameSuffix << ".off";
  std::cout << "saving " << nameManifold.str() << std::endl;
  manifRec_->saveManifold(nameManifold.str());
  logger_.endEventAndPrint("save manifold\t\t\t\t\t", true);

//
//  logger_.startEvent();
//  std::ostringstream nameFreespace; nameFreespace << namePrefix << "free_space_manifold_" << nameSuffix << ".off";
//  std::cout << "saving " << nameFreespace.str() << std::endl;
//  manifRec_->saveFreespace("FreeFinal.off");
//  logger_.endEventAndPrint("save free_space_manifold\t\t\t", true);
//
//  std::vector<int> age;
//  for (int cur = 0; cur < cp_data_.numCameras(); cur++) age.push_back(cur);
//
//  logger_.startEvent();
//  std::ostringstream nameManifoldWithoutSteinerPoints; nameManifoldWithoutSteinerPoints << namePrefix << "manifold_without_steiner_points_" << nameSuffix << ".off";
//  std::cout << "saving " << nameManifoldWithoutSteinerPoints.str().str() << std::endl;
//  manifRec_->saveOldManifold(nameManifoldWithoutSteinerPoints.str(), age);
//  logger_.endEventAndPrint("save manifold_without_steiner_points\t", true);

}


void ReconstructFromSLAMData::overwriteFocalY(float f) {
  for (auto const &kvCamera : cp_data_.getCameras()) {
    kvCamera.second->intrinsics[1][1] = f;
  }

}

//
//void ReconstructFromSLAMData::outlierFiltering(std::vector<bool>& inliers) {
//
//  inliers.assign(sfm_data_.points_.size(), false); // assign all true EP
//  std::vector<cv::Mat> cameras;
//  std::vector<cv::Point2f> measures;
//  cv::Point3f init3Dpoint;
//  cv::Point3f optimizedPoint;
//
//  for (int curPt3D = 0; curPt3D < sfm_data_.points_.size(); curPt3D++) {
//    cameras.clear();
//    cameras.assign(sfm_data_.camViewingPointN_[curPt3D].size(), cv::Mat());
//    for (int curC = 0; curC < sfm_data_.camViewingPointN_[curPt3D].size(); curC++) {
//      cameras[curC] = cv::Mat(4, 4, CV_32F);
//      for (int row = 0; row < 4; row++) {
//        for (int col = 0; col < 4; col++) {
//          cameras[curC].at<float>(row, col) = sfm_data_.camerasList_[sfm_data_.camViewingPointN_[curPt3D][curC]].cameraMatrix[row][col];
//        }
//      }
//
//    }
//
//    measures.clear();
//    measures.assign(sfm_data_.point2DoncamViewingPoint_[curPt3D].size(), cv::Point2f());
//    for (int curMeas = 0; curMeas < sfm_data_.point2DoncamViewingPoint_[curPt3D].size(); curMeas++) {
//      measures[curMeas].x = sfm_data_.point2DoncamViewingPoint_[curPt3D][curMeas].x;
//      measures[curMeas].y = sfm_data_.point2DoncamViewingPoint_[curPt3D][curMeas].y;
//    }
//
//    init3Dpoint.x = sfm_data_.points_[curPt3D].x;
//    init3Dpoint.y = sfm_data_.points_[curPt3D].y;
//    init3Dpoint.z = sfm_data_.points_[curPt3D].z;
//
//    if (GaussNewton(cameras, measures, init3Dpoint, optimizedPoint) != -1) {
//
//      sfm_data_.points_[curPt3D].x = optimizedPoint.x;
//      sfm_data_.points_[curPt3D].y = optimizedPoint.y;
//      sfm_data_.points_[curPt3D].z = optimizedPoint.z;
//      inliers[curPt3D] = true;
//    }
//  }
//
//}

int ReconstructFromSLAMData::GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint,
    cv::Point3f &optimizedPoint) {
  int numMeasures = points.size();
  cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_32F);

  cv::Mat curEstimate3DPoint = cv::Mat(3, 1, CV_32F);
  cv::Mat curEstimate3DPointH = cv::Mat(4, 1, CV_32F);
  curEstimate3DPoint.at<float>(0, 0) = init3Dpoint.x;
  curEstimate3DPoint.at<float>(1, 0) = init3Dpoint.y;
  curEstimate3DPoint.at<float>(2, 0) = init3Dpoint.z;

  cv::Mat J, H;
  float last_mse = 0;
  int i;
  for (i = 0; i < 30; i++) {

    float mse = 0;
    //compute residuals
    for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
      curEstimate3DPointH.at<float>(0, 0) = curEstimate3DPoint.at<float>(0, 0);
      curEstimate3DPointH.at<float>(1, 0) = curEstimate3DPoint.at<float>(1, 0);
      curEstimate3DPointH.at<float>(2, 0) = curEstimate3DPoint.at<float>(2, 0);
      curEstimate3DPointH.at<float>(3, 0) = 1.0;
      cv::Mat cur2DpositionH = cameras[curMeas] * curEstimate3DPointH;

      r.at<float>(2 * curMeas, 0) = ((points[curMeas].x - cur2DpositionH.at<float>(0, 0) / cur2DpositionH.at<float>(2, 0)));
      mse += r.at<float>(2 * curMeas, 0) * r.at<float>(2 * curMeas, 0);

      r.at<float>(2 * curMeas + 1, 0) = ((points[curMeas].y - cur2DpositionH.at<float>(1, 0) / cur2DpositionH.at<float>(2, 0)));
      mse += r.at<float>(2 * curMeas + 1, 0) * r.at<float>(2 * curMeas + 1, 0);
#ifdef DEBUG_OPTIMIZATION_VERBOSE
      if(i==0) {
        std::cout<<"CurMeas: "<<curMeas<<std::endl<<"curEstimate3DPointH="<< curEstimate3DPointH.t()<<std::endl;
        std::cout<<"CurCam"<<cameras[curMeas]<<std::endl;
        std::cout<<"cur2DpositionH: "<<cur2DpositionH.at<float>(0, 0)/cur2DpositionH.at<float>(2, 0)<<", "<<cur2DpositionH.at<float>(1, 0)/cur2DpositionH.at<float>(2, 0)<<std::endl;
        std::cout<<"points[curMeas]: "<<points[curMeas]<<std::endl;
        std::cout<<"residual on x: "<<r.at<float>(2 * curMeas, 0)<<std::endl;
        std::cout<<"residual on y: "<<r.at<float>(2 * curMeas + 1 , 0)<<std::endl;
        std::cout<<std::endl;}
#endif
    }
    //mse = sqrt(mse)/(numMeasures*2);

    if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000000005) {
      break;
    }
    last_mse = mse / (numMeasures * 2);

    if (point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1) {
      //std::cout<<"It= "<<" point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1 "<<std::endl;
      return -1;
    }
#ifdef DEBUG_OPTIMIZATION_VERBOSE
    std::cout<<"J: "<<J<<std::endl;
    std::cout<<"H: "<<H<<std::endl;
#endif

    curEstimate3DPoint += H.inv() * J.t() * r;

#ifdef DEBUG_OPTIMIZATION
    std::cout << "It= " << i << " last_mse " << last_mse << std::endl;
#endif
  }

  if (last_mse < 0.25/*3 pixels*/) {
    optimizedPoint.x = curEstimate3DPoint.at<float>(0, 0);
    optimizedPoint.y = curEstimate3DPoint.at<float>(1, 0);
    optimizedPoint.z = curEstimate3DPoint.at<float>(2, 0);
    return 1;
  } else {
    return -1;
  }
}

int ReconstructFromSLAMData::point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian) {

  int numMeasures = cameras.size();
  cv::Mat cur3DPointHomog = cv::Mat(4, 1, CV_32F);
  ;
  cur3DPointHomog.at<float>(0, 0) = cur3Dpoint.at<float>(0, 0);
  cur3DPointHomog.at<float>(1, 0) = cur3Dpoint.at<float>(1, 0);
  cur3DPointHomog.at<float>(2, 0) = cur3Dpoint.at<float>(2, 0);
  cur3DPointHomog.at<float>(3, 0) = 1.0;

  J = cv::Mat(2 * numMeasures, 3, CV_32FC1);  //2 rows for each point: one for x, the other for y
  hessian = cv::Mat(3, 3, CV_32FC1);

  /*std::cout << "gdevre" <<std::endl;
   std::cout << cameras[0] <<std::endl;*/
  for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
    cv::Mat curReproj = cameras[curMeas] * cur3DPointHomog;
    float xH = curReproj.at<float>(0, 0);
    float yH = curReproj.at<float>(1, 0);
    float zH = curReproj.at<float>(2, 0);
    float p00 = cameras[curMeas].at<float>(0, 0);
    float p01 = cameras[curMeas].at<float>(0, 1);
    float p02 = cameras[curMeas].at<float>(0, 2);
    float p10 = cameras[curMeas].at<float>(1, 0);
    float p11 = cameras[curMeas].at<float>(1, 1);
    float p12 = cameras[curMeas].at<float>(1, 2);
    float p20 = cameras[curMeas].at<float>(2, 0);
    float p21 = cameras[curMeas].at<float>(2, 1);
    float p22 = cameras[curMeas].at<float>(2, 2);

    //d(P*X3D)/dX
    J.at<float>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

    //d(P*X3D)/dY
    J.at<float>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

    //d(P*X3D)/dZ
    J.at<float>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
  }

  hessian = J.t() * J;
  float d;
  d = cv::determinant(hessian);
  if (d < 0.0000000001) {
    //printf("doh");
    return -1;
  } else {
    return 1;
  }
}


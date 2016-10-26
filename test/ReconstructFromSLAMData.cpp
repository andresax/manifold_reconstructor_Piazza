/*
 * ReconstructFromSLAMData.cpp
 *
 *  Created on: 27 mar 2016
 *      Author: enrico
 */

#include <CameraPointsCollection.h>
#include <ReconstructFromSLAMData.h>
#include <utilities.hpp>

#define VERBOSE_UPDATE_MANIFOLD_FUNCTION
//#define VERBOSE_ADD_CAMERA_FUNCTION
//#define VERBOSE_SAVE_MANIFOLD_FUNCTION
//#define VERBOSE_CAMERA_ADD
//#define VERBOSE_CAMERA_UPDATE
//#define VERBOSE_POINT_ADD
//#define VERBOSE_POINT_UPDATE
//#define VERBOSE_POINT_IGNORE
//#define VERBOSE_ADD_VISIBILITY_PAIR
#define VERBOSE_POINTS_COUNT
//#define OUTLIER_FILTERING

//#define PRIMARY_POINTS_VISIBILITY_THRESHOLD     3
#define SECONDARY_POINTS_VISIBILITY_THRESHOLD   2

ReconstructFromSLAMData::ReconstructFromSLAMData(ManifoldReconstructionConfig& conf) : manifConf_(conf) {
	iterationCount = 0;

//	manifConf_ = manifConf;
	manifRec_ = new ManifoldMeshReconstructor(manifConf_);
	utilities::Logger logger_;

	cameraNextId = 0;
	pointNextId = 0;

	// initially the manifold is empty, so it is considered not to be saved until the first update
	manifoldUpdatedSinceSave_ = false;

	if (manifConf_.primary_points_visibility_threshold < SECONDARY_POINTS_VISIBILITY_THRESHOLD)
		std::cerr << "PRIMARY_POINTS_VISIBILITY_THRESHOLD should be greater than SECONDARY_POINTS_VISIBILITY_THRESHOLD" << std::endl;
}

ReconstructFromSLAMData::~ReconstructFromSLAMData() {
}

void ReconstructFromSLAMData::setExpectedTotalIterationsNumber(int n){
	expectedTotalIterationsNumber_ = n;
}

void ReconstructFromSLAMData::addCamera(CameraType* newCamera) {
#ifdef VERBOSE_ADD_CAMERA_FUNCTION
	std::cout << std::endl << "ReconstructFromSLAMData::addCamera iteration " << iterationCount << " / " << expectedTotalIterationsNumber_-1 << std::endl;
#endif

#ifdef OUTLIER_FILTERING
	//std::cout << "start outlier filtering" << std::endl;
	//std::vector<bool> inliers;
	//outlierFiltering(inliers);
#endif

	bool isCameraNew;

	if (newCamera->idReconstruction < 0) {

		// Add the new camera to ManifoldMeshReconstructor
		glm::vec3 center = newCamera->center;
		manifRec_->addCameraCenter(center.x, center.y, center.z);
		//insertNewPointsFromCamSet_.insert(newCamera); TODO remove

		// Generate the ManifoldMeshReconstructor's index if the camera hasn't got one already
		isCameraNew = true;
		newCamera->idReconstruction = cameraNextId++;

#ifdef VERBOSE_CAMERA_ADD
		std::cout << "ADD cam " << newCamera->idCam << " (" << newCamera->idReconstruction << ")" << ": " << center.x << ", " << center.y << ", " << center.z << std::endl;
#endif

	} else {
		// if the camera already has the ManifoldMeshReconstructor's index, then it is updated
		isCameraNew = false;
		rayTracingSet_.insert(newCamera);
		glm::vec3 center = newCamera->center;
		manifRec_->moveCamera(newCamera->idReconstruction, center.x, center.y, center.z);

#ifdef VERBOSE_CAMERA_UPDATE
		std::cout << "UPDATE cam " << newCamera->idCam << " (" << newCamera->idReconstruction << ")" << ": " << center.x << ", " << center.y << ", " << center.z << std::endl;
#endif
	}

	// Add the points associated to the new camera to ManifoldMeshReconstructor
	int countIgnoredPoints = 0, countUpdatedPoints = 0, countAddedPoints = 0, countPairedPoints = 0;
	for (auto const &p : newCamera->visiblePointsT) {

		// Only consider points that were observed in many frames
		if (p->getNunmberObservation() < manifConf_.primary_points_visibility_threshold) {
			countIgnoredPoints++;
#ifdef VERBOSE_POINT_IGNORE
			//std::cout << "IGNORE point \t" << p->idPoint << "\t (recId:" <<  p->idReconstruction << "),\tobs: " << p->getNunmberObservation() << std::endl;
#endif
			continue;
		}

		if (p->idReconstruction < 0) {
			// Generate the ManifoldMeshReconstructor's index if the point hasn't got one already
			p->idReconstruction = pointNextId++;

			// If the point didn't have the idReconstruction index, then it is a new point for sure
			glm::vec3 position = p->position;
			manifRec_->addPoint(position.x, position.y, position.z);
			countAddedPoints++;

			// Since the point has just been added, the visibility with the previous (and current) cameras wasn't added before. Add it now
			for (auto coCamera : p->viewingCams) {
				if (coCamera->idReconstruction >= 0) {
					manifRec_->addVisibilityPair(coCamera->idReconstruction, p->idReconstruction);
					rayTracingSet_.insert(coCamera);
					countPairedPoints++;
#ifdef VERBOSE_ADD_VISIBILITY_PAIR
					std::cout << "add visibility with co-camera " << coCamera->idCam << ", point " << p->idPoint << std::endl;
#endif
				} else {
					// All the previous cameras should have been added already.
					// This shouldn't happen, unless some cameras are in CameraPointsCollection but weren't added by ReconstructFromSLAMData::addCamera (this function)
					;//std::cerr << "camera " << coCamera->idCam << " ignored" << std::endl;
				}
			}

#ifdef VERBOSE_POINT_ADD
			std::cout << "ADD    point \t" << p->idPoint << "\t (recId:" << p->idReconstruction << "),\tobs: " << p->getNunmberObservation()
			<< ": " << position.x << ", " << position.y << ", " << position.z
			<< std::endl;
#endif

		} else {

			// The point was already added to ManifoldMeshReconstructor, so it is only updated
			if (true || iterationCount > 7) { // 7: workaround. movePoint only after insertNewPointsFomrCam // TODO try to remove
				glm::vec3 position = p->position;
				manifRec_->movePoint(p->idReconstruction, position.x, position.y, position.z);
				countUpdatedPoints++;
#ifdef VERBOSE_POINT_UPDATE
				std::cout << "UPDATE point \t" << p->idPoint << "\t (recId:" << p->idReconstruction << "),\tobs: " << p->getNunmberObservation()
				<< ": " << position.x << ", " << position.y << ", " << position.z
				<< std::endl;
#endif
			}

			if (isCameraNew) {
				// Add visibility between the (already added) point and the just added camera
				manifRec_->addVisibilityPair(newCamera->idReconstruction, p->idReconstruction);
				rayTracingSet_.insert(newCamera);
				countPairedPoints++;
#ifdef VERBOSE_ADD_VISIBILITY_PAIR
				std::cout << "add visibility with new camera " << newCamera->idCam << ", point " << p->idPoint << std::endl;
#endif
			} //else: if the camera is being updated, the visibility pair was already added
		}
	}

#ifdef VERBOSE_POINTS_COUNT
	std::cout << "Added   Points:   " << countAddedPoints << std::endl << "Updated Points:   " << countUpdatedPoints << std::endl << "Ignored Points:   " << countIgnoredPoints << std::endl << "Paired  Points:   " << countPairedPoints << std::endl << std::endl;
#endif

	iterationCount++;
}

void ReconstructFromSLAMData::updateManifold() {
#ifdef VERBOSE_UPDATE_MANIFOLD_FUNCTION
	std::cout << std::endl << "ReconstructFromSLAMData::updateManifold iteration " << iterationCount << " / " << expectedTotalIterationsNumber_-1 << std::endl;
#endif

	manifoldUpdatedSinceSave_ = true;
	//TODO actually manage the case of newCameras have too few or no points associated (in ManifoldMeshReconstructior) : count( newCamera.points st idRec>=0 )

	// Tell ManifoldMeshReconstructor to update the triangulation with the new cameras and points
	std::cout << "┍ updateTriangulation" << std::endl;
	logger_.startEvent();
	manifRec_->updateTriangulation();
	logger_.endEventAndPrint("updateTriangulation\t\t", true);

//	std::cout << std::endl << "┍ updateTriangulation" << std::endl;
//	logger_.startEvent();
//	for (auto camera : rayTracingSet_) {
//		std::cout << "├ insertNewPointsFromCam and rayTracingFromCam " << camera->idCam << std::endl;
//		manifRec_->insertNewPointsFromCam(camera->idReconstruction, true);
//		//manifRec_->rayTracingFromCam(camera->idReconstruction);
//	}
//	rayTracingSet_.clear();
//	logger_.endEventAndPrint("updateTriangulation\t\t\t", true);


	// Tell ManifoldMeshReconstructor to grow back the manifold
//	logger_.startEvent();
//
//	logger_.startEvent();
//	std::cout << std::endl << "┍ growManifold" << std::endl;
//	manifRec_->growManifold();
//	logger_.endEventAndPrint("├ growManifold\t\t\t", true);
//	manifRec_->timeStatsFile_ << logger_.getLastDelta() << ", ";
//
//	logger_.startEvent();
//	manifRec_->growManifoldSev();
//	logger_.endEventAndPrint("├ growManifoldSev\t\t", true);
//	manifRec_->timeStatsFile_ << logger_.getLastDelta() << ", ";
//
//	logger_.startEvent();
//	manifRec_->growManifold();
//	logger_.endEventAndPrint("├ growManifold\t\t\t", true);
//	manifRec_->timeStatsFile_ << logger_.getLastDelta() << ", ";
//
//	logger_.endEventAndPrint("growManifold\t\t\t", true);
}

void ReconstructFromSLAMData::saveManifold(std::string namePrefix, std::string nameSuffix) {
#ifdef VERBOSE_SAVE_MANIFOLD_FUNCTION
	std::cout << std::endl << "ReconstructFromSLAMData::saveManifold iteration " << iterationCount << " / " << expectedTotalIterationsNumber_-1 << std::endl;
#endif

	if (manifoldUpdatedSinceSave_ == false) return;
	manifoldUpdatedSinceSave_ = false;


	logger_.startEvent();
	std::ostringstream nameManifold;
	nameManifold << namePrefix << "manifold_" << nameSuffix << ".off";
	std::cout << "saving " << nameManifold.str() << std::endl;
	manifRec_->saveManifold(nameManifold.str());
	logger_.endEventAndPrint("save manifold\t\t\t", true);
//	manifRec_->timeStatsFile_ << logger_.getLastDelta() << ", ";


//  logger_.startEvent();
//  std::ostringstream nameFreespace; nameFreespace << namePrefix << "free_space_manifold_" << nameSuffix << ".off";
//  std::cout << "saving " << nameFreespace.str() << std::endl;
//  manifRec_->saveFreespace(nameFreespace.str());
//  logger_.endEventAndPrint("save free_space_manifold\t\t\t", true);
//
//  std::vector<int> age;
//  for (int cur = 0; cur < cp_data_.numCameras(); cur++) age.push_back(cur);
//
//  logger_.startEvent();
//  std::ostringstream nameManifoldWithoutSteinerPoints; nameManifoldWithoutSteinerPoints << namePrefix << "manifold_without_steiner_points_" << nameSuffix << ".off";
//  std::cout << "saving " << nameManifoldWithoutSteinerPoints.str() << std::endl;
//  manifRec_->saveOldManifold(nameManifoldWithoutSteinerPoints.str(), age);
//  logger_.endEventAndPrint("save manifold_without_steiner_points\t", true);

}

void ReconstructFromSLAMData::insertStatValue(float v){
	manifRec_->timeStatsFile_ << v << ", ";
}

//void ReconstructFromSLAMData::overwriteFocalY(float f) {
//	for (auto const &kvCamera : cp_data_.getCameras()) {
//		kvCamera.second->intrinsics[1][1] = f;
//	}
//
//}

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

int ReconstructFromSLAMData::GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint) {
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


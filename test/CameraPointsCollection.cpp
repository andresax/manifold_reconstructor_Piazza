/*
 * CameraPointsCollection.cpp
 *
 *  Created on: 25 Mar 2016
 *      Author: enrico
 */

#include <CameraPointsCollection.h>
#include <types_reconstructor.hpp>
#include <map>
#include <utility>

CameraPointsCollection::CameraPointsCollection() {

}

CameraPointsCollection::~CameraPointsCollection() {
  clear();
}

void CameraPointsCollection::clear(){
  points_.clear();
  cameras_.clear();
}

void CameraPointsCollection::initCameras(int numCameras){
  //cameras_.assign(numCameras, NULL);
}

void CameraPointsCollection::addCamera(CameraType* cam){
  cameras_.insert(std::pair<long unsigned int,CameraType*>(cam->idCam, cam));
}

CameraType* CameraPointsCollection::getCamera(long unsigned int camId){
  return cameras_[camId];
}

int CameraPointsCollection::numCameras(){
  return cameras_.size();
}


void CameraPointsCollection::initPoints(int numPoints){
  //points_.assign(numPoints, NULL);
}

void CameraPointsCollection::addPoint(PointType* point){
  points_.insert(std::pair<long unsigned int,PointType*>(point->idPoint, point));
}

PointType* CameraPointsCollection::getPoint(long unsigned int pointId){
  return points_[pointId];
}

int CameraPointsCollection::numPoints(){
  return points_.size();
}


void CameraPointsCollection::addVisibility(long unsigned int camId, long unsigned int pointId){
  CameraType* cam = getCamera(camId);
  PointType* point = getPoint(pointId);

  cam->addPoint(point);
  point->addCamera(cam);
}



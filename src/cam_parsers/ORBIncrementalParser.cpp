#include <ORBIncrementalParser.h>
#include <Exceptions.hpp>
#include <stdexcept>
#include <rapidjson/reader.h>
#include <utilities.hpp>

#define COMMA <<", "<<

ORBIncrementalParser::ORBIncrementalParser(std::string path) {
  fileStream_.open(path.c_str());

  std::string str((std::istreambuf_iterator<char>(fileStream_)), std::istreambuf_iterator<char>());
  document_.Parse(str.c_str());

  try {
    if (!document_.IsObject()) throw JsonParseException("JsonParseException--> the json file " + fileName_ + " is not valid");
  } catch (JsonParseException& e) {
    std::cerr << e.what() << std::endl;
    std::cout << e.what() << std::endl;
  }

  basePath_ = std::string(document_["root_path"].GetString());
  if(!document_.HasMember("slam_data_version")
      || document_["slam_data_version"].GetString() != "0.2"){
    std::cerr << "incorrect data version" << std::endl << document_["slam_data_version"].GetString()  << std::endl ;
  }

  jsonViewIndex_ = 0;
  jsonViewsArray_ = document_["views"];

  parseIntrinsics();
}

ORBIncrementalParser::~ORBIncrementalParser() {
  ORB_data_.clear();
}

int ORBIncrementalParser::numCameras(){
  return jsonViewsArray_.Size();
}

CameraType* ORBIncrementalParser::nextCamera(){
  if(jsonViewIndex_ < jsonViewsArray_.Size()){
    const rapidjson::Value& jsonView = jsonViewsArray_[jsonViewIndex_];
    jsonViewIndex_++;

    CameraType* camera = new CameraType();

    camera->idCam = jsonView["viewId"].GetInt();
    ORB_data_.addCamera(camera);

    std::string local(jsonView["local_path"].GetString());
    std::string filename(jsonView["filename"].GetString());
    camera->pathImage = basePath_ + local + filename;

    camera->imageWidth = jsonView["width"].GetInt();
    camera->imageHeight = jsonView["height"].GetInt();

    glm::mat3 rotation;
    glm::vec3 center;
    glm::vec3 translation;
    glm::mat4 eMatrix(0.0), kMatrix(0.0);
    glm::mat3 intrinsic = intrinsics_.at(jsonView["id_intrinsic"].GetInt());

    for (int curR = 0; curR < 3; curR++) {
      center[curR] = jsonView["extrinsic"]["center"][curR].GetFloat();
    }

    for(int curR = 0; curR < 3; curR++) for(int curC = 0; curC < 3; curC++){
      kMatrix[curR][curC] = intrinsic[curR][curC];
    }

    for (int curR = 0; curR < 3; curR++) for (int curC = 0; curC < 3; curC++){
      eMatrix[curR][curC] = jsonView["extrinsic"]["rotation"][curR][curC].GetFloat();
      rotation[curR][curC] = jsonView["extrinsic"]["rotation"][curR][curC].GetFloat();
    }

    translation = -center * rotation;

    eMatrix[0][3] = translation[0];
    eMatrix[1][3] = translation[1];
    eMatrix[2][3] = translation[2];
    eMatrix[3][3] = 1.0;

    camera->cameraMatrix =  eMatrix*kMatrix;
    camera->rotation = glm::mat3(rotation);
    camera->translation = glm::vec3(translation);
    camera->center = glm::vec3(center);
    camera->intrinsics =  intrinsic;
    // TODO check camera properties are correct


    const rapidjson::Value& jsonObservationsArray = jsonView["observations"];
    for (rapidjson::SizeType jsonObservationIndex = 0; jsonObservationIndex < jsonObservationsArray.Size(); jsonObservationIndex++){
      const rapidjson::Value& jsonObservationObject = jsonObservationsArray[jsonObservationIndex];

      long unsigned int pointId = jsonObservationObject["pointId"].GetInt();
      PointType* point;

      if(ORB_data_.hasPoint(pointId)){
        point = ORB_data_.getPoint(pointId);

        //std::cout << "UPDATE idPoint: "<<point->idPoint << "\tidReconstruction: "<<point->idReconstruction << "\tgetNunmberObservation: "<<point->getNunmberObservation() << std::endl;

      }else{
        point = new PointType();
        point->idPoint = pointId;
        ORB_data_.addPoint(point);
      }

      ORB_data_.addVisibility(camera, point);

      const rapidjson::Value& jsonCameraCenter = jsonObservationObject["X"];
      float x = jsonCameraCenter[0].GetFloat(), y = jsonCameraCenter[1].GetFloat(), z = jsonCameraCenter[2].GetFloat();
      point->position = glm::vec3(x, y, z);
     // std::cout << "       idPoint: "<<point->idPoint << "\tidReconstruction: "<<point->idReconstruction << "\tgetNunmberObservation: "<<point->getNunmberObservation() << std::endl;

      //TODO point's 2D coordinates in frame
//      const rapidjson::Value& jsonCameraFrameCoordinates = jsonObservationObject["x"];
//      float u = jsonCameraFrameCoordinates[0].GetFloat(), v = jsonCameraFrameCoordinates[1].GetFloat();
//      ORB_data_.addFrameCoordinates(camera, point, glm::vec3(u, v));
    }

    return camera;

  }else{
    return NULL;
  }


}
//
//// TODO do a camera at a time
//void ORBIncrementalParser::parseViews() {
//  if(jsonViewIndex_ < jsonViewsArray_.Size()){
//    const rapidjson::Value& jsonView = jsonViewsArray_[jsonViewIndex_];
//    jsonViewIndex_++;
//
//    CameraType* camera = new CameraType();
//
//    camera->idCam = jsonView["key"].GetInt();
//    ORB_data_.addCamera(camera);
//
//    std::string local(jsonView["local_path"].GetString());
//    std::string filename(jsonView["filename"].GetString());
//    camera->pathImage = basePath + local + filename;
//
//    camera->imageWidth = jsonView["width"].GetInt();
//    camera->imageHeight = jsonView["height"].GetInt();
//
//    glm::mat3 rotation;
//    glm::vec3 center;
//    glm::vec3 translation;
//    glm::mat4 eMatrix(0.0), kMatrix(0.0);
//    glm::mat3 intrinsic = intrinsics_.at(jsonView["id_intrinsic"].GetInt());
//
//    for (int curR = 0; curR < 3; curR++) {
//      center[curR] = jsonView["extrinsic"]["center"][curR].GetFloat();
//    }
//
//    for(int curR = 0; curR < 3; curR++) for(int curC = 0; curC < 3; curC++){
//      kMatrix[curR][curC] = intrinsic[curR][curC];
//    }
//
//    for (int curR = 0; curR < 3; curR++) for (int curC = 0; curC < 3; curC++){
//      eMatrix[curR][curC] = jsonView["extrinsic"]["rotation"][curR][curC].GetFloat();
//      rotation[curR][curC] = jsonView["extrinsic"]["rotation"][curR][curC].GetFloat();
//    }
//
//    translation = -center * rotation;
//
//    eMatrix[0][3] = translation[0];
//    eMatrix[1][3] = translation[1];
//    eMatrix[2][3] = translation[2];
//    eMatrix[3][3] = 1.0;
//
//    camera->cameraMatrix =  eMatrix*kMatrix;
//    camera->rotation = glm::mat3(rotation);
//    camera->translation = glm::vec3(translation);
//    camera->center = glm::vec3(center);
//    camera->intrinsics =  intrinsic;
//    // TODO check camera properties are correct
//
//
//    const rapidjson::Value& jsonObservationsArray = jsonView["observations"];
//    for (rapidjson::SizeType jsonObservationIndex = 0; jsonObservationIndex < jsonObservationsArray.Size(); jsonObservationIndex++){
//      const rapidjson::Value& jsonObservationObject = jsonObservationsArray[jsonObservationIndex];
//
//      long unsigned int pointId = jsonObservationObject["key"].GetInt();
//      PointType* point;
//
//      if(ORB_data_.hasPoint(pointId)){
//        point = ORB_data_.getPoint(pointId);
//      }else{
//        point = new PointType();
//        point->idPoint = pointId;
//        ORB_data_.addPoint(point);
//      }
//
//      ORB_data_.addVisibility(camera, point);
//
//      const rapidjson::Value& jsonCameraCenter = jsonObservationObject["X"];
//      float x = jsonCameraCenter[0].GetFloat(), y = jsonCameraCenter[1].GetFloat(), z = jsonCameraCenter[2].GetFloat();
//      point->position = glm::vec3(x, y, z);
//
//      //TODO point's 2D coordinates in frame
////      const rapidjson::Value& jsonCameraFrameCoordinates = jsonObservationObject["x"];
////      float u = jsonCameraFrameCoordinates[0].GetFloat(), v = jsonCameraFrameCoordinates[1].GetFloat();
////      ORB_data_.addFrameCoordinates(camera, point, glm::vec3(u, v));
//    }
//  }
//}


void ORBIncrementalParser::parseIntrinsics() {

  try {

    if (!document_.HasMember("intrinsics"))
      throw JsonAccessException("JsonAccessException--> error while querying HasMember(views)");
    const rapidjson::Value& intrinsicsJson = document_["intrinsics"];

    for (rapidjson::SizeType curInt = 0; curInt < intrinsicsJson.Size(); curInt++) {
      int key = intrinsicsJson[curInt]["intrinsicId"].GetInt();
      glm::mat3 temp(0.0);
      temp[0][0] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["focal_length"].GetFloat();
      temp[1][1] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["focal_length"].GetFloat();
      temp[0][2] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["principal_point"][0].GetFloat();
      temp[1][2] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["principal_point"][1].GetFloat();
      temp[2][2] = 1.0;

      intrinsics_.insert(std::pair<int, glm::mat3>(key, temp));
    }

  } catch (JsonAccessException& e) {
    std::cerr << e.what() << std::endl;
    std::cout << e.what() << std::endl;

  }
}




std::string ORBIncrementalParser::getDataSPlot() {
  std::stringstream out;

  for(auto mCamera : ORB_data_.getCameras()){
    CameraType* c = mCamera.second;
    long unsigned int idCam =c->idCam;
    for(auto p : c->visiblePointsT){
      out << c->idCam COMMA p->position.x COMMA p->position.y COMMA p->position.z COMMA p->getNunmberObservation() << std::endl;
    }

  }

  return out.str();
}


std::string ORBIncrementalParser::getDataCSV() {
  std::stringstream out;

  for(auto mCamera : ORB_data_.getCameras()){
    CameraType* c = mCamera.second;
    glm::vec3 center = c->center;

    out << "c" COMMA c->idCam COMMA center.x COMMA center.y COMMA center.z << std::endl;

    for(auto p : c->visiblePointsT){
      out << "p" COMMA p->idPoint COMMA c->idCam COMMA p->position.x COMMA p->position.y COMMA p->position.z << std::endl;
    }

  }

  return out.str();
}


std::string ORBIncrementalParser::getStats() {
  std::stringstream out;

  for(auto mCamera : ORB_data_.getCameras()){
    CameraType* c = mCamera.second;

    out << "cam " << c->idCam << std::endl;
    std::map<int, int> count;

    for(auto p : c->visiblePointsT){
      int nOcc = p->getNunmberObservation();

      std::map<int,int>::iterator it = count.find(nOcc);
      if(it != count.end()){
        count[nOcc] += 1;
      }else{
        count.insert(std::pair<int, int>(p->getNunmberObservation(), 1));
      }


    }

    for(auto vkOcc : count){
      int sum = 0;
      for(auto vkOcc_ : count){
        if(vkOcc_.first >= vkOcc.first) sum += vkOcc_.second;
      }

      out << " =" << vkOcc.first << ":\t" << vkOcc.second << ";\t\t >=" << vkOcc.first << ":\t" << sum << std::endl;
    }
  }

  return out.str();

}

//  Copyright 2014 Andrea Romanoni
//
//  This file is part of manifoldReconstructor.
//
//  edgePointSpaceCarver is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version see
//  <http://www.gnu.org/licenses/>..
//
//  edgePointSpaceCarver is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

#include <CameraPointsCollection.h>
#include <OpenMvgParser.h>
#include <ORBParser.h>
#include <ReconstructFromSLAMData.h>
#include <types_config.hpp>
#include <iostream>


//*************************************************************************************************/
//********************************RECONSTRUCTION FROM VISIBILITY***********************************/
//*************************************************************************************************/
int main(int argc, char **argv) {
//  OpenMvgParser op("/home/andrea/Scrivania/Datasets/Middelbury/templeRing/openmvgMatch/out.json");
  std::cout << "start parsing OpenMvgParser" << std::endl;
  OpenMvgParser op_openmvg("data/dinoRing/dinoRing.json");
  op_openmvg.parse();
  std::cout << "end parsing OpenMvgParser" << std::endl;

  std::cout << "start parsing ORBParser" << std::endl;
  ORBParser op("data/dinoRing/dinoRing.json");
  op.parse();
  std::cout << "end parsing ORBParser" << std::endl;

  //utilities::saveVisibilityPly(op.getSfmData());

  ManifoldReconstructionConfig confManif;
  confManif.inverseConicEnabled = true;
  confManif.maxDistanceCamFeature = 10.0;
  confManif.probOrVoteThreshold = 2.1;
  confManif.enableSuboptimalPolicy = false;
  confManif.suboptimalMethod = 0;
  confManif.w_1 = 1.0;
  confManif.w_2 = 0.0;
  confManif.w_3 = 0.00;

  CameraPointsCollection orb_data_ = op.getData();

  std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;
  std::cout << "orb: " <<  orb_data_.numCameras() << " cams; " << orb_data_.numPoints() << " points" << std::endl << std::endl;

  ReconstructFromSLAMData m(orb_data_, confManif);
//m.overwriteFocalY(1525.900000);
  m.increment();

  return 0;
}

//*************************************************************************************************/
//************************************EDGE-POINT RECONSTRUCTION************************************/
//*************************************************************************************************/
//
//int main22(int argc, char** argv) {
//  /*std::string numKitti;
//   std::string arg1, arg2;
//   if (argc > 2) {
//   arg1 = argv[1];
//   arg2 = argv[2];
//   std::cout << "Two arguments are needed: the first is the path of the configuration file, "
//   << "the second one is the number of the KITTI sequence, do not omit the '0' (i.e., a right argument is 0095, not 95)" << std::endl;
//   }*/
//
//  PathCreator pathcreator("0095");
//
//  KittiRectifiedCamParser *kittiParser = new KittiRectifiedCamParser("./examples/kitti0095_rect.out");
//  kittiParser->parseFile();
//  std::vector<CameraRect> cameras = kittiParser->getCamerasList();
//  EdgePointConfigurator epc("./config/ConfigFile_0095.config");
//
//  Configuration myConf = epc.parseConfigFile();
//
//  pathcreator.setPrefixAndSpaceC(std::string("ReconstKitti_"), std::string("Results"), myConf.spaceCarvingConfig);
//  myConf.videoConfig.folderImage = pathcreator.getPathFolderImage();
//
//  myConf.outputSpaceCarving.pathToSave = pathcreator.getPathToSave();
//  myConf.outputSpaceCarving.pathLog = pathcreator.getPathLog();
//  myConf.outputSpaceCarving.pathLogPoints = pathcreator.getPathLogPoints();
//  myConf.outputSpaceCarving.pathStats = pathcreator.getPathStats();
//  myConf.outputSpaceCarving.pathStatsManifold = pathcreator.getPathStatsManifold();
//
//  std::stringstream filepath;
//  filepath << myConf.outputSpaceCarving.pathToSave << "configurationUsed.txt";
//  std::ofstream fileConf(filepath.str().c_str());
//  fileConf << myConf.toString();
//  std::cout << myConf.toString() << std::endl;
//  fileConf.close();
//
//  EdgePointSpaceCarver spaceCarver(myConf, cameras);
//  spaceCarver.run();
//
//  return 0;
//}

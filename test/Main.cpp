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
#include <Logger.h>
#include <ORBIncrementalParser.h>
#include <ReconstructFromSLAMData.h>
#include <types_config.hpp>
#include <types_reconstructor.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <utility>

//#define USE_SFM
//#define PRODUCE_STATS
//#define SAVE_POINTS_TO_OFF_AND_EXIT

#define MANIFOLD_UPDATE_EVERY 5
#define INITIAL_MANIFOLD_UPDATE_SKIP 2
#define SAVE_MANIFOLD_EVERY 5

//*************************************************************************************************/
//********************************RECONSTRUCTION FROM VISIBILITY***********************************/
//*************************************************************************************************/
int main(int argc, char **argv) {
  utilities::Logger log;  log.startEvent();
  std::ofstream statsFile, visiblePointsFile;
  int maxIterations_ = 0;

  ManifoldReconstructionConfig confManif;
  confManif.inverseConicEnabled = true;
  confManif.maxDistanceCamFeature = 100.0;
  confManif.probOrVoteThreshold = 1.1;
  confManif.enableSuboptimalPolicy = false;
  confManif.suboptimalMethod = 0;
  confManif.w_1 = 1.0;
  confManif.w_2 = 0.2;
  confManif.w_3 = 0.0;


  if(argc < 2){
    std::cerr << std::endl << "Usage: ./manifoldReconstructor path_to_input.json [max_iterations]" << std::endl;
    return 1;
  }else{
    std::cout << "input set to: " << argv[1] << std::endl << std::endl;
  }

  if(argc > 2){
    maxIterations_ = atoi(argv[2]);
    std::cout << "max_iterations set to: " << maxIterations_ << std::endl << std::endl;
  }else{
    std::cout << "max_iterations not set" << std::endl << std::endl;
  }

#ifdef USE_SFM

  OpenMvgParser op_openmvg(argv[1]);
  op_openmvg.parse();
  std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;

  ReconstructFromSfMData mSfM(op_openmvg.getSfmData(), confManif);

  mSfM.run();

  log.endEventAndPrint("\t\t\t\t\t\t", true);
  return 0;

#else


#ifdef SAVE_POINTS_TO_OFF_AND_EXIT
  std::cout << "start parsing " << argv[1] << std::endl;
  log.startEvent();
  ORBIncrementalParser op(argv[1]);
  op.ParseToOFF("output/all_points_", 10);
  log.endEventAndPrint("Parsing\t\t\t\t\t\t", true);
  return 0;
#else
  std::cout << "start parsing " << argv[1] << std::endl;
  log.startEvent();
  ORBIncrementalParser op(argv[1]);
  log.endEventAndPrint("Parsing\t\t\t\t\t\t", true);

  CameraPointsCollection orb_data_ = op.getData();
  std::cout << "orb: " <<  op.numCameras() << " cams" << std::endl << std::endl;

  ReconstructFromSLAMData m(orb_data_, confManif);

  // main loop
  for(int i=0; i < op.numCameras(); i++){
    CameraType* camera = op.nextCamera();

    if(camera == NULL){
      continue;
    }

    // If maxIterations_ is set, only execute ReconstructFromSLAMData::addCamera maxIterations_ times
    if(maxIterations_ && m.iterationCount >= maxIterations_) break;

    log.startEvent();

    m.addCamera(camera);

    // Skip the manifold update for the first INITIAL_MANIFOLD_UPDATE_SKIP cameras
    if(m.iterationCount > INITIAL_MANIFOLD_UPDATE_SKIP  && !(m.iterationCount%MANIFOLD_UPDATE_EVERY)) m.updateManifold();

    if(m.iterationCount && !(m.iterationCount%SAVE_MANIFOLD_EVERY)) m.saveManifold("output/partial/", std::to_string(m.iterationCount));

    log.endEventAndPrint("main loop\t\t\t\t\t\t\t", true); std::cout << std::endl;

#ifdef PRODUCE_STATS
    statsFile.open("output/stats/stats.txt", std::ios_base::app);
    statsFile << std::endl << std::endl << "Iteration " << m.iterationCount << std::endl;
    statsFile << op.getStats();
    statsFile.close();

    std::ostringstream nameVisiblePoints; nameVisiblePoints << "output/vp/visible_points_" << camera->idCam << ".off";
    visiblePointsFile.open(nameVisiblePoints.str().c_str());
    visiblePointsFile << op.getDataOFF();
    visiblePointsFile.close();
#endif
  }

  // Do a last manifold update in case op.numCameras() isn't a multiple of MANIFOLD_UPDATE_EVERY
  if(m.iterationCount > INITIAL_MANIFOLD_UPDATE_SKIP) m.updateManifold();

  m.saveManifold("output/", "final");

  log.endEventAndPrint("main\t\t\t\t\t\t", true);

  return 0;

#endif
#endif
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

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

#include <string>

#include <PathCreator.h>
//#include <KittiRectifiedCamParser.h>
#include <ManifoldReconstructorConfigurator.h>
//#include <EdgePointConfigurator.h>
//#include <EdgePointSpaceCarver.h>
#include <ReconstructorFromOut.h>
#include <types_reconstructor.hpp>
#include <OpenMvgParser.h>
#include <ORBIncrementalParser.h>
#include <ReconstructFromSfMData.h>
//#include <types_config.hpp>

//*************************************************************************************************/
//********************************RECONSTRUCTION FROM VISIBILITY***********************************/
//*************************************************************************************************/
int main(int argc, char **argv) {
//  OpenMvgParser op("/home/andrea/Scrivania/Datasets/Middelbury/templeRing/openmvgMatch/out.json");
//  OpenMvgParser op("data/0095/0095.json");

//  op.parse();
	//utilities::saveVisibilityPly(op.getSfmData());

	std::ofstream statsFile, visiblePointsFile;
	ManifoldReconstructionConfig confManif;
	std::string input_file;
	std::string config_file;

	int maxIterations_ = 0;
	if (argc == 4) {
		maxIterations_ = atoi(argv[3]);
		config_file = argv[2];
	} else if (argc == 3) {
		config_file = argv[2];
		std::cout << "max_iterations not set" << std::endl << std::endl;
	} else if (argc == 2) {
		input_file = argv[1];
		config_file = "config/manifRecConfig_default.json";
		std::cout << "max_iterations not set" << std::endl << std::endl;
	} else {
		std::cerr << std::endl << "Usage: ./manifoldReconstructor path_to_input.json [path_to_config.json [max_iterations]]" << std::endl;
		return 1;
	}

	ORBIncrementalParser p(argv[1]);

//	ManifoldReconstructionConfig confManif;
	confManif.inverseConicEnabled = true;
	confManif.maxDistanceCamFeature = 40.0;
	confManif.probOrVoteThreshold = 1;
//	confManif.probOrVoteThreshold = 10.0;
	confManif.enableSuboptimalPolicy = false;
	confManif.suboptimalMethod = 0;
	confManif.w_1 = 1.0;
	confManif.w_2 = 0.0;
	confManif.w_3 = 0.0;
//	confManif.w_2 = 0.8;
//	confManif.w_3 = 0.4;

	// Parse all the keyframes without updating
	for (int i = 0; i < p.numCameras(); i++)
		p.nextCamera();

	CameraPointsCollection cpc = p.getData();
	cpc.computeSfmData();

	ReconstructFromSfMData m(cpc.getSfmData(), confManif);
//m.overwriteFocalY(1525.900000);

	m.run();
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

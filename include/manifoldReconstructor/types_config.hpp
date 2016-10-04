//  Copyright 2014 Andrea Romanoni
//
//  This file is part of edgePointSpaceCarver.
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

#ifndef TYPES_TEST_HPP_
#define TYPES_TEST_HPP_

#include <sstream>
#include <string>

typedef struct {
  bool inverseConicEnabled;
  float probOrVoteThreshold;
  float maxDistanceCamFeature;
  bool enableSuboptimalPolicy;
  int suboptimalMethod;
  float w_1;
  float w_2;
  float w_3;
  float steinerGridSideLength;
  float steinerGridStepLength;

  int manifold_update_every;
  int initial_manifold_update_skip;
  int save_manifold_every;
  int primary_points_visibility_threshold;

  int fake_points_multiplier;

  bool all_sort_of_output;
  bool update_points_position;


  std::string toString() {
    std::stringstream out;
    out << "inverseConicEnabled: " << inverseConicEnabled << std::endl;
    out << "maxDistanceCamFeature: " << maxDistanceCamFeature << std::endl;
    out << "probOrVoteThreshold: " << probOrVoteThreshold << std::endl;
    out << "enableSuboptimalPolicy: " << enableSuboptimalPolicy << std::endl;
    out << "suboptimalMethod: " << suboptimalMethod << std::endl;
    out << "w_1: " << w_1 << std::endl;
    out << "w_2: " << w_2 << std::endl;
    out << "w_3: " << w_3 << std::endl;
    out << "steinerGridStepLength: " << steinerGridStepLength << std::endl;
    out << "steinerGridSideLength: " << steinerGridSideLength << std::endl;

    out << "manifold_update_every: " << manifold_update_every << std::endl;
    out << "initial_manifold_update_skip: " << initial_manifold_update_skip << std::endl;
    out << "save_manifold_every: " << save_manifold_every << std::endl;
    out << "primary_points_visibility_threshold: " << primary_points_visibility_threshold << std::endl;

    out << "fake_points_multiplier: " << fake_points_multiplier << std::endl;

    out << "all_sort_of_output: " << all_sort_of_output << std::endl;
    out << "update_points_position: " << update_points_position << std::endl;

    return out.str();
  }

} ManifoldReconstructionConfig;


#endif /* TYPES_TEST_HPP_ */

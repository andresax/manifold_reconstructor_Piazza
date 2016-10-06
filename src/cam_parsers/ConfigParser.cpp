/*
 * ConfigParser.cpp
 *
 *  Created on: 18 May 2016
 *      Author: enrico
 */

#include "ConfigParser.h"
#include <Exceptions.hpp>
#include <stdexcept>
#include <rapidjson/reader.h>

ConfigParser::ConfigParser() {
	// TODO Auto-generated constructor stub

}

ConfigParser::~ConfigParser() {
	// TODO Auto-generated destructor stub
}

ManifoldReconstructionConfig ConfigParser::parse(std::string path) {

	rapidjson::Document document;
	std::ifstream fileStream;
	ManifoldReconstructionConfig c;

	fileStream.open(path.c_str());
	std::string str((std::istreambuf_iterator<char>(fileStream)), std::istreambuf_iterator<char>());
	document.Parse(str.c_str());

	try {

		if (!document.HasMember("inverseConicEnabled")) throw JsonAccessException("");
		if (!document["inverseConicEnabled"].IsBool()) throw JsonAccessException("");
		c.inverseConicEnabled = document["inverseConicEnabled"].GetBool();

		if (!document.HasMember("maxDistanceCamFeature")) throw JsonAccessException("");
		if (!document["maxDistanceCamFeature"].IsFloat()) throw JsonAccessException("");
		c.maxDistanceCamFeature = document["maxDistanceCamFeature"].GetFloat();

		if (!document.HasMember("probOrVoteThreshold")) throw JsonAccessException("");
		if (!document["probOrVoteThreshold"].IsFloat()) throw JsonAccessException("");
		c.probOrVoteThreshold = document["probOrVoteThreshold"].GetFloat();

		if (!document.HasMember("rayRemovalThreshold")) throw JsonAccessException("");
		if (!document["rayRemovalThreshold"].IsFloat()) throw JsonAccessException("");
		c.rayRemovalThreshold = document["rayRemovalThreshold"].GetFloat();

		if (!document.HasMember("enableSuboptimalPolicy")) throw JsonAccessException("");
		if (!document["enableSuboptimalPolicy"].IsBool()) throw JsonAccessException("");
		c.enableSuboptimalPolicy = document["enableSuboptimalPolicy"].GetBool();

		if (!document.HasMember("suboptimalMethod")) throw JsonAccessException("");
		if (!document["suboptimalMethod"].IsInt()) throw JsonAccessException("");
		c.suboptimalMethod = document["suboptimalMethod"].GetInt();

		if (!document.HasMember("w_1")) throw JsonAccessException("");
		if (!document["w_1"].IsFloat()) throw JsonAccessException("");
		c.w_1 = document["w_1"].GetFloat();

		if (!document.HasMember("w_2")) throw JsonAccessException("");
		if (!document["w_2"].IsFloat()) throw JsonAccessException("");
		c.w_2 = document["w_2"].GetFloat();

		if (!document.HasMember("w_3")) throw JsonAccessException("");
		if (!document["w_3"].IsFloat()) throw JsonAccessException("");
		c.w_3 = document["w_3"].GetFloat();

		if (!document.HasMember("w_m")) throw JsonAccessException("");
		if (!document["w_m"].IsFloat()) throw JsonAccessException("");
		c.w_m = document["w_m"].GetFloat();

		if (!document.HasMember("steinerGridStepLength")) throw JsonAccessException("");
		if (!document["steinerGridStepLength"].IsFloat()) throw JsonAccessException("");
		c.steinerGridStepLength = document["steinerGridStepLength"].GetFloat();

		if (!document.HasMember("steinerGridSideLength")) throw JsonAccessException("");
		if (!document["steinerGridSideLength"].IsFloat()) throw JsonAccessException("");
		c.steinerGridSideLength = document["steinerGridSideLength"].GetFloat();

		if (!document.HasMember("manifold_update_every")) throw JsonAccessException("");
		if (!document["manifold_update_every"].IsInt()) throw JsonAccessException("");
		c.manifold_update_every = document["manifold_update_every"].GetInt();

		if (!document.HasMember("initial_manifold_update_skip")) throw JsonAccessException("");
		if (!document["initial_manifold_update_skip"].IsInt()) throw JsonAccessException("");
		c.initial_manifold_update_skip = document["initial_manifold_update_skip"].GetInt();

		if (!document.HasMember("save_manifold_every")) throw JsonAccessException("");
		if (!document["save_manifold_every"].IsInt()) throw JsonAccessException("");
		c.save_manifold_every = document["save_manifold_every"].GetInt();

		if (!document.HasMember("primary_points_visibility_threshold")) throw JsonAccessException("");
		if (!document["primary_points_visibility_threshold"].IsInt()) throw JsonAccessException("");
		c.primary_points_visibility_threshold = document["primary_points_visibility_threshold"].GetInt();

		if (!document.HasMember("all_sort_of_output")) throw JsonAccessException("");
		if (!document["all_sort_of_output"].IsBool()) throw JsonAccessException("");
		c.all_sort_of_output = document["all_sort_of_output"].GetBool();

		if (!document.HasMember("time_stats_output")) throw JsonAccessException("");
		if (!document["time_stats_output"].IsBool()) throw JsonAccessException("");
		c.time_stats_output = document["time_stats_output"].GetBool();

		if (document.HasMember("fake_points_multiplier")){
			if (!document["fake_points_multiplier"].IsInt()) throw JsonAccessException("");
			c.fake_points_multiplier = document["fake_points_multiplier"].GetInt();
		}else c.fake_points_multiplier = 0;

		if (!document.HasMember("update_points_position")) throw JsonAccessException("");
		if (!document["update_points_position"].IsBool()) throw JsonAccessException("");
		c.update_points_position = document["update_points_position"].GetBool();

	} catch (JsonAccessException& e) {
		std::cerr << e.what() << std::endl;
		std::cout << e.what() << std::endl;

	}

	return c;
}

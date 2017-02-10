#!/bin/bash

i=0
for ppc in 10 20 50 100 200 500
do
	for cpi in 10 8 6 4 2 1
	do
		
		echo "\
{
	\"inverseConicEnabled\": true,
	\"enableSuboptimalPolicy\": false,
	\"suboptimalMethod\": 0,
	\"enableRayMistrust\": false,
	
	\"w_1\": 1.0,
	\"w_2\": 0.8,
	\"w_3\": 0.4,
	\"w_m\": 1.0,
	\"freeVoteThreshold\": 40.0,
	\"rayRemovalThreshold\": 100000000.0,
	\"vertexRemovalThreshold\": 100000000.0,
	\"primary_points_visibility_threshold\": 2,
	\"outlierFilteringThreshold\": 0.25,
	\"maxPointsPerCamera\": $ppc,
	
	\"maxDistanceCamFeature\": 40.0,
	\"steinerGridStepLength\": 5.0,
	\"steinerGridSideLength\": 280.0,
	
	\"manifold_update_every\": $cpi,
	\"initial_manifold_update_skip\": 2,
	\"save_manifold_every\": 10,
	
	\"time_stats_output\": true,
	\"all_sort_of_output\": false,
	\"fake_points_multiplier\": 0,
	\"update_points_position\": false,
	
	\"statsId\": \"ppc$ppc cpi$cpi\"
}\
" >> gconf_$i.json
		i=$((i+1))
	done
done

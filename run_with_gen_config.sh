#!/bin/bash

i=0
for i in {0..35}
do
	echo "



################################################################################################################
########                                                                                                   #######
#######    RUNNING ./manifoldReconstructor data/0095/0095.json config/parametric_config/gconf_$i.json       ########
########                                                                                                   #######
################################################################################################################



"
	
	./manifoldReconstructor data/0095/0095.json config/parametric_config/gconf_$i.json
done

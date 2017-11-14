#!/bin/bash

MODEL_NAME=20170707_offline

# touch $log_file
index=0

for i in $(ls -d1 outputs/$MODEL_NAME\/*)
do
	epoch=${i}
	epoch=${epoch:8}
	hist_list[index]=$epoch
	index=$index+1
done


for file in snapshots/$MODEL_NAME\/*
do
	done_infer=0
	# if [[ $file =~ -[0-9]+\.params ]]
	if [[ $file =~ [0-9]+.params ]]
		then
			epoch=${BASH_REMATCH}
			epoch=${epoch:0:${#epoch}-7}
	
		for hist_epoch in ${hist_list[@]}
		do
			if [[ $epoch == $hist_epoch ]]
				then
				done_infer=1
			fi
		done
		if ! (( $done_infer ))
			then
			if 
			(( epoch > 0 ))
				then
				python test.py "$@" --epoch $epoch
			fi
		fi
	fi
done


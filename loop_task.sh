#!/bin/bash

if [ "$1" == "--help" ]; then
	echo "Script to run Gazebo calibration for a given number of times (trials) for a given task (if not specified for the task in current folder)"
	echo "Params:"
	echo "        First param is number of trials for each ptool" 
	echo "        Second param is the task name (if not given will use current directory as task)"
	echo "The script assumes your gazebo models are in ~/.gazebo/gazebo_models/ and that you have a structure of model folders corresponding to different tasks:"
	echo "        (e.g.)~/.gazebo/gazebo_models/hammering_training and/or ~/.gazebo/gazebo_models/lifting_pancake"
	echo ""
	echo "Example call: bash loop_task.sh hammering_nail 5"
	echo "This will run 5 trials for the under the folder tool_TASK_NAME under the task folder (e.g. ~/.gazebo/gazebo_models/hammering_training/tool_hammering_nail)"
	exit 0
fi

N_ITER=$1
if [ "$N_ITER" == "" ]
then
	echo "Please specify the number of trials as the first param"
	exit 1
fi

TASK_FOLDER=$2
if [ "$TASK_FOLDER" == "" ]
then
	TASK_FOLDER=${PWD##*/} 
	echo "WARNING: task not specified: using current folder as task folder"
fi

ROUND=3 # number of decimal places for showing results


FILENAME="loop_"$TASK_FOLDER".txt"
GAZEBO_CALL=$2
if [ -z "$2" ]
then
	GAZEBO_CALL="gazebo"
fi


rm $FILENAME
for ((i=1; i<=$N_ITER; i++))
do	
	#echo $i
 	$GAZEBO_CALL hammering_nail.world >> $FILENAME	
done

echo ""
echo "Looping completed"
echo ""

# loop through results file and calculate mean
SUM="0"
N_ITER_NOERRORS=0
N_VALID_ITER=0
while read -r line_num
do	
	N_ITER_NOERRORS=$(echo $N_ITER_NOERRORS+1 | bc)
	if [ $(echo " $line_num > 0" | bc) -eq 1 ]; then
		N_VALID_ITER=$(echo $N_VALID_ITER+1 | bc)
		SUM=$(echo $SUM+$line_num | bc)
		echo "Result: "$(echo "scale="$ROUND"; "$line_num/1 | bc -l)
	else
		echo "Result is smaller than 0 albeit no Gazebo error"
	fi	
done < "$FILENAME"
echo ""
echo "Number of iterations without Gazebo error: "$N_ITER_NOERRORS
echo "Number of valid iterations: "$N_VALID_ITER
echo "Sum of results: "$(echo "scale="$ROUND"; "$SUM/1 | bc -l)
echo "Mean: "$(echo "scale="$ROUND"; "$SUM/$N_VALID_ITER| bc -l)

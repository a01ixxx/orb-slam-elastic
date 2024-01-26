#!/bin/bash

roscore &

sleep 3 &&

# Number of experiments
NUM_EXPERIMENTS=2

NUM_DATASET=11

BAG_NAMES[0]="-"
BAG_NAMES[1]="MH_01_easy.bag"
BAG_NAMES[2]="MH_02_easy.bag"
BAG_NAMES[3]="MH_03_medium.bag"
BAG_NAMES[4]="MH_04_difficult.bag"
BAG_NAMES[5]="MH_05_difficult.bag"
BAG_NAMES[6]="V1_01_easy.bag"
BAG_NAMES[7]="V1_02_medium.bag"
BAG_NAMES[8]="V1_03_difficult.bag"
BAG_NAMES[9]="V2_01_easy.bag"
BAG_NAMES[10]="V2_02_medium.bag"
BAG_NAMES[11]="V2_03_difficult.bag"

# Base directory to store results
BASE_DIR="./results"

# Ensure the base directory exists
mkdir -p "$BASE_DIR"


for (( j=1; j<=NUM_DATASET; j++ ))
do
    OUTER_DIR="$BASE_DIR/Trace$j"
    mkdir -p "$OUTER_DIR"

    # Loop for experiments
    for (( i=1; i<=NUM_EXPERIMENTS; i++ ))
    do
        # Create a new directory for this experiment's results
        EXP_DIR="$OUTER_DIR/experiment_$i"
        mkdir -p "$EXP_DIR"

        # Launch ros orb-slam3
        rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt /home/ao/orbslam3/Examples_old/Stereo-Inertial/EuRoC.yaml true &

        # wait for orb-slam3 to be ready
        sleep 12 &&

        # launch the rosbag replay
        BAG_NAME="/home/ao/Downloads/${BAG_NAMES[$j]}"
        rosbag play "$BAG_NAME" /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu -r 0.7 &&

        # Wait for the termination of all data processing in orb-slam3
        sleep 1 &&

        pkill -INT Stereo_Inertial

        mv *Format.txt "$EXP_DIR"
        mv *file.txt   "$EXP_DIR"

    done

done

echo "All experiments completed!"






# mkdir mh03_1

# # Launch ros orb-slam3
# rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt /home/ao/orbslam3/Examples_old/Stereo-Inertial/EuRoC.yaml true &

# # wait for orb-slam3 to be ready
# sleep 15

# # launch the rosbag replay
# rosbag play MH_03_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu &&

# pkill Stereo_Inertial

# mv *Format.txt mh03_1
# mv *file.txt   mh03_1




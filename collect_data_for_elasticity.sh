#!/bin/bash

roscore &

sleep 3 &&

# Number of experiments
NUM_IMU=10

NUM_IMG=5

BAG_NAMES[0]="-"
BAG_NAMES[1]="MH_01_easy.bag"
BAG_NAMES[2]="MH_02_easy.bag"
BAG_NAMES[3]="MH_03_medium.bag"
BAG_NAMES[4]="MH_04_difficult.bag"
BAG_NAMES[5]="MH_05_difficult.bag"

# Base directory to store results
BASE_DIR="./results"

# Ensure the base directory exists
mkdir -p "$BASE_DIR"


for (( j=2; j<=NUM_IMG; j++ ))
do
    OUTER_DIR="$BASE_DIR"
    mkdir -p "$OUTER_DIR"


    # Create a new directory for this experiment's results
    EXP_DIR="$OUTER_DIR/image_$j"
    mkdir -p "$EXP_DIR"

    # Launch ros orb-slam3
    rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt /home/ao/orbslam3/Examples_old/Stereo-Inertial/EuRoC.yaml true xx "$j" 1 &

    # wait for orb-slam3 to be ready
    sleep 12 &&

    # launch the rosbag replay
    BAG_NAME="/home/ao/Downloads/${BAG_NAMES[1]}"
    rosbag play "$BAG_NAME" /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu  &&

    # Wait for the termination of all data processing in orb-slam3
    sleep 1 &&

    pkill -INT Stereo_Inertial

    mv *Format.txt "$EXP_DIR"
    mv *file.txt   "$EXP_DIR"

done


for (( j=2; j<=NUM_IMU; j++ ))
do
    OUTER_DIR="$BASE_DIR"
    mkdir -p "$OUTER_DIR"


    # Create a new directory for this experiment's results
    EXP_DIR="$OUTER_DIR/imu_$j"
    mkdir -p "$EXP_DIR"

    # Launch ros orb-slam3
    rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt /home/ao/orbslam3/Examples_old/Stereo-Inertial/EuRoC.yaml true xx 1 "$j" &

    # wait for orb-slam3 to be ready
    sleep 12 &&

    # launch the rosbag replay
    BAG_NAME="/home/ao/Downloads/${BAG_NAMES[1]}"
    rosbag play "$BAG_NAME" /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu  &&

    # Wait for the termination of all data processing in orb-slam3
    sleep 1 &&

    pkill -INT Stereo_Inertial

    mv *Format.txt "$EXP_DIR"
    mv *file.txt   "$EXP_DIR"

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




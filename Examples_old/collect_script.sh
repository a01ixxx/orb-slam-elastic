#!/bin/bash

rosocre &


# Number of experiments
NUM_EXPERIMENTS=2

NUM_DATASET=5

# Base directory to store results
BASE_DIR="./results"

# Ensure the base directory exists
mkdir -p "$BASE_DIR"


for (( j=1; j<=NUM_DATASET; j++ ))
do
    OUTER_DIR="$BASE_DIR/MH0$j"
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
        rosbag play "MH_0$j_*.bag" /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu &&

        pkill Stereo_Inertial

        mv "*Format.txt" "$EXP_DIR"
        mv "*file.txt"   "$EXP_DIR"

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




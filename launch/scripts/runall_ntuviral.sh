#!/usr/bin/env bash
catkin_make -C /home/$USER/workspace/FAST_LIO_ws;
source /home/$USER/workspace/FAST_LIO_ws/devel/setup.bash # TODO: Please change this one based on your env
export EPOC_DIR=/home/$USER/bags/results/ntuviral_fastlio # TODO: Please change this one based on your env
export DATASET_LOCATION=/home/$USER/bags/ntu_viral/ # TODO: Please change this one based on your env


# Get the current directory
CURR_DIR=$(pwd)
# Get the location of the viral package
roscd fast_lio
PACKAGE_DIR=$(pwd)
# Return to the current dir, print the directions
cd $CURR_DIR
echo CURRENT DIR: $CURR_DIR
echo VIRAL DIR:   $PACKAGE_DIR
export CAPTURE_SCREEN=false;
export LOG_DATA=true;

#region 0 UWB NO VIS --------------------------------------------------------------------------------------------------

# export EPOC_DIR=$1;
# export DATASET_LOCATION=$2;
# export ROS_PKG_DIR=$3;
# export EXP_NAME=$4;
# export CAPTURE_SCREEN=$5;
# export LOG_DATA=$6;
# export LOG_DUR=$7;
# export FUSE_UWB=$8;
# export FUSE_VIS=$9;
# export UWB_BIAS=${10};
# export ANC_ID_MAX=${11};

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;

# # more seq !!! ====> 
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR rtp_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR rtp_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR rtp_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;

# wait;
# ./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR tnp_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
# wait;
# ./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR tnp_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
# wait;
# ./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR tnp_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;

# wait;
# ./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR spms_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
# wait;
# ./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR spms_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
# wait;
# ./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR spms_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
#endregion NO UWB NO VIS ----------------------------------------------------------------------------------------------


#region ## Poweroff ---------------------------------------------------------------------------------------------------

wait;
# poweroff

#endregion ## Poweroff ------------------------------------------------------------------------------------------------

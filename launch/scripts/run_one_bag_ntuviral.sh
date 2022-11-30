#!/bin/bash

export EPOC_DIR=$1;
export DATASET_LOCATION=$2;
export ROS_PKG_DIR=$3;
export EXP_NAME=$4;
export CAPTURE_SCREEN=$5;
export LOG_DATA=$6;
export LOG_DUR=$7;
export FUSE_UWB=$8;
export FUSE_VIS=$9;
export UWB_BIAS=${10};
export ANC_ID_MAX=${11};

export BAG_DUR=$(rosbag info $DATASET_LOCATION/$EXP_NAME/$EXP_NAME.bag | grep 'duration' | sed 's/^.*(//' | sed 's/s)//');
let LOG_DUR=BAG_DUR+20

echo "BAG DURATION:" $BAG_DUR "=> LOG_DUR:" $LOG_DUR;

let ANC_MAX=ANC_ID_MAX+1

export EXP_OUTPUT_DIR=$EPOC_DIR/result_${EXP_NAME}_${ANC_MAX}anc;
if ((FUSE_VIS==1))
then
export EXP_OUTPUT_DIR=${EXP_OUTPUT_DIR}_vis;
fi
echo OUTPUT DIR: $EXP_OUTPUT_DIR;

export BA_LOOP_LOG_DIR=/home/$USER;
if $LOG_DATA
then
export BA_LOOP_LOG_DIR=$EXP_OUTPUT_DIR;
fi
echo BA LOG DIR: $BA_LOOP_LOG_DIR;

mkdir -p $EXP_OUTPUT_DIR/ ;
cp -R $ROS_PKG_DIR/config $EXP_OUTPUT_DIR;
cp -R $ROS_PKG_DIR/launch $EXP_OUTPUT_DIR;

echo "Starting ROS LAUNCH HERE!!!"
roslaunch fast_lio mapping_NTUviral.launch autorun:=true \
bag_file:=$DATASET_LOCATION/$EXP_NAME/$EXP_NAME.bag \
& \

if $CAPTURE_SCREEN
then
echo CAPTURING SCREEN ON;
sleep 1;
ffmpeg -video_size 1920x1080 -framerate 5 -f x11grab -i :1.0 -qscale 0 \
-loglevel quiet -t $LOG_DUR -y /home/kin/DATA_HDD/$EXP_NAME.avi \
& \
else
echo CAPTURING SCREEN OFF;
sleep 1;
fi

if $LOG_DATA
then
sleep 5;
echo " ===========> LOGGING ON";
rosparam dump $EXP_OUTPUT_DIR/allparams.yaml;
timeout $LOG_DUR rostopic echo -b $DATASET_LOCATION/$EXP_NAME/$EXP_NAME.bag -p --nostr --noarr /leica/pose/relative > $EXP_OUTPUT_DIR/leica_pose.csv \
& \
timeout $LOG_DUR rostopic echo -b $DATASET_LOCATION/$EXP_NAME/$EXP_NAME.bag -p --nostr --noarr /imu/imu > $EXP_OUTPUT_DIR/vn100_imu.csv \
& \
timeout $LOG_DUR rostopic echo -p --nostr --noarr /Odometry > $EXP_OUTPUT_DIR/predict_odom.csv
# & \
# timeout $LOG_DUR rostopic echo -b $DATASET_LOCATION/$EXP_NAME/$EXP_NAME.bag -p --nostr --noarr /dji_sdk/imu > $EXP_OUTPUT_DIR/dji_sdk_imu.csv \
# & \
# timeout $LOG_DUR rostopic echo -p --nostr --noarr /viral2_odometry/optimization_status > $EXP_OUTPUT_DIR/optimization_status.csv \
# ;
else
echo LOGGING OFF;
sleep $LOG_DUR;
fi
echo "Move to next one run";
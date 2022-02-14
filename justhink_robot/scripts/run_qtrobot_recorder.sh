echo "Starting QTrobot ROS logger..."

# Define a timestamp function
timestamp() {
  date +"%Y-%m-%d_%H-%M-%S" # current time
}

STUDENT_NO=${1:-0}

FILE_NAME=${STUDENT_NO}_$(timestamp)
echo "$(timestamp): Record interaction at $FILE_NAME"

DIR=.

mkdir -p $DIR
ABSOLUTE_FILE_NAME=${DIR}/${FILE_NAME}

echo "$(timestamp): Begin recording."
rosbag record \
    --split --size 2048 \
    --output-name $ABSOLUTE_FILE_NAME \
    --lz4 \
    /camera/color/image_raw \
    \
    /qt_nuitrack_app/faces \
    /qt_nuitrack_app/gestures \
    /qt_nuitrack_app/hands \
    /qt_nuitrack_app/skeletons \
    \
    /qt_respeaker_app/channel0 \
    /qt_respeaker_app/is_speaking \
    /qt_respeaker_app/sound_direction \
    \
    /qt_robot/audio/play \
    /qt_robot/behavior/talkAudio \
    /qt_robot/behavior/talkText \
    /qt_robot/emotion/show \
    /qt_robot/gesture/play \
    /qt_robot/head_position/command \
    /qt_robot/joints/state \
    /qt_robot/left_arm_position/command \
    /qt_robot/motors/states \
    /qt_robot/right_arm_position/command \
    /qt_robot/speech/say

echo "$(timestamp): End recording."
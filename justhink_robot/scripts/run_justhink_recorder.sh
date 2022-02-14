echo "Starting JUSThink ROS logger..."

# Define a timestamp function
timestamp() {
  date +"%Y-%m-%d_%H-%M-%S" # current time
}

STUDENT_NO=${1:-0}

FILE_NAME=${STUDENT_NO}_$(timestamp)
echo "$(timestamp): Record interaction at $FILE_NAME"

DIR=$(rospack find justhink_robot)/data/rosbags

mkdir -p $DIR
ABSOLUTE_FILE_NAME=${DIR}/${FILE_NAME}

echo "$(timestamp): Begin recording."
rosbag record \
  --output-name $ABSOLUTE_FILE_NAME \
    /env/situation/act \
    /env/situation/event \
    /env/situation/outset \
    /env/situation/button_press \
    /env/situation/drawing_change \
    /env/situation/key_press \
    /env/situation/key_release \
    /env/situation/mouse_drag \
    /env/situation/mouse_motion \
    /env/situation/mouse_press \
    /env/situation/mouse_release \
    /agent/cognition/log_act \
    /agent/cognition/log_emote \
    /agent/cognition/log_express \
    /agent/cognition/log_observe_activity \
    /agent/cognition/log_observe_state \
    /agent/cognition/log_pause \
    /agent/cognition/log_say \
    /agent/cognition/log_set_activity \
    /agent/cognition/log_set_robot_text \
    /agent/embodiment/emote \
    /agent/embodiment/express \
    /agent/embodiment/say \
    /qt_robot/behavior/talkText \
    /qt_robot/gesture/play \
    /qt_robot/emotion/show \
    /rosout \
    /rosout_agg

echo "$(timestamp): End recording."
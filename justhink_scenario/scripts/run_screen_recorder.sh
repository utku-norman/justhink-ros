echo "Starting JUSThink screen logger..."

# Define a timestamp function
timestamp() {
  date +"%Y-%m-%d_%H-%M-%S" # current time
}

STUDENT_NO=${1:-0}

FILE_NAME=${STUDENT_NO}_$(timestamp).mp4
echo "$(timestamp): Record screen at $FILE_NAME"


DIR=~/data/screen_recordings
mkdir -p $DIR
ABSOLUTE_FILE_NAME=${DIR}/${FILE_NAME}

ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0+3840,0 -timestamp now $ABSOLUTE_FILE_NAME

echo "$(timestamp): End recording at at $FILE_NAME"
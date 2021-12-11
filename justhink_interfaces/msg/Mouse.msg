# This points to a point, left or right button and a timestamp
std_msgs/Header header            # timestamp in the header is the event time
                         # in frame frame_id, the activity's name is given
int16 x
int16 y
int16 dx
int16 dy
bool left
bool right

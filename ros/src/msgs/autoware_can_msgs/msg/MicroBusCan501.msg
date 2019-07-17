int8 DRIVE_MODE_NONE = 0
int8 DRIVE_MODE_STROKE = 1
int8 DRIVE_MODE_VELOCITY = 2

Header header
bool emergency
bool steer_auto
bool drive_auto
int8 drive_mode
int16 velocity
int16 steering_angle
int16 pedal
uint32 read_counter

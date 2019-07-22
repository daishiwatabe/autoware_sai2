int8 DRIVE_MODE_NONE = 0
int8 DRIVE_MODE_STROKE = 1
int8 DRIVE_MODE_VELOCITY = 2

int8 SHIFT_MANUAL = 0
int8 SHIFT_NOW_CHANGE = 15
int8 SHIFT_P = 8
int8 SHIFT_R = 9
int8 SHIFT_N = 10
int8 SHIFT_D = 11
int8 SHIFT_2 = 12
int8 SHIFT_L = 13

Header header
bool emergency
bool steer_auto
bool drive_auto
int8 drive_mode
int16 velocity
int16 steering_angle
int16 pedal
uint8 emergency_stop
bool engine_start
bool ignition
bool wiper
bool light_high
bool light_low
bool light_small
bool horn
bool hazard
bool blinker_right
bool blinker_left
uint8 shift
uint32 read_counter

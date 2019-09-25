int8 DRIVE_MODE_NONE = 0
int8 DRIVE_MODE_STROKE = 10
int8 DRIVE_MODE_VELOCITY = 11

int8 SHIFT_MANUAL = 0
int8 SHIFT_NOW_CHANGE = 15
int8 SHIFT_P = 8
int8 SHIFT_R = 9
int8 SHIFT_N = 10
int8 SHIFT_D = 11
int8 SHIFT_4 = 12
int8 SHIFT_L = 13

uint8 STEER_V0 = 0
uint8 STEER_AUTO = 10
uint8 STEER_NOT_V0 = 4
uint8 STEER_NOT_JOY_CENTER = 1
uint8 STEER_NOT_BOARD_RES = 8

uint8 DRIVE_V0 = 0
uint8 DRIVE_AUTO = 10
uint8 DRIVE_NOT_V0 = 4
uint8 DRIVE_NOT_JOY_CENTER = 1
uint8 DRIVE_NOT_BOARD_RES = 8

Header header
bool emergency
uint8 steer_auto
uint8 drive_auto
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
uint8 side_brake
uint8 automatic_door
uint32 read_counter

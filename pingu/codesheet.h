// Put this file, or a symbolic link to it, in an arduino library folder
// The code sent is an int32_t, a 32-bit number
#define CODE_BEAK_NOOT 100
#define CODE_BEAK_VOLDOWN 101
#define CODE_BEAK_VOLUP 102

#define CODE_BEAK_NOOT_BITMASK 0b001
#define CODE_BEAK_VOLUP_BITMASK 0b010
#define CODE_BEAK_VOLDOWN_BITMASK 0b100

// 4 bit sign-magnitude for each wheel. bit 24-27 for left wheel and bit 28-31 for right wheel.
#define WHEEL_BITMASK_UNSHIFTED 0b1111
#define WHEEL_LEFT_BITSHIFT 24
#define WHEEL_RIGHT_BITSHIFT 28

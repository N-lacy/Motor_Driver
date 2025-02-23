/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define GET_BAT        'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_BRAKE    'l'
#define MOTOR_SPEEDS   'm'
#define RELEASE_BRAKE  'n'
#define MOTOR_RAW_PWM  'o'
#define ULTRA_SONIC    'p'
#define RESET_ENCODERS 'r'
#define E_STATE        's'
#define TEMP_READ      't'
#define UPDATE_PID     'u'
#define E_STOP         'v'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define LEFT            0
#define RIGHT           1

#endif

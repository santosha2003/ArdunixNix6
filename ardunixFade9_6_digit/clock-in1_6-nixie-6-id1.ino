

#include <avr/io.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <Shifter.h>
#include <DS3231.h>             // https://github.com/NorthernWidget/DS3231 (Wickert 1.0.2) ? //install from lib manager!
#include <Time.h>
#include <TimeLib.h>            // https://github.com/michaelmargolis/arduino_time    //Установить через менеджер библиотек 


// Other parts of the code, broken out for clarity //make folder in arduino libraries - copy to folder
#include "ClockButton.h"
#include "Transition.h"

/*  name=DS3231
version=1.0.7
author=Andrew Wickert <awickert@umn.edu>, Eric Ayars, Jean-Claude Wippler, Northern Widget LLC <info@northernwidget.com>
maintainer=Andrew Wickert <awickert@umn.edu>
sentence=Arduino library for the DS3231 real-time clock (RTC)
paragraph=Abstracts functionality for clock reading, clock setting, and alarms for the DS3231 high-precision real-time clock. This is a splice of Ayars' (http://hacks.ayars.org/2011/04/ds3231-real-time-clock.html) and Jeelabs/Ladyada's (https://github.com/adafruit/RTClib) libraries.
category=Timing
url=https://github.com/NorthernWidget/DS3231
architectures=*
includes=DS3231.h   
*/

/* 

serial monitor ( battery voltage >3v) reset pin NC or 100k + to +5v
0 50 2 0 100 0 0
0 4 700 100 1 0
7 100 0 1000 2 1 1
0
50
verify this write to ds32321 - 7:34:56 year 21-5-2-doW-4
verify ds32321 - 25:165:165 year 117-85-165
0:9:48 flag rtc 1-1-t-41.68-21.50
DS3231_T=21
DS3231_rd0=48:9:0-7-1-1-0
0:09:59 1 1 2000
20
DS3231_T=128 41.45
wrong read DS3231!
DS3231_T=21
50
0 50 2 0 100 0 0
0 4 700 100 1 0
7 100 0 1000 2 1 1
0
50
verify this write to ds32321 - 2:22:29 year 0-1-2-doW-4
verify ds32321 - 2:22:29 year 208-1-2
2:22:29 flag rtc 1-2-t-41.45-21.00
DS3231_T_1-st-byte=21
DS3231_rd0=2:22:29-day-4-2-1-2000
2:23:00 2 1 2000
20
DS3231_T-2byte=21.0 F 41.45
DS3231_T=21.0

 
 */

#define SER_Pin 11 //SER_IN //data 14 pin 74hc595
#define RCLK_Pin 9 //L_CLOCK  //latch rclk 12 74hc595
#define SRCLK_Pin 8 //CLOCK //clock  srclk 11 74hc595

// RTC address
#define RTC_I2C_ADDRESS                 0x68
#define DS3231_I2C_ADDRESS                0x68

// Clock modes - normal running is MODE_TIME, other modes accessed by a middle length ( 1S < press < 2S ) button press
#define MODE_MIN                        0
#define MODE_TIME                       0

// Time setting, need all six digits, so no flashing mode indicator
#define MODE_HOURS_SET                  1
#define MODE_MINS_SET                   2
#define MODE_SECS_SET                   3
#define MODE_DAYS_SET                   4
#define MODE_MONTHS_SET                 5
#define MODE_YEARS_SET                  6

// Basic settings
#define MODE_12_24                      7  // 0 = 24, 1 = 12
#define HOUR_MODE_DEFAULT               false
#define MODE_LEAD_BLANK                 8  // 1 = blanked
#define LEAD_BLANK_DEFAULT              false
#define MODE_SCROLLBACK                 9  // 1 = use scrollback
#define SCROLLBACK_DEFAULT              false
#define MODE_FADE                       10 // 1 = use fade
#define FADE_DEFAULT                    false
#define MODE_DATE_FORMAT                11 // 
#define MODE_DAY_BLANKING               12 // 
#define MODE_HR_BLNK_START              13 // 
#define MODE_HR_BLNK_END                14 // 
#define MODE_SUPPRESS_ACP               15 // 1 = suppress ACP when fully dimmed
#define SUPPRESS_ACP_DEFAULT            true
#define MODE_USE_LDR                    16 // 1 = use LDR, 0 = don't (and have 100% brightness)
#define MODE_USE_LDR_DEFAULT            true
#define MODE_BLANK_MODE                 17 // 
#define MODE_BLANK_MODE_DEFAULT         BLANK_MODE_BOTH

// Display tricks
#define MODE_FADE_STEPS_UP              18 // 
#define MODE_FADE_STEPS_DOWN            19 // 
#define MODE_DISPLAY_SCROLL_STEPS_UP    20 // 
#define MODE_DISPLAY_SCROLL_STEPS_DOWN  21 // 
#define MODE_SLOTS_MODE                 22 // 

// I2C Interface definition
#define I2C_SLAVE_ADDR                 0x69
#define I2C_TIME_UPDATE                0x00
#define I2C_GET_OPTIONS                0x01
#define I2C_SET_OPTION_12_24           0x02
#define I2C_SET_OPTION_BLANK_LEAD      0x03
#define I2C_SET_OPTION_SCROLLBACK      0x04
#define I2C_SET_OPTION_SUPPRESS_ACP    0x05
#define I2C_SET_OPTION_DATE_FORMAT     0x06
#define I2C_SET_OPTION_DAY_BLANKING    0x07
#define I2C_SET_OPTION_BLANK_START     0x08
#define I2C_SET_OPTION_BLANK_END       0x09
#define I2C_SET_OPTION_FADE_STEPS      0x0a
#define I2C_SET_OPTION_SCROLL_STEPS    0x0b
#define I2C_SET_OPTION_BACKLIGHT_MODE  0x0c
#define I2C_SET_OPTION_RED_CHANNEL     0x0d
#define I2C_SET_OPTION_GREEN_CHANNEL   0x0e
#define I2C_SET_OPTION_BLUE_CHANNEL    0x0f
#define I2C_SET_OPTION_CYCLE_SPEED     0x10
#define I2C_SHOW_IP_ADDR               0x11
#define I2C_SET_OPTION_FADE            0x12
#define I2C_SET_OPTION_USE_LDR         0x13
#define I2C_SET_OPTION_BLANK_MODE      0x14
#define I2C_SET_OPTION_SLOTS_MODE      0x15
#define I2C_SET_OPTION_MIN_DIM         0x16

#define I2C_DATA_SIZE                  22
#define I2C_PROTOCOL_NUMBER            54

#define EE_12_24              1      // 12 or 24 hour mode
#define EE_FADE_STEPS         2      // How quickly we fade, higher = slower
#define EE_DATE_FORMAT        3      // Date format to display
#define EE_DAY_BLANKING       4      // Blanking setting
#define EE_DIM_DARK_LO        5      // Dimming dark value
#define EE_DIM_DARK_HI        6      // Dimming dark value
#define EE_BLANK_LEAD_ZERO    7      // If we blank leading zero on hours
#define EE_DIGIT_COUNT_HI     8      // The number of times we go round the main loop
#define EE_DIGIT_COUNT_LO     9      // The number of times we go round the main loop
#define EE_SCROLLBACK         10     // if we use scollback or not
#define EE_FADE               11     // if we use fade or not
#define EE_PULSE_LO           12     // The pulse on width for the PWM mode
#define EE_PULSE_HI           13     // The pulse on width for the PWM mode
#define EE_SCROLL_STEPS       14     // The steps in a scrollback
#define EE_BACKLIGHT_MODE     15     // The back light node
#define EE_DIM_BRIGHT_LO      16     // Dimming bright value
#define EE_DIM_BRIGHT_HI      17     // Dimming bright value
#define EE_DIM_SMOOTH_SPEED   18     // Dimming adaptation speed
#define EE_RED_INTENSITY      19     // Red channel backlight max intensity
#define EE_GRN_INTENSITY      20     // Green channel backlight max intensity
#define EE_BLU_INTENSITY      21     // Blue channel backlight max intensity
#define EE_HV_VOLTAGE         22     // The HV voltage we want to use
#define EE_SUPPRESS_ACP       23     // Do we want to suppress ACP during dimmed time
#define EE_HOUR_BLANK_START   24     // Start of daily blanking period
#define EE_HOUR_BLANK_END     25     // End of daily blanking period
#define EE_CYCLE_SPEED        26     // How fast the color cycling does it's stuff
#define EE_PWM_TOP_LO         27     // The PWM top value if we know it, 0xFF if we need to calculate
#define EE_PWM_TOP_HI         28     // The PWM top value if we know it, 0xFF if we need to calculate
#define EE_HVG_NEED_CALIB     29     // 1 if we need to calibrate the HVGenerator, otherwise 0
#define EE_MIN_DIM_LO         30     // The min dim value
#define EE_MIN_DIM_HI         31     // The min dim value
#define EE_ANTI_GHOST         32     // The value we use for anti-ghosting
#define EE_NEED_SETUP         33     // used for detecting auto config for startup. By default the flashed, empty EEPROM shows us we need to do a setup 
#define EE_USE_LDR            34     // if we use the LDR or not (if we don't use the LDR, it has 100% brightness
#define EE_BLANK_MODE         35     // blank tubes, or LEDs or both
#define EE_SLOTS_MODE         36     // Show date every now and again

// Display mode, set per digit
#define BLANKED  0
#define DIMMED   1
#define FADE     2
#define NORMAL   3
#define BLINK    4
#define SCROLL   5
#define BRIGHT   6
#define STATIC   7

// const byte rgb_backlight_curve[] = {0, 16, 32, 48, 64, 80, 99, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255};

#define DIGIT_COUNT 6                 // 6 ламп или 4

#define DATE_FORMAT_MIN                 0
#define DATE_FORMAT_YYMMDD              0
#define DATE_FORMAT_MMDDYY              1
#define DATE_FORMAT_DDMMYY              2
#define DATE_FORMAT_MAX                 2
#define DATE_FORMAT_DEFAULT             2

#define DAY_BLANKING_MIN                0
#define DAY_BLANKING_NEVER              0  // Don't blank ever (default)
#define DAY_BLANKING_WEEKEND            1  // Blank during the weekend
#define DAY_BLANKING_WEEKDAY            2  // Blank during weekdays
#define DAY_BLANKING_ALWAYS             3  // Always blank
#define DAY_BLANKING_HOURS              4  // Blank between start and end hour every day
#define DAY_BLANKING_WEEKEND_OR_HOURS   5  // Blank between start and end hour during the week AND all day on the weekend
#define DAY_BLANKING_WEEKDAY_OR_HOURS   6  // Blank between start and end hour during the weekends AND all day on week days
#define DAY_BLANKING_WEEKEND_AND_HOURS  7  // Blank between start and end hour during the weekend
#define DAY_BLANKING_WEEKDAY_AND_HOURS  8  // Blank between start and end hour during week days
#define DAY_BLANKING_MAX                8
#define DAY_BLANKING_DEFAULT            0
#define BLANK_MODE_MIN                  0
#define BLANK_MODE_TUBES                0  // Use blanking for tubes only 
#define BLANK_MODE_LEDS                 1  // Use blanking for LEDs only
#define BLANK_MODE_BOTH                 2  // Use blanking for tubes and LEDs
#define BLANK_MODE_MAX                  2
#define BLANK_MODE_DEFAULT              2

#define BACKLIGHT_MIN                   0  //подсветку отключил
#define BACKLIGHT_FIXED                 0   // Just define a colour and stick to it
#define BACKLIGHT_PULSE                 1   // pulse the defined colour
#define BACKLIGHT_CYCLE                 2   // cycle through random colours
#define BACKLIGHT_FIXED_DIM             3   // A defined colour, but dims with bulb dimming
#define BACKLIGHT_PULSE_DIM             4   // pulse the defined colour, dims with bulb dimming
#define BACKLIGHT_CYCLE_DIM             5   // cycle through random colours, dims with bulb dimming
#define BACKLIGHT_MAX                   5
#define BACKLIGHT_DEFAULT               0

#define CYCLE_SPEED_MIN                 4
#define CYCLE_SPEED_MAX                 64
#define CYCLE_SPEED_DEFAULT             10

// Display handling
#define DIGIT_DISPLAY_COUNT   1000 // 500 //1000 // The number of times to traverse inner fade loop per digit
#define DIGIT_DISPLAY_ON      0    // Switch on the digit at the beginning by default
#define DIGIT_DISPLAY_OFF    999 // 499  // Switch off the digit at the end by default
#define DIGIT_DISPLAY_NEVER   -1   // When we don't want to switch on or off (i.e. blanking)
#define DISPLAY_COUNT_MAX     1001 // Maximum value we can set to
#define DISPLAY_COUNT_MIN     300  // Minimum value we can set to

#define MIN_DIM_DEFAULT       120  // The default minimum dim count
#define MIN_DIM_MIN           10  // The minimum dim count
#define MIN_DIM_MAX           350  // The maximum dim count

#define SENSOR_LOW_MIN        0
#define SENSOR_LOW_MAX        900
#define SENSOR_LOW_DEFAULT    60 //100  // Dark
#define SENSOR_HIGH_MIN       0
#define SENSOR_HIGH_MAX       900
#define SENSOR_HIGH_DEFAULT   450 //700 // Bright

#define SENSOR_SMOOTH_READINGS_MIN     1
#define SENSOR_SMOOTH_READINGS_MAX     255
#define SENSOR_SMOOTH_READINGS_DEFAULT 100  // Speed at which the brighness adapts to changes

#define BLINK_COUNT_MAX                25   // The number of impressions between blink state toggle

// The target voltage we want to achieve  hv gen not use (mc34063)
#define HVGEN_TARGET_VOLTAGE_DEFAULT 180
#define HVGEN_TARGET_VOLTAGE_MIN     150
#define HVGEN_TARGET_VOLTAGE_MAX     200

// The PWM parameters
#define PWM_TOP_DEFAULT   10000
#define PWM_TOP_MIN       300
#define PWM_TOP_MAX       10000
#define PWM_PULSE_DEFAULT 200
#define PWM_PULSE_MIN     100
#define PWM_PULSE_MAX     500
#define PWM_OFF_MIN       50

// How quickly the scroll works
#define SCROLL_STEPS_DEFAULT 6
#define SCROLL_STEPS_MIN     1
#define SCROLL_STEPS_MAX     40

#define ANTI_GHOST_MIN                  0  //for hv gen // sets into eeprom config but not use
#define ANTI_GHOST_MAX                  50
#define ANTI_GHOST_DEFAULT              0

// The number of dispay impessions we need to fade by default
// 100 is about 1 second   //50 is 1 sec
#define FADE_STEPS_DEFAULT 42
#define FADE_STEPS_MAX     49
#define FADE_STEPS_MIN     5

#define SECS_MAX  60
#define MINS_MAX  60
#define HOURS_MAX 24

// Clock modes - normal running is MODE_TIME, other modes accessed by a middle length ( 1S < press < 2S ) button press
#define MODE_MIN                        0
#define MODE_TIME                       0

// Time setting, need all six digits, so no flashing mode indicator
#define MODE_HOURS_SET                  1
#define MODE_MINS_SET                   2
#define MODE_SECS_SET                   3
#define MODE_DAYS_SET                   4
#define MODE_MONTHS_SET                 5
#define MODE_YEARS_SET                  6


// HV generation
#define MODE_TARGET_HV_UP               28 // 
#define MODE_TARGET_HV_DOWN             29 // 
#define MODE_PULSE_UP                   30 // 
#define MODE_PULSE_DOWN                 31 // 

#define MODE_MIN_DIM_UP                 32 // 
#define MODE_MIN_DIM_DOWN               33 // 

#define MODE_ANTI_GHOST_UP              34 // 
#define MODE_ANTI_GHOST_DOWN            35 // 

// Temperature
#define MODE_TEMP                       36 // 

// Software Version
#define MODE_VERSION                    37 // 

// Tube test - all six digits, so no flashing mode indicator
#define MODE_TUBE_TEST                  38

#define MODE_MAX                        38

// Pseudo mode - burn the tubes and nothing else
#define MODE_DIGIT_BURN                 99 // Digit burn mode - accesible by super long press

// Temporary display modes - accessed by a short press ( < 1S ) on the button when in MODE_TIME
#define TEMP_MODE_MIN                   0
#define TEMP_MODE_DATE                  0 // Display the date for 5 S
#define TEMP_MODE_TEMP                  1 // Display the temperature for 5 S
#define TEMP_MODE_LDR                   2 // Display the normalised LDR reading for 5S, returns a value from 100 (dark) to 999 (bright)
#define TEMP_MODE_VERSION               3 // Display the version for 5S
#define TEMP_IP_ADDR12                  4 // IP xxx.yyy.zzz.aaa: xxx.yyy
#define TEMP_IP_ADDR34                  5 // IP xxx.yyy.zzz.aaa: zzz.aaa
#define TEMP_IMPR                       6 // number of impressions per second
#define TEMP_MODE_MAX                   6

#define DATE_FORMAT_MIN                 0
#define DATE_FORMAT_YYMMDD              0
#define DATE_FORMAT_MMDDYY              1
#define DATE_FORMAT_DDMMYY              2
#define DATE_FORMAT_MAX                 2
#define DATE_FORMAT_DEFAULT             2

#define DAY_BLANKING_MIN                0
#define DAY_BLANKING_NEVER              0  // Don't blank ever (default)
#define DAY_BLANKING_WEEKEND            1  // Blank during the weekend
#define DAY_BLANKING_WEEKDAY            2  // Blank during weekdays
#define DAY_BLANKING_ALWAYS             3  // Always blank
#define DAY_BLANKING_HOURS              4  // Blank between start and end hour every day
#define DAY_BLANKING_WEEKEND_OR_HOURS   5  // Blank between start and end hour during the week AND all day on the weekend
#define DAY_BLANKING_WEEKDAY_OR_HOURS   6  // Blank between start and end hour during the weekends AND all day on week days
#define DAY_BLANKING_WEEKEND_AND_HOURS  7  // Blank between start and end hour during the weekend
#define DAY_BLANKING_WEEKDAY_AND_HOURS  8  // Blank between start and end hour during week days
#define DAY_BLANKING_MAX                8
#define DAY_BLANKING_DEFAULT            0

#define BLANK_MODE_MIN                  0
#define BLANK_MODE_TUBES                0  // Use blanking for tubes only 
#define BLANK_MODE_LEDS                 1  // Use blanking for LEDs only
#define BLANK_MODE_BOTH                 2  // Use blanking for tubes and LEDs
#define BLANK_MODE_MAX                  2
#define BLANK_MODE_DEFAULT              2


#define CYCLE_SPEED_MIN                 4
#define CYCLE_SPEED_MAX                 64
#define CYCLE_SPEED_DEFAULT             10

#define ANTI_GHOST_MIN                  0
#define ANTI_GHOST_MAX                  50
#define ANTI_GHOST_DEFAULT              0

#define TEMP_DISPLAY_MODE_DUR_MS        5000

#define USE_LDR_DEFAULT                 true  //light detect resistor  // фоторезистор если сделаю на статике регулировку без дерганий
                                       //  можно снижать напряжение на  mc34063 или пробовать гасить цифры и включать 1 - 2 -3 -4 -5 циклов из 10 
                                        // сейчас время цикла 50 мс - это может быть дергание - убрать задержку и будет 6 - 10 мс что не заметно сильно мерцание 

// Limit on the length of time we stay in test mode
#define TEST_MODE_MAX_MS                60000

#define SLOTS_MODE_MIN                  0
#define SLOTS_MODE_NONE                 0   // Don't use slots effect
#define SLOTS_MODE_1M_SCR_SCR           1   // use slots effect every minute, scroll in, scramble out
#define SLOTS_MODE_MAX                  1
#define SLOTS_MODE_DEFAULT              1

#define MAX_WIFI_TIME                   5

#define DO_NOT_APPLY_LEAD_0_BLANK     false
#define APPLY_LEAD_0_BLANK            true


// button input
#define inputPin1   7     // package pin 13 // PD7  //d7    - Single button operation with software debounce   


#define sensorPin   A0    // Package pin 23 // PC0 // Analog input pin for HV sense: HV divided through 390k and 4k7 divider, using 5V reference
#define LDRPin      A1    // Package pin 24 // PC1 // Analog input for Light dependent resistor.
//**********************************************************************************

Transition transition(500, 1000, 3000);
// ********************** HV generator variables *********************
int hvTargetVoltage = HVGEN_TARGET_VOLTAGE_DEFAULT;
int pwmTop = PWM_TOP_DEFAULT;
int pwmOn = PWM_PULSE_DEFAULT;

// All-In-One Rev1 has a mix up in the tube wiring. All other clocks are 
// correct.
#define NOT_AIO_REV1 // [AIO_REV1,NOT_AIO_REV1]

// Used for special mappings of the K155ID1 -> digit (wiring aid)
// allows the board wiring to be much simpler
#ifdef AIO_REV1 
  // This is a mapping for All-In-One Revision 1 ONLY! Not generally used.
  byte decodeDigit[16] = {3,2,8,9,0,1,5,4,6,7,10,10,10,10,10,10};
#else
  byte decodeDigit[16] = {2,3,7,6,4,5,1,0,9,8,10,10,10,10,10,10};
#endif

// Driver pins for the anodes
//byte anodePins[6] = {ledPin_a_1, ledPin_a_2, ledPin_a_3, ledPin_a_4, ledPin_a_5, ledPin_a_6};

// precalculated values for turning on and off the HV generator
// Put these in TCCR1B to turn off and on
byte tccrOff;
byte tccrOn;
int rawHVADCThreshold;
double sensorHVSmoothed = 0;

#define NUM_REGISTERS 3 //how many registers are in the chain

bool debug = false;    // if (debug)  // works without serial connection if debug=false

byte zero = 0x00;
int RTC_hours, RTC_minutes, RTC_seconds, RTC_day, RTC_month, RTC_year, RTC_day_of_week;  // use Clock. (DS3231 lib), RTClib for subroutine code from in12 project

// ************************ Display management ************************
byte NumberArray[6]    = {0, 0, 0, 0, 0, 0};
byte currNumberArray[6] = {10, 10, 10, 10, 10, 10};
byte displayType[6]    = {FADE, FADE, FADE, FADE, FADE, FADE};
byte fadeState[6] = {0, 0, 0, 0, 0, 0};
byte OnOff[6]    = {1, 1, 1, 1, 1, 1};    //current state of digit at this time - use for dimming - blink - fade 
byte ourIP[4]          = {0, 0, 0, 0}; // set by the WiFi module, if attached
byte threesectimer = 0;
  int digitOnTime[6];
  int digitOffTime[6];
  int digitSwitchTime[6]= {0, 0, 0, 0, 0, 0};
// how many fade steps to increment (out of DIGIT_DISPLAY_COUNT) each impression
// 100 // 50 is about 1 second
int fadeSteps = FADE_STEPS_DEFAULT;  //45
int digitOffCount = DIGIT_DISPLAY_OFF;  //999
int scrollSteps = SCROLL_STEPS_DEFAULT;
boolean scrollback = true;
boolean fade = true;
byte antiGhost = ANTI_GHOST_DEFAULT;  // not use - approx. 189 v max 
int dispCount = DIGIT_DISPLAY_COUNT + antiGhost;
float fadeStep = digitOffCount / fadeSteps;  //DIGIT_DISPLAY_COUNT  //int fadeStep

// For software blinking
int blinkCounter = 0;
boolean blinkState = true;

// leading digit blanking
boolean blankLeading = false;

// Dimming value
//const int DIM_VALUE = DIGIT_DISPLAY_COUNT / 5;  //200  
 int DIM_VALUE = 25;  //200 is too bright  
int minDim = MIN_DIM_DEFAULT;

unsigned int tempDisplayModeDuration;      // time for the end of the temporary display
int  tempDisplayMode;

int acpOffset = 0;        // Used to provide one arm bandit scolling
int acpTick = 0;          // The number of counts before we scroll
boolean suppressACP = true;
byte slotsMode = SLOTS_MODE_DEFAULT;

byte currentMode = MODE_TIME;   // Initial cold start mode
byte nextMode = currentMode;
byte blankHourStart = 0;
byte blankHourEnd = 0;
byte blankMode = 0;
boolean blankTubes = false;
boolean blankLEDs = false;

// ************************ Ambient light dimming ************************
int dimDark = SENSOR_LOW_DEFAULT;
int dimBright = SENSOR_HIGH_DEFAULT;
double sensorLDRSmoothed = 0;
double sensorFactor = (double)(DIGIT_DISPLAY_OFF) / (double)(dimBright - dimDark);
int sensorSmoothCountLDR = SENSOR_SMOOTH_READINGS_DEFAULT;
int sensorSmoothCountHV = SENSOR_SMOOTH_READINGS_DEFAULT / 8;
boolean useLDR = true;

DS3231 Clock;

//initaize shifter using the Shifter library
Shifter shifter(SER_Pin, RCLK_Pin, SRCLK_Pin, NUM_REGISTERS); 

// State variables for detecting changes
byte lastSec;
unsigned long nowMillis = 0;
unsigned long lastCheckMillis = 0;

byte dateFormat = DATE_FORMAT_DEFAULT;
byte dayBlanking = DAY_BLANKING_DEFAULT;
boolean blanked = false;
byte blankSuppressStep = 0;    // The press we are on: 1 press = suppress for 1 min, 2 press = 1 hour, 3 = 1 day
unsigned long blankSuppressedMillis = 0;   // The end time of the blanking, 0 if we are not suppressed
unsigned long blankSuppressedSelectionTimoutMillis = 0;   // Used for determining the end of the blanking period selection timeout
boolean hourMode = false;
boolean triggeredThisSec = false;

byte useRTC = true;  // true if we detect an RTC  //now use  - true
byte useWiFi = 0; // the number of minutes ago we recevied information from the WiFi module, 0 = don't use WiFi
byte cycleCount = 0;
byte cycleSpeed = CYCLE_SPEED_DEFAULT;

  //byte Currentday;
  //DateTime now = RTC.now(); Currentday = now.day();     // RTClib not use - now DS3231
  //if(Currentday == Startday){ 
int impressionsPerSec = 0;  //loop time (20 per second now - delay 50 added) 
int lastImpressionsPerSec = 0;

// ********************** Input switch management **********************
ClockButton button1(inputPin1, false);                                 // one button

// **************************** digit healing ****************************
// This is a special mode which repairs cathode poisoning by driving a
// single element at full power. To be used with care!
// In theory, this should not be necessary because we have ACP every 10mins,
// but some tubes just want to be poisoned
byte digitBurnDigit = 0;
byte digitBurnValue = 0;


/*
// create an array that translates decimal numbers into an appropriate byte for sending to the shift register
int charTable[] = {0,128,64,192,32,160,96,224,16,144,8,136,72,200,40,168,104,232,24,152,4,132,68,196,36,164,100,228,20,148,12,140,76,204,44,
172,108,236,28,156,2,130,66,194,34,162,98,226,
18,146,10,138,74,202,42,170,106,234,26,154,6,134,70,198,38,166,102,230,22,150,14,142,78,206,46,174,110,238,30,158,1,129,
65,193,33,161,97,225,17,145,9,137,73,201,41,169,105,233,25,153};
*/

byte nixies = 255; //initiate the byte to be sent to the shift register and set it to blank the nixies
int x; //create a counting variable
  byte secondes = 86;
//byte
byte minutes = 79;
//byte
byte hours = 78;

// **********************************************************************************
// **********************************************************************************
// *                                    Setup                                       *
// **********************************************************************************
// **********************************************************************************
void setup(){
  Wire.begin();     // init twowire sda scl pins A4 A5  - just connect 2 LED from 5v 300 Ohm resistor - see how works
 if (Serial)  debug=true;
    if (debug)Serial.begin(57600);  //de bug to serial Monitor  port - in the right corner - press ctrl -m - serial speed 57000
  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);
    // Grounding the input pin causes it to actuate
  pinMode(inputPin1, INPUT_PULLUP ); // set the input pin 1    // add resistor
 
  // вернул настройку HV  и сброс до заводских настроек (нажать кнопку при включении)
  /* disable global interrupts while we set up them up */
  cli();

  // **************************** HV generator ****************************

  TCCR1A = 0;    // disable all PWM on Timer1 whilst we set it up
  TCCR1B = 0;    // disable all PWM on Timer1 whilst we set it up

  // Configure timer 1 for Fast PWM mode via ICR1, with prescaling=1
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

  tccrOff = TCCR1A;

  TCCR1A |= (1 <<  COM1A1);  // enable PWM on port PD4 in non-inverted compare mode 2

  tccrOn = TCCR1A;

  // Set up timer 2 like timer 0 (for RGB leds)
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << CS22);

  // we don't need the HV yet, so turn it off
  TCCR1A = tccrOff;

  /* enable global interrupts */
  sei();

  // **********************************************************************

  // Set up the PRNG with something so that it looks random
  randomSeed(analogRead(LDRPin));

  // Test if the button is pressed for factory reset
  for (int i = 0 ; i < 20 ; i++ ) {
    button1.checkButton(nowMillis);
  }

  // User requested factory reset
  if (button1.isButtonPressedNow()) {
    // do this before the flashing, because this way we can set up the EEPROM to
    // autocalibrate on first start (press factory reset and then power off before
    // flashing ends)
    EEPROM.write(EE_HVG_NEED_CALIB, true);

    // Flash some ramdom c0lours to signal that we have accepted the factory reset
 /*   for (int i = 0 ; i < 60 ; i++ ) {
      randomRGBFlash(20);
     }

  */
    // mark that we need the EEPROM setup
    EEPROM.write(EE_NEED_SETUP, true);
  }

  // If the button is held down while we are flashing, then do the test pattern
  boolean doTestPattern = false;

  // Detect factory reset: button pressed on start or uninitialised EEPROM
  if (EEPROM.read(EE_NEED_SETUP)) {
    doTestPattern = true;
  }

  // Clear down any spurious button action
  button1.reset();

  // Test if the button is pressed for factory reset
  for (int i = 0 ; i < 20 ; i++ ) {
    button1.checkButton(nowMillis);
  }

  if (button1.isButtonPressedNow()) {
    doTestPattern = true;
  } 
   if (debug) Serial.println("Vklu4ilsja i'M Citten Lynx");
// comment - first run
// factoryReset();
 EEPROM.write(EE_FADE_STEPS, fadeSteps);  //46 default or 42
  blankLeading =1;
  fadeSteps=3;
    // Read EEPROM values    (test - reads! -2 lines below)
  readEEPROMValues();
 if (debug) {
  Serial.print("eeprom internal read 2 byte blankLeading fadeSteps - ");
   Serial.print(blankLeading); 
   Serial.print(" ");//test what reads
     Serial.println(fadeSteps);  //test what reads
 }
     // fadeSteps=49; // for test    now 52 // 46 big cycles per second *1000 display timer  46000 установок регистров в секунду, а если 2 цифры сразу то 100000 (fade)  


  // initialise the internal time (in case we don't find the time provider)  первая установка времени здесь
 //1-st try  getRTCTime();   // try to autodetect 
  // Startday = now.day();
  
    // de-bug - rtc ds3231 works - uncomment line run? comment line - run again 
 //(comment - after first run) 
 setTime(2, 48, 47, 5, 6, 2021);  // ! и оно работает  16 37 36 через 3 минуты     Works - manual set time
    // de-bug - rtc ds3231 works - uncomment line run? comment line - run again   !! works!! time read OK from DS3231 chip
 useRTC = true;   // autodetect near 
 
  DateTime compiled = DateTime(__DATE__, __TIME__);  
  //set time by current (DateTime compiled or manual)  run once - comment line   
  
  byte dayOfWeek= weekday(); //3   // 0- sunday воскресенье установка первый раз - вручную  !! 1 раз после настройки  
 // setDS3231time(second(), minute(), hour(), dayOfWeek, day(), month(), year());  // установка часов DS3231 по времени компьютера  
 
 // setRTC();   //ok works - 1-st run

/*
        Clock.setMinute(24);//Set the minute 
        Clock.setHour(19);  //Set the hour 
        Clock.setDoW(5);    //Set the day of the week
        Clock.setDate(4);  //Set the date of the month
        Clock.setMonth(6);  //Set the month of the year
        Clock.setYear(21);  //Set the year (Last two digits of the year)
                Clock.setSecond(50);//Set the second  - last - start clock    // ok 1-st run set time
 */
//  useRTC = false;


byte year2digit=year() -2000;
//setDS3231time(second(), minute(), hour(), dayOfWeek, dayOfMonth, decToBcd1(month()), decToBcd1(year()));
   if (debug){
     Serial.print("setup now read from ds32321 - ");
          Serial.print(hour()); 
     Serial.print(":");
    Serial.print(minute());     
       Serial.print(":");
       Serial.print(second());
 Serial.print(" year ");
    Serial.print(year2digit);
    Serial.print("-");
    Serial.print(month());
     Serial.print("-");  
    Serial.print(day());    
 Serial.print("-doW-");
    Serial.println(dayOfWeek);
  }
  


/*
// установка значений времени (попытка)   achtung
// achtung this routine works - arduino set time into Maxim DS3231 (vcc=5, vbat=3 - 2 diodes, sda pin 15 to arduino nano A4, scl 16 - A5, R pullup 5k6 , R reset pin pull up 100k)
 dayOfWeek=  Clock.getDoW();   // get or set 
   Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(zero); //stop Oscillator
// bool oscillatorCheck();;
// Clock.setDoW(dayOfWeek);
Clock.setMinute(minute());
 Clock.setHour(hour());
 
 Clock.setDoW(dayOfWeek);
 Clock.setDate(day()); 
 Clock.setMonth(month());
 Clock.setYear(year2digit); 
 Clock.setClockMode(false);  //24h
 Clock.enableOscillator(true, true, 0);
 Clock.setSecond(second());  //enable OSC clear flag  
*/
     bool PM;
    bool twentyFourHourClock;
    bool century = false;

    int years = Clock.getYear() + 2000;
    byte months = Clock.getMonth(century);
    byte days = Clock.getDate();
     byte hours = Clock.getHour(twentyFourHourClock, PM);
    byte mins = Clock.getMinute();
    byte secs = Clock.getSecond();
  //  setTime(hours, mins, secs, days, months, years);

    // Make sure the clock keeps running even on battery
    if (!Clock.oscillatorCheck())
      Clock.enableOscillator(true, true, 0);

       // если зависает то смотреть монитор порта и закомментировать строку ниже  i found what may hung up 
 if (useRTC)  getRTCTime();   
 
   if (debug){
     Serial.print("verify ds32321 - ");
      Serial.print(hours);
      Serial.print(":");
         Serial.print(mins);
       Serial.print(":");
       Serial.print(secs);
 Serial.print(" year ");
    Serial.print(years);
    Serial.print("-");
    Serial.print(months);
    Serial.print("-");
    Serial.println(days);
  }
   if (doTestPattern) {
    boolean oldUseLDR = useLDR;
//    byte oldBacklightMode = backlightMode;

    // reset the EEPROM values
    factoryReset();

    // turn off LDR
    useLDR = false;

    // turn off Scrollback
    scrollback = false;

    // All the digits on full
    allBright();

    int secCount = 0;
    lastCheckMillis = millis();

    boolean inLoop = true;

    // We don't want to stay in test mode forever
    long startTestMode = lastCheckMillis;

    while (inLoop) {
      nowMillis = millis();
      if (nowMillis - lastCheckMillis > 1000) {
                lastCheckMillis = nowMillis;
        secCount++;
        secCount = secCount % 10;
      }

      // turn off test mode
      blankTubes = (nowMillis - startTestMode > TEST_MODE_MAX_MS);

      loadNumberArraySameValue(secCount);
      outputDisplay();
          //  checkHVVoltage();

//      setLedsTestPattern(nowMillis);
      button1.checkButton(nowMillis);

      if (button1.isButtonPressedNow() && (secCount == 8)) {
        inLoop = false;
        blankTubes = false;
      }
    }

    useLDR = oldUseLDR;
  //  backlightMode = oldBacklightMode;
  }
/* debug eeprom values 0 50 2 0 100 0 0
0 4 700 100 1 0
7 100 0 1000 2 1 1
eeprom internal read 2 byte blankLeading fadeSteps - 0 50
setup now read from ds3232
1 - 12:56:47 year 21-6-3-doW-5
verify ds32321 - 12:56:47 year 2021-6-3
12:56:47 flag rtc 1-3-t-45.84-30.75
DS3231_T_1-st-byte=30
12:57:00 3 6 2021
20
DS3231_T-2byte=30.192 F 45.50
DS3231_T=30.75   */

 // time rtc set OK

  /*    void setSecond(byte Second); //ds3231 lib readme Eric Ayars
      // In addition to setting the seconds, this clears the 
      // "Oscillator Stop Flag".
    void setMinute(byte Minute); 
      // Sets the minute
    void setHour(byte Hour); 
      // Sets the hour
    void setDoW(byte DoW); 
      // Sets the Day of the Week (1-7);
    void setDate(byte Date); 
      // Sets the Date of the Month
    void setMonth(byte Month); 
      // Sets the Month of the year
    void setYear(byte Year); 
      // Last two digits of the year
    void setClockMode(bool h12); 
      // Set 12/24h mode. True is 12-h, false is 24-hour.

    // Temperature function

    float getTemperature();  


    // another RTC lib 
    #include <RTClib.h>     // Date, Time and Alarm functions by https://github.com/MrAlvin/RTClib

    //AT24C32 I2C eeprom 
//******************
#define EEPROM_ADDRESS 0x57               // I2C Buss address of AT24C32 32K EEPROM (first block)
#define EEPromPageSize 32                 // 32 bytes is page size for the AT24C32
unsigned int CurrentPageStartAddress = 0; // set to zero at the start of each cycle
char EEPROMBuffer[28];                    // this buffer contains a string of ascii because I am using the pstring function to load it
uint8_t BytesWrittentoSD = 0;

//DS3231 RTC
//**********
#define DS3231_ADDRESS 0x68                 //=104 dec
#define DS3231_STATUS_REG 0x0f
#define DS3231_CTRL_REG 0x0e
#define Bit0_MASK         B00000001        //Bit 0=Alarm 1 Flag (A1F)
#define Bit1_MASK         B00000010        //Bit 1 = Alarm 2 Flag (A2F)
#define Bit2_MASK         B00000100
#define Bit3_MASK         B00001000        //Bit 3: Enable/disable 32kHz Output (EN32kHz) - has no effect on sleep current
#define Bit4_MASK         B00010000        //Bit 4: Bits 4&5 of status reg adjust Time Between Temperature Updates  see http://www.maximintegrated.com/en/app-notes/index.mvp/id/3644
#define Bit5_MASK         B00100000        //Bit 5:
#define Bit6_MASK         B01000000
#define Bit7_MASK         B10000000
#define RTCPOWER_PIN 7                      //When the arduino is awake, power the rtc from this pin (draws ~70uA), when arduino sleeps pin set low & rtc runs on battery at <3uA
                                            // SEE http://www.gammon.com.au/forum/?id=11497 for an example powering ds1307 from pin, alarms still work!
RTC_DS3231 RTC;                            //DS3231 will function with a VCC ranging from 2.3V to 5.5V
byte Alarmhour = 1;
byte Alarmminute = 1;
byte Alarmday = 1;                         //only used for sub second alarms
byte Alarmsecond = 1;                      //only used for sub second alarms
byte INTERRUPT_PIN = 2;                    // SQW is soldered to this pin on the arduino
volatile boolean clockInterrupt = false;
char CycleTimeStamp[ ]= "0000/00/00,00:00"; //16 characters without seconds!
byte Startday;

      Serial.begin(9600);
  Wire.begin();
  RTC.begin();

  // check RTC   //another library
  //**********
  clearClockTrigger(); //stops RTC from holding the interrupt low if system reset just occured
  RTC.turnOffAlarm(1);
  i2c_writeRegBits(DS3231_ADDRESS,DS3231_STATUS_REG,0,Bit3_MASK); // disable the 32khz output  pg14-17 of datasheet  //This does not reduce the sleep current
  i2c_writeRegBits(DS3231_ADDRESS,DS3231_STATUS_REG,1,Bit4_MASK); // see APPLICATION NOTE 3644 - this might only work on the DS3234?
  i2c_writeRegBits(DS3231_ADDRESS,DS3231_STATUS_REG,1,Bit5_MASK); // setting bits 4&5 to 1, extends the time between RTC temp updates to 512seconds (from default of 64s)
  DateTime now = RTC.now();
  Startday = now.day();
  DateTime compiled = DateTime(__DATE__, __TIME__);  
  if (now.unixtime() < compiled.unixtime()) { //checks if the RTC is not set yet
    Serial.println(F("RTC is older than compile time! Updating"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
    Serial.println(F("Clock updated...."));
    DateTime now = RTC.now();
    Startday = now.day(); //get the current day for the error routine
  }
    
    */
    
 //   getRTCTime();  //comment if stop responding

    float c = Clock.getTemperature();
     float f = c / 4. * 9. / 5. + 32.;    // Fahrenheit

 if (debug){
      Serial.print(hour());
      Serial.print(":");
         Serial.print(minute());
       Serial.print(":");
       Serial.print(second());
 Serial.print(" flag rtc ");
    Serial.print(useRTC);
    Serial.print("-");
    Serial.print(Clock.getDate());
    Serial.print("-t-");
     Serial.print(f);
        Serial.print("-");
    Serial.println(c);
  }
// Clear down any spurious button action
  button1.reset();

  // initialise the internal time (in case we don't find the time provider)
  nowMillis = millis();
 // setTime(12, 34, 56, 1, 3, 2017);
 // getRTCTime();

  // Show the version for 1 s
  tempDisplayMode = TEMP_MODE_VERSION;
  tempDisplayModeDuration = TEMP_DISPLAY_MODE_DUR_MS;

  // don't blank anything right now
  blanked = false;
  setTubesAndLEDSBlankMode();

  // mark that we have done the EEPROM setup
  EEPROM.write(EE_NEED_SETUP, false);

  // enable watchdog
  wdt_enable(WDTO_8S);
 

}

void loop(){

        nowMillis = millis();
          // shows us how fast the inner loop is running
  impressionsPerSec++;

  if (abs(nowMillis - lastCheckMillis) >= 1000) {
    if ((second() == 0) && (!triggeredThisSec)) {
      if ((minute() == 0)) {
        if (hour() == 0) {
      //    performOncePerDayProcessing();
        }
        
        performOncePerHourProcessing();
      }
      
      performOncePerMinuteProcessing();
    }

    performOncePerSecondProcessing();

    // Make sure we don't call multiple times
    triggeredThisSec = true;
    
    if ((second() > 0) && triggeredThisSec) {
      triggeredThisSec = false;
    }

    lastCheckMillis = nowMillis;
  }
  // byte second, minute, hour;
   //  shifter.setAll(HIGH);

 //minutes = 77;
 //hours = 77;
// time for cycle (impressions)
//delay(50);                                   // 


 // Check button, we evaluate below
  button1.checkButton(nowMillis);

  // ******* Preview the next display mode *******
  // What is previewed here will get actioned when
  // the button is released
  if (button1.isButtonPressed2S()) {
  if (debug){
  Serial.println(" butt 2s ");
  }  
    // Just jump back to the start
    nextMode = MODE_MIN;
  } else if (button1.isButtonPressed1S()) {
    nextMode = currentMode + 1;
if (debug){
  Serial.println(" butt 1s ");
  }
    if (nextMode > MODE_MAX) {
      nextMode = MODE_MIN;
    }
  }

  // ******* Set the display mode *******
  if (button1.isButtonPressedReleased8S()) {
    // 8 Sec press toggles burn mode
    if (currentMode == MODE_DIGIT_BURN) {
      currentMode = MODE_MIN;
    } else {
      currentMode = MODE_DIGIT_BURN;
    }

    nextMode = currentMode;
  } else if (button1.isButtonPressedReleased2S()) {
    currentMode = MODE_MIN;

    // Store the EEPROM if we exit the config mode
    saveEEPROMValues();

    // Preset the display
    allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);

    nextMode = currentMode;
  } else if (button1.isButtonPressedReleased1S()) {
    if (debug){
  Serial.println(" button 1s p-r ");
  }
    currentMode = nextMode;

    if (currentMode > MODE_MAX) {
      currentMode = MODE_MIN;

      // Store the EEPROM if we exit the config mode
      saveEEPROMValues();

      // Preset the display
      allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);
    }

    nextMode = currentMode;
  }

  // -------------------------------------------------------------------------------

  // ************* Process the modes *************
  if (nextMode != currentMode) {
    setNextMode(nextMode);
  } else {
    processCurrentMode(currentMode);
  }
  // get the LDR ambient light reading
  digitOffCount = getDimmingFromLDR();   // 120 dark (night)  999 bright - day
  fadeStep = digitOffCount / fadeSteps;   // 3 -dark ... 20 bright
 
  // One armed bandit trigger every 10th minute
  if ((currentMode != MODE_DIGIT_BURN) && (nextMode != MODE_DIGIT_BURN)) {
    if (acpOffset == 0) {
      if (((minute() % 10) == 9) && (second() == 15)) {
        // suppress ACP when fully dimmed
        if (suppressACP) {
          if (digitOffCount > minDim) {
            acpOffset = 0;        //  0 - не дергаю однорукого бандита  acp off
          }
        } else {
          acpOffset = 1;   //  0 ? 1 is anti poison on // effects at 15 +50 seconds - do anti-poison too
        }
      }
    }

    // One armed bandit handling
    if (acpOffset > 0) {
      if (acpTick >= acpOffset) {
        acpTick = 0;
        acpOffset++;
        if (acpOffset == 7) {
          acpOffset = 0;
        }
      } else {
        acpTick++;
      }
    }

    // Set normal output display
    outputDisplay();
  } else {
    // Digit burn mode

    // Santosha - turn on digit using Shifter // but useless - current set as normal light // dynamic mode - max bright
 OnOff[0]=0;
 OnOff[1]=0;
 OnOff[2]=0;
 OnOff[3]=0;
 OnOff[4]=0;
 OnOff[5]=0;
    OnOff[digitBurnDigit] =1;
//    digitOn(digitBurnDigit, digitBurnValue,10,10,10,10,10,10);
 digitOn(digitBurnDigit, digitBurnValue);
  }
  
  // -------------------------------------------------------------------------------
  // Prepare the tick and backlight LEDs
  //setLeds();
}  //main loop xwost tail

// modes - set time or display clock (press button for 1 s)
// ************************************************************
// Show the preview of the next mode - we stay in this mode until the 
// button is released
// ************************************************************
void setNextMode(int displayMode) {
  // turn off blanking
  blanked = false;
  setTubesAndLEDSBlankMode();

  switch (displayMode) {
    case MODE_TIME: {
        loadNumberArrayTime();
        allFadeOrNormal(APPLY_LEAD_0_BLANK);
        break;
      }
    case MODE_HOURS_SET: {
        if (useWiFi > 0) {
          setNewNextMode(MODE_12_24);
        }
        loadNumberArrayTime();
        highlight0and1();
        break;
      }
    case MODE_MINS_SET: {
        loadNumberArrayTime();
        highlight2and3();
        break;
      }
    case MODE_SECS_SET: {
        loadNumberArrayTime();
        highlight4and5();
        break;
      }
    case MODE_DAYS_SET: {
        loadNumberArrayDate();
        highlightDaysDateFormat();
        break;
      }
    case MODE_MONTHS_SET: {
        loadNumberArrayDate();
        highlightMonthsDateFormat();
        break;
      }
    case MODE_YEARS_SET: {
        loadNumberArrayDate();
        highlightYearsDateFormat();
        break;
      }
    case MODE_12_24: {
        loadNumberArrayConfBool(hourMode, displayMode);
        displayConfig();
        break;
      }
    case MODE_LEAD_BLANK: {
        loadNumberArrayConfBool(blankLeading, displayMode);
        displayConfig();
        break;
      }
    case MODE_SCROLLBACK: {
        loadNumberArrayConfBool(scrollback, displayMode);
        displayConfig();
        break;
      }
    case MODE_FADE: {
        loadNumberArrayConfBool(fade, displayMode);
        displayConfig();
        break;
      }
    case MODE_DATE_FORMAT: {
        loadNumberArrayConfInt(dateFormat, displayMode);
        displayConfig();
        break;
      }
    case MODE_DAY_BLANKING: {
        loadNumberArrayConfInt(dayBlanking, displayMode);
        displayConfig();
        break;
      }
    case MODE_HR_BLNK_START: {
        if ((dayBlanking == DAY_BLANKING_NEVER) ||
            (dayBlanking == DAY_BLANKING_WEEKEND) ||
            (dayBlanking == DAY_BLANKING_WEEKDAY) ||
            (dayBlanking == DAY_BLANKING_ALWAYS)) {
          // Skip past the start and end hour if the blanking mode says it is not relevant
          setNewNextMode(MODE_SUPPRESS_ACP);
        }
        loadNumberArrayConfInt(blankHourStart, displayMode);
        displayConfig();
        break;
      }
    case MODE_HR_BLNK_END: {
        loadNumberArrayConfInt(blankHourEnd, displayMode);
        displayConfig();
        break;
      }
    case MODE_SUPPRESS_ACP: {
        loadNumberArrayConfBool(suppressACP, displayMode);
        displayConfig();
        break;
      }
    case MODE_USE_LDR: {
        loadNumberArrayConfBool(useLDR, displayMode);
        displayConfig();
        break;
      }
    case MODE_BLANK_MODE: {
        loadNumberArrayConfInt(blankMode, displayMode);
        displayConfig();
        break;
      }
    case MODE_FADE_STEPS_UP:
    case MODE_FADE_STEPS_DOWN: {
        loadNumberArrayConfInt(fadeSteps, displayMode);  //50 .. 45   not more than 1 second = 46 // 49 cycles
        displayConfig();
        break;
      }
    case MODE_DISPLAY_SCROLL_STEPS_UP:
    case MODE_DISPLAY_SCROLL_STEPS_DOWN: {
        loadNumberArrayConfInt(scrollSteps, displayMode);
        displayConfig();
        break;
      }
    case MODE_SLOTS_MODE: {
        loadNumberArrayConfInt(slotsMode, displayMode);
        displayConfig();
        break;
      }

    case MODE_MIN_DIM_UP:
    case MODE_MIN_DIM_DOWN: {
        loadNumberArrayConfInt(minDim, displayMode);
        displayConfig();
        break;
      }
    case MODE_ANTI_GHOST_UP:
    case MODE_ANTI_GHOST_DOWN: {
        loadNumberArrayConfInt(antiGhost, displayMode);
        displayConfig();
        break;
      }
    case MODE_TEMP: {
        loadNumberArrayTemp(displayMode);
        displayConfig();
        break;
      }

    case MODE_TUBE_TEST: {
        loadNumberArrayTestDigits();
        allNormal();
        break;
      }
    case MODE_DIGIT_BURN: {
        // Nothing: handled separately to suppress multiplexing
      }
  }
}

// ************************************************************
// Show the next mode - once the button is released
// ************************************************************
void processCurrentMode(int displayMode) {
  switch (displayMode) {
    case MODE_TIME: {
        if (button1.isButtonPressedAndReleased()) {
          // Deal with blanking first

          if ((nowMillis < blankSuppressedSelectionTimoutMillis) || blanked) {
            if (blankSuppressedSelectionTimoutMillis == 0) {
              // Apply 5 sec tineout for setting the suppression time
              blankSuppressedSelectionTimoutMillis = nowMillis + TEMP_DISPLAY_MODE_DUR_MS;
            }

            blankSuppressStep++;
            if (blankSuppressStep > 3) {
              blankSuppressStep = 3;
            }

            if (blankSuppressStep == 1) {
              blankSuppressedMillis = 10000;
            } else if (blankSuppressStep == 2) {
              blankSuppressedMillis = 3600000;
            } else if (blankSuppressStep == 3) {
              blankSuppressedMillis = 3600000 * 4;
            }
            blanked = false;
            setTubesAndLEDSBlankMode();
          } else {
            // Always start from the first mode, or increment the temp mode if we are already in a display
            if (tempDisplayModeDuration > 0) {
              tempDisplayModeDuration = TEMP_DISPLAY_MODE_DUR_MS;
              tempDisplayMode++;
            } else {
              tempDisplayMode = TEMP_MODE_MIN;
            }

            if (tempDisplayMode > TEMP_MODE_MAX) {
              tempDisplayMode = TEMP_MODE_MIN;
            }

            tempDisplayModeDuration = TEMP_DISPLAY_MODE_DUR_MS;
          }
        }
             if (second() == 15) {

                // initialise the slots values temperature 
                loadNumberArrayTemp(11);
                transition.setAlternateValues();
                loadNumberArrayTime();
                transition.setRegularValues();
                allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);

                transition.start(nowMillis);
              }

        if (tempDisplayModeDuration > 0) {
          blanked = false;
          setTubesAndLEDSBlankMode();
          if (tempDisplayMode == TEMP_MODE_DATE) {
            loadNumberArrayDate();
          }

          if (tempDisplayMode == TEMP_MODE_TEMP) {
            byte timeProv = 10;
            if (useRTC) timeProv += 1;
            if (useWiFi > 0) timeProv += 2;
            loadNumberArrayTemp(timeProv);
          }

          if (tempDisplayMode == TEMP_MODE_LDR) {
            loadNumberArrayLDR();
          }

          if (tempDisplayMode == TEMP_IP_ADDR12) {
            if (useWiFi > 0) {
              loadNumberArrayIP(ourIP[0], ourIP[1]);
            } else {
              // we can't show the IP address if we have the RTC, just skip
              tempDisplayMode++;
            }
          }

          if (tempDisplayMode == TEMP_IP_ADDR34) {
            if (useWiFi > 0) {
              loadNumberArrayIP(ourIP[2], ourIP[3]);
            } else {
              // we can't show the IP address if we have the RTC, just skip
              tempDisplayMode++;
            }
          }

          if (tempDisplayMode == TEMP_IMPR) {
            loadNumberArrayConfInt(lastImpressionsPerSec, 0);
          }

          allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);

        } else {
          if (acpOffset > 0) {
            loadNumberArrayACP();
            allBright();
          } else {
            if (slotsMode > SLOTS_MODE_MIN) {
              if (second() == 50) {

                // initialise the slots values
                loadNumberArrayDate();
                transition.setAlternateValues();
                loadNumberArrayTime();
                transition.setRegularValues();
                allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);

                transition.start(nowMillis);
              }

              // initialise the slots mode
              boolean msgDisplaying;
              switch (slotsMode) {
                case SLOTS_MODE_1M_SCR_SCR:
                {
                  msgDisplaying = transition.scrollInScrambleOut(nowMillis);
                  break;
                }
              }

              // Continue slots
              if (msgDisplaying) {
                transition.updateRegularDisplaySeconds(second());
              } else {
                // do normal time thing when we are not in slots
                loadNumberArrayTime();

                allFadeOrNormal(APPLY_LEAD_0_BLANK);
              }
            } else {
              // no slots mode, just do normal time thing
              loadNumberArrayTime();

              allFadeOrNormal(APPLY_LEAD_0_BLANK);
            }
          }
        }
        break;
      }
    case MODE_MINS_SET: {
        if (button1.isButtonPressedAndReleased()) {
          incMins();
        }
        loadNumberArrayTime();
        highlight2and3();
        break;
      }
    case MODE_HOURS_SET: {
        if (button1.isButtonPressedAndReleased()) {
          incHours();
        }
        loadNumberArrayTime();
        highlight0and1();
        break;
      }
    case MODE_SECS_SET: {
        if (button1.isButtonPressedAndReleased()) {
          resetSecond();
        }
        loadNumberArrayTime();
        highlight4and5();
        break;
      }
    case MODE_DAYS_SET: {
        if (button1.isButtonPressedAndReleased()) {
          incDays();
        }
        loadNumberArrayDate();
        highlightDaysDateFormat();
        break;
      }
    case MODE_MONTHS_SET: {
        if (button1.isButtonPressedAndReleased()) {
          incMonths();
        }
        loadNumberArrayDate();
        highlightMonthsDateFormat();
        break;
      }
    case MODE_YEARS_SET: {
        if (button1.isButtonPressedAndReleased()) {
          incYears();
        }
        loadNumberArrayDate();
        highlightYearsDateFormat();
        break;
      }
    case MODE_12_24: {
        if (button1.isButtonPressedAndReleased()) {
          hourMode = ! hourMode;
        }
        loadNumberArrayConfBool(hourMode, displayMode);
        displayConfig();
        break;
      }
    case MODE_LEAD_BLANK: {
        if (button1.isButtonPressedAndReleased()) {
          blankLeading = !blankLeading;
        }
        loadNumberArrayConfBool(blankLeading, displayMode);
        displayConfig();
        break;
      }
    case MODE_SCROLLBACK: {
        if (button1.isButtonPressedAndReleased()) {
          scrollback = !scrollback;
        }
        loadNumberArrayConfBool(scrollback, displayMode);
        displayConfig();
        break;
      }
    case MODE_FADE: {
        if (button1.isButtonPressedAndReleased()) {
          fade = !fade;
        }
        loadNumberArrayConfBool(fade, displayMode);
        displayConfig();
        break;
      }
    case MODE_DATE_FORMAT: {
        if (button1.isButtonPressedAndReleased()) {
          dateFormat++;
          if (dateFormat > DATE_FORMAT_MAX) {
            dateFormat = DATE_FORMAT_MIN;
          }
        }
        loadNumberArrayConfInt(dateFormat, displayMode);
        displayConfig();
        break;
      }
    case MODE_DAY_BLANKING: {
        if (button1.isButtonPressedAndReleased()) {
          dayBlanking++;
          if (dayBlanking > DAY_BLANKING_MAX) {
            dayBlanking = DAY_BLANKING_MIN;
          }
        }
        loadNumberArrayConfInt(dayBlanking, displayMode);
        displayConfig();
        break;
      }
    case MODE_HR_BLNK_START: {
        if (button1.isButtonPressedAndReleased()) {
          blankHourStart++;
          if (blankHourStart > HOURS_MAX) {
            blankHourStart = 0;
          }
        }
        loadNumberArrayConfInt(blankHourStart, displayMode);
        displayConfig();
        break;
      }
    case MODE_HR_BLNK_END: {
        if (button1.isButtonPressedAndReleased()) {
          blankHourEnd++;
          if (blankHourEnd > HOURS_MAX) {
            blankHourEnd = 0;
          }
        }
        loadNumberArrayConfInt(blankHourEnd, displayMode);
        displayConfig();
        break;
      }
    case MODE_SUPPRESS_ACP: {
        if (button1.isButtonPressedAndReleased()) {
          suppressACP = !suppressACP;
        }
        loadNumberArrayConfBool(suppressACP, displayMode);
        displayConfig();
        break;
      }
    case MODE_BLANK_MODE: {
        if (button1.isButtonPressedAndReleased()) {
          blankMode++;
          if (blankMode > BLANK_MODE_MAX) {
            blankMode = BLANK_MODE_MIN;
          }
        }
        loadNumberArrayConfInt(blankMode, displayMode);
        displayConfig();
        break;
      }
    case MODE_USE_LDR: {
        if (button1.isButtonPressedAndReleased()) {
          useLDR = !useLDR;
        }
        loadNumberArrayConfBool(useLDR, displayMode);
        displayConfig();
        break;
      }
    case MODE_FADE_STEPS_UP: {
        if (button1.isButtonPressedAndReleased()) {
          fadeSteps++;
          if (fadeSteps > FADE_STEPS_MAX) {
            fadeSteps = FADE_STEPS_MIN;
          }
        }
        loadNumberArrayConfInt(fadeSteps, displayMode);
        displayConfig();
        // fadeStep = DIGIT_DISPLAY_COUNT / fadeSteps;  // if bright 1000 = 100%  1000 /50 =20
      fadeStep = digitOffCount / fadeSteps;  // if night dimming may be 2 -3 timer tick  ' one bright step'
        break;
      }
    case MODE_FADE_STEPS_DOWN: {
        if (button1.isButtonPressedAndReleased()) {
          fadeSteps--;
          if (fadeSteps < FADE_STEPS_MIN) {
            fadeSteps = FADE_STEPS_MAX;
          }
        }
        loadNumberArrayConfInt(fadeSteps, displayMode);
        displayConfig();
       // fadeStep = DIGIT_DISPLAY_COUNT / fadeSteps;
      fadeStep = digitOffCount / fadeSteps;         // time diagram .. 1000 /45.. 46 =~ 20 ticks - 'timer' one brightness step - if max dimming - ~~3 ..4 
        break;
      }
    case MODE_DISPLAY_SCROLL_STEPS_DOWN: {
        if (button1.isButtonPressedAndReleased()) {
          scrollSteps--;
          if (scrollSteps < SCROLL_STEPS_MIN) {
            scrollSteps = SCROLL_STEPS_MAX;
          }
        }
        loadNumberArrayConfInt(scrollSteps, displayMode);
        displayConfig();
        break;
      }
    case MODE_DISPLAY_SCROLL_STEPS_UP: {
        if (button1.isButtonPressedAndReleased()) {
          scrollSteps++;
          if (scrollSteps > SCROLL_STEPS_MAX) {
            scrollSteps = SCROLL_STEPS_MIN;
          }
        }
        loadNumberArrayConfInt(scrollSteps, displayMode);
        displayConfig();
        break;
      }
    case MODE_SLOTS_MODE: {
        if (button1.isButtonPressedAndReleased()) {
          slotsMode++;
          if (slotsMode > SLOTS_MODE_MAX) {
            slotsMode = SLOTS_MODE_MIN;
          }
        }
        loadNumberArrayConfInt(slotsMode, displayMode);
        displayConfig();
        break;
      }

    case MODE_MIN_DIM_UP: {
        if (button1.isButtonPressedAndReleased()) {
          minDim += 10;
          if (minDim > MIN_DIM_MAX) {
            minDim = MIN_DIM_MAX;
          }
        }
        loadNumberArrayConfInt(minDim, displayMode);
        displayConfig();
        break;
      }
    case MODE_MIN_DIM_DOWN: {
        if (button1.isButtonPressedAndReleased()) {
          minDim -= 10;
          if (minDim < MIN_DIM_MIN) {
            minDim = MIN_DIM_MIN;
          }
        }
        loadNumberArrayConfInt(minDim, displayMode);
        displayConfig();
        break;
      }

    case MODE_TEMP: {
        loadNumberArrayTemp(displayMode);
        displayConfig();
        break;
      }
    case MODE_TUBE_TEST: {
        allNormal();
        loadNumberArrayTestDigits();
        break;
      }
    case MODE_DIGIT_BURN: {
        if (button1.isButtonPressedAndReleased()) {
          digitBurnValue += 1;
          if (digitBurnValue > 9) {
            digitBurnValue = 0;

            digitOff(0);
            digitBurnDigit += 1;
            if (digitBurnDigit > 5) {
              digitBurnDigit = 0;
            }
          }
        }
      }
  }
}

  /*
//**********************************************************************************
//**********************************************************************************
//*                          High Voltage generator                                *
//**********************************************************************************
//**********************************************************************************

// ************************************************************
// Adjust the HV gen to achieve the voltage we require
// Pre-calculate the threshold value of the ADC read and make
// a simple comparison against this for speed
// We control only the PWM "off" time, because the "on" time
// affects the current consumption and MOSFET heating
// ************************************************************
void checkHVVoltage() {
  if (getSmoothedHVSensorReading() > rawHVADCThreshold) {
    setPWMTopTime(pwmTop + getInc());
  } else {
    setPWMTopTime(pwmTop - getInc());
  }
}

// Get the increment value we are going to use based on the magnitude of the 
// difference we have measured
int getInc() {
  int diffValue = abs(getSmoothedHVSensorReading() - rawHVADCThreshold);
  int incValue = 1;
  if (diffValue > 20) incValue = 50;
  else if (diffValue > 10) incValue = 5;
  return incValue;  
}

// ************************************************************
// Calculate the target value for the ADC reading to get the
// defined voltage
// ************************************************************
int getRawHVADCThreshold(double targetVoltage) {
  double externalVoltage = targetVoltage * 4.7 / 394.7 * 1023 / 5;
  int rawReading = (int) externalVoltage;
  return rawReading;
}  */

// **********************************************************************************
// **********************************************************************************
// *                          Light Dependent Resistor                              *
// **********************************************************************************
// **********************************************************************************

// ******************************************************************
// Check the ambient light through the LDR (Light Dependent Resistor)
// Smooths the reading over several reads.
//
// The LDR in bright light gives reading of around 50, the reading in
// total darkness is around 900.
//
// The return value is the dimming count we are using. 999 is full
// brightness, 100 is very dim.
//
// Because the floating point calculation may return more than the
// maximum value, we have to clamp it as the final step
// ******************************************************************
int getDimmingFromLDR() {
  if (useLDR) {
    int rawSensorVal = 1023 - analogRead(LDRPin);
    double sensorDiff = rawSensorVal - sensorLDRSmoothed;
    sensorLDRSmoothed += (sensorDiff / sensorSmoothCountLDR);

    double sensorSmoothedResult = sensorLDRSmoothed - dimDark;
    if (sensorSmoothedResult < dimDark) sensorSmoothedResult = dimDark;
    if (sensorSmoothedResult > dimBright) sensorSmoothedResult = dimBright;

 // no ldr but useLDR true (test) =dimBright (999)  // фоторезистор - если не запаян то установить значение на полную яркость
 sensorSmoothedResult = dimDark +1;
 //sensorSmoothedResult = dimBright;
 
    sensorSmoothedResult = (sensorSmoothedResult - dimDark) * sensorFactor;

    int returnValue = sensorSmoothedResult;

    if (returnValue < minDim) returnValue = minDim;
    if (returnValue > DIGIT_DISPLAY_OFF) returnValue = DIGIT_DISPLAY_OFF;
    return returnValue;
  } else {
    return DIGIT_DISPLAY_OFF;  
  }
}  

/*




/*void doTest()
{
  Serial.print(F("test version: "));
 // Serial.println(FirmwareVersion.substring(1,2)+"."+FirmwareVersion.substring(2,5));
 // for (byte k = 0; k < strlen_P(HardwareVersion); k++) {
  //  Serial.print((char)pgm_read_byte_near(HardwareVersion + k));
 // }
 // Serial.println();
  Serial.println(F("Start Test"));
  
//  p=song;
 // parseSong(p);
  //p=0; //need to be deleted

//  LEDsTest();
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if (Serial1.available() > 10) Serial.println(F("GPS detected"));
    else Serial.println(F("GPS NOT detected!"));
  #endif
//  #ifdef tubes8
 // String testStringArray[11]={"00000000","11111111","22222222","33333333","44444444","55555555","66666666","77777777","88888888","99999999",""};
 // testStringArray[10]=FirmwareVersion+"00";
//  #endif
//  #ifdef tubes6
  String testStringArray[11]={"000000","111111","222222","333333","444444","555555","666666","777777","888888","999999",""};
  testStringArray[10]="123";
 // #endif
  
  int dlay=500;
  bool test=1;
  byte strIndex=-1;
  unsigned long startOfTest=millis()+1000; //disable delaying in first iteration
  bool digitsLock=false;
  while (test)
  {
  //  if (digitalRead(pinDown)==0) digitsLock=true;
  //  if (digitalRead(pinUp)==0) digitsLock=false;

   if ((millis()-startOfTest)>dlay) 
   {
     startOfTest=millis();
     if (!digitsLock) strIndex=strIndex+1;
     if (strIndex==10) dlay=2000;
     if (strIndex>10) { test=false; strIndex=10;}
     Serial.println(testStringArray[strIndex]);
   //  doIndication();
   }
   delayMicroseconds(2000);
  }; 
  
//  if ( !ds.search(addr)) 
 // {
 //   Serial.println(F("Temp. sensor not found."));
//  } else TempPresent=true;
  
  Serial.println(F("Stop Test"));
 // while(1);
}
*/

void doDotBlink()
{
  byte dotPattern = B00000000;
  if (second()%2 == 0) dotPattern = B11000000;
    else dotPattern = B00000000;
}



// ************************************************************
// Called once per second
// ************************************************************
void performOncePerSecondProcessing() {
 
  // Store the current value and reset
 lastImpressionsPerSec = impressionsPerSec;
  impressionsPerSec = 0;
  threesectimer= threesectimer+1 ;
  if (threesectimer==3) { 
    threesectimer=0;
  int     digitSwitchTime[6]= {-1, -1, -1, -1, -1, -1};   //clear fade effect each 3 seconds  //was 3-4 digit turn on at one time in 2 -3 tubes tens - ones seconds

//byte   currNumberArray[6] = {10, 10, 10, 10, 10, 10};
//  byte OnOff[6]    = {0, 0, 0, 0, 0, 0};
  byte fadeState[6] = {0, 0, 0, 0, 0, 0};
                  // initialise the slots values temperature 
                loadNumberArrayTemp(11);
                transition.setAlternateValues();
                loadNumberArrayTime();
                transition.setRegularValues();
                allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);

               // transition.start(nowMillis);
  // убрал - моргает 
  /* digitOff(5);  
  digitOff(4);
  digitOff(3);
  digitOff(2);
  digitOff(1);
  digitOff(0);
  */
  
  }
  for ( int i = 0 ; i < DIGIT_COUNT ; i ++ ) {
currNumberArray[i] = NumberArray[i];    // clear Fade if over-run 
}

  // Change the direction of the pulse
//  upOrDown = !upOrDown;

 
  // If we are in temp display mode, decrement the count
  if (tempDisplayModeDuration > 0) {
    if (tempDisplayModeDuration > 1000) {
      tempDisplayModeDuration -= 1000;
    } else {
      tempDisplayModeDuration = 0;
    }
  }
    // decrement the blanking supression counter
  if (blankSuppressedMillis > 0) {
    if (blankSuppressedMillis > 1000) {
      blankSuppressedMillis -= 1000;
    } else {
      blankSuppressedMillis = 0;
    }
  }
  // Get the blanking status, this may be overridden by blanking suppression
  // Only blank if we are in TIME mode
  if (currentMode == MODE_TIME) {
    boolean nativeBlanked = checkBlanking();

    // Check if we are in blanking suppression mode
    blanked = nativeBlanked && (blankSuppressedMillis == 0);

    // reset the blanking period selection timer
    if (nowMillis > blankSuppressedSelectionTimoutMillis) {
      blankSuppressedSelectionTimoutMillis = 0;
      blankSuppressStep = 0;
    }
  } else {
    blanked = false;
  }
  setTubesAndLEDSBlankMode();
  // Slow regulation of the voltage
  //checkHVVoltage();

  // feed the watchdog
  wdt_reset();
     // getRTCTime();   //перегорит через неделю или сядет батарейка - включить батарейку через 2 диодика


//       byte   secondes = second();
//byte minutes = minute();
//byte hours = hour()  ;
   //  display_nixietubes(secondes, minutes, hours); // display the real-time clock data on the Nixies set data  //change to NumberArray as in the original code - display different data

//        loadNumberArrayTime();

// do complete display (this was test)
   // display_nixietubes(); // display the real-time clock data on the Nixies set data  //change to NumberArray as in the original code - display different data
   //   shifter.write();
}

// ************************************************************
// Called once per minute
// ************************************************************
void performOncePerMinuteProcessing() {

 // digitalClockDisplay()  ; //print time - digits as hh:mm:ss to serial port 57600
   if (debug)Serial.print(lastImpressionsPerSec);   // cycles per second - delay 50 show 20 cycles
   if (debug)Serial.println();
 /*      nixietrainer();      // anti cathodes poisoning  1.8sec all
// if (debug){
  if (useRTC) testDS3231TempSensor();  //display temperature
// }
 */
 //shifter.setAll(HIGH);   // off all tubes
  //   shifter.write(); //send changes to the chain and display them
     
 /*        
  *    время с ds3231 - раз в час поменял     
  *         allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);
 OnOff[0]=0;
 OnOff[1]=0;
 OnOff[2]=0;
 OnOff[3]=0;
 OnOff[4]=0;
 OnOff[5]=0;
                 // initialise the slots values temperature //  после чтения чипа часов есть ошибка - засветка третьей одновременно цифры нуля или единички на 4 лампах кроме часовых!
                loadNumberArrayTemp(11);
                transition.setAlternateValues();
                loadNumberArrayTime();
                transition.setRegularValues();
                allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);  
                */
int     digitSwitchTime[6]= {-1, -1, -1, -1, -1, -1};       // OK fade 2 digit together Works //  заработал режим переключения с плавным гашением  was glitch 3 digits together each minute..
}

// ************************************************************
// Called once per hour
// ************************************************************
void performOncePerHourProcessing() {

    //  считывание ds3231 перенес на 1 раз в час .. глюк - сбивается режим fade  .. не только - сбрасываю еще  digitSwithcTime
    if (useWiFi > 0) {
    if (useWiFi == MAX_WIFI_TIME) {
      // We recently got an update, send to the RTC (if installed)
      setRTC();      
    }
    useWiFi--;
  } else {
    // get the time from the external RTC provider - (if installed) one  time each minute  перенес на каждый час 
  if (useRTC)  getRTCTime();
  }
                // initialise the slots values temperature //  после чтения чипа часов есть ошибка - засветка третьей одновременно цифры нуля или единички на 4 лампах кроме часовых!
                loadNumberArrayTemp(11);
                transition.setAlternateValues();
                loadNumberArrayTime();
                transition.setRegularValues();
                allFadeOrNormal(DO_NOT_APPLY_LEAD_0_BLANK);

  
}

//subs

/*
 // shiftout
  void updateShiftRegister(){
  digitalWrite(RCLK_Pin, LOW);
  shiftOut(SER_Pin, SRCLK_Pin, LSBFIRST, nixies);
  digitalWrite(RCLK_Pin, HIGH);
} */

void testDS3231TempSensor()
{
   byte DS3231InternalTemperature=0;   //another RTC

  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(0x11);
  Wire.endTransmission();

  Wire.requestFrom(RTC_I2C_ADDRESS, 2);
  DS3231InternalTemperature=Wire.read();
    float c = DS3231InternalTemperature ;
  float f = c / 4. * 9. / 5. + 32.;
  DS3231InternalTemperature=Wire.read();  //fractial 2-nd byte
  if (debug) {
  Serial.print(F("DS3231_T-2byte="));
  int wholeDegrees = int(c);
  Serial.print(wholeDegrees);
    Serial.print(".");
     Serial.print(int (DS3231InternalTemperature));
   Serial.print(" F ");
  Serial.println(f);
   if ((c<2) || (c>95)) 
   {
    Serial.println(F("wrong read DS3231!"));
    for (int i=0; i<5; i++)
    {
      //tone(pinBuzzer, 1000);
     // tone1.play(1000, 1000);
     // delay(2000);
    }
   } 
  }
  float temp = getRTCTemp();
  int wholeDegrees = int(temp);
  temp = (temp - float(wholeDegrees)) * 100.0;
  int fractDegrees = int(temp);
       byte   minutes = wholeDegrees; // Rus температура на 6 лампах - первые 2 гасятся, потом целая часть - градусы, потом дробная.
 byte secondes = fractDegrees;
 //byte hours = hour();
 if (debug) {
     Serial.print(F("DS3231_T="));
     Serial.print(wholeDegrees);
     Serial.print(".");
       Serial.println(fractDegrees);   // DS3231_T=21.75 (Celsius)
 }
   // display_nixietubes(secondes, minutes, hours); // display the real-time clock data on the nixies
    //shifter.setAll(HIGH);
     prints(secondes);
  printm(minutes);
        shifter.setPin(0, HIGH);
      shifter.setPin(1, HIGH);
      shifter.setPin(2, HIGH);
      shifter.setPin(3, HIGH);
              shifter.setPin(4, HIGH);
      shifter.setPin(5, HIGH);
      shifter.setPin(6, HIGH);
      shifter.setPin(7, HIGH);
      shifter.write();    //display temperature (2 -digit hours blank)
       delay(2000);
      
}
void display_nixietubes() {
//void display_nixietubes(byte secondes, byte minutes, byte hours) 
//set data for display - 6 Nixie tubes with driver K155ID1
  //  change to display NumberArray    
  //byte second, minute, hour;  // shifter set pins of (3) or 4 registers 74hc595, then call shifter.write();
  // retrieve data from DS3231
  //readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
 // second = 81;
//minute = 98;
//hour = 69;
  //prints(secondes);
  //printm(minutes);
  //printh(hours);
  
// prints
  nixie (20, NumberArray[5]);  //seconds - lamp 5 -6 
  nixie (16, NumberArray[4]);

// printm
  nixie (12, NumberArray[3]);
  nixie (8, NumberArray[2]);  //minutes
  
//printh
  nixie (4, NumberArray[1]);
  nixie (0,  NumberArray[0]);  //hours lamp 1-2
  
}

// atmega8 basic
//Shiftout Portb.4 , Portb.0 , Seco , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Mine , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Hour , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Dat , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Month , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Year , 3 , 8 , 3


void digitalClockDisplay()
{
  if (debug){ Serial.print(hour());
  printDigits(minute());   // 12:35:00  time format (trranslate from Russian - printTime)  +4 = 16:35
  printDigits(second());
  Serial.print(' ');
  Serial.print(day());
  Serial.print(' ');
  Serial.print(month());
  Serial.print(' ');
  Serial.print(year());
  Serial.println();
  }
}
void printDigits(int digits) {
  Serial.print(':');
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void nixietrainer() {
  int i=2;
  if (i> 0) {
  shifter.clear();
  shifter.write();
  prints(00);
  printm(00);
  printh(00);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(11);
  printm(11);
  printh(11);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(22);
  printm(22);
  printh(22);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(33);
  printm(33);
  printh(33);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(44);
  printm(44);
  printh(44);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(55);
  printm(55);
  printh(55);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(66);
  printm(66);
  printh(66);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(77);
  printm(77);
  printh(77);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(88);
  printm(88);
  printh(88);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  prints(99);
  printm(99);
  printh(99);
  shifter.write();
  delay(100);
  shifter.clear();
  shifter.write();
  i=i-1;
  }
  return;
}

void prints(int v) {
  int oness;
  int tenss;
  oness = v % 10;
  v = v / 10;
  tenss = v % 10;
  nixie (20, oness);
  nixie (16, tenss);

}

void printm(int m) {
  int onesm;
  int tensm;
  onesm = m % 10;
  m = m / 10;
  tensm = m % 10;
  nixie (12, onesm);
  nixie (8, tensm);

}

void printh(int h) {
  int onesh;
  int tensh;
  onesh = h % 10;
  h = h / 10;
  tensh = h % 10;
  nixie (4, onesh);
  nixie (0, tensh);

}


// ************************************************************
// Get the time from the RTC - rtc ds3231 + eeprom 24c32 - no module! solder to registers board 
// ************************************************************
void getRTCTime() {
  // Start the RTC communication in master mode                       
  Wire.end();
  Wire.begin();

  // Set up the time provider
  // first try to find the RTC, if not available, go into slave mode
 Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(zero);  //0x00 pointer - read from internal register 0
  Wire.endTransmission();
 //  useRTC = (Wire.endTransmission() == 0);
  if (useRTC) {
    bool PM;
    bool twentyFourHourClock;
    bool century = false;
    //bool h12;

    byte years = Clock.getYear() + 2000;
    byte months = Clock.getMonth(century);
    byte days = Clock.getDate();
    //byte hours = Clock.getHour(h12, PM);
     byte hours = Clock.getHour(twentyFourHourClock, PM);
    byte mins = Clock.getMinute();
    byte secs = Clock.getSecond();
    setTime(hours, mins, secs, days, months, years);

    // Make sure the clock keeps running even on battery
    if (!Clock.oscillatorCheck())
      Clock.enableOscillator(true, true, 0);   // or set seconds - start osc
  }

  // Return back to I2C in slave mode
  Wire.end();
  // Wire.begin(I2C_SLAVE_ADDR);
 // Wire.onReceive(receiveEvent);
 // Wire.onRequest(requestEvent);
}

// ************************************************************
// Set the date/time in the RTC from the internal time
// Always hold the time in 24 format, we convert to 12 in the
// display.
// ************************************************************
void setRTC() {
  if (useRTC) {
    // Start the RTC communication in master mode
    Wire.end();
    Wire.begin();

    // Set up the time provider
    // first try to find the RTC, if not available, go into slave mode   ?wifi module // rtc DS3231 vcc=5v  vbat >3v
   Wire.beginTransmission(RTC_I2C_ADDRESS);
    Wire.write(zero); //stop Oscillator
    Clock.setClockMode(false); // false = 24h
    Clock.setYear(year() % 100);
    Clock.setMonth(month());
    Clock.setDate(day());
    Clock.setDoW(weekday());
    Clock.setHour(hour());
    Clock.setMinute(minute());
    Clock.setSecond(second());   // start osc
//  Wire.write(zero); //start
   Wire.endTransmission();
    Wire.end();
  //  Wire.begin(I2C_SLAVE_ADDR);
   // Wire.onReceive(receiveEvent);
  //  Wire.onRequest(requestEvent);
  }
}

// ************************************************************
// Get the temperature from the RTC
// ************************************************************
float getRTCTemp() {
  if (useRTC) {
    return Clock.getTemperature();
  } else {
    return 0.0;
  }
}

//**********************************************************************************
//**********************************************************************************
//*                               EEPROM interface                                 *
//**********************************************************************************
//**********************************************************************************

// ************************************************************
// Save current values back to EEPROM
// ************************************************************
void saveEEPROMValues() {
  EEPROM.write(EE_12_24, hourMode);
  EEPROM.write(EE_FADE_STEPS, fadeSteps);
  EEPROM.write(EE_DATE_FORMAT, dateFormat);
  EEPROM.write(EE_DAY_BLANKING, dayBlanking);
  EEPROM.write(EE_DIM_DARK_LO, dimDark % 256);
  EEPROM.write(EE_DIM_DARK_HI, dimDark / 256);
  EEPROM.write(EE_BLANK_LEAD_ZERO, blankLeading);
  EEPROM.write(EE_SCROLLBACK, scrollback);
  EEPROM.write(EE_FADE, fade);
  EEPROM.write(EE_SCROLL_STEPS, scrollSteps);
  EEPROM.write(EE_DIM_BRIGHT_LO, dimBright % 256);
  EEPROM.write(EE_DIM_BRIGHT_HI, dimBright / 256);
  EEPROM.write(EE_DIM_SMOOTH_SPEED, sensorSmoothCountLDR); //  EEPROM.write(EE_RED_INTENSITY, redCnl);//  EEPROM.write(EE_GRN_INTENSITY, grnCnl);//  EEPROM.write(EE_BLU_INTENSITY, bluCnl);//  EEPROM.write(EE_BACKLIGHT_MODE, backlightMode);
//  EEPROM.write(EE_HV_VOLTAGE, hvTargetVoltage);
  EEPROM.write(EE_SUPPRESS_ACP, suppressACP);
  EEPROM.write(EE_HOUR_BLANK_START, blankHourStart);
  EEPROM.write(EE_HOUR_BLANK_END, blankHourEnd);
//  EEPROM.write(EE_CYCLE_SPEED, cycleSpeed);
//  EEPROM.write(EE_PULSE_LO, pwmOn % 256);
//  EEPROM.write(EE_PULSE_HI, pwmOn / 256);
//  EEPROM.write(EE_PWM_TOP_LO, pwmTop % 256);
//  EEPROM.write(EE_PWM_TOP_HI, pwmTop / 256);
  EEPROM.write(EE_MIN_DIM_LO, minDim % 256);
  EEPROM.write(EE_MIN_DIM_HI, minDim / 256);
  EEPROM.write(EE_ANTI_GHOST, antiGhost);
  EEPROM.write(EE_USE_LDR, useLDR);
  EEPROM.write(EE_BLANK_MODE, blankMode);
  EEPROM.write(EE_SLOTS_MODE, slotsMode);
}

// ************************************************************
// read EEPROM values
// ************************************************************
void readEEPROMValues() {
  hourMode = EEPROM.read(EE_12_24);
  fadeSteps = EEPROM.read(EE_FADE_STEPS);
    if ((fadeSteps < FADE_STEPS_MIN) || (fadeSteps > FADE_STEPS_MAX)) {
    fadeSteps = FADE_STEPS_DEFAULT;
  }
  dateFormat = EEPROM.read(EE_DATE_FORMAT);
  dayBlanking = EEPROM.read(EE_DAY_BLANKING);
  dimDark = EEPROM.read(EE_DIM_DARK_HI) * 256 + EEPROM.read(EE_DIM_DARK_LO);
    if ((dimDark < SENSOR_LOW_MIN) || (dimDark > SENSOR_LOW_MAX)) {
    dimDark = SENSOR_LOW_DEFAULT;
  }
  blankLeading = EEPROM.read(EE_BLANK_LEAD_ZERO);
  scrollback = EEPROM.read(EE_SCROLLBACK);
  fade = EEPROM.read(EE_FADE);
  scrollSteps = EEPROM.read(EE_SCROLL_STEPS);
  dimBright = EEPROM.read(EE_DIM_BRIGHT_HI) * 256 + EEPROM.read(EE_DIM_BRIGHT_LO);
    if ((dimBright < SENSOR_HIGH_MIN) || (dimBright > SENSOR_HIGH_MAX)) {
    dimBright = SENSOR_HIGH_DEFAULT;
  }
  sensorSmoothCountLDR = EEPROM.read(EE_DIM_SMOOTH_SPEED);
    if ((sensorSmoothCountLDR < SENSOR_SMOOTH_READINGS_MIN) || (sensorSmoothCountLDR > SENSOR_SMOOTH_READINGS_MAX)) {
    sensorSmoothCountLDR = SENSOR_SMOOTH_READINGS_DEFAULT;
  }
  suppressACP = EEPROM.read(EE_SUPPRESS_ACP);
  blankHourStart = EEPROM.read(EE_HOUR_BLANK_START);
    if ((blankHourStart < 0) || (blankHourStart > HOURS_MAX)) {
    blankHourStart = 0;
  }
  blankHourEnd = EEPROM.read(EE_HOUR_BLANK_END);
    if ((blankHourEnd < 0) || (blankHourEnd > HOURS_MAX)) {
    blankHourEnd = 7;
  }
  minDim = EEPROM.read(EE_MIN_DIM_HI) * 256 + EEPROM.read(EE_MIN_DIM_LO);
    if ((minDim < MIN_DIM_MIN) || (minDim > MIN_DIM_MAX)) {
    minDim = MIN_DIM_DEFAULT;
  }
  antiGhost = EEPROM.read(EE_ANTI_GHOST);
  dispCount = DIGIT_DISPLAY_COUNT + antiGhost;
  blankMode= EEPROM.read(EE_BLANK_MODE);
  useLDR = EEPROM.read(EE_USE_LDR);
  slotsMode= EEPROM.read(EE_SLOTS_MODE);
    if (debug){ Serial.print(hourMode);
    Serial.print(' ');
Serial.print(fadeSteps);
  Serial.print(' ');
   Serial.print(dateFormat);
  Serial.print(' '); 
  Serial.print(dayBlanking);
  Serial.print(' ');
  Serial.print(dimDark);
  Serial.print(' ');
  Serial.print(blankLeading);
    Serial.print(' ');
  Serial.print(scrollback);
  Serial.println();

Serial.print(fade);
  Serial.print(' ');
   Serial.print(scrollSteps);
  Serial.print(' '); 
  Serial.print(dimBright);
  Serial.print(' ');
  Serial.print(sensorSmoothCountLDR);
  Serial.print(' ');
  Serial.print(suppressACP);
    Serial.print(' ');
  Serial.print(blankHourStart);
  Serial.println();
    Serial.print(blankHourEnd);
    Serial.print(' ');
Serial.print(minDim);
  Serial.print(' ');
   Serial.print(antiGhost);
  Serial.print(' '); 
  Serial.print(dispCount);
  Serial.print(' ');
  Serial.print(blankMode);
  Serial.print(' ');
  Serial.print(useLDR);
    Serial.print(' ');
  Serial.print(slotsMode);
  Serial.println();
    }
/*  backlightMode = EEPROM.read(EE_BACKLIGHT_MODE);
  if ((backlightMode < BACKLIGHT_MIN) || (backlightMode > BACKLIGHT_MAX)) {
    backlightMode = BACKLIGHT_DEFAULT;
  }

 redCnl = EEPROM.read(EE_RED_INTENSITY);
  if ((redCnl < COLOUR_CNL_MIN) || (redCnl > COLOUR_CNL_MAX)) {
    redCnl = COLOUR_RED_CNL_DEFAULT;
  }

  grnCnl = EEPROM.read(EE_GRN_INTENSITY);
  if ((grnCnl < COLOUR_CNL_MIN) || (grnCnl > COLOUR_CNL_MAX)) {
    grnCnl = COLOUR_GRN_CNL_DEFAULT;
  }

  bluCnl = EEPROM.read(EE_BLU_INTENSITY);
  if ((bluCnl < COLOUR_CNL_MIN) || (bluCnl > COLOUR_CNL_MAX)) {
    bluCnl = COLOUR_BLU_CNL_DEFAULT;
  }

  hvTargetVoltage = EEPROM.read(EE_HV_VOLTAGE);
  if ((hvTargetVoltage < HVGEN_TARGET_VOLTAGE_MIN) || (hvTargetVoltage > HVGEN_TARGET_VOLTAGE_MAX)) {
    hvTargetVoltage = HVGEN_TARGET_VOLTAGE_DEFAULT;
  }

  pwmOn = EEPROM.read(EE_PULSE_HI) * 256 + EEPROM.read(EE_PULSE_LO);
  if ((pwmOn < PWM_PULSE_MIN) || (pwmOn > PWM_PULSE_MAX)) {
    pwmOn = PWM_PULSE_DEFAULT;

    // Hmmm, need calibration
    EEPROM.write(EE_HVG_NEED_CALIB, true);
  }

  pwmTop = EEPROM.read(EE_PWM_TOP_HI) * 256 + EEPROM.read(EE_PWM_TOP_LO);
  if ((pwmTop < PWM_TOP_MIN) || (pwmTop > PWM_TOP_MAX)) {
    pwmTop = PWM_TOP_DEFAULT;

    // Hmmm, need calibration
    EEPROM.write(EE_HVG_NEED_CALIB, true);
  }
    cycleSpeed = EEPROM.read(EE_CYCLE_SPEED);
  if ((cycleSpeed < CYCLE_SPEED_MIN) || (cycleSpeed > CYCLE_SPEED_MAX)) {
    cycleSpeed = CYCLE_SPEED_DEFAULT;
  }
*/

  if ((dayBlanking < DAY_BLANKING_MIN) || (dayBlanking > DAY_BLANKING_MAX)) {
    dayBlanking = DAY_BLANKING_DEFAULT;
  }
  if ((dimDark < SENSOR_LOW_MIN) || (dimDark > SENSOR_LOW_MAX)) {
    dimDark = SENSOR_LOW_DEFAULT;
  }           
  if ((scrollSteps < SCROLL_STEPS_MIN) || (scrollSteps > SCROLL_STEPS_MAX)) {
    scrollSteps = SCROLL_STEPS_DEFAULT;
  }
  if ((dimBright < SENSOR_HIGH_MIN) || (dimBright > SENSOR_HIGH_MAX)) {
    dimBright = SENSOR_HIGH_DEFAULT;
  }
  if ((sensorSmoothCountLDR < SENSOR_SMOOTH_READINGS_MIN) || (sensorSmoothCountLDR > SENSOR_SMOOTH_READINGS_MAX)) {
    sensorSmoothCountLDR = SENSOR_SMOOTH_READINGS_DEFAULT;
  }
  if ((fadeSteps < FADE_STEPS_MIN) || (fadeSteps > FADE_STEPS_MAX)) {
    fadeSteps = FADE_STEPS_DEFAULT;
  }
  if ((blankHourStart < 0) || (blankHourStart > HOURS_MAX)) {
    blankHourStart = 0;
  }
  if ((blankHourEnd < 0) || (blankHourEnd > HOURS_MAX)) {
    blankHourEnd = 7;
  }
  if ((minDim < MIN_DIM_MIN) || (minDim > MIN_DIM_MAX)) {
    minDim = MIN_DIM_DEFAULT;
  }
  if ((antiGhost < ANTI_GHOST_MIN) || (antiGhost > ANTI_GHOST_MAX)) {
    antiGhost = ANTI_GHOST_DEFAULT;
  }
  if ((blankMode < BLANK_MODE_MIN) || (blankMode > BLANK_MODE_MAX)) {
    blankMode = BLANK_MODE_DEFAULT;
  }
  if ((dateFormat < DATE_FORMAT_MIN) || (dateFormat > DATE_FORMAT_MAX)) {
    dateFormat = DATE_FORMAT_DEFAULT;
  }
  if ((slotsMode < SLOTS_MODE_MIN) || (slotsMode > SLOTS_MODE_MAX)) {
    slotsMode = SLOTS_MODE_DEFAULT;
  }

}

// ************************************************************
// Reset EEPROM values back to what they once were
// ************************************************************
void factoryReset() {
  hourMode = HOUR_MODE_DEFAULT;
  blankLeading = LEAD_BLANK_DEFAULT;
  scrollback = SCROLLBACK_DEFAULT;
  fade = FADE_DEFAULT;
  fadeSteps = FADE_STEPS_DEFAULT;
  dateFormat = DATE_FORMAT_DEFAULT;
  dayBlanking = DAY_BLANKING_DEFAULT;
  dimDark = SENSOR_LOW_DEFAULT;
  scrollSteps = SCROLL_STEPS_DEFAULT;
  dimBright = SENSOR_HIGH_DEFAULT;
  sensorSmoothCountLDR = SENSOR_SMOOTH_READINGS_DEFAULT;
  dateFormat = DATE_FORMAT_DEFAULT;
  dayBlanking = DAY_BLANKING_DEFAULT;//  backlightMode = BACKLIGHT_DEFAULT;//  redCnl = COLOUR_RED_CNL_DEFAULT;//  grnCnl = COLOUR_GRN_CNL_DEFAULT;//  bluCnl = COLOUR_BLU_CNL_DEFAULT;//  hvTargetVoltage = HVGEN_TARGET_VOLTAGE_DEFAULT;
  suppressACP = SUPPRESS_ACP_DEFAULT;
  blankHourStart = 0;
  blankHourEnd = 7;
//  cycleSpeed = CYCLE_SPEED_DEFAULT;
 // pwmOn = PWM_PULSE_DEFAULT;
//  pwmTop = PWM_TOP_DEFAULT;
  minDim = MIN_DIM_DEFAULT;
  //antiGhost = ANTI_GHOST_DEFAULT;  HV by arduino pwm not use - I prefer MC34063
  useLDR = USE_LDR_DEFAULT;
  blankMode = BLANK_MODE_DEFAULT;
  slotsMode = SLOTS_MODE_DEFAULT;

  saveEEPROMValues();
}
//**********************************************************************************
//**********************************************************************************
//*                                 I2C interface                                  *
//**********************************************************************************
//**********************************************************************************

/**
 * receive information from the master
 */
void receiveEvent(int bytes) {
  // the operation tells us what we are getting
  int operation = Wire.read();

  if (operation == I2C_TIME_UPDATE) {
    // If we're getting time from the WiFi module, mark that we have an active WiFi with a 5 min time out
    useWiFi = MAX_WIFI_TIME;

    int newYears = Wire.read();
    int newMonths = Wire.read();
    int newDays = Wire.read();

    int newHours = Wire.read();
    int newMins = Wire.read();
    int newSecs = Wire.read();

    setTime(newHours, newMins, newSecs, newDays, newMonths, newYears);
  } else if (operation == I2C_SET_OPTION_12_24) {
    byte readByte1224 = Wire.read();
    hourMode = (readByte1224 == 1);
    EEPROM.write(EE_12_24, hourMode);
  } else if (operation == I2C_SET_OPTION_BLANK_LEAD) {
    byte readByteBlank = Wire.read();
    blankLeading = (readByteBlank == 1);
    EEPROM.write(EE_BLANK_LEAD_ZERO, blankLeading);
  } else if (operation == I2C_SET_OPTION_SCROLLBACK) {
    byte readByteSB = Wire.read();
    scrollback = (readByteSB == 1);
    EEPROM.write(EE_SCROLLBACK, scrollback);
  } else if (operation == I2C_SET_OPTION_SUPPRESS_ACP) {
    byte readByteSA = Wire.read();
    suppressACP = (readByteSA == 1);
    EEPROM.write(EE_SUPPRESS_ACP, suppressACP);
  } else if (operation == I2C_SET_OPTION_DATE_FORMAT) {
    dateFormat = Wire.read();
    EEPROM.write(EE_DATE_FORMAT, dateFormat);
  } else if (operation == I2C_SET_OPTION_DAY_BLANKING) {
    dayBlanking = Wire.read();
    EEPROM.write(EE_DAY_BLANKING, dayBlanking);
  } else if (operation == I2C_SET_OPTION_BLANK_START) {
    blankHourStart = Wire.read();
    EEPROM.write(EE_HOUR_BLANK_START, blankHourStart);
  } else if (operation == I2C_SET_OPTION_BLANK_END) {
    blankHourEnd = Wire.read();
    EEPROM.write(EE_HOUR_BLANK_END, blankHourEnd);
  } else if (operation == I2C_SET_OPTION_FADE_STEPS) {
    fadeSteps = Wire.read();
    EEPROM.write(EE_FADE_STEPS, fadeSteps);
  } else if (operation == I2C_SET_OPTION_SCROLL_STEPS) {
    scrollSteps = Wire.read();
    EEPROM.write(EE_SCROLL_STEPS, scrollSteps);
  /*  } else if (operation == I2C_SET_OPTION_BACKLIGHT_MODE)  
  {
    backlightMode = Wire.read();
    EEPROM.write(EE_BACKLIGHT_MODE, backlightMode);
  } else if (operation == I2C_SET_OPTION_RED_CHANNEL) {
   redCnl = Wire.read();
    EEPROM.write(EE_RED_INTENSITY, redCnl);
  } else if (operation == I2C_SET_OPTION_GREEN_CHANNEL) {
    grnCnl = Wire.read();
    EEPROM.write(EE_GRN_INTENSITY, grnCnl);
  } else if (operation == I2C_SET_OPTION_BLUE_CHANNEL) {
    bluCnl = Wire.read();
    EEPROM.write(EE_BLU_INTENSITY, bluCnl);
  } else if (operation == I2C_SET_OPTION_CYCLE_SPEED) {
    cycleSpeed = Wire.read();
    EEPROM.write(EE_CYCLE_SPEED, cycleSpeed); */
  } else if (operation == I2C_SHOW_IP_ADDR) {
    ourIP[0] = Wire.read();
    ourIP[1] = Wire.read();
    ourIP[2] = Wire.read();
    ourIP[3] = Wire.read();
  } else if (operation == I2C_SET_OPTION_FADE) {
    fade = Wire.read();
    EEPROM.write(EE_FADE, fade);
  } else if (operation == I2C_SET_OPTION_USE_LDR) {
    byte readByteUseLDR = Wire.read();
    useLDR = (readByteUseLDR == 1);
    EEPROM.write(EE_USE_LDR, useLDR);
  } else if (operation == I2C_SET_OPTION_BLANK_MODE) {
    blankMode = Wire.read();
    EEPROM.write(EE_BLANK_MODE, blankMode);
  } else if (operation == I2C_SET_OPTION_SLOTS_MODE) {
    slotsMode = Wire.read();
    EEPROM.write(EE_SLOTS_MODE, slotsMode);
  } else if (operation == I2C_SET_OPTION_MIN_DIM) {
    byte dimHI = Wire.read();
    byte dimLO = Wire.read();
    minDim = dimHI * 256 + dimLO;
    EEPROM.write(EE_MIN_DIM_HI, dimHI);
    EEPROM.write(EE_MIN_DIM_LO, dimLO);
  }
}

/**
   send information to the master
*/
void requestEvent() {
  byte configArray[I2C_DATA_SIZE];
  int idx = 0;
  configArray[idx++] = I2C_PROTOCOL_NUMBER;  // protocol version
  configArray[idx++] = encodeBooleanForI2C(hourMode);
  configArray[idx++] = encodeBooleanForI2C(blankLeading);
  configArray[idx++] = encodeBooleanForI2C(scrollback);
  configArray[idx++] = encodeBooleanForI2C(suppressACP);
  configArray[idx++] = encodeBooleanForI2C(fade);
  configArray[idx++] = dateFormat;
  configArray[idx++] = dayBlanking;
  configArray[idx++] = blankHourStart;
  configArray[idx++] = blankHourEnd;
  configArray[idx++] = fadeSteps;
  configArray[idx++] = scrollSteps;//  configArray[idx++] = backlightMode; //  configArray[idx++] = redCnl;//  configArray[idx++] = grnCnl;//  configArray[idx++] = bluCnl;
 // configArray[idx++] = cycleSpeed;
  configArray[idx++] = encodeBooleanForI2C(useLDR);
  configArray[idx++] = blankMode;
  configArray[idx++] = slotsMode;
  configArray[idx++] = minDim / 256;
  configArray[idx++] = minDim % 256;

  Wire.write(configArray, I2C_DATA_SIZE);
}

byte encodeBooleanForI2C(boolean valueToProcess) {
  if (valueToProcess) {
    byte byteToSend = 1;
    return byteToSend;
  } else {
    byte byteToSend = 0;
    return byteToSend;
  }
}
// ************************************************************
// Break the time into displayable digits
// ************************************************************
void loadNumberArrayTime() {
  NumberArray[5] = second() % 10;
  NumberArray[4] = second() / 10;
  NumberArray[3] = minute() % 10;
  NumberArray[2] = minute() / 10;
  if (hourMode) {
    NumberArray[1] = hourFormat12() % 10;
    NumberArray[0] = hourFormat12() / 10;
  } else {
    NumberArray[1] = hour() % 10;
    NumberArray[0] = hour() / 10;
  }
}

// ************************************************************
// Break the time into displayable digits
// ************************************************************
void loadNumberArraySameValue(byte val) {
  NumberArray[5] = val;
  NumberArray[4] = val;
  NumberArray[3] = val;
  NumberArray[2] = val;
  NumberArray[1] = val;
  NumberArray[0] = val;
}

// ************************************************************
// Break the time into displayable digits
// ************************************************************
void loadNumberArrayDate() {
  switch (dateFormat) {
    case DATE_FORMAT_YYMMDD:
      NumberArray[5] = day() % 10;
      NumberArray[4] = day() / 10;
      NumberArray[3] = month() % 10;
      NumberArray[2] = month() / 10;
      NumberArray[1] = (year() - 2000) % 10;
      NumberArray[0] = (year() - 2000) / 10;
      break;
    case DATE_FORMAT_MMDDYY:
      NumberArray[5] = (year() - 2000) % 10;
      NumberArray[4] = (year() - 2000) / 10;
      NumberArray[3] = day() % 10;
      NumberArray[2] = day() / 10;
      NumberArray[1] = month() % 10;
      NumberArray[0] = month() / 10;
      break;
    case DATE_FORMAT_DDMMYY:
      NumberArray[5] = (year() - 2000) % 10;
      NumberArray[4] = (year() - 2000) / 10;
      NumberArray[3] = month() % 10;
      NumberArray[2] = month() / 10;
      NumberArray[1] = day() % 10;
      NumberArray[0] = day() / 10;
      break;
  }
}

// ************************************************************
// Break the temperature into displayable digits
// ************************************************************
void loadNumberArrayTemp(int confNum) {
  NumberArray[5] = (confNum) % 10;
  NumberArray[4] = (confNum / 10) % 10;
  float temp = getRTCTemp();
  int wholeDegrees = int(temp);
  temp = (temp - float(wholeDegrees)) * 100.0;
  int fractDegrees = int(temp);

  NumberArray[3] = fractDegrees % 10;
  NumberArray[2] =  fractDegrees / 10;
  NumberArray[1] =  wholeDegrees % 10;
  NumberArray[0] = wholeDegrees / 10;
}

// ************************************************************
// Break the LDR reading into displayable digits
// ************************************************************
void loadNumberArrayLDR() {
  NumberArray[5] = 0;
  NumberArray[4] = 0;

  NumberArray[3] = (digitOffCount / 1) % 10;
  NumberArray[2] = (digitOffCount / 10) % 10;
  NumberArray[1] = (digitOffCount / 100) % 10;
  NumberArray[0] = (digitOffCount / 1000) % 10;
}

// ************************************************************
// Test digits
// ************************************************************
void loadNumberArrayTestDigits() {
  NumberArray[5] =  second() % 10;
  NumberArray[4] = (second() + 1) % 10;
  NumberArray[3] = (second() + 2) % 10;
  NumberArray[2] = (second() + 3) % 10;
  NumberArray[1] = (second() + 4) % 10;
  NumberArray[0] = (second() + 5) % 10;
}

// ************************************************************
// Do the Anti Cathode Poisoning
// ************************************************************
void loadNumberArrayACP() {
  NumberArray[5] = (second() + acpOffset) % 10;
  NumberArray[4] = (second() / 10 + acpOffset) % 10;
  NumberArray[3] = (minute() + acpOffset) % 10;
  NumberArray[2] = (minute() / 10 + acpOffset) % 10;
  NumberArray[1] = (hour() + acpOffset)  % 10;
  NumberArray[0] = (hour() / 10 + acpOffset) % 10;
}

// ************************************************************
// Show an integer configuration value
// ************************************************************
void loadNumberArrayConfInt(int confValue, int confNum) {
  NumberArray[5] = (confNum) % 10;
  NumberArray[4] = (confNum / 10) % 10;
  NumberArray[3] = (confValue / 1) % 10;
  NumberArray[2] = (confValue / 10) % 10;
  NumberArray[1] = (confValue / 100) % 10;
  NumberArray[0] = (confValue / 1000) % 10;
}

// ************************************************************
// Show a boolean configuration value
// ************************************************************
void loadNumberArrayConfBool(boolean confValue, int confNum) {
  int boolInt;
  if (confValue) {
    boolInt = 1;
  } else {
    boolInt = 0;
  }
  NumberArray[5] = (confNum) % 10;
  NumberArray[4] = (confNum / 10) % 10;
  NumberArray[3] = boolInt;
  NumberArray[2] = 0;
  NumberArray[1] = 0;
  NumberArray[0] = 0;
}

// ************************************************************
// Show an integer configuration value
// ************************************************************
void loadNumberArrayIP(byte byte1, byte byte2) {
  NumberArray[5] = (byte2) % 10;
  NumberArray[4] = (byte2 / 10) % 10;
  NumberArray[3] = (byte2 / 100) % 10;
  NumberArray[2] = (byte1) % 10;
  NumberArray[1] = (byte1 / 10) % 10;
  NumberArray[0] = (byte1 / 100) % 10;
}


// ************************************************************
// Do a single complete display, including any fading and
// dimming requested. Performs the display loop
// DIGIT_DISPLAY_COUNT times for each digit, with no delays. (1000)
// This is the heart of the display processing!
//
// Santosha - edit for 6 K155ID1 - 3 - 74hc595, Shifter lib
// use OnOff[i] - current state of digit - not able to read 74hc595 registers
// ************************************************************
void outputDisplay()
{

  float digitSwitchTimeFloat;
  int tmpDispType;
 // digitOff(5);  //all correctly but .. 36 cycles per second
 // digitOff(4);
 // digitOff(3);
 // digitOff(2);
 // digitOff(1);
 // digitOff(0);
// shifter.setAll(HIGH);  //turn off all 6 K155ID1
// shifter.write();    // set registers 74HC595

 /* byte l0 = NumberArray[0] ; //fast Shifter byte for lamp 0 -  tens hours
 byte l1 = NumberArray[1] ;
 byte l2 = NumberArray[2] ;
 byte l3 = NumberArray[3] ;
 byte l4 = NumberArray[4] ;
 byte l5 = NumberArray[5] ; //ones of seconds
 */
  // used to blank all leading digits if 0
  boolean leadingZeros = true;

  for ( int i = 0 ; i < DIGIT_COUNT ; i ++ )    {   // tubes 0 - 5,  5-ones seconds, 0 -tens hours
 //    OnOff[i]= 0; 

    if (blankTubes) {
      tmpDispType = BLANKED;
    } else {
      tmpDispType = displayType[i];
    }

    switch (tmpDispType) {
      case BLANKED:
        {
          digitOnTime[i] = DIGIT_DISPLAY_NEVER;
          digitOffTime[i] = DIGIT_DISPLAY_ON;  //0
          break;
        }
      case DIMMED:
        {
          digitOnTime[i] = DIGIT_DISPLAY_ON;  //0
          digitOffTime[i] = DIM_VALUE;
          break;
        }
      case BRIGHT:
        {
          digitOnTime[i] = DIGIT_DISPLAY_ON; //0    0 .... 1000  on -0 *20mks off 20000 mks ~~ always on
          digitOffTime[i]= DIGIT_DISPLAY_OFF;   //1000     
          //digitOffTime = 995;
          break;
        }
      case FADE:
      case NORMAL:
        {
          digitOnTime[i] = DIGIT_DISPLAY_ON;  //0
          digitOffTime[i] = digitOffCount;
          break;
        }
      case BLINK:
        {
          if (blinkState) {
            digitOnTime[i] = DIGIT_DISPLAY_ON;
            digitOffTime[i] = digitOffCount;
          } else {
            digitOnTime[i] = DIGIT_DISPLAY_NEVER;  // -1 = off all cycle
            digitOffTime[i] = DIGIT_DISPLAY_ON;
          }
          break;
        }
      case SCROLL:
        {
          digitOnTime[i] = DIGIT_DISPLAY_ON;
          digitOffTime[i] = digitOffCount;
          break;
        }
    }

    // Do scrollback when we are going to 0
    if ((NumberArray[i] != currNumberArray[i]) &&
        (NumberArray[i] == 0) &&
        scrollback) {
      tmpDispType = SCROLL;
    }

    // manage scrolling, count the digits down
    if (tmpDispType == SCROLL) {
      digitSwitchTime[i] = DIGIT_DISPLAY_OFF;   //1000
      if (NumberArray[i] != currNumberArray[i]) {
 
        if (fadeState[i] == 0) {
          // Start the fade
          fadeState[i] = scrollSteps;
        }
      
        if (fadeState[i] == 1) {
          // finish the fade  .. ??scroll
          fadeState[i] = 0;
         if (i>0)  currNumberArray[i] = currNumberArray[i-1]; //wrong - for fade not scroll  - fade digit  = digit - 1 
        //currNumberArray[i] = currNumberArray[i] -1 ; 
        } else if (fadeState[i] > 1) {
          // Continue the scroll countdown
          fadeState[i] = fadeState[i] - 1;
        }
      }
    // manage fading, each impression we show 1 fade step less of the old
    // digit and 1 fade step more of the new
    } else if (tmpDispType == FADE) {
      if (NumberArray[i] != currNumberArray[i]) {    // first time currNumberArray[i] = 10  - k155id1 turn off output when input code 10-15, nixie subroutine do the same 
        if (fadeState[i] == 0) {
         //    if (i != 5) {    // no fade for seconds lamp #5 (#0 - tens hours)
          // Start the fade   - this for old digit - darken, then off
          fadeState[i] = fadeSteps;       // set value for  Fade - 46 impr/sec  -> 45 max, 49 --> 47
          digitSwitchTime[i] = (int) fadeState[i] * fadeStep ;    // calc value - 2 when dark 20 when brightly    45*2 = 120 max bright when dark
      // turn on digit slowly
            digitOnTime[i] = ((int) fadeState[i] * fadeStep)+50  ;    // calc value - 1 when dark 10 when brightly    45*2 .. 3 = 120 max bright when dark  1 * 20 =20 'on timer' - bright,  45*10 =450 slow on

      //  }
       }
      }
      if (fadeState[i] == 1) {
        // finish the fade
        fadeState[i] = 0;
        currNumberArray[i] = NumberArray[i];
        digitSwitchTime[i] = DIGIT_DISPLAY_COUNT; //1000
          // digitOnTime[i] = 0 ;    //  1 * 20 =20 'on timer' - bright,  45*20 =900 slow on
      } else if (fadeState[i] > 1) {
        // Continue the fade
        fadeState[i] = fadeState[i] - 1;
        digitSwitchTime[i] = (int) fadeState[i] * fadeStep ;
         digitOnTime[i] = ((int) fadeState[i] * fadeStep)+50  ; // 450 'on time' (900 off) dark -> 3 brightly
      //  digitOnTime[i]=digitOnTime[i] - fadeStep;
       // if (digitOnTime[i] <0) digitOnTime[i]=0;
      }
    } else {
       // finish the fade - when time change - start fade - NumberArray will be next digit, currNumber - previous
      digitSwitchTime[i] = DIGIT_DISPLAY_COUNT;
    currNumberArray[i] = NumberArray[i];   // fadeState[i] =0 bulb is Fade but Fade not start, copy current digit to currNumberArray[i], wait for change time
      digitOnTime[i] = 0 ;
      // digitOnTime[i] = ((int) fadeState[i] * fadeStep)/2  ; 0*3  =0
    }
   //}
/*        
//for turn on lapm slowly (minutes - tens of seconds)
 if (tmpDispType == FADE) {

    }
 */
 
 // 6 tubes at one time first loop
  }
  // 6 tubes set - inner cycle 1000 times  (dispCount) (change -2-nd loop all together 6 digits)
    for (int timer = 0 ; timer < dispCount ; timer++) {
      if (timer == digitOnTime[0]) {    //fade or transition - turn on together 2 digit
      //  digitOn(i, currNumberArray[i],l0,l1,l2,l3,l4,l5);   // i=0 .. i=5 lamp ten of hour ... ones of seconds, currNumberArray[i] - digit to display at [i] position 
       digitOn(0, currNumberArray[0]);
      }
      if (timer == digitOnTime[1]) {
 
       digitOn(1, currNumberArray[1]);
      }
            if (timer == digitOnTime[2]) {
   
       digitOn(2, currNumberArray[2]);
      }
            if (timer == digitOnTime[3]) {
  
       digitOn(3, currNumberArray[3]);
      }
            if (timer == digitOnTime[4]) {

       digitOn(4, currNumberArray[4]);
      }
            if (timer == digitOnTime[5]) {
  
       digitOn(5, currNumberArray[5]);
      }

      if  (timer == digitSwitchTime[0]) {     //fade or transition - turn on together 2 digit
       // SetSN74141Chip(i,NumberArray[i],l0,l1,l2,l3,l4,l5); 
        SetSN74141Chip(0,NumberArray[0]);
      }
            if  (timer == digitSwitchTime[1]) {

        SetSN74141Chip(1,NumberArray[1]);
      }
      if  (timer == digitSwitchTime[2]) {
  
        SetSN74141Chip(2,NumberArray[2]);
      }
      if  (timer == digitSwitchTime[3]) {
 
        SetSN74141Chip(3,NumberArray[3]);
      }
      if  (timer == digitSwitchTime[4]) {
 
        SetSN74141Chip(4,NumberArray[4]);
      }
      if  (timer == digitSwitchTime[5]) {
  
        SetSN74141Chip(5,NumberArray[5]);
      }


         if (timer == digitOffTime[0]) {
         digitOff(0);
      }
           if (timer == digitOffTime[1]) {
        digitOff(1);
      }
           if (timer == digitOffTime[2]) {
        digitOff(2);
      }
           if (timer == digitOffTime[3]) {
        digitOff(3);
      }
           if (timer == digitOffTime[4]) {
        digitOff(4);
      }
           if (timer == digitOffTime[5]) {
        digitOff(5);
      }
    }
 

  // Deal with blink, calculate if we are on or off
  blinkCounter++;
  if (blinkCounter == BLINK_COUNT_MAX) {
    blinkCounter = 0;
    blinkState = !blinkState;
  }
}

// ************************************************************
// Set a digit with the given value and turn the HVGen on
// Assumes that all digits have previously been turned off
// by a call to "digitOff"
//
// Santosha - edit for 6 K155ID1 - 3 - 74hc595, Shifter lib 
// optimize for speed ! up to 300000 cycles/1s
// //next try optimize - 6 digit at one time
// ************************************************************
// void digitOn(int digit, int value, byte l0,byte l1,byte l2,byte l3,byte l4,byte l5)
void digitOn(int digit, int value) {

 OnOff[digit]= 1;         // now digit turns on  ..   по отладке 46 раз в секунду основной цикл - не моргает. (46000 /sec digit)
 
  nixie (digit*4,value);
  /*
  switch (digit) {
   // case 0: PORTC = PORTC | B00001000; break; // PC3 - equivalent to digitalWrite(ledPin_a_1,HIGH);
   case 0: {   // tens of hours tube

 // nixie (20, value);  //seconds - lamp 5 -6  - Shifter fast setPin 4 pins connected to decoder K155ID1 74141
 // nixie (8, value);  //minutes
 // nixie subroutine switch by value

  nixie (0, value);  //hours lamp 1-2
       }
   // case 1: PORTC = PORTC | B00000100; break; // PC2 - equivalent to digitalWrite(ledPin_a_2,HIGH);
   case 1: {

  nixie (4, value);

   }
   // case 2: PORTD = PORTD | B00010000; break; // PD4 - equivalent to digitalWrite(ledPin_a_3,HIGH);
   case 2: {

   nixie (8, value);  //minutes

   }
   
   // case 3: PORTD = PORTD | B00000100; break; // PD2 - equivalent to digitalWrite(ledPin_a_4,HIGH);
    case 3: {

   nixie (12, value);
 
   }
   // case 4: PORTD = PORTD | B00000010; break; // PD1 - equivalent to digitalWrite(ledPin_a_5,HIGH);
   case 4: {

  nixie (16, value);

   }
   // case 5: PORTD = PORTD | B00000001; break; // PD0 - equivalent to digitalWrite(ledPin_a_6,HIGH);

   case 5: {  //tube show ones of seconds
  nixie (20, value);  //seconds - lamp 5 -6 
 
   }

  }
*/
   

       // turn off (dynamic) 1/50000 s * digitOffTime
      if (OnOff[0] == 0 ) {          // state for this time - off digits if they turn off now 1/50/1000*6 sec  // выключаются цифры которые снижают яркость или сдвигаются или в эффекте затемнения
       shifter.setPin(0, HIGH);
       shifter.setPin(1, HIGH);
       shifter.setPin(2, HIGH);
       shifter.setPin(3, HIGH);
      }
    if (OnOff[1] == 0 ) {
       shifter.setPin(4, HIGH);
       shifter.setPin(5, HIGH);
       shifter.setPin(6, HIGH);
       shifter.setPin(7, HIGH);
    }
    if (OnOff[2] == 0 ) {
       shifter.setPin(8, HIGH);
       shifter.setPin(9, HIGH);
       shifter.setPin(10, HIGH);
       shifter.setPin(11, HIGH);
    }
    if (OnOff[3] == 0 ) {
       shifter.setPin(12, HIGH);
       shifter.setPin(13, HIGH);
       shifter.setPin(14, HIGH);
       shifter.setPin(15, HIGH);    
    }
    if (OnOff[4] == 0 ) {
       shifter.setPin(16, HIGH);
       shifter.setPin(17, HIGH);
       shifter.setPin(18, HIGH);
       shifter.setPin(19, HIGH);      
    }
    if (OnOff[5] == 0 ) {
       shifter.setPin(20, HIGH);
       shifter.setPin(21, HIGH);
       shifter.setPin(22, HIGH);
       shifter.setPin(23, HIGH);      
    }
 
  
     shifter.write();    // set registers 74HC595

  //SetSN74141Chip(value); // do here - shifter_write();

 // TCNT1 = 0; // Thanks Phil!
  // TCCR1A = tccrOn;
}

// ************************************************************
// Finish displaying a digit and turn the HVGen on
//
// Santosha - turn off all digits use Shifter - code 0xff - off 74141
// ************************************************************
void digitOff(int i) {
 // TCCR1A = tccrOff;
  //digitalWrite(anodePins[digit], LOW);
 OnOff[i] = 0;
  // turn all digits off - equivalent to digitalWrite(ledPin_a_n,LOW); (n=1,2,3,4,5,6) but much faster
 //shifter.setAll(HIGH);
  int j=i*4;
   
          shifter.setPin(j, HIGH);
       shifter.setPin(j+1, HIGH);
       shifter.setPin(j+2, HIGH);
       shifter.setPin(j+3, HIGH);
    
       
  /*
        if (OnOff[0] == 0 ) {          // state for this time - off digits if they turn off now
       shifter.setPin(0, HIGH);
       shifter.setPin(1, HIGH);
       shifter.setPin(2, HIGH);
       shifter.setPin(3, HIGH);
      }
    if (OnOff[1] == 0 ) {
       shifter.setPin(4, HIGH);
       shifter.setPin(5, HIGH);
       shifter.setPin(6, HIGH);
       shifter.setPin(7, HIGH);
    }
    if (OnOff[2] == 0 ) {
       shifter.setPin(8, HIGH);
       shifter.setPin(9, HIGH);
       shifter.setPin(10, HIGH);
       shifter.setPin(11, HIGH);
    }
    if (OnOff[3] == 0 ) {
       shifter.setPin(12, HIGH);
       shifter.setPin(13, HIGH);
       shifter.setPin(14, HIGH);
       shifter.setPin(15, HIGH);    
    }
    if (OnOff[4] == 0 ) {
       shifter.setPin(16, HIGH);
       shifter.setPin(17, HIGH);
       shifter.setPin(18, HIGH);
       shifter.setPin(19, HIGH);      
    }
    if (OnOff[5] == 0 ) {
       shifter.setPin(20, HIGH);
       shifter.setPin(21, HIGH);
       shifter.setPin(22, HIGH);
       shifter.setPin(23, HIGH);      
    }
  */
      
   // nixie (j,10);    //10=off
     shifter.write();    // set registers 74HC595

 /*
  PORTC = PORTC & B11110011;
  PORTD = PORTD & B11101000; */
  
}
// ************************************************************
// Decode the value to send to the 74141 and send it
// We do this via the decoder to allow easy adaptation to
// other pin layouts.
//
// Santosha - edit for 6 K155ID1 - 3 - 74hc595, Shifter lib - set one digit "value", set another as is
// decodeDigit not use - digit conection direct, 0-0 , 1-1 into nixie subroutine 
// not able reading current state of registers - calculate which digits need turn on at this time - !test now  (OK)
// 46 impression per sec  - 53 (correction) turn off 1 digit, OnOff variable not use - switch digits for Fade
// this use for Fade - one digit change to another
// Fade - 60 steps - each step * 120 minDim // 30-300 times see OK ?? (max bright 1000) not more 1 second - when 46 impr/ sec check - not more 45
// ************************************************************
// void SetSN74141Chip(int digit, int value, byte l0,byte l1,byte l2,byte l3,byte l4,byte l5)
void SetSN74141Chip(int digit, int value) {
// note that digit turns on but not save state into OnOff variable
OnOff[digit]= 1;         // now digit turns off (on for transition)  ..   по отладке 46 раз в секунду основной цикл - не моргает. (46000 /sec digit)
// test - turn on only current digit (faster) do not change state of another registry bits
 
  nixie (digit*4,value);
  /*
  switch (digit) {
   // case 0: PORTC = PORTC | B00001000; break; // PC3 - equivalent to digitalWrite(ledPin_a_1,HIGH);
   case 0: {
 
  nixie (0, value);  //hours lamp 1-2
   }
   // case 1: PORTC = PORTC | B00000100; break; // PC2 - equivalent to digitalWrite(ledPin_a_2,HIGH);
   case 1: {

  nixie (4, value);

   }
   // case 2: PORTD = PORTD | B00010000; break; // PD4 - equivalent to digitalWrite(ledPin_a_3,HIGH);
   case 2: {

  nixie (8, value);  //minutes

   }
   
   // case 3: PORTD = PORTD | B00000100; break; // PD2 - equivalent to digitalWrite(ledPin_a_4,HIGH);
    case 3: {

  nixie (12, value);

   }
   // case 4: PORTD = PORTD | B00000010; break; // PD1 - equivalent to digitalWrite(ledPin_a_5,HIGH);
   case 4: {

  nixie (16, value);

   }
   // case 5: PORTD = PORTD | B00000001; break; // PD0 - equivalent to digitalWrite(ledPin_a_6,HIGH);

   case 5: {
  nixie (20, value);  //seconds - lamp 5 -6 

   }
  }
  */
 // //useless .. set 2 - digit at one time is ok 
 
/*
 if (OnOff[0] == 0 ) {          // state for this time - off digits if they turn off now
       shifter.setPin(0, HIGH);
       shifter.setPin(1, HIGH);
       shifter.setPin(2, HIGH);
       shifter.setPin(3, HIGH);
      }
    if (OnOff[1] == 0 ) {
       shifter.setPin(4, HIGH);
       shifter.setPin(5, HIGH);
       shifter.setPin(6, HIGH);
       shifter.setPin(7, HIGH);
    }
    if (OnOff[2] == 0 ) {
       shifter.setPin(8, HIGH);
       shifter.setPin(9, HIGH);
       shifter.setPin(10, HIGH);
       shifter.setPin(11, HIGH);
    }
    if (OnOff[3] == 0 ) {
       shifter.setPin(12, HIGH);
       shifter.setPin(13, HIGH);
       shifter.setPin(14, HIGH);
       shifter.setPin(15, HIGH);    
    }
    if (OnOff[4] == 0 ) {
       shifter.setPin(16, HIGH);
       shifter.setPin(17, HIGH);
       shifter.setPin(18, HIGH);
       shifter.setPin(19, HIGH);      
    }
    if (OnOff[5] == 0 ) {
       shifter.setPin(20, HIGH);
       shifter.setPin(21, HIGH);
       shifter.setPin(22, HIGH);
       shifter.setPin(23, HIGH);      
    }
  
  */   
     shifter.write();    // set registers 74HC595 - turn on output

  /*
  // Map the logical numbers to the hardware pins we send to the SN74141 IC
  int decodedDigit = decodeDigit[num1];

  // Mask all digit bits to 0
  byte portb = PORTB;
  portb = portb & B11001010;

  // Set the bits we need
  switch ( decodedDigit )
  {
    case 0:                             break; // a=0;b=0;c=0;d=0
    case 1:  portb = portb | B00100000; break; // a=1;b=0;c=0;d=0
    case 2:  portb = portb | B00000100; break; // a=0;b=1;c=0;d=0
    case 3:  portb = portb | B00100100; break; // a=1;b=1;c=0;d=0
    case 4:  portb = portb | B00000001; break; // a=0;b=0;c=1;d=0
    case 5:  portb = portb | B00100001; break; // a=1;b=0;c=1;d=0
    case 6:  portb = portb | B00000101; break; // a=0;b=1;c=1;d=0
    case 7:  portb = portb | B00100101; break; // a=1;b=1;c=1;d=0
    case 8:  portb = portb | B00010000; break; // a=0;b=0;c=0;d=1
    case 9:  portb = portb | B00110000; break; // a=1;b=0;c=0;d=1
    default: portb = portb | B00110101; break; // a=1;b=1;c=1;d=1
  }
  PORTB = portb;
  */
  
}
// atmega8 basic
//Shiftout Portb.4 , Portb.0 , Seco , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Mine , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Hour , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Dat , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Month , 3 , 8 , 3
//Shiftout Portb.4 , Portb.0 , Year , 3 , 8 , 3

void nixie (int n, int val) {
  // use 74141
  switch (val) {
    case 0:
      shifter.setPin(n, LOW);
      shifter.setPin(n + 1, LOW);
      shifter.setPin(n + 2, LOW);
      shifter.setPin(n + 3, LOW);
      break;
    case 1:
       shifter.setPin(n, HIGH);
      shifter.setPin(n + 1, LOW);
      shifter.setPin(n + 2, LOW);
      shifter.setPin(n + 3, LOW);
      break;
    case 2:
       shifter.setPin(n, LOW);
      shifter.setPin(n + 1, HIGH);
      shifter.setPin(n + 2, LOW);
      shifter.setPin(n + 3, LOW);
      break;
    case 3:
      shifter.setPin(n, HIGH);
      shifter.setPin(n + 1, HIGH);
      shifter.setPin(n + 2, LOW);
      shifter.setPin(n + 3, LOW);
      break;
    case 4:
      shifter.setPin(n, LOW);
      shifter.setPin(n + 1, LOW);
      shifter.setPin(n + 2, HIGH);
      shifter.setPin(n + 3, LOW);
      break;
    case 5:
      shifter.setPin(n, HIGH);
      shifter.setPin(n + 1, LOW);
      shifter.setPin(n + 2, HIGH);
      shifter.setPin(n + 3, LOW);
      break;
    case 6:
      shifter.setPin(n, LOW);
      shifter.setPin(n + 1, HIGH);
      shifter.setPin(n + 2, HIGH);
      shifter.setPin(n + 3, LOW);
      break;
    case 7:
      shifter.setPin(n, HIGH);
      shifter.setPin(n + 1, HIGH);
      shifter.setPin(n + 2, HIGH);
      shifter.setPin(n + 3, LOW);
      break;
    case 8:
      shifter.setPin(n, LOW);
      shifter.setPin(n + 1, LOW);
      shifter.setPin(n + 2, LOW);
      shifter.setPin(n + 3, HIGH);
      break;
    case 9:
      shifter.setPin(n, HIGH);
      shifter.setPin(n + 1, LOW);
      shifter.setPin(n + 2, LOW);
      shifter.setPin(n + 3, HIGH);
      break;
    case 10: //off
      shifter.setPin(n, HIGH);
      shifter.setPin(n + 1, HIGH);
      shifter.setPin(n + 2, HIGH);
      shifter.setPin(n + 3, HIGH);
      break;
  }
}
// ************************************************************
// Display preset - apply leading zero blanking
// ************************************************************
void applyBlanking() {
  // If we are not blanking, just get out
  if (blankLeading == false) {
    return;
  }

  // We only want to blank the hours tens digit
  if (NumberArray[0] == 0) {
    if (displayType[0] != BLANKED) {
      displayType[0] = BLANKED;
    }
  }
}

// ************************************************************
// Display preset
// ************************************************************
void allFadeOrNormal(boolean blanking) {
  if (fade) {
    allFade();
  } else {
    allNormal();
  }

  if (blanking) {
    applyBlanking();
  }
}

// ************************************************************
// Display preset
// ************************************************************
void allFade() {
  if (displayType[0] != FADE) displayType[0] = FADE;
  if (displayType[1] != FADE) displayType[1] = FADE;
  if (displayType[2] != FADE) displayType[2] = FADE;
  if (displayType[3] != FADE) displayType[3] = FADE;
  if (displayType[4] != FADE) displayType[4] = FADE;
  if (displayType[5] != FADE) displayType[5] = FADE;
}

// ************************************************************
// Display preset
// ************************************************************
void allBright() {
  if (displayType[0] != BRIGHT) displayType[0] = BRIGHT;
  if (displayType[1] != BRIGHT) displayType[1] = BRIGHT;
  if (displayType[2] != BRIGHT) displayType[2] = BRIGHT;
  if (displayType[3] != BRIGHT) displayType[3] = BRIGHT;
  if (displayType[4] != BRIGHT) displayType[4] = BRIGHT;
  if (displayType[5] != BRIGHT) displayType[5] = BRIGHT;
}

// ************************************************************
// highlight years taking into account the date format
// ************************************************************
void highlightYearsDateFormat() {
  switch (dateFormat) {
    case DATE_FORMAT_YYMMDD:
      highlight0and1();
      break;
    case DATE_FORMAT_MMDDYY:
      highlight4and5();
      break;
    case DATE_FORMAT_DDMMYY:
      highlight4and5();
      break;
  }
}

// ************************************************************
// highlight years taking into account the date format
// ************************************************************
void highlightMonthsDateFormat() {
  switch (dateFormat) {
    case DATE_FORMAT_YYMMDD:
      highlight2and3();
      break;
    case DATE_FORMAT_MMDDYY:
      highlight0and1();
      break;
    case DATE_FORMAT_DDMMYY:
      highlight2and3();
      break;
  }
}

// ************************************************************
// highlight days taking into account the date format
// ************************************************************
void highlightDaysDateFormat() {
  switch (dateFormat) {
    case DATE_FORMAT_YYMMDD:
      highlight4and5();
      break;
    case DATE_FORMAT_MMDDYY:
      highlight2and3();
      break;
    case DATE_FORMAT_DDMMYY:
      highlight0and1();
      break;
  }
}

// ************************************************************
// Display preset, highlight digits 0 and 1
// ************************************************************
void highlight0and1() {
  DIM_VALUE = 3;
  if (displayType[0] != BRIGHT) displayType[0] = BRIGHT;
  if (displayType[1] != BRIGHT) displayType[1] = BRIGHT;
  if (displayType[2] != DIMMED) displayType[2] = DIMMED;
  if (displayType[3] != DIMMED) displayType[3] = DIMMED;
  if (displayType[4] != DIMMED) displayType[4] = DIMMED;
  if (displayType[5] != DIMMED) displayType[5] = DIMMED;
}

// ************************************************************
// Display preset, highlight digits 2 and 3
// ************************************************************
void highlight2and3() {
    DIM_VALUE = 3;
  if (displayType[0] != DIMMED) displayType[0] = DIMMED;
  if (displayType[1] != DIMMED) displayType[1] = DIMMED;
  if (displayType[2] != BRIGHT) displayType[2] = BRIGHT;
  if (displayType[3] != BRIGHT) displayType[3] = BRIGHT;
  if (displayType[4] != DIMMED) displayType[4] = DIMMED;
  if (displayType[5] != DIMMED) displayType[5] = DIMMED;
}

// ************************************************************
// Display preset, highlight digits 4 and 5
// ************************************************************
void highlight4and5() {
    DIM_VALUE = 3;
  if (displayType[0] != DIMMED) displayType[0] = DIMMED;
  if (displayType[1] != DIMMED) displayType[1] = DIMMED;
  if (displayType[2] != DIMMED) displayType[2] = DIMMED;
  if (displayType[3] != DIMMED) displayType[3] = DIMMED;
  if (displayType[4] != BRIGHT) displayType[4] = BRIGHT;
  if (displayType[5] != BRIGHT) displayType[5] = BRIGHT;
}

// ************************************************************
// Display preset
// ************************************************************
void allNormal() {
    DIM_VALUE =   MIN_DIM_DEFAULT;
  if (displayType[0] != NORMAL) displayType[0] = NORMAL;
  if (displayType[1] != NORMAL) displayType[1] = NORMAL;
  if (displayType[2] != NORMAL) displayType[2] = NORMAL;
  if (displayType[3] != NORMAL) displayType[3] = NORMAL;
  if (displayType[4] != NORMAL) displayType[4] = NORMAL;
  if (displayType[5] != NORMAL) displayType[5] = NORMAL;
}

// ************************************************************
// Display preset
// ************************************************************
void displayConfig() {
  if (displayType[0] != BRIGHT) displayType[0] = BRIGHT;
  if (displayType[1] != BRIGHT) displayType[1] = BRIGHT;
  if (displayType[2] != BRIGHT) displayType[2] = BRIGHT;
  if (displayType[3] != BRIGHT) displayType[3] = BRIGHT;
  if (displayType[4] != BLINK)  displayType[4] = BLINK;
  if (displayType[5] != BLINK)  displayType[5] = BLINK;
}

// ************************************************************
// Display preset
// ************************************************************
void displayConfig3() {
  if (displayType[0] != BLANKED) displayType[0] = BLANKED;
  if (displayType[1] != NORMAL) displayType[1] = BRIGHT;
  if (displayType[2] != NORMAL) displayType[2] = BRIGHT;
  if (displayType[3] != NORMAL) displayType[3] = BRIGHT;
  if (displayType[4] != BLINK)  displayType[4] = BLINK;
  if (displayType[5] != BLINK)  displayType[5] = BLINK;
}

// ************************************************************
// Display preset
// ************************************************************
void displayConfig2() {
  if (displayType[0] != BLANKED) displayType[0] = BLANKED;
  if (displayType[1] != BLANKED) displayType[1] = BLANKED;
  if (displayType[2] != NORMAL) displayType[2] = BRIGHT;
  if (displayType[3] != NORMAL) displayType[3] = BRIGHT;
  if (displayType[4] != BLINK)  displayType[4] = BLINK;
  if (displayType[5] != BLINK)  displayType[5] = BLINK;
}

// ************************************************************
// Display preset
// ************************************************************
void allBlanked() {
  if (displayType[0] != BLANKED) displayType[0] = BLANKED;
  if (displayType[1] != BLANKED) displayType[1] = BLANKED;
  if (displayType[2] != BLANKED) displayType[2] = BLANKED;
  if (displayType[3] != BLANKED) displayType[3] = BLANKED;
  if (displayType[4] != BLANKED) displayType[4] = BLANKED;
  if (displayType[5] != BLANKED) displayType[5] = BLANKED;
}

// ************************************************************
// Reset the seconds to 00
// ************************************************************
void resetSecond() {
  byte tmpSecs = 0;
  setTime(hour(), minute(), tmpSecs, day(), month(), year());
  setRTC();
}

// ************************************************************
// increment the time by 1 Sec
// ************************************************************
void incSecond() {
  byte tmpSecs = second();
  tmpSecs++;
  if (tmpSecs >= SECS_MAX) {
    tmpSecs = 0;
  }
  setTime(hour(), minute(), tmpSecs, day(), month(), year());
  setRTC();
}

// ************************************************************
// increment the time by 1 min
// ************************************************************
void incMins() {
  byte tmpMins = minute();
  tmpMins++;
  if (tmpMins >= MINS_MAX) {
    tmpMins = 0;
  }
  setTime(hour(), tmpMins, 0, day(), month(), year());
  setRTC();
}

// ************************************************************
// increment the time by 1 hour
// ************************************************************
void incHours() {
  byte tmpHours = hour();
  tmpHours++;

  if (tmpHours >= HOURS_MAX) {
    tmpHours = 0;
  }
  setTime(tmpHours, minute(), second(), day(), month(), year());
  setRTC();
}

// ************************************************************
// increment the date by 1 day
// ************************************************************
void incDays() {
  byte tmpDays = day();
  tmpDays++;

  int maxDays;
  switch (month())
  {
    case 4:
    case 6:
    case 9:
    case 11:
      {
        maxDays = 31;
        break;
      }
    case 2:
      {
        // we won't worry about leap years!!!
        maxDays = 28;
        break;
      }
    default:
      {
        maxDays = 31;
      }
  }

  if (tmpDays > maxDays) {
    tmpDays = 1;
  }
  setTime(hour(), minute(), second(), tmpDays, month(), year());
  setRTC();
}

// ************************************************************
// increment the month by 1 month
// ************************************************************
void incMonths() {
  byte tmpMonths = month();
  tmpMonths++;

  if (tmpMonths > 12) {
    tmpMonths = 1;
  }
  setTime(hour(), minute(), second(), day(), tmpMonths, year());
  setRTC();
}

// ************************************************************
// increment the year by 1 year
// ************************************************************
void incYears() {
  byte tmpYears = year() % 100;
  tmpYears++;

  if (tmpYears > 50) {
    tmpYears = 15;
  }
  setTime(hour(), minute(), second(), day(), month(), 2000 + tmpYears);
  setRTC();
}

// ************************************************************
// Check the blanking
// ************************************************************
boolean checkBlanking() {
  // Check day blanking, but only when we are in
  // normal time mode
  if (currentMode == MODE_TIME) {
    switch (dayBlanking) {
      case DAY_BLANKING_NEVER:
        return false;
      case DAY_BLANKING_HOURS:
        return getHoursBlanked();
      case DAY_BLANKING_WEEKEND:
        return ((weekday() == 1) || (weekday() == 7));
      case DAY_BLANKING_WEEKEND_OR_HOURS:
        return ((weekday() == 1) || (weekday() == 7)) || getHoursBlanked();
      case DAY_BLANKING_WEEKEND_AND_HOURS:
        return ((weekday() == 1) || (weekday() == 7)) && getHoursBlanked();
      case DAY_BLANKING_WEEKDAY:
        return ((weekday() > 1) && (weekday() < 7));
      case DAY_BLANKING_WEEKDAY_OR_HOURS:
        return ((weekday() > 1) && (weekday() < 7)) || getHoursBlanked();
      case DAY_BLANKING_WEEKDAY_AND_HOURS:
        return ((weekday() > 1) && (weekday() < 7)) && getHoursBlanked();
      case DAY_BLANKING_ALWAYS:
        return true;
    }
  }
}

// ************************************************************
// If we are currently blanked based on hours
// ************************************************************
boolean getHoursBlanked() {
  if (blankHourStart > blankHourEnd) {
    // blanking before midnight
    return ((hour() >= blankHourStart) || (hour() < blankHourEnd));
  } else if (blankHourStart < blankHourEnd) {
    // dim at or after midnight
    return ((hour() >= blankHourStart) && (hour() < blankHourEnd));
  } else {
    // no dimming if Start = End
    return false;
  }
}

// ************************************************************
// Set the tubes and LEDs blanking variables based on blanking mode and 
// blank mode settings
// ************************************************************
void setTubesAndLEDSBlankMode() {
  if (blanked) {
    switch(blankMode) {
      case BLANK_MODE_TUBES:
      {
        blankTubes = true;
        blankLEDs = false;
        break;
      }
      case BLANK_MODE_LEDS:
      {
        blankTubes = false;
        blankLEDs = true;
        break;
      }
      case BLANK_MODE_BOTH:
      {
        blankTubes = true;
        blankLEDs = true;
        break;
      }
    }
  } else {
    blankTubes = false;
    blankLEDs = false;
  }
}

/*
// ************************************************************
// Show a random RGB colour: used to indicate a factory reset
// ************************************************************
void randomRGBFlash(int delayVal) {
  digitalWrite(tickLed, HIGH);
  if (random(3) == 0) {
    digitalWrite(RLed, HIGH);
  }
  if (random(3) == 0) {
    digitalWrite(GLed, HIGH);
  }
  if (random(3) == 0) {
    digitalWrite(BLed, HIGH);
  }
  delay(delayVal);
  digitalWrite(tickLed, LOW);
  digitalWrite(RLed, LOW);
  digitalWrite(GLed, LOW);
  digitalWrite(BLed, LOW);
  delay(delayVal);
}  */

/*
// ************************************************************
// Used to set the LEDs during test mode
// ************************************************************
void setLedsTestPattern(unsigned long currentMillis) {
  unsigned long currentSec = currentMillis / 1000;
  byte phase = currentSec % 4;
  digitalWrite(tickLed, LOW);
  digitalWrite(RLed, LOW);
  digitalWrite(GLed, LOW);
  digitalWrite(BLed, LOW);

  if (phase == 0) {
    digitalWrite(tickLed, HIGH);
  }
  
  if (phase == 1) {
    digitalWrite(RLed, HIGH);
  }
  
  if (phase == 2) {
    digitalWrite(GLed, HIGH);
  }
  
  if (phase == 3) {
    digitalWrite(BLed, HIGH);
  }
}  */

// ************************************************************
// Jump to a new position in the menu - used to skip unused items
// ************************************************************
void setNewNextMode(int newNextMode) {
  nextMode = newNextMode;
  currentMode = newNextMode - 1;
}


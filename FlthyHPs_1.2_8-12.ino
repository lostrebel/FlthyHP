////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///                                    Flthy Holoprojectors v1.2                                     ///
///                                       by Ryan Sondgeroth                                         ///
///                                         aka FlthyMcNsty                                          ///
///                                            8-16-2016                                             ///
///                                                                                                  ///
///      Combines the movement and display functions of each Holoprojector into a single easy        ///
///     to control sketch.  It utilizes 7 LED NeoPixel boards inside each HP to produce a more       ///
///        life-like representation of a hologram projection. Servo control is handled by a          ///
///      Adafruit 16 Channel I2C Breakout board, giving increased flexibility over the I2C bus,      ///
///     as well as the ability to operate the HP Servos with Neopixel LEDs from the same sketch.     ///
///       The regular arduino servo library doesn't get along with Adafruits Neopixel library,       ///
///                  Big Happy Dude's Slow Servo Library is much better any way, :-D                 ///
///                                                                                                  ///
///                          Rhyno45 is a Wanker, in case Ya'll didn't know!                         ///
///                                                                                                  ///
///   Version History:                                                                               ///
///     v1.1 - Added One Time HP Twitch Function                                                     ///
///     v1.2 - Fixed Center Positioning Bug and others; streamlined Servo & LED functions;           ///
///            optimized variable use & streamlined variable containers; increased servo position    ///
///            presets to 8 directions; added status light on Serial/I2C receive; added a            ///
///            random color option; added support for the RGBW version of the Adafruit Jewels;       ///
///            added a new HP wag function; improved the flicker behavior and color to white         ///
///            balance on the default LED function; added speed option to Dim Pulse function;        ///
///            corrected unwanted behavior when attemping to clear LEDs while auto sequence is       ///
///            active                                                                                ///
///                                                                                                  ///
///                                      Special thanks to...                                        ///
///   LostRebel and Knightshade for significant input on both the general functions of the system    ///
///          and this code!  It helped me make vast improvements in operation and effeciency.        ///                                  
///                                                                                                  ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///     Commands and Structure                                                                       ///
///                                                                                                  ///.
///     DT##C or DT##CS or DT##P                                                                     ///
///     D - the HP designator with F-Front HP, R-Rear HP, T-Top HP, and A-All 3 HPs                  ///
///     T - the Sequence Type is either 0-Led Fuctions and 1-Servo Functions                         ///
///    ## - the Sequence Value including leading zero if necessary, ie sequence 3 is 03              ///
///     C - (Optional), the Color integer value from list below:                                     ///
///        Basic Color Integer Values                                                                ///
///           1 = Red                                                                                ///
///           2 = Yellow                                                                             ///
///           3 = Green                                                                              ///
///           4 = Cyan (Aqua)                                                                        ///
///           5 = Blue                                                                               ///
///           6 = Magenta                                                                            ///
///           7 = Orange                                                                             ///
///           8 = Purple                                                                             ///
///           9 = White                                                                              ///
///           0 = Random                                                                             ///
///     S - (Optional), Speed setting integer for the Dim Pulse LED function below (0-9)             ///
///     P - (Optional), the Position integer value from list below:                                  ///
///        Preset Position Integer Values                                                            ///
///           0 = Down                                                                               ///
///           1 = Center                                                                             ///
///           2 = Up                                                                                 ///
///           3 = Left                                                                               ///
///           4 = Upper Left                                                                         ///
///           5 = Lower Left                                                                         ///
///           6 = Right                                                                              ///
///           7 = Upper Right                                                                        ///
///           8 = Lower Right                                                                        /// 
///                                                                                                  ///
///     D001    - Leia Sequence, Random shades of blue to mimic Leia Hologram                        ///
///     D002C   - Color Projector Sequence, Like Leia above but using color command value            ///
///     D003CS  - Dim Pulse Sequence, Color slowly pulses on and off                                 ///
///     D004C   - Cycle Sequence, using color command value                                          ///
///     D005C   - Short Circuit, Led flashes on and off with interval slowing over time              ///
///     D006C   - Toggles Color, Simply sets LEDs tp solid color value.                              ///
///     D007    - Rainbow Sequence                                                                   ///
///     D098    - Clears LED, Disables Auto LED Sequence                                             ///
///     D099    - Clears LED, Enables Auto LED Sequence                                              ///
///                                                                                                  ///
///     D101P   - Sends HP to a Preset Position*                                                     ///
///     D102    - Enables RC Control on HP (Left/Right)*                                             ///
///     D103    - Enables RC Control on HP (Up/Down)*                                                ///
///     D104    - Sends HP to a Random Position                                                      ///
///     D105    - Wags HP Left/Right 5 times*                                                        ///
///     D106    - Wags HP Up/Down 5 times*                                                           ///
///     D198    - Disables Auto HP Twitch                                                            ///
///     D199    - Enables Auto HP Twitch                                                             ///
///                                                                                                  ///
///       S1    - Leia Mode (Front HP in Down Position, Leia LED Sequence, all other HPs disabled)*  ///
///       S8    - Clear all LEDs, Disable Auto HP Twitch, Disable Auto LED Sequence                  ///
///       S9    - Clear all LEDs, Enable Auto HP Twitch, Enable Auto LED Sequence                    ///
///                                                                                                  ///
///   * Function disabled or severely limited when Basic HP Positioning in enabled.                  /// 
///     I recomend using Preset Position Coordinates                                                 ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///                                  Required Libraries                                              ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>                        //
#include <Adafruit_NeoPixel.h>           // Source: https://github.com/adafruit/Adafruit_NeoPixel
#include <Adafruit_PWMServoDriver.h>     // Source: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <Servos.h>                      // Source: https://drive.google.com/file/d/0B5B8A65frsBgZ3VpeGxpM1lzaFE/edit  Thanks BHD!

#define VERSION 2
#define HPCOUNT 3
#define HPPOSITIONS 9

////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///                                  Start User Adjustable Settings                                  ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
///*****                 Assign Arduino IC2 Address Below                 *****///
///*****                                                                  *****///
///*****  An I2C address of 25 has been standardized for other HP devices *****///
///*****       so we will use it to. It can be changed here if needed.    *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
#define I2CADDRESS 0x19  // 25 in Hexadecmal


//////////////////////////////////////////////////////////////////////////////////
///*****               Assign Servo Board IC2 Address Below               *****///
///*****                                                                  *****///
///*****           The I2C address of the servo breakout board can        *****/// 
///*****       be set below, and corresponds with the address selected    *****///
///*****          on the board using the following jumper settings:       *****///
///*****                                                                  *****///
///*****      Address= 0x40 (64) Offset = binary 00000 (no jumpers)       *****///
///*****      Address= 0x41 (65) Offset = binary 00001 (A0)               *****///
///*****      Address= 0x42 (66) Offset = binary 00010 (A1)               *****///
///*****      Address= 0x43 (67) Offset = binary 00011 (A0 & A1)          *****///
///*****      Address= 0x44 (68) Offset = binary 00100 A2)                *****///
///*****      Address= 0x45 (69) Offset = binary 00101 (A2 & A0)          *****/// 
///*****      Address= 0x46 (70) Offset = binary 00110 (A2 & A1)          *****/// 
///*****      Address= 0x47 (71) Offset = binary 00111 (A2, A1, & A0)     *****/// 
///*****                 ...So on and so forth                            *****/// 
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
#define SERVOI2CADDRESS 0x40  // 64 in Hexadecmal


//////////////////////////////////////////////////////////////////////////////////
///*****               Arduino Digital Pin Assignments for LEDs           *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
const uint8_t LEDpins[HPCOUNT]   = {2,3,4};   // {Front, Rear, Top}

//////////////////////////////////////////////////////////////////////////////////////////
///*****                         Servo Board Pin Assignments                      *****///
///*****                                                                          *****///
///*****  HPpins[HPCOUNT][2] = {{Front 1, Front2}, {Rear 1, Rear 2}, {Top 1, Top 2}};   *****///
///*****                                                                          *****///
//////////////////////////////////////////////////////////////////////////////////////////    
const uint8_t HPpins[HPCOUNT][2] = {{0,1},{2,3},{4,5}};           // Front HP Pins, Rear HP Pins, Top HP Pins


//////////////////////////////////////////////////////////////////////////////////
///*****       Arduino Digital Pin Assignment for Output Enable (OE)      *****///
///*****                                                                  *****///
///*****  Output Enable (OE) is used to disable/enable servos between     *****///
///*****          movements to minimize unnecessary hum/noise.            *****///
///*****                                                                  *****///
///*****    The only reasons you would want to disable this is if...      *****///
///*****    1. You wish to use an additional microcontroller to send      *****/// 
///*****       servo control commands via the Adafruit library.           *****///
///*****    2. You want to use additional servos on the same servo        *****///
///*****        breakout board as your HP servos.                         *****///
///*****                                                                  *****///
///*****       Neither scenario is recommended because the increased      *****///
///*****   complexity far outweighs the costs savings of not purchasing   *****///
///*****                   an additional servo board.                     *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
#define OUTPUT_ENABLED_PIN 10       
#define OUTPUT_ENABLED_ON true    
  

//////////////////////////////////////////////////////////////////////////////////
///*****                                                                  *****///
///*****            Arduino Digital Pin Assignment for RC Input           *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
#define RCPIN 9

//////////////////////////////////////////////////////////////////////////////////
///*****             Pulse Width Range of RC Stick/Controller             *****///
///*****               assigned to control HP movement.                   *****///
//////////////////////////////////////////////////////////////////////////////////
const int RCRange[2] = {1250, 1750};     // {Min, Max}
    

//////////////////////////////////////////////////////////////////////////////////
///*****                    Adafruit Jewel Board Version                  *****///
///*****                                                                  *****///
///*****       The Adafruit Jewels now come in RGB and RGBW versions,     *****///
///*****        uncomment the following if you have the RGBW version.     *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
#define NEO_JEWEL_RGBW

//////////////////////////////////////////////////////////////////////////////////
///*****                          LED Brightness                          *****///
///*****                                                                  *****///
///*****                 Adjust LED Brightness Level (0-255)              *****///
//////////////////////////////////////////////////////////////////////////////////
#define BRIGHT 100  // Set LED Brightness Level (0-255)

//////////////////////////////////////////////////////////////////////////////////
///*****                Enable/Disable LED & Servo Auto Twitch            *****///
///*****                                                                  *****///
///*****                   Enabled = true, Disabled = false               *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
boolean enableTwitchLED[HPCOUNT]  = {true,true,true};  // Leds:   Front, Rear, Top 
boolean enableTwitchHP[HPCOUNT] = {true,true,true};    // Servos: Front, Rear, Top


///////////////////////////////////////////////////////////////////////////////////
///*****               LED & Servo Auto Twitch Interval Ranges             *****///
///*****                                                                   *****///
///*****      Use these values to select the range (min, max) in           *****///
///*****     seconds to randomly select the next auto twitch interval.     *****///
///*****      This helps give the illusion these displays & movements      *****///
///*****                     are totally autonomous.                       *****///
///*****                                                                   *****///
///////////////////////////////////////////////////////////////////////////////////  
    
const unsigned int LEDTwitchInterval[HPCOUNT][2] = {{240,360},           // (4mins-6mins) Enter min and max seconds Front HP Led Twitches  
                                                    {300,420},           // (5mins-7mins) Enter min and max seconds Rear HP Led Twitches 
                                                    {300,420}};          // (5mins-7mins) Enter min and max seconds Top HP Led Twitches    
    
const unsigned int HPTwitchInterval[HPCOUNT][2] = {{45,120},             // (45s-2mins) Enter min and max seconds Front HP Servo Twitches  
                                                   {60,180},             // (1min-3mins) Enter min and max seconds Rear HP Servo Twitches
                                                   {60,180}};            // (1min-3mins) Enter min and max seconds Top HP Servo Twitches  

///////////////////////////////////////////////////////////////////////////////////
///*****                  LED Auto Twitch Run Time Ranges                  *****///
///*****                                                                   *****///
///*****    Use these values to select the range (min, max) in seconds     *****///
///*****   to randomly select how long the sequence runs each time the     *****///
///*****                      LED twitch is executed.                      *****///
///*****                                                                   *****///
///////////////////////////////////////////////////////////////////////////////////  
const unsigned int LEDTwitchRunInterval[HPCOUNT][2] = {{5,25},   // (5-25s) Front LED Runtime
                                                       {5,25},   // (5-25s) Rear LED Runtime
                                                       {5,25}};  // (5-25s) Top LED Runtime  

//////////////////////////////////////////////////////////////////////////////////
///*****                           Servo Speed                            *****///
///*****                                                                  *****///
///*****      Tweak these value to adjust the range of movement speeds    *****///
///*****     for your HP servos. Values in ms, Lower values results in    *****///
///*****      faster movement.   By using a range, the servos will not    *****///
///*****     always move at the same speed giving the subtle illusion     *****/// 
///*****           that each movement is purposeful and unique.           *****///
///*****                   They say its the little things.                *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////       
const int SERVO_SPEED[2] = {150, 400};


//////////////////////////////////////////////////////////////////////////////////
///*****                Preset HP Servo Position Coordinates              *****///
///*****                                                                  *****///
///*****   Since each Builder's droid is assembled differently, we found  *****///
///*****   using position coordinates generated randomly on the fly was   *****///
///*****   problematic due to the occasional servo movement attempting   *****///
///*****   to travel beyond the outer cowl path.  Therefore we use a set  *****///
///*****   of 9 position pairs for each HP, set to ensure they lie with   *****///
///*****    in a freely accessible position. These coordinates consist    *****///
///*****     of the servo position of each servo using its pulse width    *****///
///*****    value in microseconds. I suggest using a servo tester with    *****///
///*****    a digital display to find these position pairs while the HPs  *****///
///*****                      are mounted in the dome.                    *****///
///*****                                                                  *****///
///*****     The first coordinate pair in each row references the DOWN    *****/// 
///*****    position of the HP, the second pair is the CENTER position,   *****/// 
///*****    and the 3rd pair the UP position.  LEFT, UPPER LEFT & LOWER   *****///
///*****      LEFT are plotted using the fourth, fifth and sixth pairs    *****///
///*****      while RIGHT, UPPER RIGHT & LOWER RIGHT are plotted using    *****/// 
///*****               the seventh, eight and ninth pairs.                *****/// 
///*****                                                                  *****///
///*****     Random twitch movement is then chosen to randomly travel     *****/// 
///*****     between these 9 safe points minimizing the chance of the     *****/// 
///*****            movement being impeded causing excessive and          *****/// 
///*****                     unpleasant servo noise                       *****/// 
///*****                                                                  *****///
///*****          If you would rather do it the simple way, set           *****///
///*****   enableBasicHP to true and it will use the Center, Min and Max  *****///
///*****    values set in the HPposBasic array. These are based on the    *****///
///*****      assumed default center value of a normal servo so it is     *****///
///*****    important that your servo linkage is installed in a manner    *****///
///*****       where the center servo position corresponds with the       *****///
///*****       orientation you want your HP to be in when centered.       *****///
///*****                                                                  *****///
///*****          RC mode requires position coordinates to work well,     *****///
///*****           so Basic Positioning must be disabled by setting       *****///
///*****                        enableBasicHP to false.                   *****///
///*****                                                                  *****///
///*****     IT IS IMPORTANT TO NOTE...if you disassemble your HPs and    *****///
///*****     servo linkage after setting the following values, you will   *****///
///*****    most likely need to recheck and reset them as the chances of  *****///
///*****       reinstalling them exactly the same way is approximately    *****///
///*****                             3,720 to 1.                          *****///
///*****                                                                  *****///
///*****   To get the most out of this sytem, it is recomended that you   *****///
///*****    disable basic positioning and use the 9 custom locations for  *****///
///*****                         for each of your HPs                     *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////     
const int HPpos[HPCOUNT][HPPOSITIONS][2] = {{{1434,1800},   // Front HP Down
                                             {1411,1553},   // Front HP Center
                                             {1409,1166},   // Front HP Up
                                             {1212,1516},   // Front HP Left
                                             {1236,1283},   // Front HP Upper Left
                                             {1226,1715},   // Front HP Lower Left
                                             {1675,1605},   // Front HP Right 
                                             {1652,1355},   // Front HP Upper Right
                                             {1667,1768}},  // Front HP Lower Right
                                            {{1601,1084},  // Rear HP Down
                                             {1437,1537},   // Rear HP Center
                                             {1385,1954},   // Rear HP Up
                                             {1343,1406},   // Rear HP Left
                                             {1633,1764},   // Rear HP Upper Left
                                             {1803,1283},   // Rear HP Lower Left
                                             {1343,1406},   // Rear HP Right
                                             {1633,1764},   // Rear HP Upper Right
                                             {1803,1283}},  // Rear HP Lower Right
                                            {{1547,1038},   // Top HP Down
                                             {1431,1395},   // Top HP Center
                                             {1473,1793},   // Top HP Up
                                             {1633,1737},   // Top HP Left
                                             {1324,1200},   // Top HP Upper Left
                                             {1677,1151},   // Top HP Lower Left
                                             {1343,1406},   // Top HP Right
                                             {1633,1764},   // Top HP Upper Right
                                             {1803,1283}}}; // Top HP Lower right
                  
#define ENABLEBASICHP false                            // false - Disabled, true- Enabled, Basic Servo Positioning
const int HPposBasic[2][3] = {{1500,1300,1700},        // HP Servo 1 Values for Basic Mode (Center, Min, Max)
                              {1500,1300,1700}};       // HP Servo 2 Values for Basic Mode (Center, Min, Max)

///////////////////////////////////////////////////////////////////////////////////
///*****                      Default Color Settings                       *****///
///*****                                                                   *****///
///*****     Select Default Colors for certain LED sequences using the     *****///
///*****                       integer values below:                       *****///
///*****                                                                   *****///
///*****             Basic Color Integer Values                            *****///
///*****               1 = Red                                             *****///
///*****               2 = Yellow                                          *****///
///*****               3 = Green                                           *****///
///*****               4 = Cyan (Aqua)                                     *****///
///*****               5 = Blue                                            *****///
///*****               6 = Magenta                                         *****///
///*****               7 = Orange                                          *****///
///*****               8 = Purple                                          *****///
///*****               9 = White                                           *****///
///*****               0 = Random                                          *****///
///*****                                                                   *****///
///////////////////////////////////////////////////////////////////////////////////        
#define DEFAULTCOLOR 5      // Blue, Color integer value for the hue of default sequence. This is to appease MKelly and MKarp's whiney little asses, lol
#define SHORTCOLOR   7      // Orange, Color integer value for the hue of ShortCircuit Message. 


//////////////////////////////////////////////////////////////////////////////////
///*****               Dim Pulse Speed Range and Default Speed            *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
#define DIMPULSESPEED 2                         // Deafault speed if no value is given
const int dimPulseSpeedRange[2] = {5, 75};      // Range used to map to value options 0-9, Lower is faster.


//////////////////////////////////////////////////////////////////////////////////
///*****                      HP SERVO WAG COUNT                          *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
#define WAGCOUNT 5        //  Number of times to wag up/down or left right during Wag Function

////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///                                  End User Adjustable Settings                                    ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
///*****  Sequence/Function Varaiables, Containers & Counters *****///
//////////////////////////////////////////////////////////////////////

// General counters and timers shared across more than one animation
unsigned long tCounter[HPCOUNT] = {0, 0, 0};
unsigned int  interval[HPCOUNT] = {100, 100, 100};


// Short Circuit Animation
#define SCMAXLOOPS 20
byte         SCloop[HPCOUNT] = {0, 0, 0};   // Loop counter for short circuit animation.
boolean      SCflag[HPCOUNT] = {false, false, false};
unsigned int SCinterval[HPCOUNT] = {10, 10, 10};

// Cycle Animation
byte Frame[HPCOUNT] = {0, 0, 0};

// Wag Animation
byte WagCount[HPCOUNT] = {-1,-1,-1};  // Use of negative value to determine first loop 
long WagTimer[HPCOUNT] = {0,0,0};

int RCinputCh1;

long twitchLEDTime[HPCOUNT] = {4000, 4000, 4000}; //LEDS Start up 4 seconds after boot
long twitchHPTime[HPCOUNT]  = {4000, 4000, 4000}; //HPs Start 4 seconds after boot
long twitchLEDRunTime[HPCOUNT];

unsigned long ledtimer = 0;      

// Output enabled timers and flag
unsigned long OETime = 0;  // The time at which output was enabled
boolean       OEFlag = false;

//////////////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////

#define INPUTBUFFERLEN 7
char inputBuffer[INPUTBUFFERLEN];  
String inputString = "";                       // a string to hold incoming data
volatile boolean stringComplete = false;       // whether the serial string is complete

int typeState;         // Hold variable that identifies if it is a LED, HP, or Sequence command
int functionState;     // Holds variable that identifies the function within the type above
int optionState;       // Holds variable that identifies the option value for those commands that use it. (color, position, etc)
int optionState2;

typedef struct LEDCmdStruct 
{
   byte LEDFunction;
   byte LEDOption1;
   byte LEDOption2;
} LEDCmd;

LEDCmd LED_command[HPCOUNT];

typedef struct HPCmdStruct 
{
   byte HPFunction;
   byte HPOption1;
} HPCmd;

HPCmd HP_command[HPCOUNT];


//////////////////////////////////////////////////////////////////////
///*****                  Color Values & Labels               *****///
//////////////////////////////////////////////////////////////////////
   
#define C_OFF      0x000000
#define C_RED      0xFF0000
#define C_ORANGE   0xFF8000
#define C_YELLOW   0xFFFF00
#define C_GREEN    0x00FF00
#define C_CYAN     0x00FFFF
#define C_BLUE     0x0000FF
#define C_MAGENTA  0xFF00FF
#define C_PURPLE   0x800080
#define C_WHITE    0xFFFFFF

//
// The colors in the table are repeated several time to give them a
// frequency of being shown. Same is true for the OFF state.
// This is done to give the HPLeds a more realistic light show.
// 
const uint32_t basicColors[10][10] = {{     C_RED,  C_YELLOW,   C_GREEN,   C_CYAN,   C_BLUE, C_MAGENTA, C_PURPLE, C_WHITE, C_OFF, C_OFF},    // Random
                                      {     C_RED,     C_RED,     C_RED,  C_WHITE, 0xFFA0A0,  0xFD5555, 0xFFD3D3,   C_OFF, C_OFF, C_OFF},    // Red
                                      {  C_YELLOW,  C_YELLOW,  C_YELLOW,  C_WHITE, 0xFDFD43,  0xFFFF82, 0xFFFFBA,   C_OFF, C_OFF, C_OFF},    // Yellow
                                      {   C_GREEN,   C_GREEN,   C_GREEN,  C_WHITE, 0x57FC57,  0x80FC80, 0xBDFFB1,   C_OFF, C_OFF, C_OFF},    // Green
                                      {    C_CYAN,    C_CYAN,    C_CYAN,  C_WHITE, 0x38FFFF,  0x71FDFD, 0xA4FDFD,   C_OFF, C_OFF, C_OFF},    // Cyan
                                      {    C_BLUE,    C_BLUE,    C_BLUE,  C_WHITE, 0xACACFF,  0x7676FF, 0x5A5AFF,   C_OFF, C_OFF, C_OFF},    // Blue
                                      { C_MAGENTA, C_MAGENTA, C_MAGENTA,  C_WHITE, 0xFB3BFB,  0xFD75FD, 0xFD9EFD,   C_OFF, C_OFF, C_OFF},    // Megenta
                                      {  C_ORANGE,  C_ORANGE,  C_ORANGE,  C_WHITE, 0xFB9B3A,  0xFFBE7D, 0xFCD2A7,   C_OFF, C_OFF, C_OFF},    // Orange
                                      {  C_PURPLE,  C_PURPLE,  C_PURPLE,  C_WHITE, 0xA131A1,  0x9B449B, 0xBD5FBD,   C_OFF, C_OFF, C_OFF},    // Purple
                                      {   C_WHITE,   C_WHITE,   C_WHITE,  C_WHITE, 0xB7B6B6,  0x858484, 0xA09F9F,   C_OFF, C_OFF, C_OFF}};   // White



//////////////////////////////////////////////////////////////////////
///*****               Initialize NeoPixel Strips             *****///
///*****               code suggested by LostRebel            *****///
//////////////////////////////////////////////////////////////////////
#define NEO_JEWEL_LEDS 7

#ifdef NEO_JEWEL_RGBW
 #define HP_NEO_TYPE (NEO_GRBW + NEO_KHZ800)
#else
 #define HP_NEO_TYPE (NEO_GRB + NEO_KHZ800)
#endif

Adafruit_NeoPixel neoStrips[HPCOUNT];


//////////////////////////////////////////////////////////////////////
///*****                 Initialize Servo Board               *****///
//////////////////////////////////////////////////////////////////////
Servos servos(SERVOI2CADDRESS);

#define STATUSLEDMSECS 200                 // Amount of time to flash the led pin
#define STATUSLEDPIN 13                    // The builtin LED on the Arduino mini is on pin 13

#define TESTMODEMSECS 10000                // 10 secs each before moving on
unsigned long testModeTimer = 0;      
boolean       testMode = false;
int           testModeIndex;  // index into testModeTests array

#define TESTMODENUMTESTS 10
const char * const testModeTests[TESTMODENUMTESTS] = {"A001",
                                                      "A0022",
                                                      "A0031",
                                                      "A0043",
                                                      "A005",
                                                      "A0066",
                                                      "A007",
                                                      "A008",  // Bogus command. Make sure it doesn't crash
                                                      "A009",  // Bogus command.
                                                      "END"};  // Bogus command.

  


////////////////////////////////////////////////////////////
/// Setup and initialization                             ///
////////////////////////////////////////////////////////////

void setup() { 
  randomSeed(analogRead(0));               // Seeds psuedo-random number generator with current value on unconnected anolog pin to make random more randomy.
 
  Serial.begin(9600);                      // Starts Serial with a baudrate of 9600
  inputString.reserve(10);                 // reserve 10 bytes for the inputString
 
  Wire.begin(I2CADDRESS);                  // Connects to I2C Bus and establishes address.
  Wire.onReceive(i2cEvent);                // Register event so when we receive something we jump to i2cEvent();


  //////////////////////////////////////////////////////
  ///                Status LED Setup                ///
  //////////////////////////////////////////////////////
  pinMode(STATUSLEDPIN, OUTPUT);
  
  //////////////////////////////////////////////////////
  ///                  RC Input Setup                ///
  //////////////////////////////////////////////////////
  pinMode(RCPIN, INPUT);

  //////////////////////////////////////////////////////
  ///                     OE Setup                   ///
  //////////////////////////////////////////////////////
  if(OUTPUT_ENABLED_ON) {pinMode(OUTPUT_ENABLED_PIN, OUTPUT);}

  //////////////////////////////////////////////////////
  ///                 HP Servo Setup                 ///
  //////////////////////////////////////////////////////
  for (int i=0; i<HPCOUNT; i++)
    positionHP(i,1,300);             // Centers Each HP

  //////////////////////////////////////////////////////
  ///                  HP LED Setup                  ///
  //////////////////////////////////////////////////////    
  for (int i=0; i<HPCOUNT; i++)
  {
    neoStrips[i] = Adafruit_NeoPixel(7, LEDpins[i], HP_NEO_TYPE);
    neoStrips[i].begin(); 
    neoStrips[i].setBrightness(BRIGHT);
    neoStrips[i].show();
    twitchLEDRunTime[i] = (1000*random(LEDTwitchRunInterval[i][0],LEDTwitchRunInterval[i][1]));  // Randomly sets initial LED Twitch Run Time value
  }

  Serial.print(F("\nFlthyHPs, Sketch Version 1.")); Serial.println(VERSION);
  Serial.print(F("Basic HP Positioning "));
  if(ENABLEBASICHP)
    Serial.println(F("Enabled\n"));
  else
    Serial.println(F("Disabled\n"));

}

//////////////////////////////////////////////////////////////////////
///                        Main Loop                               ///
//////////////////////////////////////////////////////////////////////

void loop(){
  int commandLength;
  unsigned long currTime;

  Servos::move(millis());  // this performs the actual moves

  currTime = millis();
  statusLEDCheck();        // see if the status led should be turned off

  // Test/Demo mode works by injecting commands every n seconds to simulate
  // receiving actual commands. When a real command is received, it cancels 
  // Test/Demo mode.
  //
  // So each time through this loop, check if we are in Demo mode and check
  // if we should move to the next command.

  testModeCheck();
  
  if(stringComplete) {
    commandLength = inputString.length();
    if (commandLength > INPUTBUFFERLEN) // Overflow? Truncate/Ignore
      commandLength = INPUTBUFFERLEN;
    
    inputString.toCharArray(inputBuffer, INPUTBUFFERLEN);
    inputString="";       // If a complete Command string has been flagged, write the string to array to process.

    if( inputBuffer[0]=='F' ||      // Front HP
        inputBuffer[0]=='R' ||      // Rear HP
        inputBuffer[0]=='T' ||      // Top HP
        inputBuffer[0]=='A' )       // All three HP
    {  
      if(commandLength >= 2)
        typeState = inputBuffer[1]-'0';
      if(commandLength >= 4)
        functionState = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');

      if(commandLength >= 5)
      {
        optionState = inputBuffer[4]-'0';
      }
      else if(typeState == 0)
      {
        if(functionState==5)
          optionState = SHORTCOLOR;
        else
          optionState = DEFAULTCOLOR;
      }
      else if(typeState == 1)
        optionState = 1;         // Set Center as the fallback position
      
      if(commandLength >= 6) {optionState2 = inputBuffer[5]-'0';}

      if(inputBuffer[0]=='F' || inputBuffer[0]=='A')
        processCommands(0);
      if(inputBuffer[0]=='R' || inputBuffer[0]=='A')
        processCommands(1);
      if(inputBuffer[0]=='T' || inputBuffer[0]=='A')
        processCommands(2);
    }
    else if (inputBuffer[0]=='S')                      // Major Sequences (Both LEDS and HP Servos)
    {
      if(commandLength >= 2)
      {
        functionState = (inputBuffer[1]-'0');

        switch (functionState)
        { 
               case 1: varResets(0); varResets(1); varResets(2);                        // Resets the Variables for LEDs on all 3 HPs
                       for (int i=0; i<HPCOUNT; i++) {enableTwitchHP[i]=false;}         // Disables Auto HP Twith on all HPs           
                       positionHP(0,0,SERVO_SPEED[0]);                                  // Moves Front HP to Down Position
                       ledOFF(1);ledOFF(2);                                             // Turns Off LEDs on Rear and Top HPs
                       flushCommandArray(1, 0);flushCommandArray(2, 0);                 // Flushes LED Command Array for Rear and Top HP
                       LED_command[0].LEDFunction = 1;                                  // Set Leia LED Sequence to run each loop.
                       break;
               case 8: for (int i=0; i<HPCOUNT; i++) {
                                enableTwitchHP[i]=false;                                 // Disables Auto HP Twith on all HPs 
                                flushCommandArray(i, 0);                                 // Flushes LED Command Array for all HPs
                                flushCommandArray(i, 1);                                 // Flushes HP Command Array for all HPs
                                ledOFF(i);                                               // Turns Off LEDs on all HPs
                                enableTwitchLED[i]=false;                                // Disables Auto LED Twith on all HPs
                       }
                       break;
               case 9: for (int i=0; i<HPCOUNT; i++) {
                                enableTwitchHP[i]=true;                                  // Enables Auto HP Twith on all HPs 
                                flushCommandArray(i, 0);                                 // Flushes LED Command Array for all HPs
                                flushCommandArray(i, 1);                                 // Flushes HP Command Array for all HPs
                                ledOFF(i);                                               // Turns Off LEDs on all HPs
                                enableTwitchLED[i]=true;                                 // Enables Auto LED Twith on all HPs
                       }
                       break;
        }
      }
    }

    ///***  Clear States and Reset for next command.  ***///
    inputString = "";
    stringComplete = false; 
    memset(inputBuffer,0,sizeof(inputBuffer));
          
  }

  for(int i=0; i<HPCOUNT; i++) {
    switch (LED_command[i].LEDFunction) {
      case 1: colorProjectorLED(i,DEFAULTCOLOR); break;       // Leia Sequence
      case 2: colorProjectorLED(i,LED_command[i].LEDOption1); break; 
      case 3: dimPulse(i, LED_command[i].LEDOption1, LED_command[i].LEDOption2); break;
      case 4: cycle(i,LED_command[i].LEDOption1); break;
      case 5: ShortCircuit(i,LED_command[i].LEDOption1); break;
      case 6: ledColor(i,LED_command[i].LEDOption1);flushCommandArray(i,0); break;
      case 7: rainbow(i); break;  
      case 98: enableTwitchLED[i]=false; resetLEDtwitch(i); flushCommandArray(i,0); break;   // Clear Function, Disable Random LED Twitch
      case 99: enableTwitchLED[i]=true; resetLEDtwitch(i); flushCommandArray(i,0); break;   // Clear Function, Enable Random LED Twitch
      default: break;
    }
  
    switch (HP_command[i].HPFunction) {
      case 1: positionHP(i,HP_command[i].HPOption1,SERVO_SPEED[0]); flushCommandArray(i,1); break;  
      case 2: RCHP(i,1); break;
      case 3: RCHP(i,2); break;
      case 4: twitchHP(i); flushCommandArray(i,1); break;
      case 5: wagHP(i,1); break;  // Wags HP Left/Right   
      case 6: wagHP(i,2); break;  // Wags HP Up/Down  
      case 98: enableTwitchHP[i]=false; flushCommandArray(i,1); break;   // Clear Function, Disable Servo Random Twitch
      case 99: enableTwitchHP[i]=true; flushCommandArray(i,1); break;   // Clear Function, Enable Servo Random Twitch
      default: break;
    } 
  }     
  
  for (int i=0; i<HPCOUNT; i++)
    if (currTime > twitchLEDTime[i] && enableTwitchLED[i])
      LEDtwitch(i, DEFAULTCOLOR);

  for (int i=0; i<HPCOUNT; i++)
    if (currTime > twitchHPTime[i] && enableTwitchHP[i])
    {
      twitchHP(i);
      twitchHPTime[i] = (1000*random(HPTwitchInterval[i][0],HPTwitchInterval[i][1])) + millis();
    } 

  // If OutputEnabled and the time since last move is greater that the speed allowance,
  // then turn off the servos to keep them from humming.
  if(OEFlag &&
     (millis() - OETime) >= SERVO_SPEED[1]) {            
              digitalWrite(OUTPUT_ENABLED_PIN, HIGH);       
              OEFlag = false;                               
  }
              
}


////////////////////////////////////////////////////////////
// This needs an explanation                             /// 
////////////////////////////////////////////////////////////
int mapPulselength(int microseconds) {
  int y = map(microseconds, 1000, 2000, 200, 550);
  return y;
}


//////////////////////////////////////////////////////////////////////////////////////////
///*****                       OE Servo Enable Function                           *****///
//////////////////////////////////////////////////////////////////////////////////////////

void enableServos() {
  if(OUTPUT_ENABLED_ON) {                        // If OE control of the Servo Break Out Board is enabled
    digitalWrite(OUTPUT_ENABLED_PIN, LOW);       // Set OE Pin to low to enable Servo Movement 
    OEFlag = true;                               // Set OE Flag so loop knows servos are enabled
    OETime = millis();                           // Set OE Time to current millis so we can eventually run disableServos after movement is complete 
  }
}

 
//////////////////////////////////////////////////////////////////////////////////////////
///*****                     HP Twitch/Movement Functions                         *****///
//////////////////////////////////////////////////////////////////////////////////////////

void positionHP(byte hp, byte pos, int speed) {
  int pos1;
  int pos2; 
  if(ENABLEBASICHP && pos == 1) {
    pos1 = mapPulselength(HPposBasic[0][0]);
    pos2 = mapPulselength(HPposBasic[1][0]);
  }
  else if (!ENABLEBASICHP) {
    pos1 = mapPulselength(HPpos[hp][pos][0]);
    pos2 = mapPulselength(HPpos[hp][pos][1]);
  }
  if(!ENABLEBASICHP || (ENABLEBASICHP && pos == 1)) {     
    enableServos();
    servos.moveTo(HPpins[hp][0],speed,pos1);
    servos.moveTo(HPpins[hp][1],speed,pos2);
  }
}

void twitchHP(byte hp) {
  int speed = random(SERVO_SPEED[0],SERVO_SPEED[1]);
  enableServos();   
  if(ENABLEBASICHP) {
    servos.moveTo(HPpins[hp][0],speed,mapPulselength(random(HPposBasic[0][1],HPposBasic[0][2])));
    servos.moveTo(HPpins[hp][1],speed,mapPulselength(random(HPposBasic[1][1],HPposBasic[1][2])));
  }
  else {positionHP(hp,random(0,9),speed);}
}

void RCHP(byte hp, byte type) {     
  int servo1;
  int servo2;
  if(!ENABLEBASICHP) {
    RCinputCh1 = pulseIn(RCPIN, HIGH); // each channel
    //Serial.println(RCinputCh1);
    if(RCinputCh1>0) {
      enableServos();
      if(type==1) {                                                                                  // RC Controls Left-Right Movement
        servo1 = map(RCinputCh1, RCRange[0], RCRange[1], HPpos[hp][3][0], HPpos[hp][6][0]);
        servo2 = map(RCinputCh1, RCRange[0], RCRange[1], HPpos[hp][3][1], HPpos[hp][6][1]);
      }
      else if(type==2) {                                                                           // RC Controls Up-Down Movement
        servo1 = map(RCinputCh1, RCRange[0], RCRange[1], HPpos[hp][0][0], HPpos[hp][2][0]);
        servo2 = map(RCinputCh1, RCRange[0], RCRange[1], HPpos[hp][0][1], HPpos[hp][2][1]);
      }
      servos.moveTo(HPpins[hp][0],0,mapPulselength(servo1));servos.moveTo(HPpins[hp][1],0,mapPulselength(servo2));
    }
    else { Serial.println(F("No RC Input Detected!")); }
  }      
}

void wagHP(byte hp, byte type){
  int speed = 250;
  if(!ENABLEBASICHP) {
    if(WagCount[hp] < 0) {WagCount[hp] = 0;WagTimer[hp]=millis();}                  
    if(millis()-WagTimer[hp]>=400) {
      WagCount[hp]++;
      WagTimer[hp]=millis();          
      if (WagCount[hp] % 2) {
        if(type==1) {positionHP(hp, 3, speed);}    //  On Odd Count, Wag HP Left
        if(type==2) {positionHP(hp, 0, speed);}    //  On Odd Count, Wag HP Down
      }
      else {
        if(type==1) {positionHP(hp, 6, speed);}    //  On Even Count, Wag HP Right  
        if(type==2) {positionHP(hp, 2, speed);}    //  On Even Count, Wag HP Up 
      } 
    }
    if(WagCount[hp]>=(WAGCOUNT*2)) {
      WagCount[hp] = -1;
      flushCommandArray(hp,1);
      positionHP(hp, 1, speed);  // Center
    }  
  }
}
    
//////////////////////////////////////////////////////////////////////////////////////////
///*****                            HP LED Functions                              *****///
//////////////////////////////////////////////////////////////////////////////////////////

void ledOFF(byte hp) {
  for(int i=0; i<NEO_JEWEL_LEDS; i++) {neoStrips[hp].setPixelColor(i,C_OFF);} 
  neoStrips[hp].show();
}

      
void ledColor(byte hp, int c) {
  for(int i=0; i<NEO_JEWEL_LEDS; i++) {neoStrips[hp].setPixelColor(i,basicColors[c][0]);} 
  neoStrips[hp].show();
}
       
 
void LEDtwitch(int hp, int c) { 
  if((millis()-twitchLEDTime[hp]) < twitchLEDRunTime[hp]) {colorProjectorLED(hp, c);} 
  else { resetLEDtwitch(hp); }             
}

void resetLEDtwitch(int hp) {
  ledOFF(hp);
  twitchLEDTime[hp] = (1000*random(LEDTwitchInterval[hp][0],LEDTwitchInterval[hp][1]))+millis();
  twitchLEDRunTime[hp] = (1000*random(LEDTwitchRunInterval[hp][0],LEDTwitchRunInterval[hp][1])); 
}

//////////////////////////////////////////////////////////
///*****        Color Projector Function          *****///
//////////////////////////////////////////////////////////
void colorProjectorLED(byte hp, int c) {
  if ((millis() - tCounter[hp]) > interval[hp]) {
    for(int i=0; i<NEO_JEWEL_LEDS; i++) { neoStrips[hp].setPixelColor(i,basicColors[c][random(0,10)]); }
    neoStrips[hp].show();
    tCounter[hp]=millis();
    interval[hp]=random(50,150);
  }
}
        
/////////////////////////////////////////////////////////
///*****           Dim Pulse Function            *****///
/////////////////////////////////////////////////////////
        
void dimPulse(byte hp, int c, int setting) {
  int inter = map(setting, 0, 9, dimPulseSpeedRange[1], dimPulseSpeedRange[0]);
  long elapsed;
  int frames;
  if ((millis() - tCounter[hp]) > interval[hp]) {
    elapsed = millis() - tCounter[hp];
    frames = elapsed/inter;
    if(frames >= 64) {tCounter[hp] = millis();}     
    if (frames > 32) {frames = 32-(frames - 32);}
    if(elapsed>=inter) {
      for(int i=0; i<NEO_JEWEL_LEDS; i++)
        neoStrips[hp].setPixelColor(i, dimColorVal(c,(frames*8)));
      neoStrips[hp].show();
    }
  }
}

void ShortCircuit(byte hp, int c) {
  if(SCloop[hp] <= SCMAXLOOPS) {
    if ((millis() - tCounter[hp]) > SCinterval[hp]) {
      if(SCflag[hp] == false) {
        for(int i=0; i<NEO_JEWEL_LEDS; i++)
          neoStrips[hp].setPixelColor(i,C_OFF);
        SCflag[hp] = true;
        SCinterval[hp] = 10+(SCloop[hp]*random(15,25));
      } 
      else {
        for(int i=0; i<NEO_JEWEL_LEDS; i++)
          neoStrips[hp].setPixelColor(i,basicColors[c][random(0,10)]);
        neoStrips[hp].show();
        SCflag[hp] = false;
        SCloop[hp]++;
      }
      tCounter[hp] = millis();
      neoStrips[hp].show();
    }  
  }   
}

            
/////////////////////////////////////////////////////////
///*****           Cycle Function            *****///
/////////////////////////////////////////////////////////

void cycle(byte hp, int c) { 
  int inter = 75;
  if ((millis() - tCounter[hp]) > inter) {
    tCounter[hp]=millis();
    neoStrips[hp].setPixelColor(0, C_OFF);     // Center LED is always off
    if (Frame[hp] >= NEO_JEWEL_LEDS) {Frame[hp]=0;}
    for (int i = 1; i<NEO_JEWEL_LEDS; i++) { 
      if (i == Frame[hp]) {neoStrips[hp].setPixelColor(i, basicColors[c][0]);}
      else {neoStrips[hp].setPixelColor(i, C_OFF);}                      
    }
    neoStrips[hp].show(); 
    Frame[hp]++;
  }
}




///////////////////////////////////////////////////////
///*****           Rainbow Functions           *****///
///////////////////////////////////////////////////////
      
void rainbow(byte hp) {
  int inter = 10;
  long elapsed;
  byte frames;  
  elapsed = millis() - tCounter[hp];
  frames = elapsed/inter;
  if(frames > 256*5) {
    tCounter[hp]=millis();
  }
  else {        
    for(int i=0; i<NEO_JEWEL_LEDS; i++) {
      neoStrips[hp].setPixelColor(i, Wheel(((i * 256 / NEO_JEWEL_LEDS) + frames) & 255));
    }
    if(elapsed>=inter) {neoStrips[hp].show();}
  }              
}


/////////////////////////////////////////////////////////
///*****             Wheel Function              *****///
/////////////////////////////////////////////////////////
/// Input a value 0 to 255 to get a color value. The  ///   
/// colours are a transition r - g - b - back to r.   ///
/////////////////////////////////////////////////////////
      
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else {
    WheelPos -= 170;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
} 
      
/////////////////////////////////////////////////////////
///*****          Dim Color Function 1           *****///
/////////////////////////////////////////////////////////
/// Input a value 0 to 255 to get a color value at a  ///   
/// specific brightness.                              ///
/// Takes and int color value and returns an          ///
/// uint32_tcolor value.                              ///
/// Used for soft fade in/outs.                       ///
/////////////////////////////////////////////////////////  
      
uint32_t dimColorVal(int c, int brightness) {
  switch (c) {                                                
  case 1: return Color(255/brightness, 0, 0); break;
  case 2: return Color(255/brightness, 255/brightness, 0);break;
  case 3: return Color(0, 255/brightness, 0); break;
  case 4: return Color(0, 255/brightness, 255/brightness);break;
  case 5: return Color(0, 0, 255/brightness); break;
  case 6: return Color(255/brightness, 0, 255/brightness); break;
  case 7: return Color(255/brightness, 180/brightness, 0); break;
  case 8: return Color(255/brightness, 255/brightness, 255/brightness); break;
  case 9: return C_OFF; break;
  default: return C_OFF; break;                                          
  }   
}  

/////////////////////////////////////////////////////////
///*****    RGB to Hex Color Value Converter      *****///
/////////////////////////////////////////////////////////
///    Converts and returns a color's 3 seperat RGB   /// 
///       value as an uint32_t value.                 ///   
/////////////////////////////////////////////////////////        

uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
} 


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                                    Constructs Command Arrays                                  /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////// 
void processCommands(byte hp) {
  flushCommandArray(hp, typeState);          // Flushes Command Array
  if (typeState == 0) {
    LED_command[hp].LEDFunction = functionState;
    if(optionState==0)
      LED_command[hp].LEDOption1=random(0,9);
    else
      LED_command[hp].LEDOption1 = optionState;
    if(optionState2 && functionState == 3)
      LED_command[hp].LEDOption2 = optionState2;
    else if (!optionState2 && functionState == 3)
      LED_command[hp].LEDOption2 = DIMPULSESPEED;
    varResets(hp);
  }
  else if (typeState == 1) {
    HP_command[hp].HPFunction = functionState;
    HP_command[hp].HPOption1 = optionState;
    enableTwitchHP[hp]=false;           // Toggle Auto HP Twitch off so it doesn't interupt sequence
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                   Consolidates Flushing Command Arrays into Single Function                   /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////// 
void flushCommandArray(byte hp, byte type) {
  if(type==0)
    LED_command[hp] = (LEDCmd){0};
  else if(type==1)
    HP_command[hp] = (HPCmd){0};
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                          Consolidates Variable Reset in Single Function                       /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////            
             
void varResets(byte hp) {
  Frame[hp]=0;
  SCflag[hp] = false;
  SCloop[hp] = 0;
  SCinterval[hp] = 10;
  ledOFF(hp);                            // Clears Any Remaining Lit LEDs 
  enableTwitchLED[hp]=false;             // Toggle Auto LED Twitch off so it doesn't interupt sequence
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                          Check for TEST mode and iterate through the tests                    /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void testModeCheck() {
  
  if(stringComplete)
  {
    if(inputString == "TEST") // Enter test mode?
    {
      Serial.println(F("Start TEST mode"));
      testMode=true;
      testModeIndex = 0;
      testModeTimer = millis() - (TESTMODEMSECS + 1); // Force test to start this cycle
    }
    else                      // Exit test mode due to any command override
    {
      testMode = false;
    }
  }
  
  if(testMode &&
     millis() - testModeTimer >= TESTMODEMSECS) // Inject a test command
  {
    if(testModeIndex > TESTMODENUMTESTS) // Finished all tests? Revert back to default mode and exit test mode
    {
      inputString = "S9";
      Serial.print(F("Exit TEST mode: ")); Serial.println(inputString);
      stringComplete = true;
      testMode = false;
    }
    else
    {
      inputString = testModeTests[testModeIndex];
      stringComplete = true;
      testModeTimer = millis();
      testModeIndex++;
      Serial.print(F("TEST: ")); Serial.println(inputString);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                            Onboard LED Communication Status Trigger                           /////
/////                                                                                               /////
///// statusLEDOn - Turn on the LED                                                                 /////
///// statusLEDCheck - Check if enough time has passed and we should turn off the LED               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////// 
void statusLEDOn() {
  digitalWrite(STATUSLEDPIN, HIGH);
  ledtimer = millis();
}

void statusLEDCheck() {
  if(millis() - ledtimer >= STATUSLEDMSECS)
    digitalWrite(13, LOW);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                             Serial & I2C Communication Functions                              /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////  
    
      
/////////////////////////////////////////////////////////
///*****          Serial Event Function          *****///
/////////////////////////////////////////////////////////
/// This routine is run between loop() runs, so using ///
/// delay inside loop can delay response.  Multiple   ///
/// bytes of data may be available.                   ///
/////////////////////////////////////////////////////////
      
void serialEvent() {
  while (Serial.available()) {                         // Loop through all incoming bytes
    char inChar = (char)Serial.read();                 // Receive each byte as a character        
    if (inChar == '\r' || inChar == '\n') {            // if the incoming character is a carriage return (\r) or newline (\n), it denotes end of command
      stringComplete = true;                           // Once done, set a flag so the main loop can do something about it. 
    } 
    else { 
      inputString += inChar;                           // Add each Character to inputString     
    }
    statusLEDOn();
  }
}
/////////////////////////////////////////////////////////
///*****            I2C Event Function           *****///
/////////////////////////////////////////////////////////
///  This routine is run when an onRecieve event is   ///
///     triggered.  Multiple bytes of data may be     ///
///                    available.                     ///
/////////////////////////////////////////////////////////            
      
void i2cEvent(int howMany)
{  
  inputString = "";                                     // Ensures inputString is empty
  while(Wire.available())  {                            // loop through all incoming bytes
    char inChar = (char)Wire.read();                    // Receive each byte as a character
    inputString += inChar;                              // Add each Character to inputString
  }
  stringComplete = true;                                // Once done, set a flag so the main loop can do something about it. 
  statusLEDOn();
}

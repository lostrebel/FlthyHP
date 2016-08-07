////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///                                    Flthy Holoprojectors v1.1                                     ///
///                                       by Ryan Sondgeroth                                         ///
///                                         aka FlthyMcNsty                                          ///
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
///                                                                                                  ///
///         Version 1.1: Adeded One Time HP Twitch Function                                          ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///     Commands and Structure                                                                       ///
///                                                                                                  ///.
///     DT##C                                                                                        ///
///     D - the HP designator with F-Front HP, R-Rear HP, T-Top HP, and A-All 3 HPs                  ///
///     T - the Sequence Type is either 0-Led Fuctions and 1-Servo Functions                         ///
///    ## - the Sequence Value including leading zero if necessary, ie sequence 3 is 03              ///
///     C - (Optional), the Color integer value from list below:                                     ///
///        Basic Color Integer Values                                                                ///
///           0 = Off (Black)                                                                        ///
///           1 = Red                                                                                ///
///           2 = Yellow                                                                             ///
///           3 = Green                                                                              ///
///           4 = Cyan (Aqua)                                                                        ///
///           5 = Blue                                                                               ///
///           6 = Magenta                                                                            ///
///           7 = Orange                                                                             ///
///           8 = Purple                                                                             ///
///           9 = White                                                                              ///
///                                                                                                  ///
///     D001   - Leia Sequence, Random shades of blue to mimic Leia Hologram                         ///
///     D002C  - Color Projector Sequence, Like Leia above but using color command value             ///
///     D003C  - Dim Pulse Sequence, Color slowly pulses on and off                                  ///
///     D004C  - Cycle Sequence, using color command value                                           ///
///     D005C  - Short Circuit, Led flashes on and off with interval slowing over time               ///
///     D006C  - Toggles Color, Simply sets LEDs tp solid color value.                               ///
///     D007   - Rainbow Sequence                                                                    ///
///     D098   - Clears LED, Disables Auto LED Sequence                                              ///
///     D099   - Clears LED, Enables Auto LED Sequence                                               ///
///                                                                                                  ///
///     D101   - Sends HP to the Center Position                                                     ///
///     D102   - Sends HP to the Down Position                                                       ///
///     D103   - Sends HP to the Up Position                                                         ///
///     D104   - Enables RC Control on HP                                                            ///
///     D198   - Disables Auto HP Twitch                                                             ///
///     D199   - Enables Auto HP Twitch                                                              ///
///                                                                                                  ///
///       S1   - Leia Mode (Front HP in Down Position, Leia LED Sequence, all other HPs disabled)    ///
///       S8   - Clear all LEDs, Disable Auto HP Twitch, Disable Auto LED Sequence                   ///
///       S9   - Clear all LEDs, Enable Auto HP Twitch, Enable Auto LED Sequence                     ///
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
/* PAA: Converted to define: reason - it never changes at runtime */
#define I2CADDRESS 0x19 


//////////////////////////////////////////////////////////////////////////////////
///*****                Assign Servo Board IC2 Address Below              *****///
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
/* PAA: Converted to define: reason - it never changes at runtime */
#define SERVOI2CADDRESS 0x40 

// For now we make a distinction between Holos, Leds and Servos.
// It's possible that somebody could have for example 3 Holos, 3 Leds,
// but only two servos but's that unlikely. So this logic could be collapsed
// into one triplet but for now we keep everything addressable.
enum HolosProjectors
{
  HPFront = 0,
  HPRear,
  HPTop,
  HPCount
};

enum HPLeds
{
  HPLedFront = 0,
  HPLedRear,
  HPLedTop,
  HPLedCount 
};

enum HPServos
{
  HPServoFront = 0,
  HPServoRear,
  HPServoTop,
  HPServoCount 
};

enum HPServoAxis
{
  HPAxisX = 0,
  HPAxisY,
  HPAxisCount
};

// 10 Digital pins for now
enum PinDigital
{
  PinDigital_0 = 0,
  PinDigital_1,
  PinDigital_2,  // Front LED
  PinDigital_3,  // Read  LED
  PinDigital_4,  // Top   LED
  PinDigital_5,
  PinDigital_6,
  PinDigital_7,
  PinDigital_8,
  PinDigital_9,  // RC Input Control
  PinDigital_Count
};


// 6 Servo pins for now
enum PinServo
{
  PinServo_0 = 0,  // Front X servo
  PinServo_1,      // Front Y servo
  PinServo_2,      // Rear X servo
  PinServo_3,      // Rear Y servo
  PinServo_4,      // Top X servo
  PinServo_5,      // Top Y servo
  PinServo_Count
};

/* Each holo can have an LED board and 2 servos (x,y) *.

//////////////////////////////////////////////////////////////////////////////////
///*****               Arduino Digital Pin Assignments for LEDs           *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////

int LEDpins[HPLedCount] = {PinDigital_2,    //Front
                           PinDigital_3,    //Rear
                           PinDigital_4};   //  Top

//////////////////////////////////////////////////////////////////////////////////////////
///*****                         Servo Board Pin Assignments                      *****///
///*****                                                                          *****///
///*****  HPpins[3][2] = {{Front 1, Front2}, {Rear 1, Rear 2}, {Top 1, Top 2}};   *****///
///*****                                                                          *****///
//////////////////////////////////////////////////////////////////////////////////////////    

int HPpins[HPServoCount][HPAxisCount] = {{PinServo_0, PinServo_1},            // Front HP Pins
                                         {PinServo_2, PinServo_3},            // Rear HP Pins
                                         {PinServo_4, PinServo_5}};           // Top HP Pins


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
#define OUTPUT_ENABLED_ON  true

//////////////////////////////////////////////////////////////////////////////////
///*****                                                                  *****///
///*****            Arduino Digital Pin Assignment for RC Input           *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////
#define RC_PIN PinDigital_9

//////////////////////////////////////////////////////////////////////////////////
///*****             Pulse Width Range of RC Stick/Controller             *****///
///*****               assigned to control HP movement.                   *****///
//////////////////////////////////////////////////////////////////////////////////
#define RC_PULSE_MIN 1250
#define RC_PULSE_MAX 1750
enum RCRangeBounds
{
  RCRangeMin = 0,
  RCRangeMax,
  RCRangeCount
};
int RCRange[RCRangeCount] = {RC_PULSE_MIN, RC_PULSE_MAX};     // {Min, Max}
    

//////////////////////////////////////////////////////////////////////////////////
///*****                          LED Brightness                          *****///
///*****                                                                  *****///
///*****                 Adjust LED Brightness Level (0-255)              *****///
//////////////////////////////////////////////////////////////////////////////////
#define BRIGHT 20  // Set LED Brightness Level (0-255)

//////////////////////////////////////////////////////////////////////////////////
///*****                Enable/Disable LED & Servo Auto Twitch            *****///
///*****                                                                  *****///
///*****                      Enabled = true, Disabled = false            *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////

boolean enableTwitchLED[HPLedCount]  = {true,true,true}; /* Leds:   Front, Rear, Top */
boolean enableTwitchHP[HPServoCount] = {true,true,true};/* Servos: Front, Rear, Top */

///////////////////////////////////////////////////////////////////////////////////
///*****               LED & Servo Auto Twitch Interval Ranges             *****///
///*****                                                                   *****///
///*****      Use these values to select the range (min, max) in           *****///
///*****     seconds to randomly select the next auto twitch interval.     *****///
///*****      This helps give the illusion these displays & movements      *****///
///*****                      are totally autonomous.                      *****///
///*****                                                                   *****///
///*****           **TwitchInterval[2]  = {Min Value, Max Value}           *****///
///*****                                                                   *****///
///////////////////////////////////////////////////////////////////////////////////  

enum TwitchRangeBounds
{
  TwitchRangeMin = 0,
  TwitchRangeMax,
  TwitchRangeCount
};

const unsigned int LEDTwitchInterval[HPLedCount][TwitchRangeCount] = {{240,360},  // (4mins-6mins) Enter min and max seconds Front HP Led Twitches  
                                                                      {300,420},  // (5mins-7mins) Enter min and max seconds Rear HP Led Twitches  
                                                                      {300,420}}; // (5mins-7mins) Enter min and max seconds Top HP Led Twitches  

const unsigned int HPTwitchInterval[HPServoCount][TwitchRangeCount] = {{45,120},  // (45s-2mins) Enter min and max seconds Front HP Servo Twitches  
                                                                       {60,180},  // (1min-3mins) Enter min and max seconds Rear HP Servo Twitches  
                                                                       {60,180}}; // (1min-3mins) Enter min and max seconds Top HP Servo Twitches  

///////////////////////////////////////////////////////////////////////////////////
///*****                  LED Auto Twitch Run Time Ranges                  *****///
///*****                                                                   *****///
///*****    Use these values to select the range (min, max) in seconds     *****///
///*****   to randomly select how long the sequence runs each time the     *****///
///*****                      LED twitch is executed.                      *****///
///*****                                                                   *****///
///*****      *LEDTwitchRunInterval[2]  = {Min Value, Max Value}          *****///
///*****                                                                   *****///
///////////////////////////////////////////////////////////////////////////////////

const unsigned int LEDTwitchRunInterval[HPServoCount][TwitchRangeCount] = {{5,25},   // (5-25s) Front LED Runtime
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

enum ServoSpeedDelayBounds
{
  ServoDelayMin = 0,
  ServoDelayMax,
  ServoDelayCount
};

const int SERVO_SPEED[ServoDelayCount] = {150, 400};


//////////////////////////////////////////////////////////////////////////////////
///*****                Preset HP Servo Position Coordinates              *****///
///*****                                                                  *****///
///*****   Since each Builder's droid is assembled differently, we found  *****///
///*****   using position coordinates generated randomly on the fly was   *****///
///*****   problematic due to the occasional servo movement attempting   *****///
///*****   to travel beyond the outer cowl path.  Therefore we use a set  *****///
///*****   of 6 position pairs for each HP, set to ensure they lie with   *****///
///*****    in a freely accessible position. These coordinates consist    *****///
///*****     of the servo position of each servo using its pulse width    *****///
///*****    value in microseconds. I suggest using a servo tester with    *****///
///*****    a digital display to find these position pairs while the HPs  *****///
///*****                      are mounted in the dome.                    *****///
///*****                                                                  *****///
///*****     The first coordinate pair in each row references the DOWN    *****/// 
///*****    position of the HP, the second pair is the CENTER position,   *****/// 
///*****   and the 3rd pair the UP position.  The three remaining pairs   *****/// 
///*****    are auxiliary positions picked from a position free from      *****/// 
///*****                     obstructed servo travel.                     *****/// 
///*****                                                                  *****///
///*****     Random twitch movement is then chosen to randomly travel     *****/// 
///*****     between these 6 safe points minimizing the chance of the     *****/// 
///*****            movement being impeded causing excessive and          *****/// 
///*****                     unpleasant servo noise                       *****/// 
///*****                                                                  *****///
///*****          But if you would rather do it the simple way, set       *****///
///*****    ENABLE_BASIC_HP to true and it will use the Center, Min, Max  *****///
///*****    values set in the HPposBasic array. These are based on the    *****///
///*****      assumed default center value of a normal servo so it is     *****///
///*****    important that your servo linkage is installed in a manner    *****///
///*****       where the center servo position corresponds with the       *****///
///*****       orientation you want your HP to be in when centered.       *****///
///*****                                                                  *****///
///*****          RC mode requires position coordinates to work well,     *****///
///*****                so you must have enableBasicHP set to 0.          *****///
///*****                                                                  *****///
///*****     IT IS IMPORTANT TO NOTE...if you disassemble your HPs and    *****///
///*****     servo linkage after setting the following values, you will   *****///
///*****    most likely need to recheck and reset them as the chances of  *****///
///*****       reinstalling them exactly the same way is approximately    *****///
///*****                             3,720 to 1.                          *****///
///*****                                                                  *****///
//////////////////////////////////////////////////////////////////////////////////     

enum HPPositionVector
{
  HPPosVecDown = 0,
  HPPosVecCenter,
  HPPosVecUp,
  HPPosVecAux1,
  HPPosVecAux2,
  HPPosVecAux3,
  HPPosVecCount
};


// Caution: These values must be tuned to your specific droid!
// If you don't want to do that, set ENABLE_BASIC_HP to true
// and generic and presumed safe values will be used.
const int HPpos[HPCount][HPPosVecCount][HPAxisCount] =
                         {{{1492,1964},   // Front HP Down
                           {1492,1565},   // Front HP Center
                           {1492,1218},   // Front HP Up
                           {1281,1443},   // Front HP Aux1
                           {1692,1757},   // Front HP Aux2
                           {1255,1940}},  // Front HP Aux3
                          {{1601, 1084},  // Rear HP Down
                           {1437,1537},   // Rear HP Center
                           {1385,1954},   // Rear HP Up
                           {1343,1406},   // Rear Aux1
                           {1633,1764},   // Rear Aux2
                           {1803,1283}},  // Rear Aux3
                          {{1547,1038},   // Top HP Down
                           {1431,1395},   // Top HP Center
                           {1473,1793},   // Top HP Up
                           {1633,1737},   // Top Aux1
                           {1324,1200},   // Top Aux2
                           {1677,1151}}}; // Top Aux3


#define ENABLE_BASIC_HP true

enum HPPosBasicVector
{
  HPPosBasicVecCenter=0,
  HPPosBasicVecMin,
  HPPosBasicVecMax,
  HPPosBasicVecCount
};
// These values are presumed to be safe enough not to drive your servo
// to extremes or hit any obstructions
int HPposBasic[HPPosBasicVecCount][HPAxisCount] = {{1500,1500},  // Center X,Y
                                                   {1300,1300},  // Min X,Y
                                                   {1700,1700}}; // Max X,Y 


///////////////////////////////////////////////////////////////////////////////////
///*****                      Default Color Settings                       *****///
///*****                                                                   *****///
///*****     Select Default Colors for certain LED sequences using the     *****///
///*****                       integer values below:                       *****///
///*****                                                                   *****///
///*****             Basic Color Integer Values                            *****///
///*****               0 = Off (Black}                                     *****///
///*****               1 = Red                                             *****///
///*****               2 = Yellow                                          *****///
///*****               3 = Green                                           *****///
///*****               4 = Cyan (Aqua)                                     *****///
///*****               5 = Blue                                            *****///
///*****               6 = Magenta                                         *****///
///*****               7 = Orange                                          *****///
///*****               8 = Purple                                          *****///
///*****               9 = White                                           *****///
///*****                                                                   *****///
///////////////////////////////////////////////////////////////////////////////////  

enum ColorNames
{
  ColorOffBlack = 0,
  ColorRed,
  ColorYellow,
  ColorGreen,
  ColorCyan,
  ColorBlue,
  ColorMagenta,
  ColorOrange,
  ColorPurple,
  ColorWhite,
  ColorCount
};

#define COLOR_DEFAULT  ColorBlue       // Blue, Color integer value for the hue of default sequence.
                                       // This is to appease MKelly and MKarp's whiney little asses, lol
#define COLOR_SHORT    ColorOrange     // Orange, Color integer value for the hue of ShortCircuit Message.  


////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///                                  End User Adjustable Settings                                    ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
///*****  Sequence/Function Varaiables, Containers & Counters *****///
//////////////////////////////////////////////////////////////////////

unsigned long tCounter[HPCount] = {0, 0, 0};
unsigned long interval[HPCount] = {100, 100, 100};
unsigned long SCinterval[HPCount] = {10, 10, 10};

int SCloop[HPCount] = {0, 0, 0};
int SCflag[HPCount] = {0, 0, 0};
byte Frame[HPCount] = {0, 0, 0};

/* TODO: AFAIK This is only referenced by RCHP so move it there instead of global */
int RCinputCh1;

long twitchLEDTime[HPLedCount]  = {4000, 4000, 4000}; //LEDS Start up 4 seconds after boot
long twitchHPTime[HPServoCount] = {4000, 4000, 4000}; //HPs Start 4 seconds after boot
long twitchLEDRunTime[HPLedCount];

long int OECounter = 0;
boolean OEFlag = false;

//////////////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
enum LEDCommands
{
  LEDCmdLeia=1,
  LEDCmdColorProj,
  LEDCmdDimPulse,
  LEDCmdCycle,
  LEDCmdShortCircuit,
  LEDCmdLedColor,
  LEDCmdRainbow,
  LEDCmdAutoDisable = 98,
  LEDCmdAutoEnable = 99
};

enum ServoCommands
{
  ServoCmdCenter = 1,
  ServoCmdDown,
  ServoCmdUp,
  ServoCmdRCEnable,
  ServoCmdAutoDisable = 98,
  ServoCmdAutoEnable = 99
};

int commandInt;
char inputBuffer[5];  
String inputString = "";         // a string to hold incoming data
volatile boolean stringComplete  = false;      // whether the serial string is complete

int typeState;
int functionState;
int colorState;



byte HP_command[HPServoCount][4] = {{0,0,0,0},
                                    {0,0,0,0},
                                    {0,0,0,0}};

byte LED_command[HPLedCount][4] = {{0,0,0,0},
                                   {0,0,0,0},
                                   {0,0,0,0}};

int commandLength;

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

enum ColorShades
{
  ShadeSolid=0,
  Shade1,
  Shade2,
  Shade3,
  Shade4,
  Shade5,
  ShadeOff,
  ShadeCount
};
const uint32_t basicColors[ColorCount][ShadeCount] =
                                       {{    C_OFF,    C_OFF,    C_OFF,    C_OFF,    C_OFF,    C_OFF, C_OFF},
                                        {    C_RED,  C_WHITE, 0xFB6161, 0xFFA0A0, 0xFD5555, 0xFFD3D3, C_OFF},
                                        { C_YELLOW,  C_WHITE, 0xFDFD43, 0xFFFF82, 0xFFFFBA, 0xFEDED7, C_OFF},
                                        {  C_GREEN,  C_WHITE, 0x57FC57, 0x80FC80, 0xBDFFB1, 0xDDFEDD, C_OFF},
                                        {   C_CYAN,  C_WHITE, 0x38FFFF, 0x71FDFD, 0xA4FDFD, 0xCCFEFE, C_OFF},
                                        {   C_BLUE,  C_WHITE, 0xACACFF, 0x7676FF, 0x5A5AFF, 0x3D3DFF, C_OFF},
                                        {C_MAGENTA,  C_WHITE, 0xFB3BFB, 0xFD75FD, 0xFD9EFD, 0xFDCEFD, C_OFF},
                                        { C_ORANGE,  C_WHITE, 0xFB9B3A, 0xFFBE7D, 0xFCD2A7, 0xFDE9D5, C_OFF},
                                        { C_PURPLE,  C_WHITE, 0xA131A1, 0x9B449B, 0xBD5FBD, 0xD08BD0, C_OFF},
                                        {  C_WHITE, 0xC2C0C0, 0xB7B6B6, 0x858484, 0xA09F9F, 0xD1D1D1, C_OFF}};


//////////////////////////////////////////////////////////////////////
///*****               Initialize NeoPixel Strips             *****///
//////////////////////////////////////////////////////////////////////
//
// There are two main types of NeoPixel Jewel boards:
// 7 Element RGB and
// 7 Element RGBW
//
// The RGB space is defined in Adafruit_NeoPixel.h

#define NEO_JEWEL_LEDS 7

// If you have upgraded jewel boards to RGBW uncomment the following
#define NEO_JEWEL_RGBW

#ifdef NEO_JEWEL_RGBW
 #define HP_NEO_TYPE (NEO_WGRB + NEO_KHZ800)
#else
 #define HP_NEO_TYPE (NEO_GRB + NEO_KHZ800)
#endif

Adafruit_NeoPixel neoStrips[HPLedCount];

//////////////////////////////////////////////////////////////////////
///*****                 Initialize Servo Board              *****///
//////////////////////////////////////////////////////////////////////
Servos servos(SERVOI2CADDRESS);


void setup()
{
  int i;

  randomSeed(analogRead(0));               // Seeds psuedo-random number generator with current value on unconnected anolog pin to make random more randomy.

  for (i = HPLedFront; i < HPLedCount; i++)
  {
    neoStrips[i] = Adafruit_NeoPixel(7, LEDpins[i], HP_NEO_TYPE);
    twitchLEDRunTime[i] = (1000*random(LEDTwitchRunInterval[i][TwitchRangeMin],LEDTwitchRunInterval[i][TwitchRangeMax]));  // Randomly sets initial LED Twitch Run Time value
  }

 
  Serial.begin(9600);                      // Starts Serial with a baudrate of 9600
  inputString.reserve(10);                 // reserve 50 bytes for the inputString
  /* PAA: 10 bytes not 50 as commented */
 
  Wire.begin(I2CADDRESS);                  // Connects to I2C Bus and establishes address. 

  Wire.onReceive(i2cEvent);                // Register event so when we receive something we jump to i2cEvent();


  
  //////////////////////////////////////////////////////
  ///                  RC Input Setup                ///
  //////////////////////////////////////////////////////

  pinMode(RC_PIN, INPUT);

  //////////////////////////////////////////////////////
  ///                     OE Setup                   ///
  //////////////////////////////////////////////////////
  /*  if(enableOE==1) {pinMode(oepin, OUTPUT);} */
  if(OUTPUT_ENABLED_ON)
    pinMode(OUTPUT_ENABLED_PIN, OUTPUT); 

  //////////////////////////////////////////////////////
  ///                 HP Servo Setup                 ///
  //////////////////////////////////////////////////////
  centerHP(HPFront);    // Centers Front HP
  centerHP(HPRear);     // Centers Rear HP
  centerHP(HPTop);      // Centers Top HP

  ///////////////////////////////////////////////////////
  ///                  HP LED Setup                  ///
  //////////////////////////////////////////////////////    

  for (i = HPLedFront; i < HPLedCount; i++)
  {
    //***  HP SET UP  ***///
    neoStrips[i].begin(); 
    neoStrips[i].setBrightness(BRIGHT);
    neoStrips[i].show();
  }


  if(ENABLE_BASIC_HP)
    Serial.println("Basic HP Positioning Enabled");
  else
    Serial.println("Basic HP Positioning Disabled");
}



void loop(){
  Servos::move(millis());  // this performs the actual moves
  int i;
  unsigned long currTime = millis();


  if(stringComplete) {
    inputString.toCharArray(inputBuffer, 6);inputString="";                    // If a complete Command string has been flagged, write the string to array to process. 
    commandLength = (sizeof(inputBuffer) / sizeof(inputBuffer[0]));            //  Determines length of command character array.  
    if( inputBuffer[0]=='F' ||      // Front HP
        inputBuffer[0]=='R' ||      // Rear HP
        inputBuffer[0]=='T' ||      // Top HP
        inputBuffer[0]=='A'         // All three HPs
      ) {  
        if(commandLength >= 2) {typeState = inputBuffer[1]-'0';}
        if(commandLength >= 4) {functionState = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');} 
        if(commandLength >= 5) {colorState = inputBuffer[4]-'0';} 
        else {
              if(functionState==5) {colorState = COLOR_SHORT;}
              else {colorState = COLOR_DEFAULT;}
        }
        if(inputBuffer[0]=='F' || inputBuffer[0]=='A') {
          if (typeState == 0){ 
            LED_command[HPLedFront][0] = functionState;
            LED_command[HPLedFront][1] = colorState; 
            varResets(HPFront);
          }
          else if (typeState == 1) { 
            HP_command[HPServoFront][0]  = functionState;
            enableTwitchHP[HPServoFront] = false;           // Toggle Auto HP Twitch off so it doesn't interupt sequence
          }
        }

        if(inputBuffer[0]=='R' || inputBuffer[0]=='A') {
          if (typeState == 0) { 
            LED_command[HPLedRear][0] = functionState;
            LED_command[HPLedRear][1] = colorState; 
            varResets(HPRear);
          }
          else if (typeState == 1) { 
            HP_command[HPServoRear][0]  = functionState;
            enableTwitchHP[HPServoRear] = false;        // Toggle Auto HP Twitch off so it doesn't interupt sequence
          }
        } 
      
        if(inputBuffer[0]=='T' || inputBuffer[0]=='A') {
          if (typeState == 0) { 
            LED_command[HPLedTop][0] = functionState;
            LED_command[HPLedTop][1] = colorState; 
            varResets(HPTop);
          }
          else if (typeState == 1) { 
            HP_command[HPServoTop][0]  = functionState;
            enableTwitchHP[HPServoTop] = false;         // Toggle Auto HP Twitch off so it doesn't interupt sequence
          }
        }
      }
      else if (inputBuffer[0]=='S') {                     // Major Sequences (Both LEDS and HP Servos)
        if(commandLength >= 2) {
             functionState = (inputBuffer[1]-'0');
             switch (functionState) { 
               case 1: 
                       for(i=HPFront; i<HPCount;i++)
                       {
                         varResets(i);                      // Resets the Variables for LEDs on all 3 HPs
                         enableTwitchHP[i]=false;           // Disables Auto HP Twith on all HPs  
                       }
                       downHP(HPServoFront);                                              // Moves Front HP to Down Position
                       ledOFF(HPLedRear);ledOFF(HPLedTop);                                // Turns Off LEDs on Rear and Top HPs
                       LED_command[HPLedRear][0] = NULL; LED_command[HPLedTop][0] = NULL; // Flushes Command Array for Rear and Top HP
                       LED_command[HPLedFront][0] = 1;                                    // Set Leia LED Sequence to run each loop.
                       break;
               case 8: 
                       enableTwitchHP[HPServoFront]=false;  // Disables Auto HP Twith on all HPs
                       enableTwitchHP[HPServoRear]=false;               
                       enableTwitchHP[HPServoTop]=false;
                       for(i=HPLedFront; i<HPLedCount;i++)
                       {
                         LED_command[i][0]=NULL;            // Flushes Command Array for Rear and Top HP
                         ledOFF(i);                         // Turns Off LEDs on all HPs
                         enableTwitchLED[i] = false;        // Disables Auto LED Twith on all HPs 
                       }
                       break;
               case 9: 
                       for(i=HPFront; i<HPCount;i++)
                       {
                         enableTwitchHP[i] = true;          // Enables Auto HP Twith on all HPs'
                         LED_command[i][0]=NULL;            // Flushes Command Array for all HPs
                         ledOFF(i);
                         enableTwitchLED[i] = true;
                       }
                       break;
             }
         }
      }
      ///***  Clear States and Reset for next command.  ***///
    inputString = "";
    stringComplete = false; 
    inputBuffer[0] = '\0';
  }

  /*
  ** Execute commands for Fronts, Rears, Tops.
  */
  for(i=HPFront; i < HPCount; i++)
  {
    switch (LED_command[i][0]) {
      case 1: leiaLED(i);break;
      case 2: colorProjectorLED(i,LED_command[i][1]); break; 
      case 3: dimPulse(i, LED_command[i][1]); break;
      case 4: cycle(i,LED_command[i][1]); break;
      case 5: ShortCircuit(i,LED_command[i][1]); break;
      case 6: ledColor(i,LED_command[i][1]);LED_command[i][0]=NULL; break;
      case 7: rainbow(i); break;   
      case 98: ledOFF(i); enableTwitchLED[i] = false; LED_command[i][0]=NULL; break;   // Clear Function, Disable Random LED Twitch
      case 99: ledOFF(i); enableTwitchLED[i] = true;  LED_command[i][0]=NULL; break;   // Clear Function, Enable Random LED Twitch
      default: break;
    }

    switch (HP_command[i][0]) {
      case 1: centerHP(HPServoFront); HP_command[i][0]=NULL; break; 
      case 2: downHP(HPServoFront);   HP_command[i][0]=NULL; break;
      case 3: upHP(HPServoFront);     HP_command[i][0]=NULL; break;
      case 4: RCHP(HPServoFront); break; 
      case 5: twitchHP(HPServoFront); HP_command[i][0]=NULL; break;
      case 98: enableTwitchHP[HPServoFront]=false; HP_command[i][0]=NULL; break;   // Clear Function, Disable Servo Random Twitch
      case 99: enableTwitchHP[HPServoFront]=true;  HP_command[i][0]=NULL; break;   // Clear Function, Enable Servo Random Twitch
      default: break;
    }
  }

  for (i=HPLedFront; i<HPLedCount; i++)
  {
    if (currTime > twitchLEDTime[i] && enableTwitchLED[i])
      leiaLEDtwitch(i);
  }

  for (i=HPServoFront; i<HPServoCount; i++)
  {
    if (currTime > twitchHPTime[i] && enableTwitchHP[i])
    {
      twitchHP(i);
      twitchHPTime[i] = (1000*random(HPTwitchInterval[i][TwitchRangeMin],HPTwitchInterval[i][TwitchRangeMax])) + millis();
    } 
  }

  // If Servo output is enabled and we has been on longer than our slowest move speed
  // (which means the servos should have moved to where they were going)
  // then we can turn it off. This avoids servo hum when the servos are static.
  if(OEFlag &&
     (millis() - OECounter) >= SERVO_SPEED[ServoDelayMax])
  {
    digitalWrite(OUTPUT_ENABLED_PIN, HIGH);
    OEFlag = false; 
  }
              
}

int mapPulselength(int microseconds) {
  int y = map(microseconds, 1000, 2000, 200, 550);
  return y;
}


//////////////////////////////////////////////////////////////////////////////////////////
///*****                       OE Servo Enable Function                           *****///
//////////////////////////////////////////////////////////////////////////////////////////

void enableServos() {
  if(OUTPUT_ENABLED_ON) {                // If OE control of the Servo Break Out Board is enabled
   digitalWrite(OUTPUT_ENABLED_PIN, LOW);       // Set OE Pin to low to enable Servo Movement 
   OEFlag = true;                     // Set OE Flag so loop knows servos are enabled
   OECounter = millis();           // Set OE Timer to current millis so we can eventually run disableServos after movement is complete 
  }
}

 
//////////////////////////////////////////////////////////////////////////////////////////
///*****                     HP Twitch/Movement Functions                         *****///
//////////////////////////////////////////////////////////////////////////////////////////
void centerHP(byte hp){
  int pos1;
  int pos2; 

  enableServos();
  if(ENABLE_BASIC_HP) {
    pos1 = mapPulselength(HPposBasic[HPPosBasicVecCenter][HPAxisX]);
    pos2 = mapPulselength(HPposBasic[HPPosBasicVecCenter][HPAxisY]);
  }
  else {
    pos1 = mapPulselength(HPpos[hp][HPPosVecCenter][HPAxisX]);
    pos2 = mapPulselength(HPpos[hp][HPPosVecCenter][HPAxisY]);
  }
  servos.moveTo(HPpins[hp][HPAxisX],SERVO_SPEED[ServoDelayMin],pos1);
  servos.moveTo(HPpins[hp][HPAxisY],SERVO_SPEED[ServoDelayMin],pos2);
}
   
void downHP(byte hp){
  if(!ENABLE_BASIC_HP) {      //  Only Works when Basic Position Mode is Disabled because it relies on specific position Coordinates
    enableServos();
    servos.moveTo(HPpins[hp][HPAxisX],SERVO_SPEED[ServoDelayMin],mapPulselength(HPpos[hp][HPPosVecDown][HPAxisX]));
    servos.moveTo(HPpins[hp][HPAxisY],SERVO_SPEED[ServoDelayMin],mapPulselength(HPpos[hp][HPPosVecDown][HPAxisY])); 
  }
}
    
void upHP(byte hp){      //  Only Works when Basic Position Mode is Disabled because it relies on specific position Coordinates
  if(!ENABLE_BASIC_HP) {
    enableServos();
    servos.moveTo(HPpins[hp][HPAxisX],SERVO_SPEED[ServoDelayMin],mapPulselength(HPpos[hp][HPPosVecUp][HPAxisX]));
    servos.moveTo(HPpins[hp][HPAxisY],SERVO_SPEED[ServoDelayMin],mapPulselength(HPpos[hp][HPPosVecUp][HPAxisY]));
  }
}


    
void twitchHP(byte hp) {
  int pos1;
  int pos2;
     
  enableServos();   
  if(ENABLE_BASIC_HP){
    // Pick a random X,Y spot given our min/max bounds
    pos1 = mapPulselength(random(HPposBasic[HPPosBasicVecMin][HPAxisX],HPposBasic[HPPosBasicVecMax][HPAxisX]));
    pos2 = mapPulselength(random(HPposBasic[HPPosBasicVecMin][HPAxisY],HPposBasic[HPPosBasicVecMax][HPAxisY]));
  }
  else {
    int posInt = random(0,HPPosVecCount);                // Pick a random position
    pos1 = mapPulselength(HPpos[hp][posInt][HPAxisX]);
    pos2 = mapPulselength(HPpos[hp][posInt][HPAxisY]);
  }

  int speed = random(SERVO_SPEED[ServoDelayMin],SERVO_SPEED[ServoDelayMax]);
  servos.moveTo(HPpins[hp][HPAxisX],speed,pos1);
  servos.moveTo(HPpins[hp][HPAxisY],speed,pos2);
}


// Drive the Servos according the the RC input
void RCHP(byte hp) {
  int servo1;
  int servo2;

  if(!ENABLE_BASIC_HP) {
    RCinputCh1 = pulseIn(RC_PIN, HIGH); // each channel
    Serial.println(RCinputCh1);
    enableServos();
    servo1 = map(RCinputCh1, RCRange[RCRangeMin], RCRange[RCRangeMax], HPpos[hp][HPPosVecDown][HPAxisX], HPpos[hp][HPPosVecUp][HPAxisX]);
    servo2 = map(RCinputCh1, RCRange[RCRangeMin], RCRange[RCRangeMax], HPpos[hp][HPPosVecDown][HPAxisY], HPpos[hp][HPPosVecUp][HPAxisY]);
    servos.moveTo(HPpins[hp][HPAxisX],0,mapPulselength(servo1)); // Speed 0 means asap!
    servos.moveTo(HPpins[hp][HPAxisY],0,mapPulselength(servo2)); 
  }      
}


//////////////////////////////////////////////////////////////////////////////////////////
///*****                            HP LED Functions                              *****///
//////////////////////////////////////////////////////////////////////////////////////////

void ledOFF(byte hp){
  int i;

  for(i=0; i < NEO_JEWEL_LEDS; i++)
    neoStrips[hp].setPixelColor(i,C_OFF);
  neoStrips[hp].show();
}

      
void ledColor(byte hp, int c)
{
  int i;

  for(i=0; i < NEO_JEWEL_LEDS; i++)
    neoStrips[hp].setPixelColor(i,basicColors[c][ShadeSolid]);
  neoStrips[hp].show();
}


      
//////////////////////////////////////////////////////////
///*****       Leia Function          *****///
//////////////////////////////////////////////////////////
void leiaLED(byte hp)
{
  int i;
        
  if ((millis() - tCounter[hp]) > interval[hp]) {
    for(i=0; i < NEO_JEWEL_LEDS; i++)
      neoStrips[hp].setPixelColor(i,basicColors[COLOR_DEFAULT][random(ShadeSolid,ShadeCount)]);
    neoStrips[hp].show();
    tCounter[hp]=millis();
    interval[hp]=random(50,150);
  }
}
        
 
void leiaLEDtwitch(int hp)
{
  if((millis() - twitchLEDTime[hp]) < twitchLEDRunTime[hp])
  {
    leiaLED(hp);
  }
  else
  {
    ledOFF(hp);
    twitchLEDTime[hp] = (1000*random(LEDTwitchInterval[hp][TwitchRangeMin],LEDTwitchInterval[hp][TwitchRangeMax]))+millis();
    twitchLEDRunTime[hp] = (1000*random(LEDTwitchRunInterval[hp][TwitchRangeMin],LEDTwitchRunInterval[hp][TwitchRangeMax]));
  }
}

//////////////////////////////////////////////////////////
///*****       Color Projector Function          *****///
///                                                   ///
///   Same as Leia Function above but can pic color   ///
///                                                   ///
//////////////////////////////////////////////////////////
void colorProjectorLED(byte hp, int c)
{
  if ((millis() - tCounter[hp]) > interval[hp])
  {
    for(int i=0; i < NEO_JEWEL_LEDS; i++)
      neoStrips[hp].setPixelColor(i,basicColors[c][random(ShadeSolid,ShadeCount)]);
    neoStrips[hp].show();
    tCounter[hp]=millis();
    interval[hp]=random(50,150); /* TODO fix hardcode */
  }
}
        
/////////////////////////////////////////////////////////
///*****           Dim Pulse Function            *****///
/////////////////////////////////////////////////////////
        
void dimPulse(byte hp, int c) {
  int frame_interval = 15;
  long elapsed;
  int frame;

  if ((millis() - tCounter[hp]) > interval[hp])
  {
    elapsed = millis() - tCounter[hp];
    frame = elapsed/frame_interval;
    if(frame >= 64)
      tCounter[hp] = millis();
    if (frame > 32)
      frame = 32 - (frame - 32);
    if(elapsed >= frame_interval)
    {
      for(int i=0; i < NEO_JEWEL_LEDS; i++)
        neoStrips[hp].setPixelColor(i, dimColorVal(c,(frame*8)));
      neoStrips[hp].show();
    }
  }
}


void ShortCircuit(byte hp, int c) {
  //int interval;

  if(SCloop[hp] <= 20) {
    if ((millis() - tCounter[hp]) > SCinterval[hp]) {
      if(SCflag[hp] == 0) {
        for(int i=0; i < NEO_JEWEL_LEDS; i++)
          neoStrips[hp].setPixelColor(i,C_OFF);
        SCflag[hp]=1;
        SCinterval[hp] = 10 + (SCloop[hp] * random(15,25));
      } 
      else {
        for(int i=0; i < NEO_JEWEL_LEDS; i++)
          neoStrips[hp].setPixelColor(i,basicColors[c][random(ShadeSolid,ShadeCount)]); 
        neoStrips[hp].show(); // PAA This is redundant with the one below
        SCflag[hp] = 0;
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
  byte frame;
  int interval = 75; // TODO inteval is a constant?


  if ((millis() - tCounter[hp]) > interval) {
    tCounter[hp] = millis();
    if (Frame[hp] >= 6)
      Frame[hp]=0;
    for (int i = 0; i <=5; i++) { 
      if (i == Frame[hp])
        neoStrips[hp].setPixelColor(i+1, basicColors[c][ShadeSolid]);
      else
        neoStrips[hp].setPixelColor(i+1, C_OFF);
    }
    neoStrips[hp].show(); 
    Frame[hp]++;
  }
}




///////////////////////////////////////////////////////
///*****           Rainbow Functions           *****///
///////////////////////////////////////////////////////
      
void rainbow(byte hp) {
  int interval = 10;
  long elapsed;
  byte frame;        

  Serial.println("Rainbow");


  elapsed = millis() - tCounter[hp];
  frame = elapsed/interval;
  if(frame > 256*5)
  {
    tCounter[hp] = millis();
  }
  else
  {
    for(int i=0; i< 7; i++)
    {
      neoStrips[hp].setPixelColor(i, Wheel(((i * 256 / 7) + frame) & 255));
    }
    if(elapsed >= interval)
      neoStrips[hp].show();
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
/// uint32_tcolor value.                             ///
/// Used for soft fade in/outs.                      ///
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
/////                          Consolidates Variable Reset in Single Function                       /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////            

              
void varResets(byte hp)
{
  Frame[hp]=0;
  SCflag[hp]=0;
  SCloop[hp]=0;
  SCinterval[hp]= 10;
  ledOFF(hp);                      // Clears Any Remaining Lit LEDs 
  enableTwitchLED[hp] = false;         // Toggle Auto LED Twitch off so it doesn't interupt sequence
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

    Serial.println("Serial received");
    Serial.println(inChar);

    if (inChar == '\r' || inChar == '\n') {            // if the incoming character is a carriage return (\r) or newline (\n), it denotes end of command
      stringComplete = true;                           // Once done, set a flag so the main loop can do something about it. 
    } 
    else { 
      inputString += inChar;                           // Add each Character to inputString     
    }
  }
  Serial.println("Serial received");

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
}


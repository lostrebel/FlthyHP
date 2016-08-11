////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///                                    Flthy Holoprojectors v1.2                                     ///
///                                       by Ryan Sondgeroth                                         ///
///                                         aka FlthyMcNsty                                          ///
///                                            8-10-2016                                             ///
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
///     v1.2 - Fixed Center Positioning Bug and others, streamlined Servo & LED functions;           ///
///            optimized variable use & streamlined variable containers, increased servo position    ///
///            presets to 8 directions, added status light on Serial/I2C receive, added a.           ///
///            random color option, added support for the RGBW version of the Adafruit Jewels,       ///
///            added a new HP wag function, improved the flicker behavior and color to white         ///
///            balance on the default LED function.                                                  ///
///                                                                                                  ///
///                                      Special thanks to...                                        ///
///   LostRebel and Knightshade for significant input on both the general functions of the system    ///
///          and this code!  It helped me make vast improvements in operation and efficiency.        ///                                  
///                                                                                                  ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///     Commands and Structure                                                                       ///
///                                                                                                  ///.
///     DT##C or DT##P                                                                               ///
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
///                                                                                                  ///
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
///     D101P  - Sends HP to a Preset Position*                                                      ///
///     D102   - Enables RC Control on HP (Left/Right)*                                              ///
///     D103   - Enables RC Control on HP (Up/Down)*                                                 ///
///     D104   - Sends HP to a Random Position                                                       ///
///     D105   - Wags HP Left/Right 5 times*                                                         ///
///     D106   - Wags HP Up/Down 5 times*                                                            ///
///     D198   - Disables Auto HP Twitch                                                             ///
///     D199   - Enables Auto HP Twitch                                                              ///
///                                                                                                  ///
///       S1   - Leia Mode (Front HP in Down Position, Leia LED Sequence, all other HPs disabled)*   ///
///       S8   - Clear all LEDs, Disable Auto HP Twitch, Disable Auto LED Sequence                   ///
///       S9   - Clear all LEDs, Enable Auto HP Twitch, Enable Auto LED Sequence                     ///
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
    int LEDpins[HPCOUNT]   = {2,3,4};   // {Front, Rear, Top}

  //////////////////////////////////////////////////////////////////////////////////////////
  ///*****                         Servo Board Pin Assignments                      *****///
  ///*****                                                                          *****///
  ///*****  HPpins[HPCOUNT][2] = {{Front 1, Front2}, {Rear 1, Rear 2}, {Top 1, Top 2}};   *****///
  ///*****                                                                          *****///
  //////////////////////////////////////////////////////////////////////////////////////////    
    int HPpins[HPCOUNT][2] = {{0,1},{2,3},{4,5}};           // Front HP Pins, Rear HP Pins, Top HP Pins


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
    int RCRange[2] = {1250, 1750};     // {Min, Max}
    

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
    
    unsigned int LEDTwitchInterval[HPCOUNT][2] = {{240,360},           // (4mins-6mins) Enter min and max seconds Front HP Led Twitches  
                                                  {300,420},           // (5mins-7mins) Enter min and max seconds Rear HP Led Twitches 
                                                  {300,420}};          // (5mins-7mins) Enter min and max seconds Top HP Led Twitches    
    
    unsigned int HPTwitchInterval[HPCOUNT][2] = {{45,120},             // (45s-2mins) Enter min and max seconds Front HP Servo Twitches  
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
    unsigned int LEDTwitchRunInterval[HPCOUNT][2] = {{5,25},   // (5-25s) Front LED Runtime
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
  ///*****   problematic due to the occasional servo movement attempting    *****///
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

// PAA: Recommend that define the number of positions and use that define as your
// bounds in the code.
// You can make an enum to enumerate the positions in the array which might make
// the code easier to read.
#define HPPOSITIONS 9
    int HPpos[HPCOUNT][HPPOSITIONS][2] =
                               {{{1434,1800},   // Front HP Down
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

// PAA: enableBasicHP: Since this isn't modified at runtime it could be a const or a #define
// In fact the code could be conditionally compiled for supporting one or the other
// but then you would have to compile then test, compile the other way then test.
//
// That said, I think it would be a cool idea to have a command which invoked all the features in order
// to make sure they are all working. This test/demo mode which just cycle through all the commands
// moving from one mode to the next as they complete their thing. You could also switch at runtime from
// enableBasicHP = false to true and restart the servo cycle to test that everything works as expected
// in both modes.
    boolean enableBasicHP = false;                   // false - Disabled, true- Enabled, Basic Servo Positioning
    int HPposBasic[2][3] = {{1500,1300,1700},        // HP Servo 1 Values for Basic Mode (Center, Min, Max)
                            {1500,1300,1700}};       // HP Servo 2 Values for Basic Mode (Center, Min, Max)

  ///////////////////////////////////////////////////////////////////////////////////
  ///*****                      Default Color Settings                       *****///
  ///*****                                                                   *****///
  ///*****     Select Default Colors for certain LED sequences using the     *****///
  ///*****                       integer values below:                       *****///
  ///*****                                                                   *****///
  ///*****             Basic Color Integer Values                            *****///
  ///*****               0 = Random                                          *****///
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

// PAA: These never change at runtime so make them #defines and save 2 ints. Eg
// #define DEFAULTCOLOR 5
// #define SHORTCOLOR   7
    int defaultColor = 5;     // Blue, Color integer value for the hue of default sequence. This is to appease MKelly and MKarp's whiney little asses, lol
    int shortColor   = 7;     // Orange, Color integer value for the hue of ShortCircuit Message.  


////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                  ///
///                                  End User Adjustable Settings                                    ///
///                                                                                                  ///
////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
///*****  Sequence/Function Varaiables, Containers & Counters *****///
//////////////////////////////////////////////////////////////////////
unsigned long tCounter[HPCOUNT] = {0, 0, 0};
unsigned long interval[HPCOUNT] = {100, 100, 100};
unsigned long SCinterval[HPCOUNT] = {10, 10, 10};

int SCloop[HPCOUNT] = {0, 0, 0};
int SCflag[HPCOUNT] = {0, 0, 0};
byte Frame[HPCOUNT] = {0, 0, 0};

int WagCount[HPCOUNT]= {-1,-1,-1};  // Use of negative value to determine first loop 
long WagTimer[HPCOUNT]={0,0,0};

int RCinputCh1;

long twitchLEDTime[HPCOUNT]  = {4000, 4000, 4000}; //LEDS Start up 4 seconds after boot
long twitchHPTime[HPCOUNT] = {4000, 4000, 4000}; //HPs Start 4 seconds after boot
long twitchLEDRunTime[HPCOUNT];

long ledtimer = 0;      
long int OECounter = 0;
boolean OEFlag = false;

//////////////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
int commandInt;

// PAA: NOTE 01: DANGER: You define inputBuffer as 5 chars big here but else you say it is 6 chars, risking overwriting memory.
// better would be to #define INPUTBUFLEN X and use INPUTBUFLEN as the size rather than using literal constants
char inputBuffer[5];  
String inputString = "";                       // a string to hold incoming data
volatile boolean stringComplete  = false;      // whether the serial string is complete

int typeState;
int functionState;
int colorState;
int positionState;

// PAA: You have arrays of 3x4 elements but only use 3x2
byte HP_command[HPCOUNT][4] = {{0,0,0,0},
                               {0,0,0,0},
                               {0,0,0,0}};

byte LED_command[HPCOUNT][4] = {{0,0,0,0},
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

// PAA: A comment explaining why colours are repeated 3 times, then white, then shades of colour, then off 3 times would help here!
// I think I understand what your intent was but not 100% 
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
// PAA: Should this be in setup()?
Servos servos(SERVOI2CADDRESS);

// PAA: I'd move this further up the file to group it with all the other pin references
#define STATUSLEDPIN 13

void setup() { 
  randomSeed(analogRead(0));               // Seeds psuedo-random number generator with current value on unconnected anolog pin to make random more randomy.
 
  Serial.begin(9600);                      // Starts Serial with a baudrate of 9600

  // PAA: 10 bytes vs. 50 bytes in the comment. Might want to make them match!
  inputString.reserve(10);                 // reserve 50 bytes for the inputString
 
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
 for (int i=0; i<HPCOUNT; i++) {positionHP(i,1,300);}  // Centers Each HP

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

  Serial.println("");
  Serial.print(F("FlthyHPs, Sketch Version 1.")); Serial.println(VERSION);
  if(enableBasicHP) {Serial.println(F("Basic HP Positioning Enabled")); }
  else {Serial.println(F("Basic HP Positioning Disabled"));}
  Serial.println("");
}

void loop(){
  Servos::move(millis());  // this performs the actual moves
  unsigned long currTime = millis();
  if(millis() - ledtimer >= 200) {digitalWrite(13, LOW);}                     

  // PAA: NOTE 01a: DANGER see matching note above.
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
        if(commandLength >= 5) {
             if(typeState == 0) {colorState = inputBuffer[4]-'0';}
             else {positionState = inputBuffer[4]-'0';}
        } 
        else {
              if(functionState==5) {colorState = shortColor;}
              else {colorState = defaultColor;}
        }
        // PAA: You don't need these flushes array statements. The optimizer is probably removing it for you but since you assign
        // the same location immediately in the next statement it does nothing.
        if(inputBuffer[0]=='F' || inputBuffer[0]=='A') {
          if (typeState == 0) { LED_command[0][0] = '\0';               // Flushes Array
                                LED_command[0][0] = functionState;
                                if(colorState==0) {LED_command[0][1]=random(0,9);}
                                else {LED_command[0][1] = colorState;} 
                                varResets(0);
                                }
          else if (typeState == 1) { HP_command[0][0] = '\0';           // Flushes Array
                                     HP_command[0][0] = functionState;
                                     HP_command[0][1] = positionState;
                                     enableTwitchHP[0]=false;           // Toggle Auto HP Twitch off so it doesn't interupt sequence
                                     }
        }

        if(inputBuffer[0]=='R' || inputBuffer[0]=='A') {
          if (typeState == 0) { LED_command[1][0] = '\0';               // Flushes Array
                                LED_command[1][0] = functionState;
                                if(colorState==0) {LED_command[1][1]=random(0,9);}
                                else {LED_command[1][1] = colorState;} 
                                varResets(1);
                                }
          else if (typeState == 1) { HP_command[1][0] = '\0';           // Flushes Array
                                     HP_command[1][0] = functionState;
                                     HP_command[1][1] = positionState;
                                     enableTwitchHP[1]=false;           // Toggle Auto HP Twitch off so it doesn't interupt sequence
                                     }
        } 
      
        if(inputBuffer[0]=='T' || inputBuffer[0]=='A') {
          if (typeState == 0) { LED_command[2][0]   = '\0';             // Flushes Array
                                LED_command[2][0] = functionState;
                                if(colorState==0) {LED_command[2][1]=random(0,9);}
                                else {LED_command[2][1] = colorState;} 
                                varResets(2);
                                }
          else if (typeState == 1) { HP_command[2][0] = '\0';           // Flushes Array
                                     HP_command[2][0] = functionState;
                                     HP_command[2][1] = positionState;
                                     enableTwitchHP[2]=false;           // Toggle Auto HP Twitch off so it doesn't interupt sequence
                                     }
        }
      }
      else if (inputBuffer[0]=='S') {                     // Major Sequences (Both LEDS and HP Servos)
        if(commandLength >= 2) {
             functionState = (inputBuffer[1]-'0');
             switch (functionState) { 
               case 1: varResets(0); varResets(1); varResets(2);                         // Resets the Variables for LEDs on all 3 HPs
                       for (int i=0; i<HPCOUNT; i++) {enableTwitchHP[i]=false;}          // Disables Auto HP Twith on all HPs           
                       positionHP(0,0);                                                  // Moves Front HP to Down Position
                       ledOFF(1);ledOFF(2);                                              // Turns Off LEDs on Rear and Top HPs
                       LED_command[1][0] = '\0'; LED_command[2][0] = '\0';               // Flushes Command Array for Rear and Top HP
                       LED_command[0][0] = 1;                                            // Set Leia LED Sequence to run each loop.
                       break;
               case 8: for (int i=0; i<HPCOUNT; i++) {
                                enableTwitchHP[i]=false;                                 // Disables Auto HP Twith on all HPs 
                                LED_command[i][0] = '\0';                                // Flushes Command Array for all HPs
                                ledOFF(i);                                               // Turns Off LEDs on all HPs
                                enableTwitchLED[i]=false;                                // Disables Auto LED Twith on all HPs
                       }
                       break;
               case 9: for (int i=0; i<HPCOUNT; i++) {
                                enableTwitchHP[i]=true;                                  // Enables Auto HP Twith on all HPs 
                                LED_command[i][0] = '\0';                                // Flushes Command Array for all HPs
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
       inputBuffer[0] = '\0';
       int displayState;
       int typeState;
       int colorState;
       int functionState;   
    }

  for(int i=0; i<HPCOUNT; i++) {

    switch (LED_command[i][0]) {
      case 1: colorProjectorLED(i,defaultColor); break;       // Leia Sequence
      case 2: colorProjectorLED(i,LED_command[i][1]); break; 
      case 3: dimPulse(i, LED_command[i][1]); break;
      case 4: cycle(i,LED_command[i][1]); break;
      case 5: ShortCircuit(i,LED_command[i][1]); break;
      case 6: ledColor(i,LED_command[i][1]);LED_command[i][0]='\0'; break;
      case 7: rainbow(i); break;  
      case 98: ledOFF(i); enableTwitchLED[i]=false; LED_command[i][0]='\0'; break;   // Clear Function, Disable Random LED Twitch
      case 99: ledOFF(i); enableTwitchLED[i]=true; LED_command[i][0]='\0'; break;   // Clear Function, Enable Random LED Twitch
      default: break;
    }
  
    switch (HP_command[i][0]) {
      case 1: positionHP(i,HP_command[i][1]); HP_command[i][0]='\0'; break;  
      case 2: RCHP(i,1); break;
      case 3: RCHP(i,2); break;
      case 4: twitchHP(i); HP_command[i][0]='\0'; break;
      case 5: wagHP(i,1); break;  // Wags HP Left/Right   
      case 6: wagHP(i,2); break;  // Wags HP Up/Down  
      case 98: enableTwitchHP[i]=false; HP_command[i][0]='\0'; break;   // Clear Function, Disable Servo Random Twitch
      case 99: enableTwitchHP[i]=true; HP_command[i][0]='\0'; break;   // Clear Function, Enable Servo Random Twitch
      default: break;
    } 
  }     
  
 for (int i=0; i<HPCOUNT; i++) {
    if (currTime > twitchLEDTime[i] && enableTwitchLED[i]) {LEDtwitch(i, defaultColor);}
  }


 for (int i=0; i<HPCOUNT; i++) {
    if (currTime > twitchHPTime[i] && enableTwitchHP[i]) {
      twitchHP(i);
      twitchHPTime[i] = (1000*random(HPTwitchInterval[i][0],HPTwitchInterval[i][1])) + millis();
    } 
  }

  if(OEFlag &&
     (millis()-OECounter) >= SERVO_SPEED[1]) {                // Checks the time since the last enable Servo flag was set, to ensure the servos were given plenty of 
              digitalWrite(OUTPUT_ENABLED_PIN, HIGH);         // time to execute thier full movement before diasabling servos on breakoutboard.    This is done to stop 
              OEFlag = false;                                 // servo hum if it is a problem.
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
  if(OUTPUT_ENABLED_ON) {                       // If OE control of the Servo Break Out Board is enabled
   digitalWrite(OUTPUT_ENABLED_PIN, LOW);       // Set OE Pin to low to enable Servo Movement 
   OEFlag = true;                               // Set OE Flag so loop knows servos are enabled
   OECounter = millis();                        // Set OE Timer to current millis so we can eventually run disableServos after movement is complete 
  }
 }

 
//////////////////////////////////////////////////////////////////////////////////////////
///*****                     HP Twitch/Movement Functions                         *****///
//////////////////////////////////////////////////////////////////////////////////////////

// PAA: The two versions of positionHP seem to duplicate a lot of
// code for very little functional difference. You could probably
// use just one and change your speed range arrays to have
// some thing like {default, min, max}

    void positionHP(byte hp, byte pos, int speed) {
     int pos1;
     int pos2; 
     if(enableBasicHP && pos == 1) {
        pos1 = mapPulselength(HPposBasic[0][0]);
        pos2 = mapPulselength(HPposBasic[1][0]);
      }
      else if (!enableBasicHP) {
        pos1 = mapPulselength(HPpos[hp][pos][0]);
        pos2 = mapPulselength(HPpos[hp][pos][1]);
      }
      if(!enableBasicHP || (enableBasicHP && pos == 1)) {     
        enableServos();
        servos.moveTo(HPpins[hp][0],speed,pos1);
        servos.moveTo(HPpins[hp][1],speed,pos2);
      }
    }

   void positionHP(byte hp, byte pos) {
     int pos1;
     int pos2; 
     if(enableBasicHP && pos == 1) {
        pos1 = mapPulselength(HPposBasic[0][0]);
        pos2 = mapPulselength(HPposBasic[1][0]);
      }
      else if (!enableBasicHP) {
        pos1 = mapPulselength(HPpos[hp][pos][0]);
        pos2 = mapPulselength(HPpos[hp][pos][1]);
      }
      if(!enableBasicHP || (enableBasicHP && pos == 1)) {     
        enableServos();
        servos.moveTo(HPpins[hp][0],SERVO_SPEED[0],pos1);
        servos.moveTo(HPpins[hp][1],SERVO_SPEED[0],pos2);
      }
    }

    
    void twitchHP(byte hp) {
     int speed = random(SERVO_SPEED[0],SERVO_SPEED[1]);
     enableServos();   
     if(enableBasicHP) {
        servos.moveTo(HPpins[hp][0],speed,mapPulselength(random(HPposBasic[0][1],HPposBasic[0][2])));
        servos.moveTo(HPpins[hp][1],speed,mapPulselength(random(HPposBasic[1][1],HPposBasic[1][2])));
      }
     else {positionHP(hp,random(0,HPPOSITIONS),speed);}
    }

    void RCHP(byte hp, byte type) {     
       int servo1;
       int servo2;
       if(!enableBasicHP) {
           RCinputCh1 = pulseIn(RCPIN, HIGH); // each channel
           //Serial.println(RCinputCh1);
           if(RCinputCh1>0) {
             enableServos();
             if(type==1) {                                                                                  // RC Controls Left-Right Movement
               servo1 = map(RCinputCh1, RCRange[0], RCRange[1], HPpos[hp][3][0], HPpos[hp][6][0]);
               servo2 = map(RCinputCh1, RCRange[0], RCRange[1], HPpos[hp][3][1], HPpos[hp][6][1]);
             } else if(type==2) {                                                                           // RC Controls Up-Down Movement
               servo1 = map(RCinputCh1, RCRange[0], RCRange[1], HPpos[hp][0][0], HPpos[hp][2][0]);
               servo2 = map(RCinputCh1, RCRange[0], RCRange[1], HPpos[hp][0][1], HPpos[hp][2][1]);
             }
             servos.moveTo(HPpins[hp][0],0,mapPulselength(servo1));servos.moveTo(HPpins[hp][1],0,mapPulselength(servo2));
           }
           else { Serial.println(F("No RC Input Detected!")); }
       }      
    }

// PAA: Nice addition! I love it!
    void wagHP(byte hp, byte type){
      if(!enableBasicHP) {
        if(WagCount[hp] < 0) {WagCount[hp] = 0;WagTimer[hp]=millis();}                  
        if(millis()-WagTimer[hp]>=400) {
           WagCount[hp]++;
           WagTimer[hp]=millis();          
           if (WagCount[hp] % 2) {
            if(type==1) {positionHP(hp, 3, 250);}    //  On Odd Count, Wag HP Left
            if(type==2) {positionHP(hp, 0, 250);}    //  On Odd Count, Wag HP Down
            }
           else {
            if(type==1) {positionHP(hp, 6, 250);}    //  On Even Count, Wag HP Right  
            if(type==2) {positionHP(hp, 2, 250);}    //  On Even Count, Wag HP Up 
            } 
        }
        if(WagCount[hp]>=10) {
           WagCount[hp] = -1;
           HP_command[hp][0]='\0';
           positionHP(hp, 1);  // Center
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
          else {
                ledOFF(hp);
                twitchLEDTime[hp] = (1000*random(LEDTwitchInterval[hp][0],LEDTwitchInterval[hp][1]))+millis();
                twitchLEDRunTime[hp] = (1000*random(LEDTwitchRunInterval[hp][0],LEDTwitchRunInterval[hp][1])); 
                }             
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
// PAA: Enhancement request: A neat addition to the pulse routine would be to have
// duration option so that you could make the pulse more of a
// throb vs. blip.
      void dimPulse(byte hp, int c) {
        int inter = 15;
        long elapsed;
        int frames;
        if ((millis() - tCounter[hp]) > interval[hp]) {
            elapsed = millis() - tCounter[hp];
            frames = elapsed/inter;
            if(frames >= 64) {tCounter[hp] = millis();}     
            if (frames > 32) {frames = 32-(frames - 32);}
            if(elapsed>=inter) {
               for(int i=0; i<NEO_JEWEL_LEDS; i++) {neoStrips[hp].setPixelColor(i, dimColorVal(c,(frames*8)));}
               neoStrips[hp].show();
            }
        }
      }

      void ShortCircuit(byte hp, int c) {
        if(SCloop[hp]<=20) {
           if ((millis() - tCounter[hp]) > SCinterval[hp]) {
              if(SCflag[hp]==0) {
                for(int i=0; i<NEO_JEWEL_LEDS; i++) { neoStrips[hp].setPixelColor(i,C_OFF); }  
                SCflag[hp]=1;
                SCinterval[hp] = 10+(SCloop[hp]*random(15,25));
                } 
              else {
                for(int i=0; i<NEO_JEWEL_LEDS; i++) { neoStrips[hp].setPixelColor(i,basicColors[c][random(0,10)]); }
                neoStrips[hp].show();
                SCflag[hp]=0;
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
        if(frames > 256*5) { tCounter[hp]=millis(); }
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
        } else if(WheelPos < 170) {
         WheelPos -= 85;
         return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        } else {
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
/////                          Consolidates Variable Reset in Single Function                       /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////            
             
       void varResets(byte hp) {
            Frame[hp]=0;
            SCflag[hp]=0;
            SCloop[hp]=0;
            SCinterval[hp] = 10;
            ledOFF(hp);                            // Clears Any Remaining Lit LEDs 
            enableTwitchLED[hp]=false;             // Toggle Auto LED Twitch off so it doesn't interupt sequence
         }

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                            Onboard LED Communication Status Trigger                           /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////// 
    void statusLED() {
      digitalWrite(STATUSLEDPIN, HIGH);
      ledtimer = millis();
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
          statusLED();
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
         statusLED();
      }

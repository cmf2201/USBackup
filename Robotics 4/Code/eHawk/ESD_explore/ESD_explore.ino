


#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <RA8875.h>
#include <FlexCAN_T4.h>
#include <TimeLib.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

/*
  NOTES:
  1. I’ll make a full video with better lighting tomorrow (maybe I’ll bring it outside).
  2. In said video, I’ll vary update time to mimic more realistic readings.
  3. I still have to comment and push relevant code to GitHub (also TBD tomorrow).
  4. I’m trying to figure out an easy way to import a 7-segment font, but I think this one works well enough for now.
  5. Segmenting off the display with a grey line is so much better than doing nothing. The battery reading, hours/minutes, time, and energy consumed all mushed together in the middle, making it very difficult to read at a glance without lines breaking the display up.
  6. We could put battery % in the middle of a battery symbol with the same width without reducing text size, but it looks ugly.
  7. There’s one more flicker optimization that can be done. Currently, I have readings flicker if and only if there is a change in # of digits. Not only can I make this more specific to be a decrease in # of digits (because increases automatically overwrite smaller readings), but I can also remove the excess around the new smaller reading instead of use a fixed black rectangle to erase. It’s a fickle change and might not be worth the time, given the low amount of flicker I’ve observed on both high and low update speeds, but it might be useful to eliminate flicker completely. I’ll probably end up doing it anyway just because any flicker really irritates me.
  8. I think some more improvements could be made: increasing the font size of the Motor, Contrl, Bat, Main, and Aux readings so that they match the middle column, decreasing Current (A) font size to match middle column readings as well, and minor adjustments to spacing so that the middle isn’t cramped.
  9. Temperature values should be updated to use the same VTEXT function if the values exceed 100C or go below 10C. Only testing/research will tell.
  Final Instrumentation Layout V1 on 5" display
*/

//pins
#define rotaryButton 2  //define rotary encoder

//debug tools
#define debugMode true     //use to enable all debug commands
#define overideSensors (debugMode && true)  //overide "X" if sensors are not connected
#define disableAudio (debugMode && true)  //disable teensy onboard audio

//SD card
String fileNAME = "testFlight2";

String FILENAME = fileNAME;

const int chipSelect = BUILTIN_SDCARD;

#define RA8875_CS 10 //see below...
#define RA8875_RESET 9//any pin or nothing!

#define TFT_BLACK   0x0000
#define TFT_GREY    0x8410
#define TFT_WHITE   0xFFFF
#define TFT_RED     0xF800
#define TFT_ORANGE  0xFA60
#define TFT_YELLOW  0xFFE0
#define TFT_LIME    0x07FF
#define TFT_GREEN   0x07E0
#define TFT_CYAN    0x07FF
#define TFT_AQUA    0x04FF
#define TFT_BLUE    0x001F
#define TFT_MAGENTA 0xF81F
#define TFT_PINK    0xF8FF

#define RPMMAX 3000
#define RED2 2800
#define YELLOW2 2500
#define GREEN -1
#define YELLOW1 -1

int TXRXtime1 = 0;
int TXRXtime2 = 0;

int degred2 = 240 - ((270 * RED2) / RPMMAX);
int degyellow2 = 240 - ((270 * YELLOW2) / RPMMAX);
int deggreen = 240 - ((270 * GREEN) / RPMMAX);
int degyellow1 = 240 - ((270 * YELLOW1) / RPMMAX);

#include "fonts/akashi_36.c"

String wrap = ";";
bool READING1 = false;
bool READING2 = false;
char str1[30]; //Initialized variable to store recieved data 
String storesave1 = "";
String storesave2 = "";
String storesave = "";

float voltage = 0;
float voltageprev = 0;
int voltyprev = 425;
float current = 0;
float currentprev = 0;
float rpm = 0;
float rpmprev = 0;
float rpmdegprev = 240;
int perthrot = 0;
int perthrotprev = 0;
float powerdegprev = 0;
float power = 0;
float powerprev = 0;
int v9yprev = 100;

int TEST = 0;
int UPDATETEST = 0;

int updateTime = 0;
int k = 0; // setting up parameters that will simulate display values
int d = 0;
int k1 = 0;
int d1 = 90;
int v1 = 500;
int v2 = 500;
int v3 = 500;
int v4 = 500;
int v5 = 500;
int v6 = 0;
int v7 = 0;
int val4prev = 0; // previous values to update for checking changes
int val5prev = 0;
int val6prev = 0;
int val7prev = 0;
int v8prev = 0;
int v8 = 0;
int v9prev = 0;
int dvalprev = 0;
int dval1prev = 0;

int per = 100;
int perprev = 100;

int rQ = 0;
int rK = 0;
int rQi = 0;
int rKi = 0;

int contemp = 0;
int mottemp = 0;
int battemp = 0;
int contempprev = 0;
int mottempprev = 0;
int battempprev = 0;
int contempprevy = 425;
int mottempprevy = 425;
int battempprevy = 425;

double batprev = 0;
double bat = 0;

double batCAP = 50; // Battery Max. Capacity in Ah
int celln = 28; // number of cells in battery

float AMPHR = 0;
float KWHR = 0;
float AMPHRPREV = 0;
float KWHRPREV = 0;

int count = 0;
int count1 = 0;
int pressedTime = 0;
int betweenpressedTime = 0;
int sdTime = 0;

int countaux = 0;
float auxavg = -1;
float auxavgprev = 0;
double cellvoltage = 0;
double cellvoltageprev = 0;

int getdata = 1;
int getdataprev = 1;
int cancount = 0;
int cantime = 0;
bool canON = false;
bool canONprev = false;

int PPR = 3;
int PR = 3;
int DP = 4;
int D = 4;
int scrolltime = 0;

float aux = 12;
float auxprev = 12;
unsigned int eeAddress = 0;
bool buttonON = false;
bool arrowTOG = false;
bool SDlog = false;
bool resetlight = false;
bool newname = false;
String csv = ".csv";
int initialT = 0;
int ATB = 0;
int ATBprev = 0;

RA8875 tft = RA8875(RA8875_CS, RA8875_RESET); //Teensy3/arduino's

//-----------------enables debugging values---------------------
void debugging()
{

}

// function calling a bar display, with TL and BR coordinates (topx, topy) and (botx, boty), respectively
// p1, p2, p3, and p4 are the respective pixel heights of the red, yellow, green, and yellow regions
// the final red region is calculated from the remainder of the height minus the sum of other region heights
void visual2() 
{
  //Throttle
  tft.setCursor(20, 5);
  tft.println("Throttle");
  //RPM
  //Battery
  //KW
  //Motor
  //Contrl
  //Bat
  //Time
  //Energy
  //Main
  //Current
  //Aux
  //SD/reset
  //segment display
  tft.drawLine(0, 230, 330, 230, TFT_GREY); // segmenting display into five parts (top,
  tft.drawLine(330, 230, 330, 480, TFT_GREY); // flight time, energy consumed, temperatures,
  tft.drawLine(470, 210, 470, 480, TFT_GREY); // and battery information)
  tft.drawLine(470, 210, 800, 210, TFT_GREY);
  tft.drawLine(330, 250, 470, 250, TFT_GREY);
  tft.drawLine(330, 325, 470, 325, TFT_GREY);
}

void visual() {

  
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // static labels
  tft.setFontScale(1);
  tft.setCursor(35, 230);
  tft.println("Motor");
  tft.setCursor(135, 230);
  tft.println("Contrl");
  tft.setCursor(260, 230);
  tft.println("Bat");
  tft.setCursor(520, 395);
  tft.println("Aux");
  tft.setCursor(660, 385);
  tft.println("RESET");
  tft.setCursor(655, 440);
  if (SDlog == false) {
    tft.println("SD OFF");
  }
  if (SDlog == true) {
    tft.println("SD ON");
  }
  tft.setFontScale(2);
  tft.setCursor(490, 235);
  tft.println("Main");
  tft.setCursor(620, 235);
  tft.println("Current");

  tft.setFontScale(0.75);
  tft.setCursor(103, 463);
  tft.println("C");
  tft.drawCircle(100, 463, 2, TFT_WHITE);
  tft.setCursor(209, 463);
  tft.println("C");
  tft.drawCircle(206, 463, 2, TFT_WHITE);
  tft.setCursor(311, 463);
  tft.println("C");
  tft.drawCircle(308, 463, 2, TFT_WHITE);
  tft.setCursor(345, 255);
  tft.println("Hours");
  tft.setCursor(405, 255);
  tft.println("Minutes");
  tft.setCursor(340, 340);
  tft.println("Energy Consumed");
  tft.setCursor(20, 5);
  tft.println("Throttle");

  tft.setFontScale(2);
  tft.setCursor(185, 175);
  tft.println("RPM");
  tft.setCursor(585, 155);
  tft.println("KW");
  tft.setFontScale(1);


  circledisplay(200, 118, 95, 15, -30, 240, degred2, degyellow2, deggreen, degyellow1); // calling all static display symbols
  circledisplay(600, 108, 90, 15, -30, 240, 0, 20, 270, 270);
  vertbardisplay(60, 270, 89, 445, 15, 23, 130, 9);
  vertbardisplay(166, 270, 195, 445, 15, 23, 130, 9);
  vertbardisplay(268, 270, 297, 445, 15, 23, 130, 9);
  linedisp(60, 30, 130, 3, 10, 20);
  battery(400, 105, 150, 100, 20, 50);

  tft.drawLine(0, 230, 330, 230, TFT_GREY); // segmenting display into five parts (top, flight time, energy consumed, temperatures, and battery information)
  tft.drawLine(330, 230, 330, 480, TFT_GREY);
  tft.drawLine(470, 210, 470, 480, TFT_GREY);
  tft.drawLine(470, 210, 800, 210, TFT_GREY);
  tft.drawLine(330, 250, 470, 250, TFT_GREY);
  tft.drawLine(330, 325, 470, 325, TFT_GREY);

  bat = (float)map((batCAP - AMPHR), 0, batCAP, 0, 100) / 100;
  per = int(100 * bat);

  VTEXT(378, 410, 105, 10, 0, AMPHR, PR, PPR, "Ah", 2, 0.75, -5, 28, 65, 48, 1, TFT_WHITE); // Ah
  VTEXT(378, 360, 105, 10, 0, KWHR, PR, PPR, "kWh", 2, 0.75, -5, 28, 65, 48, 1, TFT_WHITE);
  VTEXT(330, 180, 115, 15, 0, per, PR, PPR, "%", 3, 1, 15, 28, 80, 64, 0, TFT_WHITE);
  VTEXT(135, 85, 140, 0, 0, rpm, D, DP, "", 3, 1, 15, 28, 80, 64, 0, TFT_WHITE);
  VTEXT(490, 345, 95, 5, 0, cellvoltage, PR, PPR, " / CELL", 1, 0.75, 8, 15, 80, 32, 1, TFT_WHITE);
  VTEXT(635, 280, 95, 30, 0, current, PR, PPR, "A", 3, 1, 15, 28, 80, 64, 0, TFT_WHITE);
  VTEXT(480, 280, 95, 30, 0, voltage, PR, PPR, "V", 3, 1, 15, 28, 80, 64, 0, TFT_WHITE);
  VTEXT(517, 73, 95, 30, 0, power, PR, PPR, "", 3, 1, 15, 28, 80, 64, 1, TFT_WHITE);
  VTEXT(15, 170, 80, 5, 0, perthrot, PR, PPR, "%", 1, 0.75, 8, 15, 45, 32, 0, TFT_WHITE);
  tft.drawRect(650, 426, 100, 10, TFT_WHITE);
  if (arrowTOG == false) {
    tft.fillTriangle(615, 445, 615, 465, 630, 455, TFT_WHITE);
  }
  else {
    tft.fillTriangle(615, 395, 615, 415, 630, 405, TFT_WHITE);
  }
}

void vertbardisplay(int topx, int topy, int botx, int boty, int p1, int p2, int p3, int p4) {
  tft.fillRect(topx, topy, botx - topx, p1, TFT_RED);
  tft.fillRect(topx, topy + p1, botx - topx, p2, TFT_YELLOW);
  tft.fillRect(topx, topy + p1 + p2, botx - topx, p3, TFT_GREEN);
  tft.fillRect(topx, topy + p1 + p2 + p3, botx - topx, p4, TFT_YELLOW);
  tft.fillRect(topx, topy + p1 + p2 + p3 + p4, botx - topx, boty - (topy + p1 + p2 + p3 + p4), TFT_RED);
  tft.drawLine(topx, topy, botx, topy, TFT_WHITE);
  tft.drawLine(topx, topy, topx, boty, TFT_WHITE);
  tft.drawLine(topx, boty, botx, boty, TFT_WHITE);
  tft.drawLine(botx, topy, botx, boty, TFT_WHITE);
}

// battery function called, effectively two rectanges with a deleted line between top and middle parts
void battery(int centerx , int centery , int H1 , int W1 , int H2 , int W2) {
  int x = centerx - (W1 / 2);
  int y = centery - (H1 / 2);
  tft.drawRect(x, y, W1, H1, TFT_WHITE);
  tft.drawRect(x + ((W1 - W2) / 2), y - H2, W2, H2, TFT_WHITE);
  tft.drawLine(x + ((W1 - W2) / 2) + 1, y, x + ((W1 + W2) / 2) - 2, y, TFT_BLACK);
  tft.drawLine(x + ((W1 - W2) / 2) + 1, y - 1, x + ((W1 + W2) / 2) - 2, y - 1, TFT_BLACK);
}

void linedisp(int lxi, int lyi, int heig, int ticks, int ST, int LT) {
  tft.drawLine(lxi, lyi, lxi, lyi + heig, TFT_WHITE);
  tft.drawLine(lxi, lyi, lxi - LT, lyi, TFT_WHITE);
  tft.drawLine(lxi, lyi + heig, lxi - LT, lyi + heig, TFT_WHITE);
  int diff = heig / (ticks + 1);
  for (int i = 1; i <= ticks; i++) {
    tft.drawLine(lxi, lyi + (i * diff), lxi - ST, lyi + (i * diff), TFT_WHITE);
  }
}

// function calling dial display, with centers, radius of display, added thickness of colored regions, and degree limits (mindeg, maxdeg, y1, g, y2, r2) for colored regions

void circledisplay(int centerx, int centery, int radius, int thickness, int mindeg, int maxdeg, int y1 , int g , int y2 , int r2) {
  for (int i = mindeg; i < maxdeg; i++) {

    // this part of the loop creates the dial of desired thickness by creating two triangles that fill in the region of a quadrilateral between four points: (r,theta), (r+thickness, theta), (r, theta+delta(theta)), (r+thickness, theta+delta(theta)) [here,  delta(theta) = 1 degree]

    int x_cur = radius * cos(i * PI / 180.0) + centerx;
    int y_cur = radius * sin(-i * PI / 180.0) + centery;

    int x_cur_thick = (radius + thickness) * cos(i * PI / 180.0) + centerx;
    int y_cur_thick = (radius + thickness) * sin(-i * PI / 180.0) + centery;

    int x_next = radius * cos((i + 2) * PI / 180.0) + centerx;
    int y_next = radius * sin(-(i + 2) * PI / 180.0) + centery;

    int x_next_thick = (radius + thickness) * cos((i + 2) * PI / 180.0) + centerx;
    int y_next_thick = (radius + thickness) * sin(-(i + 2) * PI / 180.0) + centery;


    // coloring the quadrilaterals that make up the static part of the dial, depending on their degree limits

    if (i <= y1 & i >= mindeg) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_RED);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_RED);
    }

    if (i <= g & i > y1) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_YELLOW);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_YELLOW);
    }

    if (i <= y2 & i > g) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_GREEN);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_GREEN);
    }

    if (i <= r2 & i > y2) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_YELLOW);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_YELLOW);
    }

    if (i <= maxdeg & i > r2) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_RED);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_RED);
    }

    // drawing white border lines at each step to create a continuous curve border on inside and out

    tft.drawLine(x_cur, y_cur, x_next, y_next, TFT_WHITE);
    tft.drawLine(x_cur_thick, y_cur_thick, x_next_thick, y_next_thick, TFT_WHITE);
  }

  // closing the border on both ends with straight lines

  tft.drawLine(radius * cos(mindeg * PI / 180.0) + centerx, radius * sin(-mindeg * PI / 180.0) + centery, (radius + thickness)*cos(mindeg * PI / 180.0) + centerx, (radius + thickness)*sin(-mindeg * PI / 180.0) + centery, TFT_WHITE);
  tft.drawLine(radius * cos((maxdeg + 1)*PI / 180.0) + centerx, radius * sin(-(maxdeg + 1)*PI / 180.0) + centery, (radius + thickness)*cos((maxdeg + 1)*PI / 180.0) + centerx, (radius + thickness)*sin(-(maxdeg + 1)*PI / 180.0) + centery, TFT_WHITE);
}


//=========================SETUP=========================
void setup() {
  if (debugMode)
  {
    debugging();
  }
  tft.begin(RA8875_800x480);
  tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600); // For debug
  Serial1.begin(115200); //Serial Communicaiton over pins 0(RX) and 1(TX)
  Serial2.begin(115200); //Serial Communicaiton over pins 7(RX) and 8(TX)

  //begin flexcan
  can1.begin();
  can1.setBaudRate(125000);

  EEPROM.get(eeAddress, KWHR);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, AMPHR);

  visual();

  //define pins 9 and 2 and pullups
  pinMode(8, INPUT_PULLUP);
  pinMode(rotaryButton, INPUT_PULLUP);

  //enable teensy audio board
  if(!disableAudio)
  {
    SPI.setMOSI(7);  // Audio shield has MOSI on pin 7
    SPI.setSCK(14);  // Audio shield has SCK on pin 14
  }


 

  // see if the card is present and can be initialized: 
  Serial.print("Initializing SD card...");
  
  if (!SD.begin(chipSelect)) 
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

}



//-----------------------function to fill battery---------------------------

void fillbat(int centerx /* center x */, int centery /* center y*/, int H1 /* height of middle part*/, int W1 /* width of middle part*/, int H2 /* height of top part */, int W2 /* width of top part */, double P /* percent of battery full */, uint16_t /* color to fill with */color) {

  int HM = int(P * (H1 + H2));
  int x = centerx - (W1 / 2);
  int y = centery - (H1 / 2);
  float prop = (float)H1 / (H1 + H2);

  // two cases, depending on whether or not fill region reaches top part

  if (P <= prop) {
    tft.fillRect(x + 2, y + (H1 - HM) + 2, W1 - 4, HM - 4, color);
  }

  if (P > prop) {
    tft.fillRect(x + 2, y + 2, W1 - 4, H1 - 4, color);
    tft.fillRect(x + (W1 - W2) / 2 + 2, y - (HM - H1) + 2, W2 - 4, HM - H1, color);
  }

}



//--------------------function to fill inverse of battery,----------------------
//    used to fill remainder of battery black and mitigate flicker whenever there
//  is a change in the amount of battery that is full (P is percent battery full)

void invfillbat(int centerx, int centery, int H1, int W1, int H2, int W2, double P, uint16_t color) {

  int HM = int(P * (H1 + H2));
  int x = centerx - (W1 / 2);
  int y = centery - (H1 / 2);
  float prop = (float)H1 / (H1 + H2);

  if (P <= prop) {
    tft.fillRect(x + 2, y + 2, W1 - 4, (H1 - HM) - 4, color);
    tft.fillRect(x + (W1 - W2) / 2 + 2, y - (H2) + 2, W2 - 4, H2, color);
  }

  if (P > prop) {
    tft.fillRect(x + (W1 - W2) / 2 + 2, y - H2 + 2, W2 - 4, H2 - HM + H1, color);
  }

}

//----------------function calling the pointer of the dial,---------------------
//                which is drawn as a triangle with points:
// (radius-len, deg+(thick/2)), (radius-len, deg-(thick/2)), and (radius, deg)

void dialcirc(int16_t centerx, int16_t centery, int16_t radius, int16_t len, int16_t deg, int16_t thick, uint16_t colorfill, uint16_t colorborder) {
  int x1 = (radius - len) * cos((deg - (thick / 2)) * PI / 180.0) + centerx;
  int y1 = (radius - len) * sin(-(deg - (thick / 2)) * PI / 180.0) + centery;
  int x2 = (radius - len) * cos((deg + (thick / 2)) * PI / 180.0) + centerx;
  int y2 = (radius - len) * sin(-(deg + (thick / 2)) * PI / 180.0) + centery;
  int x3 = (radius) * cos((deg) * PI / 180.0) + centerx;
  int y3 = (radius) * sin(-(deg) * PI / 180.0) + centery;
  tft.fillTriangle(x1, y1, x2, y2, x3, y3, colorfill);
  tft.drawLine(x1, y1, x2, y2, colorborder);
  tft.drawLine(x2, y2, x3, y3, colorborder);
  tft.drawLine(x3, y3, x1, y1, colorborder);
}

//---------------countdown-----------------

void countdown (int16_t P) {
  for (int i = 0; i < 6; i++) {
    tft.drawLine(647 + P, 428 + i, 747, 428 + i, TFT_GREEN);
    tft.drawLine(655, 428 + i, 655 + (P - 8), 428 + i, TFT_BLACK);
  }
}

//------------------bar display indicator with global coordinates (x,y)---------------

void vertdialbar (int16_t x, int16_t y, int16_t len, int16_t thick, uint16_t colorfill, uint16_t colorborder) {
  int x1 = x;
  int y1 = y;
  int x2 = x + (len);
  int y2 = y + (thick / 2);
  int x3 = x + (len);
  int y3 = y - (thick / 2);
  tft.fillTriangle(x1, y1, x2, y2, x3, y3, colorfill);
  tft.drawLine(x1, y1, x2, y2, colorborder);
  tft.drawLine(x2, y2, x3, y3, colorborder);
  tft.drawLine(x3, y3, x1, y1, colorborder);
}

//------------ determines what the color of the dial indicator/triangle should be-------------

void dialcirccolor (int16_t centerx, int16_t centery, int16_t radius, int16_t len, int16_t deg, int16_t thick, uint16_t colord, int mindeg, int maxdeg, int y1, int g, int y2, int r2) {
  if (deg >= mindeg & deg <= y1) {
    colord = TFT_RED;
  }
  if (deg > y1 & deg <= g ) {
    colord = TFT_YELLOW;
  }
  if (deg > g & deg <= y2) {
    colord = TFT_GREEN;
  }
  if (deg > y2 & deg <= r2) {
    colord = TFT_YELLOW;
  }
  if (deg > r2 & deg <= maxdeg) {
    colord = TFT_RED;
  }
  dialcirc(centerx, centery, radius, len, deg, thick, colord, TFT_WHITE);
}

//--------------------ditto the above funciton for a vertical bar--------------
void vertdialbarcolor (int16_t x, int y, int16_t topy, int16_t boty, int16_t len, int16_t thick, uint16_t colord, int p1, int p2, int p3, int p4) {
  if (y >= topy & y <= (topy + p1)) {
    colord = TFT_RED;
  }
  if (y > (topy + p1) & y <= (topy + p1 + p2)) {
    colord = TFT_YELLOW;
  }
  if (y > (topy + p1 + p2) & y <= (topy + p1 + p2 + p3)) {
    colord = TFT_GREEN;
  }
  if (y > (topy + p1 + p2 + p3) & y <= (topy + p1 + p2 + p3 + p4)) {
    colord = TFT_YELLOW;
  }
  if (y > (topy + p1 + p2 + p3 + p4) & y <= boty) {
    colord = TFT_RED;
  }
  vertdialbar(x, y, len, thick, colord, TFT_WHITE);
}

// this function can display up to 4-digit values so that the values are almost centered
// at the same x position no matter how many digits
// it erases the readout every time there is a change in the number of digits displayed
// PR is the current # of digits
// PPR is the previous # of digits
// adjx, adjy, and buf adjust the value of the deletion box when PR is not PPR
// per and perprev determine the PR and PPR values, respectively
// STR is the units string, like V for volts or mWh for mega-watthours
// F1 and F2 are the number and unit font sizes, respectively
// diffx and diffy determine the placement of the units value relative to the number
// shift determines the x directional shift that the units must initially be placed at
// heig is deletion box height (usually a power of 2 for various font sizes)
// Dec is the number of preferred decimals in the numerical readout
// it toggle buf to change the buffer of the blackout region that is use

void VTEXT(int x, int y, int buf, int adjx, int adjy, double per, int PR, int PPR, String STR, int F1, int F2, int diffx, int diffy, int shift, int heig, int Dec, uint16_t color) {
  tft.setFontScale(F1);
  tft.setCursor(x, y);
  tft.setTextColor(color);

  if (per < 10) {
    tft.setCursor(x + (3 * diffx), y);
    PR = 1;
  }

  if (per >= 10 && per < 100) {
    tft.setCursor(x + (2 * diffx), y);
    PR = 2;
  }

  if (per >= 100 && per < 1000) {
    tft.setCursor(x + diffx, y);
    PR = 3;
  }

  tft.println(per, Dec);
  tft.setFontScale(F2);
  tft.setCursor(x + shift + (3 * diffx), y + diffy);

  if (per < 10) {
    tft.setCursor(x + shift, y + diffy);
  }
  if (per >= 10 && per < 100) {
    tft.setCursor(x + shift + diffx, y + diffy);
  }
  if (per >= 100 && per < 1000) {
    tft.setCursor(x + shift + (2 * diffx), y + diffy);
  }

  tft.println(STR);
}

// line display
// ticks is # of ticks
// ST is small tick length
// LT is large tick lengths (there are always two to begin and end the line display)
// lxi and lyi are coordinates of display
// heig is height


//------------------size Directory------------------------
//  this function simply finds and returns the number of
// files on SD card with the same "FILENAME"
//NOTE: This may overwrite files in the same directory
// if the files are not in conceutive order 

int sizeDirectory(File dir,String fileName) {

  int k = 0;
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      break;
    }
    //checks how many other files of the same name exist    
    if(String(entry.name()).length() >= fileName.length())
    {
      if(String(entry.name()).substring(0,fileName.length()) == fileName)
      {
        k = k + 1;
      }
    }
    entry.close();
  }
  
  return k;

}


//-----------------read sensors----------------
// streamlines reading of sensors for ease of 
// orginization
void readSensors()
{
  buttonON = !digitalRead(rotaryButton);
  
  //------------------------------------------------------------------------
  //these 2 functions takes data from Serial1 line and stores the data from it into
  //a string called "store save". Serial Readings must begin with ";"
  if ((Serial1.available()) && (READING1 == false)) {
    TXRXtime1 = millis() + 10;
    READING1 = true;
  } 
  
  if ((TXRXtime1 < millis()) && (READING1 == true)) {
    int i = 0;
    storesave1 = ",";
    while (Serial1.available() && i < 30) {
      str1[i++] = Serial1.read();
      if (String(str1[0]) != ";") {
        break;
      }
      if (i > 0) {
        storesave1 += String(str1[i]);
      }
    }
    str1[i++] = '\0';
    READING1 = false;
  }
  
  if ((Serial2.available()) && (READING2 == false)) {
    TXRXtime2 = millis() + 10;
    READING2 = true;
  }

  if ((TXRXtime2 < millis()) && (READING2 == true)) {

    int i = 0;
    storesave2 = ",";
    while (Serial2.available() && i < 30) {
      str1[i++] = Serial2.read();
      if (String(str1[0]) != ";") {
        break;
      }
      if (i > 0) {
        storesave2 += String(str1[i]);
      }
    }
    str1[i++] = '\0';
    READING2 = false;
  }
  //---------------------------------------------------
  
}


//======================= main loop =======================
void loop() {
  
  readSensors();

  //code that will log data onto the SD card
  if ((SDlog == true) && (sdTime < millis())) {
    //creates a new name for data, by finding using the defined fileNAME 
    //then adding consecutive numbers to the end of file (starting with 0)
    if (newname == true) {
      //reset Filename
      FILENAME = fileNAME;
      //File root = SD.open("/");
      //int A = sizeDirectory(root,FILENAME);
      //root.close();
      String bruh = String(month());
      Serial.println();
      FILENAME += monthShortStr(month());
      //+ "_" + dayShortStr(weekday())
      //+ "_" + String(year()) + "_" + String(hour()) + "_" + String(minute()) + "_" + String(second()))      FILENAME += "_";
      FILENAME += dayShortStr(weekday()); //this is causing problems??
      FILENAME += ".csv";
      Serial.println("TEST");
      Serial.println(FILENAME);
      newname = false;
      initialT = millis();

    }

    int logTime = millis() - initialT;

    String dataString = "";
    dataString += String(logTime);
    dataString += ",";
    dataString += String(voltage);
    dataString += ",";
    dataString += String(aux);
    dataString += ",";
    dataString += String(current);
    dataString += ",";
    dataString += String(rpm);
    dataString += ",";
    dataString += String(perthrot);
    dataString += ",";
    dataString += String(contemp);
    dataString += ",";
    dataString += String(mottemp);
    dataString += ",";
    dataString += String(battemp);
    dataString += ",";
    dataString += String(KWHR);
    dataString += ",";
    dataString += String(AMPHR);

    //converts FILENAME to a pointer char (necessay for SD.open)
    char FILENAMEchar[FILENAME.length()];
    for (int i = 0; i < FILENAME.length(); i++)
    {
      FILENAMEchar[i] = FILENAME[i];
    }
    Serial.println("TEST");
    Serial.println(FILENAMEchar);
    File dataFile = SD.open(FILENAMEchar, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      Serial.println("TEST2");
      dataFile.print(dataString);
      dataFile.print(storesave1);
      dataFile.println(storesave2);
      dataFile.close();
      // print to the serial port too:
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.csv");
    }

    sdTime = millis() + 100;
  }
  
  if (can1.read(msg)) {

    if (msg.id == 346095618) {

      voltage = (msg.buf[0] + (256 * msg.buf[1])) / 57.45;
      current = (msg.buf[2] + (256 * msg.buf[3])) / 10;
      rpm = (msg.buf[4] + (256 * msg.buf[5]) + (65536 * msg.buf[6])) * 10;

    }

    if (msg.id == 346095619) {

      contemp = msg.buf[4];
      mottemp = msg.buf[5];
      battemp = msg.buf[6];

    }

    if (msg.id == 346095620) {

      perthrot = (msg.buf[2] + (256 * msg.buf[3])) / 10;

    }

    getdata = getdata + 1;
    getdata = getdata % 2;

  }

  if (getdataprev != getdata) {
    cancount = 0;
  }

  if (getdataprev == getdata) {
    cancount = cancount + 1;
  }

  if (cancount > 600) {
    cancount = 601;
    canON = true; // CHANGE BACK TO FALSE FOR REAL TEST
  }

  else {
    canON = true;
  }


  //!!!!!!!!!!!!!!!!!! see if this can all be put into a array? !!!!!!!!!!!!!!!!!!!!
  // mapping parameters that change on a repeated millis() loop according to trigonometric functions
  int val1 = map(v1, 425, 250, 30, 90);
  int val2 = map(v2, 425, 250, 10, 35);
  int val3 = map(v3, 425, 250, 10, 35);
  int volty = map(voltage, 0, 100, 425, 250);
  double val5 = map(v5, 425, 250, 80, 130);
  val5 = val5 / 10;
  int val6 = map(v6, 425, 250, 0, 25);
  int val7 = map(v6, 425, 250, 0, 200);

  // calculating hours and minutes from value for flight time
  int hours = floor(v7 / 60);
  int minutes = v7 - 60 * floor(v7 / 60);

  int v9y = map(perthrot, 100, 0, 30, 160);
  int rpmdeg = map(rpm, 0, RPMMAX, 240, -30);

  float power = ((voltage * current) / 1000);
  int powerdeg = map(int(power), 0, 20, 240, -30);

  int mottempy = map(mottemp, 99, 0, 270, 445);
  int battempy = map(battemp, 99, 0, 270, 445);
  int contempy = map(contemp, 99, 0, 270, 445);

  //defining dial display PR and PPR values for all variable text (VTEXT) outputs
  // only the RPM output uses 4 digits

  // updating display tickers

  // updating variable text values with VTEXT according to current and previous values

  // the only values that do not change # of digits and thus don't need VTEXT are temperature and flight time values, updated in the loop below

  // updating flight time

  tft.setFontScale(2);
  tft.setCursor(410, 270);
  if (floor(minutes / 10) == 0) {
    tft.print(0);
    tft.println(minutes);
  }

  else {
    tft.println(minutes);
  }

  tft.setCursor(340, 270);
  if (floor(hours / 10) == 0) {
    tft.print(0);
    tft.print(hours);
    tft.println(":");
  }
  else {
    tft.print(hours);
    tft.println(":");
  }

  // screen update loop

  if (updateTime <= millis()) {

    if (canON == true) {

      if (canONprev != canON) {
        tft.fillRect(0, 0, 800, 480, TFT_BLACK);
        visual();

      }

      if (rpmdegprev != rpmdeg) {
        dialcirc(200, 118, 93, 20, rpmdegprev, 15, TFT_BLACK, TFT_BLACK);
      }

      vertdialbar(91, v1, 20, 20, TFT_BLACK, TFT_BLACK);
      vertdialbar(197, v2, 20, 20, TFT_BLACK, TFT_BLACK);
      vertdialbar(299, v3, 20, 20, TFT_BLACK, TFT_BLACK);

      if (v9yprev != v9y) {
        vertdialbar(62, v9yprev, 10, 10, TFT_BLACK, TFT_BLACK);
      }

      if (powerprev != power) {
        dialcirc(600, 108, 88, 20, powerdegprev, 15, TFT_BLACK, TFT_BLACK);
      }

      if (mottemp != mottempprev) {
        vertdialbar(91, mottempprevy, 20, 20, TFT_BLACK, TFT_BLACK);
      }

      if (battemp != battempprev) {
        vertdialbar(299, battempprevy, 20, 20, TFT_BLACK, TFT_BLACK);
      }

      if (contemp != contempprev) {
        vertdialbar(197, contempprevy, 20, 20, TFT_BLACK, TFT_BLACK);
      }


      dialcirccolor(600, 108, 88, 20, powerdeg, 15, TFT_BLUE, -30, 240, degred2, degyellow2, deggreen, degyellow1);

      if (power != powerprev) {
        VTEXT(517, 73, 95, 30, 0, powerprev, PR, PPR, "", 3, 1, 15, 28, 80, 64, 1, TFT_BLACK);
        VTEXT(517, 73, 95, 30, 0, power, PR, PPR, "", 3, 1, 15, 28, 80, 64, 1, TFT_WHITE);
      }

      if (perthrot != perthrotprev) {
        VTEXT(15, 170, 80, 5, 0, perthrotprev, PR, PPR, "%", 1, 0.75, 8, 15, 45, 32, 0, TFT_BLACK);
        VTEXT(15, 170, 80, 5, 0, perthrot, PR, PPR, "%", 1, 0.75, 8, 15, 45, 32, 0, TFT_WHITE);
      }

      if (voltage != voltageprev) {
        VTEXT(480, 280, 95, 30, 0, voltageprev, PR, PPR, "V", 3, 1, 15, 28, 80, 64, 0, TFT_BLACK);
        VTEXT(480, 280, 95, 30, 0, voltage, PR, PPR, "V", 3, 1, 15, 28, 80, 64, 0, TFT_WHITE);
      }

      if (current != currentprev) {
        VTEXT(635, 280, 95, 30, 0, currentprev, PR, PPR, "A", 3, 1, 15, 28, 80, 64, 0, TFT_BLACK);
        VTEXT(635, 280, 95, 30, 0, current, PR, PPR, "A", 3, 1, 15, 28, 80, 64, 0, TFT_WHITE);
      }

      vertdialbar(62, v9y, 10, 10, TFT_WHITE, TFT_WHITE);

      cellvoltage = voltage / celln ;
      cellvoltageprev = voltageprev / celln ;

      if (cellvoltage != cellvoltageprev) {
        VTEXT(490, 345, 95, 5, 0, cellvoltageprev, PR, PPR, " / CELL", 1, 0.75, 8, 15, 80, 32, 1, TFT_BLACK);
        VTEXT(490, 345, 95, 5, 0, cellvoltage, PR, PPR, " / CELL", 1, 0.75, 8, 15, 80, 32, 1, TFT_WHITE);
      }

      dialcirccolor(200, 118, 93, 20, rpmdeg, 15, TFT_BLUE, -30, 240, 0, 20, 270, 270);

      if (rpm != rpmprev) {
        VTEXT(135, 85, 140, 0, 0, rpmprev, D, DP, "", 3, 1, 15, 28, 80, 64, 0, TFT_BLACK);
        VTEXT(135, 85, 140, 0, 0, rpm, D, DP, "", 3, 1, 15, 28, 80, 64, 0, TFT_WHITE);
      }

      vertdialbarcolor(91, mottempy, 270, 445, 20, 20, TFT_BLUE, 15, 23, 130, 9);
      vertdialbarcolor(197, contempy, 270, 445, 20, 20, TFT_BLUE, 15, 23, 130, 9);
      vertdialbarcolor(299, battempy, 270, 445, 20, 20, TFT_BLUE, 15, 23, 130, 9);

      tft.setFontScale(1);
      tft.setCursor(60, 450);
      tft.println(mottemp);
      tft.setCursor(166, 450);
      tft.println(contemp);
      tft.setCursor(268, 450);
      tft.println(battemp);

      // store previous values for VTEXT calculations

      // iterate parameters
      // use parameters to update values

      // update battery display

    }

    bat = (float)map((batCAP - AMPHR), 0, batCAP, 0, 100) / 100;
    perprev = int(100 * batprev);
    per = int(100 * bat);
    auxprev = aux;

    ATB = millis();

    if (power > 0) {
      KWHR = KWHR + ((ATB - ATBprev) * 0.00000027777777) * (power);
    }

    if (current > 0) {
      AMPHR = AMPHR + ((ATB - ATBprev) * 0.000000277777) * (current);
    }

    //t Serial.println(per);

    if (AMPHR < 0) {
      AMPHR = 0;
    }

    if (KWHR < 0) {
      AMPHR = 0;
    }


    if (per != perprev) {
      VTEXT(330, 180, 115, 15, 0, perprev, PR, PPR, "%", 3, 1, 15, 28, 80, 64, 0, TFT_BLACK);
      VTEXT(330, 180, 115, 15, 0, per, PR, PPR, "%", 3, 1, 15, 28, 80, 64, 0, TFT_WHITE);
    }




    if (ceil(10 * KWHR) != ceil(10 * KWHRPREV)) {
      VTEXT(378, 360, 105, 10, 0, KWHRPREV, PR, PPR, "kWh", 2, 0.75, -5, 28, 65, 48, 1, TFT_BLACK);
      VTEXT(378, 360, 105, 10, 0, KWHR, PR, PPR, "kWh", 2, 0.75, -5, 28, 65, 48, 1, TFT_WHITE);
    }

    //    Serial.print(ceil(10*AMPHR));
    //    Serial.print(",");
    //    Serial.println(ceil(10*AMPHRPREV));

    if (ceil(10 * AMPHR) != ceil(10 * AMPHRPREV)) {
      VTEXT(378, 410, 105, 10, 0, AMPHRPREV, PR, PPR, "Ah", 2, 0.75, -5, 28, 65, 48, 1, TFT_BLACK); // Ah
      VTEXT(378, 410, 105, 10, 0, AMPHR, PR, PPR, "Ah", 2, 0.75, -5, 28, 65, 48, 1, TFT_WHITE); // Ah
    }

    AMPHRPREV = AMPHR;
    KWHRPREV = KWHR;
    contempprevy = contempy;
    mottempprevy = mottempy;
    battempprevy = battempy;
    contempprev = contemp;
    mottempprev = mottemp;
    battempprev = battemp;
    powerprev = power;
    powerdegprev = powerdeg;
    perthrotprev = perthrot;
    rpmprev = rpm;
    rpmdegprev = rpmdeg;
    currentprev = current;
    voltyprev = volty;
    voltageprev = voltage;
    v9yprev = v9y;
    v8prev = v8;
    val7prev = val7;
    val6prev = val6;
    val5prev = val5;
    batprev = bat;
    ATBprev = ATB;

    aux = analogRead(2);
    aux = map(aux, 0, 1023, 0, 3.293);
    aux = 1.04167 * (19.06 + 5.495) / 5.49 * aux;

    if (countaux < 10) {
      auxavg = auxavg + aux;
      countaux = countaux + 1;
    }

    else {

      VTEXT(495, 430, 95, 5, 0, auxavgprev / 10, PR, PPR, "V", 1, 0.75, 8, 15, 80, 32, 1, TFT_BLACK);
      VTEXT(495, 430, 95, 5, 0, auxavg / 10, PR, PPR, "V", 1, 0.75, 8, 15, 80, 32, 1, TFT_WHITE);
      auxavgprev = auxavg;
      countaux = 0;
      auxavg = 0;
    }

    fillbat(400, 105, 150, 100, 20, 50, bat, TFT_GREEN);
    invfillbat(400, 105, 150, 100, 20, 50, bat, TFT_BLACK);

    //actives red "X" if
    if (canON == 0) {

      for (int i = 0; i <= 10; i++) {
        tft.drawLine(i, 0, 320 + i, 480, TFT_RED);
        tft.drawLine(i, 480, 320 + i, 0, TFT_RED);
        tft.drawLine(470 + i, 0, 800 + i, 380, TFT_RED);
        tft.drawLine(470 + i, 380, 800 + i, 0, TFT_RED);
      }

    }

    if (count1 > 10) {

      int countper = map(count1 - 10, 1, 20, 5, 100);
      countdown(countper);

    }

    else {
      if (pressedTime == 0) {
        tft.fillRect(652, 428, 96, 6, TFT_GREEN);
      }
      else {
        tft.fillRect(652, 428, 96, 6, TFT_BLACK);
      }
    }

    tft.setFontScale(1);

    if (resetlight == true) {
      if (pressedTime > millis()) {
        tft.setCursor(660, 385);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.println("RESET");
      }
      else {
        tft.fillRect(650, 390, 100, 30, TFT_BLACK);
        tft.setCursor(660, 385);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.println("RESET");
        resetlight = false;
      }
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    updateTime = millis() + 30;
    canONprev = canON;

  }

  if (aux <= 11.5 && (count < 1)) {

    eeAddress = 0;

    EEPROM.put(eeAddress, KWHR);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, AMPHR);

    count = count + 1;

  }
  buttonON = !digitalRead(rotaryButton);

  if (buttonON && (pressedTime == 0) && (millis() > betweenpressedTime)) {

    count1 = count1 + 1;
    betweenpressedTime = millis() + 50;

    if (count1 == 30) {
      tft.setFontScale(1);
      if (arrowTOG == true) {
        KWHR = 0;
        AMPHR = 0;

        EEPROM.put(eeAddress, KWHR);
        eeAddress += sizeof(float);
        EEPROM.put(eeAddress, AMPHR);

        tft.fillRect(650, 390, 100, 30, TFT_BLACK);
        resetlight = true;

      }

      if (arrowTOG == false) {

        SDlog = !SDlog;
        tft.fillRect(650, 440, 100, 30, TFT_BLACK);
        tft.setCursor(655, 440);

        if (SDlog == true) {
          tft.println("SD ON");
          newname = true;
        }
        else {
          tft.println("SD OFF");
        }
      }

      count1 = 0;
      pressedTime = millis() + 500;

    }

  }

  if (millis() - 25 > betweenpressedTime) {
    count1 = 0;
  }

  if (pressedTime < millis()) {
    pressedTime = 0;
  }

  getdataprev = getdata;

  rQ = digitalRead(3);
  rK = digitalRead(4);
  //function for determining which option is selected for rotary encoder
  if (((rQ != rQi) || (rK != rKi)) && (scrolltime <= millis())) {
    arrowTOG = !arrowTOG;

//    tft.fillRect(600, 390, 40, 80, TFT_BLACK);

//    if (arrowTOG == true) {
//      tft.fillTriangle(615, 395, 615, 415, 630, 405, TFT_WHITE);
//    }
//
//    if (arrowTOG == false) {
//      tft.fillTriangle(615, 445, 615, 465, 630, 455, TFT_WHITE);
//    }
    scrolltime = millis() + 200;
  }

  rQi = rQ;
  rKi = rK;
  
  //visual();
}
/*
this is how I want the main loop to be partitioned:
all the logic in the beggining, at the end a visual() function is ran 
*/

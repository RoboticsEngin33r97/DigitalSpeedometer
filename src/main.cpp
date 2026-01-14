#include <Arduino.h>
#include <U8g2lib.h>
#include <MD_MAX72xx.h>
#include <MD_Parola.h>
#include <mcp_can.h>
#include <SPI.h>


//                                Arduino Nano ESP32
//
//                                    ____---____
//                  SCK/D13~  (16) --|O  |   |  O|-- (15)   D12/MISO
//                  +3V3      (17) --|O  |___|  O|-- (14)   D11/MOSI
//                  AREF      (18) --|O         O|-- (13)   D10
//                  A0        (19) --|O   [O]   O|-- (12)   D9
//                  A1        (20) --|O         O|-- (11)   D8
//                  A2        (21) --|O []  ___ O|-- (10)   D7
//                  A3        (22) --|O    |   |O|-- (9)    D6
//                  SDA/A4    (23) --|O    |___|O|-- (8)    D5
//                  SCL/A5    (24) --|O         O|-- (7)    D4
//                  A6        (25) --|O _______ O|-- (6)    D3
//                  A7        (26) --|O|       |O|-- (5)    D2
//                  +5V       (27) --|O| ESP32 |O|-- (4)    GND
//                  RESET     (28) --|O|   S3  |O|-- (3)    RESET
//                  GND       (29) --|O|_______|O|-- (2)    D0/RX0
//                  VIN       (30) --|O|_______|O|-- (1)    D1/TX0
//                  
//                  Pinout for Arduino Nano ESP32
//                  
//                  CAN BUS MCP2515
//                  CS            D6
//                  SCK           D13
//                  MISO          D12
//                  MOSI          D11
//                  
//                  OLED
//                  CS            D7
//                  SCK           D13
//                  MISO          --
//                  MOSI          D11
//                  RST           D10
//                  DC            D9
//                  
//                  8x8 Matrix
//                  CS            D5
//                  DATA          D11
//                  CLK           D13
//                  
//                  Reset btn     D4

#define MATRIXTYPE MD_MAX72XX::DR1CR1RR1_HW


// Define library objects
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI oled(U8G2_R0, /* CS */ 7, /* DC */ 9, /* Reset */ 8);
MD_Parola speedometer = MD_Parola(MATRIXTYPE, /* CS */ 5, /* number of devices */ 3);
MCP_CAN CAN0(6);

// Global variables
float oilT, oilP, fuelT, fuelP, boostP, e85conc, rpm, lambda;
int speed;

int buttonPin = 4;

// Vehicle data
uint64_t totalDistance_m = 0;
uint32_t tripDistance_m  = 0;
float distanceRemainder_m = 0.0;

// Save data
static bool savePending = false;
static uint32_t stopStartMs = 0;
const uint32_t STOP_DELAY_MS = 3000; // 3 sekunder
static uint32_t lastSavedOdo = totalDistance_m;

float tripDisplay;   // km + 100 m precision
float odoDisplay;   // hela km

// Timers
const unsigned long OLED_UPDATE_MS = 250;
unsigned long lastOledUpdate = 0;

const unsigned long MATRIX_UPDATE_MS = 100;
unsigned long lastMatrixUpdate = 0;

// Button handling
const unsigned long LONG_PRESS = 3000;
const unsigned long DEBUG_PRESS = 10000;
const int MAX_SCREEN_INDEX = 5;
bool buttonState = HIGH;
bool lastButtonState = HIGH; 
unsigned long pressStart = 0;
bool debugMode = false;
int screenIndex = 0;



void updateOdometer(float speed_kmh) {
  static uint32_t lastMs = 0;
  uint32_t now = millis();

  if (lastMs == 0) {
    lastMs = now;
    return;
  }

  uint32_t dt_ms = now - lastMs;
  lastMs = now;

  // ---- Beräkna tillryggalagd distans ----
  float dt_s = dt_ms / 1000.0;
  float speed_ms = speed_kmh / 3.6;   // km/h -> m/s
  float delta_m = speed_ms * dt_s;

  distanceRemainder_m += delta_m;

  uint32_t wholeMeters = (uint32_t)distanceRemainder_m;
  if (wholeMeters > 0) {
    totalDistance_m += wholeMeters;
    tripDistance_m  += wholeMeters;
    distanceRemainder_m -= wholeMeters;
  }

  // ---- Uppdatera displayvärden ----
  odoDisplay  = floor(totalDistance_m / 1000.0);      // hela km
  tripDisplay = floor(tripDistance_m / 100.0) / 10.0; // km + 100 m precision

  /*
  // ---- Spara var 1000:e meter ----
  if (totalDistance_m - lastSavedOdo >= 1000) {
    prefs.begin("car", false);
    prefs.putUInt("odo", (uint32_t)totalDistance_m);
    prefs.putUInt("trip", (uint32_t)tripDistance_m);
    prefs.end();
    lastSavedOdo = totalDistance_m;
    Serial.println("Saved to NVS (1000 m interval)");
  }

  // ---- Spara en gång när bilen står still ≥3 sek ----
  static bool stillSaved = false;
  static uint32_t stopStartMs = 0;
  if (speed_kmh == 0) {
    if (!stillSaved) {
      if (stopStartMs == 0) stopStartMs = now;

      if (now - stopStartMs >= 3000) { // 3 sek
        prefs.begin("car", false);
        prefs.putUInt("odo", (uint32_t)totalDistance_m);
        prefs.putUInt("trip", (uint32_t)tripDistance_m);
        prefs.end();
        lastSavedOdo = totalDistance_m;
        stillSaved = true;
        Serial.println("Saved to NVS (stopped 3s)");
      }
    }
  } else {
    // bilen rör sig, nollställ timer och flagga
    stopStartMs = 0;
    stillSaved = false;
  }
  */
}



// Main OLED
void drawMainOled() {
  oled.clearBuffer();

  oled.setFont(u8g2_font_7x14B_tr);
  oled.drawStr(0, 11, "ODO:");
  oled.setCursor(48, 11);
  oled.print(odoDisplay,0);
  oled.print(" km");

  oled.drawStr(0, 27, "Trip:");
  oled.setCursor(48, 27);
  oled.print(tripDisplay, 1);
  oled.print(" km");

  oled.setFont(u8g2_font_5x7_tr);
  oled.drawStr(0, 37, "OIL   T:     C   P:    bar");
  oled.setCursor(40, 37);
  oled.print(oilT, 1);
  oled.setCursor(95, 37);
  oled.print(oilP, 1);

  oled.drawStr(0, 45, "FUEL  T:     C   P:    bar");
  oled.setCursor(40, 45);
  oled.print(fuelT, 1);
  oled.setCursor(95, 45);
  oled.print(fuelP, 1);

  oled.drawStr(0, 54, "BOOST P:     bar E85:    %");
  oled.setCursor(40, 54);
  oled.print(boostP, 2);
  oled.setCursor(104, 54);
  oled.print(e85conc, 1);

  oled.drawStr(0, 64, "RPM:      Lambda:");
  oled.setCursor(20, 64);
  oled.print(rpm, 0);
  oled.setCursor(85, 64);
  oled.print(lambda, 3);

  oled.sendBuffer();
}

void drawOledScreen1() {
  oled.clearBuffer();
  oled.setCursor(0,8);
  oled.print(F("Screen 1"));
  oled.sendBuffer();
}

void drawDebugOled() {
  oled.clearBuffer();
  oled.setCursor(0,8);
  oled.print(F("DEBUG MODE"));
  oled.sendBuffer();
}

void updateOLED(int index){
  // Update OLED based on screen index
  oled.clearBuffer();
  switch(index){
    case 0:
      drawMainOled();
      break;
    case 1:
      //drawScreen1Oled();
      break;
    default:
      drawMainOled();
      break;
  }
}

void handleButton() {
  buttonState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && buttonState == LOW){
    pressStart = millis();
  }

  if (lastButtonState == LOW && buttonState == HIGH){
    unsigned long pressDuration = millis() - pressStart;

    if(pressDuration < LONG_PRESS){
      // Short press
      screenIndex++;
      if(screenIndex > MAX_SCREEN_INDEX) screenIndex = 0;
      // Update display based on screen index
    }
    else if (pressDuration >= LONG_PRESS && pressDuration < DEBUG_PRESS){
      // Long press - reset trip meter
      tripDistance_m = 0;
      tripDisplay = 0.0;

    }
    else if (pressDuration >= DEBUG_PRESS){
      debugMode = !debugMode;
      if (debugMode) {
        speedometer.displayText("DBUG",PA_CENTER, 50, 0, PA_OPENING_CURSOR, PA_NO_EFFECT);
        speedometer.displayAnimate();
        while(!speedometer.displayAnimate());
        drawDebugOled();
      } else {
        speedometer.displayText("0",PA_CENTER, 50, 0, PA_OPENING_CURSOR, PA_NO_EFFECT);
        speedometer.displayAnimate();
        while(!speedometer.displayAnimate());
      }
    }
  }

}

void drawMatrix(){
    speedometer.print(speed);
}

void setup() {
    // Initialization code here

    //Setup I/O
    pinMode(buttonPin, INPUT_PULLUP);

    speedometer.begin();
    speedometer.setIntensity(15);
    speedometer.displayClear();
    speedometer.setTextAlignment(PA_CENTER);
    speedometer.displayText("BOOT",PA_CENTER, 50, 0, PA_OPENING_CURSOR, PA_NO_EFFECT);
    while(!speedometer.displayAnimate());

    oled.begin();
    oled.clearBuffer();
    oled.setFont(u8g2_font_5x7_tr);
    oled.print(F("Booting..."));
    oled.sendBuffer();
    delay(200);
    oled.setCursor(0,16);
    oled.print(F("Init OLED... OK"));
    oled.setCursor(0,24);
    oled.print(F("Init MATRIX... OK"));
    oled.sendBuffer();
    delay(200);
    
    
    /*
    // Initialize MCP2515 CAN controller at 500kbps
    oled.setCursor(0,32);
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
      Serial.println("MCP2515 init OK!");
      oled.print(F("MCP2515 init OK!"));
      oled.sendBuffer();
      CAN0.setMode(MCP_NORMAL);
    } else {
      Serial.println("MCP2515 init FAIL");
      oled.print(F("MCP2515 init FAILED!"));
      oled.sendBuffer();
      speedometer.print("FAIL");
      while (1);
    }
    */

    oled.setCursor(0,40);
    oled.print(F("BOOT OK!"));
    oled.sendBuffer();

    speedometer.displayText("0",PA_CENTER, 50, 0, PA_OPENING_CURSOR, PA_NO_EFFECT);
    while(!speedometer.displayAnimate());

    delay(1000);
}

void loop() {
    // Main code here
    unsigned long now = millis();
    //speedometer.displayAnimate();

    // Display updates
    if(now - lastOledUpdate >= OLED_UPDATE_MS) {
      lastOledUpdate = now;
      oilT = random(0,1200)/10.0;
      oilP = random(0,100)/10.0;
      fuelT = random(-50,850)/10.0;
      fuelP = random(0,50)/10.0;
      boostP = random(-10,20)/10.0;
      e85conc = random(600,900)/10.0;
      rpm = random(450,7500);
      lambda = random(850,1150)/1000.0;
      updateOLED(screenIndex);
    }

    if(now - lastMatrixUpdate >= MATRIX_UPDATE_MS && !debugMode) {
      lastMatrixUpdate = now;
      speed = speed + 1;
      if (speed > 300) speed = 0;
      drawMatrix();
    }

    updateOdometer(speed);
    handleButton();
  
}

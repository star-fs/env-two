/*
 2023-07-14 Star Morin / Autospkr, Labs 
 Attribution 3.0 United States (CC BY 3.0 US) 

 Share — copy and redistribute the material in any medium or format
 Adapt — remix, transform, and build upon the material for any purpose, even commercially.
 
 This license is acceptable for Free Cultural Works.
 The licensor cannot revoke these freedoms as long as you follow the license terms.

 https://creativecommons.org/licenses/by/3.0/us/legalcode
*/

#include <SPI.h>
#include <Bounce2.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>
#include <RotaryEncoder.h>
#include <map>
#include <vector>
#include <algorithm>

#include "Wire.h"
#include "MCP4725.h"

#include "Free_Fonts.h"

#define BUTTON_W 60
#define BUTTON_H 26

TFT_eSPI tft = TFT_eSPI();

ButtonWidget btnClear1 = ButtonWidget(&tft);
ButtonWidget btnClear2 = ButtonWidget(&tft);
ButtonWidget* btn[] = { &btnClear1, &btnClear2 };
uint8_t buttonCount = sizeof(btn) / sizeof(btn[0]);

uint16_t x, y = 0;  // touch x/y
typedef struct {
  uint16_t x;
  uint16_t y;
} coords;

std::map<int, coords> envelope1;
std::map<int, coords> envelope2;

coords outerRectStart;
coords outerRectEnd;
coords innerRectStart;
coords env1CoordsStart;
coords env1CoordsEnd;
coords env2CoordsStart;
coords env2CoordsEnd;

uint16_t rectW;
uint16_t rectH;

int duration1 = 1000;
int duration2 = 1000;

MCP4725 MCP(0x62, &Wire1);

RotaryEncoder *encoder = nullptr;

Bounce2::Button b1 = Bounce2::Button();
Bounce2::Button b2 = Bounce2::Button();

void setup() {

  Serial.begin(115200);
  delay(100);

  tft.init();
  tft.setRotation(0);

  // from Touch_calibrate.ino
  // Use this calibration code in setup():
  uint16_t calData[5] = { 248, 3618, 379, 3543, 4 };
  tft.setTouch(calData);  

  outerRectStart = {2, 3};
  outerRectEnd   = {(uint16_t)(tft.width() - 2), (uint16_t)(tft.height() - 4)};

  innerRectStart = {(uint16_t)2, (uint16_t)(((tft.height() / 2) - 15))};
  rectW = ((uint16_t)(tft.width() - 3));
  rectH = 30;

  env1CoordsStart = {(uint16_t)(outerRectStart.x + 1), 3};
  env1CoordsEnd = {(uint16_t)(outerRectEnd.x - 2), (uint16_t)(innerRectStart.y - 4)};

  env2CoordsStart = {(uint16_t)(outerRectStart.x + 1), (uint16_t)(innerRectStart.y + 30)};
  env2CoordsEnd = {(uint16_t)(outerRectEnd.x - 2), (uint16_t)(outerRectEnd.y - 2)};

  tft.fillScreen(TFT_BLACK);
  tft.drawRect(outerRectStart.x, outerRectStart.x, outerRectEnd.x, outerRectEnd.y, TFT_BLUE);
  tft.fillRect(innerRectStart.x, innerRectStart.y, rectW, rectH, TFT_BLUE);
  tft.fillRect(innerRectStart.x + 20 + BUTTON_W, innerRectStart.y + 1, 78, 28, TFT_BLACK);

  MCP.begin(14, 15);
  Wire1.setClock(800000);
  MCP.setValue(0);

  if (!MCP.isConnected()) {
    Serial.println("failed to connect to MCP4725.\n");
  }

  // buttons
  b1.attach(6, INPUT_PULLUP);
  b1.interval(10);
  b1.setPressedState(LOW); 
  
  // rotary encoders
  // encoder 1
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7), checkPositionEnv1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), checkPositionEnv1, CHANGE);  

  encoder = new RotaryEncoder(7, 8, RotaryEncoder::LatchMode::FOUR0);
  encoder->setPosition(0);

  // encoder 2
  /*
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7), checkPositionEnv1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), checkPositionEnv1, CHANGE);  

  encoder = new RotaryEncoder(7, 8, RotaryEncoder::LatchMode::FOUR0);
  encoder->setPosition(0);
  */

  initEnvelopes();
  initButtons();
  drawDurationText();

}

// rotary encoder interrupt handlers
void checkPositionEnv1() {
  encoder->tick();
  duration1 = encoder->getPosition();
  if (duration1 < 0) {
      encoder->setPosition(0);
  }
  drawDurationText();
  return;
}

void checkPositionEnv2() {
  encoder->tick(); // just call tick() to check the state.
}

// trigger interupt handle 
void outputLInterp(int env) {

  Serial.print("outputLInterp triggered\n");

  float x0,y0,x1,y1,xp,yp,interval,stepLen = 0.000;
  
  int duration = duration1;

  std::map<int, coords> target = envelope1;

  if (env == 2) {
    target = envelope2;
    duration = duration2;
  }

  coords last = target[0];
  auto it=target.end();
  it--;
  coords end=it->second;

  Serial.printf("Starting Point: %d/%d\n", last.x, last.y);
  Serial.printf("Ending   Point: %d/%d\n", end.x, end.y);

  // calculate the duration of a pixel
  stepLen = duration / 240;

  std::vector<std::pair<int, coords>> sortedPairs(target.begin(), target.end());
  std::sort(sortedPairs.begin(), sortedPairs.end(), compareKeys);
  for (const auto& pair : sortedPairs) {
    x0 = (float)last.x;
    y0 = (float)last.y;
    x1 = (float)pair.second.x;
    y1 = (float)pair.second.y;
    Serial.printf("ERG: %d/%d -> %d/%d ... x1 - x0 = %.4f\n", last.x, last.y, pair.second.x, pair.second.y, (x1 - x0) / 141);
    for (int xp=x0;xp < x1; xp++) {
      yp = y0 + (((y1-y0)/(x1-x0)) * (xp - x0));
      // set the output value to yp
      Serial.printf("%.4f:", yp);
      MCP.setValue(yp);
      delay(stepLen);
    }
    last = pair.second;
    Serial.print("\n");
  }
  Serial.print(">0<\n");
  MCP.setValue(0);
}

void drawDurationText() {
  uint16_t evnF1X = ((innerRectStart.x + 20 + BUTTON_W) + 2) + 26;
  uint16_t evnF1Y = innerRectStart.y + 5;
  uint16_t evnF2X = evnF1X;
  uint16_t evnF2Y = evnF1Y + 14;
  
  tft.fillRect(innerRectStart.x + 20 + BUTTON_W, innerRectStart.y + 1, 78, 28, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(evnF1X, evnF1Y);
  tft.setTextFont(GLCD);
  tft.print(duration1);
  tft.setCursor(evnF2X, evnF2Y);
  tft.print(duration2);
  return;
}

void loop() {

  coords last1 = envelope1[0];
  coords last2 = envelope2[0];
  bool pressed = tft.getTouch(&x, &y);

  //env1Coords 3,5 -> 237,145
  //env2Coords 3,175 -> 236,314

  if (pressed) {

    if ((y > env1CoordsStart.y && y < env1CoordsEnd.y) && (x > env1CoordsStart.x && x < env1CoordsEnd.x)) {
      //Serial.printf("ENV1 x/y:%d,%d\n", x,y);
      envelope1[x] = {x,y};
      tft.fillRect(env1CoordsStart.x, env1CoordsStart.y, env1CoordsEnd.x, env1CoordsEnd.y, TFT_BLACK);
      std::vector<std::pair<int, coords>> sortedPairs(envelope1.begin(), envelope1.end());
      std::sort(sortedPairs.begin(), sortedPairs.end(), compareKeys);
      for (const auto& pair : sortedPairs) {
        tft.drawWedgeLine(last1.x, last1.y, pair.second.x, pair.second.y, 1, 1, TFT_MAGENTA, TFT_BLACK);
        last1 = pair.second;
      }
    } else {
      if ((y > env2CoordsStart.y && y < outerRectEnd.y) && (x > env2CoordsStart.x && x < outerRectEnd.x)){
        //Serial.printf("ENV2 x/y:%d,%d\n", x,y);
        envelope2[x] = {x,y};
        tft.fillRect(env2CoordsStart.x, env2CoordsStart.y, env2CoordsEnd.x, (env2CoordsEnd.y - env2CoordsStart.y) + 2, TFT_BLACK);
        std::vector<std::pair<int, coords>> sortedPairs(envelope2.begin(), envelope2.end());
        std::sort(sortedPairs.begin(), sortedPairs.end(), compareKeys);
        for (const auto& pair : sortedPairs) {
          tft.drawWedgeLine(last2.x, last2.y, pair.second.x, pair.second.y, 1, 1, TFT_MAGENTA, TFT_BLACK);
          last2 = pair.second;
        }
      }
    }
  }

  // virtual display buttons
  for (uint8_t b = 0; b < buttonCount; b++) {
    if (pressed) {
      if (btn[b]->contains(x, y)) {
        btn[b]->press(true);
        btn[b]->pressAction();
      }
    } else {
      btn[b]->press(false);
      btn[b]->releaseAction();
    }
  }

  // hardware buttons
  b1.update();

  if (b1.pressed() ) {
    Serial.print("erp\n");
    outputLInterp(1);
  }

}

void initEnvelopes() {
  // add the inital points
  envelope1[0].x = env1CoordsStart.x + 2;
  envelope1[0].y = env1CoordsEnd.y;
  envelope2[0].x = env2CoordsStart.x;
  envelope2[0].y = env2CoordsEnd.y;

  envelope1[238].x = env1CoordsEnd.x + 4;
  envelope1[238].y = env1CoordsEnd.y;
  envelope2[238].x = env2CoordsEnd.x;
  envelope2[238].y = env2CoordsEnd.y;

  tft.drawRect(outerRectStart.x, outerRectStart.x, outerRectEnd.x, outerRectEnd.y, TFT_BLUE);
}

void btnClear1_pressAction(void) {
  if (btnClear1.justPressed()) {
    btnClear1.drawSmoothButton(!btnClear1.getState(), 1, TFT_MAGENTA, "clear ^");
    btnClear1.setPressTime(millis());

    // clear data
    tft.fillRect(env1CoordsStart.x, env1CoordsStart.y, env1CoordsEnd.x, env1CoordsEnd.y, TFT_BLACK);
    envelope1.clear();
    initEnvelopes();

    btnClear1.drawSmoothButton(!btnClear1.getState(), 1, TFT_MAGENTA, "clear ^");
    btnClear1.setPressTime(millis());
  }
}

void btnClear2_pressAction(void) {
  if (btnClear2.justPressed()) {
    btnClear2.drawSmoothButton(!btnClear2.getState(), 1, TFT_MAGENTA, "clear v");
    btnClear2.setPressTime(millis());

    // clear data
    tft.fillRect(env2CoordsStart.x, env2CoordsStart.y, env2CoordsEnd.x, (env2CoordsEnd.y - env2CoordsStart.y) + 2, TFT_BLACK);
    envelope2.clear();
    initEnvelopes();

    btnClear2.drawSmoothButton(!btnClear2.getState(), 1, TFT_MAGENTA, "clear v");
    btnClear2.setPressTime(millis());
  }
}

void initButtons() {
  btnClear1.initButtonUL(innerRectStart.x + 10, innerRectStart.y + 2, BUTTON_W, BUTTON_H, TFT_MAGENTA, TFT_BLACK, TFT_MAGENTA, (char*)"clear ^", 1);
  btnClear1.setPressAction(btnClear1_pressAction);
  btnClear1.drawSmoothButton(false, 1, TFT_BLACK);  // 3 is outline width, TFT_BLACK is the surrounding background colour for anti-aliasing

  btnClear2.initButtonUL(tft.width() - (10 + BUTTON_W), innerRectStart.y + 2, BUTTON_W, BUTTON_H, TFT_MAGENTA, TFT_BLACK, TFT_MAGENTA, (char*)"clear v", 1);
  btnClear2.setPressAction(btnClear2_pressAction);
  btnClear2.drawSmoothButton(false, 1, TFT_BLACK);  // 3 is outline width, TFT_BLACK is the surrounding background colour for anti-aliasing
}

bool compareKeys(const std::pair<int, coords>& a, const std::pair<int, coords>& b) {
    return a.first < b.first;
}
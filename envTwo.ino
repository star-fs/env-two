/*
 Copyright 2023-07-14 by Star Morin (autospkr.org labs) 
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

#include "MCP_DAC.h"

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

int minLenMs = 6;
int msLen1, msLen2, msLen1Last, msLen2Last = minLenMs;

MCP4822 MCP(255, 255, &SPI1);

RotaryEncoder *encoder1 = nullptr;
RotaryEncoder *encoder2 = nullptr;

Bounce2::Button b1 = Bounce2::Button();
Bounce2::Button b2 = Bounce2::Button();
Bounce2::Button b3 = Bounce2::Button();
Bounce2::Button b4 = Bounce2::Button();

bool retrig1, retrig2 = false;

bool envLoop1, envLoop2, envGate1, envGate2, envTrig1, envTrig2 = false;

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
  outerRectEnd   = {(uint16_t)(tft.width() - 2), (uint16_t)(tft.height() - 2)};

  innerRectStart = {(uint16_t)2, (uint16_t)(((tft.height() / 2) - 15))};
  rectW = ((uint16_t)(tft.width() - 3));
  rectH = 30;

  env1CoordsStart = {(uint16_t)(outerRectStart.x + 1), 3};
  env1CoordsEnd = {(uint16_t)(outerRectEnd.x - 2), (uint16_t)(innerRectStart.y - 2)};

  env2CoordsStart = {(uint16_t)(outerRectStart.x + 1), (uint16_t)(innerRectStart.y + 30)};
  env2CoordsEnd = {(uint16_t)(outerRectEnd.x - 2), (uint16_t)(outerRectEnd.y - 2)};

  tft.fillScreen(TFT_BLACK);
  tft.drawRect(outerRectStart.x, outerRectStart.x, outerRectEnd.x, outerRectEnd.y, TFT_BLUE);
  tft.fillRect(innerRectStart.x, innerRectStart.y, rectW, rectH, TFT_BLUE);
  tft.fillRect(innerRectStart.x + 20 + BUTTON_W, innerRectStart.y + 1, 78, 28, TFT_BLACK);

  MCP.setGPIOpins(14, 8, 15, 9);  // SELECT should match the param of begin()
  MCP.begin(9);
  MCP.setGain(1);
  
  // buttons
  b1.attach(6, INPUT_PULLUP);
  b1.interval(10);
  b1.setPressedState(LOW); 
  
  b2.attach(7, INPUT_PULLUP);
  b2.interval(10);
  b2.setPressedState(LOW); 

  // analog input "buttons"
  b3.attach(26, INPUT);
  b3.interval(10);
  b3.setPressedState(HIGH); 

  b4.attach(27, INPUT);
  b4.interval(10);
  b4.setPressedState(HIGH);

  // rotary encoders

  // encoder 1
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(10), checkPositionEnv1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(11), checkPositionEnv1, CHANGE);  
  encoder1 = new RotaryEncoder(10, 11, RotaryEncoder::LatchMode::FOUR0);
  encoder1->setPosition(minLenMs);

  // encoder 2
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(12), checkPositionEnv2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(13), checkPositionEnv2, CHANGE);
  encoder2 = new RotaryEncoder(12, 13, RotaryEncoder::LatchMode::FOUR0);
  encoder2->setPosition(0);
  
  // digital switches

  // switch 1
  pinMode(28, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);

  // check boot-up state
  if (digitalRead(28) == 0) {
    envLoop1 = true;
  }
  if (digitalRead(22) == 0) {
    envGate1 = true;
  }

  // switch 2
  pinMode(5, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  // check boot-up state
  if (digitalRead(5) == 0) {
    envLoop2 = true;
  }
  if (digitalRead(4) == 0) {
    envGate2 = true;
  }

  // switch interupts
  attachInterrupt(digitalPinToInterrupt(5), toggleEnvLoop2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), toggleEnvGate2, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(28), toggleEnvLoop1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), toggleEnvGate1, CHANGE);  
  
  initEnvelopes();
  initButtons();
  drawDurationText();

}

void loop() {

  coords last;
  int whichEnv = 0;

  std::map<int, coords> target;

  bool pressed = tft.getTouch(&x, &y);

  String buttonPressed = "";

  uint16_t sx, sy, ex, ey = 0;

  // enable one-shot trigger mode if no other mode is active
  // envLoop1, envLoop2, envGate1, envGate2, envTrig1, envTrig2
  if (envLoop1 == false && envGate1 == false) {
    envTrig1 = true;
  } else {
    envTrig1 = false;
  }
  if (envLoop2 == false && envGate2 == false) {
    envTrig2 = true;
  } else {
    envTrig2 = false;
  }

  if (pressed) {
    // inside boundry of env 1
    if ((y > env1CoordsStart.y && y < env1CoordsEnd.y) && (x > env1CoordsStart.x && x < env1CoordsEnd.x)) {
      envelope1[x] = {x,y};
      target = envelope1;
      whichEnv = 1;
      last = envelope1[0];
      sx = env1CoordsStart.x;
      sy = env1CoordsStart.y;
      ex = env1CoordsEnd.x;
      ey = env1CoordsEnd.y;

    // inside boundry of env 2
    } else if ((y > env2CoordsStart.y && y < outerRectEnd.y) && (x > env2CoordsStart.x && x < outerRectEnd.x)) {
      envelope2[x] = {x,y};
      target = envelope2;
      whichEnv = 2;
      last = envelope2[0];
      sx = env2CoordsStart.x;
      sy = env2CoordsStart.y;
      ex = env2CoordsEnd.x;
      ey = env2CoordsEnd.y;
    }

    tft.fillRect(sx, sy, ex, ey, TFT_BLACK);
    for (const auto& pair : target) {
      tft.drawWedgeLine(last.x, last.y, pair.second.x, pair.second.y, 1, 1, TFT_MAGENTA, TFT_BLACK);
      last = pair.second;
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
  b2.update();

  // analog in trigger "buttons"
  b3.update();
  b4.update();

  // handle trigger events
  if (b1.pressed()) { buttonPressed = "b1";}
  if (b2.pressed()) { buttonPressed = "b2";}
  if (b3.pressed()) { buttonPressed = "b3";}
  if (b4.pressed()) { buttonPressed = "b4";}

  if (buttonPressed == "b1" || buttonPressed == "b3" || retrig1) {
    outputLInterp(1, true, buttonPressed);
  }

  if (buttonPressed == "b2" || buttonPressed == "b4" || retrig2) {
    outputLInterp(2, true, buttonPressed);
  }

  // update duration values and display
  msLen1 = encoder1->getPosition();
  if (msLen1 != msLen1Last) {
    if (msLen1 < minLenMs) {
      encoder1->setPosition(minLenMs);
      msLen1 = minLenMs;
    }
    drawDurationText();
    msLen1Last = msLen1;
  }

  msLen2 = encoder2->getPosition();
  if (msLen2 != msLen2Last) {
    if (msLen2 < minLenMs) {
      encoder2->setPosition(minLenMs);
      msLen2 = minLenMs;
    }
    drawDurationText();
    msLen2Last = msLen2;
  }

  /*
  if (Serial.available() > 0) {
    // read the incoming byte:
    String incoming = Serial.readString();

    tft.setCursor(20, 20);
    tft.setTextFont(GLCD);

    tft.print("Jules says: " + incoming);
  }
  */
}

// rotary encoder interrupt handlers
void checkPositionEnv1() {
  encoder1->tick();
}

void checkPositionEnv2() {
  encoder2->tick(); // just call tick() to check the state.
}

void toggleEnvLoop1() {
  int val = digitalRead(28);
  if (val == 1) {
    envLoop1 = false;
  } else {
    envLoop1 = true;
  }
}
void toggleEnvGate1() {
  int val = digitalRead(22);
  if (val == 1) {
    envGate1 = false;
  } else {
    envGate1 = true;
  }
}
void toggleEnvLoop2() {
 int val = digitalRead(5);
  if (val == 1) {
    envLoop2 = false;
  } else {
    envLoop2 = true;
  }
}
void toggleEnvGate2() {
 int val = digitalRead(4);
  if (val == 1) {
    envGate2 = false;
  } else {
    envGate2 = true;
  }
}

// trigger interupt handle 
void outputLInterp(int env, bool analogOut, String pressed) {

  /*
  Serial.printf("envLoop1:%d envGate1:%d - envLoop2:%d envGate2:%d ::::: envTrig1:%d envTrig2:%d\n", 
    envLoop1,
    envGate1,
    envLoop2, 
    envGate2, 
    envTrig1, 
    envTrig2
  );
  */

  float x0,y0,x1,y1,xp,yp,interval = 0.000;
  unsigned long stepLen;

  // debug
  unsigned long endtime = 0;
  unsigned long mcpTimeStart, mcpTimeEnd = 0;

  bool b1Press, b2Press, b3Press, b4Press = false;

  int iterations = 0;

  float timeDivisor = 1;

  int loopCount = 236;

  std::map<int, coords> target = envelope1;
  int duration = msLen1;

  int yOffsetDivisor = env1CoordsEnd.y;

  if (env == 2) {
    target = envelope2;
    duration = msLen2;
    yOffsetDivisor = env2CoordsEnd.y;
  }

  // if we are less than 66ms, we get rid of the multiplier

  if (duration > 66) {
    timeDivisor = 0.1;
    if (env == 1) {
      loopCount = 2440;
    }
  }

  coords last = target[0];
  auto it=target.end();
  it--;
  coords end=it->second;

  // calculate the duration of loop time in microseconds
  stepLen = (((float)duration * 1000) / (float)loopCount) - (7 / 1000);

  unsigned long start = micros();
  restart: for (const auto& pair : target) { //sortedPairs) {
    
    x0 = (float)last.x;
    y0 = (float)last.y;
    x1 = (float)pair.second.x;
    y1 = (float)pair.second.y;

    for (int xp=x0;xp < x1; xp++) {
      for (float xpSub=xp;xpSub < (xp + 1); xpSub += timeDivisor) {

        mcpTimeStart = micros();
        
        // linear interpolate, invert and scale between 0 and 1, then convert to a percent
        yp = (((((y0 + (((y1-y0)/(x1-x0)) * (xpSub - x0))) / yOffsetDivisor) - 1) * -1) * 100);
        
        // fudge factor 2 for env 2
        if (env == 2) {
          yp = yp * 2;
        }

        if (analogOut) {
          
          // check to see if we were pressed during call and exit retriggering the selected env
          b1.update();
          b2.update();
          b3.update();
          b4.update();
          b1Press = b1.pressed();
          b2Press = b2.pressed();
          b3Press = b3.pressed();
          b4Press = b4.pressed();
          if (b1Press || b3Press || b2Press || b4Press) {
            MCP.fastWriteA(0);
            if (b1Press || b3Press) {
              retrig1=true;
            }
            if (b2Press || b4Press) {
              retrig2=true;
            }
            b1.update();
            b2.update();
            b3.update();
            b4.update();
            return;
          }

          // handle gate off events
          if (env == 1 && (envGate1 == true || envLoop1 == true) 
            && ((pressed == "b3" && b3.released()) || (pressed == "b1" && b1.released()))) {
            MCP.fastWriteA(0);
            return;
          } 
          if (env == 2 && (envGate2 == true || envLoop2 == true)
            && ((pressed == "b4" && b4.released()) || (pressed == "b2" && b2.released()))) {
            MCP.fastWriteA(0);
            return;
          }

          // output analog voltages on the MCP4822
          MCP.fastWriteA((yp / 100) * 4095);
          
          mcpTimeEnd = micros();
          
          // delayMicroseconds with args <= 0 will crash.
          if ((mcpTimeEnd - mcpTimeStart) > 1 && (stepLen) > (mcpTimeEnd - mcpTimeStart)) {
            delayMicroseconds((stepLen) - (mcpTimeEnd - mcpTimeStart));
          }

        }
        iterations++;
      }
    }

    last = pair.second;
    
  }

  // restart loop if envLoop 
  if (env == 1 && envLoop1 == true 
    && ((pressed == "b3" && digitalRead(26) == 1) || (pressed == "b1" && digitalRead(6) == 0))) {

    last = target[0];
    goto restart;
  }

  if (env == 2 && envLoop2 == true 
    && ((pressed == "b4" && digitalRead(27) == 1) || (pressed == "b2" && digitalRead(7) == 0))) {

    last = target[0];
    goto restart;
  }

  if (analogOut) {
    MCP.fastWriteA(0);
  }

  endtime = micros();

  // debug print for timing

  Serial.printf("ENV number: %d following iterations: %d.  microseconds per iteration %d.  total time in microseconds: %d. target duration in ms: %d. step length in microseconds: %d.\n",
    env, 
    iterations, 
    ((mcpTimeEnd - mcpTimeStart)), 
    (endtime - start),
    duration,
    stepLen
  );
  
  // we got here, so we are not retriggering
  retrig1 = false;
  retrig2 = false;
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
  tft.print(msLen1 + 1);
  tft.setCursor(evnF2X, evnF2Y);
  tft.print(msLen2 + 1);
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

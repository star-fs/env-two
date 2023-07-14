#include <SPI.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>
#include <map>
#include <vector>
#include <algorithm>

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

void setup() {

  Serial.begin(115200);
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

  env1CoordsStart = {(uint16_t)(outerRectStart.x + 1, 3)};
  env1CoordsEnd = {(uint16_t)(outerRectEnd.x - 2), (uint16_t)(innerRectStart.y - 2)};

  env2CoordsStart = {(uint16_t)(outerRectStart.x + 1), (uint16_t)(innerRectStart.y + 30)};
  env2CoordsEnd = {(uint16_t)(outerRectEnd.x - 2), (uint16_t)(outerRectEnd.y - 2)};

  tft.fillScreen(TFT_BLACK);
  tft.drawRect(outerRectStart.x, outerRectStart.x, outerRectEnd.x, outerRectEnd.y, TFT_BLUE);
  tft.fillRect(innerRectStart.x, innerRectStart.y, rectW, rectH, TFT_BLUE);

  // add the inital points
  envelope1[0].x = env1CoordsStart.x;
  envelope1[0].y = env1CoordsEnd.y;
  envelope2[0].x = env2CoordsStart.x;
  envelope2[0].y = env2CoordsEnd.y;

  envelope1[238].x = env1CoordsEnd.x;
  envelope1[238].y = env1CoordsEnd.y;
  envelope2[238].x = env2CoordsEnd.x;
  envelope2[238].y = env2CoordsEnd.y;

  initButtons();
  
}

void loop() {

  coords last1 = envelope1[0];
  coords last2 = envelope2[0];
  bool pressed = tft.getTouch(&x, &y);

  //env1Coords 3,5 -> 237,145
  //env2Coords 3,175 -> 236,314

  if (pressed) {

    Serial.printf("env1Coords %d,%d -> %d,%d\n", env1CoordsStart.x, env1CoordsStart.y, env1CoordsEnd.x, env1CoordsEnd.y);
    Serial.printf("env2Coords %d,%d -> %d,%d\n", env2CoordsStart.x, env2CoordsStart.y, env2CoordsEnd.x, env2CoordsEnd.y);

    if ((y > env1CoordsStart.y && y < env1CoordsEnd.y) && (x > env1CoordsStart.x && x < env1CoordsEnd.x)) {
      Serial.printf("ENV1 x/y:%d,%d\n", x,y);
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
        Serial.printf("ENV2 x/y:%d,%d\n", x,y);
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
}

void btnClear1_pressAction(void) {
  if (btnClear1.justPressed()) {
    btnClear1.drawSmoothButton(!btnClear1.getState(), 1, TFT_MAGENTA, "clear ^");
    btnClear1.setPressTime(millis());
  }
}

void btnClear2_pressAction(void) {
  if (btnClear2.justPressed()) {
    btnClear2.drawSmoothButton(!btnClear2.getState(), 1, TFT_MAGENTA, "clear v");
    btnClear2.setPressTime(millis());
  }
}

void initButtons() {
  // 119, 2, 119, 159
  uint16_t x = 60;
  uint16_t y = 119;
  btnClear1.initButtonUL(innerRectStart.x + 10, innerRectStart.y, BUTTON_W, BUTTON_H, TFT_WHITE, TFT_RED, TFT_BLACK, (char*)"clear ^", 1);
  btnClear1.setPressAction(btnClear1_pressAction);
  btnClear1.drawSmoothButton(false, 1, TFT_BLACK);  // 3 is outline width, TFT_BLACK is the surrounding background colour for anti-aliasing

  y += 20;
  btnClear2.initButtonUL(tft.width() - (10 + BUTTON_W), innerRectStart.y, BUTTON_W, BUTTON_H, TFT_WHITE, TFT_BLACK, TFT_GREEN, (char*)"clear v", 1);
  btnClear2.setPressAction(btnClear2_pressAction);
  btnClear2.drawSmoothButton(false, 1, TFT_BLACK);  // 3 is outline width, TFT_BLACK is the surrounding background colour for anti-aliasing
}

bool compareKeys(const std::pair<int, coords>& a, const std::pair<int, coords>& b) {
    return a.first < b.first;
}
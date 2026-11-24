/*
 core 0: user interface

 Owns the TFT, the touch controller, the two clear buttons and the two
 duration encoders. Envelopes are drawn with a finger (top half = envelope 1,
 bottom half = envelope 2) and kept as a map of x -> point. Whenever a drawing
 or a duration changes, the envelope is converted to an EnvelopeShape and
 published for the engine on core 1 (see publishEnvelope()).

 Nothing on this core touches the DAC or the trigger inputs, so drawing and
 redrawing never interrupts envelope playback.
*/

#define BUTTON_W 60
#define BUTTON_H 26

TFT_eSPI tft = TFT_eSPI();

ButtonWidget btnClear1 = ButtonWidget(&tft);
ButtonWidget btnClear2 = ButtonWidget(&tft);
ButtonWidget* btn[] = { &btnClear1, &btnClear2 };
uint8_t buttonCount = sizeof(btn) / sizeof(btn[0]);

uint16_t x = 0, y = 0;  // touch x/y
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

int msLen1 = minLenMs, msLen2 = minLenMs, msLen1Last = minLenMs, msLen2Last = minLenMs;

RotaryEncoder *encoder1 = nullptr;
RotaryEncoder *encoder2 = nullptr;

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

  // rotary encoders

  // encoder 1
  pinMode(PIN_ENC1_A, INPUT_PULLUP);
  pinMode(PIN_ENC1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), checkPositionEnv1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1_B), checkPositionEnv1, CHANGE);
  encoder1 = new RotaryEncoder(PIN_ENC1_A, PIN_ENC1_B, RotaryEncoder::LatchMode::FOUR0);
  encoder1->setPosition(minLenMs);

  // encoder 2
  pinMode(PIN_ENC2_A, INPUT_PULLUP);
  pinMode(PIN_ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), checkPositionEnv2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2_B), checkPositionEnv2, CHANGE);
  encoder2 = new RotaryEncoder(PIN_ENC2_A, PIN_ENC2_B, RotaryEncoder::LatchMode::FOUR0);
  encoder2->setPosition(minLenMs);

  initEnvelopes();
  initButtons();
  drawDurationText();

  // give the engine something to play from the start (flat envelopes)
  publishEnvelope(1);
  publishEnvelope(2);
}

void loop() {

  int whichEnv = 0;
  std::map<int, coords> *target = nullptr;

  bool pressed = tft.getTouch(&x, &y);

  if (pressed) {
    // inside boundry of env 1
    if ((y > env1CoordsStart.y && y < env1CoordsEnd.y) && (x > env1CoordsStart.x && x < env1CoordsEnd.x)) {
      envelope1[x] = {x,y};
      target = &envelope1;
      whichEnv = 1;

    // inside boundry of env 2
    } else if ((y > env2CoordsStart.y && y < outerRectEnd.y) && (x > env2CoordsStart.x && x < outerRectEnd.x)) {
      envelope2[x] = {x,y};
      target = &envelope2;
      whichEnv = 2;
    }

    // only redraw when the touch landed inside one of the envelope areas
    if (whichEnv != 0) {
      clearEnvArea(whichEnv);
      coords last = target->begin()->second;
      for (const auto& pair : *target) {
        tft.drawWedgeLine(last.x, last.y, pair.second.x, pair.second.y, 1, 1, TFT_MAGENTA, TFT_BLACK);
        last = pair.second;
      }
      publishEnvelope(whichEnv);
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

  // update duration values and display
  msLen1 = encoder1->getPosition();
  if (msLen1 != msLen1Last) {
    if (msLen1 < minLenMs) {
      encoder1->setPosition(minLenMs);
      msLen1 = minLenMs;
    }
    drawDurationText();
    publishEnvelope(1);
    msLen1Last = msLen1;
  }

  msLen2 = encoder2->getPosition();
  if (msLen2 != msLen2Last) {
    if (msLen2 < minLenMs) {
      encoder2->setPosition(minLenMs);
      msLen2 = minLenMs;
    }
    drawDurationText();
    publishEnvelope(2);
    msLen2Last = msLen2;
  }
}

// rotary encoder interrupt handlers
void checkPositionEnv1() {
  encoder1->tick();
}

void checkPositionEnv2() {
  encoder2->tick(); // just call tick() to check the state.
}

// Screen y -> DAC value. The bottom of a drawing area is 0 V and higher on
// the screen is higher output. Envelope 2's area does not start at the top of
// the screen, so dividing by its bottom edge only reaches ~45%; the original
// "fudge factor" of 2 brings it back up to roughly full scale.
uint16_t levelForY(int env, float yp) {
  float bottom = (env == 1) ? env1CoordsEnd.y : env2CoordsEnd.y;
  float frac = 1.0f - (yp / bottom);
  if (env == 2) {
    frac = frac * 2;
  }
  if (frac < 0) {
    frac = 0;
  }
  if (frac > 1) {
    frac = 1;
  }
  return (uint16_t)(frac * DAC_MAX);
}

// Build the playable form of one envelope (one DAC value per pixel column,
// linearly interpolated between the drawn points) with its current duration,
// and hand it to the engine on core 1.
void publishEnvelope(int env) {
  static EnvelopeShape shape;  // ~0.5 KB, kept off the stack
  const std::map<int, coords> &points = (env == 1) ? envelope1 : envelope2;

  shape.count = 0;
  shape.durationUs = (uint32_t)((env == 1) ? msLen1 : msLen2) * 1000;

  if (!points.empty()) {
    coords last = points.begin()->second;
    for (const auto& pair : points) {
      const coords &next = pair.second;
      for (int xp = last.x; xp < next.x && shape.count < MAX_LEVELS - 1; xp++) {
        float yp = last.y + (float)(next.y - last.y) * (xp - last.x) / (next.x - last.x);
        shape.level[shape.count++] = levelForY(env, yp);
      }
      last = next;
    }
    shape.level[shape.count++] = levelForY(env, last.y);
  }

  publishShape(env, shape);
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
  tft.print(msLen1);
  tft.setCursor(evnF2X, evnF2Y);
  tft.print(msLen2);
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

// Blank the drawing area of one envelope: everything inside the blue outer
// border (drawn at x 2..outerRectEnd.x+1, y 2..outerRectEnd.y+1) and above
// (env 1) or below (env 2) the blue middle bar. fillRect takes width/height,
// not end coordinates.
void clearEnvArea(int env) {
  uint16_t left   = outerRectStart.x + 1;
  uint16_t right  = outerRectEnd.x;             // last column inside the border
  uint16_t top    = (env == 1) ? outerRectStart.y : (uint16_t)(innerRectStart.y + rectH);
  uint16_t bottom = (env == 1) ? (uint16_t)(innerRectStart.y - 1) : outerRectEnd.y;
  tft.fillRect(left, top, right - left + 1, bottom - top + 1, TFT_BLACK);
}

void btnClear1_pressAction(void) {
  if (btnClear1.justPressed()) {
    btnClear1.drawSmoothButton(!btnClear1.getState(), 1, TFT_MAGENTA, "clear ^");
    btnClear1.setPressTime(millis());

    // clear data
    clearEnvArea(1);
    envelope1.clear();
    initEnvelopes();
    publishEnvelope(1);

    btnClear1.drawSmoothButton(!btnClear1.getState(), 1, TFT_MAGENTA, "clear ^");
    btnClear1.setPressTime(millis());
  }
}

void btnClear2_pressAction(void) {
  if (btnClear2.justPressed()) {
    btnClear2.drawSmoothButton(!btnClear2.getState(), 1, TFT_MAGENTA, "clear v");
    btnClear2.setPressTime(millis());

    // clear data
    clearEnvArea(2);
    envelope2.clear();
    initEnvelopes();
    publishEnvelope(2);

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

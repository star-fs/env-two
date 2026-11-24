/*
 Copyright 2023-07-14 by Star Morin (autospkr.org labs)
 Attribution 3.0 United States (CC BY 3.0 US)

 Share — copy and redistribute the material in any medium or format
 Adapt — remix, transform, and build upon the material for any purpose, even commercially.

 This license is acceptable for Free Cultural Works.
 The licensor cannot revoke these freedoms as long as you follow the license terms.

 https://creativecommons.org/licenses/by/3.0/us/legalcode
*/

/*
 env-two: dual envelope generator on a Raspberry Pi Pico (RP2040).

 The work is split across the RP2040's two cores:

   core 0  core0_ui.ino      setup() / loop()
           touchscreen drawing, clear buttons, duration encoders, display.
           Turns each drawn envelope into an EnvelopeShape and publishes it.

   core 1  core1_engine.ino  setup1() / loop1()
           trigger buttons + trigger jacks, mode switches, and two
           independent, non-blocking envelope players driving the MCP4822
           (envelope 1 -> DAC channel A, envelope 2 -> DAC channel B).

 The only data that crosses between the cores is the EnvelopeShape below,
 handed over under a mutex. The UI may republish at any time; a player picks
 up the latest shape at the start of each pass, so edits take effect on the
 next trigger (or the next loop cycle).

 Arduino concatenates the .ino files: this one first, then the others in
 alphabetical order, so everything declared here is visible to both cores.
*/

#include <SPI.h>
#include <Bounce2.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>
#include <RotaryEncoder.h>
#include <map>
#include <pico/mutex.h>

#include "MCP_DAC.h"

#include "Free_Fonts.h"

// ---------------------------------------------------------------------------
// pin map (GPIO numbers)
// ---------------------------------------------------------------------------

// core 0: duration encoders (A, B)
const uint8_t PIN_ENC1_A = 10, PIN_ENC1_B = 11;
const uint8_t PIN_ENC2_A = 12, PIN_ENC2_B = 13;
// (the TFT and its touch controller are on SPI0, configured in
//  Setup_envTwo_RP2040_ILI9341_touch.h)

// core 1: per-envelope inputs              env 1  env 2
const uint8_t PIN_TRIG_BUTTON[2] = {          6,     7 };  // panel button, active LOW
const uint8_t PIN_TRIG_JACK[2]   = {         26,    27 };  // PC817 trigger input, active HIGH
const uint8_t PIN_MODE_LOOP[2]   = {         28,     5 };  // mode switch, active LOW
const uint8_t PIN_MODE_GATE[2]   = {         22,     4 };  // mode switch, active LOW

// core 1: MCP4822 DAC on SPI1
const uint8_t PIN_DAC_SCK = 14, PIN_DAC_MISO = 8, PIN_DAC_MOSI = 15, PIN_DAC_CS = 9;

// DAC channel per envelope (0 = VA, 1 = VB)
const uint8_t DAC_CHANNEL[2] = { 0, 1 };

// ---------------------------------------------------------------------------
// shared settings
// ---------------------------------------------------------------------------

const int minLenMs = 6;          // shortest envelope duration the encoders allow
const uint16_t DAC_MAX = 4095;   // MCP4822 is 12-bit

// ---------------------------------------------------------------------------
// envelope handoff between the cores
// ---------------------------------------------------------------------------

// One envelope, ready to play: the DAC value at every pixel column of the
// drawing, from the first point to the last. The player interpolates between
// neighbouring columns, so the output is the same piecewise-linear line that
// is drawn on screen, stretched over durationUs.
const int MAX_LEVELS = 256;      // the drawing areas are at most ~240 px wide

struct EnvelopeShape {
  uint16_t level[MAX_LEVELS];
  uint16_t count;                // valid entries; fewer than 2 = nothing to play
  uint32_t durationUs;
};

EnvelopeShape publishedShape[2]; // index 0 = envelope 1, 1 = envelope 2
auto_init_mutex(shapeMutex);

// core 0: hand a freshly built shape to the engine
void publishShape(int env, const EnvelopeShape &shape) {
  mutex_enter_blocking(&shapeMutex);
  publishedShape[env - 1] = shape;
  mutex_exit(&shapeMutex);
}

// core 1: take a private copy of the latest shape
void fetchShape(int env, EnvelopeShape &out) {
  mutex_enter_blocking(&shapeMutex);
  out = publishedShape[env - 1];
  mutex_exit(&shapeMutex);
}

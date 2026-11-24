/*
 core 1: envelope engine

 Owns the trigger inputs (panel buttons and trigger jacks), the mode
 switches and the MCP4822. Each envelope has its own Voice, and loop1() just
 keeps updating both. A Voice never waits: on every update it works out from
 micros() where in the envelope it should be and writes that level, so the
 two envelopes run at the same time and their timing does not depend on how
 long a loop iteration takes.

 Modes (3-position switch per envelope):
   trigger  plays the whole envelope once, whatever the input does
   gate     plays once, stops as soon as the input that started it is released
   loop     repeats while that input is held, stops when it is released
 A new trigger always restarts the envelope from the beginning.

 Set ENGINE_DEBUG to 1 to print the measured length of every completed
 one-shot pass on the serial port (115200 baud).
*/

#define ENGINE_DEBUG 1

MCP4822 MCP(255, 255, &SPI1);

struct Voice {
  enum class Mode { Trigger, Gate, Loop };

  int env = 0;                        // 1 or 2
  uint8_t dacChannel = 0;
  uint8_t loopPin = 0, gatePin = 0;
  Bounce2::Button panelButton;
  Bounce2::Button triggerJack;

  EnvelopeShape shape;                // private copy for the pass being played
  bool playing = false;
  Bounce2::Button *source = nullptr;  // the input that started it (gate/loop follow it)
  uint32_t passStartUs = 0;
  int32_t lastWritten = -1;

  void begin(int envNumber) {
    env = envNumber;
    int i = env - 1;
    dacChannel = DAC_CHANNEL[i];
    loopPin = PIN_MODE_LOOP[i];
    gatePin = PIN_MODE_GATE[i];

    panelButton.attach(PIN_TRIG_BUTTON[i], INPUT_PULLUP);
    panelButton.interval(10);
    panelButton.setPressedState(LOW);

    // trigger jack, via the optocoupler: reads as a button that is pressed when HIGH
    triggerJack.attach(PIN_TRIG_JACK[i], INPUT);
    triggerJack.interval(10);
    triggerJack.setPressedState(HIGH);

    pinMode(loopPin, INPUT_PULLUP);
    pinMode(gatePin, INPUT_PULLUP);

    write(0);
  }

  Mode readMode() {
    if (digitalRead(loopPin) == LOW) return Mode::Loop;
    if (digitalRead(gatePin) == LOW) return Mode::Gate;
    return Mode::Trigger;
  }

  void update() {
    panelButton.update();
    triggerJack.update();
    if (panelButton.pressed()) {
      start(&panelButton);
    } else if (triggerJack.pressed()) {
      start(&triggerJack);
    }

    if (!playing) {
      return;
    }

    Mode mode = readMode();

    // gate off
    if (mode != Mode::Trigger && !source->isPressed()) {
      stop();
      return;
    }

    uint32_t elapsed = micros() - passStartUs;
    if (elapsed >= shape.durationUs) {
      if (mode == Mode::Loop) {
        // next cycle, picking up any edits; keep the period exact unless the
        // new duration is already over
        passStartUs += shape.durationUs;
        fetchShape(env, shape);
        if (!playable()) {
          stop();
          return;
        }
        elapsed = micros() - passStartUs;
        if (elapsed >= shape.durationUs) {
          passStartUs = micros();
          elapsed = 0;
        }
      } else {
#if ENGINE_DEBUG
        Serial.printf("env %d pass: %lu us (target %lu us)\n", env,
                      (unsigned long)elapsed, (unsigned long)shape.durationUs);
#endif
        stop();
        return;
      }
    }

    write(levelAt(elapsed));
  }

  void start(Bounce2::Button *input) {
    source = input;
    fetchShape(env, shape);
    passStartUs = micros();
    playing = playable();
  }

  bool playable() {
    return shape.count >= 2 && shape.durationUs > 0;
  }

  void stop() {
    playing = false;
    write(0);
  }

  // DAC value at a point in time, interpolating between pixel columns
  // (position kept with 8 fractional bits)
  uint16_t levelAt(uint32_t elapsedUs) {
    uint32_t span = shape.count - 1;
    uint64_t pos = (uint64_t)elapsedUs * span * 256 / shape.durationUs;
    uint32_t i = pos / 256;
    if (i >= span) {
      return shape.level[span];
    }
    int32_t a = shape.level[i];
    int32_t b = shape.level[i + 1];
    return (uint16_t)(a + (b - a) * (int32_t)(pos % 256) / 256);
  }

  void write(uint16_t value) {
    if (value > DAC_MAX) {
      value = DAC_MAX;           // fastWrite does not mask: higher bits would corrupt the command
    }
    if (value == lastWritten) {
      return;
    }
    if (dacChannel == 0) {
      MCP.fastWriteA(value);
    } else {
      MCP.fastWriteB(value);
    }
    lastWritten = value;
  }
};

Voice voices[2];

void setup1() {
  MCP.setGPIOpins(PIN_DAC_SCK, PIN_DAC_MISO, PIN_DAC_MOSI, PIN_DAC_CS);  // SELECT should match the param of begin()
  MCP.begin(PIN_DAC_CS);
  MCP.setGain(1);

  voices[0].begin(1);
  voices[1].begin(2);
}

void loop1() {
  voices[0].update();
  voices[1].update();
}

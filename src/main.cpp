#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Adafruit_NeoPixel.h>
#include <MsTimer2.h>
#include <ctype.h>
#include <math.h>
#include "DHT.h"

constexpr uint8_t LCD_COLUMNS = 20;
constexpr uint8_t LCD_ROWS = 4;
constexpr uint8_t NUM_PIXELS = 8;

constexpr uint8_t neopixel_pin = 2;
constexpr uint8_t trig_pin = 3;
constexpr uint8_t echo_pin = 4;
constexpr uint8_t dht11_pin = 5;
constexpr uint8_t buzzer_pin = 6;
constexpr uint8_t motor_pin = 7;
constexpr uint8_t ldr_pin = A0;

constexpr uint8_t ROWS = 3;
constexpr uint8_t COLS = 3;

constexpr uint8_t LCD_DEGREE_CHAR = 1;
constexpr uint8_t LCD_BLOCK_CHAR = 2;

constexpr uint32_t dhtReadIntervalMs = 2000;
constexpr uint32_t digitOnMs = 1;
constexpr uint32_t fndTestIntervalMs = 300;
constexpr uint32_t mode1TestingMs = 3000;
constexpr uint32_t mode1ResultMs = 3000;
constexpr uint32_t mode2MoveStepMs = 2000;
constexpr uint32_t mode2ArrivedMs = 1000;
constexpr uint32_t mode2WaitMs = 2000;
constexpr uint32_t mode2OpenMs = 1000;
constexpr uint32_t mode2CompleteMs = 1000;
constexpr uint32_t mode2WarningIntervalMs = 500;
constexpr uint32_t mode3ResultMs = 1500;
constexpr uint32_t mode3LcdFillStepMs = 40;
constexpr uint32_t mode3NeoStepMs = 500;
constexpr uint32_t mode3BuzzerStepMs = 500;
constexpr uint32_t blinkIntervalMs = 500;

constexpr uint16_t BEEP_MODE_SELECT_FREQ = 1000;
constexpr uint32_t BEEP_MODE_SELECT_MS = 100;
constexpr uint16_t BEEP_MODE2_FREQ = 2000;
constexpr uint32_t BEEP_MODE2_MS = 50;
constexpr uint16_t BEEP_MODE3_FREQ = 1000;
constexpr uint32_t BEEP_MODE3_MS = 100;

constexpr uint32_t COLOR_BLUE = 0x0000FF;
constexpr uint32_t COLOR_RED = 0xFF0000;
constexpr uint32_t COLOR_GREEN = 0x00FF00;
constexpr uint32_t COLOR_ORANGE = 0xFFA500;
constexpr uint32_t COLOR_PINK = 0xFF1493;
constexpr uint32_t COLOR_OFF = 0x000000;

constexpr uint8_t seg_pins1[14] = {15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint8_t seg_pins2[14] = {33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46};
constexpr uint8_t digit_pins[4] = {31, 32, 47, 48};

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'}
};

uint8_t rowPins[ROWS] = {8, 9, 10};
uint8_t colPins[COLS] = {11, 12, 13};

constexpr uint16_t SEG_A = 1 << 0;
constexpr uint16_t SEG_B = 1 << 1;
constexpr uint16_t SEG_C = 1 << 2;
constexpr uint16_t SEG_D = 1 << 3;
constexpr uint16_t SEG_E = 1 << 4;
constexpr uint16_t SEG_F = 1 << 5;
constexpr uint16_t SEG_G = 1 << 6;
constexpr uint16_t SEG_H = 1 << 7;
constexpr uint16_t SEG_J = 1 << 8;
constexpr uint16_t SEG_K = 1 << 9;
constexpr uint16_t SEG_L = 1 << 10;
constexpr uint16_t SEG_M = 1 << 11;
constexpr uint16_t SEG_N = 1 << 12;
constexpr uint16_t SEG_P = 1 << 13;

constexpr uint16_t DOOR_CLOSED_MASK = SEG_A | SEG_G | SEG_L | SEG_D;
constexpr uint16_t DOOR_OPEN_MASK = SEG_F | SEG_E | SEG_B | SEG_C;

const uint16_t fndPinTestSequence[14] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, 
                                   SEG_G, SEG_H, SEG_J, SEG_K, SEG_L, SEG_M, SEG_N, SEG_P};
constexpr size_t fndPinTestSequenceLength = sizeof(fndPinTestSequence) / sizeof(fndPinTestSequence[0]);

const uint8_t degreeCharBitmap[8] = {
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};

const uint8_t blockCharBitmap[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

enum AppMode : uint8_t {
  MODE_INITIAL = 0,
  MODE_1 = 1,
  MODE_2 = 2,
  MODE_3 = 3
};

enum Mode1Stage : uint8_t {
  MODE1_STAGE_SCAN = 0,
  MODE1_STAGE_TESTING = 1,
  MODE1_STAGE_RESULT = 2
};

enum Mode2State : uint8_t {
  MODE2_IDLE = 0,
  MODE2_MOVING_UP = 1,
  MODE2_MOVING_DOWN = 2,
  MODE2_ARRIVED = 3,
  MODE2_WAIT_OPEN = 4,
  MODE2_OPEN = 5,
  MODE2_WAIT_CLOSE = 6,
  MODE2_COMPLETE = 7,
  MODE2_WARNING = 8
};

enum Mode3State : uint8_t {
  MODE3_MENU = 0,
  MODE3_LCD_TEST = 1,
  MODE3_LCD_RESULT = 2,
  MODE3_DHT_TEST = 3,
  MODE3_DHT_RESULT = 4,
  MODE3_NEO_TEST = 5,
  MODE3_NEO_RESULT = 6,
  MODE3_KEYPAD_TEST = 7,
  MODE3_KEYPAD_RESULT = 8,
  MODE3_BUZZER_TEST = 9,
  MODE3_BUZZER_RESULT = 10
};

DHT dht(dht11_pin, DHT11);
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
Adafruit_NeoPixel pixels(NUM_PIXELS, neopixel_pin, NEO_GRB + NEO_KHZ800);

AppMode currentMode = MODE_INITIAL;
Mode1Stage mode1Stage = MODE1_STAGE_SCAN;
Mode2State mode2State = MODE2_IDLE;
Mode3State mode3State = MODE3_MENU;

float currentTemperature = NAN;
float currentHumidity = NAN;
uint32_t lastDhtReadMs = 0;

char lcdCache[LCD_ROWS][LCD_COLUMNS + 1] = {{0}};
volatile uint16_t currentDisplayMasks[4] = {0, 0, 0, 0};

bool mode1Paused = false;
uint8_t mode1DigitIndex = 0;
uint8_t mode1SegmentIndex = 0;
uint32_t mode1LastStepMs = 0;
uint32_t mode1StageStartedMs = 0;
uint32_t mode1PausedElapsedMs = 0;

bool mode2Initialized = false;
bool mode2Paused = false;
uint8_t mode2CurrentFloor = 1;
uint8_t mode2TargetFloor = 1;
uint32_t mode2StateStartedMs = 0;
uint32_t mode2PausedElapsedMs = 0;

bool mode3MenuPrinted = false;
char mode3PendingCommand = '\0';
char mode3LastPressedKey = '\0';
uint32_t mode3StateStartedMs = 0;
uint32_t mode3LastStepMs = 0;
uint8_t mode3StepIndex = 0;
uint8_t mode2LastWarningPhase = 0;
int mode3StartTemperature = 0;
int mode3StartHumidity = 0;

void resetLcdCache()
{
  for (uint8_t row = 0; row < LCD_ROWS; ++row) {
    lcdCache[row][0] = '\0';
  }
}

void clearLcdDirect()
{
  lcd.clear();
  resetLcdCache();
}

void writeLcdLine(uint8_t row, const char *text)
{
  char padded[LCD_COLUMNS + 1];
  for (uint8_t i = 0; i < LCD_COLUMNS; ++i) {
    padded[i] = ' ';
  }
  padded[LCD_COLUMNS] = '\0';

  if (text != nullptr) {
    for (uint8_t i = 0; i < LCD_COLUMNS && text[i] != '\0'; ++i) {
      padded[i] = text[i];
    }
  }

  if (strncmp(lcdCache[row], padded, LCD_COLUMNS) != 0) {
    lcd.setCursor(0, row);
    lcd.print(padded);
    memcpy(lcdCache[row], padded, sizeof(padded));
  }
}

void showLcdLines(const char *line0, const char *line1, const char *line2, const char *line3)
{
  writeLcdLine(0, line0);
  writeLcdLine(1, line1);
  writeLcdLine(2, line2);
  writeLcdLine(3, line3);
}

int roundSensorValue(float value)
{
  if (isnan(value)) {
    return 0;
  }

  return static_cast<int>(value + (value >= 0.0f ? 0.5f : -0.5f));
}

void buildTemperatureHumidityLine(char *out)
{
  snprintf(
    out,
    LCD_COLUMNS + 1,
    "TEMP:%2d%cC HUMI:%2d%%",
    roundSensorValue(currentTemperature),
    static_cast<char>(LCD_DEGREE_CHAR),
    roundSensorValue(currentHumidity)
  );
}

uint16_t charToMask(char rawChar)
{
  char c = static_cast<char>(toupper(static_cast<unsigned char>(rawChar)));

  switch (c) {
    case '0':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case '1':
      return SEG_B | SEG_C;
    case '2':
      return SEG_A | SEG_B | SEG_D | SEG_E | SEG_N | SEG_J;
    case '3':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_N | SEG_J;
    case '4':
      return SEG_B | SEG_C | SEG_F | SEG_N | SEG_J;
    case '5':
      return SEG_A | SEG_C | SEG_D | SEG_F | SEG_N | SEG_J;
    case '6':
      return SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_N | SEG_J;
    case '7':
      return SEG_A | SEG_B | SEG_C;
    case '8':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_N | SEG_J;
    case '9':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_N | SEG_J;
    case 'A':
      return SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_N | SEG_J;
    case 'B':
      return SEG_C | SEG_D | SEG_E | SEG_F | SEG_J | SEG_L | SEG_N;
    case 'C':
      return SEG_A | SEG_D | SEG_E | SEG_F;
    case 'D':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'E':
      return SEG_A | SEG_D | SEG_E | SEG_F | SEG_N | SEG_J;
    case 'F':
      return SEG_A | SEG_E | SEG_F | SEG_N;
    case 'G':
      return SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_J | SEG_N;
    case 'H':
      return SEG_B | SEG_C | SEG_E | SEG_F | SEG_N | SEG_J;
    case 'I':
      return SEG_A | SEG_D | SEG_G | SEG_L;
    case 'J':
      return SEG_B | SEG_C | SEG_D | SEG_E;
    case 'K':
      return SEG_F | SEG_E | SEG_N | SEG_H | SEG_K;
    case 'L':
      return SEG_D | SEG_E | SEG_F;
    case 'M':
      return SEG_B | SEG_C | SEG_E | SEG_F | SEG_P | SEG_H;
    case 'N':
      return SEG_B | SEG_C | SEG_E | SEG_F | SEG_P | SEG_K;
    case 'O':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'P':
      return SEG_A | SEG_B | SEG_E | SEG_F | SEG_N | SEG_J;
    case 'R':
      return SEG_A | SEG_B | SEG_E | SEG_F | SEG_N | SEG_J | SEG_K;
    case 'S':
      return SEG_A | SEG_F | SEG_N | SEG_J | SEG_C | SEG_D;
    case 'T':
      return SEG_A | SEG_G | SEG_L;
    case 'U':
      return SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'V':
      return SEG_E | SEG_F | SEG_H | SEG_M;
    case 'W':
      return SEG_B | SEG_C | SEG_E | SEG_F | SEG_P | SEG_M;
    case 'Y':
      return SEG_P | SEG_H | SEG_L;
    case '-':
      return SEG_N | SEG_J;
    default:
      return 0;
  }
}

void clearDisplayMasks()
{
  for (uint8_t i = 0; i < 4; ++i) {
    currentDisplayMasks[i] = 0;
  }
}

void setFndText(const char *text)
{
  for (uint8_t i = 0; i < 4; ++i) {
    char c = ' ';
    if (text != nullptr && text[i] != '\0') {
      c = text[i];
    }
    currentDisplayMasks[i] = charToMask(c);
  }
}

void writeDigitSegments(uint8_t digit, uint16_t mask)
{
  const uint8_t *pins = (digit < 2) ? seg_pins1 : seg_pins2;
  for (uint8_t i = 0; i < 14; ++i) {
    digitalWrite(pins[i], ((mask>>i) & 0x1)? LOW : HIGH);
  }
}

void displayISR()
{
  static uint8_t phase = 0;
  digitalWrite(digit_pins[phase], LOW);
  digitalWrite(digit_pins[phase + 2], LOW);
  phase ^= 1;
  writeDigitSegments(phase, (uint16_t)currentDisplayMasks[phase]);
  writeDigitSegments(phase + 2, (uint16_t)currentDisplayMasks[phase + 2]);
  digitalWrite(digit_pins[phase], HIGH);
  digitalWrite(digit_pins[phase + 2], HIGH);
}

void setAllPixels(uint32_t color)
{
  for (uint8_t i = 0; i < NUM_PIXELS; ++i) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void clearPixels()
{
  setAllPixels(COLOR_OFF);
}

void showTemperatureHumidityPixels()
{
  uint32_t tempColor = (roundSensorValue(currentTemperature) >= 24) ? COLOR_RED : COLOR_BLUE;
  uint32_t humidityColor = (roundSensorValue(currentHumidity) > 50) ? COLOR_ORANGE : COLOR_GREEN;

  for (uint8_t i = 0; i < 4; ++i) {
    pixels.setPixelColor(i, tempColor);
  }
  for (uint8_t i = 4; i < NUM_PIXELS; ++i) {
    pixels.setPixelColor(i, humidityColor);
  }
  pixels.show();
}

void updateSensorValues(uint32_t now, bool force)
{
  if (!force && now - lastDhtReadMs < dhtReadIntervalMs) {
    return;
  }

  float humidity = dht.readHumidity(force);
  float temperature = dht.readTemperature(false, force);

  if (!isnan(humidity)) {
    currentHumidity = humidity;
  }
  if (!isnan(temperature)) {
    currentTemperature = temperature;
  }

  lastDhtReadMs = now;
}

void enterInitialScreen()
{
  currentMode = MODE_INITIAL;
  showLcdLines(
    " ELEVATOR SYSTEM",
    "  CIRCUIT DESIGN",
    "  & PROGRAMMING",
    "   2026.06.13"
  );
}

void resetMode1State()
{
  mode1Stage = MODE1_STAGE_SCAN;
  mode1DigitIndex = 0;
  mode1SegmentIndex = 0;
  mode1LastStepMs = millis();
  mode1StageStartedMs = millis();
  mode1PausedElapsedMs = 0;
  mode1Paused = false;
}

bool isMode1LastStep()
{
  return mode1DigitIndex == 3 && mode1SegmentIndex == (fndPinTestSequenceLength - 1);
}

uint8_t mode1CurrentPortNumber()
{
  const uint8_t *pins = (mode1DigitIndex < 2) ? seg_pins1 : seg_pins2;
  return pins[mode1SegmentIndex];
}

void beginMode1(uint32_t now)
{
  currentMode = MODE_1;
  if (mode1Paused) {
    mode1Paused = false;
    if (mode1Stage == MODE1_STAGE_SCAN) {
      mode1LastStepMs = now - mode1PausedElapsedMs;
    } else {
      mode1StageStartedMs = now - mode1PausedElapsedMs;
    }
    return;
  }

  resetMode1State();
  mode1LastStepMs = now;
  mode1StageStartedMs = now;
}

void pauseMode1(uint32_t now)
{
  mode1Paused = true;
  if (mode1Stage == MODE1_STAGE_SCAN) {
    mode1PausedElapsedMs = now - mode1LastStepMs;
  } else {
    mode1PausedElapsedMs = now - mode1StageStartedMs;
  }
  enterInitialScreen();
}

void showMode1ActiveScreen()
{
  char line1[LCD_COLUMNS + 1];
  char line2[LCD_COLUMNS + 1];

  snprintf(line1, sizeof(line1), "CHECK DIGIT: F%u", mode1DigitIndex + 1);
  snprintf(line2, sizeof(line2), "CHECK PORT: IO%02u", mode1CurrentPortNumber());

  showLcdLines("PIN TEST: ACTIVE", line1, line2, "");
  clearDisplayMasks();
  currentDisplayMasks[mode1DigitIndex] = fndPinTestSequence[mode1SegmentIndex];
}

void showMode1TestingScreen()
{
  showLcdLines("PIN TESTING......", "", "", "");
  setFndText("TEST");
}

void showMode1ResultScreen()
{
  showLcdLines("PIN TEST COMPLETED", "STATUS: DONE", "RESULT: OK", "");
  setFndText(" OK ");
}

void updateMode1(uint32_t now)
{
  switch (mode1Stage) {
    case MODE1_STAGE_SCAN:
      showMode1ActiveScreen();  // LCD 출력만 담당하도록 변경
      if (now - mode1LastStepMs >= fndTestIntervalMs) {
        mode1LastStepMs = now;
        if (isMode1LastStep()) {
          mode1Stage = MODE1_STAGE_TESTING;
          mode1StageStartedMs = now;
        } else {
          ++mode1SegmentIndex;
          if (mode1SegmentIndex >= fndPinTestSequenceLength) {
            mode1SegmentIndex = 0;
            ++mode1DigitIndex;
          }
        }
        // step이 바뀔 때만 mask 갱신
        clearDisplayMasks();
        currentDisplayMasks[mode1DigitIndex] = fndPinTestSequence[mode1SegmentIndex];
      }
      break;

    case MODE1_STAGE_TESTING:
      showMode1TestingScreen();
      if (now - mode1StageStartedMs >= mode1TestingMs) {
        mode1Stage = MODE1_STAGE_RESULT;
        mode1StageStartedMs = now;
      }
      break;

    case MODE1_STAGE_RESULT:
      showMode1ResultScreen();
      if (now - mode1StageStartedMs >= mode1ResultMs) {
        resetMode1State();
        enterInitialScreen();
      }
      break;
  }
}

const char *mode2ModeLabel()
{
  switch (mode2State) {
    case MODE2_MOVING_UP:
      return "UP";
    case MODE2_MOVING_DOWN:
      return "DOWN";
    default:
      return "STOP";
  }
}

const char *mode2DoorLabel()
{
  switch (mode2State) {
    case MODE2_WAIT_OPEN:
    case MODE2_WAIT_CLOSE:
      return "WAIT";
    case MODE2_OPEN:
      return "OPEN";
    default:
      return "CLOSE";
  }
}

void setMode2DoorMasks(bool showDoor, bool doorOpen)
{
  currentDisplayMasks[0] = showDoor ? (doorOpen ? DOOR_OPEN_MASK : DOOR_CLOSED_MASK) : 0;
  currentDisplayMasks[1] = showDoor ? (doorOpen ? DOOR_OPEN_MASK : DOOR_CLOSED_MASK) : 0;
  currentDisplayMasks[2] = charToMask(static_cast<char>('0' + mode2CurrentFloor));
  currentDisplayMasks[3] = charToMask('F');
}

void updateMode2Fnd(uint32_t now)
{
  if (mode2State == MODE2_MOVING_UP) {
    currentDisplayMasks[0] = charToMask('U');
    currentDisplayMasks[1] = charToMask('P');
    currentDisplayMasks[2] = charToMask(static_cast<char>('0' + mode2CurrentFloor));
    currentDisplayMasks[3] = charToMask('F');
    return;
  }

  if (mode2State == MODE2_MOVING_DOWN) {
    currentDisplayMasks[0] = charToMask('D');
    currentDisplayMasks[1] = charToMask('N');
    currentDisplayMasks[2] = charToMask(static_cast<char>('0' + mode2CurrentFloor));
    currentDisplayMasks[3] = charToMask('F');
    return;
  }

  bool showDoor = true;
  bool doorOpen = false;

  if (mode2State == MODE2_WAIT_OPEN) {
    showDoor = ((now / blinkIntervalMs) % 2U) == 0U;
    doorOpen = false;
  } else if (mode2State == MODE2_WAIT_CLOSE) {
    showDoor = ((now / blinkIntervalMs) % 2U) == 0U;
    doorOpen = true;
  } else if (mode2State == MODE2_OPEN) {
    doorOpen = true;
  } else if (mode2State == MODE2_WARNING) {
    showDoor = (((now - mode2StateStartedMs) / mode2WarningIntervalMs) % 2U) == 0U;
  }

  if (mode2State == MODE2_WARNING && !showDoor) {
    clearDisplayMasks();
    return;
  }

  setMode2DoorMasks(showDoor, doorOpen);
}

void showMode2Lcd()
{
  char line1[LCD_COLUMNS + 1];
  char line2[LCD_COLUMNS + 1];
  char line3[LCD_COLUMNS + 1];

  snprintf(line1, sizeof(line1), "ELEV:%uF", mode2CurrentFloor);
  snprintf(line2, sizeof(line2), "MODE:%-4s DOOR:%-5s", mode2ModeLabel(), mode2DoorLabel());
  buildTemperatureHumidityLine(line3);

  showLcdLines("    ELEV SYSTEM", line1, line2, line3);
}

void updateMode2Pixels(uint32_t now)
{
  if (mode2State == MODE2_WAIT_OPEN || mode2State == MODE2_WAIT_CLOSE) {
    if (((now / blinkIntervalMs) % 2U) == 0U) {
      setAllPixels(COLOR_ORANGE);
    } else {
      clearPixels();
    }
    return;
  }

  if (mode2State == MODE2_OPEN) {
    setAllPixels(COLOR_GREEN);
    return;
  }

  setAllPixels(COLOR_PINK);
}

void beginMode2(uint32_t now)
{
  currentMode = MODE_2;
  if (!mode2Initialized) {
    mode2Initialized = true;
    mode2State = MODE2_IDLE;
    mode2CurrentFloor = 1;
    mode2TargetFloor = 1;
    mode2StateStartedMs = now;
    mode2Paused = false;
    return;
  }

  if (mode2Paused) {
    mode2Paused = false;
    mode2StateStartedMs = now - mode2PausedElapsedMs;
  }
}

void pauseMode2(uint32_t now)
{
  mode2Paused = true;
  mode2PausedElapsedMs = now - mode2StateStartedMs;
  enterInitialScreen();
}

bool mode2CanAcceptCall()
{
  return mode2State == MODE2_IDLE || mode2State == MODE2_COMPLETE || mode2State == MODE2_WARNING;
}

uint8_t buttonToFloor(char key)
{
  return static_cast<uint8_t>((key - '4') + 1);
}

void startMode2Call(uint8_t targetFloor, uint32_t now)
{
  if (!mode2CanAcceptCall()) {
    return;
  }

  mode2TargetFloor = targetFloor;
  if (targetFloor == mode2CurrentFloor) {
    mode2State = MODE2_WARNING;
    mode2StateStartedMs = now;
    mode2LastWarningPhase = 255;
    return;
  }

  mode2State = (targetFloor > mode2CurrentFloor) ? MODE2_MOVING_UP : MODE2_MOVING_DOWN;
  mode2StateStartedMs = now;
}

void updateMode2(uint32_t now)
{
  if (mode2State == MODE2_WARNING) {
    uint8_t currentPhase = static_cast<uint8_t>((now - mode2StateStartedMs) / mode2WarningIntervalMs);
    if (currentPhase != mode2LastWarningPhase) {
      mode2LastWarningPhase = currentPhase;
      if (currentPhase < 4 && (currentPhase % 2U) == 0U) {
        tone(buzzer_pin, BEEP_MODE2_FREQ, 100);
      }
    }

    if (currentPhase >= 4) {
      mode2State = MODE2_IDLE;
      mode2StateStartedMs = now;
      mode2LastWarningPhase = 0;
    }
  } else if (mode2State == MODE2_MOVING_UP || mode2State == MODE2_MOVING_DOWN) {
    if (now - mode2StateStartedMs >= mode2MoveStepMs) {
      mode2StateStartedMs += mode2MoveStepMs;
      if (mode2State == MODE2_MOVING_UP) {
        ++mode2CurrentFloor;
      } else {
        --mode2CurrentFloor;
      }

      if (mode2CurrentFloor == mode2TargetFloor) {
        mode2State = MODE2_ARRIVED;
        mode2StateStartedMs = now;
      }
    }
  } else if (mode2State == MODE2_ARRIVED && now - mode2StateStartedMs >= mode2ArrivedMs) {
    mode2State = MODE2_WAIT_OPEN;
    mode2StateStartedMs = now;
  } else if (mode2State == MODE2_WAIT_OPEN && now - mode2StateStartedMs >= mode2WaitMs) {
    mode2State = MODE2_OPEN;
    mode2StateStartedMs = now;
  } else if (mode2State == MODE2_OPEN && now - mode2StateStartedMs >= mode2OpenMs) {
    mode2State = MODE2_WAIT_CLOSE;
    mode2StateStartedMs = now;
  } else if (mode2State == MODE2_WAIT_CLOSE && now - mode2StateStartedMs >= mode2WaitMs) {
    mode2State = MODE2_COMPLETE;
    mode2StateStartedMs = now;
  } else if (mode2State == MODE2_COMPLETE && now - mode2StateStartedMs >= mode2CompleteMs) {
    mode2State = MODE2_IDLE;
    mode2TargetFloor = mode2CurrentFloor;
    mode2StateStartedMs = now;
  }

  showMode2Lcd();
  updateMode2Fnd(now);
}

void printMode3MenuToSerial()
{
  Serial.println(F("[ 관리자 모드 ]"));
  Serial.println();
  Serial.println(F("> 장치 리스트 (DEVICE LIST):"));
  Serial.println(F("  1. LCD 전체 점등 테스트"));
  Serial.println(F("  2. DHT11 온습도 동적 감지"));
  Serial.println(F("  3. NeoPixel RGB 순자 점등"));
  Serial.println(F("  4. 키패드 입력 스캔 테스트"));
  Serial.println(F("  5. 부저 주파수 출력 테스트"));
  Serial.println(F("모드 탈출은 키 '9'를 사용하십시오."));
  Serial.println(F("------------------------------------------"));
  Serial.print(F("명령어 입력 : [  ]"));
}

void returnToMode3Menu()
{
  mode3State = MODE3_MENU;
  mode3MenuPrinted = false;
  mode3PendingCommand = '\0';
  mode3LastPressedKey = '\0';
  mode3StepIndex = 0;
  mode3StateStartedMs = millis();
  mode3LastStepMs = millis();
  noTone(buzzer_pin);
}

void exitMode3ToInitial()
{
  returnToMode3Menu();
  enterInitialScreen();
}

void beginMode3(uint32_t now)
{
  currentMode = MODE_3;
  returnToMode3Menu();
  mode3StateStartedMs = now;
  mode3LastStepMs = now;
}

void showMode3Menu()
{
  char line3[LCD_COLUMNS + 1];
  snprintf(line3, sizeof(line3), "SELECT NUMBER:[%c]", mode3PendingCommand == '\0' ? ' ' : mode3PendingCommand);

  showLcdLines(
    "    DEVICE TESTING",
    "1:LCD 2:DHT 3:NEO",
    "4:KEY 5:BUZ 9:EXIT",
    line3
  );
  setFndText("ADMI");

  if (!mode3MenuPrinted) {
    printMode3MenuToSerial();
    mode3MenuPrinted = true;
  }
}

void startMode3LcdTest(uint32_t now)
{
  mode3State = MODE3_LCD_TEST;
  mode3StateStartedMs = now;
  mode3LastStepMs = now - mode3LcdFillStepMs;
  mode3StepIndex = 0;
  clearLcdDirect();
}

void startMode3DhtTest(uint32_t now)
{
  updateSensorValues(now, true);
  mode3State = MODE3_DHT_TEST;
  mode3StateStartedMs = now;
  mode3LastStepMs = now;
  mode3StartTemperature = roundSensorValue(currentTemperature);
  mode3StartHumidity = roundSensorValue(currentHumidity);
  Serial.print(F("진행 시간 : 0s | T:"));
  Serial.print(currentTemperature, 1);
  Serial.print(F(" H:"));
  Serial.print(currentHumidity, 1);
}

void startMode3NeoTest(uint32_t now)
{
  mode3State = MODE3_NEO_TEST;
  mode3StateStartedMs = now;
  mode3LastStepMs = now - mode3NeoStepMs;
  mode3StepIndex = 0;
}

void startMode3KeypadTest(uint32_t now)
{
  mode3State = MODE3_KEYPAD_TEST;
  mode3StateStartedMs = now;
  mode3LastPressedKey = '\0';
}

void startMode3BuzzerTest(uint32_t now)
{
  mode3State = MODE3_BUZZER_TEST;
  mode3StateStartedMs = now;
  mode3LastStepMs = now - mode3BuzzerStepMs;
  mode3StepIndex = 0;
}

void executeMode3Command(char command, uint32_t now)
{
  Serial.println();

  switch (command) {
    case '1':
      startMode3LcdTest(now);
      break;
    case '2':
      startMode3DhtTest(now);
      break;
    case '3':
      startMode3NeoTest(now);
      break;
    case '4':
      startMode3KeypadTest(now);
      break;
    case '5':
      startMode3BuzzerTest(now);
      break;
    case '9':
      exitMode3ToInitial();
      break;
    default:
      mode3MenuPrinted = false;
      break;
  }

  mode3PendingCommand = '\0';
}

void redrawMode3InputLine()
{
  Serial.print(F("\r명령어 입력 : [ "));
  if (mode3PendingCommand != '\0') {
    Serial.print(mode3PendingCommand);
  } else {
    Serial.print(' ');
  }
  Serial.print(F(" ]"));
}

void handleMode3Serial(uint32_t now)
{
  while (Serial.available() > 0) {
    char received = static_cast<char>(Serial.read());

    if (currentMode != MODE_3 || mode3State != MODE3_MENU) {
      continue;
    }

    if (received == '\r' || received == '\n') {
      if (mode3PendingCommand != '\0') {
        Serial.println();
        char cmd = mode3PendingCommand;
        mode3PendingCommand = '\0';
        executeMode3Command(cmd, now);
      }
      continue;
    }

    if (received == 8 || received == 127) {
      mode3PendingCommand = '\0';
      redrawMode3InputLine();
      continue;
    }

    if (received == '1' || received == '2' || received == '3' ||
        received == '4' || received == '5' || received == '9') {
      mode3PendingCommand = received;
      redrawMode3InputLine();
    }
  }
}

void showMode3ResultScreen(const char *line1, const char *line2)
{
  showLcdLines("", line1, line2, "");
  setFndText("ADMI");
}

void updateMode3(uint32_t now)
{
  static const uint16_t buzzerTestFrequencies[4] = {262, 330, 392, 523};
  static const uint32_t neoColors[3] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};

  switch (mode3State) {
    case MODE3_MENU:
      showMode3Menu();
      break;

    case MODE3_LCD_TEST:
      setFndText("ADMI");
      if (now - mode3LastStepMs >= mode3LcdFillStepMs) {
        mode3LastStepMs = now;
        uint8_t row = mode3StepIndex / LCD_COLUMNS;
        uint8_t col = mode3StepIndex % LCD_COLUMNS;
        lcd.setCursor(col, row);
        lcd.write(LCD_BLOCK_CHAR);
        ++mode3StepIndex;

        if (mode3StepIndex >= LCD_COLUMNS * LCD_ROWS) {
          mode3State = MODE3_LCD_RESULT;
          mode3StateStartedMs = now;
          clearLcdDirect();
        }
      }
      break;

    case MODE3_LCD_RESULT:
      showMode3ResultScreen("    LCD TEST OK", "   [RESULT: PASS]");
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;

    case MODE3_DHT_TEST: {
      char line2[LCD_COLUMNS + 1];
      buildTemperatureHumidityLine(line2);
      showLcdLines("DHT11 DYNAMIC TEST", "WAIT FOR CHANGE.....", line2, "");
      setFndText("ADMI");

      uint32_t elapsedSec = (now - mode3StateStartedMs) / 1000;
      if (now - mode3LastStepMs >= 1000) {
        mode3LastStepMs = now;
        Serial.print(F("\r진행 시간 : "));
        Serial.print(elapsedSec);
        Serial.print(F("s | T:"));
        Serial.print(currentTemperature, 1);
        Serial.print(F(" H:"));
        Serial.print(currentHumidity, 1);
        Serial.print(F("  "));
      }

      {
        int currentTemp = roundSensorValue(currentTemperature);
        int currentHumi = roundSensorValue(currentHumidity);
        if (currentTemp != mode3StartTemperature || currentHumi != mode3StartHumidity) {
          Serial.println();
          Serial.print(F("진행 시간 : "));
          Serial.print(elapsedSec);
          Serial.print(F("s | T:"));
          Serial.print(currentTemperature, 1);
          Serial.print(F(" H:"));
          Serial.print(currentHumidity, 1);
          Serial.println(F(" [변경 감지]"));
          mode3State = MODE3_DHT_RESULT;
          mode3StateStartedMs = now;
        }
      }
      break;
    }

    case MODE3_DHT_RESULT:
      showMode3ResultScreen("   DHT11 TEST OK", "   [RESULT: PASS]");
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;

    case MODE3_NEO_TEST:
      showLcdLines("NEO PIXEL TESTING..", "", "", "");
      setFndText("ADMI");
      if (now - mode3LastStepMs >= mode3NeoStepMs) {
        mode3LastStepMs = now;
        uint8_t colorIndex = mode3StepIndex / NUM_PIXELS;
        uint8_t pixelIndex = mode3StepIndex % NUM_PIXELS;
        clearPixels();
        if (colorIndex < 3) {
          pixels.setPixelColor(pixelIndex, neoColors[colorIndex]);
          pixels.show();
          ++mode3StepIndex;
        }
        if (mode3StepIndex >= NUM_PIXELS * 3) {
          clearPixels();
          mode3State = MODE3_NEO_RESULT;
          mode3StateStartedMs = now;
        }
      }
      break;

    case MODE3_NEO_RESULT:
      showMode3ResultScreen(" NEO PIXEL TEST OK", " [RESULT: ALL PASS]");
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;

    case MODE3_KEYPAD_TEST: {
      char line3[LCD_COLUMNS + 1];
      if (mode3LastPressedKey >= '1' && mode3LastPressedKey <= '8') {
        snprintf(line3, sizeof(line3), "PRESSED: SW%c", mode3LastPressedKey);
      } else {
        snprintf(line3, sizeof(line3), "PRESSED: --");
      }
      showLcdLines("KEYPAD TESTING...", "PUSH ANY KEY (9:END)", "", line3);
      setFndText("ADMI");
      break;
    }

    case MODE3_KEYPAD_RESULT:
      showMode3ResultScreen(" KEYPAD TEST OK", " [RESULT: ALL PASS]");
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;

    case MODE3_BUZZER_TEST: {
      char line1[LCD_COLUMNS + 1];
      showLcdLines("BUZZER TESTING...", "", "", "");
      setFndText("ADMI");

      if (mode3StepIndex < 4) {
        snprintf(line1, sizeof(line1), "[FREQ:%3uHz]", buzzerTestFrequencies[mode3StepIndex]);
        writeLcdLine(1, line1);
      }

      if (mode3StepIndex < 4 && now - mode3LastStepMs >= mode3BuzzerStepMs) {
        mode3LastStepMs = now;
        tone(buzzer_pin, buzzerTestFrequencies[mode3StepIndex], mode3BuzzerStepMs - 20);
        ++mode3StepIndex;
      }

      if (mode3StepIndex >= 4 && now - mode3LastStepMs >= mode3BuzzerStepMs) {
        noTone(buzzer_pin);
        mode3State = MODE3_BUZZER_RESULT;
        mode3StateStartedMs = now;
      }
      break;
    }

    case MODE3_BUZZER_RESULT:
      showMode3ResultScreen("   BUZZER TEST OK", "   [RESULT: PASS]");
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;
  }
}

void handleButtonPress(char key, uint32_t now)
{
  switch (currentMode) {
    case MODE_INITIAL:
      if (key == '1') {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        beginMode1(now);
      } else if (key == '2') {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        beginMode2(now);
      } else if (key == '3') {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        beginMode3(now);
      }
      break;

    case MODE_1:
      if (key == '9') {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        pauseMode1(now);
      }
      break;

    case MODE_2:
      if (key >= '4' && key <= '8') {
        tone(buzzer_pin, BEEP_MODE2_FREQ, BEEP_MODE2_MS);
        startMode2Call(buttonToFloor(key), now);
      } else if (key == '9') {
        pauseMode2(now);
      }
      break;

    case MODE_3:
      if (mode3State == MODE3_KEYPAD_TEST) {
        if (key >= '1' && key <= '8') {
          tone(buzzer_pin, BEEP_MODE3_FREQ, BEEP_MODE3_MS);
          mode3LastPressedKey = key;
        } else if (key == '9') {
          mode3State = MODE3_KEYPAD_RESULT;
          mode3StateStartedMs = now;
        }
      } else if (key == '9') {
        exitMode3ToInitial();
      }
      break;
  }
}

void updateInitialMode()
{
  showLcdLines(
    " ELEVATOR SYSTEM",
    "  CIRCUIT DESIGN",
    "  & PROGRAMMING",
    "   2026.06.13"
  );
  setFndText("ELEV");
}

void updateNeoPixelsForCurrentMode(uint32_t now)
{
  if (currentMode == MODE_2) {
    updateMode2Pixels(now);
    return;
  }

  if (currentMode == MODE_3 && mode3State == MODE3_NEO_TEST) {
    return;
  }

  showTemperatureHumidityPixels();
}

void setup()
{
  Serial.begin(9600);

  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(motor_pin, OUTPUT);
  pinMode(ldr_pin, INPUT);
  pinMode(buzzer_pin, OUTPUT);

  dht.begin();

  lcd.begin();
  lcd.backlight();
  lcd.createChar(LCD_DEGREE_CHAR, const_cast<uint8_t *>(degreeCharBitmap));
  lcd.createChar(LCD_BLOCK_CHAR, const_cast<uint8_t *>(blockCharBitmap));
  clearLcdDirect();

  for (uint8_t i = 0; i < 14; ++i) {
    pinMode(seg_pins1[i], OUTPUT);
    digitalWrite(seg_pins1[i], HIGH);
    pinMode(seg_pins2[i], OUTPUT);
    digitalWrite(seg_pins2[i], HIGH);
  }

  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(digit_pins[i], OUTPUT);
    digitalWrite(digit_pins[i], LOW);
  }

  pixels.begin();
  clearPixels();

  tone(buzzer_pin, 440, 500);
  delay(500);
  noTone(buzzer_pin);

  updateSensorValues(millis(), true);
  resetMode1State();
  enterInitialScreen();

  MsTimer2::set(digitOnMs, displayISR);
  MsTimer2::start();
}

void loop()
{
  uint32_t now = millis();

  updateSensorValues(now, false);
  handleMode3Serial(now);

  char key = customKeypad.getKey();
  if (key != NO_KEY) {
    handleButtonPress(key, now);
  }

  switch (currentMode) {
    case MODE_INITIAL:
      updateInitialMode();
      break;
    case MODE_1:
      updateMode1(now);
      break;
    case MODE_2:
      updateMode2(now);
      break;
    case MODE_3:
      updateMode3(now);
      break;
  }

  updateNeoPixelsForCurrentMode(now);
}

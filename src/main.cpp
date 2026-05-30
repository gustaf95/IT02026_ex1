#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Adafruit_NeoPixel.h>
#include <MsTimer2.h>
#include "DHT.h"

uint8_t trig_pin = 3;
uint8_t echo_pin = 4;
uint8_t motor_pin = 7;
uint8_t ldr_pin = A0;


#define MODE_INITIAL  0
#define MODE_1        1
#define MODE_2        2
#define MODE_3        3

#define MODE1_STAGE_SCAN     0
#define MODE1_STAGE_TESTING  1
#define MODE1_STAGE_RESULT   2

#define MODE2_IDLE         0
#define MODE2_MOVING_UP    1
#define MODE2_MOVING_DOWN  2
#define MODE2_ARRIVED      3
#define MODE2_WAIT_OPEN    4
#define MODE2_OPEN         5
#define MODE2_WAIT_CLOSE   6
#define MODE2_COMPLETE     7
#define MODE2_WARNING      8

#define MODE3_MENU          0
#define MODE3_LCD_TEST      1
#define MODE3_LCD_RESULT    2
#define MODE3_DHT_TEST      3
#define MODE3_DHT_RESULT    4
#define MODE3_NEO_TEST      5
#define MODE3_NEO_RESULT    6
#define MODE3_KEYPAD_TEST   7
#define MODE3_KEYPAD_RESULT 8
#define MODE3_BUZZER_TEST   9
#define MODE3_BUZZER_RESULT 10

uint8_t currentMode = MODE_INITIAL;
uint8_t mode1Stage  = MODE1_STAGE_SCAN;
uint8_t mode2State  = MODE2_IDLE;
uint8_t mode3State  = MODE3_MENU;

int currentTemperature = 27;
int currentHumidity = 80;
uint32_t lastDhtReadMs = 0;

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

// ─── LCD ───────────────────────────────────────────────────────────────────
uint8_t LCD_DEGREE_CHAR = 1;
uint8_t LCD_BLOCK_CHAR = 2;

LiquidCrystal_I2C lcd(0x27, 20, 4);
char lcdCache[4][20 + 1] = {{0,},};
uint8_t degreeCharBitmap[8] = {
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};

uint8_t blockCharBitmap[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

void resetLcdCache()
{
  for (uint8_t row = 0; row < 4; row++) {
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
  if (strcmp(lcdCache[row], text) != 0) {
    lcd.setCursor(0, row);
    lcd.print("                    ");
    lcd.setCursor(0, row);
    lcd.print(text);
    strcpy(lcdCache[row], text);
  }
}

void showLcdLines(const char *line0, const char *line1, const char *line2, const char *line3)
{
  writeLcdLine(0, line0);
  writeLcdLine(1, line1);
  writeLcdLine(2, line2);
  writeLcdLine(3, line3);
}

// ─── DHT / 센서 ────────────────────────────────────────────────────────────
uint8_t dht11_pin = 5;
uint32_t dhtReadIntervalMs = 2000;

DHT dht(dht11_pin, DHT11);

void updateSensorValues(uint32_t now)
{
  if (now - lastDhtReadMs > dhtReadIntervalMs) {  
    currentHumidity = dht.readHumidity();
    currentTemperature = dht.readTemperature();
    lastDhtReadMs = now;
  }
}

void buildTemperatureHumidityLine(char *out)
{
  sprintf(out, "TEMP:%2d%cC HUMI:%2d%%", currentTemperature, LCD_DEGREE_CHAR, currentHumidity);
}

// ─── FND ───────────────────────────────────────────────────────────────────
uint32_t digitOnMs = 1;
uint32_t fndTestIntervalMs = 300;

uint8_t seg_pins1[14] = {15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 27, 28, 29, 30};
uint8_t seg_pins2[14] = {33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46};
uint8_t digit_pins[4] = {31, 32, 47, 48};

uint16_t SEG_A = 1 << 0;
uint16_t SEG_B = 1 << 1;
uint16_t SEG_C = 1 << 2;
uint16_t SEG_D = 1 << 3;
uint16_t SEG_E = 1 << 4;
uint16_t SEG_F = 1 << 5;
uint16_t SEG_G = 1 << 6;
uint16_t SEG_H = 1 << 7;
uint16_t SEG_J = 1 << 8;
uint16_t SEG_K = 1 << 9;
uint16_t SEG_L = 1 << 10;
uint16_t SEG_M = 1 << 11;
uint16_t SEG_N = 1 << 12;
uint16_t SEG_P = 1 << 13;


const uint16_t fndPinTestSequence[14] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F,
                                   SEG_G, SEG_H, SEG_J, SEG_K, SEG_L, SEG_M, SEG_N, SEG_P};
size_t fndPinTestSequenceLength = 14;

uint16_t CHAR_1_MASK = (SEG_B | SEG_C);
uint16_t CHAR_2_MASK = (SEG_A | SEG_B | SEG_D | SEG_E | SEG_N | SEG_J);
uint16_t CHAR_3_MASK = (SEG_A | SEG_B | SEG_C | SEG_D | SEG_N | SEG_J);
uint16_t CHAR_4_MASK = (SEG_B | SEG_C | SEG_F | SEG_N | SEG_J);
uint16_t CHAR_5_MASK = (SEG_A | SEG_C | SEG_D | SEG_F | SEG_N | SEG_J);
uint16_t CHAR_A_MASK = (SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_N | SEG_J);
uint16_t CHAR_D_MASK = (SEG_A | SEG_B | SEG_C | SEG_D | SEG_G | SEG_L);
uint16_t CHAR_E_MASK = (SEG_A | SEG_D | SEG_E | SEG_F | SEG_N | SEG_J);
uint16_t CHAR_F_MASK = (SEG_A | SEG_E | SEG_F | SEG_N);
uint16_t CHAR_I_MASK = (SEG_A | SEG_D | SEG_G | SEG_L);
uint16_t CHAR_K_MASK = (SEG_F | SEG_E | SEG_N | SEG_H | SEG_K);
uint16_t CHAR_L_MASK = (SEG_D | SEG_E | SEG_F);
uint16_t CHAR_M_MASK = (SEG_B | SEG_C | SEG_E | SEG_F | SEG_P | SEG_H);
uint16_t CHAR_N_MASK = (SEG_B | SEG_C | SEG_E | SEG_F | SEG_P | SEG_K);
uint16_t CHAR_O_MASK = (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F);
uint16_t CHAR_P_MASK = (SEG_A | SEG_B | SEG_E | SEG_F |SEG_N |SEG_J);
uint16_t CHAR_S_MASK = (SEG_A | SEG_F | SEG_N | SEG_J | SEG_C | SEG_D);
uint16_t CHAR_T_MASK = (SEG_A | SEG_G | SEG_L);
uint16_t CHAR_U_MASK = (SEG_B | SEG_C | SEG_D | SEG_E | SEG_F);
uint16_t CHAR_V_MASK = (SEG_E | SEG_F | SEG_H | SEG_M);
uint16_t CHAR_ALL_MASK = 0x3FFF;
uint16_t CHAR_BLANK_MASK = 0;

uint16_t DOOR_CLOSED_MASK = SEG_A | SEG_G | SEG_L | SEG_D;
uint16_t DOOR_OPEN_MASK = SEG_F | SEG_E | SEG_B | SEG_C;

volatile uint16_t fndMasks[4] = {0, 0, 0, 0};

uint16_t floorMasks[6] = {0, CHAR_1_MASK, CHAR_2_MASK, CHAR_3_MASK, CHAR_4_MASK, CHAR_5_MASK};


void clearFndMasks()
{
  for (int i = 0; i < 4; ++i) {
    fndMasks[i] = 0;
  }
}

void setFndChar(uint16_t f1, uint16_t f2, uint16_t f3, uint16_t f4)
{
  fndMasks[0] = f1;
  fndMasks[1] = f2;
  fndMasks[2] = f3;
  fndMasks[3] = f4;
}


void writeDigitSegments(uint8_t digit, uint16_t mask)
{
  uint8_t *pins = (digit < 2) ? seg_pins1 : seg_pins2;
  for (int i = 0; i < 14; ++i) {
    digitalWrite(pins[i], ((mask>>i) & 0x1)? LOW : HIGH);
  }
}

void fndISR()
{
  static uint8_t phase = 0;
  digitalWrite(digit_pins[phase], LOW);
  digitalWrite(digit_pins[phase + 2], LOW);
  phase ^= 1;
  writeDigitSegments(phase, (uint16_t)fndMasks[phase]);
  writeDigitSegments(phase + 2, (uint16_t)fndMasks[phase + 2]);
  digitalWrite(digit_pins[phase], HIGH);
  digitalWrite(digit_pins[phase + 2], HIGH);
}
// ─── keypads ───────────────────────────────────────────────────────────────
const uint8_t ROWS = 3;
const uint8_t COLS = 3;

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'}
};

uint8_t rowPins[ROWS] = {8, 9, 10};
uint8_t colPins[COLS] = {11, 12, 13};

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// ─── Buzzer ───────────────────────────────────────────────────────────────
uint8_t buzzer_pin = 6;

uint16_t BEEP_MODE_SELECT_FREQ = 1000;
uint32_t BEEP_MODE_SELECT_MS = 100;
uint16_t BEEP_MODE2_FREQ = 2000;
uint32_t BEEP_MODE2_MS = 50;
uint16_t BEEP_MODE3_FREQ = 1000;
uint32_t BEEP_MODE3_MS = 100;

// ─── NeoPixel ──────────────────────────────────────────────────────────────
uint8_t neopixel_pin = 2;
uint8_t NUM_PIXELS = 8;

Adafruit_NeoPixel pixels(NUM_PIXELS, neopixel_pin, NEO_GRB + NEO_KHZ800);

uint32_t COLOR_BLUE = 0x0000FF;
uint32_t COLOR_RED = 0xFF0000;
uint32_t COLOR_GREEN = 0x00FF00;
uint32_t COLOR_ORANGE = 0xFFA500;
uint32_t COLOR_PINK = 0xFF1493;
uint32_t COLOR_OFF = 0x000000;

void setAllPixels(uint32_t color)
{
  for (int i = 0; i < NUM_PIXELS; ++i) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void showTemperatureHumidityPixels()
{
  uint32_t tempColor = (currentTemperature >= 24) ? COLOR_RED : COLOR_BLUE;
  uint32_t humidityColor = (currentHumidity >= 50) ? COLOR_ORANGE : COLOR_GREEN;

  for (int i = 0; i < 4; ++i) {
    pixels.setPixelColor(i, tempColor);
  }
  for (int i = 4; i < NUM_PIXELS; ++i) {
    pixels.setPixelColor(i, humidityColor);
  }
  pixels.show();
}

// ─── 화면 / 탐색 ──────────────────────────────────────────────────────────

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

void updateInitialMode()
{
  showLcdLines(
    " ELEVATOR SYSTEM",
    "  CIRCUIT DESIGN",
    "  & PROGRAMMING",
    "   2026.06.13"
  );
  setFndChar(CHAR_E_MASK, CHAR_L_MASK, CHAR_E_MASK, CHAR_V_MASK);
}

// ─── Mode 1 (FND 핀 테스트) ────────────────────────────────────────────────
uint32_t mode1TestingMs = 3000;
uint32_t mode1ResultMs = 3000;


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
  char line1[20 + 1];
  char line2[20 + 1];

  sprintf(line1, "CHECK DIGIT: F%u", mode1DigitIndex + 1);
  uint8_t *pins = (mode1DigitIndex < 2) ? seg_pins1 : seg_pins2;
  sprintf(line2, "CHECK PORT: IO%02u", pins[mode1SegmentIndex]);

  showLcdLines("PIN TEST: ACTIVE", line1, line2, "");
  clearFndMasks();
  fndMasks[mode1DigitIndex] = fndPinTestSequence[mode1SegmentIndex];
}

void showMode1TestingScreen()
{
  showLcdLines("PIN TESTING......", "", "", "");
  setFndChar(CHAR_T_MASK, CHAR_E_MASK, CHAR_S_MASK, CHAR_T_MASK);
}

void showMode1ResultScreen()
{
  showLcdLines("PIN TEST COMPLETED", "STATUS: DONE", "RESULT: OK", "");
  setFndChar(CHAR_BLANK_MASK, CHAR_O_MASK, CHAR_K_MASK, CHAR_BLANK_MASK);
}

void updateMode1(uint32_t now)
{
  switch (mode1Stage) {
    case MODE1_STAGE_SCAN:
      showMode1ActiveScreen();
      if (now - mode1LastStepMs >= fndTestIntervalMs) {
        mode1LastStepMs = now;
        if (mode1DigitIndex == 3 && mode1SegmentIndex == (fndPinTestSequenceLength - 1)) {
          mode1Stage = MODE1_STAGE_TESTING;
          mode1StageStartedMs = now;
        } else {
          ++mode1SegmentIndex;
          if (mode1SegmentIndex >= fndPinTestSequenceLength) {
            mode1SegmentIndex = 0;
            ++mode1DigitIndex;
          }
        }
        clearFndMasks();
        fndMasks[mode1DigitIndex] = fndPinTestSequence[mode1SegmentIndex];
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

// ─── Mode 2 (엘리베이터) ───────────────────────────────────────────────────
uint32_t mode2MoveStepMs = 2000;
uint32_t mode2ArrivedMs = 1000;
uint32_t mode2WaitMs = 2000;
uint32_t mode2OpenMs = 1000;
uint32_t mode2CompleteMs = 1000;
uint32_t mode2WarningIntervalMs = 500;
uint32_t blinkIntervalMs = 500;


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
  fndMasks[0] = showDoor ? (doorOpen ? DOOR_OPEN_MASK : DOOR_CLOSED_MASK) : 0;
  fndMasks[1] = showDoor ? (doorOpen ? DOOR_OPEN_MASK : DOOR_CLOSED_MASK) : 0;
  fndMasks[2] = floorMasks[mode2CurrentFloor];
  fndMasks[3] = CHAR_F_MASK;
}

void updateMode2Fnd(uint32_t now)
{
  if (mode2State == MODE2_MOVING_UP) {
    fndMasks[0] = CHAR_U_MASK;
    fndMasks[1] = CHAR_P_MASK;
    fndMasks[2] = floorMasks[mode2CurrentFloor];
    fndMasks[3] = CHAR_F_MASK;
    return;
  }

  if (mode2State == MODE2_MOVING_DOWN) {
    fndMasks[0] = CHAR_D_MASK;
    fndMasks[1] = CHAR_N_MASK;
    fndMasks[2] = floorMasks[mode2CurrentFloor];
    fndMasks[3] = CHAR_F_MASK;
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
    clearFndMasks();
    return;
  }

  setMode2DoorMasks(showDoor, doorOpen);
}

void showMode2Lcd()
{
  char line1[20 + 1];
  char line2[20 + 1];
  char line3[20 + 1];

  sprintf(line1, "ELEV:%uF", mode2CurrentFloor);
  sprintf(line2, "MODE:%-4s DOOR:%-5s", mode2ModeLabel(), mode2DoorLabel());
  buildTemperatureHumidityLine(line3);

  showLcdLines("    ELEV SYSTEM", line1, line2, line3);
}

void updateMode2Pixels(uint32_t now)
{
  if (mode2State == MODE2_WAIT_OPEN || mode2State == MODE2_WAIT_CLOSE) {
    if (((now / blinkIntervalMs) % 2U) == 0U) {
      setAllPixels(COLOR_ORANGE);
    } else {
      setAllPixels(COLOR_OFF);
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

void startMode2Call(uint8_t targetFloor, uint32_t now)
{
  if (mode2State != MODE2_IDLE && mode2State != MODE2_COMPLETE && mode2State != MODE2_WARNING) {
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
    uint8_t currentPhase = (uint8_t)((now - mode2StateStartedMs) / mode2WarningIntervalMs);
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

// ─── Mode 3 (관리자 모드) ──────────────────────────────────────────────────
uint32_t mode3ResultMs = 1500;
uint32_t mode3LcdFillStepMs = 40;
uint32_t mode3NeoStepMs = 500;
uint32_t mode3BuzzerStepMs = 500;


void printMode3MenuToSerial()
{
  Serial.println("[ 관리자 모드 ]");
  Serial.println();
  Serial.println("> 장치 리스트 (DEVICE LIST):");
  Serial.println("  1. LCD 전체 점등 테스트");
  Serial.println("  2. DHT11 온습도 동적 감지");
  Serial.println("  3. NeoPixel RGB 순자 점등");
  Serial.println("  4. 키패드 입력 스캔 테스트");
  Serial.println("  5. 부저 주파수 출력 테스트");
  Serial.println("모드 탈출은 키 '9'를 사용하십시오.");
  Serial.println("------------------------------------------");
  Serial.print("명령어 입력 : [  ]");
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


void beginMode3(uint32_t now)
{
  currentMode = MODE_3;
  returnToMode3Menu();
  mode3StateStartedMs = now;
  mode3LastStepMs = now;
}

void showMode3Menu()
{
  char line3[20 + 1];
  sprintf(line3, "SELECT NUMBER:[%c]", mode3PendingCommand == '\0' ? ' ' : mode3PendingCommand);

  showLcdLines(
    "    DEVICE TESTING",
    "1:LCD 2:DHT 3:NEO",
    "4:KEY 5:BUZ 9:EXIT",
    line3
  );
  setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);

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
  updateSensorValues(now);
  mode3State = MODE3_DHT_TEST;
  mode3StateStartedMs = now;
  mode3LastStepMs = now;
  mode3StartTemperature = currentTemperature;
  mode3StartHumidity = currentHumidity;
  Serial.print("진행 시간 : 0s | T:");
  Serial.print((float)currentTemperature, 1);
  Serial.print(" H:");
  Serial.print((float)currentHumidity, 1);
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
      returnToMode3Menu();
      enterInitialScreen();
      break;
    default:
      mode3MenuPrinted = false;
      break;
  }

  mode3PendingCommand = '\0';
}

void redrawMode3InputLine()
{
  Serial.print("\r명령어 입력 : [ ");
  if (mode3PendingCommand != '\0') {
    Serial.print(mode3PendingCommand);
  } else {
    Serial.print(' ');
  }
  Serial.print(" ]");
}

void handleMode3Serial(uint32_t now)
{
  while (Serial.available() > 0) {
    char received = Serial.read();

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


void updateMode3(uint32_t now)
{
  static const uint16_t buzzerTestFrequencies[4] = {262, 330, 392, 523};
  static const uint32_t neoColors[3] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};

  switch (mode3State) {
    case MODE3_MENU:
      showMode3Menu();
      break;

    case MODE3_LCD_TEST:
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);
      if (now - mode3LastStepMs >= mode3LcdFillStepMs) {
        mode3LastStepMs = now;
        uint8_t row = mode3StepIndex / 20;
        uint8_t col = mode3StepIndex % 20;
        lcd.setCursor(col, row);
        lcd.write(LCD_BLOCK_CHAR);
        ++mode3StepIndex;

        if (mode3StepIndex >= 20 * 4) {
          mode3State = MODE3_LCD_RESULT;
          mode3StateStartedMs = now;
          clearLcdDirect();
        }
      }
      break;

    case MODE3_LCD_RESULT:
      showLcdLines("", "    LCD TEST OK", "   [RESULT: PASS]", "");
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;

    case MODE3_DHT_TEST: {
      char line2[20 + 1];
      buildTemperatureHumidityLine(line2);
      showLcdLines("DHT11 DYNAMIC TEST", "WAIT FOR CHANGE.....", line2, "");
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);

      uint32_t elapsedSec = (now - mode3StateStartedMs) / 1000;
      if (now - mode3LastStepMs >= 1000) {
        mode3LastStepMs = now;
        Serial.print("\r진행 시간 : ");
        Serial.print(elapsedSec);
        Serial.print("s | T:");
        Serial.print(currentTemperature);
        Serial.print(" H:");
        Serial.print(currentHumidity);
        Serial.print("  ");
      }

      {
        int currentTemp = currentTemperature;
        int currentHumi = currentHumidity;
        if (currentTemp != mode3StartTemperature || currentHumi != mode3StartHumidity) {
          Serial.println();
          Serial.print("진행 시간 : ");
          Serial.print(elapsedSec);
          Serial.print("s | T:");
          Serial.print(currentTemperature);
          Serial.print(" H:");
          Serial.print(currentHumidity);
          Serial.println(" [변경 감지]");
          mode3State = MODE3_DHT_RESULT;
          mode3StateStartedMs = now;
        }
      }
      break;
    }

    case MODE3_DHT_RESULT:
      showLcdLines("", "   DHT11 TEST OK", "   [RESULT: PASS]", "");
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;

    case MODE3_NEO_TEST:
      showLcdLines("NEO PIXEL TESTING..", "", "", "");
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);
      if (now - mode3LastStepMs >= mode3NeoStepMs) {
        mode3LastStepMs = now;
        uint8_t colorIndex = mode3StepIndex / NUM_PIXELS;
        uint8_t pixelIndex = mode3StepIndex % NUM_PIXELS;
        setAllPixels(COLOR_OFF);
        if (colorIndex < 3) {
          pixels.setPixelColor(pixelIndex, neoColors[colorIndex]);
          pixels.show();
          ++mode3StepIndex;
        }
        if (mode3StepIndex >= NUM_PIXELS * 3) {
          setAllPixels(COLOR_OFF);
          mode3State = MODE3_NEO_RESULT;
          mode3StateStartedMs = now;
        }
      }
      break;

    case MODE3_NEO_RESULT:
      showLcdLines("", " NEO PIXEL TEST OK", " [RESULT: ALL PASS]", "");
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;

    case MODE3_KEYPAD_TEST: {
      char line3[20 + 1];
      if (mode3LastPressedKey >= '1' && mode3LastPressedKey <= '8') {
        sprintf(line3, "PRESSED: SW%c", mode3LastPressedKey);
      } else {
        sprintf(line3, "PRESSED: --");
      }
      showLcdLines("KEYPAD TESTING...", "PUSH ANY KEY (9:END)", "", line3);
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);
      break;
    }

    case MODE3_KEYPAD_RESULT:
      showLcdLines("", " KEYPAD TEST OK", " [RESULT: ALL PASS]", "");
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;

    case MODE3_BUZZER_TEST: {
      char line1[20 + 1];
      showLcdLines("BUZZER TESTING...", "", "", "");
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);

      if (mode3StepIndex < 4) {
        sprintf(line1, "[FREQ:%3uHz]", buzzerTestFrequencies[mode3StepIndex]);
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
      showLcdLines("", "   BUZZER TEST OK", "   [RESULT: PASS]", "");
      setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);
      if (now - mode3StateStartedMs >= mode3ResultMs) {
        returnToMode3Menu();
      }
      break;
  }
}

// ─── 공통 업데이트 / 입력 처리 ─────────────────────────────────────────────

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
        startMode2Call((key - '4') + 1, now);
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
        returnToMode3Menu();
      enterInitialScreen();
      }
      break;
  }
}

// ─── Arduino 진입점 ────────────────────────────────────────────────────────

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
  lcd.createChar(LCD_DEGREE_CHAR, degreeCharBitmap);
  lcd.createChar(LCD_BLOCK_CHAR, blockCharBitmap);
  clearLcdDirect();

  for (int i = 0; i < 14; ++i) {
    pinMode(seg_pins1[i], OUTPUT);
    digitalWrite(seg_pins1[i], HIGH);
    pinMode(seg_pins2[i], OUTPUT);
    digitalWrite(seg_pins2[i], HIGH);
  }

  for (int i = 0; i < 4; ++i) {
    pinMode(digit_pins[i], OUTPUT);
    digitalWrite(digit_pins[i], LOW);
  }

  pixels.begin();
  setAllPixels(COLOR_OFF);

  tone(buzzer_pin, 440, 500);
  delay(500);
  noTone(buzzer_pin);

  updateSensorValues(millis());
  resetMode1State();
  enterInitialScreen();

  MsTimer2::set(digitOnMs, fndISR);
  MsTimer2::start();
}

void loop()
{
  uint32_t now = millis();

  updateSensorValues(now);
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




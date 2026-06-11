#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Adafruit_NeoPixel.h>
#include <TimerOne.h>
#include <DHT.h>

uint8_t trig_pin = 3;
uint8_t echo_pin = 4;
uint8_t motor_pin = 7;
uint8_t ldr_pin = A0;

#define MODE_INITIAL 0
#define MODE_1 1
#define MODE_2 2
#define MODE_3 3

#define MODE1_STAGE_SCAN 0
#define MODE1_STAGE_TESTING 1
#define MODE1_STAGE_RESULT 2

#define MODE2_IDLE 0
#define MODE2_MOVING_UP 1
#define MODE2_MOVING_DOWN 2
#define MODE2_ARRIVED 3
#define MODE2_WAIT_OPEN 4
#define MODE2_OPEN 5
#define MODE2_WAIT_CLOSE 6
#define MODE2_COMPLETE 7
#define MODE2_WARNING 8

#define MODE3_MENU 0
#define MODE3_LCD_TEST 1
#define MODE3_LCD_RESULT 2
#define MODE3_DHT_TEST 3
#define MODE3_DHT_RESULT 4
#define MODE3_NEO_TEST 5
#define MODE3_NEO_RESULT 6
#define MODE3_KEYPAD_TEST 7
#define MODE3_KEYPAD_RESULT 8
#define MODE3_BUZZER_TEST 9
#define MODE3_BUZZER_RESULT 10
#define MODE3_PASSWORD 11
#define MODE3_PW_FAIL  12

uint8_t currentMode = MODE_INITIAL;
uint8_t mode1Stage = MODE1_STAGE_SCAN;
uint8_t mode2State = MODE2_IDLE;
uint8_t mode3State = MODE3_MENU;

int currentTemperature = 27;
int currentHumidity = 80;
uint32_t lastDhtReadMs = 0;

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
char mode3PwInput[9] = {0};
uint8_t mode3PwLen = 0;

// ─── LCD ───────────────────────────────────────────────────────────────────
uint8_t LCD_DEGREE_CHAR = 1;
uint8_t LCD_BLOCK_CHAR = 2;

LiquidCrystal_I2C lcd(0x3F, 20, 4);
char lcdData[4][20 + 1] = {{0,},};
char lcdCache[4][20 + 1] = {{0,},};
uint8_t degreeCharBitmap[8] = {0x6, 0x9, 0x9, 0x6, 0, 0, 0, 0};
uint8_t blockCharBitmap[8] = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};    

void writeLcdLine(uint8_t row, const char *text)
{
  if (strcmp(lcdCache[row], text) != 0)
  {
    lcd.setCursor(0, row);
    lcd.print(text);
    strcpy(lcdCache[row], text);
  }
}

void setLcdData(const char *line0, const char *line1, const char *line2, const char *line3)
{
  snprintf(lcdData[0], 20 + 1, "%-20s", line0);
  snprintf(lcdData[1], 20 + 1, "%-20s", line1);
  snprintf(lcdData[2], 20 + 1, "%-20s", line2);
  snprintf(lcdData[3], 20 + 1, "%-20s", line3);
}

void updateLcdLines(void)
{
  for (uint8_t row = 0; row < 4; row++)  
    writeLcdLine(row, lcdData[row]);  
}

// ─── DHT / 센서 ────────────────────────────────────────────────────────────
uint8_t dht11_pin = 5;
uint32_t dhtReadIntervalMs = 2000;

DHT dht(dht11_pin, DHT11);

void updateSensorValues(uint32_t now)
{
  if (now - lastDhtReadMs > dhtReadIntervalMs)
  {
    currentHumidity = dht.readHumidity();
    currentTemperature = dht.readTemperature();
    lastDhtReadMs = now;
  }
}

// ─── FND ───────────────────────────────────────────────────────────────────
uint32_t digitOnMs = 1;

uint8_t seg_pins1[15] = {15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
uint8_t seg_pins2[15] = {35, 36, 37, 38, 39, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};
uint8_t digit_pins[] = {32, 33, 52, 53};

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
uint16_t SEG_DP = 0x1 << 14;

uint16_t fndPinTestSequence[15] =  {SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_P, SEG_G, SEG_H, SEG_J, SEG_K,
			   SEG_L, SEG_M, SEG_N, SEG_DP};

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
uint16_t CHAR_P_MASK = (SEG_A | SEG_B | SEG_E | SEG_F | SEG_N | SEG_J);
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
  for (int i = 0; i < 15; ++i)  
    digitalWrite(pins[i], ((mask >> i) & 0x1) ? LOW : HIGH);  
}

void fndISR()
{
  static uint8_t phase = 0;
  digitalWrite(digit_pins[phase], LOW);
  digitalWrite(digit_pins[phase + 2], LOW);
  phase = !phase;
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
    {'7', '8', '9'}};

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
  for (int i = 0; i < NUM_PIXELS; ++i)  
    pixels.setPixelColor(i, color);  
  pixels.show();
}


// ─── 화면 / 탐색 ──────────────────────────────────────────────────────────
// ─── Mode 1 (FND 핀 테스트) ────────────────────────────────────────────────
uint8_t mode1Step = 0; // 현재 몇 번째 핀을 테스트 중인지. step / 14 → 자릿수. step % 14 → segment
uint32_t fndTestIntervalMs = 300;
uint32_t mode1LastStepMs = 0;     // 마지막으로 스텝이 바뀐 시각
uint32_t mode1StageStartedMs = 0; // 현재 스테이지(TESTING 또는 RESULT)가 시작된 시각.  TESTING/RESULT 단계의 타이머

void updateMode1(uint32_t now)
{
  switch (mode1Stage)
  {
  case MODE1_STAGE_SCAN:
  {
    // digit과 segment 계산
    uint8_t digit = mode1Step / 15; // 15는 segment 개수 (A, B, C, D, E, F, G, H, J, K, L, M, N, P, DP)
    uint8_t segment = mode1Step % 15;
    // LCD에 현재 테스트 중인 digit과 segment 정보 표시
    char line1[20 + 1], line2[20 + 1];
    sprintf(line1, "CHECK DIGIT: F%d", digit + 1);    
    sprintf(line2, "CHECK PORT: IO%02d", (digit < 2) ? seg_pins1[segment] : seg_pins2[segment]);
    setLcdData("PIN TEST: ACTIVE", line1, line2, "");
    // FND에 현재 테스트 중인 segment 표시
    setFndChar(0,0,0,0);
    fndMasks[digit] = fndPinTestSequence[segment];
    // 일정 시간이 지나면 다음 단계로 이동
    if (now - mode1LastStepMs >= fndTestIntervalMs)
    {
      mode1LastStepMs = now;
      mode1Step++;
      // 모든 핀 테스트가 완료되면 TESTING 단계로 이동
      if (mode1Step == 4 * 15) // 4 digits * 15 segments
      {
        mode1Stage = MODE1_STAGE_TESTING;
        mode1StageStartedMs = now;
      }
    }
    break;
  }

  case MODE1_STAGE_TESTING:
    // show Mode1 Testing Screen
    setLcdData("PIN TESTING......", "", "", "");
    setFndChar(CHAR_T_MASK, CHAR_E_MASK, CHAR_S_MASK, CHAR_T_MASK);
    if (now - mode1StageStartedMs >= 3000)
    {
      mode1Stage = MODE1_STAGE_RESULT;
      mode1StageStartedMs = now;
    }
    break;

  case MODE1_STAGE_RESULT:
    // show Mode1 Result Screen
    setLcdData("PIN TEST COMPLETED", "STATUS: DONE", "RESULT: OK", "");
    setFndChar(CHAR_BLANK_MASK, CHAR_O_MASK, CHAR_K_MASK, CHAR_BLANK_MASK);
    if (now - mode1StageStartedMs >= 3000)
    {
      // Reset Mode1 State
      mode1Stage = MODE1_STAGE_SCAN;
      mode1Step = 0;
      currentMode = MODE_INITIAL;      
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

void updateMode2(uint32_t now)
{
  // 상태 전이
  if (mode2State == MODE2_WARNING)
  {
    //경과시간 0ms~499ms  → phase = 0  (짝수 → 부저+FND ON)
    //경과시간 500~999ms  → phase = 1  (홀수 → 무음+FND OFF)
    //경과시간 1000~1499ms → phase = 2  (짝수 → 부저+FND ON)
    //경과시간 1500~1999ms → phase = 3  (홀수 → 무음+FND OFF)
    //경과시간 2000ms~    → phase = 4  → WARNING 종료
    uint8_t phase = (uint8_t)((now - mode2StateStartedMs) / mode2WarningIntervalMs); //WARNING 상태가 시작된 이후 몇 번째 0.5초 구간인지 계산
    if (phase != mode2LastWarningPhase)
    {
      mode2LastWarningPhase = phase;
      if (phase < 4 && (phase % 2U) == 0U)
        tone(buzzer_pin, BEEP_MODE2_FREQ, 100);
    }
    if (phase >= 4) //2초 이상 경과한 상태. 2회 점멸이 끝났다는 뜻이므로 → MODE2_IDLE로 복귀
    {
      mode2State = MODE2_IDLE;
      mode2StateStartedMs = now;
      mode2LastWarningPhase = 0;
    }
  }
  else if (mode2State == MODE2_MOVING_UP || mode2State == MODE2_MOVING_DOWN)
  {
    if (now - mode2StateStartedMs >= mode2MoveStepMs)
    {
      mode2StateStartedMs += mode2MoveStepMs;
      mode2CurrentFloor += (mode2State == MODE2_MOVING_UP) ? 1 : -1;
      if (mode2CurrentFloor == mode2TargetFloor)
      {
        mode2State = MODE2_ARRIVED;
        mode2StateStartedMs = now;
      }
    }
  }
  else if (mode2State == MODE2_ARRIVED && now - mode2StateStartedMs >= mode2ArrivedMs)
  {
    mode2State = MODE2_WAIT_OPEN;
    mode2StateStartedMs = now;
  }
  else if (mode2State == MODE2_WAIT_OPEN && now - mode2StateStartedMs >= mode2WaitMs)
  {
    mode2State = MODE2_OPEN;
    mode2StateStartedMs = now;
  }
  else if (mode2State == MODE2_OPEN && now - mode2StateStartedMs >= mode2OpenMs)
  {
    mode2State = MODE2_WAIT_CLOSE;
    mode2StateStartedMs = now;
  }
  else if (mode2State == MODE2_WAIT_CLOSE && now - mode2StateStartedMs >= mode2WaitMs)
  {
    mode2State = MODE2_COMPLETE;
    mode2StateStartedMs = now;
  }
  else if (mode2State == MODE2_COMPLETE && now - mode2StateStartedMs >= mode2CompleteMs)
  {
    mode2State = MODE2_IDLE;
    mode2TargetFloor = mode2CurrentFloor;
    mode2StateStartedMs = now;
  }

  // LCD
  const char *modeStr = (mode2State == MODE2_MOVING_UP) ? "UP" :
                        (mode2State == MODE2_MOVING_DOWN) ? "DOWN" : "STOP";
  const char *doorStr = (mode2State == MODE2_WAIT_OPEN || mode2State == MODE2_WAIT_CLOSE) ? "WAIT" :
                        (mode2State == MODE2_OPEN) ? "OPEN" : "CLOSE";
  char line1[21], line2[21], line3[21];
  sprintf(line1, "ELEV:%dF", mode2CurrentFloor);
  sprintf(line2, "MODE:%-4s DOOR:%-5s", modeStr, doorStr);
  sprintf(line3, "TEMP:%2d%cC HUMI:%2d%%", currentTemperature, LCD_DEGREE_CHAR, currentHumidity);
  setLcdData("ELEVATOR SYSTEM", line1, line2, line3);

  // FND
  bool blink = ((now / blinkIntervalMs) % 2U) == 0U;
  if (mode2State == MODE2_MOVING_UP)
    setFndChar(CHAR_U_MASK, CHAR_P_MASK, floorMasks[mode2CurrentFloor], CHAR_F_MASK);
  else if (mode2State == MODE2_MOVING_DOWN)
    setFndChar(CHAR_D_MASK, CHAR_N_MASK, floorMasks[mode2CurrentFloor], CHAR_F_MASK);
  else if (mode2State == MODE2_WAIT_OPEN)
  {
    uint16_t d = blink ? DOOR_CLOSED_MASK : 0;
    setFndChar(d, d, floorMasks[mode2CurrentFloor], CHAR_F_MASK);
  }
  else if (mode2State == MODE2_OPEN)
    setFndChar(DOOR_OPEN_MASK, DOOR_OPEN_MASK, floorMasks[mode2CurrentFloor], CHAR_F_MASK);
  else if (mode2State == MODE2_WAIT_CLOSE)
  {
    uint16_t d = blink ? DOOR_OPEN_MASK : 0;
    setFndChar(d, d, floorMasks[mode2CurrentFloor], CHAR_F_MASK);
  }
  else if (mode2State == MODE2_WARNING)
  {
    bool wb = (((now - mode2StateStartedMs) / mode2WarningIntervalMs) % 2U) == 0U;
    uint16_t d = wb ? DOOR_CLOSED_MASK : 0;
    setFndChar(d, d, wb ? floorMasks[mode2CurrentFloor] : 0, wb ? CHAR_F_MASK : 0);
  }
  else
    setFndChar(DOOR_CLOSED_MASK, DOOR_CLOSED_MASK, floorMasks[mode2CurrentFloor], CHAR_F_MASK);
}

// ─── Mode 3 (관리자 모드) ──────────────────────────────────────────────────
uint32_t mode3ResultMs = 1500;
uint32_t mode3LcdFillStepMs = 40;
uint32_t mode3NeoStepMs = 500;
uint32_t mode3BuzzerStepMs = 500;

void returnToMode3Menu()
{
  mode3State = MODE3_MENU;
  mode3MenuPrinted = false;
  mode3PendingCommand = '\0';
  mode3LastPressedKey = '\0';
  mode3StepIndex = 0;
  noTone(buzzer_pin);
}

void handleMode3Serial(uint32_t now)
{
  while (Serial.available() > 0)
  {
    char received = Serial.read();
    if (currentMode != MODE_3) continue;

    if (mode3State == MODE3_PASSWORD)
    {
      if (received == '\r' || received == '\n')
      {
        mode3PwInput[mode3PwLen] = '\0';
        if (strcmp(mode3PwInput, "password") == 0)
        {
          Serial.println();
          returnToMode3Menu();
        }
        else
        {
          mode3State = MODE3_PW_FAIL;
          mode3StateStartedMs = now;
          Serial.println("\n틀린 패스워드입니다.");
        }
      }
      else if ((received == 8 || received == 127) && mode3PwLen > 0)
      {
        mode3PwInput[--mode3PwLen] = '\0';
        char stars[9]; memset(stars, ' ', 8); stars[8] = '\0';
        for (uint8_t i = 0; i < mode3PwLen; i++) stars[i] = '*';
        Serial.print("\rPW: ["); Serial.print(stars); Serial.print("]");
      }
      else if (mode3PwLen < 8 && received >= 32 && received < 127)
      {
        mode3PwInput[mode3PwLen++] = received;
        char stars[9]; memset(stars, ' ', 8); stars[8] = '\0';
        for (uint8_t i = 0; i < mode3PwLen; i++) stars[i] = '*';
        Serial.print("\rPW: ["); Serial.print(stars); Serial.print("]");
      }
      continue;
    }

    if (mode3State != MODE3_MENU)
      continue;

    if (received == '\r' || received == '\n')
    {
      if (mode3PendingCommand == '\0') continue;
      char cmd = mode3PendingCommand;
      mode3PendingCommand = '\0';
      Serial.println();

      switch (cmd)
      {
      case '1':
        mode3State = MODE3_LCD_TEST;
        mode3StateStartedMs = now;
        mode3LastStepMs = now - mode3LcdFillStepMs;
        mode3StepIndex = 0;
        setLcdData("", "", "", "");
        break;
      case '2':
        updateSensorValues(now);
        mode3StartTemperature = currentTemperature;
        mode3StartHumidity = currentHumidity;
        mode3State = MODE3_DHT_TEST;
        mode3StateStartedMs = now;
        mode3LastStepMs = now;
        Serial.print("진행 시간 : 0s | T:");
        Serial.print(currentTemperature);
        Serial.print(" H:");
        Serial.print(currentHumidity);
        break;
      case '3':
        mode3State = MODE3_NEO_TEST;
        mode3StateStartedMs = now;
        mode3LastStepMs = now - mode3NeoStepMs;
        mode3StepIndex = 0;
        break;
      case '4':
        mode3State = MODE3_KEYPAD_TEST;
        mode3StateStartedMs = now;
        mode3LastPressedKey = '\0';
        break;
      case '5':
        mode3State = MODE3_BUZZER_TEST;
        mode3StateStartedMs = now;
        mode3LastStepMs = now - mode3BuzzerStepMs;
        mode3StepIndex = 0;
        break;
      case '9':
        returnToMode3Menu();
        currentMode = MODE_INITIAL;
        break;
      default:
        mode3MenuPrinted = false;
        break;
      }
      continue;
    }

    if (received == 8 || received == 127)
      mode3PendingCommand = '\0';
    else if (received == '1' || received == '2' || received == '3' ||
             received == '4' || received == '5' || received == '9')
      mode3PendingCommand = received;
    else
      continue;

    Serial.print("\r명령어 입력 : [ ");
    Serial.print(mode3PendingCommand != '\0' ? mode3PendingCommand : ' ');
    Serial.print(" ]");
  }
}

void updateMode3(uint32_t now)
{
  static const uint16_t buzzerFreqs[4] = {262, 330, 392, 523};
  static const uint32_t neoColors[3] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};

  setFndChar(CHAR_A_MASK, CHAR_D_MASK, CHAR_M_MASK, CHAR_I_MASK);

  switch (mode3State)
  {
  case MODE3_PASSWORD:
  {
    char stars[9]; memset(stars, ' ', 8); stars[8] = '\0';
    for (uint8_t i = 0; i < mode3PwLen; i++) stars[i] = '*';
    char pwLine[21];
    sprintf(pwLine, "PW: [%-8s]", stars);
    setLcdData("   ADMIN MODE   ", "", pwLine, "");
    break;
  }
  case MODE3_PW_FAIL:
    setLcdData("   ADMIN MODE   ", "  WRONG PASSWORD", "  PLEASE RETRY  ", "");
    if (now - mode3StateStartedMs >= 1500)
    {
      mode3PwLen = 0;
      memset(mode3PwInput, 0, sizeof(mode3PwInput));
      mode3State = MODE3_PASSWORD;
      Serial.print("\rPW: [        ]");
    }
    break;
  case MODE3_MENU:
  {
    char line3[21];
    sprintf(line3, "SELECT NUMBER:[%c]", mode3PendingCommand ? mode3PendingCommand : ' ');
    setLcdData("    DEVICE TESTING", "1:LCD 2:DHT 3:NEO", "4:KEY 5:BUZ 9:EXIT", line3);
    if (!mode3MenuPrinted)
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
      mode3MenuPrinted = true;
    }
    break;
  }

  case MODE3_LCD_TEST:
    if (now - mode3LastStepMs >= mode3LcdFillStepMs)
    {
      mode3LastStepMs = now;
      lcd.setCursor(mode3StepIndex % 20, mode3StepIndex / 20);
      lcd.write(LCD_BLOCK_CHAR);
      if (++mode3StepIndex >= 80)
      {
        mode3State = MODE3_LCD_RESULT;
        mode3StateStartedMs = now;
        setLcdData("", "", "", "");
      }
    }
    break;

  case MODE3_LCD_RESULT:
    setLcdData("", "    LCD TEST OK", "   [RESULT: PASS]", "");
    if (now - mode3StateStartedMs >= mode3ResultMs)
      returnToMode3Menu();
    break;

  case MODE3_DHT_TEST:
  {
    char line2[21];
    sprintf(line2, "TEMP:%2d%cC HUMI:%2d%%", currentTemperature, LCD_DEGREE_CHAR, currentHumidity);
    setLcdData("DHT11 DYNAMIC TEST", "WAIT FOR CHANGE.....", line2, "");

    uint32_t elapsedSec = (now - mode3StateStartedMs) / 1000;
    if (now - mode3LastStepMs >= 1000)
    {
      mode3LastStepMs = now;
      Serial.print("\r진행 시간 : ");
      Serial.print(elapsedSec);
      Serial.print("s | T:");
      Serial.print(currentTemperature);
      Serial.print(" H:");
      Serial.print(currentHumidity);
      Serial.print("  ");
    }

    if (currentTemperature != mode3StartTemperature || currentHumidity != mode3StartHumidity)
    {
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
    break;
  }

  case MODE3_DHT_RESULT:
    setLcdData("", "   DHT11 TEST OK", "   [RESULT: PASS]", "");
    if (now - mode3StateStartedMs >= mode3ResultMs)
      returnToMode3Menu();
    break;

  case MODE3_NEO_TEST:
    setLcdData("NEO PIXEL TESTING..", "", "", "");
    if (now - mode3LastStepMs >= mode3NeoStepMs)
    {
      mode3LastStepMs = now;
      setAllPixels(COLOR_OFF);
      if (mode3StepIndex < NUM_PIXELS * 3)
      {
        pixels.setPixelColor(mode3StepIndex % NUM_PIXELS, neoColors[mode3StepIndex / NUM_PIXELS]);
        pixels.show();
        ++mode3StepIndex;
      }
      if (mode3StepIndex >= NUM_PIXELS * 3)
      {
        setAllPixels(COLOR_OFF);
        mode3State = MODE3_NEO_RESULT;
        mode3StateStartedMs = now;
      }
    }
    break;

  case MODE3_NEO_RESULT:
    setLcdData("", " NEO PIXEL TEST OK", " [RESULT: ALL PASS]", "");
    if (now - mode3StateStartedMs >= mode3ResultMs)
      returnToMode3Menu();
    break;

  case MODE3_KEYPAD_TEST:
  {
    char line3[21];
    if (mode3LastPressedKey >= '1' && mode3LastPressedKey <= '8')
      sprintf(line3, "PRESSED: SW%c", mode3LastPressedKey);
    else
      sprintf(line3, "PRESSED: --");
    setLcdData("KEYPAD TESTING...", "PUSH ANY KEY (9:END)", "", line3);
    break;
  }

  case MODE3_KEYPAD_RESULT:
    setLcdData("", " KEYPAD TEST OK", " [RESULT: ALL PASS]", "");
    if (now - mode3StateStartedMs >= mode3ResultMs)
      returnToMode3Menu();
    break;

  case MODE3_BUZZER_TEST:
  {
    char line1[21];
    sprintf(line1, mode3StepIndex < 4 ? "[FREQ:%3uHz]" : "", buzzerFreqs[mode3StepIndex < 4 ? mode3StepIndex : 0]);
    setLcdData("BUZZER TESTING...", line1, "", "");

    if (mode3StepIndex < 4 && now - mode3LastStepMs >= mode3BuzzerStepMs)
    {
      mode3LastStepMs = now;
      tone(buzzer_pin, buzzerFreqs[mode3StepIndex], mode3BuzzerStepMs - 20);
      ++mode3StepIndex;
    }
    if (mode3StepIndex >= 4 && now - mode3LastStepMs >= mode3BuzzerStepMs)
    {
      noTone(buzzer_pin);
      mode3State = MODE3_BUZZER_RESULT;
      mode3StateStartedMs = now;
    }
    break;
  }

  case MODE3_BUZZER_RESULT:
    setLcdData("", "   BUZZER TEST OK", "   [RESULT: PASS]", "");
    if (now - mode3StateStartedMs >= mode3ResultMs)
      returnToMode3Menu();
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
  setLcdData("", "", "", "");

  for (int i = 0; i < 15; ++i)
  {
    pinMode(seg_pins1[i], OUTPUT);
    digitalWrite(seg_pins1[i], HIGH);
    pinMode(seg_pins2[i], OUTPUT);
    digitalWrite(seg_pins2[i], HIGH);
  }

  for (int i = 0; i < 4; ++i)
  {
    pinMode(digit_pins[i], OUTPUT);
    digitalWrite(digit_pins[i], LOW);
  }

  pixels.begin();
  setAllPixels(COLOR_OFF);

  tone(buzzer_pin, 440, 500);
  delay(500);
  noTone(buzzer_pin);

  updateSensorValues(millis());
  // Reset Mode1 State
  mode1Stage = MODE1_STAGE_SCAN;
  mode1Step = 0;
  
  Timer1.initialize(digitOnMs * 1000);
  Timer1.attachInterrupt(fndISR);
}

void loop()
{
  uint32_t now = millis();

  updateSensorValues(now);
  handleMode3Serial(now);

  char key = customKeypad.getKey();

  // 키값에 따라 입력 처리 (현재 모드 기반)
  if (key)
  {
    switch (currentMode)
    {
    case MODE_INITIAL:
      if (key == '1')
      {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);        
        currentMode = MODE_1; // begin Mode1
        mode1LastStepMs = now;
      }
      else if (key == '2')
      {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        currentMode = MODE_2;
        if (!mode2Initialized)
        {
          mode2Initialized = true;
          mode2State = MODE2_IDLE;
          mode2CurrentFloor = 1;
          mode2TargetFloor = 1;
          mode2StateStartedMs = now;
          mode2Paused = false;
          return;
        }

        if (mode2Paused)
        {
          mode2Paused = false;
          mode2StateStartedMs = now - mode2PausedElapsedMs;
        }
      }
      else if (key == '3')
      {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        currentMode = MODE_3;
        mode3State = MODE3_PASSWORD;
        mode3PwLen = 0;
        memset(mode3PwInput, 0, sizeof(mode3PwInput));
        Serial.println("\n패스워드를 입력하세요:");
        Serial.print("PW: [        ]");
      }
      break;

    case MODE_1:
      if (key == '9')         // pause Mode1      
      {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        currentMode = MODE_INITIAL;        
      }
      break;

    case MODE_2:
      if (key >= '4' && key <= '8') // 층 번호 입력 시
      {
        tone(buzzer_pin, BEEP_MODE2_FREQ, BEEP_MODE2_MS);
        if (mode2State != MODE2_IDLE && mode2State != MODE2_COMPLETE && mode2State != MODE2_WARNING)
          return;

        mode2TargetFloor = (key - '4') + 1;
        if (mode2TargetFloor == mode2CurrentFloor)
        {
          mode2State = MODE2_WARNING;
          mode2StateStartedMs = now;
          mode2LastWarningPhase = 255;
          return;
        }

        mode2State = (mode2TargetFloor > mode2CurrentFloor) ? MODE2_MOVING_UP : MODE2_MOVING_DOWN;
        mode2StateStartedMs = now;
      }
      else if (key == '9')
      {
        mode2Paused = true;
        mode2PausedElapsedMs = now - mode2StateStartedMs;
        currentMode = MODE_INITIAL;        
      }
      break;

    case MODE_3:
      if (mode3State == MODE3_KEYPAD_TEST)
      {
        if (key >= '1' && key <= '8')
        {
          tone(buzzer_pin, BEEP_MODE3_FREQ, BEEP_MODE3_MS);
          mode3LastPressedKey = key;
        }
        else if (key == '9')
        {
          mode3State = MODE3_KEYPAD_RESULT;
          mode3StateStartedMs = now;
        }
      }
      else if (key == '9')
      {
        returnToMode3Menu();
        currentMode = MODE_INITIAL;        
      }
      break;
    }
  }

  // 현재 모드에 따른 업데이트 수행
  switch (currentMode)
  {
  case MODE_INITIAL:
    setLcdData(" ELEVATOR SYSTEM", "  CIRCUIT DESIGN", "  & PROGRAMMING", "   2026.06.13");
    setFndChar(CHAR_E_MASK, CHAR_L_MASK, CHAR_E_MASK, CHAR_V_MASK);
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

  if (currentMode == MODE_2)
  {
    bool blink = ((now / blinkIntervalMs) % 2U) == 0U;
    if (mode2State == MODE2_WAIT_OPEN || mode2State == MODE2_WAIT_CLOSE)
      setAllPixels(blink ? COLOR_ORANGE : COLOR_OFF);
    else if (mode2State == MODE2_OPEN)
      setAllPixels(COLOR_GREEN);
    else
      setAllPixels(COLOR_PINK);
  }
  else if (!(currentMode == MODE_3 && mode3State == MODE3_NEO_TEST))
  {
    uint32_t tempColor = (currentTemperature >= 24) ? COLOR_RED : COLOR_BLUE;
    uint32_t humColor  = (currentHumidity   >= 50) ? COLOR_ORANGE : COLOR_GREEN;
    for (int i = 0; i < 4; ++i) pixels.setPixelColor(i, tempColor);
    for (int i = 4; i < NUM_PIXELS; ++i) pixels.setPixelColor(i, humColor);
    pixels.show();
  }
  updateLcdLines();
}

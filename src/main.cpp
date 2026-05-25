#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Adafruit_NeoPixel.h>

// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// IO 선언
const uint8_t neopixel_pin = 2;    // NEOPIXEL1 네오픽셀
const uint8_t trig_pin = 3;        // US1 초음파 트리거
const uint8_t echo_pin = 4;        // US1 초음파 에코
const uint8_t dht11_pin = 5;       // DHT11 온습도 센서
const uint8_t buzzer_pin = 6;      // BUZ1 버저
const uint8_t motor_pin = 7;      // MOTOR 모터 제어 핀. PWM 제어 가능 핀으로 설정
const uint8_t ldr_pin = A0;        // LDR1 아날로그 입력 A0

#define NUM_PIXELS 8
Adafruit_NeoPixel pixels(NUM_PIXELS, neopixel_pin, NEO_GRB + NEO_KHZ800);


// 14세그먼트 GPIO 핀 배열
// datasheet 표기:              A,  B,  C,  D,  E,  F,  G,  H,  J,  K,  L,  M,  N,  P
const uint8_t seg_pins1[14] = {15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 27, 28, 29, 30};
const uint8_t seg_pins2[14] = {33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46};
const uint8_t digit_pins[4] = {31, 32, 47, 48}; // COM1, COM2, COM3, COM4

const uint8_t ROWS = 3; 
const uint8_t COLS = 3; 
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'}
};
// keypad 핀 배열
uint8_t rowPins[ROWS] = {8, 9, 10}; 
uint8_t colPins[COLS] = {11, 12, 13};
//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

const uint16_t SEG_A = 1 << 0;
const uint16_t SEG_B = 1 << 1;
const uint16_t SEG_C = 1 << 2;
const uint16_t SEG_D = 1 << 3;
const uint16_t SEG_E = 1 << 4;
const uint16_t SEG_F = 1 << 5;
const uint16_t SEG_G = 1 << 6;
const uint16_t SEG_H = 1 << 7;
const uint16_t SEG_J = 1 << 8;
const uint16_t SEG_K = 1 << 9;
const uint16_t SEG_L = 1 << 10;
const uint16_t SEG_M = 1 << 11;
const uint16_t SEG_N = 1 << 12;
const uint16_t SEG_P = 1 << 13;

// 14-seg 문자 패턴 (datasheet 표기 순서 A..P)
const uint16_t CHAR_E = SEG_A | SEG_D | SEG_E | SEG_F | SEG_N | SEG_J;           // E
const uint16_t CHAR_L = SEG_D | SEG_E | SEG_F;                                   // L
const uint16_t CHAR_V = SEG_F | SEG_E | SEG_M | SEG_H;                           // V
const uint16_t CHAR_T = SEG_A | SEG_G | SEG_L;                                   // T
const uint16_t CHAR_S = SEG_A | SEG_F | SEG_N | SEG_J | SEG_C | SEG_D;           // S
const uint16_t CHAR_O = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;           // O
const uint16_t CHAR_K = SEG_F | SEG_E | SEG_N | SEG_H | SEG_K;                   // K
const uint16_t CHAR_I = SEG_A | SEG_D | SEG_G | SEG_L;                           // I
const uint16_t CHAR_F = SEG_A | SEG_E | SEG_F | SEG_J | SEG_N;                   // F
const uint16_t CHAR_U = SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;                   // U
const uint16_t CHAR_P = SEG_A | SEG_B | SEG_E | SEG_F | SEG_J | SEG_N;           // P
const uint16_t CHAR_2 = SEG_A | SEG_B | SEG_J | SEG_N | SEG_E | SEG_D;           // 2
const uint16_t CHAR_N = SEG_F | SEG_E | SEG_P | SEG_K | SEG_B | SEG_C;           // N

enum AppMode : uint8_t {
  MODE_INITIAL = 0,
  MODE_1 = 1,
  MODE_2 = 2,
  MODE_3 = 3
};

uint8_t currentMode = MODE_INITIAL;
const unsigned int BEEP_MODE_SELECT_FREQ = 1000;
const unsigned long BEEP_MODE_SELECT_MS = 100;
const unsigned int BEEP_MODE2_FREQ = 2000;
const unsigned long BEEP_MODE2_MS = 50;
const unsigned int BEEP_MODE3_FREQ = 1000;
const unsigned long BEEP_MODE3_MS = 100;

float currentTemperature = 0.0f;
float currentHumidity = 0.0f;
unsigned long lastDhtReadMs = 0;
const unsigned long dhtReadIntervalMs = 1000;

unsigned long lastModeChange = 0;
const unsigned long modeIntervalMs = 3000;
const unsigned long digitOnMs = 2;

const uint16_t fndPinTestSequence[14] = {
  SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F,
  SEG_G, SEG_H, SEG_J, SEG_K, SEG_L, SEG_M,
  SEG_N, SEG_P
};

uint16_t currentMode1Masks[4] = {0, 0, 0, 0};
bool mode1FndTestDone = false;
uint8_t currentFndDigit = 0;
uint8_t currentFndStep = 0;
unsigned long lastFndTestMs = 0;
const unsigned long fndTestIntervalMs = 300;

// forward declaration for functions defined later in this file
void displayLcd(const char *line0, const char *line1, const char *line2, const char *line3);
void refreshDisplay(const uint16_t masks[4]);
void updateMode1Fnd(unsigned long now);
bool readDHT11(float &temperature, float &humidity)
{
  uint8_t data[5] = {0, 0, 0, 0, 0};

  pinMode(dht11_pin, OUTPUT);
  digitalWrite(dht11_pin, LOW);
  delay(20);
  digitalWrite(dht11_pin, HIGH);
  delayMicroseconds(40);
  pinMode(dht11_pin, INPUT_PULLUP);

  unsigned long timeout = micros();
  while (digitalRead(dht11_pin) == HIGH) {
    if (micros() - timeout > 100) return false;
  }
  timeout = micros();
  while (digitalRead(dht11_pin) == LOW) {
    if (micros() - timeout > 100) return false;
  }
  timeout = micros();
  while (digitalRead(dht11_pin) == HIGH) {
    if (micros() - timeout > 100) return false;
  }

  for (uint8_t i = 0; i < 40; ++i) {
    timeout = micros();
    while (digitalRead(dht11_pin) == LOW) {
      if (micros() - timeout > 100) return false;
    }
    timeout = micros();
    while (digitalRead(dht11_pin) == HIGH) {
      if (micros() - timeout > 100) return false;
    }
    unsigned long pulseLength = micros() - timeout;
    data[i / 8] <<= 1;
    if (pulseLength > 40) {
      data[i / 8] |= 1;
    }
  }

  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    return false;
  }

  humidity = data[0] + data[1] * 0.1f;
  temperature = data[2] + data[3] * 0.1f;
  return true;
}

void updateNeoPixels(float temperature, float humidity)
{
  uint32_t tempColor = (temperature >= 24.0f) ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 255);
  uint32_t humColor = (humidity > 50.0f) ? pixels.Color(255, 165, 0) : pixels.Color(0, 255, 0);

  for (uint8_t i = 0; i < 4; ++i) {
    pixels.setPixelColor(i, tempColor);
  }
  for (uint8_t i = 4; i < 8; ++i) {
    pixels.setPixelColor(i, humColor);
  }
  pixels.show();
}

void updateMode1Fnd(unsigned long now)
{
  if (mode1FndTestDone) {
    return;
  }

  if (now - lastFndTestMs < fndTestIntervalMs) {
    return;
  }

  lastFndTestMs = now;

  currentMode1Masks[0] = 0;
  currentMode1Masks[1] = 0;
  currentMode1Masks[2] = 0;
  currentMode1Masks[3] = 0;
  currentMode1Masks[currentFndDigit] = fndPinTestSequence[currentFndStep];

  if (currentFndDigit == 3 && currentFndStep == 13) {
    mode1FndTestDone = true;
    return;
  }

  currentFndStep++;
  if (currentFndStep >= sizeof(fndPinTestSequence) / sizeof(fndPinTestSequence[0])) {
    currentFndStep = 0;
    currentFndDigit++;
    if (currentFndDigit >= 4) {
      currentFndDigit = 3;
      currentFndStep = 13;
      mode1FndTestDone = true;
    }
  }
}

void enterInitialScreen()
{
  currentMode = MODE_INITIAL;
  displayLcd("  ELEVATORSYSTEM", "   CIRCUIT DESIGN", "   & PROGRAMMING", "    2026.06.13");
}

void showModeScreen(uint8_t mode)
{
  currentMode = mode;
  switch (mode) {
    case MODE_1:
      displayLcd("Mode 1 LCD", "< Pin TESTING >", "Mode 1 FND", "< TEST >");
      currentFndDigit = 0;
      currentFndStep = 0;
      lastFndTestMs = 0;
      mode1FndTestDone = false;
      currentMode1Masks[0] = fndPinTestSequence[0];
      currentMode1Masks[1] = 0;
      currentMode1Masks[2] = 0;
      currentMode1Masks[3] = 0;
      break;
    case MODE_2:
      displayLcd("      MODE 2     ", "    Floor Calls   ", "  SW4~SW8 beep", "  SW9 returns   ");
      break;
    case MODE_3:
      displayLcd("      MODE 3     ", "   Keypad Testing ", "  keypad key beep", "  SW9 returns   ");
      break;
    default:
      enterInitialScreen();
      break;
  }
}

void handleButtonPress(char key)
{
  switch (currentMode) {
    case MODE_INITIAL:
      if (key == '1' || key == '2' || key == '3') {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        showModeScreen(key - '0');
      }
      break;
    case MODE_1:
      if (key == '9') {
        tone(buzzer_pin, BEEP_MODE_SELECT_FREQ, BEEP_MODE_SELECT_MS);
        enterInitialScreen();
      }
      break;
    case MODE_2:
      if (key >= '4' && key <= '8') {
        tone(buzzer_pin, BEEP_MODE2_FREQ, BEEP_MODE2_MS);
      } else if (key == '9') {
        enterInitialScreen();
      }
      break;
    case MODE_3:
      if (key >= '1' && key <= '8') {
        tone(buzzer_pin, BEEP_MODE3_FREQ, BEEP_MODE3_MS);
      } else if (key == '9') {
        enterInitialScreen();
      }
      break;
  }
}

void enableAllDigits(bool on)
{
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(digit_pins[i], on ? HIGH : LOW);
  }
}

void writeDigitSegments(uint8_t digit, uint16_t mask)
{
  const uint8_t *pins = (digit < 2) ? seg_pins1 : seg_pins2;
  for (uint8_t i = 0; i < 14; i++) {
    digitalWrite(pins[i], (mask & (1 << i)) ? LOW : HIGH);
  }
}

void refreshDisplay(const uint16_t masks[4])
{
  for (uint8_t digit = 0; digit < 2; digit++) {
    writeDigitSegments(digit, masks[digit]);
    writeDigitSegments(digit+2, masks[digit+2]);
    digitalWrite(digit_pins[digit], HIGH);
    digitalWrite(digit_pins[digit+2], HIGH);
    delay(digitOnMs);
    digitalWrite(digit_pins[digit], LOW);
    digitalWrite(digit_pins[digit+2], LOW);
  }
}

void refreshDisplay( uint16_t masks0, uint16_t masks1, uint16_t masks2, uint16_t masks3)
{
  const uint16_t masks[4] = {masks0, masks1, masks2, masks3};
  for (uint8_t digit = 0; digit < 2; digit++) {
    writeDigitSegments(digit, masks[digit]);
    writeDigitSegments(digit+2, masks[digit+2]);
    digitalWrite(digit_pins[digit], HIGH);
    digitalWrite(digit_pins[digit+2], HIGH);
    delay(digitOnMs);
    digitalWrite(digit_pins[digit], LOW);
    digitalWrite(digit_pins[digit+2], LOW);
  }
}

void displayLcd(const char *line0, const char *line1, const char *line2, const char *line3)
{
	lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
  lcd.setCursor(0, 2);
  lcd.print(line2);
  lcd.setCursor(0, 3);
  lcd.print(line3);
}

void initDisplay(void)
{
	displayLcd("  ELEVATORSYSTEM", "   CIRCUIT DESIGN", "   & PROGRAMMING", "    2026.06.13");
}

void setup() {
  Serial.begin(9600);
  Serial.println("System initializing...");

  lcd.begin();
  lcd.backlight();
  lcd.clear();

  for (uint8_t i = 0; i < 14; i++) {
    pinMode(seg_pins1[i], OUTPUT);
    digitalWrite(seg_pins1[i], HIGH);
    pinMode(seg_pins2[i], OUTPUT);
    digitalWrite(seg_pins2[i], HIGH);
  }

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(digit_pins[i], OUTPUT);
    digitalWrite(digit_pins[i], LOW);
  }

  pinMode(buzzer_pin, OUTPUT);
  pixels.begin();
  pixels.clear();
  tone(buzzer_pin, 440, 500);
  delay(500);
  noTone(buzzer_pin);

  displayLcd("  ELEVATORSYSTEM", "   CIRCUIT DESIGN", "   & PROGRAMMING", "    2026.06.13");
}

void loop() {
  char customKey = customKeypad.getKey();
  static char lastKey = 0;
  unsigned long now = millis();

  if (customKey && customKey != lastKey) {
    Serial.println(customKey);
    handleButtonPress(customKey);
  }

  if (currentMode == MODE_1) {
    updateMode1Fnd(now);
    refreshDisplay(currentMode1Masks);
  } else {
    refreshDisplay(CHAR_E, CHAR_L, CHAR_E, CHAR_V);
  }

  if (currentMode != MODE_2) {
    if (now - lastDhtReadMs >= dhtReadIntervalMs) {
      float temp, hum;
      if (readDHT11(temp, hum)) {
        currentTemperature = temp;
        currentHumidity = hum;
      }
      updateNeoPixels(currentTemperature, currentHumidity);
      lastDhtReadMs = now;
    }
  } else {
    pixels.clear();
  }

  lastKey = customKey;
}

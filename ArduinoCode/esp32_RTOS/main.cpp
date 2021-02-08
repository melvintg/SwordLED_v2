#include <Arduino.h>

#include <Wire.h>
#include <FastLED.h>

static const uint8_t PIN_TS = 4;          // Touch Sensor
static const uint8_t PIN_LED_W = 15;       // LED White for Touch Sensor 8
static const uint8_t PIN_LED_G = 23;       // LED Green for ATmega328P
static const uint8_t PIN_BZ = 19;          // Buzzer
static const uint8_t PIN_LEDS = 18;        // LEDs WS2812B Data pin
static const uint8_t PIN_LED_R = 5;       // LED Red for Battery check
static const uint8_t PIN_TXB = 17;        // Tx Bluetooth
static const uint8_t PIN_RXB = 16;        // Rx Bluetooth
static const uint8_t PIN_BAT_CH = 2;     // Charge pin LOW/HIGH
static const uint8_t PIN_BAT = 0;        // Battery level

//#define MPU6050
#define BT06

#ifdef BT06
  //HardwareSerial BT(2);
  #include "BluetoothSerial.h"
  BluetoothSerial BT;
  void xBT(void *pvParam);
#endif

static const int VBAT = 421;
static const uint8_t TS_THRESHOLD = 40;

static const int DELAY_LED_R_FAST = 200;
static const int DELAY_LED_R_SLOW = 500;
static const uint8_t DELAY_LED_G = 200;
static const uint8_t DELAY_IMPACT = 15;
static const uint8_t DELAY_BAT_CHECK = 200;
static const uint8_t DELAY_BZ = 4;
static const uint8_t DELAY_LED_FADE_ON = 10;
static const uint8_t DELAY_LED_FADE_OFF = 10;
static const uint8_t DELAY_LED_WAVE = 4;
static const uint8_t DELAY_HG_WAVE = 4;
static const uint8_t DELAY_HG_BLINK = 80;
static const uint8_t DELAY_FAST_BLINK = 15;
static const uint8_t DELAY_BT = 5;
static const uint8_t DELAY_MPU6050 = 5;

// Bluetooth global variables
static volatile boolean g_bBTa = false;

// MPU6050 global variables
#ifdef MPU6050
typedef struct Accelerometer {
  float S;       // Sensitivity
  float X;
  float Y;
  float Z;
  float angleX;
  float angleY;
  float errorX;
  float errorY;
  unsigned long elapsedTime;
  unsigned long currentTime;
  unsigned long PrevTime;
} Acc_t;
static Acc_t *g_pAcc;

typedef struct Gyroscope {
  float S;       // Sensitivity
  float X;
  float Y;
  float Z;
  float angleX;
  float angleY;
  float angleZ;
  float errorX;
  float errorY;
  float errorZ;
  float Roll;
  float Pitch;
  float Yaw;
} Gyro_t;
static Gyro_t *g_pGyro;

void xReadMPU6050(void *pvParam);
#endif

// LED animations using Accel.
typedef struct AccelAnimations {
  byte nImact;        // Impact Action
  boolean bAgr;       // Agressive Action
  boolean impDetect;  // Impact detected
  boolean agrDetect;  // Agressive detected
  boolean bLG;        // Low guard
  boolean bDG;        // Defensive guard
  boolean bHG;        // High guard
  byte tHG;           // High guard wave time
  byte delayTime;     // High guard refresh time
  int nHG;            // High guard counter
} AccFx_t;
static AccFx_t *g_pAccFx;

// LED fade Animation variables
typedef struct LedFade {
  byte progress;
  byte waveCount;
  boolean isWaveInv;
  byte delayTime;
  byte fadeValue;
} LedFade_t;
static LedFade_t *g_pLedFade;

// LED wave Animation variables
typedef struct LedWave {
  byte progress = 0;
  byte delayTime = 0;
  byte fadeValue = 0;
} LedWave_t;
static LedWave_t *g_pLedWave;

// Battery Check variables
typedef struct BatCheck {
  boolean active;
  boolean mantained;
  boolean led;
  int delayTime;
} BatCheck_t;
static BatCheck_t *pBatCheck;

// WS2812B Constants and variables
static const uint8_t NUM_LEDS = 44;
static const uint8_t NUM_COLOR = 5;
static CRGB Leds[NUM_LEDS];
static CRGB TargetLeds[NUM_LEDS];
static const CRGB COLOR[NUM_COLOR] = {CRGB::White, CRGB::Orange, CRGB::Blue, CRGB::HotPink,  CRGB::Green};
static volatile uint8_t ColorPos;
static CRGB g_CurrentColor;

// Constant definitions
static const uint8_t MPU = 0x68;
static const uint8_t NUM_IMPACT = 14;

// Global flags variables
static volatile boolean g_bTs;
static volatile boolean g_bMantainedTS;
static volatile boolean g_bLEDs;
static volatile boolean g_bLEDg;
static volatile boolean g_bSword;
static volatile boolean g_bFastLedBlink;
static volatile boolean g_bBz;

// Global variables
static volatile unsigned long g_time;
static volatile unsigned long g_touchTime;
static volatile uint8_t g_nTouch;
static volatile int g_nBz;

void setupVariables(void);

// Touch sensor function prototypes
void touchSensor(void);

// LED on/off function prototypes
void ledON(CRGB color, int nLeds);
void ledOFF(void);

int askBat(void);

void startLedWave(byte fadeVal, byte delayTime);
void startLedFade(CRGB color, byte fadeVal, byte delayTime, int wave);
void ledFadeAll(int fadeVal);
CRGB fadeTowardColor( CRGB& cur, const CRGB& target, uint8_t amount);
void nblendU8TowardU8( uint8_t& cur, const uint8_t target, uint8_t amount);

void xCheckTS(void *parameter);
void xLedGreen(void *parameter);
void xCheckBattery(void *pvParam);
void xFastLedBlink(void *pvparam);
void xAskBat(void *pvParam);
void xSwordImpact(void *pvParam);
void xLedWave(void *pvParam);
void xLedFade(void *pvParam);

void startBzFade(boolean isFadeON);
void xBzFadeON(void *pvParam);
void xBzFadeOFF(void *pvParam);
void startBzWave(boolean isInv);
void xBzWave(void *pvParam);
void xBzWaveInv(void *pvParam);
void bzBlink(int f);
void startLedHighGuard(void);
void xLedHighGuard(void *pvParam);

static TaskHandle_t xAskBat_Hdlr;
static TaskHandle_t xLedsBlink_Hdlr;

static SemaphoreHandle_t LedsMutex;
static SemaphoreHandle_t BzMutex;

void setupVariables() {

#ifdef MPU6050
g_pAcc = (Acc_t*) pvPortMalloc(sizeof(Acc_t));
g_pGyro = (Gyro_t*) pvPortMalloc(sizeof(Gyro_t));
#endif

  g_pLedFade = (LedFade_t*) pvPortMalloc(sizeof(LedFade_t));
  g_pLedWave = (LedWave_t*) pvPortMalloc(sizeof(LedWave_t));
  pBatCheck = (BatCheck_t*) pvPortMalloc(sizeof(BatCheck_t));
  pBatCheck->active = 0;
  pBatCheck->delayTime = DELAY_LED_R_FAST;
  pBatCheck->led = false;


  // LED animations using Accel.
  g_pAccFx = (AccFx_t*) pvPortMalloc(sizeof(AccFx_t));
  g_pAccFx->nImact = 0;
  g_pAccFx->nHG = 0;
  g_pAccFx->tHG = DELAY_HG_WAVE;
  g_pAccFx->delayTime = DELAY_HG_BLINK;
  g_pAccFx->bHG = false;
  g_pAccFx->bLG = false;
  g_pAccFx->bDG = false;
  g_pAccFx->bAgr = false;
  g_pAccFx->impDetect = false;
  g_pAccFx->agrDetect = false;

  g_bLEDs = false;
  g_bLEDg = true;
  g_bSword = false;
  g_bBz = false;
  g_bFastLedBlink = false;
  g_bTs = false;
  g_bMantainedTS = false;

  g_nTouch = 0;
  g_nBz = 0;
  g_time =  0;
  g_touchTime = 0;
}

void setup() {

  setupVariables();

  // Configure pin
  pinMode(PIN_TS, INPUT);
  pinMode(PIN_LED_W, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LEDS, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BAT_CH, INPUT);
  pinMode(PIN_BAT, INPUT);

  digitalWrite(PIN_LED_W, LOW);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_R, LOW);

  ledcSetup(0, 2000, 12);
  ledcAttachPin(PIN_BZ,0);

  touchAttachInterrupt(PIN_TS, touchSensor, TS_THRESHOLD);

  // See https://hackaday.com/2017/01/20/cheating-at-5v-ws2812-control-to-use-a-3-3v-data-line/
  FastLED.addLeds<WS2812, PIN_LEDS, GRB>(Leds, NUM_LEDS);
  LEDS.setBrightness(180);
  ColorPos = 0;
  g_CurrentColor = COLOR[ColorPos];
  ledOFF();

#ifdef MPU6050
  mpu6050setup();
#endif

#ifdef BT06
  BT.begin("esp32");
#endif

  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Starting Sword");

  LedsMutex = xSemaphoreCreateMutex();
  BzMutex = xSemaphoreCreateMutex();

  // Start blink task
  xTaskCreatePinnedToCore(      // Use xTaskCreate() in vanilla FreeRTOS
            xLedGreen,          // Function to be called
            "Blink Green LED",  // Name of task
            1024,               // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,               // Parameter to pass
            1,                  // Task priority
            NULL,               // Task handle
            1);                 // Run on core one

  xTaskCreatePinnedToCore(          // Use xTaskCreate() in vanilla FreeRTOS
            xCheckTS,               // Function to be called
            "Check Touch Sensor",   // Name of task
            2048,                   // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,                   // Parameter to pass
            2,                      // Task priority
            NULL,                   // Task handle
            1);                     // Run on core one

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
            xCheckBattery,    // Function to be called
            "Check Battery",  // Name of task
            1024,             // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,             // Parameter to pass
            1,                // Task priority
            NULL,             // Task handle
            1);               // Run on core one

#ifdef BT06
  xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS
            xBT,              // Function to be called
            "Read BT",        // Name of task
            2024,             // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,             // Parameter to pass
            2,                // Task priority
            NULL,             // Task handle
            0);               // Run on core zero
#endif

#ifdef MPU6050
  xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS
            xReadMPU6050,     // Function to be called
            "Read MPU6050",   // Name of task
            3072,             // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,             // Parameter to pass
            1,                // Task priority
            NULL,             // Task handle
            0);               // Run on core zero
#endif


  // If this was vanilla FreeRTOS, you'd want to call vTaskStartScheduler() in
  // main after setting up your tasks.

  // Delete "setup and loop" task
  vTaskDelete(NULL);
}


void loop() {
}

/* Interrupt function for touch sensor */
void touchSensor() {
  if (!g_bTs) {
    g_bTs = true;
    g_touchTime = millis();
    g_time = 0;

    // Update state
    digitalWrite(PIN_LED_W, g_bTs);
    Serial.println(g_bTs);
  }
}

/****************************** LEDs functions ********************************

##############################################################################*/

/**************************** ON/OFF LEDs function ***************************/
void ledON(CRGB color, int nLeds) {
  g_CurrentColor = color;
  for(int i = 0; i < nLeds; i++) {
      Leds[i] = color;
  }
  for (int i = nLeds; i < NUM_LEDS; i++) {
    Leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void ledOFF() {
  for(int i = 0; i < NUM_LEDS; i++) {
      Leds[i] = CRGB::Black;
  }
  FastLED.show();
}

/* Perfomr a pulse led wave animation */
void ledPulse() {
  if (g_bSword) {
    Serial.println("Pulse led wave animation...");
    startLedWave(235, DELAY_LED_WAVE);
    startBzWave(true);
  }
}

void ledDifuseChange() {
  Serial.println("Changing base color...");
  ColorPos++;
  if (ColorPos == NUM_COLOR) ColorPos = 0;

  if (g_bSword) {
    startLedFade(COLOR[ColorPos], 8, 0, 0);
    startBzWave(false);
  }
}

void ledFadeON() {
  Serial.println("Switching ON Sword..");
  g_bLEDs = true;
  g_bSword = true;
  ColorPos = 0;
  startLedFade(COLOR[ColorPos], 2, DELAY_LED_FADE_ON, 1);
  startBzFade(true);
}

void ledFadeOFF() {
  Serial.println("Switching OFF Sword..");
  g_bLEDs = false;
  g_bSword = false;
  startLedFade(CRGB::Black, 12, DELAY_LED_FADE_OFF, -1);
  startBzFade(false);
}

void ledNormal() {
  if (g_bSword) {
    Serial.println("Sword back to normal");
    g_bLEDs = true;
    startLedFade(COLOR[ColorPos], 8, 0, 0);
  }
  ledcWrite(0, 0);
}

// Fast lEDs blink motion
void fastLedBlink(CRGB color) {
  g_bLEDs = !g_bLEDs;
  if (g_bLEDs) {
    ledON(color, NUM_LEDS);
  } else {
    ledOFF();
  }
}

void ledAgressive() {
  Serial.println("Sword movement detected...");
  startLedFade(CRGB::Red, 8, 0, 1);
  startBzWave(true);
}

void ledLowGuard() {
  // Difuse to Green
  startLedFade(CRGB::Green, 8, 0, 1);
  startBzWave(false);
}

void ledDefenseGuard() {
  // Difuse to Blue
  startLedFade(CRGB::Blue, 8, 0, 1);
  startBzWave(false);
}


//*****************************************************************************
// Tasks
//*****************************************************************************

void xLedGreen(void *pvParam) {
  (void) pvParam;

  while (1) {
    // Blink Green LED while normal operation (excludes battery charging)
    if (digitalRead(PIN_BAT_CH)) {
      digitalWrite(PIN_LED_G, LOW);
    } else {
      g_bLEDg = !g_bLEDg;
      digitalWrite(PIN_LED_G, g_bLEDg);
    }
    vTaskDelay(DELAY_LED_G / portTICK_PERIOD_MS);
  }

  // Cannot be here. Delete task.
  vTaskDelete(NULL);
}

void xCheckTS(void *pvParam) {
  (void) pvParam;
  while (1) {

    // If timer is ON, update timer.
    if (g_touchTime != 0) g_time = millis() - g_touchTime;

    if (touchRead(PIN_TS) > TS_THRESHOLD && g_bTs) {
      // Touch sensor released.
      g_bTs = false;

      // Check if it is a pulse or a maintained touch
      if (!g_bMantainedTS)  g_nTouch++;
      else                  g_touchTime = 0;
      g_bMantainedTS = false;

      // Battery check mantained press is released. End Battery check.
      if (pBatCheck->active) {
        Serial.println("End battery check.");
        pBatCheck->active = false;
        g_nTouch = 0;
        xSemaphoreGive(LedsMutex);
        vTaskDelete(xAskBat_Hdlr);
        ledNormal();
      }

        // Update state
      digitalWrite(PIN_LED_W, g_bTs);
      Serial.println(g_bTs);
    }

      // Touch Sensor High. Check for Maintained touch
    if (g_bTs) {

      // Check if Touch Sensor is mantained for 1000 ms
      if (g_time > 1000 && !g_bMantainedTS) {
        // Button maintained
        g_bMantainedTS = true;
        // Stop Timer
        g_touchTime = 0;
        // If only was one touch previously
        if (g_nTouch > 0) {
          Serial.println("1 press & 1 mantained Touch Sensor");
          pBatCheck->active = true;
          // Battery status Blink
          xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
                    xAskBat,        // Function to be called
                    "Ask bat",      // Name of task
                    1024,           // Stack size (bytes in ESP32, words in FreeRTOS)
                    NULL,           // Parameter to pass
                    3,              // Task priority
                    &xAskBat_Hdlr,  // Task handle
                    1);             // Run on core one
        } else {
          Serial.println("1 mantained Touch Sensor");
          // Only maintained (ON/OFF)
          g_bSword = !g_bSword;
          if (g_bSword)  ledFadeON();
          else          ledFadeOFF();
        }
      }

    } else {

      // Touch Sensor Low. Check for n press.
      if (g_nTouch != 0) {
        // Wait for another Touch Sensor press. 200 ms
        if (g_time > 200) {
          // Stop Timer
          g_touchTime = 0;
          if (g_nTouch == 1) {
            Serial.println("1 press Touch Sensor");
            ledPulse();
          } else if (g_nTouch == 2) {
            Serial.println("2 press Touch Sensor");
            ledDifuseChange();
          } else {
            Serial.println("3 press Touch Sensor");
            g_bFastLedBlink = !g_bFastLedBlink;
            if (g_bFastLedBlink) {
              // Fast lEDs blink motion
              xTaskCreatePinnedToCore(        // Use xTaskCreate() in vanilla FreeRTOS
                        xFastLedBlink,        // Function to be called
                        "Fast LED blink",     // Name of task
                        1024,                 // Stack size (bytes in ESP32, words in FreeRTOS)
                        NULL,                 // Parameter to pass
                        3,                    // Task priority
                        &xLedsBlink_Hdlr,     // Task handle
                        1);                   // Run on core one
            } else {
              xSemaphoreGive(LedsMutex);
              vTaskDelete(xLedsBlink_Hdlr);
              ledNormal();
            }
          }
          g_nTouch = 0;
        }
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  // Cannot be here. Delete task.
  vTaskDelete(NULL);
}

/* Indicates the battery state using a red LED blink animation */
void xCheckBattery(void *pvParam) {
  (void) pvParam;
  while(1) {
    int batValue = analogRead(PIN_BAT);
    // Due to NMOS voltage drop, each BAT value is previously measured.
    // Scale analog reading to voltage value up to two decimals.
    // 0-4095 -> 0-3.300V
    int val = map(batValue, 0, 4095, 0, 3300);

    // 100% -> 4.2Vbat | 80% -> 3.9Vbat | 60% -> 3.8Vbat | 40% -> 3.7Vbat | 20% -> 3.6Vbat | 0% -> 3Vbat
    if (digitalRead(PIN_BAT_CH)) {
      // Charge mode
      if (val < VBAT-7*4) { // 7.5 Vbat -> 3.72v
        // LED_R blink slow for battery 0%-50%
        pBatCheck->delayTime = DELAY_LED_R_SLOW;

        pBatCheck->led = !pBatCheck->led;
        digitalWrite(PIN_LED_R, pBatCheck->led);

      } else if (val < VBAT+20) { // 7.8 Vbat -> 3.89v
        // LED_R blink fast for battery 50%-90%
        pBatCheck->delayTime = DELAY_LED_R_FAST;

        pBatCheck->led = !pBatCheck->led;
        digitalWrite(PIN_LED_R, pBatCheck->led);
      } else {
        // LED_R ON for battery 80%-100%
        digitalWrite(PIN_LED_R, HIGH);
      }
    } else {
      // Normal mode
      if (val > VBAT-7*5) { // 7.4 Vbat -> 3.64v
        // LED_R OFF for battery 100%-40%
        digitalWrite(PIN_LED_R, LOW);

      } else if (val > VBAT-7*7) { // 7.2 Vbat -> 3.52v
        // LED_R blink slow for battery 40%-20%
        pBatCheck->delayTime = DELAY_LED_R_SLOW;

        pBatCheck->led = !pBatCheck->led;
        digitalWrite(PIN_LED_R, pBatCheck->led);

      } else {
        // LED_R blink fast for battery 20%-0%
        pBatCheck->delayTime = DELAY_LED_R_FAST;

        pBatCheck->led = !pBatCheck->led;
        digitalWrite(PIN_LED_R, pBatCheck->led);
      }
    }
   vTaskDelay(pBatCheck->delayTime / portTICK_PERIOD_MS);
  }
  // Cannot be here. Delete task.
  vTaskDelete(NULL);
}


/******************************* Sword animation *****************************
##############################################################################

##############################################################################*/

// Fast lEDs blink motion
void xFastLedBlink(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(LedsMutex, portMAX_DELAY);

  while(1) {
  	fastLedBlink(g_CurrentColor);
  	vTaskDelay(DELAY_FAST_BLINK / portTICK_PERIOD_MS);
  }

  // Cannot be here. Delete task.
  vTaskDelete(NULL);
}


/************************** Battery Check animation **************************/
void xAskBat(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(LedsMutex, portMAX_DELAY);
  while(1) {
    // Battery check with Touch Sensor
    int nLEDs = askBat();

    if (g_bLEDs) {
      ledON(CRGB::Red, nLEDs);
    } else {
      ledOFF();
    }
    g_bLEDs = !g_bLEDs;

    vTaskDelay(DELAY_BAT_CHECK / portTICK_PERIOD_MS);
  }
  // Cannot be here. Delete task.
  vTaskDelete(NULL);

}

/*************************** Impact Sword animation **************************/
/* Impact detected. Start led and buzzer blink animation. */
void startSwordImpact() {
  g_pAccFx->nImact = NUM_IMPACT;
  xTaskCreatePinnedToCore(        // Use xTaskCreate() in vanilla FreeRTOS
            xSwordImpact,         // Function to be called
            "Impact",             // Name of task
            1024,                 // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,                 // Parameter to pass
            4,                    // Task priority
            NULL,                 // Task handle
            1);                   // Run on core one
}


void xSwordImpact(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(LedsMutex, portMAX_DELAY);
  xSemaphoreTake(BzMutex, portMAX_DELAY);

  while (g_pAccFx->nImact > 1) {
    fastLedBlink(CRGB::Yellow);
    bzBlink(2000);
    g_pAccFx->nImact--;

    vTaskDelay(DELAY_IMPACT / portTICK_PERIOD_MS);
  }

  xSemaphoreGive(LedsMutex);
  xSemaphoreGive(BzMutex);

  // Last impact. Get to Normal
  ledNormal();
  g_pAccFx->nImact = 0;

  // Delete Task
  vTaskDelete(NULL);
}

/***************************** Wave LED animation ****************************/
void startLedWave(byte fadeVal, byte delayTime) {

  Serial.println("Starting Wave...");

  g_pLedWave->delayTime = delayTime;
  if (delayTime == 0) g_pLedWave->delayTime = 1;   // Some delay for stability
  g_pLedWave->fadeValue = fadeVal;
  g_pLedWave->progress = 0;

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
            xLedWave,       // Function to be called
            "Led Wave",     // Name of task
            1024,           // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,           // Parameter to pass
            2,              // Task priority
            NULL,           // Task handle
            1);             // Run on core one
}

void xLedWave(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(LedsMutex, portMAX_DELAY);
  while (g_pLedWave->progress < NUM_LEDS) {
    Leds[g_pLedWave->progress].maximizeBrightness();
    ledFadeAll(g_pLedWave->fadeValue);
    FastLED.show();
    g_pLedWave->progress++;

    vTaskDelay(g_pLedWave->delayTime / portTICK_PERIOD_MS);
  }
  xSemaphoreGive(LedsMutex);
  // vPortFree(g_pLedWave);
  Serial.println("Ending Wave...");

  startLedFade(g_CurrentColor, 8, 0, 0);

  // Delete Task
  vTaskDelete(NULL);
}

/* Fade all LEDs value by (fadeVal/256ths) of its previous value
   Doc. https://github.com/FastLED/FastLED/wiki/Pixel-reference */
void ledFadeAll(int fadeVal) {
  for(int i = 0; i < NUM_LEDS; i++) {
    Leds[i].nscale8_video(fadeVal);
  }
}

/*********************** Progressive LED fade animation **********************/
/* Start LED fade animation by setting global variables and setting a mutex */
void startLedFade(CRGB color, byte fadeVal, byte delayTime, int wave) {
  g_CurrentColor = color;

  Serial.println("Starting Fade...");

  g_pLedFade->progress = 0;
  g_pLedFade->delayTime = delayTime;
  if (delayTime == 0) g_pLedFade->delayTime = 1;   // Some delay for stability
  g_pLedFade->fadeValue = fadeVal;

  /* wave = 0 -> No wave
   * wave > 0 -> Bottom to Top wave
   * wave < 0 -> Top to bottom wave
   */
  g_pLedFade->waveCount = NUM_LEDS;
  if (wave > 0) {
    g_pLedFade->waveCount = 1;
  }

  if (wave < 0) {
    g_pLedFade->isWaveInv = true;
  } else {
    g_pLedFade->isWaveInv = false;
  }

  for(int i = 0; i < NUM_LEDS; i++) {
      TargetLeds[i] = color;
  }

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
            xLedFade,       // Function to be called
            "Led Fade",     // Name of task
            1024,           // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,           // Parameter to pass
            2,              // Task priority
            NULL,           // Task handle
            1);             // Run on core one
}

/* LED fade animation task executed in main loop every g_pLedFade.delayTime*/
void xLedFade(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(LedsMutex, portMAX_DELAY);
  while (g_pLedFade->progress < NUM_LEDS) {
    g_pLedFade->progress = 0;
    if (g_pLedFade->isWaveInv) {
      for(int i = NUM_LEDS-1; i >= g_pLedFade->waveCount; i--){
        fadeTowardColor(Leds[i], TargetLeds[i], g_pLedFade->fadeValue);
        if (Leds[i].red == TargetLeds[i].red
            && Leds[i].green == TargetLeds[i].green
            && Leds[i].blue == TargetLeds[i].blue){
          g_pLedFade->progress++;
        }
      }
      if (g_pLedFade->waveCount > 0) g_pLedFade->waveCount--;

    } else {
      for(int i = 0; i < g_pLedFade->waveCount; i++){
        fadeTowardColor(Leds[i], TargetLeds[i], g_pLedFade->fadeValue);
        if (Leds[i].red == TargetLeds[i].red
            && Leds[i].green == TargetLeds[i].green
            && Leds[i].blue == TargetLeds[i].blue){
          g_pLedFade->progress++;
        }
      }
      if (g_pLedFade->waveCount < NUM_LEDS) g_pLedFade->waveCount++;
    }
    FastLED.show();
    vTaskDelay(g_pLedFade->delayTime / portTICK_PERIOD_MS);
  }

  xSemaphoreGive(LedsMutex);
  // vPortFree(g_pLedFade);
  Serial.println("Ending Fade...");

  // Delete Task
  vTaskDelete(NULL);
}

// Blend one CRGB color toward another CRGB color by a given amount.
// Blending is linear, and done in the RGB color space.
// This function modifies 'cur' in place.
CRGB fadeTowardColor( CRGB& cur, const CRGB& target, uint8_t amount) {
  nblendU8TowardU8( cur.red,   target.red,   amount);
  nblendU8TowardU8( cur.green, target.green, amount);
  nblendU8TowardU8( cur.blue,  target.blue,  amount);
  return cur;
}

// Helper function that blends one uint8_t toward another by a given amount
void nblendU8TowardU8( uint8_t& cur, const uint8_t target, uint8_t amount) {
  if( cur == target) return;

  if( cur < target ) {
    uint8_t delta = target - cur;
    delta = scale8_video( delta, amount);
    cur += delta;
  } else {
    uint8_t delta = cur - target;
    delta = scale8_video( delta, amount);
    cur -= delta;
  }
}

/****************************** Buzzer functions *****************************

##############################################################################*/
/* Start fade Buzzer. Used for ON/OFF Sword, */
void startBzFade(boolean isFadeON) {
  g_bBz = true;

  if (isFadeON) {
    g_nBz = 0;
    xTaskCreatePinnedToCore(      // Use xTaskCreate() in vanilla FreeRTOS
              xBzFadeON,          // Function to be called
              "Buzzer Fade ON",   // Name of task
              1024,               // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,               // Parameter to pass
              2,                  // Task priority
              NULL,               // Task handle
              1);                 // Run on core one
  } else {
    g_nBz = 5000;
    xTaskCreatePinnedToCore(      // Use xTaskCreate() in vanilla FreeRTOS
              xBzFadeOFF,         // Function to be called
              "Buzzer Fade OFF",  // Name of task
              1024,               // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,               // Parameter to pass
              2,                  // Task priority
              NULL,               // Task handle
              1);                 // Run on core one
  }
}

/* Buzzer fade ON task executed in main loop every gBz.delayTime */
void xBzFadeON(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(BzMutex, portMAX_DELAY);
  while  (g_nBz < 5000) {
      if (g_nBz < 2000) {
        g_nBz += 5;
      } else {
        g_nBz += g_nBz/200;
      }
      if (g_bBz) {
        ledcWriteTone(0, g_nBz);
      } else {
        ledcWrite(0, 0);
      }
      g_bBz = !g_bBz;
      vTaskDelay(DELAY_BZ / portTICK_PERIOD_MS);
    }

    ledcWrite(0, 0);
    xSemaphoreGive(BzMutex);

    // Delete Task
    vTaskDelete(NULL);
}

/* Buzzer fade OFF task executed in main loop every gBz.delayTime */
void xBzFadeOFF(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(BzMutex, portMAX_DELAY);
  while  (g_nBz > 100) {
    if (g_nBz < 1000) {
      g_nBz -= 10;
    } else {
      g_nBz -= g_nBz/50;
    }
    if (g_bBz) {
      ledcWriteTone(0, g_nBz);
    } else {
      ledcWrite(0, 0);
    }
    g_bBz = !g_bBz;
    vTaskDelay(DELAY_BZ / portTICK_PERIOD_MS);
  }

  ledcWrite(0, 0);
  xSemaphoreGive(BzMutex);

  // Delete Task
  vTaskDelete(NULL);
}


/* Start fade Buzzer. Used for ON/OFF Sword and pulse wave animations. */
void startBzWave(boolean isInv) {
  g_bBz = true;

  if (isInv) {
    g_nBz = 5000;
    xTaskCreatePinnedToCore(      // Use xTaskCreate() in vanilla FreeRTOS
              xBzWaveInv,         // Function to be called
              "Buzzer Wave Inv",  // Name of task
              1024,               // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,               // Parameter to pass
              2,                  // Task priority
              NULL,               // Task handle
              0);                 // Run on core zero

  } else {
    g_nBz = 0;
    xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              xBzWave,        // Function to be called
              "Buzzer Wave",  // Name of task
              1024,           // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,           // Parameter to pass
              2,              // Task priority
              NULL,           // Task handle
              1);             // Run on core one
  }
}

/* Buzzer fade inverted pulse wave task executed in main loop every gBz.delayTime */
void xBzWave(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(BzMutex, portMAX_DELAY);
  while (g_nBz < 5000) {
      if (g_nBz < 2000) {
        g_nBz += 100;
      } else {
        g_nBz += g_nBz/10;
      }

      if (g_bBz) {
        ledcWriteTone(0, g_nBz);
      } else {
        ledcWrite(0, 0);
      }
      g_bBz = !g_bBz;
      vTaskDelay(DELAY_BZ / portTICK_PERIOD_MS);
    }

    ledcWrite(0, 0);
    xSemaphoreGive(BzMutex);

    // Delete Task
    vTaskDelete(NULL);
}

/* Buzzer fade pulse wave task executed in main loop every gBz.delayTime */
void xBzWaveInv(void *pvParam) {
  (void) pvParam;

  xSemaphoreTake(BzMutex, portMAX_DELAY);
  while(g_nBz > 100) {
    if (g_nBz < 1000) {
      g_nBz -= 50;
    } else {
      g_nBz -= g_nBz/10;
    }
    if (g_bBz) {
      ledcWriteTone(0, g_nBz);
    } else {
      ledcWrite(0, 0);
    }
    g_bBz = !g_bBz;
    vTaskDelay(DELAY_BZ / portTICK_PERIOD_MS);
  }
  ledcWrite(0, 0);
  xSemaphoreGive(BzMutex);

  // Delete Task
  vTaskDelete(NULL);
}

// Beeping Buzzer motion
void bzBlink(int f) {
  if (g_bLEDs) {
    ledcWriteTone(0, f);
  } else {
    ledcWrite(0, 0);
  }
}

void startLedHighGuard() {
  Serial.println("High guard...");
  g_pAccFx->bHG = true;
  g_pAccFx->nHG = 0;
  g_pAccFx->tHG = DELAY_HG_WAVE;
  g_pAccFx->delayTime = DELAY_HG_BLINK;
}

void xLedHighGuard() {
  bzBlink(2000);

  // To active, use g_pAccFx->bHG = true
  if (g_pAccFx->nHG == 0) {
      startLedFade(CRGB::Blue,50,0,0);
  } else if (g_pAccFx->tHG > 0) {
      startLedWave(235, g_pAccFx->tHG);
      // Speed the animation
      if (g_pAccFx->nHG % 2 == 0) {
        g_pAccFx->tHG--;
      }
  } else {
    fastLedBlink(CRGB::Blue);
    if (g_pAccFx->delayTime > 30) {
        g_pAccFx->delayTime--;
    } else if (g_pAccFx->nHG % 4 == 0 && g_pAccFx->delayTime > 10) {
      g_pAccFx->delayTime--;
    }
  }
  g_pAccFx->nHG++;
  if (g_pAccFx->nHG > 30000) {
    g_pAccFx->nHG = 50;
  }
}

#ifdef BT06
void xBT(void *pvParam) {
  (void) pvParam;

  while(1) {
    if(BT.available()) {
      char c = BT.read();
      switch(c) {
        case 't':
          // Test case
          break;

        // Accel animations
        case '1':
          startLedHighGuard();
          break;
        case '2':
          ledDefenseGuard();
          break;
        case '3':
          ledLowGuard();
          break;
        case '4':
          ledAgressive();
          break;

          case 'f':
          // Fast Led Blink
          g_bFastLedBlink = !g_bFastLedBlink;
          if (!g_bFastLedBlink) ledNormal();
          break;

        case 'n':
          // Disable some animations
          g_pAccFx->bHG = false;
          g_pAccFx->bLG = false;
          g_pAccFx->bDG = false;
          g_pAccFx->bAgr = false;
          pBatCheck->active = false;

          // Set to normal
          ledNormal();
          break;

        case 'B':
          // Check battery
          pBatCheck->active = true;
          // Battery status Blink
          xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
                    xAskBat,        // Function to be called
                    "Ask bat",      // Name of task
                    1024,           // Stack size (bytes in ESP32, words in FreeRTOS)
                    NULL,           // Parameter to pass
                    3,              // Task priority
                    &xAskBat_Hdlr,  // Task handle
                    1);             // Run on core one
          break;

        case 'b':
          // Ask battery
          askBat();
          break;

        case 'i':
          // Impact Animation
          startSwordImpact();
          break;

        case 'w':
          // Wave Animation
          ledPulse();
          break;

        case 'c':
          // Change color
          ledDifuseChange();
          break;

        case 'a':
          // Enables or disable accel data.
          g_bBTa = !g_bBTa;
          break;

        case 'h':
          // Switch ON/OFF Sword
          g_bSword = !g_bSword;
          if (g_bSword)  ledFadeON();
          else          ledFadeOFF();
          break;

      }
    }
  	vTaskDelay(DELAY_BT / portTICK_PERIOD_MS);
  }

  // Cannot be here. Delete task.
  vTaskDelete(NULL);
}
#endif


#ifdef MPU6050
// Accel and Gyro readings
void xReadMPU6050(void *pvParam) {
  (void) pvParam;

  while(1) {
  	mpu6050read();
  	vTaskDelay(DELAY_MPU6050 / portTICK_PERIOD_MS);
  }

  // Cannot be here. Delete task.
  vTaskDelete(NULL);
}
#endif

int askBat() {
  int batValue = analogRead(PIN_BAT);
  // Analog Read value from 0-3.300v (0-4095)
  // Due to NMOS voltage drop, each BAT value is previously measured.
  // Scale analog reading to voltage value up to two decimals.
  //
  int val = map(batValue, 0, 4095, 0, 3300);

  // Get value at voltage divider and check Vbat.
  // Calcule the corresponding voltage if Vbat is 8V
  // assuming that voltage equivalence is linear.

  int nLEDs = NUM_LEDS;
  if (val > VBAT+20) {
    nLEDs = NUM_LEDS;  // 100% | 8.4 Vbat -> 4.2v
    Serial.println("8.4Vbat, 4.2Vcell -> 100% Battery");
#ifdef BT06
    BT.println("8.4Vbat, 4.2Vcell -> 100% Battery");
#endif
  } else if (val > VBAT+10) {
    nLEDs = NUM_LEDS * 9/10;  // 95% | 8.2 Vbat -> 4.1v
    Serial.println("8.2Vbat, 4.1Vcell -> 95% Battery");
#ifdef BT06
    Serial.println("8.2Vbat, 4.1Vcell -> 95% Battery");
#endif
  } else if (val > VBAT) {
    nLEDs = NUM_LEDS * 9/10;  // 90% | 8 Vbat -> If Vmeasured = 4v
    Serial.println("8Vbat, 4Vcell -> 90% Battery");
#ifdef BT06
    BT.println("8Vbat, 4Vcell -> 90% Battery");
#endif
  } else if (val > VBAT-7) {
    nLEDs = NUM_LEDS * 8/10;  // 80% | 7.8 Vbat -> 3.93v
    Serial.println("7.8Vbat, 3.9Vcell -> 80% Battery");
#ifdef BT06
    BT.println("7.8Vbat, 3.9Vcell -> 80% Battery");
#endif
  } else if (val > VBAT-7*2) {
    nLEDs = NUM_LEDS * 7/10;  // 70% | 7.7 Vbat -> 3.86v
    Serial.println("7.7Vbat, 3.85Vcell -> 70% Battery");
#ifdef BT06
    BT.println("7.7Vbat, 3.85Vcell -> 70% Battery");
#endif
  } else if (val > VBAT-7*3) {
    nLEDs = NUM_LEDS * 6/10;  // 60% | 7.6 Vbat -> 3.79v
    Serial.println("7.6Vbat, 3.8Vcell -> 60% Battery");
#ifdef BT06
    BT.println("7.6Vbat, 3.8Vcell -> 60% Battery");
#endif
  } else if (val > VBAT-7*4) {
    nLEDs = NUM_LEDS * 5/10;  // 50% | 7.5 Vbat -> 3.72v
    Serial.println("7.5Vbat, 3.75Vcell -> 50% Battery");
#ifdef BT06
    BT.println("7.5Vbat, 3.75Vcell -> 50% Battery");
#endif
  } else if (val > VBAT-7*5) {
    nLEDs = NUM_LEDS * 4/10;  // 40% | 7.4 Vbat -> 3.65v
    Serial.println("7.4Vbat, 3.7Vcell -> 40% Battery");
#ifdef BT06
    BT.println("7.4Vbat, 3.7Vcell -> 40% Battery");
#endif
  } else if (val > VBAT-7*6) {
    nLEDs = NUM_LEDS * 3/10;  // 30% | 7.3 Vbat -> 3.58v
    Serial.println("7.3Vbat, 7.65Vcell -> 30% Battery");
#ifdef BT06
    BT.println("7.3Vbat, 7.65Vcell -> 30% Battery");
#endif
  } else if (val > VBAT-7*7) {
    nLEDs = NUM_LEDS * 2/10;  // 20% | 7.2 Vbat -> 3.51v
    Serial.println("7.2Vbat, 3.6Vcell -> 20% Battery");
#ifdef BT06
    BT.println("7.2Vbat, 3.6Vcell -> 20% Battery");
#endif
  } else {
    nLEDs = NUM_LEDS * 1/10;  // 10%
    Serial.println("7Vbat, 3.5Vcell -> 10% Battery");
#ifdef BT06
    BT.println("7Vbat, 3.5Vcell -> 10% Battery");
#endif
  }
  return nLEDs;
}


/**************************** MPU6050 functions ******************************

##############################################################################*/
#ifdef MPU6050
void mpu6050setup() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  /* Register 1C -> Bytes AFS_SEL
     2g -> 16384 LSB/g (0b00000000 -> 0x00)
     4g -> 8192 LSB/g  (0b00001000 -> 0x08)
     8g -> 4096 LSB/g (0b00010000 -> 0x10)
     16g -> 2048 LSB/g (0b00011000 -> 0x18)
  */
  g_pAcc->S = 4096;
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  // Set the register AFS_SEL
  Wire.endTransmission(true);

  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  /* Register 0x1B -> Bytes FS_SEL
     250ยบ/s -> 131 LSB/g (0b00000000 -> 0x00)
     500ยบ/s -> 65.5 LSB/g  (0b00001000 -> 0x08)
     1000ยบ/s -> 32.8 LSB/g (0b00010000 -> 0x10)
     2000ยบ/s -> 16.4 LSB/g (0b00011000 -> 0x18)
  */
  g_pGyro->S = 32.8;
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register FS_SEL
  Wire.endTransmission(true);

  g_pAcc->currentTime = millis();

  // Get the IMU error values for calibration at the start
  mpu6050errorIMU();
}

void mpu6050errorIMU() {
  g_pAcc->errorX = 0;
  g_pAcc->errorY = 0;
  g_pGyro->errorX = 0;
  g_pGyro->errorY = 0;
  g_pGyro->errorZ = 0;

  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error.
  Serial.println("Calibrating accel & gyro...");
  // Read accelerometer values 200 times
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    g_pAcc->X = (Wire.read() << 8 | Wire.read()) / g_pAcc->S ;
    g_pAcc->Y = (Wire.read() << 8 | Wire.read()) / g_pAcc->S ;
    g_pAcc->Z = (Wire.read() << 8 | Wire.read()) / g_pAcc->S ;
    // Sum all readings
    g_pAcc->errorX = g_pAcc->errorX + ((atan((g_pAcc->Y) / sqrt(pow((g_pAcc->X), 2) + pow((g_pAcc->Z), 2))) * 180 / PI));
    g_pAcc->errorY = g_pAcc->errorY + ((atan(-1 * (g_pAcc->X) / sqrt(pow((g_pAcc->Y), 2) + pow((g_pAcc->Z), 2))) * 180 / PI));
  }

  //Divide the sum by 200 to get the error value
  g_pAcc->errorX = g_pAcc->errorX / 200;
  g_pAcc->errorY = g_pAcc->errorY / 200;

  // Read gyro values 200 times
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    g_pGyro->X = Wire.read() << 8 | Wire.read();
    g_pGyro->Y = Wire.read() << 8 | Wire.read();
    g_pGyro->Z = Wire.read() << 8 | Wire.read();
    // Sum all readings
    g_pGyro->errorX = g_pGyro->errorX + (g_pGyro->X / g_pGyro->S);
    g_pGyro->errorX = g_pGyro->errorX + (g_pGyro->Y / g_pGyro->S);
    g_pGyro->errorX = g_pGyro->errorX + (g_pGyro->Z / g_pGyro->S);
  }

  //Divide the sum by 200 to get the error value
  g_pGyro->errorX = g_pGyro->errorX / 200;
  g_pGyro->errorX = g_pGyro->errorX / 200;
  g_pGyro->errorX = g_pGyro->errorX / 200;

  // Print the error values on the Serial Monitor
  Serial.print("g_pAcc->errorX: ");
  Serial.println(g_pAcc->errorX);
  Serial.print("g_pAcc->errorY: ");
  Serial.println(g_pAcc->errorY);
  Serial.print("g_pGyro->errorX: ");
  Serial.println(g_pGyro->errorX);
  Serial.print("g_pGyro->errorX: ");
  Serial.println(g_pGyro->errorX);
  Serial.print("g_pGyro->errorX: ");
  Serial.println(g_pGyro->errorX);
}

void mpu6050read() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  // Read 6 registers total, each axis value is stored in 2 registers
  g_pAcc->X = (Wire.read() << 8 | Wire.read()) / g_pAcc->S;
  g_pAcc->Y = (Wire.read() << 8 | Wire.read()) / g_pAcc->S;
  g_pAcc->Z = (Wire.read() << 8 | Wire.read()) / g_pAcc->S;

  // Compute g_pGyro->Roll and g_pGyro->Pitch from the accelerometer data
  g_pAcc->angleX = (atan(g_pAcc->Y / sqrt(pow(g_pAcc->X, 2) + pow(g_pAcc->Z, 2))) * 180 / PI) - g_pAcc->errorX;
  g_pAcc->angleY = (atan(-1 * g_pAcc->X / sqrt(pow(g_pAcc->Y, 2) + pow(g_pAcc->Z, 2))) * 180 / PI) - g_pAcc->errorY;

  // === Read gyroscope data === //
  g_pAcc->PrevTime = g_pAcc->currentTime;        // Previous time is stored before the actual time read
  g_pAcc->currentTime = millis();            // Current time actual time read
  g_pAcc->elapsedTime = (g_pAcc->currentTime - g_pAcc->PrevTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  // Read 6 registers total, each axis value is stored in 2 registers
  g_pGyro->X = (Wire.read() << 8 | Wire.read()) / g_pGyro->S;
  g_pGyro->Y = (Wire.read() << 8 | Wire.read()) / g_pGyro->S;
  g_pGyro->Z = (Wire.read() << 8 | Wire.read()) / g_pGyro->S;

  // Correct the outputs with the calculated error values
  g_pGyro->X = g_pGyro->X - g_pGyro->errorX;
  g_pGyro->Y = g_pGyro->Y - g_pGyro->errorX;
  g_pGyro->Z = g_pGyro->Z - g_pGyro->errorX;

  // Currently the raw values are in deg/s,
  // so we need to multiply by sendonds (s) to get the angle in degrees
  g_pGyro->angleX = g_pGyro->angleX + g_pGyro->X * g_pAcc->elapsedTime;
  g_pGyro->angleY = g_pGyro->angleY + g_pGyro->Y * g_pAcc->elapsedTime;
  g_pGyro->Yaw =  g_pGyro->Yaw + g_pGyro->Z * g_pAcc->elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  g_pGyro->Roll = 0.96 * g_pGyro->angleX + 0.04 * g_pAcc->angleX;
  g_pGyro->Pitch = 0.96 * g_pGyro->angleY + 0.04 * g_pAcc->angleY;

/*
  // Print the values on the serial monitor
  Serial.print("Gyro:");
  Serial.print(g_pGyro->Roll);
  Serial.print("/");
  Serial.print(g_pGyro->Pitch);
  Serial.print("/");
  Serial.println(g_pGyro->Yaw);
  Serial.print("Acc:");
  Serial.print(g_pAcc->X);
  Serial.print("/");
  Serial.print(g_pAcc->Y);
  Serial.print("/");
  Serial.println(g_pAcc->Z);
  */

  String sGyro = "/G:";
  String sAcc = "/A:";
  String sSep = ";";
  String s = sGyro+g_pGyro->Roll+sSep+g_pGyro->Pitch+sSep+g_pGyro->Yaw + sAcc+g_pAcc->X+sSep+g_pAcc->Y+sSep+g_pAcc->Z;
  Serial.println(s);

#ifdef BT06
  if (g_bBTa) {
    BT.println(s);
  }
#endif

  // Data analysis to determine if an agresive cut or impact was performed
  // TODO

  g_pAccFx->impDetect = false;
  g_pAccFx->agrDetect = false;
  g_pAccFx->bAgr = false;

  if (g_pAccFx->agrDetect) {
    // Trigger only one time
    if (!g_pAccFx->bAgr) ledAgressive();
    g_pAccFx->bAgr = true;

  } else if (g_pAccFx->impDetect) {
    // Impact in process...
    if (g_pAccFx->nImact == 0) g_pAccFx->nImact = NUM_IMPACT;

  } else {
    // High Guard
    if ( g_pAcc->X > 0.9 && 0.1 > g_pAcc->Y > -0.1 && 0.1 > g_pAcc->Z > -0.1) {
      if (!g_pAccFx->bHG) startLedHighGuard();
    } else {
      if (g_pAccFx->bHG) {
        g_pAccFx->bHG = false;
        ledNormal();
      }
    }

    // Low Guard
    if (g_pAcc->X < -0.7 && 0.1 > g_pAcc->Y > -0.1 && g_pAcc->Z < 0.8) {
      if (!g_pAccFx->bLG) {
        g_pAccFx->bLG = true;
        ledLowGuard();
      }
    } else {
      if (g_pAccFx->bLG) {
        g_pAccFx->bLG = false;
        ledNormal();
      }
    }

    // Defense Guard
    if (0.1 > g_pAcc->X > -0.1 && 0.1 > g_pAcc->Y > -0.1 && g_pAcc->Z < -0.8) {
      if (!g_pAccFx->bDG) {
        g_pAccFx->bDG = true;
        ledDefenseGuard();
      }
    } else {
      if (g_pAccFx->bDG) {
        g_pAccFx->bDG = false;
        ledNormal();
      }
    }
  }


  /* Comment
    A perfect cut with the sword is when there is a cut with acceleration
    and also a rotation velocity. If there is no rotation, the Sword hits
    the opponent without a slashing.

            ||
         |     |
       |        |
     |           |
    |       X     |
    |       ^     |
    |       '     |
    |  Y <- +     |
    |       Z     |
    |             |
    |             |
       Sword Axis

    - g_pGyro->Roll at X Axis -> Sword Rotation (Defense guard)
    - g_pGyro->Pitch at Y Axis -> Vertical Cut -> Accel Z & X
    - g_pGyro->Yaw at Z Axis -> Horizontal Cut -> Accel Y & X

  */

}
#endif

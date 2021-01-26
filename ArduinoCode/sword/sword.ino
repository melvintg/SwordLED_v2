// PINS DEFINITION
#define PIN_TS 2          // Touch Sensor
#define PIN_LED_W 8       // LED White for Touch Sensor
#define PIN_LED_G 4       // LED Green for ATmega328P
#define PIN_BZ 5          // Buzzer
#define PIN_LEDS 6        // LEDs WS2812B Data pin
#define PIN_LED_R 7       // LED Red for Battery check
#define PIN_TXB 10        // Tx Bluetooth
#define PIN_RXB 11        // Rx Bluetooth
#define PIN_BAT_CH A0     // Charge pin LOW/HIGH
#define PIN_BAT A1        // Battery level

#include <Wire.h>
#include <FastLED.h>

//#define MPU6050
#define BT06

#ifdef BT06
  #include <SoftwareSerial.h>
  SoftwareSerial BT(10,11);
#endif

const int VBAT = 421;

// Delay constant variables
#ifdef MPU6050
#define DELAY_LED_R 150
#define DELAY_LED_G 100
#define DELAY_IMPACT 8
#define DELAY_BAT_CHECK 100
#define DELAY_BZ_ON 1
#define DELAY_BZ_OFF 3
#define DELAY_LED_FADE_ON 7
#define DELAY_LED_FADE_OFF 8
#define DELAY_LED_WAVE 0
#define DELAY_HG_WAVE 6
#define DELAY_HG_BLINK 40

#else
#define DELAY_LED_R 250
#define DELAY_LED_G 200
#define DELAY_IMPACT 15
#define DELAY_BAT_CHECK 200
#define DELAY_BZ_ON 4
#define DELAY_BZ_OFF 4
#define DELAY_LED_FADE_ON 10
#define DELAY_LED_FADE_OFF 10
#define DELAY_LED_WAVE 4
#define DELAY_HG_WAVE 6
#define DELAY_HG_BLINK 40
#endif

// Bluetooth global variables
boolean g_bBTa = false;

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
Acc_t gAcc;

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
Gyro_t gGyro;
#endif

// LED animations using Accel.
typedef struct AccelAnimations {
  byte nImact;    // Impact detected
  boolean bAgr;   // Agressive Action
  boolean bLG;    // Low guard
  boolean bDG;    // Defensive guard
  boolean bHG;    // High guard
  byte tHG;
  byte delayTime;
  int nHG;
} AccFx_t;
AccFx_t gAccFx;

// LED fade Animation variables
typedef struct LedFade {
  boolean active;
  byte progress;
  byte waveCount;
  boolean isWaveInv;
  byte delayTime;
  byte fadeValue;
} LedFade_t;
LedFade_t gLedFade;

// LED wave Animation variables
typedef struct LedWave {
  boolean active = true;
  byte progress = 0;
  byte delayTime = 0;
  byte fadeValue = 0;
} LedWave_t;
LedWave_t gLedWave;

// Battery Check variables
typedef struct BatCheck {
  boolean active;
  boolean mantained;
  boolean led;
  byte nLed;
} BatCheck_t;
BatCheck_t gBatCheck;

// Buzzer Fade variables
typedef struct Buzzer {
  boolean active;
  boolean fadeON;
  boolean fadeOFF;
  boolean wave;
  int n;
  byte delayTime;
}Bz_t;
Bz_t gBz;

// WS2812B Constants and variables
#define NUM_LEDS 44
#define NUM_COLOR 5
CRGB Leds[NUM_LEDS];
CRGB TargetLeds[NUM_LEDS];
const CRGB COLOR[NUM_COLOR] = {CRGB::White, CRGB::Orange, CRGB::Blue, CRGB::HotPink,  CRGB::Green};
byte ColorPos;

// Constant definitions
const int MPU = 0x68;
const byte NUM_IMPACT = 14;

// Global flags variables
boolean g_bTs;
boolean g_bLEDs;
boolean g_bLEDg;
boolean g_bSword;

// Global variables
unsigned long g_time;
unsigned long g_touchTime;
byte g_nTouch;
int g_nLoopCount;

// LED fade function prototypes
void startLedFade(CRGB color, byte fadeVal, byte delayTime, int wave);
void xLedFade(void);
CRGB fadeTowardColor( CRGB& cur, const CRGB& target, uint8_t amount);
void nblendU8TowardU8( uint8_t& cur, const uint8_t target, uint8_t amount);

// LED wave function prototypes
void startLedWave(byte fadeVal, byte delayTime);
void xLedWave(void);
void ledFadeAll(int fadeVal);

// LED impact function prototypes
void startSwordImpact(void);
void xSwordImpact(void);
void ledImpact(void);
void bzImpact(void);

// LED on/off function prototypes
void ledON(CRGB color, int nLeds);
void ledOFF(void);

// Battery Check function prototypes
void startBatCheck(void);
void xBatCheck(void);
int askBat(void);
void checkBattery(void);

// Buzzer function prototypes
void startBzFade(byte delayTime, boolean isFadeON, boolean isWave);
void xBzFadeON(void);
void xBzFadeOFF(void);
void xBzWave(void);

// Touch sensor function prototypes
void touchSensor(void);
void checkTouchTS(void);

// LEDs functions
void ledPulse(void);
void ledDifuseChange(void);
void ledFadeON(void);
void ledFadeOFF(void);
void ledNormal(void);
void ledAgressive(void);

// LED animations using Accel. function prototypes
void startLedHighGuard(void);
void xLedHighGuard(void);
void ledLowGuard(void);
void ledDefenseGuard(void);

// Accel. function prototypes
#ifdef MPU6050
void mpu6050setup(void);
void mpu6050errorIMU(void);
void mpu6050read(void);
#endif


void setup() {

  // LED animations using Accel.
  gAccFx.nImact = 0;
  gAccFx.nHG = 0;
  gAccFx.tHG = DELAY_HG_WAVE;
  gAccFx.delayTime = DELAY_HG_BLINK;
  gAccFx.bHG = false;
  gAccFx.bLG = false;
  gAccFx.bDG = false;
  gAccFx.bAgr = false;

  // LED fade Animation variables
  gLedFade.active = false;
  gLedFade.progress = 0;
  gLedFade.waveCount = 0;
  gLedFade.isWaveInv = false;
  gLedFade.delayTime = 0;
  gLedFade.fadeValue = 0;

  // LED wave Animation variables
  gLedWave.active = false;
  gLedWave.delayTime = 0;
  gLedWave.fadeValue = 0;
  gLedWave.progress = 0;

  // Battery Check variables
  gBatCheck.active = false;
  gBatCheck.mantained = false;
  gBatCheck.led = false;
  gBatCheck.nLed = 0;

  // Buzzer Fade variables
  gBz.fadeON = false;
  gBz.fadeOFF = false;
  gBz.wave = false;

  pinMode(PIN_TS, INPUT);
  pinMode(PIN_LED_W, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_BZ,OUTPUT);
  pinMode(PIN_LEDS, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BAT_CH, INPUT);
  pinMode(PIN_BAT, INPUT);

  digitalWrite(PIN_LED_W, LOW);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_R, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_TS), touchSensor, CHANGE);

  FastLED.addLeds<WS2812, PIN_LEDS, GRB>(Leds, NUM_LEDS);
  LEDS.setBrightness(180);
  ColorPos = 0;
  ledOFF();

#ifdef MPU6050
  mpu6050setup();
#endif

#ifdef BT06
  BT.begin(9600);
#endif

  g_bLEDs = false;
  g_bLEDg = true;
  g_bSword = false;
  g_nLoopCount = 0;

  g_bTs = false;
  g_nTouch = 0;
  g_time =  0;
  g_touchTime = 0;

  Serial.begin(115200);
  Serial.println("Starting Sword");

}

void loop() {

  checkTouchTS();

#ifdef BT06
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

      case 'n':
        // Disable some animations
        gAccFx.bHG = false;
        gAccFx.bLG = false;
        gAccFx.bDG = false;
        gAccFx.bAgr = false;
        gBatCheck.active = false;

        // Set to normal
        ledNormal();
        break;

      case 'B':
        // Check battery
        startBatCheck();
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
#endif

#ifdef MPU6050
  if (g_bSword) {

    // Get Accel & Gyro data
    if (g_nLoopCount % 10 == 0) {
      mpu6050read();
    }
  }
#endif

/******************************* Sword animation *****************************/
  if (gAccFx.nImact > 0) {
    if (g_nLoopCount % DELAY_IMPACT == 0) {
      xSwordImpact();
    }
  }

  if (gBatCheck.active) {
    if (g_nLoopCount % DELAY_BAT_CHECK == 0) {
      xBatCheck();
    }
  }

  if (gLedFade.active) {
    if (g_nLoopCount % gLedFade.delayTime == 0) {
      xLedFade();
    }
  }

  if (gLedWave.active) {
    if (g_nLoopCount % gLedWave.delayTime == 0) {
      xLedWave();
    }
  }

  if (gBz.fadeON) {
    if (g_nLoopCount % gBz.delayTime == 0) {
      xBzFadeON();
    }
  }

  if (gBz.fadeOFF) {
    if (g_nLoopCount % gBz.delayTime == 0) {
      xBzFadeOFF();
    }
  }

  if (gBz.wave) {
    if (g_nLoopCount % gBz.delayTime == 0) {
      xBzWave();
    }
  }

  // Check if it is high guard animation
  if (gAccFx.bHG) {
    if (g_nLoopCount % gAccFx.delayTime == 0) {
      xLedHighGuard();
    }
  }

/*****************************************************************************/

  // Check battery and upload Red LED status
  if (g_nLoopCount % DELAY_LED_R == 0) {
    checkBattery();
  }

  // Blink LED_G each 200 ms delay
  if (g_nLoopCount % DELAY_LED_G == 0) {
    // Blink Green LED while normal operation (excludes battery charging)
    if (digitalRead(PIN_BAT_CH)) {
      digitalWrite(PIN_LED_G, LOW);
    } else {
      g_bLEDg = !g_bLEDg;
      digitalWrite(PIN_LED_G, g_bLEDg);
    }
  }

  // Use to divide code into fast or slow operation.
  g_nLoopCount++;
  if (g_nLoopCount == 30000) g_nLoopCount = 0;
  delay(1);

  if (g_nLoopCount % 1000 == 0) Serial.print(".");
}

/* Interrupt function for touch sensor */
void touchSensor() {
  g_bTs = digitalRead(PIN_TS);

  // Start Timer
  g_touchTime = millis();
  g_time = 0;

  // Check if it's a falling edge
  if (g_bTs == 0) {
    // Check if it is a pulse or a maintained touch
    if (!gBatCheck.mantained)  g_nTouch++;
    else                  g_touchTime = 0;
    gBatCheck.mantained = false;

    // Battery check mantained press is released. End Battery check.
    if (gBatCheck.active) {
      gBatCheck.active = false;
      g_nTouch = 0;
      ledNormal();
    }
  }

  // Update state
  digitalWrite(PIN_LED_W, g_bTs);
  Serial.println(g_bTs);
}

/* Check Touch Sensor after Touch Sensor interrupt */
void checkTouchTS() {
  // If timer is ON, update timer.
  if (g_touchTime != 0) g_time = millis() - g_touchTime;

  // Touch Sensor High. Check for Maintained touch
  if (g_bTs) {
    // Check if Touch Sensor is mantained for 1000 ms
    if (g_time > 1000 && !gBatCheck.mantained) {
      // Button maintained
      gBatCheck.mantained = true;
      // Stop Timer
      g_touchTime = 0;
      // If only was one touch previously
      if (g_nTouch > 0) {
        Serial.println("1 press & 1 mantained Touch Sensor");
        // Battery status Blink
        startBatCheck();
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
        } else if (g_nTouch > 1) {
          Serial.println("2 press Touch Sensor");
          ledDifuseChange();
        }
        g_nTouch = 0;
      }
    }
  }
}

/* Indicates the battery state using a red LED blink animation */
void checkBattery() {
  gBatCheck.nLed++;
  if (gBatCheck.nLed > 100) {
    gBatCheck.nLed = 1;
  }
  int batValue = analogRead(PIN_BAT);
  // Due to NMOS voltage drop, each BAT value is previously measured.
  // Scale analog reading to voltage value up to two decimals.
  int val = map(batValue, 0, 1024, 0, 500);
  // Serial.println33(val);

  // 100% -> 4.2Vbat | 80% -> 3.9Vbat | 60% -> 3.8Vbat | 40% -> 3.7Vbat | 20% -> 3.6Vbat | 0% -> 3Vbat
  if (digitalRead(PIN_BAT_CH)) {
    // Charge mode
    if (gBatCheck.nLed % 10 == 0) {
      askBat();
    }
    if (val < VBAT-7*4) { // 7.5 Vbat -> 3.72v
      // LED_R blink slow for battery 0%-50%
      if (gBatCheck.nLed % 4 == 0) {
        gBatCheck.led = !gBatCheck.led;
        digitalWrite(PIN_LED_R, gBatCheck.led);
      }
    } else if (val < VBAT+20) { // 7.8 Vbat -> 3.89v
      // LED_R blink fast for battery 50%-90%
      gBatCheck.led = !gBatCheck.led;
      digitalWrite(PIN_LED_R, gBatCheck.led);
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
      if (gBatCheck.nLed % 4 == 0) {
        gBatCheck.led = !gBatCheck.led;
        digitalWrite(PIN_LED_R, gBatCheck.led);
      }
    } else {
      // LED_R blink fast for battery 20%-0%
      gBatCheck.led = !gBatCheck.led;
      digitalWrite(PIN_LED_R, gBatCheck.led);
    }
  }
}

/****************************** LEDs functions ********************************

##############################################################################*/
/* Perfomr a pulse led wave animation */
void ledPulse() {
  if (g_bSword) {
    Serial.println("Pulse led wave animation...");
    startLedWave(235, DELAY_LED_WAVE);
    startBzFade(DELAY_BZ_OFF, false, true);
  }
}

void ledDifuseChange() {
  Serial.println("Changing base color...");
  ColorPos++;
  if (ColorPos == NUM_COLOR) ColorPos = 0;

  if (g_bSword)
    startLedFade(COLOR[ColorPos], 8, 0, 0);
}

void ledFadeON() {
  Serial.println("Switching ON Sword..");
  g_bLEDs = true;
  g_bSword = true;
  ColorPos = 0;
  startLedFade(COLOR[ColorPos], 2, DELAY_LED_FADE_ON, 1);
  startBzFade(DELAY_BZ_ON, true, false);
}

void ledFadeOFF() {
  Serial.println("Switching OFF Sword..");
  g_bLEDs = false;
  g_bSword = false;
  startLedFade(CRGB::Black, 12, DELAY_LED_FADE_OFF, -1);
  startBzFade(DELAY_BZ_OFF, false, false);
}

void ledNormal() {
  if (g_bSword) {
    Serial.println("Sword back to normal");
    g_bLEDs = true;
    startLedFade(COLOR[ColorPos], 8, 0, 0);
  }
}

void ledAgressive() {
  Serial.println("Sword movement detected...");
  startLedFade(CRGB::Red, 8, 0, 1);
}


void startLedHighGuard() {
  Serial.println("High guard...");
  gAccFx.bHG = true;
  gAccFx.nHG = 0;
  gAccFx.tHG = DELAY_HG_WAVE;
  gAccFx.delayTime = DELAY_HG_BLINK;
}

void xLedHighGuard() {
  // Animation in process
  if (gLedFade.active || gLedWave.active) return;

  // To active, use gAccFx.bHG = true
  if (gAccFx.nHG == 0) {
      startLedFade(CRGB::Blue,50,0,0);
  } else if (gAccFx.nHG < 30) {
      startLedWave(235, gAccFx.tHG);
      // Speed the animation
      if (gAccFx.nHG % 2 == 0 && gAccFx.tHG > 0) {
        gAccFx.tHG--;
      }
  } else {
    if (g_bLEDs) {
      ledON(CRGB::Blue, NUM_LEDS);
    } else {
      ledOFF();
    }
    g_bLEDs = !g_bLEDs;
    if (gAccFx.nHG % 5 == 0 && gAccFx.delayTime > 10) {
        gAccFx.delayTime--;
    }
  }
  gAccFx.nHG++;
  if (gAccFx.nHG > 30000) {
    gAccFx.nHG = 50;
  }
}

void ledLowGuard() {
  // Difuse to Green
  startLedFade(CRGB::Green, 8, 0, 1);

}

void ledDefenseGuard() {
  // Difuse to Blue
  startLedFade(CRGB::Blue, 8, 0, 1);
}



/******************************* Sword animation *****************************
##############################################################################
- Battery Check -> Highest priority
- ON / OFF Sword -> Highest priority
- Impact Animation -> High priority
- Agressive Animation -> Medium priority
- Wave & Fade -> Low priority
##############################################################################*/

/**************************** ON/OFF LEDs function ***************************/
void ledON(CRGB color, int nLeds) {
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

/************************** Battery Check animation **************************/
/* Perform a battery voltage check and display value to LEDs */
void startBatCheck() {
  Serial.println("Battery Check");
  digitalWrite(PIN_LED_R, HIGH);
  gBatCheck.active = true;
}

/* Battery check animation task executed in main loop every DELAY_BAT_CHECK*/
void xBatCheck() {
    // Battery check with Touch Sensor
    int nLEDs = askBat();

    if (g_bLEDs) {
      ledON(CRGB::Red, nLEDs);
    } else {
      ledOFF();
    }
    g_bLEDs = !g_bLEDs;
}

int askBat() {
  int batValue = analogRead(PIN_BAT);
  // Analog Read value from 0-5v (0-1023).
  // Due to NMOS voltage drop, each BAT value is previously measured.
  // Scale analog reading to voltage value up to two decimals.
  int val = map(batValue, 0, 1024, 0, 500);

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

/*************************** Impact Sword animation **************************/
/* Impact detected. Start led and buzzer blink animation. */
void startSwordImpact() {
  if (gBatCheck.active) return;

  gAccFx.nImact = NUM_IMPACT;
}

/* Impact animation task executed in main loop every DELAY_IMPACT*/
void xSwordImpact() {
  if (gAccFx.nImact == 1) {
    // Last impact. Get to Normal
    ledNormal();
    noTone(PIN_BZ);
    gAccFx.nImact = 0;
  } else {
    ledImpact();
    bzImpact();
    gAccFx.nImact--;
  }
}

// Yellow Fast blink motion
void ledImpact() {
  g_bLEDs = !g_bLEDs;
  if (g_bLEDs) {
    ledON(CRGB::Yellow, NUM_LEDS);
  } else {
    ledOFF();
  }
}

// Beeping Buzzer motion
void bzImpact() {
  if (g_bLEDs) {
    tone(PIN_BZ, 2000);
  } else {
    noTone(PIN_BZ);
  }
}


/***************************** Wave LED animation ****************************/
/* Start LED fade animation by setting global variables and setting a mutex */
void startLedWave(byte fadeVal, byte delayTime) {
  if (gBatCheck.active) return;

  gLedWave.active = true;
  gLedWave.delayTime = delayTime;
  if (delayTime == 0) gLedWave.delayTime = 1;   // Some delay for stability
  gLedWave.fadeValue = fadeVal;
  gLedWave.progress = 0;
}

/* LED wave animation task executed in main loop every gLedWave.delayTime */
void xLedWave() {
  if (gLedWave.progress < NUM_LEDS) {
    Leds[gLedWave.progress].maximizeBrightness();
    ledFadeAll(gLedWave.fadeValue);
    FastLED.show();
    gLedWave.progress++;
  } else {
    gLedWave.active = false;
    ledNormal();
  }
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
  if (gBatCheck.active) return;
  gLedFade.active = true;
  gLedFade.progress = 0;
  gLedFade.delayTime = delayTime;
  if (delayTime == 0) gLedFade.delayTime = 1;   // Some delay for stability
  gLedFade.fadeValue = fadeVal;

  /* wave = 0 -> No wave
   * wave > 0 -> Bottom to Top wave
   * wave < 0 -> Top to bottom wave
   */
  gLedFade.waveCount = NUM_LEDS;
  if (wave > 0) {
    gLedFade.waveCount = 1;
  }

  if (wave < 0) {
    gLedFade.isWaveInv = true;
  } else {
    gLedFade.isWaveInv = false;
  }

  for(int i = 0; i < NUM_LEDS; i++) {
      TargetLeds[i] = color;
  }
}

/* LED fade animation task executed in main loop every gLedFade.delayTime*/
void xLedFade() {
  if (gLedFade.progress != NUM_LEDS) {
    gLedFade.progress = 0;
    if (gLedFade.isWaveInv) {
      for(int i = NUM_LEDS-1; i >= gLedFade.waveCount; i--){
        fadeTowardColor(Leds[i], TargetLeds[i], gLedFade.fadeValue);
        if (Leds[i].red == TargetLeds[i].red
            && Leds[i].green == TargetLeds[i].green
            && Leds[i].blue == TargetLeds[i].blue){
          gLedFade.progress++;
        }
      }
      if (gLedFade.waveCount > 0) gLedFade.waveCount--;

    } else {
      for(int i = 0; i < gLedFade.waveCount; i++){
        fadeTowardColor(Leds[i], TargetLeds[i], gLedFade.fadeValue);
        if (Leds[i].red == TargetLeds[i].red
            && Leds[i].green == TargetLeds[i].green
            && Leds[i].blue == TargetLeds[i].blue){
          gLedFade.progress++;
        }
      }
      if (gLedFade.waveCount < NUM_LEDS) gLedFade.waveCount++;
    }

    FastLED.show();
  } else {
    gLedFade.active = false;
  }
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
/* Start fade Buzzer. Used for ON/OFF Sword and pulse wave animations. */
void startBzFade(byte delayTime, boolean isFadeON, boolean isWave) {
  if (gBz.fadeON || gBz.fadeOFF || gBz.wave) return;

  if (isWave) {
    gBz.wave = true;
    gBz.n = 5000;
    gBz.delayTime = delayTime;
  } else {
    if (isFadeON) {
      gBz.fadeON = true;
      gBz.n = 0;
      gBz.delayTime = delayTime;
    } else {
      gBz.fadeOFF = true;
      gBz.n = 5000;
      gBz.delayTime = delayTime;
    }
  }
  gBz.active = true;
}

/* Buzzer fade ON task executed in main loop every gBz.delayTime */
void xBzFadeON() {
  if (gBz.n < 5000) {
      if (gBz.n < 2000) {
        gBz.n += 5;
      } else {
        gBz.n += gBz.n/200;
      }
      if (gBz.active) {
        tone(PIN_BZ, gBz.n);
      } else {
        noTone(PIN_BZ);
      }
    } else {
      // Reset variables
      gBz.fadeON = false;
      gBz.active = false;
      noTone(PIN_BZ);
    }
}

/* Buzzer fade OFF task executed in main loop every gBz.delayTime */
void xBzFadeOFF() {
  if(gBz.n > 100) {
    if (gBz.n < 1000) {
      gBz.n -= 10;
    } else {
      gBz.n -= gBz.n/50;
    }
    if (gBz.active) {
      tone(PIN_BZ, gBz.n);
    } else {
      noTone(PIN_BZ);
    }
  } else {
    // Reset variables
    gBz.fadeOFF = false;
    gBz.active = false;
    noTone(PIN_BZ);
  }
}

/* Buzzer fade pulse wave task executed in main loop every gBz.delayTime */
void xBzWave() {
  if(gBz.n > 100) {
    if (gBz.n < 1000) {
      gBz.n -= 50;
    } else {
      gBz.n -= gBz.n/10;
    }
    if (gBz.active) {
      tone(PIN_BZ, gBz.n);
    } else {
      noTone(PIN_BZ);
    }
  } else {
    // Reset variables
    gBz.wave = false;
    gBz.active = false;
    noTone(PIN_BZ);
  }
}

void bzAgressive() {
  //TODO
}



void bzHitPoint() {
  //TODO
}

void bzChargePosutre() {
  //TODO
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
  gAcc.S = 4096;
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  // Set the register AFS_SEL
  Wire.endTransmission(true);

  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  /* Register 0x1B -> Bytes FS_SEL
     250º/s -> 131 LSB/g (0b00000000 -> 0x00)
     500º/s -> 65.5 LSB/g  (0b00001000 -> 0x08)
     1000º/s -> 32.8 LSB/g (0b00010000 -> 0x10)
     2000º/s -> 16.4 LSB/g (0b00011000 -> 0x18)
  */
  gGyro.S = 32.8;
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register FS_SEL
  Wire.endTransmission(true);

  // Get the IMU error values for calibration at the start
  mpu6050errorIMU();
}

void mpu6050errorIMU() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error.
  Serial.println("Calibrating accel & gyro...");
  // Read accelerometer values 200 times
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gAcc.X = (Wire.read() << 8 | Wire.read()) / gAcc.S ;
    gAcc.Y = (Wire.read() << 8 | Wire.read()) / gAcc.S ;
    gAcc.Z = (Wire.read() << 8 | Wire.read()) / gAcc.S ;
    // Sum all readings
    gAcc.errorX = gAcc.errorX + ((atan((gAcc.Y) / sqrt(pow((gAcc.X), 2) + pow((gAcc.Z), 2))) * 180 / PI));
    gAcc.errorY = gAcc.errorY + ((atan(-1 * (gAcc.X) / sqrt(pow((gAcc.Y), 2) + pow((gAcc.Z), 2))) * 180 / PI));
  }

  //Divide the sum by 200 to get the error value
  gAcc.errorX = gAcc.errorX / 200;
  gAcc.errorY = gAcc.errorY / 200;

  // Read gyro values 200 times
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gGyro.X = Wire.read() << 8 | Wire.read();
    gGyro.Y = Wire.read() << 8 | Wire.read();
    gGyro.Z = Wire.read() << 8 | Wire.read();
    // Sum all readings
    gGyro.errorX = gGyro.errorX + (gGyro.X / gGyro.S);
    gGyro.errorX = gGyro.errorX + (gGyro.Y / gGyro.S);
    gGyro.errorX = gGyro.errorX + (gGyro.Z / gGyro.S);
  }

  //Divide the sum by 200 to get the error value
  gGyro.errorX = gGyro.errorX / 200;
  gGyro.errorX = gGyro.errorX / 200;
  gGyro.errorX = gGyro.errorX / 200;

  // Print the error values on the Serial Monitor
  Serial.print("gAcc.errorX: ");
  Serial.println(gAcc.errorX);
  Serial.print("gAcc.errorY: ");
  Serial.println(gAcc.errorY);
  Serial.print("gGyro.errorX: ");
  Serial.println(gGyro.errorX);
  Serial.print("gGyro.errorX: ");
  Serial.println(gGyro.errorX);
  Serial.print("gGyro.errorX: ");
  Serial.println(gGyro.errorX);
}

void mpu6050read() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  // Read 6 registers total, each axis value is stored in 2 registers
  gAcc.X = (Wire.read() << 8 | Wire.read()) / gAcc.S;
  gAcc.Y = (Wire.read() << 8 | Wire.read()) / gAcc.S;
  gAcc.Z = (Wire.read() << 8 | Wire.read()) / gAcc.S;

  // Compute gGyro.Roll and gGyro.Pitch from the accelerometer data
  gAcc.angleX = (atan(gAcc.Y / sqrt(pow(gAcc.X, 2) + pow(gAcc.Z, 2))) * 180 / PI) - gAcc.errorX;
  gAcc.angleY = (atan(-1 * gAcc.X / sqrt(pow(gAcc.Y, 2) + pow(gAcc.Z, 2))) * 180 / PI) - gAcc.errorY;

  // === Read gyroscope data === //
  gAcc.PrevTime = gAcc.currentTime;        // Previous time is stored before the actual time read
  gAcc.currentTime = millis();            // Current time actual time read
  gAcc.elapsedTime = (gAcc.currentTime - gAcc.PrevTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  // Read 6 registers total, each axis value is stored in 2 registers
  gGyro.X = (Wire.read() << 8 | Wire.read()) / gGyro.S;
  gGyro.Y = (Wire.read() << 8 | Wire.read()) / gGyro.S;
  gGyro.Z = (Wire.read() << 8 | Wire.read()) / gGyro.S;

  // Correct the outputs with the calculated error values
  gGyro.X = gGyro.X - gGyro.errorX;
  gGyro.Y = gGyro.Y - gGyro.errorX;
  gGyro.Z = gGyro.Z - gGyro.errorX;

  // Currently the raw values are in deg/s,
  // so we need to multiply by sendonds (s) to get the angle in degrees
  gGyro.angleX = gGyro.angleX + gGyro.X * gAcc.elapsedTime;
  gGyro.angleY = gGyro.angleY + gGyro.Y * gAcc.elapsedTime;
  gGyro.Yaw =  gGyro.Yaw + gGyro.Z * gAcc.elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  gGyro.Roll = 0.96 * gGyro.angleX + 0.04 * gAcc.angleX;
  gGyro.Pitch = 0.96 * gGyro.angleY + 0.04 * gAcc.angleY;

/*
  // Print the values on the serial monitor
  Serial.print("Gyro:");
  Serial.print(gGyro.Roll);
  Serial.print("/");
  Serial.print(gGyro.Pitch);
  Serial.print("/");
  Serial.println(gGyro.Yaw);
  Serial.print("Acc:");
  Serial.print(gAcc.X);
  Serial.print("/");
  Serial.print(gAcc.Y);
  Serial.print("/");
  Serial.println(gAcc.Z);
  */

  String sGyro = "/G:";
  String sAcc = "/A:";
  String sSep = ";";
  String s = sGyro+gGyro.Roll+sSep+gGyro.Pitch+sSep+gGyro.Yaw + sAcc+gAcc.X+sSep+gAcc.Y+sSep+gAcc.Z;
  Serial.println(s);

#ifdef BT06
  if (g_bBTa) {
    BT.println(s);
  }
#endif

  if (gAccFx.bAgr) {
    // Cut or impact
    // ledAgressive();

    //TODO


  } else {
    // High Guard
    if ( gAcc.X > 0.9 && 0.1 > gAcc.Y > -0.1 && 0.1 > gAcc.Z > -0.1) {
      startLedHighGuard();
    } else {
      if (gAccFx.bHG) {
        gAccFx.bHG = false;
        ledNormal();
      }
    }

    // Low Guard
    if (gAcc.X < -0.7 && 0.1 > gAcc.Y > -0.1 && gAcc.Z < 0.8) {
      if (!gAccFx.bLG) {
        gAccFx.bLG = true;
        ledLowGuard();
      }
    } else {
      if (gAccFx.bLG) {
        gAccFx.bLG = false;
        ledNormal();
      }
    }

    // Defense Guard
    if (0.1 > gAcc.X > -0.1 && 0.1 > gAcc.Y > -0.1 && gAcc.Z < -0.8) {
      if (!gAccFx.bDG) {
        gAccFx.bDG = true;
        ledDefenseGuard();
      }
    } else {
      if (gAccFx.bDG) {
        gAccFx.bDG = false;
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

    - gGyro.Roll at X Axis -> Sword Rotation (Defense guard)
    - gGyro.Pitch at Y Axis -> Vertical Cut -> Accel Z & X
    - gGyro.Yaw at Z Axis -> Horizontal Cut -> Accel Y & X

  */

}
#endif

// PINS DEFINITION
#define PIN_TS 2          // Touch Sensor
#define PIN_LED_W 3       // LED White for Touch Sensor
#define PIN_LED_G 4       // LED Green for ATmega328P
#define PIN_BZ 5          // Buzzer
#define PIN_LEDS 6        // LEDs WS2812B Data pin
#define PIN_LED_R 13       // LED Red for Battery check 7
#define PIN_TXB 10        // Tx Bluetooth
#define PIN_RXB 11        // Rx Bluetooth
#define PIN_BAT_CH A0     // Charge pin LOW/HIGH
#define PIN_BAT A1        // Battery level

#include <Wire.h>

// Constant definitions
const int MPU = 0x68;

// Global flags variables
boolean g_bCharging;
boolean g_bTs;
boolean g_bPerformedTs;
boolean g_bLEDs;
boolean g_bLEDr;

// Global variables
unsigned long g_time;
unsigned long g_touchTime;
byte g_nTouch;
byte g_nLEDr;
int g_nLoopCount;

// MPU6050 global variables
float AccSens, GyroSens;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccAngleX, AccAngleY, GyroAngleX, GyroAngleY, GyroAngleZ;
float Roll, Pitch, Yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
unsigned long AccElapsedTime, AccCurrentTime, AccPrevTime;

// Function prototypes
void touchSensor(void);
void checkTouchTS(void);
void checkBattery(void);
void ledWave(void);
void ledDifuseChange(void);
void ledFadeON(void);
void ledFadeOFF(void);
void ledBattStatus(void);



void setup() {
  pinMode(PIN_TS, INPUT);
  pinMode(PIN_LED_W, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_BZ,OUTPUT);
  pinMode(PIN_LEDS, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  // pinMode(PIN_TXB, OUTPUT);
  // pinMode(PIN_RXB, OUTPUT);
  pinMode(PIN_BAT_CH, INPUT);
  pinMode(PIN_BAT, INPUT);

  digitalWrite(PIN_LED_W, LOW);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_R, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_TS), touchSensor, CHANGE);
  mpu6050setup();

  g_bCharging = false;
  g_bTs = false;
  g_bPerformedTs = false;
  g_bLEDs = false;
  g_bLEDr = false;
  g_time =  0;
  g_touchTime = 0;
  g_nTouch = 0;
  g_nLEDr = 0;
  g_nLoopCount = 0;


  Serial.begin(9600);

}

void loop() {

  checkTouchTS();

  // Check battery each 250 ms delay.
  if (g_nLoopCount % 250 == 0) {
    checkBattery();
  }

  // Get Accel & Gyro data each 20 ms delay
  if (g_nLoopCount % 20 == 0) {
    mpu6050read();
  }

  // Use to divide code into fast or slow operation.
  g_nLoopCount++;
  if (g_nLoopCount == 30000) g_nLoopCount = 0;
  delay(1);
}

/* Interrupt function for touch sensor */
void touchSensor() {
  // Start Timer
  g_touchTime = millis();
  g_time = 0;

  // Check if it's a falling edge
  if (g_bTs) {
    // Check if it is a pulse or a maintained touch
    if (!g_bPerformedTs)  g_nTouch++;
    else                  g_touchTime = 0;
  }

  // Update state
  g_bTs = !g_bTs;
  g_bPerformedTs = 0;
  digitalWrite(PIN_LED_W, g_bTs);
}

/* Check Touch Sensor after Touch Sensor interrupt */
void checkTouchTS() {
  // If timer is ON, update timer.
  if (g_touchTime != 0) g_time = millis() - g_touchTime;

  // Touch Sensor High. Check for Maintained touch
  if (g_bTs) {
    // Check if Touch Sensor is mantained for 1000 ms
    if (g_time > 1000 && !g_bPerformedTs) {
      // Button maintained
      g_bPerformedTs = true;
      // Stop Timer
      g_touchTime = 0;
      // If only was one touch previously
      if (g_nTouch == 1) {
        // Battery status Blink
        ledBattStatus();
      } else {
        // Only maintained (ON/OFF)
        g_bLEDs = !g_bLEDs;
        if (g_bLEDs)  ledFadeON();
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
          ledWave();
        } else if (g_nTouch == 2) {
          ledDifuseChange();
        }
        g_nTouch = 0;
      }
    }
  }
}

void checkBattery() {
  g_nLEDr++; // Increment every ~250 ms
  if (g_nLEDr > 100) {
    g_nLEDr = 1;
  }
  int batValue = analogRead(PIN_BAT);
  // Analog Read value from 0-5V (0-1023). Convert directly to 0-8.4V (0-840) battery voltage.
  int val = map(batValue, 0, 1023, 0, 840);
  // Serial.println(val);

  // 100% -> 4.2V | 80% -> 3.9V | 60% -> 3.8V | 40% -> 3.7V | 20% -> 3.6V | 0% -> 3V
  if (digitalRead(PIN_BAT_CH)) {
    // Charge mode
    if (val < 740) {
      // LED_R blink slow for battery 0%-40%
      if (g_nLEDr % 4 == 0) {
        g_bLEDr = !g_bLEDr;
        digitalWrite(PIN_LED_R, g_bLEDr);
      }
    } else if (val < 780) {
      // LED_R blink fast for battery 40%-80%
      g_bLEDr = !g_bLEDr;
      digitalWrite(PIN_LED_R, g_bLEDr);
    } else {
      // LED_R ON for battery 80%-100%
      digitalWrite(PIN_LED_R, HIGH);
    }
  } else {
    // Normal mode
    if (val > 740) {
      // LED_R OFF for battery 100%-40%
      digitalWrite(PIN_LED_R, LOW);
    } else if (val > 720) {
      // LED_R blink slow for battery 40%-20%
      if (g_nLEDr % 4 == 0) {
        g_bLEDr = !g_bLEDr;
        digitalWrite(PIN_LED_R, g_bLEDr);
      }
    } else {
      // LED_R blink fast for battery 20%-0%
      g_bLEDr = !g_bLEDr;
      digitalWrite(PIN_LED_R, g_bLEDr);
    }
  }
}

void ledWave() {
  Serial.println("1 press Touch Sensor");
  //TODO


}

void ledDifuseChange() {
  Serial.println("2 press Touch Sensor");
  //TODO

}

void ledFadeON() {
  Serial.println("Switch ON Sword");
  //TODO

}

void ledFadeOFF() {
  Serial.println("Switch OFF Sword");
  //TODO

}

void ledBattStatus() {
  Serial.println("Battery Check");
  digitalWrite(PIN_LED_R, HIGH);
  //TODO

}



void mpu6050setup() {
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
  AccSens = 4096;
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
  GyroSens = 32.8;
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
    AccX = (Wire.read() << 8 | Wire.read()) / AccSens ;
    AccY = (Wire.read() << 8 | Wire.read()) / AccSens ;
    AccZ = (Wire.read() << 8 | Wire.read()) / AccSens ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
  }

  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;

  // Read gyro values 200 times
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / GyroSens);
    GyroErrorY = GyroErrorY + (GyroY / GyroSens);
    GyroErrorZ = GyroErrorZ + (GyroZ / GyroSens);
  }

  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

void mpu6050read() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  // Read 6 registers total, each axis value is stored in 2 registers
  AccX = (Wire.read() << 8 | Wire.read()) / AccSens;
  AccY = (Wire.read() << 8 | Wire.read()) / AccSens;
  AccZ = (Wire.read() << 8 | Wire.read()) / AccSens;

  // Compute Roll and Pitch from the accelerometer data
  AccAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  AccAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  // === Read gyroscope data === //
  AccPrevTime = AccCurrentTime;        // Previous time is stored before the actual time read
  AccCurrentTime = millis();            // Current time actual time read
  AccElapsedTime = (AccCurrentTime - AccPrevTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  // Read 6 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / GyroSens;
  GyroY = (Wire.read() << 8 | Wire.read()) / GyroSens;
  GyroZ = (Wire.read() << 8 | Wire.read()) / GyroSens;

  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;

  // Currently the raw values are in deg/s,
  // so we need to multiply by sendonds (s) to get the angle in degrees
  GyroAngleX = GyroAngleX + GyroX * AccElapsedTime;
  GyroAngleY = GyroAngleY + GyroY * AccElapsedTime;
  Yaw =  Yaw + GyroZ * AccElapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  Roll = 0.96 * GyroAngleX + 0.04 * AccAngleX;
  Pitch = 0.96 * GyroAngleY + 0.04 * AccAngleY;

  // Print the values on the serial monitor
  Serial.print("Gyro:");
  Serial.print(Roll);
  Serial.print("/");
  Serial.print(Pitch);
  Serial.print("/");
  Serial.println(Yaw);
  Serial.print("Acc:");
  Serial.print(AccX);
  Serial.print("/");
  Serial.print(AccY);
  Serial.print("/");
  Serial.println(AccZ);



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

    - Roll at X Axis -> Sword Rotation (Defense guard)
    - Pitch at Y Axis -> Vertical Cut -> Accel Z & X
    - Yaw at Z Axis -> Horizontal Cut -> Accel Y & X

  */

}

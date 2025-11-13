#include <SPI.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "ESC.h"
#include "nRF24L01.h"
#include "RF24.h"

Adafruit_MPU6050 mpu;


int lastTelemetryTime = 0;


ESC fan(32, 1000, 2000, 500);  // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Arm Value)
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 500;
const int PWM_RESOLUTION = 8;

//TX Radio Values
const uint64_t planeAddr = 0xF0F0F0F0E1LL;

int progTimerStart;
int countDown = 5;
int lastCommandTime;
int R1 = 10700;
int R2 = 2000;
int adcVal;
int fanSpeed;
int indexx = 0;
const int N = 10;
float refVoltage = 3.3;
float voltPlane;
float adcVolt;
float accelY;
float averageAccelY;
float armAverageAccelY;
float accelWindow[N];
float sum = 0;
uint16_t swL;
uint16_t swR;
uint16_t JLX;
uint16_t JLY;
uint16_t JRX;
uint16_t JRY;
float voltPC;
bool filled = false;
bool parachuteLoad = false;
bool solenoidEnable = true;
bool launchDetected = false;
bool failSafe = false;
bool timerEnable = false;

Servo TL;
Servo TR;
Servo BL;
Servo Br;
Servo etc;

struct CommandPacket {
  uint16_t swL;
  uint16_t swR;
  uint16_t JLX;
  uint16_t JLY;
  uint16_t JRX;
  uint16_t JRY;
  bool solenoidEnable;
  bool failSafe;
  int remoteState;
};

struct TelemetryPacket {
  float voltPlane;
  bool failSafe;
} __attribute__((packed));


TelemetryPacket telemetry;
CommandPacket cmd;


RF24 _radio(4, 5);

enum PlaneState {
  STATE_WAITING,
  STATE_ARM_PREP,
  STATE_LAUNCHING,
  STATE_FLYING,
  STATE_FAILSAFE_PLANE
};
PlaneState state = STATE_WAITING;  //Remote is in standby at startup

enum RemoteState {
  STATE_STANDBY,
  STATE_MENU,
  STATE_PARACHUTE,
  STATE_TEST_FIRE,
  STATE_ARMING,
  STATE_CANNOT_ARM_CHUTE,
  STATE_READY_TO_FIRE,
  STATE_TEST_FIRE_WAIT,
  STATE_FIRING,
  STATE_FAILSAFE
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(35, INPUT);
  TL.attach(13);
  TR.attach(12);
  BL.attach(14);
  Br.attach(27);
  // Wings.attach(26);
  etc.attach(25);
  _radio.begin();
  _radio.setPALevel(RF24_PA_MAX);
  _radio.setDataRate(RF24_250KBPS);
  _radio.openReadingPipe(1, planeAddr);
  _radio.enableAckPayload();
  _radio.startListening();

  fan.calib();
  fan.arm();
  //ledcAttach(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  delay(1000);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  getPlaneVoltage();
  getPlaneMode();
  testFailsafe();
  readRadio();

  switch (state) {
    //Waiting when nothing is happening
    case STATE_WAITING:
      printReceive();
      break;

    case STATE_ARM_PREP:
      accelY = a.acceleration.y;
      averageAccelValues();
      armAverageAccelY = averageAccelY;
      Serial.print("averageAccelY: ");
      Serial.println(averageAccelY);
      Serial.print(" Rem State: ");
      Serial.println(state);
      break;

    case STATE_LAUNCHING:
      launchDetection();
      startCountdown();
      Serial.print(" Rem State: ");
      Serial.print(state);
      Serial.print(" countDown: ");
      countDown = 5 - (millis() - progTimerStart) / 1000;
      Serial.println(countDown);
      if (countDown <= 0 && !launchDetected) {
        state = STATE_FAILSAFE_PLANE;
        endCountdown();
        Serial.println("Launch Failure!!!");
        failSafe = true;
      } else if (launchDetected) {
        Serial.println("Launch Detected!");
        state = STATE_FLYING;
        endCountdown();
      }
      break;

    case STATE_FLYING:
      Serial.println("Flying");
      Serial.print(" Rem State: ");
      Serial.println(state);
      break;

    case STATE_FAILSAFE_PLANE:
      Serial.println("Failsafe Tripped!");
      startCountdown();
      Serial.print(" Rem State: ");
      Serial.println(state);
      solenoidEnable = false;
      countDown = 3 - (millis() - progTimerStart) / 1000;
      if (countDown <= 0) {
        state = STATE_WAITING;
        endCountdown();
      }
      break;
  }
}


//Functions to call in the main program
void launchDetection() {
  averageAccelValues();
  if (abs(averageAccelY - armAverageAccelY) > 8) {
    launchDetected = true;
  }
}

float averageAccelValues() {
  //Read the latest acceleration value

  //Subtract the old value from the sum if the buffer is full
  if (filled) {
    sum -= accelWindow[indexx];
  }

  //Add the new value to the array and sum
  accelWindow[indexx] = accelY;
  sum += accelY;

  //Advance the index, wrapping around using modulus
  indexx = (indexx + 1) % N;

  //Mark as filled after the first full pass
  if (indexx == 0) {
    filled = true;
  }

  //Return the average of the last N values
  if (filled) {
    return averageAccelY = sum / N;
  }
}

void testFailsafe() {
  if (millis() - lastCommandTime > 1000) {
    failSafe = true;
    state = STATE_FAILSAFE_PLANE;
  } else {
    failSafe = false;
  }
}

float getPlaneVoltage() {
  adcVal = analogRead(35);
  adcVolt = (adcVal * refVoltage) / 4096;
  return voltPlane = adcVolt * (R1 + R2) / R2;
}

void readRadio() {
  if (_radio.available()) {
    _radio.read(&cmd, sizeof(cmd));
    telemetry.voltPlane = voltPlane;  // Example scaling
    telemetry.failSafe = failSafe;
    _radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
    lastCommandTime = millis();
  }
}

void getPlaneMode() {
  if (!failSafe && !timerEnable) {
    if (cmd.remoteState >= 0 && cmd.remoteState < 4 || cmd.remoteState == 5) {
      state = STATE_WAITING;
    } else if (cmd.remoteState == 4 || cmd.remoteState == 6 || cmd.remoteState == 7) {
      state = STATE_ARM_PREP;
    } else if (cmd.remoteState == 8) {
      state = STATE_LAUNCHING;
    }
  } else if (cmd.remoteState == 9 || failSafe) {
    state = STATE_FAILSAFE_PLANE;
  }
}

void startCountdown() {
  if (timerEnable == false) {
    progTimerStart = millis();
    timerEnable = true;
  }
}

void endCountdown() {
  if (timerEnable) {
    timerEnable = false;
    countDown = 3;
  }
}

void printReceive() {
  Serial.print("swL: ");
  Serial.print(cmd.swL);
  Serial.print(" swR: ");
  Serial.print(cmd.swR);
  Serial.print(" JLX: ");
  Serial.print(cmd.JLX);
  Serial.print(" JLY: ");
  Serial.print(cmd.JLY);
  Serial.print(" JRX: ");
  Serial.print(cmd.JRX);
  Serial.print(" JRY: ");
  Serial.print(cmd.JRY);
  Serial.print(" solenoidEnable: ");
  Serial.print(cmd.solenoidEnable);
  Serial.print(" Rem State: ");
  Serial.println(state);
}
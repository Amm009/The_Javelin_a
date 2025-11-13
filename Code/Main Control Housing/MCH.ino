#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <FastLED.h>

//Display setup
#define i2c_Address 0x3c  //initialize with the I2C addr 0x3C Typically eBay OLED's
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     //   QT-PY / XIAO
#define LED_PIN 13
#define NUM_LEDS 22
#define BRIGHTNESS 100
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//End display setup

//LED setup
CRGB leds[NUM_LEDS];
//End LED setup

//Generic variable setup
uint16_t swL;
uint16_t swR;
uint16_t JLX;
uint16_t JLY;
uint16_t JRX;
uint16_t JRY;
int progTimerStart;
int launchPin = 32;
int menuPage = 0;
int countDown = 3;
bool timerEnable = false;
bool swL_Rel = true;
bool swR_Rel = true;
bool JLX_Rel = true;
bool testFire = false;
bool solenoidEnable = true;
bool failSafe = false;
float voltPlane;
float voltMCH;
//End generic variable setup

//Radio setup
const uint64_t planeAddr = 0xF0F0F0F0E1LL;
uint8_t retries;

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

CommandPacket cmd;
TelemetryPacket telemetry;

RF24 _radio(4, 5);
//End radio setup

//Defining states that the remote can be in
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
RemoteState state = STATE_STANDBY;  //Remote is in standby at startup


void setup() {
  //Generic startup values
  Serial.begin(115200);
  display.begin(i2c_Address, true);
  display.display();
  display.clearDisplay();
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(25, INPUT);
  pinMode(33, INPUT);
  pinMode(launchPin, OUTPUT);
  digitalWrite(launchPin, HIGH);

  //Start radio and configure settings
  _radio.begin();
  _radio.setPALevel(RF24_PA_MAX);
  _radio.setDataRate(RF24_250KBPS);
  _radio.openWritingPipe(planeAddr);
  _radio.setRetries(2, 15);
  _radio.enableAckPayload();
  _radio.stopListening();

  //Setup LEDs
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  //Show a splash screen on startup
  splashScreen();

  delay(1000);
}

void loop() {

  readInputs();  //Read all of the inputs from the controller
  sendData();  //Send relevant data to the plane and wait for an acknowledgement packet in return
  testFailsafe();

  //Check to see if the joysticks or buttons have been released
  swL_Rel = swLRelease();
  swR_Rel = swRRelease();
  JLX_Rel = JLXRelease();

  switch (state) {
    //Show the standby screen when nothing is happening
    case STATE_STANDBY:
      infoScreen();
      if (menuCombo()) {
        JLX_Rel = false;
        state = STATE_MENU;
      }
      if (leftTriggerPressed() && !rightTriggerPressed() && swL_Rel && solenoidEnable) {
        swL_Rel = false;
        state = STATE_ARMING;
      } else if (leftTriggerPressed() && !rightTriggerPressed() && swL_Rel && !solenoidEnable) {
        state = STATE_CANNOT_ARM_CHUTE;
        swL_Rel = false;
      }
      break;

    //Show the menu when the joysticks have been put in the right spot
    case STATE_MENU:
      menuScreen();
      if (nextMenu()) {
        menuPage++;
        JLX_Rel = false;
      }
      if (prevMenu()) {
        menuPage--;
        JLX_Rel = false;
      }
      if (leftTriggerPressed()) {
        state = STATE_STANDBY;
        menuPage = 0;
        swL_Rel = false;
      }
      if (menuPage == 1) {
        state = STATE_PARACHUTE;
      } else if (menuPage == 2) {
        state = STATE_TEST_FIRE;
      } else if (menuPage == 0 && !leftTriggerPressed()) {
        state = STATE_MENU;
        swL_Rel = false;
      }
      break;

    //Show the parachute door page
    case STATE_PARACHUTE:
      if (solenoidEnable) {
        parachuteLoadScreenClosed();
      } else {
        parachuteLoadScreenOpen();
      }

      if (optionToggle()) {
        solenoidEnable = !solenoidEnable;
        swR_Rel = false;
      }
      if (nextMenu()) {
        menuPage++;
        JLX_Rel = false;
      }
      if (prevMenu()) {
        menuPage--;
        JLX_Rel = false;
      }
      if (leftTriggerPressed()) {
        state = STATE_STANDBY;
        menuPage = 0;
        swL_Rel = false;
      }
      if (menuPage == 2) {
        state = STATE_TEST_FIRE;
      } else if (menuPage == 0) {
        state = STATE_MENU;
      }
      break;

    //Show the test fire page
    case STATE_TEST_FIRE:
      if (testFire) {
        testFireOn();
      } else {
        testFireOff();
      }
      if (optionToggle()) {
        testFire = !testFire;
        swR_Rel = false;
      }
      if (nextMenu()) {
        menuPage++;
        JLX_Rel = false;
      }
      if (prevMenu()) {
        menuPage--;
        JLX_Rel = false;
      }
      if (leftTriggerPressed()) {
        state = STATE_STANDBY;
        menuPage = 0;
        swL_Rel = false;
      }
      if (menuPage == 1) {
        state = STATE_PARACHUTE;
      } else if (menuPage == 0) {
        state = STATE_MENU;
      }
      break;

    //Show the cannot arm screen while the left trigger is pressed and the parachute door is unlocked
    case STATE_CANNOT_ARM_CHUTE:
      parachuteNOARM();
      if (!leftTriggerPressed()) {
        state = STATE_STANDBY;
      }
      break;

    //Show the arming screen while the left trigger is pressed while in the standby screen
    case STATE_ARMING:
      startCountdown();
      armingScreen();
      countDown = 3 - (millis() - progTimerStart) / 1000;
      if (countDown <= 0) {
        state = STATE_READY_TO_FIRE;
        endCountdown();
      }
      if (!leftTriggerPressed() || rightTriggerPressed()) {
        state = STATE_STANDBY;
        endCountdown();
      }
      break;

    //Show the ready to fire screen when the left trigger has been held for 3 or more seconds
    case STATE_READY_TO_FIRE:
      firereadyScreen();
      if (fireCombo()) {
        if (testFire) {
          state = STATE_TEST_FIRE_WAIT;
        } else {
          state = STATE_FIRING;
        }
        swR_Rel = false;
      }
      if (!leftTriggerPressed()) {
        state = STATE_STANDBY;
      }
      break;

    //Show the firing screen when the ready to fire screen has been shown and then the right trigger is pressed
    case STATE_FIRING:
      firingScreen();
      digitalWrite(launchPin, LOW);
      startCountdown();
      countDown = 3 - (millis() - progTimerStart) / 1000;
      if (countDown <= 0) {
        state = STATE_STANDBY;
        digitalWrite(launchPin, HIGH);
        endCountdown();
      }
      break;

    //Show the test fire screen when appropriate
    case STATE_TEST_FIRE_WAIT:
      testFireWaitScreen();
      startCountdown();
      countDown = 30 - (millis() - progTimerStart) / 1000;
      if (countDown <= 0) {
        state = STATE_FIRING;
        endCountdown();
      }
      if (leftTriggerPressed() && swL_Rel) {
        state = STATE_STANDBY;
        swL_Rel = false;
        endCountdown();
      }
      break;

    //Show the failsafe screen if it is triggered
    case STATE_FAILSAFE:
      parachuteNOARM();
      if (!leftTriggerPressed()) {
        state = STATE_STANDBY;
      }
      break;
  }
}


//Functions to call in the main program
bool menuCombo() {
  return (JLX > 3000 && JLY < 1000 && JRX < 1100 && JRY < 1000 && swL == 1 && swR == 1 && JLX_Rel == true);
}
bool leftTriggerPressed() {
  return (swL == 0);
}
bool rightTriggerPressed() {
  return (swR == 0);
}
bool fireCombo() {
  return (swL == 0 && swR == 0 && swL_Rel == false && swR_Rel == true);
}
bool nextMenu() {
  return (JLX > 3000 && swL == 1 && swR == 1 && menuPage < 2 && JLX_Rel == true);
}
bool prevMenu() {
  return (JLX < 1000 && swL == 1 && swR == 1 && menuPage > 0 && JLX_Rel == true);
}
bool optionToggle() {
  return (swL == 1 && swR == 0 && swR_Rel == true);
}


//Functions below:

void sendData() {
  cmd.swL = swL;
  cmd.swR = swR;
  cmd.JLX = JLX;
  cmd.JLY = JLY;
  cmd.JRX = JRX;
  cmd.JRY = JRY;
  cmd.solenoidEnable = solenoidEnable;
  cmd.failSafe = failSafe;
  cmd.remoteState = (int)state;
  if (_radio.write(&cmd, sizeof(cmd))) {
    retries = _radio.getARC();
    if (_radio.isAckPayloadAvailable()) {
      _radio.read(&telemetry, sizeof(telemetry));
    }
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

void testFailsafe() {
  if (retries >= 5) {
    failSafe = true;
    state = STATE_FAILSAFE;
  }
}

void readInputs() {
  swL = digitalRead(16);
  swR = digitalRead(15);
  JLX = analogRead(26);
  JLY = analogRead(27);
  JRX = analogRead(25);
  JRY = analogRead(33);
  voltMCH = analogRead(34);
}

void testing() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("swtchL:");
  display.setCursor(50, 0);
  display.println(swL);
  display.setCursor(70, 0);
  display.println("JLX:");
  display.setCursor(100, 0);
  display.println(JLX);
  display.setCursor(0, 20);
  display.println("JLY:");
  display.setCursor(50, 20);
  display.println(JLY);
  display.setCursor(80, 20);
  display.println("swtchR:");
  display.setCursor(120, 20);
  display.println(swR);
  display.setCursor(0, 40);
  display.println("JRX:");
  display.setCursor(50, 40);
  display.println(JRX);
  display.setCursor(80, 40);
  display.println("JRY:");
  display.setCursor(100, 40);
  display.println(JRY);
  display.display();
  delay(10);
}

void parachuteNOARM() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(5, 15);
  display.println("Chute Door Unlocked");
  display.setCursor(40, 35);
  display.println("CANNOT ARM");
  display.display();
}

void drawMenuDots() {
  int y = 58;        // Vertical position near bottom of screen
  int xStart = 50;   // Where to start drawing the dots
  int spacing = 12;  // Spacing between dots
  int radius = 3;    // Size of each dot

  for (int i = 0; i < 3; i++) {
    int x = xStart + i * spacing;

    if (i == menuPage) {
      // Active dot: filled
      display.fillCircle(x, y, radius, SH110X_WHITE);
    } else {
      // Inactive dot: outline (looks "grey")
      display.drawCircle(x, y, radius, SH110X_WHITE);
    }
  }
}

void menuScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(39, 15);
  display.println("Settings");
  drawMenuDots();
  display.display();
}

void parachuteLoadScreenClosed() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(19, 15);
  display.println("Parachtue Door:");
  display.drawRoundRect(43, 33, 39, 11, 2, SH110X_WHITE);
  display.setCursor(45, 35);
  display.println("Closed");
  drawMenuDots();
  display.display();
}

void parachuteLoadScreenOpen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(19, 15);
  display.println("Parachtue Door:");
  display.fillRoundRect(48, 33, 27, 12, 2, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(50, 35);
  display.println("Open");
  drawMenuDots();
  display.display();
}


void testFireOff() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(30, 15);
  display.println("Test Fire:");
  display.drawRoundRect(51, 33, 21, 11, 2, SH110X_WHITE);
  display.setCursor(53, 35);
  display.println("Off");
  drawMenuDots();
  display.display();
}

void testFireOn() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(30, 15);
  display.println("Test Fire:");
  display.fillRoundRect(54, 33, 15, 11, 2, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(56, 35);
  display.println("On");
  drawMenuDots();
  display.display();
}

void armingScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 0);
  display.println("ARMING...");
  display.setCursor(60, 30);
  display.setTextSize(3);
  display.println(countDown);
  display.display();
}

void firingScreen() {
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 20);
  display.println("FIRING");
  display.display();
}

void testFireWaitScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(35, 0);
  display.println("DELAYED");
  display.setCursor(43, 20);
  display.println("FIRING");
  display.setCursor(67, 40);
  display.println(countDown);
  display.display();
}

void splashScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(5, 30);
  display.println("Javelin(a)");
  display.display();
}

void firereadyScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 25);
  display.println("Press Right Trigger");
  display.setCursor(45, 35);
  display.println("to Fire");
  display.display();
}

void infoScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(30, 0);
  display.println("Info Screen");
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 15);
  display.println("Plane Batt:");
  display.setCursor(70, 15);
  display.println(telemetry.voltPlane);
  display.setCursor(100, 15);
  display.println("V");
  display.setCursor(0, 35);
  display.println("Retries:");
  display.setCursor(50, 35);
  display.println(retries);
  if (telemetry.voltPlane < 14.4) {
    display.setCursor(30, 55);
    display.println("LOW BATTERY");
  }
  display.display();
  delay(10);
}

bool swLRelease() {
  //Reset Triggers if they are let go
  if (swL == 1) {
    swL_Rel = true;
  }
  return swL_Rel;
}

bool swRRelease() {
  //Reset Triggers if they are let go
  if (swR == 1) {
    swR_Rel = true;
  }
  return swR_Rel;
}

bool JLXRelease() {
  if (JLX > 1500 && JLX < 2500) {
    JLX_Rel = true;
  }
  return JLX_Rel;
}
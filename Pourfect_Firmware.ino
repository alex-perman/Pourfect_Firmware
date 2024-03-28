/*  Pourfect Tea Machine Firmware
 *  Alex Perman Copyright 2024
 *  
 *  
 */

// Include Libraries
#include <A4988.h>
#include <HX711.h>
#include <Adafruit_PCF8574.h>

#include <ItemSubMenu.h>
#include <LcdMenu.h>
#include <utils/commandProccesors.h>

#include <Adafruit_PCF8574.h>
#include <Servo.h>

#include <PID_v1.h>

#include <RotaryEncoder.h>

// Pin Definitions
#define SERVO_0     5   // PD5
#define SERVO_1     6   // PD6
#define PUMP_0      9   // PB1
#define PUMP_1      8
#define PUMP_2      7
#define LC_DOUT     3
#define LC_SCK      2
#define SOLENOID    4
#define THERMISTOR  A0
#define HEATER      A1
#define INT         A2
#define LEAF_STEP   10
#define INF_STEP    11
#define LEAF_DIR    12
#define INF_DIR     13

#define LEAF_LIMIT  0   // P0
#define INF_LIMIT   1   // P1
#define RE_CLK      2   // P2
#define RE_DT       3   // P3
#define RE_SW       4   // P4
//=============================//

#define STEPPER_STEPS 200
A4988 S_leaf(STEPPER_STEPS, LEAF_DIR, LEAF_STEP);
A4988 S_inf(STEPPER_STEPS, INF_DIR, INF_STEP);

Servo T_Servo;
#define T_CLOSED 100
#define T_OPEN 30
#define T_WASTE 150
Servo infServo;
#define INF_DUMP 150
#define INF_UP 0

Adafruit_PCF8574 pcf;

RotaryEncoder *encoder = nullptr;

// LCD ****************************
#define LCD_ROWS  2
#define LCD_COLS  16
#define UP        56
#define DOWN      50
#define ENTER     53
#define BACK      55
extern MenuItem* settingsMenu[];
// Define Main Menu
MAIN_MENU(
  ITEM_BASIC("Make Tea"),
  ITEM_SUBMENU("Settings", settingsMenu),
  ITEM_BASIC("About Us...")
);
// Settings Sub Menu
SUB_MENU(settingsMenu, mainMenu,
  ITEM_BASIC("Steeping Time"),
  ITEM_BASIC("Steeping Temp"),
  ITEM_BASIC("Cleaning Cycle")
);
LcdMenu menu(LCD_ROWS, LCD_COLS);

// Globals
int inf_disp;
int leaf_disp;

double waterTemp, thermTemp, pumpSpeed;
double Kp=2, Ki=0, Kd=0;
PID steepingTemp(&thermTemp, &pumpSpeed, &waterTemp, Kp, Ki, Kd, DIRECT);
#define POUR_VOLUME 250   // mL of tea steeped

#define STEEPING_MINS 5   // minutes
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long steepingTime = STEEPING_MINS*60*1000;

// Rotary Encoder
void checkRE() {    
  encoder->tick();    // check RE State
}

void setup() {
  S_leaf.begin(100, 1);
  S_inf.begin(100, 1);
  T_Servo.attach(SERVO_0);
  infServo.attach(SERVO_1);

  pinMode(INF_LIMIT, INPUT);
  pinMode(LEAF_LIMIT, INPUT);

  menu.setupLcdWithMenu(0x20, mainMenu);

  // PID
  // Initialize PID Variables
  thermTemp = analogRead(THERMISTOR);
  waterTemp = 80;


  while (!Serial) { delay(10); }
  Serial.begin(115200);

  if (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }
  for (uint8_t p=0; p<8; p++) {
    pcf.pinMode(p, INPUT_PULLUP);
  }

  // TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position
  encoder = new RotaryEncoder(RE_CLK, RE_DT, RotaryEncoder::LatchMode::TWO03);
  // Register intererrupt routine
  attachInterrupt(digitalPinToInterrupt(INT), checkRE, CHANGE);
}

void stepperHome() {
  int infHB = pcf.digitalRead(INF_LIMIT); 
  int leafHB = pcf.digitalRead(LEAF_LIMIT);
  
  while (infHB == HIGH)
  {
    //backwards slowly till it hits the switch and stops
    S_inf.rotate(-100);
    infHB = pcf.digitalRead(INF_LIMIT);
  }
  inf_disp = 0;

  while (leafHB == HIGH)
  {
    S_leaf.rotate(-100);
    leafHB = pcf.digitalRead(LEAF_LIMIT);
  }
  leaf_disp = 0;
}

void moveInfuser(int pos) {
  int inf_pos;
  infServo.write(INF_UP);
  delay(100);
  
  if (pos == 0) {
    inf_pos = 0;      // Home position
  }
  else if (pos = 1) {
    inf_pos = 2000;   // Steeping Position
  }
  else if (pos = 2) {
    inf_pos = 1000;   // Cleaning Position
  }

  if (inf_disp < inf_pos) {
    while (inf_disp < inf_pos) {
      S_inf.rotate(100);
    }
  }
  else if (inf_disp > inf_pos) {
    while (inf_disp > inf_pos) {
      S_inf.rotate(-100);
    }
  }

}

void dispenseLeaf() {
  if (leaf_disp == 0) {
    S_leaf.rotate(5*360);
    int i;

    for (i = 0; i < 5; i++) {   // Shake leaves into volumetric measure
      S_leaf.rotate(5);
      S_leaf.rotate(-5);
    }

    delay(1000);
    S_leaf.rotate(-5*360);

    for (i = 0; i < 5; i++) {   // Shake leaves into infuser
      S_leaf.rotate(5);
      S_leaf.rotate(-5);
    }
  }
  
}

void T_junct(int pos) {
  if (pos = 0) {
    T_Servo.write(T_CLOSED);
  }
  else if (pos = 1) {
    T_Servo.write(T_OPEN);
    delay(10000);             // wait 10s
  }
  else if (pos = 2) {
    T_Servo.write(T_WASTE);
  }
  delay(100);
}

void waterControl() {
  float teaVolume = 0;
  int thermC;
  steepingTemp.SetMode(AUTOMATIC);    // Turn ON PID
  delay(1000);                        // Delay for fun?
  digitalWrite(HEATER, HIGH);         // Turn ON Water Heater

  while (teaVolume < POUR_VOLUME) {
    thermTemp = analogRead(THERMISTOR);
    steepingTemp.Compute();
    analogWrite(PUMP_0, pumpSpeed);   // PWM Pump Speed

    thermC = map(((thermTemp - 20) * 3.04), 0, 1023, -40, 125);   // deffo gonna have to change this for thermistor
    if ((thermC > waterTemp - 5) && (thermC < waterTemp + 5)) {   // +/- 5degC 
      digitalWrite(SOLENOID, HIGH);   // Turn Solenoid ON
      teaVolume = teaVolume + map(pumpSpeed, 0, 255, 0, 3);       // mL/100ms [default PID sample rate]
    }
    else {
      digitalWrite(SOLENOID, LOW);    // Otherwise, Solenoid OFF
    }
  }

  digitalWrite(HEATER, LOW);      // Turn OFF Water Heater
  steepingTemp.SetMode(MANUAL);   // Turn OFF PID
  analogWrite(PUMP_0, 0);         // Turn OFF Pump
}

void steepTea() {
  startMillis = millis();
  currentMillis = millis();

  digitalWrite(PUMP_1, HIGH);
  delay(5000);      // wait for pump to clear water from lines
  digitalWrite(PUMP_1, LOW);

  while (currentMillis - startMillis < steepingTime) {
  }

}

void washCycle() {
  
}

void makeTea() {
  stepperHome();        // Home steppers
  moveInfuser(0);       // Raise infuser to load leaves
  dispenseLeaf();       // Portion leaves into infuser
  T_junct(0);           // Close T_junction
  moveInfuser(1);       // Lower infuser to steeping position
  waterControl();       // Water pump & PID
  steepTea();           // Blow out lines and steep tea for steeping time
  T_junct(1);           // Pour steeped tea into cup
  T_junct(0);           // Close T-Junction
  moveInfuser(2);       // Bring infuser up to cleaning pos
  washCycle();          // Clean machine for next cup

}

void loop() {
  int command;
  static int pos = 0;

  encoder->tick();
  int newPos = encoder->getPosition();
  if (pos != newPos) {
    if (newPos < pos) {
      command = DOWN;
    }
    else if (newPos > pos) {
      command = UP;
    }
    pos = newPos;
  }

  processMenuCommand(menu, command, UP, DOWN, ENTER, BACK);

}

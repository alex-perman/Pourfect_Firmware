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
#include <ItemToggle.h>
#include <ItemInput.h>
#include <ItemCommand.h>
#include <ItemInput.h>
#include <LcdMenu.h>
#include <utils/commandProccesors.h>

#include <Servo.h>

#include <PID_v1.h>

#include "HX711.h"
HX711 scale;

//#include <rotaryDecoder.h>

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
#define T_CLOSED 75
#define T_OPEN 165
#define T_WASTE 0
Servo infServo;
#define INF_DUMP 145 
#define INF_UP 0

Adafruit_PCF8574 pcf;

// Thermistor Setup
int Vo;
float R1 = 100000;    // 100K Thermistor
float logR2, R2, T;
float c1 = 4.695312999e-03, c2 = -3.891975075e-04, c3 = 20.63019751e-07;
// Steinhart-Hart

// Rotary Encoder
int RE_pos = 0;
int aState;
int aLastState;  

// LCD ****************************
#define LCD_ROWS  2
#define LCD_COLS  16
#define UP        0
#define DOWN      1
#define LEFT      2
#define RIGHT     3
#define ENTER     4
#define BACK      5
#define CLEAR     6
#define BACKSPACE 7
extern MenuItem* settingsMenu[];
extern MenuItem* demoMenu[];
extern MenuItem* aboutUs[];
// Declare the call back function
void toggleBacklight(uint16_t isOn);
void backButton();
void preTeaMenu();
void demoRoutine();
void stepperHome();
void infDemo();
void leafDemo();
void T_Demo();
void waterDemo();
void leafEject();
void leafLoad();
void toggleCleaning(uint16_t isOn);
bool cleaningFlag = 1;
void setSteepingTime(int* value);

// Define Main Menu
MAIN_MENU(
  ITEM_COMMAND("Make Tea", preTeaMenu),
  ITEM_SUBMENU("Settings", settingsMenu),
  ITEM_SUBMENU("About Us...", aboutUs),
  ITEM_SUBMENU("DEMO", demoMenu)
);
// Settings Sub Menu
SUB_MENU(settingsMenu, mainMenu,
  ITEM_COMMAND("Back...", backButton),
  ITEM_INPUT("Steep Time", 10, setSteepingTime),
  ITEM_BASIC("Steeping Temp"),
  ITEM_TOGGLE("Cleaning", toggleCleaning),
  ITEM_COMMAND("Home Steppers", stepperHome),
  ITEM_COMMAND("Leaf Eject", leafEject),
  ITEM_COMMAND("Leaf Load", leafLoad),
  ITEM_TOGGLE("Backlight", toggleBacklight)
);
// Demo Sub Menu
SUB_MENU(demoMenu, mainMenu, 
  ITEM_COMMAND("Back...", backButton),
  ITEM_COMMAND("Infuser", infDemo),
  ITEM_COMMAND("Leaf", leafDemo),
  ITEM_COMMAND("T Junction", T_Demo),
  ITEM_COMMAND("Water", waterDemo),
  ITEM_COMMAND("Routine", demoRoutine)
);
// About Us
SUB_MENU(aboutUs, mainMenu,
  ITEM_BASIC("Pourfect Tea"),
  ITEM_BASIC("IGEN430 2024"),
  ITEM_BASIC("Logan Kahn"), 
  ITEM_BASIC("Jordan Myers"),
  ITEM_BASIC("Alex Perman"),
  ITEM_BASIC("Jessie Zhu"),
  ITEM_COMMAND("ඞ", backButton)
);
LcdMenu menu(LCD_ROWS, LCD_COLS);

// Globals
int inf_disp;
int leaf_disp;

// PID
double waterTemp, thermTemp, pumpSpeed;
double Kp=2, Ki=0, Kd=0;
PID steepingTemp(&thermTemp, &pumpSpeed, &waterTemp, Kp, Ki, Kd, DIRECT);
#define POUR_VOLUME 250   // mL of tea steeped

#define STEEPING_MINS 0.5   // minutes
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long steepingTime = STEEPING_MINS*60*1000;

//// Rotary Encoder
//void checkRE() {    
//  encoder->tick();    // check RE State
//}

void setup() {
  S_leaf.begin(150, 1);
  S_inf.begin(150, 1);
  T_Servo.attach(SERVO_0);
  infServo.attach(SERVO_1);

  pinMode(PUMP_0, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  scale.begin(LC_DOUT, LC_SCK);
  pinMode(SOLENOID, OUTPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(INT, INPUT);
  pinMode(LEAF_LIMIT, INPUT);
  pinMode(INF_LIMIT, INPUT);

  menu.setupLcdWithMenu(0x20, mainMenu);

  // PID
  // Initialize PID Variables
  thermTemp = analogRead(THERMISTOR);
  waterTemp = 20;


  while (!Serial) { delay(10); }
  Serial.begin(115200);
  Serial.println("Pourfect Tea DEBUG");

  if (!pcf.begin(0x21, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }
//  for (uint8_t p=2; p<8; p++) {           // IF YOU PUT THIS ON RELAYS BREAK IDK WHY
//    pcf.pinMode(p, INPUT_PULLUP);
//  }
   // Reads the initial state of the outputA
   aLastState = pcf.digitalRead(RE_CLK); 

   infServo.write(INF_UP);
   T_Servo.write(T_CLOSED);
}

void stepperHome() {
  int infHB = pcf.digitalRead(INF_LIMIT); 
  int leafHB = pcf.digitalRead(LEAF_LIMIT);

  //S_inf.rotate(-360);
  
  while (infHB == LOW)
  {
    //backwards slowly till it hits the switch and stops
    S_inf.rotate(30);
    infHB = pcf.digitalRead(INF_LIMIT);
  }
  inf_disp = 0;

  while (leafHB == LOW)
  {
    S_leaf.rotate(30);
    leafHB = pcf.digitalRead(LEAF_LIMIT);
  }
  leaf_disp = 0;

  infServo.write(0);
}

void moveInfuser(int pos) {
  int inf_pos;
  infServo.write(INF_UP);
  delay(100);
  
  if (pos == 0) {
    inf_pos = 0;      // Home position
  }
  else if (pos == 1) {
    inf_pos = -7.5*360;   // Steeping Position
  }
  else if (pos == 2) {
    inf_pos = -2.65*360;   // Cleaning Position
  }

  if (inf_disp < inf_pos) {
    while (inf_disp < inf_pos) {
      S_inf.rotate(30);
      inf_disp = inf_disp + 30;
//      if (pcf.digitalRead(INF_LIMIT) == HIGH) {
//        break;
//      }
    }
  }
  else if (inf_disp > inf_pos) {
    while (inf_disp > inf_pos) {
      S_inf.rotate(-30);
      inf_disp = inf_disp - 30;
//      if (pcf.digitalRead(INF_LIMIT) == HIGH) {
//        break;
//      }
    }
  }

}

void dispenseLeaf() {
  if (leaf_disp == 0) {
    S_leaf.rotate(-17.5*360);
    delay(1000);
    
    int i;
    for (i = 0; i < 100; i++) {   // Shake leaves into volumetric measure
      S_leaf.rotate(10);
      delay(5);
      S_leaf.rotate(-10);
      delay(5);
    }

    delay(1000);
    //S_leaf.rotate(18*360);
    int leafHB = pcf.digitalRead(LEAF_LIMIT);
    while (leafHB == LOW)       // Go home
    {
      S_leaf.rotate(30);
      leafHB = pcf.digitalRead(LEAF_LIMIT);
    }
    delay(1000);

    for (i = 0; i < 100; i++) {   // Shake leaves into infuser
      S_leaf.rotate(10);
      delay(5);
      S_leaf.rotate(-10);
      delay(5);
    }
  }
  
}

void T_junct(int pos) {
  if (pos == 0) {
    T_Servo.write(T_CLOSED);
  }
  else if (pos == 1) {
    T_Servo.write(T_OPEN);
  }
  else if (pos == 2) {
    T_Servo.write(T_WASTE);
  }
  delay(1000);
}

int getThermTemp() {
  Vo = analogRead(THERMISTOR);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  return T;
}

void waterControlPID() {
  float teaVolume = 0;
  int thermC;
  steepingTemp.SetMode(AUTOMATIC);    // Turn ON PID
  delay(1000);                        // Delay for fun?
  digitalWrite(HEATER, HIGH);         // Turn ON Water Heater

  while (teaVolume < POUR_VOLUME) {
    steepingTemp.Compute();
    analogWrite(PUMP_0, pumpSpeed);   // PWM Pump Speed

    //thermC = map(((thermTemp - 20) * 3.04), 0, 1023, -40, 125);   // deffo gonna have to change this for thermistor
    thermC = getThermTemp();
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

void waterControlOL() {
  delay(1000);                        // Delay for fun?
  digitalWrite(HEATER, HIGH);         // Turn ON Water Heater

  analogWrite(PUMP_0, 255);   // Full Pump Speed
  delay(2000);
  digitalWrite(SOLENOID, HIGH);   // Turn Solenoid ON
  delay(34722);                    // 250mL @ full power (255) // ya no nvm
  analogWrite(PUMP_0, 0);         // Pump OFF
  digitalWrite(PUMP_1, HIGH);
  delay(2000);
  digitalWrite(PUMP_1, LOW);
  digitalWrite(SOLENOID, LOW);    // Solenoid OFF
  digitalWrite(HEATER, LOW);      // Heater OFF
  delay(500);
}

void steepTea() {
  startMillis = millis();
  currentMillis = millis();

  digitalWrite(PUMP_1, HIGH);
  delay(1000);      // wait for pump to clear water from lines
  digitalWrite(PUMP_1, LOW);

  while (currentMillis - startMillis < steepingTime) { currentMillis = millis(); }

}

void washCycle() {
  if (cleaningFlag) {
    T_junct(0);               // Close T
    moveInfuser(2);           // Cleaning Position
    delay(500);
    infServo.write(INF_DUMP);
    delay(1000);
    //digitalWrite(PUMP_2, HIGH);
    delay(2000);
    //digitalWrite(PUMP_2, LOW);
    delay(5000);
    T_junct(2);               // Waste T
    delay(5000);
    T_junct(0);
    infServo.write(INF_UP);
    stepperHome();
  }
}

void makeTea() {
  stepperHome();        // Home steppers
  moveInfuser(0);       // Raise infuser to load leaves
  dispenseLeaf();       // Portion leaves into infuser
  T_junct(0);           // Close T_junction
  moveInfuser(1);       // Lower infuser to steeping position
  //waterControlPID();    // Water pump & PID
  waterControlOL();     // Open Loop Water
  steepTea();           // Blow out lines and steep tea for steeping time
  T_junct(1);           // Pour steeped tea into cup
  delay(2000);
  T_junct(0);           // Close T-Junction
  moveInfuser(2);       // Bring infuser up to cleaning pos
  washCycle();          // Clean machine for next cup

}

void loop() {
  int command;
  
  //  Check RE
  aState = pcf.digitalRead(RE_CLK); // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState){     
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (pcf.digitalRead(RE_DT) != aState) { 
      RE_pos ++;    // UP
      if (RE_pos % 2 == 0) {
        command = DOWN;
        Serial.print("Position: ");
        Serial.println(RE_pos);
        processMenuCommand(menu, command, UP, DOWN, LEFT, RIGHT, ENTER, BACK, CLEAR, BACKSPACE);
      }
    } else {
      RE_pos --;
      if (RE_pos % 2 == 0) {
        command = UP;
        Serial.print("Position: ");
        Serial.println(RE_pos);
        processMenuCommand(menu, command, UP, DOWN, LEFT, RIGHT, ENTER, BACK, CLEAR, BACKSPACE);
      }
    }
  } 
  aLastState = aState; // Updates the previous state of the outputA with the current state

  if (! pcf.digitalRead(RE_SW)) {
    Serial.println("CLICK!");
    command = ENTER;
    processMenuCommand(menu, command, UP, DOWN, LEFT, RIGHT, ENTER, BACK, CLEAR, BACKSPACE);
    while (! pcf.digitalRead(RE_SW)) {
    }
  delay(10);
  }

}

// LCD COMMANDS
void toggleBacklight(uint16_t isOn) { menu.setBacklight(isOn); }
void backButton() { processMenuCommand(menu, BACK, UP, DOWN, LEFT, RIGHT, ENTER, BACK, CLEAR, BACKSPACE); }
void preTeaMenu() {
  Serial.println("Making Tea!");
  makeTea();
}
void toggleCleaning(uint16_t isOn) {
  if (cleaningFlag) {
    cleaningFlag = 0;
  }
  else {
    cleaningFlag = 1;
  }
}
void leafEject() { S_leaf.rotate(-35*360); }
void leafLoad() { stepperHome(); }
void setSteepingTime(int* value) {
  //steepingTime = value*1000;
  backButton();
}

// DEMOS
void demoRoutine() {
  stepperHome();        // Home steppers
  moveInfuser(0);       // Raise infuser to load leaves
  dispenseLeaf();       // Portion leaves into infuser
  T_junct(0);           // Close T_junction
  //moveInfuser(1);       // Lower infuser to steeping position
  S_inf.rotate(-6*360);
  delay(2000);            // Delay 2s
  //steepTea();           // Blow out lines and steep tea for steeping time
  T_junct(1);           // Pour steeped tea into cup
  delay(1000);
  T_junct(0);           // Close T-Junction
  //moveInfuser(2);       // Bring infuser up to cleaning pos
  S_inf.rotate(6*360);
  //washCycle();          // Clean machine for next cup
}

void infDemo() {
  stepperHome();        // Home steppers
  moveInfuser(1);       // Move to steeping pos
  delay(1000);
  moveInfuser(2);       // Move to cleaning pos
  infServo.write(INF_DUMP);
  delay(2000);
  infServo.write(INF_UP);
  int infHB = pcf.digitalRead(INF_LIMIT); 
  while (infHB == LOW)
  {
    //backwards slowly till it hits the switch and stops
    S_inf.rotate(30);
    infHB = pcf.digitalRead(INF_LIMIT);
  }
}

void leafDemo() {
  stepperHome();        // Home steppers
  dispenseLeaf();
}

void T_Demo() {
  T_junct(0);           // Close T_junction
  delay(2000);
  T_junct(2);           // Waste
  delay(2000);
  T_junct(1);           // Cup
  delay(2000);
  T_junct(0);           // Close
}

void waterDemo() {
  stepperHome();
  T_junct(0);
  delay(2000);
  digitalWrite(SOLENOID, HIGH);
//  analogWrite(PUMP_0, 127);
  delay(2000);
  analogWrite(PUMP_0, 255);
  delay(4000);
  analogWrite(PUMP_0, 0);
  delay(1000);
  digitalWrite(SOLENOID, LOW);
  delay(2000);
  digitalWrite(PUMP_1, HIGH);
  delay(1000);
  digitalWrite(PUMP_1, LOW);
  T_junct(1);
  delay(2000);
  T_junct(0);
  washCycle();
  
}

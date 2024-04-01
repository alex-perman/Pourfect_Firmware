/* Pourfect Tea Test Firmware
 * Alex Perman Copyright 2024
 * 
 */

// Include Applicable Libraries
#include <A4988.h>
#include <Adafruit_PCF8574.h>

Adafruit_PCF8574 pcf;

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

// Globals
int inf_disp;
int leaf_disp;

#define STEPPER_STEPS 200
A4988 S_leaf(STEPPER_STEPS, LEAF_DIR, LEAF_STEP);
A4988 S_inf(STEPPER_STEPS, INF_DIR, INF_STEP);

void stepperHome() {
  int infHB = pcf.digitalRead(INF_LIMIT); 
  int leafHB = pcf.digitalRead(LEAF_LIMIT);
  
  while (infHB == LOW)
  {
    //backwards slowly till it hits the switch and stops
    S_inf.rotate(30);
    infHB = pcf.digitalRead(INF_LIMIT);
  }
  inf_disp = 0;

  while (leafHB == LOW)
  {
    //S_leaf.rotate(-100);
    leafHB = pcf.digitalRead(LEAF_LIMIT);
  }
  leaf_disp = 0;
}

void stepperTest() {
  S_inf.rotate(5*360); // {deg}
  delay(1000);
  S_inf.rotate(-5*360); // {deg}
}

void DC_Test() {
  analogWrite(PUMP_0, 127);   // Half
  delay(1000);
  analogWrite(PUMP_0, 255);   // Full
  delay(1000);
  analogWrite(PUMP_0, 0);     // Off
  delay(1000);
}

void solenoidTest() {
  digitalWrite(SOLENOID, HIGH);
  delay(1000);
  digitalWrite(SOLENOID, LOW);
  delay(1000);
}

void heaterTest() {
  digitalWrite(HEATER, HIGH);
  delay(1000);
  digitalWrite(HEATER, LOW);
  delay(1000);
}

void setup() {
  Serial.begin(115200);

  if (!pcf.begin(0x21, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }
  for (uint8_t p=0; p<8; p++) {
    pcf.pinMode(p, INPUT_PULLUP);
  }
  //S_leaf.begin(300, 1); // {rpm}, {microstepping} 
  // 300 Seems to be mint speed
  S_inf.begin(100, 1); // {rpm}, {microstepping}
  analogWrite(PUMP_0, 0);
  delay(2000);
  S_inf.rotate(-360);
  stepperHome();
}

void loop() {
  //stepperTest();
  //DC_Test();
  //solenoidTest();
  //heaterTest();
  Serial.println(pcf.digitalRead(INF_LIMIT));
  delay(50);
}

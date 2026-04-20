#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/ma730/MagneticSensorMA730.h>
#include <USB.h>
// ----------------------------------------------------------------------------

// Sensor
#define SENSOR_RESOLUTION 12
#define SENSOR_MISO 4
#define SENSOR_MOSI 7
#define SENSOR_CLK 6
#define SENSOR_CS 5
#define SENSOR_ANGLE_REG 0x0

// Motor
#define SUPPLY_VOLTAGE 9
#define NUM_POLE_PAIRS 7
#define PIN_A_H 13
#define PIN_A_L 12
#define PIN_B_H 14
#define PIN_B_L 11
#define PIN_C_H 15
#define PIN_C_L 10
#define PIN_DRV_ENABLE 9
#define PIN_VOLTAGE_SENSE 46
#define SENSE_MULT 7.849f

// User IO
#define PIN_USER_BTN 38
#define PIN_USER_LED 39

// ----------------------------------------------------------------------------

// the sensor class with desired sensor configuration
MagneticSensorMA730 sensor(SENSOR_CS);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(NUM_POLE_PAIRS);
// BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_A_H, PIN_A_L, PIN_B_H, PIN_B_L, PIN_C_H, PIN_C_L, PIN_DRV_ENABLE);

unsigned long beep_until = 0;

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void printAbsolute(char* cmd) { Serial.println(sensor.getAngle(), 3); }
void printRaw(char* cmd) { Serial.println(sensor.getMechanicalAngle(), 3); }
void printStatus(char* cmd) { Serial.println(sensor.getFieldStrength() == 1 ? "TOO LOW" : "OK"); }
void printMotorVoltage(char* cmd) { Serial.println(analogRead(PIN_VOLTAGE_SENSE) * 3.3f * SENSE_MULT / 4095.0f, 3); }
void doLED(char* cmd) { digitalWrite(PIN_USER_LED, atoi(cmd) ? HIGH : LOW); }
void doBeep(char* cmd) { beep_until = millis() + atoi(cmd); }

void setup() {
  USB.setProduct("OWL");
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PIN_USER_LED, OUTPUT);
  pinMode(PIN_USER_BTN, INPUT_PULLUP);

  SimpleFOCDebug::enable();

  SPI.setMISO(SENSOR_MISO);
  SPI.setMOSI(SENSOR_MOSI);
  SPI.setSCK(SENSOR_CLK);
  SPI.setCS(SENSOR_CS);
  SPI.begin();

  // set up sensor
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link driver
  motor.linkDriver(&driver);

  // Motor
  motor.voltage_limit = 4;

  // set control loop type to be used
  motor.controller = MotionControlType::angle;
  motor.velocity_limit = 3.14;

  // control loop settings
  motor.LPF_velocity.Tf = 1.0 / (motor.velocity_limit * 20);

  // initialise motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // align encoder and start FOC
  motor.initFOC();

  command.add('M', onMotor, "motor settings");
  command.add('T', doTarget, "target angle");
  command.add('A', printAbsolute, "print absolute angle");
  command.add('R', printRaw, "print raw angle (between 0 and 2PI)");
  command.add('V', printMotorVoltage, "print motor voltage");
  command.add('S', printStatus, "print sensor status");
  command.add('L', doLED, "user LED");
  command.add('B', doBeep, "buzz the motor");
  
  Serial.println("ready");
}

unsigned long reset_target_at = 0;

void loop() {
  motor.move();

  // Beeping logic
  if (beep_until > millis()) {
    float buzz = 5 * sin(2 * PI * 1000 * (micros() / 1000000.0));
    motor.voltage.d += buzz;
    motor.setPhaseVoltage(motor.voltage.q, motor.voltage.d, motor.electrical_angle);
  }

  // Reset motor logic
  if (!digitalRead(PIN_USER_BTN)) {
    reset_target_at = millis() + 1000;
  }
  if (reset_target_at != 0 && reset_target_at < millis()) {
    reset_target_at = 0;
    motor.move(0);
  }

  motor.loopFOC();
  command.run();
}
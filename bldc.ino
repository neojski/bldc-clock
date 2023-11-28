#include <SimpleFOC.h>
#define PI 3.14159265358979323846

// SDA & SLC to a4 & a5
// Useful commands:
// MVD0 - set velocity derivative to 0
// MVP1 - set velocity p to 1

BLDCMotor motor = BLDCMotor( 7 );
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);

// commander interface
Commander command = Commander(Serial);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  //SimpleFOCDebug::enable(&Serial);
 
  // is this set clock needed?
  //Wire.setClock(400000);
  //Wire.begin();
  sensor.init();

  pinMode(12,OUTPUT); // declares pin 12 as output and sets it to LOW

  motor.linkSensor(&sensor);

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor to the driver
  motor.linkDriver(&driver);

  // set control loop to be used
  motor.controller = MotionControlType::torque;

  // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
 /* motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  */
  //default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.0;

  // angle P controller 
  // default P=20
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  // default 20
  motor.velocity_limit = 20;

  motor.useMonitoring(Serial);
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // skip calibration
  motor.sensor_direction = Direction::CW;

  //command.add('M',doMotor,'motor');
  

  // monitoring port
  //Serial.println("Motor ready.");
  //Serial.println("Set the target angle using serial terminal:");
  _delay(1000);
}

float radToDeg(float rad){ 
  return rad / 2.0 / PI * 360.0;
}

float degToRad(float deg) {
  return deg * 2.0 * PI / 360.0;
}

float targetAngle;
void setAngleRad(float rad) {
  targetAngle = rad;
}
void setAngleDeg(float deg) {
  setAngleRad(degToRad(deg));
}

void updateTorque() {
  float currentAngle = sensor.getAngle();

  float maxError = 0.1;
  float errorAngle = constrain(targetAngle - currentAngle, -maxError, maxError);
  //Serial.println(errorAngle);
  
  motor.move(errorAngle * 8);
}

int getSeconds() {
  // TODO: maybe use fmodf
  return (float)millis() / 1000.0;
}

void setAngleSeconds(float sec) {
  setAngleDeg(sec * 360 / 60);
}

void seconds (int dir) {
  int sec = dir * getSeconds();
  setAngleSeconds(sec);
}

void pendulum() {
  float angle = 40 * cos(millis() / 1000. * PI);
  setAngleDeg(angle);
}

void forwardAndBack() {
  int multiSec = (float) (4 * millis()) / 1000.0;
  int c = multiSec % 4;
  if (c == 0 || c == 2) {
    setAngleSeconds(multiSec / 4);
  } else if (c == 1) {
    setAngleSeconds(multiSec / 4 + 1);
  }
}

void smoothSeconds() {
  float sec = millis() / 1000.;
  setAngleSeconds(sec);
}

int maxProgram = 5;
int switchTime = 3000; // ms

int program = 3;
float lastSwitchTime = millis();
int getProgram() {
  float now = millis();
  if (now - lastSwitchTime > switchTime) {
    // next program if force moved by more than 45 degress
    if (abs(targetAngle - sensor.getAngle()) > degToRad(45)) {
      program = (program + 1) % maxProgram;
      Serial.print("Program");
      Serial.print(program);
      Serial.println();
      lastSwitchTime = now;
    }
  }
  return program;
}

void loop() {
  motor.loopFOC();
  command.run();

  switch (getProgram()) {
    case 0:
      seconds(1);
      break;
    case 1:
      seconds(-1);
      break;
    case 2:
      pendulum();
      break;
    case 3:
      forwardAndBack();
      break;
    case 4:
      smoothSeconds();
      break;
  }

  updateTorque();
  //motor.monitor();
}

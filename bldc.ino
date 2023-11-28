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
  return rad / 2 / PI * 360;
}

float degToRad(float deg) {
  return deg * 2 * PI / 360;
}

float targetAngle;
void setAngle(float targetAngle0) {
  targetAngle = targetAngle0;
}

void updateTorque() {
  float currentAngle = sensor.getAngle();

  float maxError = 0.1;
  float errorAngle = constrain(targetAngle - currentAngle, -maxError, maxError);
  //Serial.println(errorAngle);
  
  motor.move(errorAngle * 8);
}

int getSeconds() {
  return (float)millis() / 1000.0;
}

// seconds
void seconds () {
  int sec = getSeconds();
  setAngle(degToRad(sec * 360 / 60));
}

// tick-tock
void tick() {
  float angle = 40 * cos(millis() / 1000. * PI);
  Serial.println(angle); 
  setAngle(degToRad(angle));
}

int maxProgram = 2;
int switchTime = 5000; // ms

int program = 1;
float lastSwitchTime = millis();
int getProgram() {
  float now = millis();
  if (now - lastSwitchTime > switchTime) {
    program = (program + 1) % maxProgram;
    lastSwitchTime = now;
  }
  return program;
}

float target = 0;
void loop() {
  motor.loopFOC();
  command.run();

  switch (getProgram()) {
    case 0:
      seconds();
      break;
    case 1:
      tick();
      break;
  }

  updateTorque();
  //motor.monitor();
}

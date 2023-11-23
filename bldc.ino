#include <SimpleFOC.h>
#define PI 3.14159265358979323846

// SDA & SLC to a4 & a5

BLDCMotor motor = BLDCMotor( 7 );
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);

// commander interface
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.motion(&motor, cmd); }

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
  motor.controller = MotionControlType::angle;

  // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 0.05;
  
  //default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

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

  command.add('M',doMotor,'motor');
  

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

float target = 0;
void loop() {
  // iterative FOC function
  motor.loopFOC();
  
  //command.run();

  target = degToRad((float)millis() / 1000.0 / 60 * 360);

  Serial.print("angle: ");
  Serial.print(radToDeg(sensor.getAngle()));
  Serial.print("\t");
  
  Serial.print("target: ");
  Serial.print(radToDeg(target));
  Serial.print("\n");

  //motor.monitor();
  motor.move(target);
}

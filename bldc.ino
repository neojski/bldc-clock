#include <SimpleFOC.h>

// SDA & SLC to a4 & a5

// init BLDC motor
BLDCMotor motor = BLDCMotor( 11 );
// init driver
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);

// commander interface
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.motion(&motor, cmd); }

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
 
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
  motor.controller = MotionControlType::velocity; // XXX: openloop
  
  // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;

  //motor.sensor_direction = Direction::CCW;
  
  //default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller 
  // default P=20
  motor.P_angle.P = 2;
  //  maximal velocity of the position control
  // default 20
  motor.velocity_limit = 4;
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  //command.add('T', onTarget, "motion control");


  // monitoring port
  Serial.println("Motor ready.");
  Serial.println("Set the target angle using serial terminal:");
  _delay(1000);
}


int i = 0;
void loop() {
  // iterative FOC function
  motor.loopFOC();

  //float target = (float)millis() / 1000.0 / 60.0 * 3.14;
  float target = 0.;


  //sensor.update();
  if (i % 100 == 0) {
    Serial.print(target);
    Serial.print("\t");

    Serial.print(sensor.getFullRotations());
    Serial.print("\t");
  
    Serial.print(sensor.getAngle() / 3.14 / 2 * 360.);
    Serial.print("\n");


    }
  i++;


  motor.move(1);
  

  // commander interface with the user
  //command.run();
  
  //delayMicroseconds(1000); 
}

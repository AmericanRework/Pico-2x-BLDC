
/*
 future user: to get serial access: usermod -a -G tty myuser

this should be interesting. See previous RP2040simplefoc docs. 
First test of simpledcmotor library.

search arduino ide library manager for simpleDCMotor
Works. woo! now I can at least have a chance at repairing motor drivers when they break! (overheat? IDK 'cause their drivers are all closed source.)

 
 */
 
//#include <Arduino.h>
#include <SimpleFOC.h>
// software interrupt library - no need rp2040 gots hardware pwm in theory.
//#include <PciManager.h>
//#include <PciListenerImp.h>
#include <SimpleFOCDrivers.h>
#include "SimpleDCMotor.h"
#include "pins.h"


// DC motor & driver instance
DCMotor motor1 = DCMotor();
DCMotor motor2 = DCMotor();
DCMotor motor3 = DCMotor();

//(reuse of bldc drivers for DC motors. pin order: AH, AL, BH, BL) 
//got this working with one motor and manual pwm on one channel and sink for the V channel. so hardware works. Lets try it with simplefoc.
DCDriver4PWM driver1 = DCDriver4PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL);
DCDriver4PWM driver2 = DCDriver4PWM(PIN_WH, PIN_WL, PIN_WHM2, PIN_WLM2);
DCDriver4PWM driver3 = DCDriver4PWM(PIN_UHM2, PIN_ULM2, PIN_VHM2, PIN_VLM2);


//sensor declare here:
// quadrature sensor = new sensor()



// commander interface
//there are zero - ZERO! examples of two motors controlled by the commander. wut!? Maybe I bad at searching. this should be interesting.
// I got AI to spit out an example it claims will send the same command to both motors. ugh. not what I need.
Commander command = Commander(Serial);
void onMotor1(char* cmd){ command.motor(&motor1, cmd); }
void onMotor2(char* cmd){ command.motor(&motor2, cmd); }
void onMotor3(char* cmd){ command.motor(&motor3, cmd); }

void setup() {

  // to use serial control we have to initialize the serial port
  Serial.begin(115200); // init serial communication
  // wait for serial connection - doesn't work with all hardware setups
  // depending on your application, you may not want to wait
  while (!Serial) {};   // wait for serial connection
  // enable debug output to the serial port
  SimpleFOCDebug::enable();
  
  // basic driver setup - set power supply voltage
  driver1.voltage_power_supply = 18.0f;
  driver2.voltage_power_supply = 18.0f;
  driver3.voltage_power_supply = 18.0f;
  // if you want, you can limit the voltage used by the driver.
  // This value has to be same as or lower than the power supply voltage.
  //THIS WORKS GREAT!
  driver1.voltage_limit = 18.0f;
  driver2.voltage_limit = 18.0f;
  driver3.voltage_limit = 18.0f;
  // Optionally set the PWM frequency.
 // driver.pwm_frequency = 20000;
  // init driver
  driver1.init();
  driver2.init();
  driver3.init();
  // init sensor
  //sensor1.init();

  //why pin 10? led?
 // pinMode(10, OUTPUT);

  // link driver to motor
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);
  motor3.linkDriver(&driver3);
  // link sensor to motor
 // motor1.linkSensor(&sensor1);



  // set control loop type to be used
 //motor.controller = MotionControlType::velocity_openloop;

//DO i EVEN HAVE TO SET THIS? IT CLUTTERS MY CODE.
   motor1.voltage_limit = 18.0f;
   motor2.voltage_limit = 18.0f;
   motor3.voltage_limit = 18.0f;
//  motor.velocity_limit = 500.0f;
  // control type - for this example we use position mode.
  //was  motor.controller = MotionControlType::angle;
   //MAYBE: motor1.controller = MotionControlType::velocity_openloop;

    motor1.controller = MotionControlType::torque; //YES!
    motor2.controller = MotionControlType::torque;
    motor3.controller = MotionControlType::torque;

  // might want this back especially for position control lol: check the dc position example: motor.torque_controller = TorqueControlType::voltage;
  
  // init motor
  motor1.init();
  motor2.init();
  motor3.init();




  // contoller configuration based on the controll type
  // do we even have to set these for open loop I think no.
  /*
  motor.PID_velocity.P = 0.0; // was 0.2f
  motor.PID_velocity.I = 0; //WAS 20
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 20.0;
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;
  // angle loop controller
  motor.P_angle.P = 20;
  // angle loop velocity limit
  motor.velocity_limit = 5;
  */

//mtrs 2 and 3 will go here if we get 1 working.
  



//serial control motor 1:
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);
  motor3.useMonitoring(Serial);
//  motor2.useMonitoring(Serial);

  // define the motor id (WHAT THIS IS NOT DEFINING A MOTOR ID IT'S LINKING THE SERIAL PORT TO THE MOTOR THRU THE ONMOTOR METHOD YA?)
  command.add('A', onMotor1, "motor1");
  command.add('B', onMotor2, "motor2");
  command.add('C', onMotor3, "motor3");

  // set the inital target value
  //motor.target = 0.5;
  motor1.enable();
  motor2.enable();
  motor3.enable();


  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("wooooooooooooooooooooo 3 dc motors."));

  _delay(1000);
}


void loop() {

  motor1.move();
  motor2.move();
  motor3.move();


  command.run();

 //print motor state once per loop lol this would be sloooow.
  //motor1.monitor();

}
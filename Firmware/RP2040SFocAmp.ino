
/*
** Most recent version as of March 2024. Servo amp for Automower!
This is mostly example code from SimpleFOC. Follow their license. 

DO NOT PLUG PICO INTO USB WHILE IT'S INSTALLED /PLUGGED INTO BEAGLEBONE. WILL SEND POSSIBLY 5V DOWN THE BEAGLEBONE 3.3V POWER LINE.

Install: 
use Arduino ide, add packages / boards:
known working:
Simplefoc 2.3.2, SimpleDCmotor 1.0.0 simplefocDrivers 1.0.6, board: Raspi pico rp2040 by Philhower 3.7.2
Testing:
SimpleDCmotor 1.0.2 simplefocDrivers 1.0.7

HAD TO delete rp2040.h under current sense in the standard simplefoc library to get this pwm on rp2040 to work... 2024: No longer have to do this!

future self: to get serial access in linux: usermod -a -G tty myusername
and possibly: sudo gpasswd --add ${USER} dialout


Downloading to a pico:
plug in, check show all devices, look for uf2 devices - may have to select the board type - a rp2040 pico not the mbed version.

Running it:
  to 'command' use ? to list commands. not that useful. but then you can say "AE0" to disable motor A, AE1 ENABLES. 
  A10: 10 Is like set torqe/velocity goal to 10. YEP. STARTING AT 1 OR EVEN LESS IS A GOOD IDEA.

//commander is set to have some extra methods: I, C, H, and V all do extra stuff we add here. Check out commander.h for more stuff you can do!
// I is (re)-init. I still can't get init to run a full alignment cycle but can force various override angles? Probably could fix but just never end up using it.
//C is measure current? I think? 
//H is hall state. Helpful when you are adding halls to a non-hall motor.
//V is current velocity. 
 */

 //motors are currently set to use velocity control - and hopefully negative values go in reverse? we'll see.
 //CONFIRMED! NEGATIVE VELOCITY IS CHANGE DIRECTION. WOO0.

//not sure what happened one time I tried sending A10 B10 and b motor just stopped working. reboot fixed lol...
//YO THIS IS IMPORTANT: Can set to use heartbeat or usb/ hardware serial ports. See below.


//#include <Arduino.h>
#include <SimpleFOC.h>
// software interrupt library
//#include <PciManager.h>
//#include <PciListenerImp.h>
#include <SimpleFOCDrivers.h>
#include "pins.h"


//motor pole pairs = # of permanent magnets in rotor.
BLDCMotor motor = BLDCMotor(11);

BLDCMotor motor2 = BLDCMotor(11);
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);

BLDCDriver6PWM driver2 = BLDCDriver6PWM(PIN_UHM2, PIN_ULM2, PIN_VHM2, PIN_VLM2, PIN_WHM2, PIN_WLM2);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// hall sensor instance
HallSensor sensor = HallSensor(PIN_HALL_U, PIN_HALL_V, PIN_HALL_W, 11);

int hallu, hallv, hallw = 0; //for manual hall testing.

HallSensor sensor2 = HallSensor(PIN_HALL_UM2, PIN_HALL_VM2, PIN_HALL_WM2, 11);
// Interrupt routine intialisation
// channel A, B and C callbacks


//////////////////////////////////////////////////////////////////////////////////////////
// YO THESE ARE IMPORTANT. SET THEM APPROPRIATELY SO THAT WE ONLY HAVE TO MAINTAIN ONE SERVOAMP CODEBASE.
HardwareSerial* whichSerial = &Serial1; //&Serial for usb or &Serial1 for hardware pins. See pins.h for which pins to use. &Serial2 Should also work but is untested.


//heartbeat:
bool useHeartbeat = true; //true to turn everything off if we dont' hear from coms. 
unsigned long last_heartbeat_millis = 0;
int heartbeat_shutdown_millis = 5000; // 5000 ms = 5 seconds no instruction shut down motors. Yes that's kind of a long time. meh. at least it's there.


void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

void doA2(){sensor2.handleA();}
void doB2(){sensor2.handleB();}
void doC2(){sensor2.handleC();}

//first # is milivolts per ... amp? 
//InlineCurrentSense the_current_sense = InlineCurrentSense(500.0, PIN_CURRENT_SENSE, PIN_CURRENT_SENSE);
//DIY that shit:
float motor1_Current = 0;
float motor1_AvgCurrent = 0;
int debugCounter = 0;

float motor2_Current = 0;
float motor2_AvgCurrent = 0;

// commander interface
//there are zero - ZERO! examples of two motors controlled by the commander. wut!? Maybe I bad at searching. this should be interesting.
// I got AI to spit out an example it claims will send the same command to both motors. ugh. not what I need.
Commander command = Commander(*whichSerial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void onMotor2(char* cmd){ command.motor(&motor2, cmd); }

//try to restart a motor? doesn't quite work right. 
void reInit1(char* cmd){ 
  /*
  motor.init();
  motor.target = 0; //wish this was to CMD then you could init @ different power levels woo.
  motor.enable();
  //motor.initFOC(); //this doesn't force a re-align for some reason...
  motor.initFOC(5.236, Direction::CCW);
  */
   }

//OK! after some testing, sampling every loop and adding to a moving average: 
//0 reads 4-10, 0.5 amps reads 165 to 175, 1.5 amps reads 370 to 390, ~ 10A 912-933, ~4 AMPS 616 TO 703.
void readCurrent(char* cmd){ 
  //motor1_Current = the_current_sense.getDCCurrent();
 // motor1_Current = analogRead(PIN_CURRENT_SENSE);
//  *whichSerial.println(motor1_Current);

    whichSerial->println(motor1_AvgCurrent);
    whichSerial->println(motor2_AvgCurrent);
 }

 void printVelo(char* cmd){ 
  whichSerial->println(motor.shaft_velocity);
  whichSerial->println(motor2.shaft_velocity);
 }

void readHalls(char* cmd){ 
  hallu = digitalRead(PIN_HALL_U);
  hallv = digitalRead(PIN_HALL_V);
  hallw = digitalRead(PIN_HALL_W);
  whichSerial->println("HALLS:");
  whichSerial->println(hallu);
  whichSerial->println(hallv);
  whichSerial->println(hallw);
 }



void setup() {

  // initialize encoder sensor hardware
  
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  
   
  sensor2.init();
  sensor2.enableInterrupts(doA2, doB2, doC2);
  


  //COMMENT THIS OUT TOO FOR OPEN LOOP!
  motor.linkSensor(&sensor);

  motor2.linkSensor(&sensor2);


  //CURRENT SENSOR - sfoc version. Doesn't 'just work.' doing my own thing but I'll leave this here maybe help someone. 
  //inlineCurrentSense(resistorValue,gain,AnalogPin#'s) or (mv/amp, analogpins)
  //the_current_sense = InlineCurrentSense(200.0, PIN_CURRENT_SENSE, PIN_CURRENT_SENSE);
  
  //the_current_sense.init();
  // do this not sure if it'll break motor alignment ability:  motor.linkCurrentSense(&current_sense);

  // link current sense and driver
  //current_sense.linkDriver(&driver);
  // link motor and current sense
  //motor.linkCurrentSense(&current_sense);


  //should be able to do this to get DC current:
  //  motor1_Current = current_sense.getDCCurrent();




  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  driver.dead_zone = 0.1;
  // link driver
  motor.linkDriver(&driver);

  driver2.voltage_power_supply = 12;
  driver2.init();
  driver2.dead_zone = 0.1;
  // link driver
  motor2.linkDriver(&driver2);



  // choose FOC modulation OPEN LOOP TO TEST A MOTOR? SEE THE BLDC ATTEMPT #5 FOR WORKING OPEN LOOP
 // motor.foc_modulation = FOCModulationType::SinePWM;
  // set control loop type to be used
 //motor.controller = MotionControlType::velocity_openloop;

 
 motor.phase_resistance = 1.8;
 motor.current_limit = 3.0;
 motor.KV_rating = 100; //RPM'S PER VOLT! THE MISSING LINK! WHO KNOWS WHAT IT DEFAULTED TO BUT NOW IT CAN GO FASTER COOL. 
 //if motor kv and phase resistance are set, simplefoc will try to follow rough guess at current_limit.


 motor2.phase_resistance = 1.8;
 motor2.current_limit = 3.0;
 motor2.KV_rating = 100;


 //Open loop? see above. 
 motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::velocity;
//Don't forget to uncomment above sensor link

 motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.controller = MotionControlType::velocity; 


  // contoller configuration based on the controll type
  motor.PID_velocity.P = 0.01f; // was 0.05f
  motor.PID_velocity.I = 0.2; //WAS 20
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 20.0;
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;
  // angle loop controller
  motor.P_angle.P = 0;
  // angle loop velocity limit. WAS 15 IS THIS # IN HZ? YES IT IS! WHY DO CS PEOPLE NEVER DO UNITS?
  motor.velocity_limit = 25;

//////////////////////////////M2!! tempted to comment this all out but let's see if straight copy works first. 
  motor2.PID_velocity.P = 0.01f; // was 0.2f
  motor2.PID_velocity.I = 0.2; //WAS 20
  motor2.PID_velocity.D = 0;
  motor2.voltage_limit = 20.0;
  motor2.LPF_velocity.Tf = 0.01f;
  motor2.P_angle.P = 0;
  motor2.velocity_limit = 25;

  

  // start monitoring / commander serial port.
  if(whichSerial == &Serial1){
    Serial1.setRX(PIN_SERIAL1_RX);
    Serial1.setTX(PIN_SERIAL1_TX);
    Serial1.begin(115200);
  }
  else{
    Serial.begin(115200);
  }
  // comment out if not needed
  motor.useMonitoring(*whichSerial);
  motor2.useMonitoring(*whichSerial);

  // initialise motor
  motor.init();

  motor2.init();
 


  // define the motor id (WHAT THIS IS NOT DEFINING A MOTOR ID IT'S LINKING THE SERIAL PORT TO THE MOTOR THRU THE ONMOTOR METHOD YA?)
  command.add('A', onMotor, "motor");
  command.add('B', onMotor2, "motor2");
  command.add('I', reInit1, "restart motor1");
  command.add('C', readCurrent, "current of motor1");
  command.add('H', readHalls, "digitalRead hall u,v,w");
  command.add('V', printVelo, "pls work - want velocity");


  // set the inital target value
  motor.target = 0;
  motor.enable();

  motor2.target = 0;
  motor2.enable();

  // align encoder and start FOC - this will move the motors to find the electrical angle.
  //motor.initFOC();
  //motor2.initFOC();

  //once you get the alignment, use this:
  motor.sensor_direction=Direction::CCW; // or Direction::CW
  motor.zero_electric_angle=5.24;     // use the real value!


  motor2.sensor_direction=Direction::CCW; // or Direction::CW
  motor2.zero_electric_angle=5.24;     // use the real value!

  /* why they screw up perfect system!?
   motor.initFOC(5.24, Direction::CCW); 
   motor2.initFOC(5.24, Direction::CCW); 
*/
motor.initFOC();
motor2.initFOC();

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  whichSerial->println(F("wooooooooooooooooooooo 2 motors."));

  _delay(1000);
}


void loop() {
  

  // iterative setting FOC phase voltage
  motor.loopFOC();

 // motor2.loopFOC();

  motor.move();

 // motor2.move();
/* This stuff is in the other core of the rp2040 'cause there was a bug with pwm and hardware serial. Or put it back here if you want.

  //heartbeat:
  if(whichSerial->available()){
    last_heartbeat_millis = millis();
  }
  else if (useHeartbeat && (last_heartbeat_millis + heartbeat_shutdown_millis <  millis())){
    //shutdown!
    motor.target = 0;
    motor2.target = 0;
  }
  // user communication
  command.run();

*/
  //custom adds:
  motor1_Current = analogRead(PIN_CURRENT_SENSE);
  motor1_AvgCurrent = (motor1_AvgCurrent + (motor1_Current * 0.001))/1.001;
  motor2_Current = analogRead(PIN_CURRENT_SENSEM2);
  motor2_AvgCurrent = (motor2_AvgCurrent + (motor2_Current * 0.001))/1.001;

  if(debugCounter > 100) {
//  motor1_Current = the_current_sense.getDCCurrent();
  //motor1_Current = analogRead(PIN_CURRENT_SENSE);
  //motor1_AvgCurrent = (motor1_AvgCurrent + (motor1_Current * 0.001))/1.001;
  //zero motor if current is high
  if((motor1_AvgCurrent > 800) or (motor2_AvgCurrent > 800))
  {
      motor.target = 0;
      motor2.target = 0;
      motor.disable();
      motor2.disable();
      whichSerial->print("OVERCURRENT! ");
      //whichSerial->println(motor1_Current);
  }
  //whichSerial->println(motor1_Current);
  debugCounter = 0;
  }
  debugCounter++;

}

void loop1() {

  motor2.loopFOC();

  motor2.move();
  //heartbeat:
  if(whichSerial->available()){
    last_heartbeat_millis = millis();
  }
  else if (useHeartbeat && (last_heartbeat_millis + heartbeat_shutdown_millis <  millis())){
    //shutdown!
    motor.target = 0;
    motor2.target = 0;
  }
  // user communication
  command.run();

}

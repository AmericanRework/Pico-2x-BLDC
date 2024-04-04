#pragma once

#define PACKET_SERIAL pioSerial

//#define STATUS_UPDATE_MILLIS 20 //dropping from 20 to 2 may have made significant improvement here... double check if this increases commutation and instead of milis can we go micros()?
#define STATUS_UPDATE_MILLIS 20
   
#define WATCHDOG_TIMEOUT_MILLIS 500

// Time to keep motor controller in fault once a fault occurs
#define MIN_FAULT_TIME_MILLIS 2000

// Hall Table

// Hall table drive motor
/*
_hall_table_0:=255
_hall_table_1:=2
_hall_table_2:=6
_hall_table_3:=1
_hall_table_4:=4
_hall_table_5:=3
_hall_table_6:=5
_hall_table_7:=255
_motor_current_limit:=0.5
_acceleration:=0.1
_has_motor_temp:=false
_min_motor_temp:=0
_max_motor_temp:=0
_min_pcb_temp:=0
_max_pcb_temp:=30
const uint8_t HALL_TABLE[] = {
    255,
    // 0b001
    2,
    // 0b010
    6,
    // 0b011
    1,
    // 0b100
    4,
    // 0b101
    3,
    // 0b110
    5,
    255
};


_hall_table_0:=255
_hall_table_1:=5
_hall_table_2:=3
_hall_table_3:=4
_hall_table_4:=1
_hall_table_5:=6
_hall_table_6:=2
_hall_table_7:=255


*/

// Motor NTC beta value
#define PCB_NTC_BETA 3380.0
#define MOTOR_NTC_BETA 3380.0


// Hardware limits before going into fault
#define HW_LIMIT_PCB_TEMP 80.0
#define HW_LIMIT_MOTOR_TEMP 80.0


// Max amps before shutting down
#define HW_LIMIT_CURRENT 10

// P and I values for the current control
#define CURRENT_P 50.0f
#define CURRENT_I 150.1f


// Hardware limit for undervoltage and overvoltage.
#define HW_LIMIT_VLOW -1.0
#define HW_LIMIT_VHIGH 40.0


// Values for the voltage sensing resistor divider
#define VIN_R1 22000.0f
#define VIN_R2 1500.0f

// Value for the shunt resistor
#define R_SHUNT 0.003f

// Gain for the current sense amplifier
#define CURRENT_SENSE_GAIN 200.0f

// Max duty cycle to allow. This can't be 1, because the gate driver doesn't allow that. Gate drivers use the low time of PWM to run a charge pump. You can get an independant power supply or just accept 99% duty cycle.
#define MAX_DUTY_CYCLE 0.99f


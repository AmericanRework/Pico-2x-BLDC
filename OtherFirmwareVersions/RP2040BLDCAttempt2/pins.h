#pragma once

#define PIN_UH 12
#define PIN_VH 13
#define PIN_WH 15
#define PIN_UL 6
#define PIN_VL 7
#define PIN_WL 10
#define PIN_CURRENT_SENSE 28
#define PIN_TEMP_MOTOR 27
#define PIN_TEMP_PCB 26
#define PIN_VIN 27

//was 18/17/16 or 16/17/18 for the automower motor that kinda worked. 
//red skateboard wheel with our pinout did not work 18/17/16 also doesn't work with 16/17/18.
//don't work: hall table was 1 ,3, 2, 5, 6,4,
//16/18/17
//18/16/17
//18/17/16
//16/17/18
//17/16/18
//17/18/16

//these aren't fully done yet, but 18/17/16 is the standard we used in the simplefoc version.
#define PIN_HALL_U 18
#define PIN_HALL_V 17
#define PIN_HALL_W 16

//UH. Pico board builtin led pin#:
#define PIN_LED_RED 25
#define PIN_LED_GREEN 25
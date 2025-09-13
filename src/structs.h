#pragma once
#include <Arduino.h>

#define MSB 0
#define LSB 1

typedef struct {
    uint32_t can_id;
    String name;
    int16_t startbit;
    uint8_t length;
    bool sign;
    bool isfloat;
    float add;
    float mult;
    uint32_t div;
    int byteorder;
    float value;
    uint32_t last_pub;
} CAN_Signal;


const int num_signals = 6;
CAN_Signal can_signals[num_signals] = {
{0x000, "null", 0,0,0,0,0,0,0,MSB,0,0},
{0x42E, "hv_volt", 24,15,0,0,0,1,64,MSB,0,0},
{0x42E, "range", 0,15,0,0,0,1,270,MSB,0,0},
{0x654, "soc", 24,8,0,0,0,1,1,MSB,0,0},
{0x5D7, "odo", 16,32,0,0,0,0.00062137119,1,MSB,0,0},
{0x62D, "chp", 8,16,0,0,0,1,1,MSB,0,0},
};
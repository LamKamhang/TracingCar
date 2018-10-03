#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// gnd, vcc, SOFT_TX_PIN, SOFT_RX_PIN
#define _SOFT_RX_PIN_   13
#define _SOFT_TX_PIN_   12

const unsigned char SetSeiDecMod[] = {0x07,0xC6,0x04,0x00,0xFF,0x8A,0x08,0xFD,0x9E};  //扫码模块初始化协议
const unsigned char StaDec[] = {0x04,0xE4,0x04,0x00,0xFF,0x14};   //扫码模块扫码协议

const char target[] = {'1','2', '3','4', '5','6','7','8', '9', '0'};
const char num_target = sizeof(target) / sizeof(target[0]);

char check_target(char scan_char);

class ScanModule
{
public:
    ScanModule();
    void init();
    char run();

private:
    SoftwareSerial mySerial;
    boolean flag_SetSeiDecMod;
    boolean flag_StaDec;
};

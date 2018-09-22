#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define _SOFT_RX_PIN_   13
#define _SOFT_TX_PIN_   12

const char SetSeiDecMod[] = {0x07,0xC6,0x04,0x00,0xFF,0x8A,0x08,0xFD,0x9E};  //扫码模块初始化协议
const char StaDec[] = {0x04,0xE4,0x04,0x00,0xFF,0x14};   //扫码模块扫码协议

class ScanModule
{
public:
    ScanModule();
    void init();
    char run();

private:
    void _init();
    SoftwareSerial mySerial;
};

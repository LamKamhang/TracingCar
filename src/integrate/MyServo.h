#ifndef MYSERVO_H
#define MYSERVO_H

#include <Arduino.h>

//发送部分的指令
#define FRAME_HEADER            0xFF   //帧头
#define CMD_SERVO_SPEED         0x01   //设置舵机速度
#define CMD_SERVO_PLACE         0x02   //设置舵机位置
#define CMD_ACTION_GROUP_RUN    0x09   //运行动作组
#define CMD_STOP_REFRESH        0x0b   //急停、恢复指令

class MyServo{
  public:
    MyServo(HardwareSerial &A);
	void moveServo(uint8_t servoID, uint16_t Position, uint16_t Speed);
    void runActionGroup(uint8_t NumOfAction);
	void stopServo();
	void refreshServo();

  public:
    bool isRunning; //正在运行
    HardwareSerial *SerialX;
};
#endif
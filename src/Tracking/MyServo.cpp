#include "MyServo.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位

MyServo::MyServo(HardwareSerial &A)
{
	//初始化运行中标识为真
	isRunning = true;
	SerialX = &A;
	SerialX->begin(9600);
}

/*********************************************************************************
 * Function:  moveServo
 * Description： 控制单个舵机转动
 * Parameters:   sevoID:舵机ID，Position:目标位置,Speed:转动速度
                    舵机ID取值:0<=舵机ID<=15,Speed取值: Speed > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void MyServo::moveServo(uint8_t servoID, uint16_t Position, uint16_t Speed)
{
	uint8_t buf[15];
	if (servoID > 15 || !(Speed > 0)) { 
		return;
	}
	Position = Position*100/9+500;	
	buf[0] = FRAME_HEADER;                  
	buf[1] = CMD_SERVO_SPEED;
	buf[2] = servoID;                        
	buf[3] = GET_LOW_BYTE(Speed);              
	buf[4] = GET_HIGH_BYTE(Speed);            
	buf[5] = FRAME_HEADER;             
	buf[6] = CMD_SERVO_PLACE;          
	buf[7] = servoID;                    
	buf[8] = GET_LOW_BYTE(Position);        
	buf[9] = GET_HIGH_BYTE(Position);       

	SerialX->write(buf, 10);
}

/*********************************************************************************
 * Function:  runActionGroup
 * Description： 运行指定动作组
 * Parameters:   NumOfAction:动作组序号
 * Return:       无返回
 **********************************************************************************/
void MyServo::runActionGroup(uint8_t NumOfAction)
{
	uint8_t buf[10];
	buf[0] = FRAME_HEADER;   //填充帧头
	buf[1] = CMD_ACTION_GROUP_RUN;
	buf[2] = 0x00;      
	buf[3] = GET_LOW_BYTE(NumOfAction); //取得动作组号的低八位
	buf[4] = GET_HIGH_BYTE(NumOfAction); //取得动作组号的高八位
	SerialX->write(buf, 5);      //发送数据帧
}

/*********************************************************************************
 * Function:  stopServo
 * Description： 急停
 * Parameters:   无
 * Return:       无返回
 **********************************************************************************/
 void MyServo::stopServo(void)
{
	if (!isRunning) { 
		return;
	}
	isRunning = !isRunning;
	uint8_t buf[10];
	buf[0] = FRAME_HEADER;   //填充帧头
	buf[1] = CMD_STOP_REFRESH;
	buf[2] = 0x00;      
	buf[3] = 0x01; 
	buf[4] = 0x00;
	SerialX->write(buf, 5);      //发送数据帧
}
 
 /*********************************************************************************
 * Function:  refreshServo
 * Description： 恢复
 * Parameters:   无
 * Return:       无返回
 **********************************************************************************/
 void MyServo::refreshServo(void)
{
	if (isRunning) { 
		return;
	}
	isRunning = !isRunning;
	uint8_t buf[10];
	buf[0] = FRAME_HEADER;   //填充帧头
	buf[1] = CMD_STOP_REFRESH;
	buf[2] = 0x00;      
	buf[3] = 0x00; 
	buf[4] = 0x00;
	SerialX->write(buf, 5);      //发送数据帧
}

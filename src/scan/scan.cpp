#include "scan.h"

ScanModule::ScanModule()
    : mySerial(_SOFT_RX_PIN_, _SOFT_TX_PIN_)
{
}

void ScanModule::init()
{
    mySerial.begin(9600);
}

char ScanModule::run()
{
    char get_char = ' ';
    String SciString = "";         // 主控板接收扫码模块发送回来的字符串
    int Position = 0;            //字符在字符串里的位置

    while (true)
    {
        _init(); // initialize.
        SciString  = ""; //接受字符串清空
        
        while(mySerial.available())    //所扫的字符串返回
        {
            char inChar = (char)mySerial.read();
            delay(5);
            SciString += inChar;
        }
        Position = SciString.indexOf('Z');  //找到字符Z的位置 

        if(Position != -1)   //字符'Z'找到
        {
            SciString = SciString.substring(Position,SciString.length());  //截取有效字符串 e.g    sdaaf,ZJGXDS01 --------->  ZJGXDS01
                                                                            //                        返回的字符串        截取到有效的字符串
                                                                            
            get_char = SciString.charAt(7);  // 返回第7位字符  e.g   ZJGXDS01  ---->    1
            break;
        }
    }

    return get_char;
}

void ScanModule::_init()
{
    //模块初始化功能
    for(int i = 0; i < sizeof(SetSeiDecMod); i++)
    {
        mySerial.write(SetSeiDecMod[i]);
        delay(1);
    }

    //模块扫码功能
    for(int i = 0; i < sizeof(StaDec); i++)
    {
        mySerial.write(StaDec[i]);
        delay(1);
    }
    delay(10); 
}

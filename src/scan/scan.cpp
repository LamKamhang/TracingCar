#include "scan.h"

ScanModule::ScanModule()
    : mySerial(_SOFT_RX_PIN_, _SOFT_TX_PIN_)
    , flag_SetSeiDecMod(true)
    , flag_StaDec(false)
{
}

void ScanModule::init()
{
    mySerial.begin(9600);
}

char ScanModule::run()
{
    delay(300);
    //Serial.println("scaning...");
   char get_char = ' ';
   String SciString = "";
   int Position = 0;
   
   //Serial.println(flag_SetSeiDecMod);
   if(flag_SetSeiDecMod == true)       //模块初始化功能
   {
      for(int i = 0;i < sizeof(SetSeiDecMod);i++)
      {
        mySerial.write(SetSeiDecMod[i]);
        delay(1);
      }
      flag_SetSeiDecMod = false;
      flag_StaDec = true;
   }

  //Serial.println(flag_StaDec);
   if(flag_StaDec == true)     //模块扫码功能
   {
      for(int i = 0;i < sizeof(StaDec);i++)
      {
         mySerial.write(StaDec[i]);
         delay(1);
       }
    }
   delay(10);    
   while(mySerial.available())    //所扫的字符串返回
   {
        char inChar = (char)mySerial.read();
        delay(5);
        SciString += inChar;
   }
   
   Position = SciString.indexOf('Z');  //找到字符Z的位置  

  //Serial.println(Position);
   if(Position != -1)   //字符'Z'找到
   {
      SciString = SciString.substring(Position,SciString.length());  //截取有效字符串 e.g    sdaaf,ZJGXDS01 --------->  ZJGXDS01
                                                                     //                        返回的字符串        截取到有效的字符串
      get_char = SciString.charAt(7);  // 返回第7位字符  e.g   ZJGXDS01  ---->    1
   }
   else   //字符'Z'未能找到   
   {
      flag_SetSeiDecMod = true;  //初始化标志位  方便第二次扫码
      flag_StaDec   = false;
    } 
    return get_char;
}


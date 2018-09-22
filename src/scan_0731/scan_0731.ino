#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>


SoftwareSerial mySerial(13,12); // TX,     RX  PCINT

String SciString = "";         // 主控板接收扫码模块发送回来的字符串

char SetSeiDecMod[] = {0x07,0xC6,0x04,0x00,0xFF,0x8A,0x08,0xFD,0x9E};  //扫码模块初始化协议
char StaDec[] = {0x04,0xE4,0x04,0x00,0xFF,0x14};   //扫码模块扫码协议
    
boolean flag_SetSeiDecMod = true;    //扫码模块初始化标志位
boolean flag_StaDec = false;        //扫码模块扫码标志位

int i,j;
int Position = 0;            //字符在字符串里的位置
char scanflag = ' ';         //字符串最后一位字符  e.g  ZJGXDS01   返回 1

void setup()
{
   Serial.begin(9600);
   mySerial.begin(9600);
}

char scan()
{
  //Serial.println("scaning...");
   char get_char = ' ';
   
   //Serial.println(flag_SetSeiDecMod);
   if(flag_SetSeiDecMod == true)       //模块初始化功能
   {
      for(i = 0;i < sizeof(SetSeiDecMod);i++)
      {
        mySerial.write(SetSeiDecMod[i]);
        delay(1);
      }
      flag_SetSeiDecMod = 0;
      flag_StaDec = 1;
   }

  //Serial.println(flag_StaDec);
   if(flag_StaDec == 1)     //模块扫码功能
   {
      for(i = 0;i < sizeof(StaDec);i++)
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

      SciString  = ""; //接受字符串清空 

      return get_char; //返回有效字符串的最后一位字符
   }
   else   //字符'Z'未能找到   
   {
      flag_SetSeiDecMod = 1;  //初始化标志位  方便第二次扫码
      flag_StaDec   = 0;
      
      SciString  = ""; //接受字符串清空 
      
      get_char = ' ';  //返回空字符
      return get_char;
    } 
}


void loop()
{    
       if(scanflag != ' ') //有效字符串末尾字符不是空字符  则打印该字符
       {
          Serial.println(scanflag);
          scanflag = ' ';
       }
       else   //每隔0.3秒 扫码一次 直到找到有效字符串
      {
        Serial.println("---");
         scanflag = scan();
         delay(300);
       }
       
}



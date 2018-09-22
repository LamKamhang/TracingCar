#include "scan.h"

ScanModule scanM;

void setup()
{
   Serial.begin(9600);
   scanM.init();
}

void loop()
{    
  char res = scanM.run(); //只有扫到才会返回 
  Serial.println(res);
}

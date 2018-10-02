#include "scan.h"

ScanModule scanM;

void setup()
{
  Serial.begin(9600);
  scanM.init();
}

char scan()
{
  char res = 0;
  char flag;
  int time = millis();
  while (millis() - time < 1000)
  {
    flag = scanM.run();
    if (flag >= '0' && flag <= '9')
    {
      res = check_target(flag);
      break;
    }
  }
  return res;
}

void loop()
{
  Serial.println(scan()+48);
}

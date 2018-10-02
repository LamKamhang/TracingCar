#include "scan.h"

ScanModule scanM;

void setup()
{
  Serial.begin(9600);
  scanM.init();
}

void loop()
{
  char scanflag = scanM.run();
  if (scanflag == ' ')
    Serial.println("---");
  else
    Serial.println(scanflag);
}

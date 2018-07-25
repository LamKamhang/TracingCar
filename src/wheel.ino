void setup(){
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  digitalWrite(10,LOW);
  analogWrite(11,255);//前右轮正转
  digitalWrite(5,LOW);
  analogWrite(4,255);//前左轮正转
  digitalWrite(6,LOW);
  analogWrite(7,255);//后右轮正转
  digitalWrite(9,LOW);
  analogWrite(8,255);//后左轮正转
  delay(1000);
  //制动程序
  digitalWrite(10,0);
  analogWrite(11,0);
  digitalWrite(5,0);
  analogWrite(4,0);
  digitalWrite(6,0);
  analogWrite(7,0);
  digitalWrite(9,0);
  analogWrite(8,0);
  delay(3000);
}
void loop(){

}


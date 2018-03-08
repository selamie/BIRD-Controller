void setup() {
  // put your setup code here, to run once:

 pinMode(9, OUTPUT);
 pinMode(2, OUTPUT);
 pinMode(4, OUTPUT);
   
}

void loop() {

  digitalWrite(2, 1);
  digitalWrite(4, 0);
  analogWrite(9, 255);
  delay(500);
  /*
  digitalWrite(2,0);
  digitalWrite(4,0);//gives a (more) hard stop
  delay(500);
  digitalWrite(2,0);
  digitalWrite(4,1);
  analogWrite(9, 20);
  delay(500);
  analogWrite(9,0);//gives a coasting stop 
  delay(500);
  */


}

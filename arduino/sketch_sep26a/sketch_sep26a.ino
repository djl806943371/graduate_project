#include<Servo.h>
Servo myservo;
const int LED_PIN=12;
uint8_t pos = 150;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(LED_PIN);
  Serial.begin(9600);
//  pinMode(LED_PIN, OUTPUT);
}

void loop() {
////   put your main code here, to run repeatedly:
//  for(pos = 165; pos <= 175; pos += 1){
    myservo.write(90);
////    Serial.println(pos);
//    delay(200);
//  }
//      delay(2000);
//      myservo.write(172);
//    Serial.println(172);
//  for(pos = 171; pos>=165; pos-=1) {
//    myservo.write(pos);
//    Serial.println(pos);
//    delay(1000);
//  }
//  delay(2000);
////  myservo.write(pos);
}

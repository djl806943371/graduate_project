  #include<Servo.h>
Servo myservo;
const int LED_PIN=12;
uint8_t pos = 150;
int p = 90;
String comdata; 

void setup() {
  // put your setup code here, to run once:
  myservo.attach(LED_PIN);
  Serial.begin(19200);
  Serial.setTimeout(50);
  myservo.write(p);
//  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if(Serial.available() >= 3){
    while (Serial.available() > 0){
      comdata += char(Serial.read());  //每次读一个char字符，并相加
      if(comdata.length() == 3)
        break;
    }
    if (comdata.length() == 3){
      p = comdata.toInt();      // 在串口数据流中查找一个有效整数。
      myservo.write(p);
      Serial.print(comdata);         //打印接收到的数字
      comdata = "";
    }
  }
//  if (Serial.available() > 3) {   // 串口收到字符数大于零。
//      p = Serial.parseInt();      // 在串口数据流中查找一个有效整数。
//      myservo.write(p);
//      Serial.print(p);         //打印接收到的数字
//    }
////  delay(2000);
}

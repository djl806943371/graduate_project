#include <Servo.h>
Servo myservo_0, myservo_1, myservo_2, myservo_3;
const int SERVO_PIN_0 = 9, SERVO_PIN_1 = 10, SERVO_PIN_2 = 11, SERVO_PIN_3 = 12;
int pos[] = {90, 90, 90, 95}, pt = 0, posMid;
String comdata, posStr = "";
int test = 180;

void gradualChange(Servo ser, int pos){
  int posNow = ser.read();
  if(pos >= posNow){
    for(int i = posNow; i <= pos; ++i){
      ser.write(i);
    }
  }
  else{
    for(int i = posNow; i >= pos; --i){
      ser.write(i);
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  myservo_0.attach(SERVO_PIN_0);
  myservo_1.attach(SERVO_PIN_1);
  myservo_2.attach(SERVO_PIN_2);
  myservo_3.attach(SERVO_PIN_3);
  gradualChange(myservo_0, pos[0]);
  gradualChange(myservo_1, pos[1]);
  gradualChange(myservo_2, pos[2]);
  gradualChange(myservo_3, pos[3]);
  Serial.begin(57600);
  Serial.setTimeout(50);
  while(Serial.read() >= 0);
}

void loop()
{
  if (Serial.available() > 0)
  {
    comdata = Serial.readStringUntil('}');
    comdata += '}';
    if (comdata[0] == '{')
    {
      for (int i = 1; i < comdata.length(); ++i)  
      {
        if (comdata[i] == ' ' || comdata[i] == '}')
        {
          posMid = posStr.toInt();
          pos[pt++] = long(150 - posMid) * 180L / 300;
          posStr = "";
          continue;
        }
        posStr += comdata[i];
      }
      if (pt == 4)
      {
        gradualChange(myservo_0, pos[0]);
        gradualChange(myservo_1, pos[1]);
        gradualChange(myservo_2, pos[2]);
        gradualChange(myservo_3, pos[3] + 5);
      }
      else
      {
        Serial.print("ParseFail#");
      }
      pt = 0;
    }
    else
    {
      Serial.print("ReceiveFail#");
    }
  }
}

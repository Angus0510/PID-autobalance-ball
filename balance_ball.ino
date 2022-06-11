
#include <PID_v1.h>

#include "ESP32SharpIR.h"

#include <ESP32Servo.h>

#include "BluetoothSerial.h"

#include <Wire.h>

#include <LiquidCrystal.h>

#include <LiquidCrystal_PCF8574.h>

#include <IRremoteESP8266.h>
#include <IRrecv.h>



// Define model and input pin:

#define IRPin 27

#define model 0


/* SharpIR Model :

  GP2Y0A02YK0F --> 20150

  GP2Y0A21YK0F --> 1080

  GP2Y0A710K0F --> 100500

  GP2YA41SK0F --> 430

*/



// Create variables for PID:

double distance_cm;

double Setpoint, Input, Output;

double kp = 1.8, ki = 0.7, kd = 1.4;

double offset = 91;


// variables for filtering:

const int Filter_size = 30;

double FIR_input[Filter_size] = {0}; //FIR average filter

double Sum_input = 0;

int index1 = 0;




ESP32SharpIR mySensor(model, IRPin);

PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_E, REVERSE);

Servo myservo;


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

LiquidCrystal_PCF8574 lcd(0x27);

int RECV_PIN =19;
IRrecv irrecv(RECV_PIN);//紅外線接收器角位
decode_results results;//把接收到的信號儲存在results

void setup() {


  // put your setup code here, to run once:



  // 提供電源給 IRSensor

  pinMode(12, OUTPUT);

  digitalWrite(12, 1);

  mySensor.setFilterRate(0.1f);


  myservo.attach(4);

  myservo.write(offset - 5);

  Input = mySensor.getDistance();

  Setpoint = 10.0;


  // PID 參數設定

  myPID.SetMode(AUTOMATIC);

  myPID.SetOutputLimits(-80.0, 80.0);

  myPID.SetSampleTime(90);


  delay(1000);


  // Signal filtering

  for (int i = 0; i < Filter_size; i++) {

    FIR_input[i] = mySensor.getDistance();

    Sum_input += FIR_input[i];

  }

  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name

  lcd.begin(16, 2);//lcd(長,寬)

  irrecv.enableIRIn();

}



void loop() {
  unsigned long ct = millis();
  static unsigned long pt = millis();
  unsigned long ct2 = millis();
  static unsigned long pt2 = millis();
  static int state = 0;
  static int state2 = 0;
  static int state3 = 0;



  // Signal filtering

  Sum_input -= FIR_input[index1];

  FIR_input[index1] = mySensor.getDistance();

  Sum_input += FIR_input[index1];

  if (++ index1 == Filter_size) index1 = 0;

  Input = Sum_input / Filter_size; // distance measured by IRsensor


  // PID calculation

  myPID.Compute();

  myservo.write(offset + Output); //send command to servo

  //lcd顯示setpoint和input
  switch (state2) {
    case 0:
      if ((ct - pt) > 30) {
        Serial.print(Input);
        Serial.print(" ");
        Serial.println(Setpoint);
        lcd.setBacklight(255);
        lcd.setCursor(0, 0);
        lcd.print("aim:");
        lcd.setCursor(4, 0);
        lcd.print(Setpoint);
        lcd.setCursor(0, 1);
        lcd.print("now:");
        lcd.setCursor(4, 1);
        lcd.print(Input);
        pt = ct;
      }
      break;
  }

  //序列阜改變平衡點
  while (Serial.available() > 0) {
    int a = Serial.parseInt();
    Setpoint = a;
  }

  //app輸入改變平衡點
  if (SerialBT.available()) {
     int x = SerialBT.parseInt();
    //Serial.println(x);
    switch (x) {
      case 8:
        Setpoint = 8;
        break;
      case 9:
        Setpoint = 9;
        break;
      case 10:
        Setpoint = 10;
        break;
      case 11:
        Setpoint = 11;
        break;
      case 12:
        Setpoint = 12;
        break;
    }
  }

  //將紅外線接收器接收到的數值改變平衡點
  switch (state3) {
    case 0:
      if ((ct2 - pt2) > 100) {
        if (irrecv.decode(&results)) {
          if (results.value == 0xFF42BD) { 
            Setpoint=7;
          }
          if (results.value == 0xFF4AB5) { 
            Setpoint=8;
          }
          if (results.value == 0xFF52AD) { 
            Setpoint=9;
          }
          if (results.value == 0xFF6897) { 
            Setpoint=10;
          }
          if (results.value == 0xFF30CF) { 
            Setpoint=11;
          }
          if (results.value == 0xFF18E7) { 
            Setpoint=12;
          }
          irrecv.resume();
          pt2 = ct2;
        }
      }
      break;
  }
}

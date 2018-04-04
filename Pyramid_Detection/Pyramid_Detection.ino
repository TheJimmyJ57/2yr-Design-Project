/*
  Software serial MSE 2202 IR tester

  The circuit:

  RX is digital pin 7 (connect to TX of other device)
  TX is digital pin 11 (connect to RX of other device)

*/

#include <SoftwareSerial.h>

SoftwareSerial IR1(A1, 11); // RX, TX
SoftwareSerial IR2(A2, 11); // RX, TX
SoftwareSerial IR3(A3, 11); // RX, TX
int  sensor_select = 0;
unsigned int timer = 0;
int Timer = 0;
int pin_out1 = 0;
int pin_out2 = 0;
int pin_out3 = 0;
const int SwitchPin = 12;
const int IR1_pin = 2;
const int IR2_pin = 3;
const int IR3_pin = 4;


char char1;
char char2;

void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("MSE 2202 IR tester");

  // set the data rate for the SoftwareSerial port
  IR1.begin(2400);
  IR2.begin(2400);
  IR3.begin(2400);
  //mySerial.println("Hello, world?");

  pinMode(12, INPUT);

  pinMode(A1, INPUT);
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);

  pinMode(IR1_pin, OUTPUT);
  pinMode(IR2_pin, OUTPUT);
  pinMode(IR3_pin, OUTPUT);
}



void loop() { // run over and over

  if (digitalRead(SwitchPin) == HIGH)
  {
    char1 = 'A';
    char2 = 'E';
  }
  else
  {
    char1 = 'O';
    char2 = 'I';
  }


  switch (sensor_select) {
    case 0: {
        IR1.listen();
        if (IR1.available())
        {
          char var = IR1.read();
          if ((var == char1) || (var == char2)) {
            pin_out1 = 1;
          }
          Serial.print("Sensor 1: ");
          Serial.write(var);
          Serial.println();
        }
        break;
      }
    case 1: {
        IR2.listen();
        if (IR2.available())
        {
          char var = IR2.read();
          if ((var == char1) || (var == char2)) {
            pin_out2 = 1;
          }
          Serial.print("Sensor 2: ");
          Serial.write(var);
          Serial.println();
        }
        break;
      }
    case 2: {
        IR3.listen();
        if (IR3.available())
        {
          char var = IR3.read();
          if ((var == char1) || (var == char2)) {
            pin_out3 = 1;
          }
          Serial.print("Sensor 3: ");
          Serial.write(var);
          Serial.println();
        }
        break;
      }
  }

  Timer ++;
  if (Timer > 3000) {
    Timer = 0;
    sensor_select++;
    //timer = millis();
    if (sensor_select > 2)
    {
      sensor_select = 0;
    }
    TimerPinStuff(sensor_select);
  }
  IRSensorOutput(pin_out1, pin_out2, pin_out3);

  //Serial.write(sensor_select);

}

void TimerPinStuff(int select) {

  if (select == 0)
  {
    pin_out1 = 0;
  }

  if (select == 1)
  {
    pin_out2 = 0;
  }

  if (select == 2)
  {
    pin_out3 = 0;
  }
}

void IRSensorOutput(bool s1, bool s2, bool s3) {
  if (s1 && s2 && s3) {
    digitalWrite(IR1_pin, HIGH);
    digitalWrite(IR2_pin, HIGH);
    digitalWrite(IR3_pin, HIGH);
  }
  else if (s1 && s2 && !s3) {
    digitalWrite(IR1_pin, HIGH);
    digitalWrite(IR2_pin, HIGH);
    digitalWrite(IR3_pin, LOW);
  }
  else if (s1 && !s2 && s3) {
    digitalWrite(IR1_pin, HIGH);
    digitalWrite(IR2_pin, LOW);
    digitalWrite(IR3_pin, HIGH);
  }
  else if (!s1 && s2 && s3) {
    digitalWrite(IR1_pin, LOW);
    digitalWrite(IR2_pin, HIGH);
    digitalWrite(IR3_pin, HIGH);
  }
  else if (!s1 && !s2 && s3) {
    digitalWrite(IR1_pin, LOW);
    digitalWrite(IR2_pin, LOW);
    digitalWrite(IR3_pin, HIGH);
  }
  else if (!s1 && s2 && !s3) {
    digitalWrite(IR1_pin, LOW);
    digitalWrite(IR2_pin, HIGH);
    digitalWrite(IR3_pin, LOW);
  }
  else if (s1 && !s2 && !s3) {
    digitalWrite(IR1_pin, HIGH);
    digitalWrite(IR2_pin, LOW);
    digitalWrite(IR3_pin, LOW);
  }
  else if (!s1 && !s2 && !s3) {
    digitalWrite(IR1_pin, LOW);
    digitalWrite(IR2_pin, LOW);
    digitalWrite(IR3_pin, LOW);
  }
}

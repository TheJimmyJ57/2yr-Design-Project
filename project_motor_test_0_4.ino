/*

  MSE 2202 test code 1
  Language: Arduino
  Authors: Drew, Brooke, Gabriel and Brad
  Date: 19/3/18

  Rev 1 - Initial version

*/

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;
Servo servo_Winch;
Servo servo_Claw;
Servo servo_ClawSwivel;


//port pin constants
const int ci_S_Ultrasonic_Ping = 2;   //input plug
const int ci_S_Ultrasonic_Data = 3;
const int ci_F_Ultrasonic_Ping = 4;   //input plug
const int ci_F_Ultrasonic_Data = 5;
const int ci_Arm_Motor = 7;
const int ci_Mode_Button = 6;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Claw = 10;
const int ci_Claw_Swivel = 11;
const int ci_Winch = 12;
const int ci_LED = 13;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

//constants

// EEPROM addresses

const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Arm_Servo_Retracted = 55;      //  "
const int ci_Arm_Servo_Extended = 120;      //  "
const int ci_Display_Time = 500;
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_F_Echo_Time;
unsigned long ul_S_Echo_Time;
unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed = 1500;
unsigned int ui_Right_Motor_Speed = 1500;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF  //B0000000011111111,  //Run
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

////////////// BUTTON TOGGLE VARIABLES //////////////////////////////////////////////////////////////////////////////
bool ON = false;
unsigned int onTimer = 0;
unsigned int onTimerDelay = 1000;
int MODE = 2;
////////////// HEARTBEAT TIMER VARIABLES/////////////////////////////////////////////////////////////////////////////////////
unsigned int heartbeatTimer = 0;
int heartbeatDelay = 0;
////////////// ACTION TIMER VARIABLES/////////////////////////////////////////////////////////////////////////////////////
unsigned int actionTimer = 0;
int actionDelay = 0;


void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // set up side ultrasonic
  pinMode(ci_S_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_S_Ultrasonic_Data, INPUT);

  //set up front ultrasonic
  pinMode(ci_F_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_F_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Winch, OUTPUT);
  servo_Winch.attach(ci_Winch);
  pinMode(ci_Claw, OUTPUT);
  servo_Claw.attach(ci_Claw);
  pinMode(ci_Claw_Swivel, OUTPUT);
  servo_ClawSwivel.attach(ci_Claw_Swivel);

  // setup button
  pinMode(ci_Mode_Button, INPUT_PULLUP);
  pinMode(ci_LED, OUTPUT);

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
  servo_Claw.write(180);
  servo_ClawSwivel.write(180);

  //digitalWrite(ci_LED,HIGH);

}

void loop()
{
  OnOffButton();

  if (ON == false) {
    HeartBeat(120);
    servo_Claw.write(0); // claw open
    servo_ClawSwivel.write(45); // arm swivel in
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);
  }
  if (ON == true) {

    //HeartBeat(20);

    switch (MODE) {
      case 0: { // case0 arm movement
          if (Delay(3000, false) == true) {
            servo_Claw.write(180); // claw open
          }
          if (Delay(1000, false) == true) {
            servo_Claw.write(0); // claw close
          }
          if (Delay(1500, false) == true) {
            servo_ClawSwivel.write(45); // arm swivel in
          }
          if (Delay(800, false) == true) {
            servo_Claw.write(180); // claw open
          }
          if (Delay(3000, false) == true) {
            servo_ClawSwivel.write(180); //arm swivel out
          }
          if (Delay(50, true) == true) { // this one just resets the timers and such, hense last == true
            // end of loop
            ModeSet(1);
          }
          break;
        }
      case 1: {
          servo_Claw.write(180); // claw open
          servo_ClawSwivel.write(45); // arm swivel in

          if (Delay(2000, false) == true) {
            ui_Right_Motor_Speed = 1600;
            ui_Left_Motor_Speed = 1600;
          }
          if (Delay(2000, false) == true) {
            ui_Right_Motor_Speed = 1400;
            ui_Left_Motor_Speed = 1600;
          }

          if (Delay(50, true) == true) { // this one just resets the timers and such, hense last == true
            // end of loop
            ModeSet(0);
          }
          break;
        }
      case 2: { // wall Following (A)
          servo_Claw.write(0 ); // claw open
          servo_ClawSwivel.write(45); // arm swivel in

          ui_Right_Motor_Speed = 1650;
          ui_Left_Motor_Speed = 1650;

          Ping(1);
          if (((ul_F_Echo_Time / 148) < 8) && ((ul_F_Echo_Time / 148) > 0)) {
            ModeSet(3);
          }
          break;
        }
      case 3: { // wall Following (B)
          Ping(1);

          if (Delay(700, false) == true) { // 700 > 800
            ui_Left_Motor_Speed = 1350;
            ui_Right_Motor_Speed = 1650;
          }
          if (Delay(1000, false) == true) {
            Ping(0); //if not working, replace with turning off the motors
            if (((ul_S_Echo_Time / 148) < 8) && ((ul_S_Echo_Time / 148) > 0)) {
              ModeSet(2);
            }
          }

          if (Delay(50, false) == true) { // this one just resets the timers and such, hense last == true
            ModeSet(2);
          }

          if (Delay(50, true) == true) { // this one just resets the timers and such, hense last == true
          }
          break;
        }
    }

    servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
    servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
  }
}

void HeartBeat(int beatSpeed) { //  LED heart beat control . example of using the HHdelay funtion.

  if (HDelay(3 * beatSpeed, false) == true) {
    digitalWrite(ci_LED, LOW);
  }
  if (HDelay(1 * beatSpeed, false) == true) {
    digitalWrite(ci_LED, HIGH);
  }
  if (HDelay(1 * beatSpeed, false) == true) {
    digitalWrite(ci_LED, LOW);
  }
  if (HDelay(1 * beatSpeed, false) == true) {
    digitalWrite(ci_LED, HIGH);
  }
  if (HDelay(50, true) == true) { // this one just resets the timers and such, hense last == true
    // end of loop
  }

}

// Hdelay basically acts as a delay: it will do whatever is in the if statment for the indicated time.

unsigned int HDelay(int milisec, bool last) {
  heartbeatDelay += milisec;

  if (last == false) {
    if (((heartbeatTimer + heartbeatDelay - milisec) < millis() ) && ((heartbeatTimer + heartbeatDelay ) >= millis() )) {
      return true;
    }
    else
    {
      return false;
    }
  }


  if (last == true) {
    if ((heartbeatTimer + heartbeatDelay ) <=  millis()) {
      heartbeatTimer = millis();
      heartbeatDelay = 0;
      return true;
    }
    else
    {
      heartbeatDelay = 0;
      return false;
    }
  }
}

unsigned int Delay(int milisec, bool last) { // Delay basically acts as a delay: it will do whatever is in the if statment for the indicated time. but it controls everything that isnt the heart beat.
  actionDelay += milisec;
  if (last == false) {

    if (((actionTimer + actionDelay - milisec) < millis() ) && ((actionTimer + actionDelay ) >= millis() )) {
      return true;
    }
    else
    {
      return false;
    }
  }


  if (last == true) {
    if ((actionTimer + actionDelay ) <=  millis()) {
      actionTimer = millis();
      actionDelay = 0;
      return true;
    }
    else
    {
      actionDelay = 0;
      return false;
    }
  }
}


void ModeSet(int mode) { // this is used to transition between modes. it resets timers, and returns the motors to stopped.
  MODE = mode;
  actionTimer = millis();
  actionDelay = 0;
  ui_Left_Motor_Speed = 1500;
  ui_Right_Motor_Speed = 1500;
}

void OnOffButton() { // code for toggling the on/off variable
  if (digitalRead(ci_Mode_Button) == HIGH) { // restarting timer if LED flickers
    onTimer = millis();
    onTimerDelay = 1000; // resetting delay
  }

  if ((onTimer + onTimerDelay) < millis()) { // holding down button for the length of the delay

    ModeSet(MODE); // resetting timers and such.

    if (ON == true) // toggling ON
      ON = false;
    else
      ON = true;

    onTimer = millis(); // resetting timer
    onTimerDelay = 90000; // increasing delay so it doesnt trigger again
  }
}

void Ping( bool which)
{
  int ul_Echo_Time;
  if (which == 1) { // front

    digitalWrite(ci_F_Ultrasonic_Ping, HIGH);
    delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
    digitalWrite(ci_F_Ultrasonic_Ping, LOW);
    ul_F_Echo_Time = pulseIn(ci_F_Ultrasonic_Data, HIGH, 10000);
    ul_Echo_Time = ul_F_Echo_Time;
  }
  if (which == 0) { // side

    digitalWrite(ci_S_Ultrasonic_Ping, HIGH);
    delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
    digitalWrite(ci_S_Ultrasonic_Ping, LOW);
    ul_S_Echo_Time = pulseIn(ci_S_Ultrasonic_Data, HIGH, 10000);
    ul_Echo_Time = ul_S_Echo_Time;
  }
  // Print Sensor Readings
  //#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm
  //#endif
}


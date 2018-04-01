/* HELLO

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
const int ci_S1_Ultrasonic_Ping = 2;   //output plug
const int ci_S1_Ultrasonic_Data = 3;   //input plug
const int ci_F_Ultrasonic_Ping = 4;   //output plug
const int ci_F_Ultrasonic_Data = 5;   //input plug
const int ci_Winch = 6;
const int ci_Arm_Motor = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Claw = 10;
const int ci_Claw_Swivel = 11;
const int ci_Mode_Button = 12;
const int ci_LED = 13;
const int ci_ContactSwitch = A0;
const int ci_S2_Ultrasonic_Ping = A1;   //output plug
const int ci_S2_Ultrasonic_Data = A2;
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
unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed = 1500;
unsigned int ui_Right_Motor_Speed = 1500;
long l_Left_Motor_Position;
long l_Right_Motor_Position;
long ul_S1_Echo_Time;
long ul_S2_Echo_Time;
int diff;
int distToSide1;
int distToSide2;
int distToFront;

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
int MODE = 0;
////////////// HEARTBEAT TIMER VARIABLES/////////////////////////////////////////////////////////////////////////////////////
unsigned int heartbeatTimer = 0;
int heartbeatDelay = 0;
////////////// ACTION TIMER VARIABLES/////////////////////////////////////////////////////////////////////////////////////
unsigned int actionTimer = 0;
int actionDelay = 0;
///////////// ALIGNMENT VARIABLES ////////////////////////////////////////////////////////////////////////
int alignTolerance = 4;
int distFromFrontWall = 15;
int distFromSideWall = 11;
int lastAction = 0; // 0 = straight, 1 = right, 2 = left
int sideDelta = 0;
///////////// SERVO VARIABLES ///////////////////////////////////////
int ClawOpen = 140;
int ClawClosed = 0;
int ClawSwivelOpen = 175;
int ClawSwivelClosed = 45;
bool ClawUp = true;
bool ClawSwivelUp = true;
bool ArmUp = true;
bool WinchUp = true;

void GrabCube();

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // set up side ultrasonic
  pinMode(ci_S1_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_S1_Ultrasonic_Data, INPUT);

  pinMode(ci_S2_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_S2_Ultrasonic_Data, INPUT);

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

  //setup contact switch
  pinMode(ci_ContactSwitch, INPUT_PULLUP);

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);

  //digitalWrite(ci_LED,HIGH);
  ui_Left_Motor_Speed = 1650;
  ui_Right_Motor_Speed = 1600;

  servo_Claw.write(ClawClosed);
  servo_ClawSwivel.write(ClawSwivelClosed);
}

void loop()
{
  digitalWrite(13, HIGH);
  switch (MODE) {
    case 0:                 //mode to find wall
      {
        Ping();
        if ((distToFront < 25) || (distToSide1 < 25)) //check to see if the side or front are close to a wall
        {
          halt();
          MODE = 1;
        }
        else          //drive forward until the robot is close to a wall
        {
          driveStraight();
        }
        break;
      }
    case 1:               //mode to align robots side with wall
      {
        Ping();              //get new distance values
        spinLeft();
        if ((distToSide2 < 20) && (distToSide1 < 20))    //spin left until the robot aligns itself with the wall
        {
          halt();
          delay(3000);
          MODE = 2;
        }
        break;
      }
    case 2:                              // follow wall
      {
        Ping();
        /*  if (distToSide1 < 12)
          {
            Ping();
            driveLeft();
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
          }
        */
        if (diff < -alignTolerance)         //see if back sensor is too far from wall and readjust
        {
          spinLeft();
        }
        else if (diff > alignTolerance)
        {
          spinRight();
        }
        else if (distToFront < 20)     //else if the robot needs to turn because a wall is close ahead
        {
          MODE = 3;
        }
        else if ((diff > -5) && (diff < 5))         //if difference is within tolerance
        {
          driveStraight();
        }
        break;
      }
    case 3:                                 //begin turning robot
      {
        ui_Left_Motor_Speed = 1500;
        ui_Right_Motor_Speed = 1500;
        servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);         //halt
        servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        delay(1000);                                                    //delay to visually see its about to turn
        ui_Left_Motor_Speed = 1380;
        ui_Right_Motor_Speed = 1620;
        servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);             //write speeds for turning
        servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        delay(1000);                                                          //delay so robot can begin turning before considering whether it is aligned with wall after turn
        MODE = 1;
        break;
      }
    case 4:                                                                       //decide whether robot is aligned with new wall
      {
        /*   if((distToSide2 < 17) && (diff < 5)) //&& (distToFront > 20))
           {
             halt();
             delay(3000);
             MODE = 2;
           }
           break;
        
        Ping();              //get new distance values
        spinLeft();
        if ((distToSide2 < 20) && (distToSide1 < 20))    //spin left until the robot aligns itself with the wall
        {
          halt();
          delay(3000);
          MODE = 2;
        }
        break;
        */
      }
    case 5:                                                                         //if cube has tripped contact switch
      {
        GrabCube();
        MODE = 6;
        break;
      }
    case 6:                                                                           //pyramid finding
      {
        //hi
      }

  }

  //  Alingwheel();
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
  Serial.println(MODE);
}

void GrabCube()
{
  servo_Claw.write(ClawClosed);
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(2000);
  servo_ClawSwivel.write(ClawSwivelClosed);
  delay (2000);
  Serial.print("DONE");
}

void driveStraight()
{
  lastAction = 0;
  ui_Left_Motor_Speed = 1650;
  ui_Right_Motor_Speed = 1650;
}

void driveRight()
{
  lastAction = 1;
  ui_Left_Motor_Speed = 1610;
  ui_Right_Motor_Speed = 1600;
}

void driveLeft()
{
  lastAction = 2;
  ui_Left_Motor_Speed = 1600;
  ui_Right_Motor_Speed = 1610;
}

void spinRight()
{
  ui_Left_Motor_Speed = 1620;
  ui_Right_Motor_Speed = 1380;
}

void spinLeft()
{
  ui_Left_Motor_Speed = 1380;
  ui_Right_Motor_Speed = 1620;
}

void halt()
{
  ui_Left_Motor_Speed = 1500;
  ui_Right_Motor_Speed = 1500;
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void Ping()
{
  // front side

  digitalWrite(ci_S1_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_S1_Ultrasonic_Ping, LOW);
  ul_S1_Echo_Time = pulseIn(ci_S1_Ultrasonic_Data, HIGH, 10000);

  // back side

  digitalWrite(ci_S2_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_S2_Ultrasonic_Ping, LOW);
  ul_S2_Echo_Time = pulseIn(ci_S2_Ultrasonic_Data, HIGH, 10000);

  digitalWrite(ci_F_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_F_Ultrasonic_Ping, LOW);
  ul_F_Echo_Time = pulseIn(ci_F_Ultrasonic_Data, HIGH, 10000);

  distToSide1 = ul_S1_Echo_Time / 58;
  if (distToSide1 == 0)
  {
    distToSide1 = 9999;
  }
  distToSide2 = ul_S2_Echo_Time / 58;
  if (distToSide2 == 0)
  {
    distToSide2 = 999;
  }
  distToFront = ul_F_Echo_Time / 58;
  if (distToFront == 0)
  {
    distToFront = 999;
  }
  diff = distToSide1 - distToSide2; //0 = parallel, +pos = back is closer, -neg = front is closer

  // Print Sensor Readings
  //#ifdef DEBUG_ULTRASONIC
  if (MODE == 0) {
    Serial.print("S1Time (microseconds): ");
    Serial.print(ul_S1_Echo_Time, DEC);
    Serial.print(", cm: ");
    Serial.println(ul_S1_Echo_Time / 58); //divide time by 58 to get distance in cm

    Serial.print("S2Time (microseconds): ");
    Serial.print(ul_S2_Echo_Time, DEC);
    Serial.print(", cm: ");
    Serial.println(ul_S2_Echo_Time / 58); //divide time by 58 to get distance in cm

    Serial.print("F()Time (microseconds): ");
    Serial.print(ul_F_Echo_Time, DEC);
    Serial.print(", cm: ");
    Serial.println(ul_F_Echo_Time / 58); //divide time by 58 to get distance in cm
  }
  //#endif

}



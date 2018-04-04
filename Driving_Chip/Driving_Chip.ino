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
const int ci_ContactSwitch = 2;
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
const int ci_S2_Ultrasonic_Ping = A1;   //output plug
const int ci_S2_Ultrasonic_Data = A2;
const int ci_S1_Ultrasonic_Ping = A3;   //output plug
const int ci_S1_Ultrasonic_Data = A4;   //input plug
const int ci_IR1 = A0;
const int ci_IR2 = A5;
const int ci_IR3 = 3;


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
int findStep = 0;
int searchMode = 1;

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
bool pyramidClose = false;

////////////// BUTTON TOGGLE VARIABLES //////////////////////////////////////////////////////////////////////////////
bool ON = false;
unsigned int onTimer = 0;
unsigned int onTimerDelay = 1000;
int MODE = 0 ;
////////////// HEARTBEAT TIMER VARIABLES/////////////////////////////////////////////////////////////////////////////////////
unsigned int heartbeatTimer = 0;
int heartbeatDelay = 0;
////////////// ACTION TIMER VARIABLES/////////////////////////////////////////////////////////////////////////////////////
unsigned int actionTimer = 0;
int actionDelay = 0;
double previousmillis;
double CSmillis;
bool switchTripped;
///////////// ALIGNMENT VARIABLES ////////////////////////////////////////////////////////////////////////
int alignTolerance = 1;
int spinTolerance = 3;
int distFromFrontWall = 15;
int distFromSideWall = 11;
int lastAction = 0; // 0 = straight, 1 = right, 2 = left
int sideDelta = 0;
///////////// SERVO VARIABLES ///////////////////////////////////////
int ClawOpen = 140;
int ClawClosed = 0;
int ClawSwivelOpen = 175;
int ClawSwivelClosed = 45;
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

  //set up IR inputs
  pinMode(ci_IR1, INPUT);
  pinMode(ci_IR2, INPUT);
  pinMode(ci_IR3, INPUT);

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
  ui_Right_Motor_Speed = 1650;

  servo_Claw.write(90);
  delay(500);
  clawSwivelUp(false);
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
          delay(100);
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
        spinLeft(50);
        delay(500);
        while (!inTolerance(distToSide1, distToSide2))
        {
          Ping();
          cubeDetection();
        }

        /*  spinRight(30);
          while (!inTolerance(distToSide1, distToSide2))
          {
            Ping();
          }
          halt();
        */

        halt();
        delay(100);
        MODE = 2;
        /*if ((distToSide2 < 15) && (distToSide1 < 15))    //spin left until the robot aligns itself with the wall
          {
          halt();
          delay(1000);
          MODE = 2;
          }*/
        break;
      }
    case 2:                              // follow wall
      {
        Ping();
        followWall();
        break;
      }
    case 3:                                 //begin turning robot
      {
        halt();
        spinLeft(180);
        writeMotor();
        //delay so robot can begin turning before considering whether it is aligned with wall after turn
        if (millis() - previousmillis > 400)
        {
          MODE = 1;
        }

        break;
      }
    case 4:                                                                         //if cube has tripped contact switch
      {
        GrabCube();
        IRRead();
        if (findStep == 0)
        {
          spinLeft(30);
          delay(100);
          while (!inTolerance(distToSide1, distToSide2))
          {
            Ping();
          }
          halt();
        }
        MODE = 5;
        break;
      }
    case 5:                                                                           //pyramid finding
      {
        IRRead();

        break;
      }
    case 6:                                                                             //pyramid collection
      {
        acquirePyramid();
      }

  }

  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
  Serial.println(MODE);

  if (!switchTripped)
  {
    cubeDetection();
  }
}

void cubeDetection()
{
  if (digitalRead(2) == HIGH)
  {
    CSmillis = millis();
  }
  if ((millis() - CSmillis) > 200)
  {
    halt();
    switchTripped = true;
    MODE = 4;
  }
}

void GrabCube()
{
  servo_Claw.write(ClawClosed);
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(2000);
  servo_ClawSwivel.write(90);                               //set claw swivel to be in jousting position
  delay (2000);
  Serial.print("DONE");
}

void followWall()
{
  if (diff < -alignTolerance)         //see if back sensor is too far from wall and readjust
  {
    driveLeft();
  }
  else if (diff > alignTolerance)
  {
    driveRight();
  }
  else if (distToFront < 20)     //else if the robot needs to turn because a wall is close ahead
  {
    previousmillis = millis();
    MODE = 3;
  }
  else if ((diff > -2) && (diff < 2))         //if difference is within tolerance
  {
    driveStraight();
  }
}

void writeMotor()
{
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void driveStraight()
{
  lastAction = 0;
  ui_Left_Motor_Speed = 1700;
  ui_Right_Motor_Speed = 1700;
  writeMotor();
}

void driveRight()
{
  lastAction = 1;
  ui_Left_Motor_Speed = 1710;
  ui_Right_Motor_Speed = 1660;
  writeMotor();
}

void driveLeft()
{
  lastAction = 2;
  ui_Left_Motor_Speed = 1660;
  ui_Right_Motor_Speed = 1710;
  writeMotor();
}

void spinRight(int speed)
{
  ui_Left_Motor_Speed = (1500 + speed);
  ui_Right_Motor_Speed = (1500 - speed);
  writeMotor();
}

void spinLeft(int speed)
{
  ui_Left_Motor_Speed = (1500 - speed);
  ui_Right_Motor_Speed = (1500 + speed);
  writeMotor();
}

void halt()
{
  ui_Left_Motor_Speed = 1500;
  ui_Right_Motor_Speed = 1500;
  writeMotor();
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

  Serial.print("S1Time (microseconds): ");
  Serial.print(ul_S1_Echo_Time, DEC);
  Serial.print(", cm: ");
  Serial.println(ul_S1_Echo_Time / 58); //divide time by 58 to get distance in cm

  Serial.print("S2Time (microseconds): ");
  Serial.print(ul_S2_Echo_Time, DEC);
  Serial.print(", cm: ");
  Serial.println(ul_S2_Echo_Time / 58); //divide time by 58 to get distance in cm
  /*
    Serial.print("F()Time (microseconds): ");
    Serial.print(ul_F_Echo_Time, DEC);
    Serial.print(", cm: ");
    Serial.println(ul_F_Echo_Time / 58); //divide time by 58 to get distance in cm
  */

  //#endif

}

bool inTolerance(int num1, int num2)
{
  if (abs(num1 - num2) < 3 * alignTolerance)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void rampUp(bool Up)
{
  if (Up == true)
  {
    if (WinchUp == false)
    {
      servo_Winch.writeMicroseconds(1600);
      delay(400);
      servo_Winch.writeMicroseconds(1500);
      WinchUp = true;
    }
  }
  else
  {
    if (WinchUp == true)
    {
      servo_Winch.writeMicroseconds(1400);
      delay(400);
      servo_Winch.writeMicroseconds(1500);
      WinchUp = false;
    }
  }
}

void clawUp(bool Up)
{
  if (Up == true)
  {
    servo_Claw.write(ClawClosed);
  }
  else
  {
    servo_Claw.write(ClawOpen);
  }
}

void clawSwivelUp(bool Up)
{
  if (Up == true)
  {
    servo_ClawSwivel.write(ClawSwivelClosed);
  }
  else
  {
    servo_ClawSwivel.write(ClawSwivelOpen);
  }
}

void armUp(bool Up)
{
  if (Up == true)
  {
    if (ArmUp == false)
    {
      servo_ArmMotor.writeMicroseconds(1300);
      delay(2800);
      servo_ArmMotor.writeMicroseconds(1500);
      ArmUp = true;
    }
  }
  else
  {
    if (ArmUp == true)
    {
      servo_ArmMotor.writeMicroseconds(1700);
      delay(2800);
      servo_ArmMotor.writeMicroseconds(1500);
      ArmUp = false;
    }
  }
}


void IRRead()
{
  bool s1, s2, s3;
  if (digitalRead(ci_IR1) == HIGH)
  {
    s1 = true;
  }
  else
  {
    s1 = false;
  }
  if (digitalRead(ci_IR2) == HIGH)
  {
    s2 = true;
  }
  else
  {
    s2 = false;
  }
  if (digitalRead(ci_IR3) == HIGH)
  {
    s3 = true;
  }
  else
  {
    s3 = false;
  }
  IRSensorAction(s1, s2, s3);
}

void IRSensorAction(bool s1, bool s2, bool s3) {
  if (s1 && s2 && s3) {
    driveStraight();
    findStep = 1;
  }
  else if (s1 && s2 && !s3) {
    driveStraight();
    findStep = 2;
  }
  else if (s1 && !s2 && s3) {
    driveStraight();
    findStep = 4;
  }
  else if (!s1 && s2 && s3) {
    driveStraight();
    findStep = 3;
  }
  else if (!s1 && !s2 && s3) {
    spinRight(30);
    findStep = 7;
  }
  else if (!s1 && s2 && !s3) {                                            // if only the middle sensor sees it, aqcuire pyramid
    MODE = 6;
    findStep = 5;
  }
  else if (s1 && !s2 && !s3) {
    spinLeft(30);
    findStep = 6;
  }
  else if (!s1 && !s2 && !s3) {
    if ((findStep == 1) || (findStep == 2) || (findStep == 3) || findStep == 4)                   //if the middle sensor had the pyramid in view then lost it
    {
      MODE = 6;
    }
    else if (findStep == 7)                 //if left sensor had it but now it lost it
    {
      spinRight(30);
    }
    else if (findStep == 6)                   //if right sensor saw it but now it lost it
    {
      spinRight(30);                                                                                              //BOTH SPINRIGHT NEED TO CORRECT
    }
    else if (MODE != 4)                         //if no readings have occured and its not in the grabcube mode
    {
      Trace();                                    //if no readings of the correct pyramid have occured
    }
  }
}

void Trace()
{
  Ping;
  pyramidSearch();
}


void pyramidSearch()
{
  if (diff < -alignTolerance)         //see if back sensor is too far from wall and readjust
  {
    driveLeft();
  }
  else if (diff > alignTolerance)
  {
    driveRight();
  }
  else if (distToFront < 20)     //else if the robot needs to turn because a wall is close ahead
  {
    previousmillis = millis();
    MODE = 3;
  }
  else if ((diff > -2) && (diff < 2))         //if difference is within tolerance
  {
    driveStraight();
  }


  if (searchMode == 4) {
    searchMode = 0;
  }
  searchMode++;
}

void turn(int x)
{
  if (x == 0)
  {

  }
}


void acquirePyramid()                          //when pyramid is right infront
{
  for (int i = 0; i < 5; i++)
  {
    servo_Claw.write(ClawOpen);
    delay(500);
    servo_Claw.write(ClawClosed);
    delay(500);
  }
}

/*
  void CS_ISR()                       //ISR function
  {
  MODE = 4;                       //change global variable to show ISR completion
  detachInterrupt(0);               //remove interrupt because it only needs to be used once (could alter mode in isr for switch statement)
  }
*/


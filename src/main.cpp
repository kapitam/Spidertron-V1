#include <SPI.h>
#include <Ramp.h>
#include <Math.h>
#include <Wire.h>
#include <PSX.h>
#include <Adafruit_PWMServoDriver.h>

#define DATA_PIN   25   // White wire
#define CMD_PIN    26  // Black wire
#define ATT_PIN    32  // Gray wire
#define CLOCK_PIN  33  // black wire

//Assign Servo on PWM adafruit
#define SERVO_PINFR1 0
#define SERVO_PINFR2 1
#define SERVO_PINFR3 2
#define SERVO_PINCR1 4
#define SERVO_PINCR2 5
#define SERVO_PINCR3 6
#define SERVO_PINBR1 8
#define SERVO_PINBR2 9
#define SERVO_PINBR3 10
#define SERVO_PINFL1 0
#define SERVO_PINFL2 1
#define SERVO_PINFL3 2
#define SERVO_PINCL1 4
#define SERVO_PINCL2 5
#define SERVO_PINCL3 6
#define SERVO_PINBL1 8
#define SERVO_PINBL2 9
#define SERVO_PINBL3 10

const int minPulse = 450;
const int maxPulse = 2500; 

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

PSX psx;

// Variables to hold the controller state and error code
PSX::PSXDATA PSXdata;
int PSXerror;

const double J2L = 10.7;  // Length of the femur
const double J3L = 16.8;  // Length of the tibia
const double D = 6.5;     // Offset length from the start of femur to axis

rampDouble FRXTarget = 0.0, FRYTarget = 0.0, FRZTarget = 0.0;
rampDouble CRXTarget = 0.0, CRYTarget = 0.0, CRZTarget = 0.0;
rampDouble BRXTarget = 0.0, BRYTarget = 0.0, BRZTarget = 0.0;
rampDouble FLXTarget = 0.0, FLYTarget = 0.0, FLZTarget = 0.0;
rampDouble CLXTarget = 0.0, CLYTarget = 0.0, CLZTarget = 0.0;
rampDouble BLXTarget = 0.0, BLYTarget = 0.0, BLZTarget = 0.0;

double FRMotorXCurrent = 0.0, FRMotorYCurrent = 0.0, FRMotorZCurrent = 0.0;
double CRMotorXCurrent = 0.0, CRMotorYCurrent = 0.0, CRMotorZCurrent = 0.0;
double BRMotorXCurrent = 0.0, BRMotorYCurrent = 0.0, BRMotorZCurrent = 0.0;
double FLMotorXCurrent = 0.0, FLMotorYCurrent = 0.0, FLMotorZCurrent = 0.0;
double CLMotorXCurrent = 0.0, CLMotorYCurrent = 0.0, CLMotorZCurrent = 0.0;
double BLMotorXCurrent = 0.0, BLMotorYCurrent = 0.0, BLMotorZCurrent = 0.0;


bool defaultPosition = false;
bool stop = true;
bool start = false;

typedef struct test_struct {
  char side;
  int leg;
  double A1;
  double A2;
  double A3;
} test_struct;

test_struct target;

String currentMovement = "Idle";
String switchMovement = "Idle";

double duration = 6000.0;
double dur = 6000.0;

uint8_t commandStep = 1;

const double walkfwdF[4][3] {
  {15, -7.5, 21.64},
  {15, -7.5, 6.64},
  {15, -2, 6.64},
  {15, -2, 21.64},
};

const double walkfwdC[4][3] {
  {15, -2, -7.5},
  {15, -2, 7.5},
  {15, -7.5, 7.5},
  {15, -7.5, -7.5},
};

const double walkfwdB[4][3] {
  {15, -7.5, -6.64},
  {15, -7.5, -21.64},
  {15, -2, -21.64},
  {15, -2, -6.64},
};

const double walkbackF[4][3] {
  {15, -7.5, 6.64},
  {15, -7.5, 21.64},
  {15, -2, 21.64},
  {15, -2, 6.64},
};

const double walkbackC[4][3] {
  {15, -2, 7.5},
  {15, -2, -7.5},
  {15, -7.5, -7.5},
  {15, -7.5, 7.5},
};

const double walkbackB[4][3] {
  {15, -7.5, -21.64},
  {15, -7.5, -6.64},
  {15, -2, -6.64},
  {15, -2, -21.64},
};

const double walkleftF[4][3] {
  {6.64, -7.5, 20},
  {21.64, -7.5, 20},
  {21.64, -2, 20},
  {6.64, -2, 20},
};

const double walkleftC[4][3] {
  {31, -2, 0},
  {20, -2, 0},
  {20, -7.5, 0},
  {31, -7.5, 0},
};

const double walkleftB[4][3] {
  {6.64, -7.5, -20},
  {21.64, -7.5, -20},
  {21.64, -2, -20},
  {6.64, -2, -20},
};

const double walkrightF[4][3] {
  {21.64, -7.5, 20},
  {6.64, -7.5, 20},
  {6.64, -2, 20},
  {21.64, -2, 20},
};

const double walkrightC[4][3] {
  {20, -2, 0},
  {31, -2, 0},
  {31, -7.5, 0},
  {20, -7.5, 0},
};

const double walkrightB[4][3] {
  {21.64, -7.5, -20},
  {6.64, -7.5, -20},
  {6.64, -2, -20},
  {21.64, -2, -20},
};

void setup() {
  Serial.begin(115200);

  Serial.println("Begin");

  psx.setupPins(DATA_PIN, CMD_PIN, ATT_PIN, CLOCK_PIN, 10);
  psx.config(PSXMODE_ANALOG);

  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

}

double translator(double angle) {
  double temp = minPulse + (angle / 180.0) * (maxPulse - minPulse);
  return temp * 4096 / (1000000 / 50); // Convert to PWM ticks
}

void UpdatePosition(char side, int leg, double A1, double A2, double A3) {
    
  A1 = translator(180 - A1);
  A2 = translator(A2);
  A3 = translator(180 - A3);

  if (side == 'R') {
      if (leg == 1) {
          //SERVOFR1.write(180 - A1);
          //SERVOFR2.write(A2);
          //SERVOFR3.write(180 - A3);
          pwm1.setPWM(SERVO_PINFR1, 0, A1);
          pwm1.setPWM(SERVO_PINFR2, 0, A2);
          pwm1.setPWM(SERVO_PINFR3, 0, A3);
      } else if (leg == 2) {
          //SERVOCR1.write(180 - A1);
          //SERVOCR2.write(A2);
          //SERVOCR3.write(180 - A3);
          pwm1.setPWM(SERVO_PINCR1, 0, A1);
          pwm1.setPWM(SERVO_PINCR2, 0, A2);
          pwm1.setPWM(SERVO_PINCR3, 0, A3);
      } else if (leg == 3) {
          //SERVOBR1.write(180 - A1);
          //SERVOBR2.write(A2);
          //SERVOBR3.write(180 - A3);
          pwm1.setPWM(SERVO_PINBR1, 0, A1);
          pwm1.setPWM(SERVO_PINBR2, 0, A2);
          pwm1.setPWM(SERVO_PINBR3, 0, A3);
      }
  } else if (side == 'L') {
      if (leg == 1) {
          //SERVOFL1.write(180 - A1);
          //SERVOFL2.write(A2);
          //SERVOFL3.write(180 - A3);
          pwm2.setPWM(SERVO_PINFL1, 0, A1);
          pwm2.setPWM(SERVO_PINFL2, 0, A2);
          pwm2.setPWM(SERVO_PINFL3, 0, A3);
      } else if (leg == 2) {
          //SERVOCL1.write(180 - A1);
          //SERVOCL2.write(A2);
          //SERVOCL3.write(180 - A3);
          pwm2.setPWM(SERVO_PINCL1, 0, A1);
          pwm2.setPWM(SERVO_PINCL2, 0, A2);
          pwm2.setPWM(SERVO_PINCL3, 0, A3);
      } else if (leg == 3) {
          //SERVOBL1.write(180 - A1);
          //SERVOBL2.write(A2);
          //SERVOBL3.write(180 - A3);
          pwm2.setPWM(SERVO_PINBL1, 0, A1);
          pwm2.setPWM(SERVO_PINBL2, 0, A2);
          pwm2.setPWM(SERVO_PINBL3, 0, A3);
      }
  } else if (side == 'M') {
      if (leg == 1) {
          //SERVOFR1.write(180 - A1);
          //SERVOFR2.write(A2);
          //SERVOFR3.write(180 - A3);
          pwm1.setPWM(SERVO_PINFR1, 0, A1);
          pwm1.setPWM(SERVO_PINFR2, 0, A2);
          pwm1.setPWM(SERVO_PINFR3, 0, A3);
      } else if (leg == 2) {
          //SERVOCR1.write(180 - A1);
          //SERVOCR2.write(A2);
          //SERVOCR3.write(180 - A3);
          pwm1.setPWM(SERVO_PINCR1, 0, A1);
          pwm1.setPWM(SERVO_PINCR2, 0, A2);
          pwm1.setPWM(SERVO_PINCR3, 0, A3);
      } else if (leg == 3) {
          //SERVOBR1.write(180 - A1);
          //SERVOBR2.write(A2);
          //SERVOBR3.write(180 - A3);
          pwm1.setPWM(SERVO_PINBR1, 0, A1);
          pwm1.setPWM(SERVO_PINBR2, 0, A2);
          pwm1.setPWM(SERVO_PINBR3, 0, A3);
      } else if (leg == 4) {
          //SERVOFL1.write(180 - A1);
          //SERVOFL2.write(A2);
          //SERVOFL3.write(180 - A3);
          pwm2.setPWM(SERVO_PINFL1, 0, A1);
          pwm2.setPWM(SERVO_PINFL2, 0, A2);
          pwm2.setPWM(SERVO_PINFL3, 0, A3);
      } else if (leg == 5) {
          //SERVOCL1.write(180 - A1);
          //SERVOCL2.write(A2);
          //SERVOCL3.write(180 - A3);
          pwm2.setPWM(SERVO_PINCL1, 0, A1);
          pwm2.setPWM(SERVO_PINCL2, 0, A2);
          pwm2.setPWM(SERVO_PINCL3, 0, A3);
      } else if (leg == 6) {
          //SERVOBL1.write(180 - A1);
          //SERVOBL2.write(A2);
          //SERVOBL3.write(180 - A3);
          pwm2.setPWM(SERVO_PINBL1, 0, A1);
          pwm2.setPWM(SERVO_PINBL2, 0, A2);
          pwm2.setPWM(SERVO_PINBL3, 0, A3);
      } 
  }
}

void CoordinateToAngle(char side, int leg, double X, double Y, double Z) {
  // Offset the coordinates to account for the rest position
  //X -= 30;
  //Y -= 50;

  // Calculate the length of the hypotenuse
  double N = sqrt((Z * Z) + (X * X)) - D;
  double L = sqrt((Y * Y) + (N * N));
  if (side == 'R') {
    target.A1 = atan2(Z,X) * (180 / PI) -30 + leg * 60;
  }
  else if (side == 'L') {
    target.A1 = atan2(Z,X) * (180 / PI) + 210 - leg * 60;
  }
  if (side == 'M') {
    target.A1 = atan2(Z,X) * (180 / PI) + 90;
  }


  // Check if the target point is reachable
  if (N > (J2L + J3L)) {
    //Serial.println("Target out of reach. Please enter valid coordinates.");
    return;
  }

  // Calculate the angles using inverse kinematics
  target.A3 = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI);
  double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI);
  double A = atan2(Y, N) * (180 / PI);
  target.A2 = A + B + 90;


  if (target.A2 > 180) {
  target.A2 = 360 - target.A2;
  }

  target.side = side;
  target.leg = leg;
  UpdatePosition(target.side, target.leg, target.A1, target.A2, target.A3);

  Serial.print(target.side);
  Serial.print(target.leg);
  Serial.print(target.A1);
  Serial.print(target.A2);
  Serial.println(target.A3);
}

void prepareMovement(String movementType) {
  dur = 500;
  if (movementType != switchMovement) {

    if (movementType == "Forward") {

    FRXTarget.go(walkfwdF[commandStep][0], dur);
    FRYTarget.go(walkfwdF[commandStep][1], dur);
    FRZTarget.go(walkfwdF[commandStep][2], dur);

    CRXTarget.go(walkfwdC[commandStep][0], dur);
    CRYTarget.go(walkfwdC[commandStep][1], dur);
    CRZTarget.go(walkfwdC[commandStep][2], dur);

    BRXTarget.go(walkfwdB[commandStep][0], dur);
    BRYTarget.go(walkfwdB[commandStep][1], dur);
    BRZTarget.go(walkfwdB[commandStep][2], dur);

    FLXTarget.go(walkfwdF[commandStep][0], dur);
    FLYTarget.go(walkfwdC[commandStep][1], dur);
    FLZTarget.go(walkfwdB[commandStep][2], dur);
    
    CLXTarget.go(walkfwdC[commandStep][0], dur);
    CLYTarget.go(walkfwdF[commandStep][1], dur);
    CLZTarget.go(walkfwdC[commandStep][2], dur);

    BLXTarget.go(walkfwdB[commandStep][0], dur);
    BLYTarget.go(walkfwdC[commandStep][1], dur);
    BLZTarget.go(walkfwdF[commandStep][2], dur);

    } else if (movementType == "Backward") {

    FRXTarget.go(walkbackF[commandStep][0], dur);
    FRYTarget.go(walkbackF[commandStep][1], dur);
    FRZTarget.go(walkbackF[commandStep][2], dur);

    CRXTarget.go(walkbackC[commandStep][0], dur);
    CRYTarget.go(walkbackC[commandStep][1], dur);
    CRZTarget.go(walkbackC[commandStep][2], dur);

    BRXTarget.go(walkbackB[commandStep][0], dur);
    BRYTarget.go(walkbackB[commandStep][1], dur);
    BRZTarget.go(walkbackB[commandStep][2], dur);

    FLXTarget.go(walkbackF[commandStep][0], dur);
    FLYTarget.go(walkbackC[commandStep][1], dur);
    FLZTarget.go(walkbackB[commandStep][2], dur);
    
    CLXTarget.go(walkbackC[commandStep][0], dur);
    CLYTarget.go(walkbackF[commandStep][1], dur);
    CLZTarget.go(walkbackC[commandStep][2], dur);

    BLXTarget.go(walkbackB[commandStep][0], dur);
    BLYTarget.go(walkbackC[commandStep][1], dur);
    BLZTarget.go(walkbackF[commandStep][2], dur);

    } else if (movementType == "Left") {

    FRXTarget.go(walkleftF[commandStep][0], dur);
    FRYTarget.go(walkleftF[commandStep][1], dur);
    FRZTarget.go(walkleftF[commandStep][2], dur);

    CRXTarget.go(walkleftC[commandStep][0], dur);
    CRYTarget.go(walkleftC[commandStep][1], dur);
    CRZTarget.go(walkleftC[commandStep][2], dur);

    BRXTarget.go(walkleftB[commandStep][0], dur);
    BRYTarget.go(walkleftB[commandStep][1], dur);
    BRZTarget.go(walkleftB[commandStep][2], dur);

    FLXTarget.go(walkleftF[commandStep][0], dur);
    FLYTarget.go(walkleftC[commandStep][1], dur);
    FLZTarget.go(-walkleftF[commandStep][2], dur);
    
    CLXTarget.go(walkleftC[commandStep][0], dur);
    CLYTarget.go(walkleftF[commandStep][1], dur);
    CLZTarget.go(-walkleftC[commandStep][2], dur);

    BLXTarget.go(walkleftB[commandStep][0], dur);
    BLYTarget.go(walkleftC[commandStep][1], dur);
    BLZTarget.go(-walkleftB[commandStep][2], dur);

    } else if (movementType == "Right") {

    FRXTarget.go(walkrightF[commandStep][0], dur);
    FRYTarget.go(walkrightF[commandStep][1], dur);
    FRZTarget.go(walkrightF[commandStep][2], dur);

    CRXTarget.go(walkrightC[commandStep][0], dur);
    CRYTarget.go(walkrightC[commandStep][1], dur);
    CRZTarget.go(walkrightC[commandStep][2], dur);

    BRXTarget.go(walkrightB[commandStep][0], dur);
    BRYTarget.go(walkrightB[commandStep][1], dur);
    BRZTarget.go(walkrightB[commandStep][2], dur);

    FLXTarget.go(walkrightF[commandStep][0], dur);
    FLYTarget.go(walkrightC[commandStep][1], dur);
    FLZTarget.go(-walkrightF[commandStep][2], dur);
    
    CLXTarget.go(walkrightC[commandStep][0], dur);
    CLYTarget.go(walkrightF[commandStep][1], dur);
    CLZTarget.go(-walkrightC[commandStep][2], dur);

    BLXTarget.go(walkrightB[commandStep][0], dur);
    BLYTarget.go(walkrightC[commandStep][1], dur);
    BLZTarget.go(-walkrightB[commandStep][2], dur);

    } else if (movementType == "Turn Left") {

    FRXTarget.go(walkfwdC[commandStep][0], dur);
    FRYTarget.go(walkfwdF[commandStep][1], dur);
    FRZTarget.go(-walkfwdC[commandStep][2], dur);

    CRXTarget.go(walkfwdC[commandStep][0], dur);
    CRYTarget.go(walkfwdC[commandStep][1], dur);
    CRZTarget.go(walkfwdC[commandStep][2], dur);

    BRXTarget.go(walkfwdC[commandStep][0], dur);
    BRYTarget.go(walkfwdF[commandStep][1], dur);
    BRZTarget.go(-walkfwdC[commandStep][2], dur);

    FLXTarget.go(walkfwdC[commandStep][0], dur);
    FLYTarget.go(walkfwdC[commandStep][1], dur);
    FLZTarget.go(walkfwdC[commandStep][2], dur);
    
    CLXTarget.go(walkfwdC[commandStep][0], dur);
    CLYTarget.go(walkfwdF[commandStep][1], dur);
    CLZTarget.go(-walkfwdC[commandStep][2], dur);

    BLXTarget.go(walkfwdC[commandStep][0], dur);
    BLYTarget.go(walkfwdC[commandStep][1], dur);
    BLZTarget.go(walkfwdC[commandStep][2], dur);

    } else if (movementType == "Turn Right") {

    FRXTarget.go(walkfwdC[commandStep][0], dur);
    FRYTarget.go(walkfwdF[commandStep][1], dur);
    FRZTarget.go(walkfwdC[commandStep][2], dur);

    CRXTarget.go(walkfwdC[commandStep][0], dur);
    CRYTarget.go(walkfwdC[commandStep][1], dur);
    CRZTarget.go(-walkfwdC[commandStep][2], dur);

    BRXTarget.go(walkfwdC[commandStep][0], dur);
    BRYTarget.go(walkfwdF[commandStep][1], dur);
    BRZTarget.go(walkfwdC[commandStep][2], dur);

    FLXTarget.go(walkfwdC[commandStep][0], dur);
    FLYTarget.go(walkfwdC[commandStep][1], dur);
    FLZTarget.go(-walkfwdC[commandStep][2], dur);
    
    CLXTarget.go(walkfwdC[commandStep][0], dur);
    CLYTarget.go(walkfwdF[commandStep][1], dur);
    CLZTarget.go(walkfwdC[commandStep][2], dur);

    BLXTarget.go(walkfwdC[commandStep][0], dur);
    BLYTarget.go(walkfwdC[commandStep][1], dur);
    BLZTarget.go(-walkfwdC[commandStep][2], dur);

    }
    
    currentMovement = "Transition";
  }

  FRMotorXCurrent = FRXTarget.update();
  FRMotorYCurrent = FRYTarget.update();
  FRMotorZCurrent = FRZTarget.update();
  CRMotorXCurrent = CRXTarget.update();
  CRMotorYCurrent = CRYTarget.update();
  CRMotorZCurrent = CRZTarget.update();
  BRMotorXCurrent = BRXTarget.update();
  BRMotorYCurrent = BRYTarget.update();
  BRMotorZCurrent = BRZTarget.update();
  FLMotorXCurrent = FLXTarget.update();
  FLMotorYCurrent = FLYTarget.update();
  FLMotorZCurrent = FLZTarget.update();
  CLMotorXCurrent = CLXTarget.update();
  CLMotorYCurrent = CLYTarget.update();
  CLMotorZCurrent = CLZTarget.update();
  BLMotorXCurrent = BLXTarget.update();
  BLMotorYCurrent = BLYTarget.update();
  BLMotorZCurrent = BLZTarget.update();

  if (movementType == "Turn Right" | movementType == "Turn Left") {
  CoordinateToAngle('M', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('M', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('M', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('M', 4, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('M', 5, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('M', 6, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);

  } else {
  CoordinateToAngle('R', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('R', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('R', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('L', 1, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('L', 2, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('L', 3, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);
  }

  if (FRXTarget.isFinished() && FRYTarget.isFinished() && FRZTarget.isFinished()
  && CRXTarget.isFinished() && CRYTarget.isFinished() && CRZTarget.isFinished()
  && BRXTarget.isFinished() && BRYTarget.isFinished() && BRZTarget.isFinished()
  && FLXTarget.isFinished() && FLYTarget.isFinished() && FLZTarget.isFinished()
  && CLXTarget.isFinished() && CLYTarget.isFinished() && CLZTarget.isFinished()
  && BLXTarget.isFinished() && BLYTarget.isFinished() && BLZTarget.isFinished()) {

    currentMovement = movementType;

  }
}

void DefaultPosition() {
  currentMovement = "Transition";
  FRMotorXCurrent = 20;
  FRMotorYCurrent = -10;
  FRMotorZCurrent = 0;
  CRMotorXCurrent = 20;
  CRMotorYCurrent = -10;
  CRMotorZCurrent = 0;
  BRMotorXCurrent = 20;
  BRMotorYCurrent = -10;
  BRMotorZCurrent = 0;
  FLMotorXCurrent = 20;
  FLMotorYCurrent = -10;
  FLMotorZCurrent = 0;
  CLMotorXCurrent = 20;
  CLMotorYCurrent = -10;
  CLMotorZCurrent = 0;
  BLMotorXCurrent = 20;
  BLMotorYCurrent = -10;
  BLMotorZCurrent = 0;
  //Serial.println(BLMotorXCurrent);
  //Serial.println(BLMotorYCurrent);
  //Serial.println(BLMotorZCurrent);

  CoordinateToAngle('M', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('M', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('M', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('M', 4, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('M', 5, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('M', 6, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);

  defaultPosition = true;
  currentMovement = "Idle";
  commandStep = 1;
  
}

void WalkForward() {
  if (FRXTarget.isFinished() && FRYTarget.isFinished() && FRZTarget.isFinished()
  && CRXTarget.isFinished() && CRYTarget.isFinished() && CRZTarget.isFinished()
  && BRXTarget.isFinished() && BRYTarget.isFinished() && BRZTarget.isFinished()
  && FLXTarget.isFinished() && FLYTarget.isFinished() && FLZTarget.isFinished()
  && CLXTarget.isFinished() && CLYTarget.isFinished() && CLZTarget.isFinished()
  && BLXTarget.isFinished() && BLYTarget.isFinished() && BLZTarget.isFinished()) {

    commandStep = (commandStep + 1) % 4; // Loop back to the first step

    double xNew = walkfwdF[commandStep][0];
    double yNew = walkfwdF[commandStep][1];
    double zNew = walkfwdF[commandStep][2];
    
    FRXTarget.go(xNew, dur);
    FRYTarget.go(yNew, dur);
    FRZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = walkfwdC[commandStep][2];
    
    CRXTarget.go(xNew, dur);
    CRYTarget.go(yNew, dur);
    CRZTarget.go(zNew, dur);

    xNew = walkfwdB[commandStep][0];
    yNew = walkfwdB[commandStep][1];
    zNew = walkfwdB[commandStep][2];

    BRXTarget.go(xNew, dur);
    BRYTarget.go(yNew, dur);
    BRZTarget.go(zNew, dur);

    xNew = walkfwdF[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = walkfwdB[commandStep][2];

    FLXTarget.go(xNew, dur);
    FLYTarget.go(yNew, dur);
    FLZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdF[commandStep][1];
    zNew = walkfwdC[commandStep][2];

    CLXTarget.go(xNew, dur);
    CLYTarget.go(yNew, dur);
    CLZTarget.go(zNew, dur);

    xNew = walkfwdB[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = walkfwdF[commandStep][2];

    BLXTarget.go(xNew, dur);
    BLYTarget.go(yNew, dur);
    BLZTarget.go(zNew, dur);
  }

  FRMotorXCurrent = FRXTarget.update();
  FRMotorYCurrent = FRYTarget.update();
  FRMotorZCurrent = FRZTarget.update();
  CRMotorXCurrent = CRXTarget.update();
  CRMotorYCurrent = CRYTarget.update();
  CRMotorZCurrent = CRZTarget.update();
  BRMotorXCurrent = BRXTarget.update();
  BRMotorYCurrent = BRYTarget.update();
  BRMotorZCurrent = BRZTarget.update();
  FLMotorXCurrent = FLXTarget.update();
  FLMotorYCurrent = FLYTarget.update();
  FLMotorZCurrent = FLZTarget.update();
  CLMotorXCurrent = CLXTarget.update();
  CLMotorYCurrent = CLYTarget.update();
  CLMotorZCurrent = CLZTarget.update();
  BLMotorXCurrent = BLXTarget.update();
  BLMotorYCurrent = BLYTarget.update();
  BLMotorZCurrent = BLZTarget.update();

  CoordinateToAngle('R', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('R', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('R', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('L', 1, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('L', 2, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('L', 3, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);

}

void WalkBackward() {
  if (FRXTarget.isFinished() && FRYTarget.isFinished() && FRZTarget.isFinished()
  && CRXTarget.isFinished() && CRYTarget.isFinished() && CRZTarget.isFinished()
  && BRXTarget.isFinished() && BRYTarget.isFinished() && BRZTarget.isFinished()
  && FLXTarget.isFinished() && FLYTarget.isFinished() && FLZTarget.isFinished()
  && CLXTarget.isFinished() && CLYTarget.isFinished() && CLZTarget.isFinished()
  && BLXTarget.isFinished() && BLYTarget.isFinished() && BLZTarget.isFinished()) {
    commandStep = (commandStep + 1) % 4; // Loop back to the first step
    double xNew = walkbackF[commandStep][0];
    double yNew = walkbackF[commandStep][1];
    double zNew = walkbackF[commandStep][2];
    
    FRXTarget.go(xNew, dur);
    FRYTarget.go(yNew, dur);
    FRZTarget.go(zNew, dur);

    xNew = walkbackC[commandStep][0];
    yNew = walkbackC[commandStep][1];
    zNew = walkbackC[commandStep][2];
    
    CRXTarget.go(xNew, dur);
    CRYTarget.go(yNew, dur);
    CRZTarget.go(zNew, dur);

    xNew = walkbackB[commandStep][0];
    yNew = walkbackB[commandStep][1];
    zNew = walkbackB[commandStep][2];

    BRXTarget.go(xNew, dur);
    BRYTarget.go(yNew, dur);
    BRZTarget.go(zNew, dur);

    xNew = walkbackF[commandStep][0];
    yNew = walkbackC[commandStep][1];
    zNew = walkbackB[commandStep][2];
    
    FLXTarget.go(xNew, dur);
    FLYTarget.go(yNew, dur);
    FLZTarget.go(zNew, dur);

    xNew = walkbackC[commandStep][0];
    yNew = walkbackF[commandStep][1];
    zNew = walkbackC[commandStep][2];
    
    CLXTarget.go(xNew, dur);
    CLYTarget.go(yNew, dur);
    CLZTarget.go(zNew, dur);

    xNew = walkbackB[commandStep][0];
    yNew = walkbackC[commandStep][1];
    zNew = walkbackF[commandStep][2];

    BLXTarget.go(xNew, dur);
    BLYTarget.go(yNew, dur);
    BLZTarget.go(zNew, dur);
  }

  FRMotorXCurrent = FRXTarget.update();
  FRMotorYCurrent = FRYTarget.update();
  FRMotorZCurrent = FRZTarget.update();
  CRMotorXCurrent = CRXTarget.update();
  CRMotorYCurrent = CRYTarget.update();
  CRMotorZCurrent = CRZTarget.update();
  BRMotorXCurrent = BRXTarget.update();
  BRMotorYCurrent = BRYTarget.update();
  BRMotorZCurrent = BRZTarget.update();
  FLMotorXCurrent = FLXTarget.update();
  FLMotorYCurrent = FLYTarget.update();
  FLMotorZCurrent = FLZTarget.update();
  CLMotorXCurrent = CLXTarget.update();
  CLMotorYCurrent = CLYTarget.update();
  CLMotorZCurrent = CLZTarget.update();
  BLMotorXCurrent = BLXTarget.update();
  BLMotorYCurrent = BLYTarget.update();
  BLMotorZCurrent = BLZTarget.update();

  CoordinateToAngle('R', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('R', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('R', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('L', 1, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('L', 2, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('L', 3, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);

}

void WalkLeft() {
  if (FRXTarget.isFinished() && FRYTarget.isFinished() && FRZTarget.isFinished()
  && CRXTarget.isFinished() && CRYTarget.isFinished() && CRZTarget.isFinished()
  && BRXTarget.isFinished() && BRYTarget.isFinished() && BRZTarget.isFinished()
  && FLXTarget.isFinished() && FLYTarget.isFinished() && FLZTarget.isFinished()
  && CLXTarget.isFinished() && CLYTarget.isFinished() && CLZTarget.isFinished()
  && BLXTarget.isFinished() && BLYTarget.isFinished() && BLZTarget.isFinished()) {
    commandStep = (commandStep + 1) % 4; // Loop back to the first step
    double xNew = walkleftF[commandStep][0];
    double yNew = walkleftF[commandStep][1];
    double zNew = walkleftF[commandStep][2];
    
    FRXTarget.go(xNew, dur);
    FRYTarget.go(yNew, dur);
    FRZTarget.go(zNew, dur);

    xNew = walkleftC[commandStep][0];
    yNew = walkleftC[commandStep][1];
    zNew = walkleftC[commandStep][2];
    
    CRXTarget.go(xNew, dur);
    CRYTarget.go(yNew, dur);
    CRZTarget.go(zNew, dur);

    xNew = walkleftB[commandStep][0];
    yNew = walkleftB[commandStep][1];
    zNew = walkleftB[commandStep][2];

    BRXTarget.go(xNew, dur);
    BRYTarget.go(yNew, dur);
    BRZTarget.go(zNew, dur);

    xNew = walkleftF[commandStep][0];
    yNew = walkleftC[commandStep][1];
    zNew = -walkleftF[commandStep][2];
    
    FLXTarget.go(xNew, dur);
    FLYTarget.go(yNew, dur);
    FLZTarget.go(zNew, dur);

    xNew = walkleftC[commandStep][0];
    yNew = walkleftF[commandStep][1];
    zNew = -walkleftC[commandStep][2];
    
    CLXTarget.go(xNew, dur);
    CLYTarget.go(yNew, dur);
    CLZTarget.go(zNew, dur);

    xNew = walkleftB[commandStep][0];
    yNew = walkleftC[commandStep][1];
    zNew = -walkleftB[commandStep][2];

    BLXTarget.go(xNew, dur);
    BLYTarget.go(yNew, dur);
    BLZTarget.go(zNew, dur);
  }

  FRMotorXCurrent = FRXTarget.update();
  FRMotorYCurrent = FRYTarget.update();
  FRMotorZCurrent = FRZTarget.update();
  CRMotorXCurrent = CRXTarget.update();
  CRMotorYCurrent = CRYTarget.update();
  CRMotorZCurrent = CRZTarget.update();
  BRMotorXCurrent = BRXTarget.update();
  BRMotorYCurrent = BRYTarget.update();
  BRMotorZCurrent = BRZTarget.update();
  FLMotorXCurrent = FLXTarget.update();
  FLMotorYCurrent = FLYTarget.update();
  FLMotorZCurrent = FLZTarget.update();
  CLMotorXCurrent = CLXTarget.update();
  CLMotorYCurrent = CLYTarget.update();
  CLMotorZCurrent = CLZTarget.update();
  BLMotorXCurrent = BLXTarget.update();
  BLMotorYCurrent = BLYTarget.update();
  BLMotorZCurrent = BLZTarget.update();

  CoordinateToAngle('R', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('R', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('R', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('L', 1, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('L', 2, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('L', 3, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);
}

void WalkRight() {
  if (FRXTarget.isFinished() && FRYTarget.isFinished() && FRZTarget.isFinished()
  && CRXTarget.isFinished() && CRYTarget.isFinished() && CRZTarget.isFinished()
  && BRXTarget.isFinished() && BRYTarget.isFinished() && BRZTarget.isFinished()
  && FLXTarget.isFinished() && FLYTarget.isFinished() && FLZTarget.isFinished()
  && CLXTarget.isFinished() && CLYTarget.isFinished() && CLZTarget.isFinished()
  && BLXTarget.isFinished() && BLYTarget.isFinished() && BLZTarget.isFinished()) {
    commandStep = (commandStep + 1) % 4; // Loop back to the first step
    double xNew = walkrightF[commandStep][0];
    double yNew = walkrightF[commandStep][1];
    double zNew = walkrightF[commandStep][2];
    
    FRXTarget.go(xNew, dur);
    FRYTarget.go(yNew, dur);
    FRZTarget.go(zNew, dur);

    xNew = walkrightC[commandStep][0];
    yNew = walkrightC[commandStep][1];
    zNew = walkrightC[commandStep][2];
    
    CRXTarget.go(xNew, dur);
    CRYTarget.go(yNew, dur);
    CRZTarget.go(zNew, dur);

    xNew = walkrightB[commandStep][0];
    yNew = walkrightB[commandStep][1];
    zNew = walkrightB[commandStep][2];

    BRXTarget.go(xNew, dur);
    BRYTarget.go(yNew, dur);
    BRZTarget.go(zNew, dur);

    xNew = walkrightF[commandStep][0];
    yNew = walkrightC[commandStep][1];
    zNew = -walkrightF[commandStep][2];
    
    FLXTarget.go(xNew, dur);
    FLYTarget.go(yNew, dur);
    FLZTarget.go(zNew, dur);

    xNew = walkrightC[commandStep][0];
    yNew = walkrightF[commandStep][1];
    zNew = -walkrightC[commandStep][2];
    
    CLXTarget.go(xNew, dur);
    CLYTarget.go(yNew, dur);
    CLZTarget.go(zNew, dur);

    xNew = walkrightB[commandStep][0];
    yNew = walkrightC[commandStep][1];
    zNew = -walkrightB[commandStep][2];

    BLXTarget.go(xNew, dur);
    BLYTarget.go(yNew, dur);
    BLZTarget.go(zNew, dur);
  }

  FRMotorXCurrent = FRXTarget.update();
  FRMotorYCurrent = FRYTarget.update();
  FRMotorZCurrent = FRZTarget.update();
  CRMotorXCurrent = CRXTarget.update();
  CRMotorYCurrent = CRYTarget.update();
  CRMotorZCurrent = CRZTarget.update();
  BRMotorXCurrent = BRXTarget.update();
  BRMotorYCurrent = BRYTarget.update();
  BRMotorZCurrent = BRZTarget.update();
  FLMotorXCurrent = FLXTarget.update();
  FLMotorYCurrent = FLYTarget.update();
  FLMotorZCurrent = FLZTarget.update();
  CLMotorXCurrent = CLXTarget.update();
  CLMotorYCurrent = CLYTarget.update();
  CLMotorZCurrent = CLZTarget.update();
  BLMotorXCurrent = BLXTarget.update();
  BLMotorYCurrent = BLYTarget.update();
  BLMotorZCurrent = BLZTarget.update();

  CoordinateToAngle('R', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('R', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('R', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('L', 1, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('L', 2, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('L', 3, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);
}

void TurnLeft() {
  if (FRXTarget.isFinished() && FRYTarget.isFinished() && FRZTarget.isFinished()
  && CRXTarget.isFinished() && CRYTarget.isFinished() && CRZTarget.isFinished()
  && BRXTarget.isFinished() && BRYTarget.isFinished() && BRZTarget.isFinished()
  && FLXTarget.isFinished() && FLYTarget.isFinished() && FLZTarget.isFinished()
  && CLXTarget.isFinished() && CLYTarget.isFinished() && CLZTarget.isFinished()
  && BLXTarget.isFinished() && BLYTarget.isFinished() && BLZTarget.isFinished()) {

    commandStep = (commandStep + 1) % 4; // Loop back to the first step

    double xNew = walkfwdC[commandStep][0];
    double yNew = walkfwdF[commandStep][1];
    double zNew = -walkfwdC[commandStep][2];
    
    FRXTarget.go(xNew, dur);
    FRYTarget.go(yNew, dur);
    FRZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = walkfwdC[commandStep][2];
    
    CRXTarget.go(xNew, dur);
    CRYTarget.go(yNew, dur);
    CRZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdF[commandStep][1];
    zNew = -walkfwdC[commandStep][2];

    BRXTarget.go(xNew, dur);
    BRYTarget.go(yNew, dur);
    BRZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = walkfwdC[commandStep][2];

    FLXTarget.go(xNew, dur);
    FLYTarget.go(yNew, dur);
    FLZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdF[commandStep][1];
    zNew = -walkfwdC[commandStep][2];

    CLXTarget.go(xNew, dur);
    CLYTarget.go(yNew, dur);
    CLZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = walkfwdC[commandStep][2];

    BLXTarget.go(xNew, dur);
    BLYTarget.go(yNew, dur);
    BLZTarget.go(zNew, dur);
  }

  FRMotorXCurrent = FRXTarget.update();
  FRMotorYCurrent = FRYTarget.update();
  FRMotorZCurrent = FRZTarget.update();
  CRMotorXCurrent = CRXTarget.update();
  CRMotorYCurrent = CRYTarget.update();
  CRMotorZCurrent = CRZTarget.update();
  BRMotorXCurrent = BRXTarget.update();
  BRMotorYCurrent = BRYTarget.update();
  BRMotorZCurrent = BRZTarget.update();
  FLMotorXCurrent = FLXTarget.update();
  FLMotorYCurrent = FLYTarget.update();
  FLMotorZCurrent = FLZTarget.update();
  CLMotorXCurrent = CLXTarget.update();
  CLMotorYCurrent = CLYTarget.update();
  CLMotorZCurrent = CLZTarget.update();
  BLMotorXCurrent = BLXTarget.update();
  BLMotorYCurrent = BLYTarget.update();
  BLMotorZCurrent = BLZTarget.update();

  CoordinateToAngle('M', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('M', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('M', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('M', 4, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('M', 5, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('M', 6, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);

}

void TurnRight() {
  if (FRXTarget.isFinished() && FRYTarget.isFinished() && FRZTarget.isFinished()
  && CRXTarget.isFinished() && CRYTarget.isFinished() && CRZTarget.isFinished()
  && BRXTarget.isFinished() && BRYTarget.isFinished() && BRZTarget.isFinished()
  && FLXTarget.isFinished() && FLYTarget.isFinished() && FLZTarget.isFinished()
  && CLXTarget.isFinished() && CLYTarget.isFinished() && CLZTarget.isFinished()
  && BLXTarget.isFinished() && BLYTarget.isFinished() && BLZTarget.isFinished()) {

    commandStep = (commandStep + 1) % 4; // Loop back to the first step

    double xNew = walkfwdC[commandStep][0];
    double yNew = walkfwdF[commandStep][1];
    double zNew = walkfwdC[commandStep][2];
    
    FRXTarget.go(xNew, dur);
    FRYTarget.go(yNew, dur);
    FRZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = -walkfwdC[commandStep][2];
    
    CRXTarget.go(xNew, dur);
    CRYTarget.go(yNew, dur);
    CRZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdF[commandStep][1];
    zNew = walkfwdC[commandStep][2];

    BRXTarget.go(xNew, dur);
    BRYTarget.go(yNew, dur);
    BRZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = -walkfwdC[commandStep][2];

    FLXTarget.go(xNew, dur);
    FLYTarget.go(yNew, dur);
    FLZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdF[commandStep][1];
    zNew = walkfwdC[commandStep][2];

    CLXTarget.go(xNew, dur);
    CLYTarget.go(yNew, dur);
    CLZTarget.go(zNew, dur);

    xNew = walkfwdC[commandStep][0];
    yNew = walkfwdC[commandStep][1];
    zNew = -walkfwdC[commandStep][2];

    BLXTarget.go(xNew, dur);
    BLYTarget.go(yNew, dur);
    BLZTarget.go(zNew, dur);
  }

  FRMotorXCurrent = FRXTarget.update();
  FRMotorYCurrent = FRYTarget.update();
  FRMotorZCurrent = FRZTarget.update();
  CRMotorXCurrent = CRXTarget.update();
  CRMotorYCurrent = CRYTarget.update();
  CRMotorZCurrent = CRZTarget.update();
  BRMotorXCurrent = BRXTarget.update();
  BRMotorYCurrent = BRYTarget.update();
  BRMotorZCurrent = BRZTarget.update();
  FLMotorXCurrent = FLXTarget.update();
  FLMotorYCurrent = FLYTarget.update();
  FLMotorZCurrent = FLZTarget.update();
  CLMotorXCurrent = CLXTarget.update();
  CLMotorYCurrent = CLYTarget.update();
  CLMotorZCurrent = CLZTarget.update();
  BLMotorXCurrent = BLXTarget.update();
  BLMotorYCurrent = BLYTarget.update();
  BLMotorZCurrent = BLZTarget.update();

  CoordinateToAngle('M', 1, FRMotorXCurrent, FRMotorYCurrent, FRMotorZCurrent);
  CoordinateToAngle('M', 2, CRMotorXCurrent, CRMotorYCurrent, CRMotorZCurrent);
  CoordinateToAngle('M', 3, BRMotorXCurrent, BRMotorYCurrent, BRMotorZCurrent);
  CoordinateToAngle('M', 4, FLMotorXCurrent, FLMotorYCurrent, FLMotorZCurrent);
  CoordinateToAngle('M', 5, CLMotorXCurrent, CLMotorYCurrent, CLMotorZCurrent);
  CoordinateToAngle('M', 6, BLMotorXCurrent, BLMotorYCurrent, BLMotorZCurrent);

}

void loop() {

  // Read controller state
  PSXerror = psx.read(PSXdata);

  if(PSXerror == PSXERROR_SUCCESS) {
  Serial.print("succes");
  } else {
  Serial.print("No success reading data. Check connections and timing.");
  }
  //END OF DEBUG
  //-----------------------------

  double leftX = PSXdata.JoyLeftX;
  double leftY = PSXdata.JoyLeftY;
  double rightX = PSXdata.JoyRightX;
  bool xlever;

  Serial.println("Enter 'leftX (x)', 'leftY (y)', or 'rightX (z)': ");
    
  if (abs(leftX) > abs(leftY)) {
    xlever = true;
  } else {
    xlever = false;
  }

  if (leftY < 0 and !xlever) {
    defaultPosition = false;
     start = false;
    if (currentMovement != "Forward") {
      prepareMovement("Forward");
    } else {
      WalkForward();
      dur = duration / abs(leftY / 128);
    }
    if (switchMovement != "Forward") {
      switchMovement = "Forward";
      //globalPause();
    }


      
  } else if (leftY > 0 and !xlever) {
    defaultPosition = false;
    start = false;
    if (currentMovement != "Backward") {
      prepareMovement("Backward");
    } else {
      WalkBackward();
      dur = duration / abs(leftY / 128);
    }
    if (switchMovement != "Backward") {
      switchMovement = "Backward";
      //globalPause();
    }


  } else if (leftX > 0 and xlever) {
    defaultPosition = false;
    start = false;
    if (currentMovement != "Right") {
      prepareMovement("Right");
    } else {
      WalkRight();
      dur = duration / abs(leftX / 128);
    }
    if (switchMovement != "Right") {
      switchMovement = "Right";
      //globalPause();
    }



  } else if (leftX < 0 and xlever) {
    defaultPosition = false;
    start = false;
    Serial.print(leftX);
    Serial.print(xlever);
    if (currentMovement != "Left") {
      prepareMovement("Left");
    } else {
      WalkLeft();
    dur = duration / abs(leftX / 128);
    }
    if (switchMovement != "Left") {
      switchMovement = "Left";
      //globalPause();
    }


  } else if (rightX > 0) {
    defaultPosition = false;
    start = false;
    if (currentMovement != "Turn Right") {
      prepareMovement("Turn Right");
    } else {
      TurnRight();
      dur = duration / abs(rightX / 128);
    }

    if (switchMovement != "Turn Right") {
      switchMovement = "Turn Right";
      //globalPause();
    }


  } else if (rightX < 0) {
    defaultPosition = false;
    start = false;
    if (currentMovement != "Turn Left") {
      prepareMovement("Turn Left");
    } else {
      TurnLeft();
      dur = duration / abs(rightX / 128);
    }
    
    if (switchMovement != "Turn Left") {
      switchMovement = "Turn Left";
      //globalPause();
    }


  } else if (!defaultPosition){
    dur = 6000;
    DefaultPosition();
    if (switchMovement != "Idle") {
      //globalPause();
      switchMovement = "Idle";
    }
  }

}

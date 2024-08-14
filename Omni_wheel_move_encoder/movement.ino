#include "CytronMotorDriver.h"
#define PI 3.1415926535897932384626433832795
const int MOT1_ENC1 = 18; 
const int MOT1_DIRECT = 17;
const int MOT2_ENC1 = 19;
const int MOT2_DIRECT = 16;
const int MOT3_ENC1 = 2;
const int MOT3_DIRECT = 4;
const int MOT4_ENC1 = 3;
const int MOT4_DIRECT = 5;
const double DIST_TOLERANCE =0.03;
const double ENCODER_RESOLUTION=268.83;
volatile float MOT1_ENC1_TICKS = 0;
volatile float MOT2_ENC1_TICKS = 0;
volatile float MOT3_ENC1_TICKS = 0;
volatile float MOT4_ENC1_TICKS = 0;

const double WHEEL_RADIUS=0.076;
const double R=0.076; //in meters
const int MAX_SPEED=25;
const int Diag_Len = 0.235;

volatile float en_cnt1 = 0;
volatile float en_cnt2 = 0;
volatile float en_cnt3 = 0;
volatile float en_cnt4 = 0;
// Configure the motor encoders.
void reset(){
   en_cnt1 = 0;
   en_cnt2 = 0;
   en_cnt3 = 0;
   en_cnt4 = 0;
}

// Configure the motor driver.
CytronMD motor_1(PWM_DIR, 6,7);  // PWM 1 = Pin 6, DIR 1 = Pin 7.
CytronMD motor_2(PWM_DIR, 8,9);  // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor_3(PWM_DIR, 10,11);  // PWM 1 = Pin 10, DIR 1 = Pin 11.
CytronMD motor_4(PWM_DIR, 12,22);  // PWM 2 = Pin 12, DIR 2 = Pin 22.


void ISR_A() {
  if (digitalRead(MOT1_ENC1) == digitalRead(MOT1_DIRECT)) {
    MOT1_ENC1_TICKS++;
  } else {
    MOT1_ENC1_TICKS--;
  }
}

void ISR_B() {
  if (digitalRead(MOT2_ENC1) == digitalRead(MOT2_DIRECT)) {
    MOT2_ENC1_TICKS++;
  } else {
    MOT2_ENC1_TICKS--;
  }
}
void ISR_C() {
  if (digitalRead(MOT3_ENC1) == digitalRead(MOT3_DIRECT)) {
    MOT3_ENC1_TICKS++;
  } else {
    MOT3_ENC1_TICKS--;
  }
}
void ISR_D() {
  if (digitalRead(MOT4_ENC1) == digitalRead(MOT4_DIRECT)) {
    MOT4_ENC1_TICKS++;
  } else {
    MOT4_ENC1_TICKS--;
  }
}

// double forward1(double D){
//   // D is desired distance to be travelled
//   //motor 1
//   en_cnt2= -(MOT2_ENC1_TICKS/268.83);
//   double rot_2= D/(2*PI*R); //required encoder count for motor1
//   if (en_cnt2 <= rot_2)
//   {
//     motor_2.setSpeed(-MAX_SPEED);
//     Serial.println("motor2 is running");
//   }
//   else
//   {
//     motor_2.setSpeed(0);
//     Serial.println("motor2 is not running");
//   }

//   //motor 2
//   en_cnt4=( MOT4_ENC1_TICKS/268.83);
//   double rot_4= D/(2*PI*R); //required encoder count for motor1
//   if (en_cnt4 <= rot_4)
//   {
//     motor_4.setSpeed(MAX_SPEED);
//     Serial.println("motor4 is running");
//   }
//   else
//   {
//     motor_4.setSpeed(0);
//     Serial.println("motor4 is not running");
//   }
// }

double forward2(double D){
  // D is desired distance to be travelled
  //motor 1
  en_cnt1= -(MOT1_ENC1_TICKS/268.83);
  double rot_1= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt1 < rot_1)
  {
    motor_1.setSpeed(-MAX_SPEED);
    Serial.println("motor1 is running");
  }
  else
  {
    motor_1.setSpeed(0);
    Serial.println("motor1 is not running");
  }

  //motor 2
  en_cnt3=( MOT3_ENC1_TICKS/268.83);
  double rot_3= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt3 != rot_3)
  {
    motor_3.setSpeed(MAX_SPEED);
    Serial.println("motor3 is running");
  }
  else
  {
    motor_3.setSpeed(0);
    Serial.println("motor3 is not running");
  }
}

double check_mot1(double D){
  // D is desired distance to be travelled
  //motor 1
  
  en_cnt1= -(MOT1_ENC1_TICKS/268.83);
  double rot_1= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt1 <= rot_1)
  {
    motor_1.setSpeed(-MAX_SPEED);
    Serial.println("motor1 is running");
  }
  else
  {
    motor_1.setSpeed(0);
    Serial.println("motor1 is not running");
  }
  Serial.println(en_cnt1);
  Serial.println(rot_1);
}

double check_mot2(double D){
  // D is desired distance to be travelled
  //motor 1
  
  en_cnt2= -(MOT2_ENC1_TICKS/268.83);
  double rot_2= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt2 <= rot_2)
  {
    motor_2.setSpeed(-MAX_SPEED);
    Serial.println("motor2 is running");
  }
  else
  {
    motor_2.setSpeed(0);
    Serial.println("motor2 is not running");
  }
  Serial.println(en_cnt2);
  Serial.println(rot_2);
}

double check_mot3(double D){
  // D is desired distance to be travelled
  //motor 1
  
  en_cnt3= -(MOT3_ENC1_TICKS/268.83);
  double rot_3= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt3 <= rot_3)
  {
    motor_3.setSpeed(MAX_SPEED);
    Serial.println("motor3 is running");
  }
  else
  {
    motor_3.setSpeed(0);
    Serial.println("motor3 is not running");
  }
  Serial.println(en_cnt3);
  Serial.println(rot_3);
}

double check_mot4(double D){
  // D is desired distance to be travelled
  //motor 1
  
  en_cnt4= (MOT4_ENC1_TICKS/268.83);
  double rot_4= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt4 <= rot_4)
  {
    motor_4.setSpeed(-MAX_SPEED);
    Serial.println("motor4 is running");
  }
  else
  {
    motor_4.setSpeed(0);
    Serial.println("motor4 is not running");
  }
  Serial.println(en_cnt4);
  Serial.println(rot_4);
}

void forward3(double D1) {
  
  double D = (D1 - DIST_TOLERANCE) / sqrt(2);

  double rot_1 = D / (2 * PI * WHEEL_RADIUS);
  double rot_2 = D / (2 * PI * WHEEL_RADIUS);
  double rot_3 = D / (2 * PI * WHEEL_RADIUS);
  double rot_4 = D / (2 * PI * WHEEL_RADIUS);
  
  while (en_cnt1 <= rot_1 || en_cnt2 <= rot_2 || en_cnt3 <= rot_3 || en_cnt4 <= rot_4) {
    if (en_cnt1 <= rot_1) {
      motor_1.setSpeed(-MAX_SPEED);
      Serial.println("motor1 is running");
      
    } else {
      motor_1.setSpeed(0);
      Serial.println("motor1 is not running");
    }

    if (en_cnt2 <= rot_2) {
      motor_2.setSpeed(-MAX_SPEED);
      Serial.println("motor2 is running");
      
    } else {
      motor_2.setSpeed(0);
      Serial.println("motor2 is not running");
    }

    if (en_cnt3 <= rot_3) {
      motor_3.setSpeed(MAX_SPEED);
      Serial.println("motor3 is running");
      
    } else {
      motor_3.setSpeed(0);
      Serial.println("motor3 is not running");
    }

    if (en_cnt4 <= rot_4) {
      motor_4.setSpeed(MAX_SPEED);
      Serial.println("motor4 is running");
     
    } else {
      motor_4.setSpeed(0);
      Serial.println("motor4 is not running");
    }
     en_cnt1 = -(MOT1_ENC1_TICKS / ENCODER_RESOLUTION);
     en_cnt2 = (MOT2_ENC1_TICKS / ENCODER_RESOLUTION);
     en_cnt3 = (MOT3_ENC1_TICKS / ENCODER_RESOLUTION);
     en_cnt4 = -(MOT4_ENC1_TICKS / ENCODER_RESOLUTION);
    Serial.println(en_cnt1);
    Serial.println(en_cnt2);
    Serial.println(en_cnt3);
    Serial.println(en_cnt4);
    Serial.println(rot_4);
  }
  motor_1.setSpeed(0);
  motor_2.setSpeed(0);
  motor_3.setSpeed(0);
  motor_4.setSpeed(0);
}

double backward(double D1){
  // D is desired distance to be travelled
  //motor 1
  double D = (D1-0.03)/sqrt(2);
  //double D = D1/2;
  en_cnt1= (MOT1_ENC1_TICKS/268.83);
  double rot_1= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt1 <= rot_1)
  {
    motor_1.setSpeed(MAX_SPEED);
    Serial.println("motor1 is running");
  }
  else
  {
    motor_1.setSpeed(0);
    Serial.println("motor1 is not running");
  }

  //motor 2
  en_cnt2=-(MOT2_ENC1_TICKS/268.83);
  double rot_2= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt2 <= rot_2)
  {
    motor_2.setSpeed(MAX_SPEED);
    Serial.println("motor2 is running");
  }
  else
  {
    motor_2.setSpeed(0);
    Serial.println("motor2 is not running");
  }

  en_cnt3=-(MOT3_ENC1_TICKS/268.83);
  double rot_3= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt3 <= rot_3)
  {
    motor_3.setSpeed(-MAX_SPEED);
    Serial.println("motor3 is running");
  }
  else
  {
    motor_3.setSpeed(0);
    Serial.println("motor3 is not running");
  }

    en_cnt4=(MOT4_ENC1_TICKS/268.83);
  double rot_4= D/(2*PI*R); //required encoder count for motor1
  if (en_cnt4 <= rot_4)
  {
    motor_4.setSpeed(-MAX_SPEED);
    Serial.println("motor4 is running");
  }
  else
  {
    motor_4.setSpeed(0);
    Serial.println("motor4 is not running");
  }
  Serial.println(en_cnt1);
  Serial.println(en_cnt2);
  Serial.println(en_cnt3);
  Serial.println(en_cnt4);
  Serial.println(rot_4);

}

void backward2(double D1) {
  
  double D = (D1 - DIST_TOLERANCE) / sqrt(2);

  double rot_1 = D / (2 * PI * WHEEL_RADIUS);
  double rot_2 = D / (2 * PI * WHEEL_RADIUS);
  double rot_3 = D / (2 * PI * WHEEL_RADIUS);
  double rot_4 = D / (2 * PI * WHEEL_RADIUS);
  
  while (en_cnt1 <= rot_1 || en_cnt2 <= rot_2 || en_cnt3 <= rot_3 || en_cnt4 <= rot_4) {
    Serial.print("distance: ");
    Serial.println(D);
    if (en_cnt1 <= rot_1) {
      motor_1.setSpeed(MAX_SPEED);
      Serial.println("motor1 is running");
      
    } else {
      motor_1.setSpeed(0);
      Serial.println("motor1 is not running");
    }

    if (en_cnt2 <= rot_2) {
      motor_2.setSpeed(MAX_SPEED);
      Serial.println("motor2 is running");
      
    } else {
      motor_2.setSpeed(0);
      Serial.println("motor2 is not running");
    }

    if (en_cnt3 <= rot_3) {
      motor_3.setSpeed(-MAX_SPEED);
      Serial.println("motor3 is running");
      
    } else {
      motor_3.setSpeed(0);
      Serial.println("motor3 is not running");
    }

    if (en_cnt4 <= rot_4) {
      motor_4.setSpeed(-MAX_SPEED);
      Serial.println("motor4 is running");
     
    } else {
      motor_4.setSpeed(0);
      Serial.println("motor4 is not running");
    }
     en_cnt1 = (MOT1_ENC1_TICKS / ENCODER_RESOLUTION);
     en_cnt2 = -(MOT2_ENC1_TICKS / ENCODER_RESOLUTION);
     en_cnt3 = -(MOT3_ENC1_TICKS / ENCODER_RESOLUTION);
     en_cnt4 = (MOT4_ENC1_TICKS / ENCODER_RESOLUTION);
    Serial.println(en_cnt1);
    Serial.println(en_cnt2);
    Serial.println(en_cnt3);
    Serial.println(en_cnt4);
    Serial.println(rot_4);
  }
  motor_1.setSpeed(0);
  motor_2.setSpeed(0);
  motor_3.setSpeed(0);
  motor_4.setSpeed(0);
}


void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(MOT1_ENC1), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOT2_ENC1), ISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOT3_ENC1), ISR_C, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOT4_ENC1), ISR_D, CHANGE);
}


void loop() {

  //motor
  
  double D = 0.6; //desired distance to be travelled
  forward3(D);
  reset();
  delay(1000);
  //
  backward2(D);
  reset();
  delay(1000);
  //check_mot1(0.4775);

}

#include <Servo.h>
#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());


volatile int roll_channel_1, pitch_channel_2, throttle_channel_3, yaw_channel_4, control_channel_5, control_channel_6;
int start, fuzzy_yaw_out_tilt, pitch_angle1, pitch_angle2, yaw_angle1, yaw_angle2, cal_int, mode, currentTime, timeslap, sleep;;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;
double roll_sensor, pitch_sensor, yaw_sensor, roll_setpoint, pitch_setpoint, yaw_setpoint, forward_speed, roll_manual1, roll_manual2;
double fuzzy_roll_out, fuzzy_pitch_out, fuzzy_yaw_out, esc1_out, esc2_out, esc3_out, esc4_out, servo1_out, servo2_out, servo3_out, servo4_out;
double roll_error = 0, pitch_error = 0, yaw_error = 0, roll_out, pitch_out, yaw_out, throttle, previous_error_roll = 0, previous_error_pitch = 0, previous_error_yaw = 0, roll_error_change, pitch_error_change, yaw_error_change;
double heading, Pitch_combined, Roll_combined, Heading_combined, dt = 0.02, gyro_pitch, gyro_roll, gyro_yaw, gyro_pitch_input, gyro_roll_input, gyro_yaw_input, roll, pitch;
boolean reverseMode;
Servo esc1, esc2, esc3, esc4, servo1, servo2, servo3, servo4; // Use servo library to declare a servo object for esc's and servo's.
double pitchManipulation = 0;
double yawManipulation = 0;
double rollManipulation = 0;

double pitchThrustUp = 0;
double pitchThrustDown = 0;
double yawThrustUp = 0;
double yawThrustDown = 0;
double rollThrustUp  = 0;
double rollThrustDown = 0;

Fuzzy* fuzzy = new Fuzzy();
//-------------------------Roll-------------------------------------
FuzzySet* NUltra_Roll_E = new FuzzySet(-80, -17 , -17, - 14);
FuzzySet* NMega_Roll_E = new FuzzySet(-17, -14, -14, -11);
FuzzySet* NSB_Roll_E = new FuzzySet(-14, -11, -11, -8);
FuzzySet* NB_Roll_E = new FuzzySet(-11, -8, -8, -5);
FuzzySet* NM_Roll_E = new FuzzySet(-8, -5, -5, -2);
FuzzySet* NS_Roll_E = new FuzzySet(-5, -2, -2, 0);
FuzzySet* Z_Roll_E = new FuzzySet(-2, 0, 0, 2);
FuzzySet* PS_Roll_E = new FuzzySet(0, 2, 2, 5);
FuzzySet* PM_Roll_E = new FuzzySet(2, 5, 5, 8);
FuzzySet* PB_Roll_E = new FuzzySet(5, 8, 8, 11);
FuzzySet* PSB_Roll_E = new FuzzySet(8, 11, 11, 14);
FuzzySet* PMega_Roll_E = new FuzzySet(11, 14, 14, 17);
FuzzySet* PUltra_Roll_E = new FuzzySet(14, 17, 17, 80);
FuzzySet* Z_Roll_EC = new FuzzySet(-2, 0, 0, 2);


FuzzySet* Z_SpeedRoll = new FuzzySet(0, 100, 100 , 150 );
FuzzySet* PS_SpeedRoll = new FuzzySet(100, 150, 150, 250);
FuzzySet* PM_SpeedRoll = new FuzzySet(150, 250, 250, 350);
FuzzySet* PB_SpeedRoll = new FuzzySet(250, 350, 350, 450);
FuzzySet* PSB_SpeedRoll = new FuzzySet(350, 450, 450, 550);
FuzzySet* PMega_SpeedRoll = new FuzzySet(450, 550, 550, 800 );
FuzzySet* PUltra_SpeedRoll = new FuzzySet(650, 750, 750, 1000);

//-------------------------Pitch-------------------------------------
FuzzySet* NUltra_Pitch_E = new FuzzySet(-80, -17 , -17, - 14);
FuzzySet* NMega_Pitch_E = new FuzzySet(-17, -14, -14, -11);
FuzzySet* NSB_Pitch_E = new FuzzySet(-14, -11, -11, -8);
FuzzySet* NB_Pitch_E = new FuzzySet(-11, -8, -8, -5);
FuzzySet* NM_Pitch_E = new FuzzySet(-8, -5, -5, -2);
FuzzySet* NS_Pitch_E = new FuzzySet(-5, -2, -2, 0);
FuzzySet* Z_Pitch_E = new FuzzySet(-2, 0, 0, 2);
FuzzySet* PS_Pitch_E = new FuzzySet(0, 2, 2, 5);
FuzzySet* PM_Pitch_E = new FuzzySet(2, 5, 5, 8);
FuzzySet* PB_Pitch_E = new FuzzySet(5, 8, 8, 11);
FuzzySet* PSB_Pitch_E = new FuzzySet(8, 11, 11, 14);
FuzzySet* PMega_Pitch_E = new FuzzySet(11, 14, 14, 17);
FuzzySet* PUltra_Pitch_E = new FuzzySet(14, 17, 17, 80);
FuzzySet* Z_Pitch_EC = new FuzzySet(-2, 0, 0, 2);


FuzzySet* Z_SpeedPitch = new FuzzySet(0, 100, 100 , 150);
FuzzySet* PS_SpeedPitch = new FuzzySet(100, 150, 150, 250);
FuzzySet* PM_SpeedPitch = new FuzzySet(150, 250, 250, 350);
FuzzySet* PB_SpeedPitch = new FuzzySet(250, 350, 350, 450);
FuzzySet* PSB_SpeedPitch = new FuzzySet(350, 450, 450, 550);
FuzzySet* PMega_SpeedPitch = new FuzzySet(450, 550, 550, 800);
FuzzySet* PUltra_SpeedPitch = new FuzzySet(650, 750, 750, 1000);

//--------------------END OF PITCH -------------------------------

//--------------------Start of Yaw-------------
FuzzySet* NUltra_Yaw_E = new FuzzySet(-180, -30, -30, -25);
FuzzySet* NMega_Yaw_E = new FuzzySet(-30, -25, -25, -20);
FuzzySet* NSB_Yaw_E = new FuzzySet(-25, -20, -20, -15);
FuzzySet* NB_Yaw_E = new FuzzySet(-20, -15, -15, -10);
FuzzySet* NM_Yaw_E = new FuzzySet(-15, -10, -10, -5);
FuzzySet* NS_Yaw_E = new FuzzySet(-10, -5, -5, 0);
FuzzySet* Z_Yaw_E = new FuzzySet(-5, 0, 0, 5);
FuzzySet* PS_Yaw_E = new FuzzySet(0, 5, 5, 10);
FuzzySet* PM_Yaw_E = new FuzzySet(5, 10, 10, 15);
FuzzySet* PB_Yaw_E = new FuzzySet(10, 15, 15, 20);
FuzzySet* PSB_Yaw_E = new FuzzySet(15, 20, 20, 25);
FuzzySet* PMega_Yaw_E = new FuzzySet(20, 25, 25, 30);
FuzzySet* PUltra_Yaw_E = new FuzzySet(25, 30, 30, 180);
FuzzySet* Z_Yaw_EC = new FuzzySet(-5, 0, 0, 5);



//esc control
FuzzySet* NUltra_SpeedYaw = new FuzzySet(-1000, -750, -750, -650);
FuzzySet* NMega_SpeedYaw = new FuzzySet(-800, -550, -550, -450);
FuzzySet* NSB_SpeedYaw = new FuzzySet(-550, -450, -450, -350);
FuzzySet* NB_SpeedYaw = new FuzzySet(-450, -350, -350, -250);
FuzzySet* NM_SpeedYaw = new FuzzySet(-350, -250, -250, -150);
FuzzySet* NS_SpeedYaw = new FuzzySet(-100, -50, -50, 0);
FuzzySet* Z_SpeedYaw = new FuzzySet(-50, 0, 0 , 50);
FuzzySet* PS_SpeedYaw = new FuzzySet(0, 50, 50, 150);
FuzzySet* PM_SpeedYaw = new FuzzySet(100, 250, 250, 350);
FuzzySet* PB_SpeedYaw = new FuzzySet(250, 350, 350, 450);
FuzzySet* PSB_SpeedYaw = new FuzzySet(350, 450, 450, 550);
FuzzySet* PMega_SpeedYaw = new FuzzySet(450, 550, 550, 800);
FuzzySet* PUltra_SpeedYaw = new FuzzySet(650, 750, 750, 1000);

//Servo Tilting
FuzzySet* NUltra_tilt = new FuzzySet(0, 0, 0, 0);
FuzzySet* NMega_tilt = new FuzzySet(0, 10, 10, 30);
FuzzySet* NSB_tilt = new FuzzySet(10, 30, 30, 50);
FuzzySet* NB_tilt = new FuzzySet(30, 50, 50, 70);
FuzzySet* NM_tilt = new FuzzySet(50, 70, 70, 80);
FuzzySet* NS_tilt = new FuzzySet(90, 90, 90, 90);
FuzzySet* Z_tilt = new FuzzySet(90, 90, 90, 90);
FuzzySet* PS_tilt = new FuzzySet(90, 90, 90, 90);
FuzzySet* PM_tilt = new FuzzySet(80, 110, 110, 130);
FuzzySet* PB_tilt = new FuzzySet(110, 130, 130, 150);
FuzzySet* PSB_tilt = new FuzzySet(130, 150, 150, 170);
FuzzySet* PMega_tilt = new FuzzySet(150, 170, 170, 180);
FuzzySet* PUltra_tilt = new FuzzySet(180, 180, 180, 180);


void setup(void) {
  initSensors();
  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(8);
  esc4.attach(10);
  servo1.attach(2);
  servo2.attach(6);
  servo3.attach(9);
  servo4.attach(11);
  pinMode(12, OUTPUT);

  attachInterrupt(52, calcRoll, CHANGE); //call ISR if pin 52 is change
  attachInterrupt(50, calcPitch, CHANGE); //call ISR if pin 50 is change
  attachInterrupt(46, calcYaw, CHANGE); //call ISR if pin 46 is change
  attachInterrupt(48, calcThrottle, CHANGE); //call ISR if pin 48 is change
  attachInterrupt(44, calcReverse, CHANGE); //call ISR if pin 44 is change

  //---------start of roll ------------------------------
  //fuzzy input for roll_error
  FuzzyInput* roll_error = new FuzzyInput(1);
  roll_error->addFuzzySet(NUltra_Roll_E);
  roll_error->addFuzzySet(NMega_Roll_E);
  roll_error->addFuzzySet(NSB_Roll_E);
  roll_error->addFuzzySet(NB_Roll_E);
  roll_error->addFuzzySet(NM_Roll_E);
  roll_error->addFuzzySet(NS_Roll_E);
  roll_error->addFuzzySet(Z_Roll_E);
  roll_error->addFuzzySet(PS_Roll_E);
  roll_error->addFuzzySet(PM_Roll_E);
  roll_error->addFuzzySet(PB_Roll_E);
  roll_error->addFuzzySet(PSB_Roll_E);
  roll_error->addFuzzySet(PMega_Roll_E);
  roll_error->addFuzzySet(PUltra_Roll_E);
  fuzzy->addFuzzyInput(roll_error);

  //fuzzy input for roll_error_change
  FuzzyInput* roll_error_change = new FuzzyInput(2);
  roll_error_change->addFuzzySet(Z_Roll_EC);
  fuzzy->addFuzzyInput(roll_error_change);

  //fuzzy Output for Roll

  FuzzyOutput* roll_Out = new FuzzyOutput(1);
  roll_Out->addFuzzySet(Z_SpeedRoll);
  roll_Out->addFuzzySet(PS_SpeedRoll);
  roll_Out->addFuzzySet(PM_SpeedRoll);
  roll_Out->addFuzzySet(PB_SpeedRoll);
  roll_Out->addFuzzySet(PSB_SpeedRoll);
  roll_Out->addFuzzySet(PMega_SpeedRoll);
  roll_Out->addFuzzySet(PUltra_SpeedRoll);
  fuzzy->addFuzzyOutput(roll_Out);

  FuzzyRuleAntecedent* ifRoll_EC_IsZ = new FuzzyRuleAntecedent();
  ifRoll_EC_IsZ->joinSingle(Z_Roll_EC);
  //-------------------------------------Rule01-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsNUltra = new FuzzyRuleAntecedent();
  ifRoll_E_IsNUltra->joinSingle(NUltra_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_NUltra_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_NUltra_And_ec_Z->joinWithAND(ifRoll_E_IsNUltra, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PUltra_Speed = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PUltra_Speed->addOutput(PUltra_SpeedRoll);
  FuzzyRule* fuzzyRule01 = new FuzzyRule(1, ifRoll_e_NUltra_And_ec_Z, Then_Roll_Speed_PUltra_Speed);
  fuzzy->addFuzzyRule(fuzzyRule01);
  //---------------------------------------------------------------------------------------//
  //-------------------------------------Rule02-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsNMega = new FuzzyRuleAntecedent();
  ifRoll_E_IsNMega->joinSingle(NMega_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_NMega_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_NMega_And_ec_Z->joinWithAND(ifRoll_E_IsNMega, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PMega_Speed = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PMega_Speed->addOutput(PMega_SpeedRoll);
  FuzzyRule* fuzzyRule02 = new FuzzyRule(2, ifRoll_e_NMega_And_ec_Z, Then_Roll_Speed_PMega_Speed);

  fuzzy->addFuzzyRule(fuzzyRule02);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule03-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsNSB = new FuzzyRuleAntecedent();
  ifRoll_E_IsNSB->joinSingle(NSB_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_NSB_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_NSB_And_ec_Z->joinWithAND(ifRoll_E_IsNSB, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PSB_Speed = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PSB_Speed->addOutput(PSB_SpeedRoll);
  FuzzyRule* fuzzyRule03 = new FuzzyRule(3, ifRoll_e_NSB_And_ec_Z, Then_Roll_Speed_PSB_Speed);
  fuzzy->addFuzzyRule(fuzzyRule03);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule04-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsNB = new FuzzyRuleAntecedent();
  ifRoll_E_IsNB->joinSingle(NB_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_NB_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_NB_And_ec_Z->joinWithAND(ifRoll_E_IsNB, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PB_Speed = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PB_Speed->addOutput(PB_SpeedRoll);
  FuzzyRule* fuzzyRule04 = new FuzzyRule(4, ifRoll_e_NB_And_ec_Z, Then_Roll_Speed_PB_Speed);
  fuzzy->addFuzzyRule(fuzzyRule04);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule05-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsNM = new FuzzyRuleAntecedent();
  ifRoll_E_IsNM->joinSingle(NM_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_NM_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_NM_And_ec_Z->joinWithAND(ifRoll_E_IsNM, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PM_Speed = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PM_Speed->addOutput(PM_SpeedRoll);
  FuzzyRule* fuzzyRule05 = new FuzzyRule(5, ifRoll_e_NM_And_ec_Z, Then_Roll_Speed_PM_Speed);
  fuzzy->addFuzzyRule(fuzzyRule05);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule06-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsNS = new FuzzyRuleAntecedent();
  ifRoll_E_IsNS->joinSingle(NS_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_NS_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_NS_And_ec_Z->joinWithAND(ifRoll_E_IsNS, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PS_Speed = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PS_Speed->addOutput(PS_SpeedRoll);
  FuzzyRule* fuzzyRule06 = new FuzzyRule(6, ifRoll_e_NS_And_ec_Z, Then_Roll_Speed_PS_Speed);
  fuzzy->addFuzzyRule(fuzzyRule06);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule07-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsZ = new FuzzyRuleAntecedent();
  ifRoll_E_IsZ->joinSingle(Z_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_Z_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_Z_And_ec_Z->joinWithAND(ifRoll_E_IsZ, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_Z_Speed = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_Z_Speed->addOutput(Z_SpeedRoll);
  FuzzyRule* fuzzyRule07 = new FuzzyRule(7, ifRoll_e_Z_And_ec_Z, Then_Roll_Speed_Z_Speed);
  fuzzy->addFuzzyRule(fuzzyRule07);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule08-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsPUltra = new FuzzyRuleAntecedent();
  ifRoll_E_IsPUltra->joinSingle(PUltra_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_PUltra_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_PUltra_And_ec_Z->joinWithAND(ifRoll_E_IsPUltra, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PUltra_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PUltra_Speed1->addOutput(PUltra_SpeedRoll);
  FuzzyRule* fuzzyRule8 = new FuzzyRule(8, ifRoll_e_PUltra_And_ec_Z, Then_Roll_Speed_PUltra_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule8);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule09-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsPMega = new FuzzyRuleAntecedent();
  ifRoll_E_IsPMega->joinSingle(PMega_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_PMega_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_PMega_And_ec_Z->joinWithAND(ifRoll_E_IsPMega, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PMega_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PMega_Speed1->addOutput(PMega_SpeedRoll);
  FuzzyRule* fuzzyRule9 = new FuzzyRule(9, ifRoll_e_PMega_And_ec_Z, Then_Roll_Speed_PMega_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule9);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule10-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsPSB = new FuzzyRuleAntecedent();
  ifRoll_E_IsPSB->joinSingle(PSB_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_PSB_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_PSB_And_ec_Z->joinWithAND(ifRoll_E_IsPSB, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PSB_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PSB_Speed1->addOutput(PSB_SpeedRoll);
  FuzzyRule* fuzzyRule10 = new FuzzyRule(10, ifRoll_e_PSB_And_ec_Z, Then_Roll_Speed_PSB_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule10);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule011-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsPB = new FuzzyRuleAntecedent();
  ifRoll_E_IsPB->joinSingle(PB_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_PB_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_PB_And_ec_Z->joinWithAND(ifRoll_E_IsPB, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PB_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PB_Speed1->addOutput(PB_SpeedRoll);
  FuzzyRule* fuzzyRule11 = new FuzzyRule(11, ifRoll_e_PB_And_ec_Z, Then_Roll_Speed_PB_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule11);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule12-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsPM = new FuzzyRuleAntecedent();
  ifRoll_E_IsPM->joinSingle(PM_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_PM_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_PM_And_ec_Z->joinWithAND(ifRoll_E_IsPM, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PM_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PM_Speed1->addOutput(PM_SpeedRoll);
  FuzzyRule* fuzzyRule12 = new FuzzyRule(12, ifRoll_e_PM_And_ec_Z, Then_Roll_Speed_PM_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule12);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule13-----------------------------------------
  FuzzyRuleAntecedent* ifRoll_E_IsPS = new FuzzyRuleAntecedent();
  ifRoll_E_IsPS->joinSingle(PS_Roll_E);

  FuzzyRuleAntecedent* ifRoll_e_PS_And_ec_Z = new FuzzyRuleAntecedent();
  ifRoll_e_PS_And_ec_Z->joinWithAND(ifRoll_E_IsPS, ifRoll_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Roll_Speed_PS_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Roll_Speed_PS_Speed1->addOutput(PS_SpeedRoll);
  FuzzyRule* fuzzyRule13 = new FuzzyRule(13, ifRoll_e_PS_And_ec_Z, Then_Roll_Speed_PS_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule13);
  //---------------------------------------------------------------------------------------//


  //---------start of Pitch ------------------------------
  //fuzzy input for pitch_error
  FuzzyInput* pitch_error = new FuzzyInput(3);
  pitch_error->addFuzzySet(NUltra_Pitch_E);
  pitch_error->addFuzzySet(NMega_Pitch_E);
  pitch_error->addFuzzySet(NSB_Pitch_E);
  pitch_error->addFuzzySet(NB_Pitch_E);
  pitch_error->addFuzzySet(NM_Pitch_E);
  pitch_error->addFuzzySet(NS_Pitch_E);
  pitch_error->addFuzzySet(Z_Pitch_E);
  pitch_error->addFuzzySet(PS_Pitch_E);
  pitch_error->addFuzzySet(PM_Pitch_E);
  pitch_error->addFuzzySet(PB_Pitch_E);
  pitch_error->addFuzzySet(PSB_Pitch_E);
  pitch_error->addFuzzySet(PMega_Pitch_E);
  pitch_error->addFuzzySet(PUltra_Pitch_E);
  fuzzy->addFuzzyInput(pitch_error);

  //fuzzy input for roll_error_change
  FuzzyInput* pitch_error_change = new FuzzyInput(4);
  pitch_error_change->addFuzzySet(Z_Pitch_EC);
  fuzzy->addFuzzyInput(pitch_error_change);

  //fuzzy Output for Pitch
  FuzzyOutput* pitch_Out = new FuzzyOutput(2);
  pitch_Out->addFuzzySet(Z_SpeedPitch);
  pitch_Out->addFuzzySet(PS_SpeedPitch);
  pitch_Out->addFuzzySet(PM_SpeedPitch);
  pitch_Out->addFuzzySet(PB_SpeedPitch);
  pitch_Out->addFuzzySet(PSB_SpeedPitch);
  pitch_Out->addFuzzySet(PMega_SpeedPitch);
  pitch_Out->addFuzzySet(PUltra_SpeedPitch);
  fuzzy->addFuzzyOutput(pitch_Out);


  //-------------------------------------Rule14-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsNUltra = new FuzzyRuleAntecedent();
  ifPitch_E_IsNUltra->joinSingle(NUltra_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_EC_IsZ = new FuzzyRuleAntecedent();
  ifPitch_EC_IsZ->joinSingle(Z_Pitch_EC);

  FuzzyRuleAntecedent* ifPitch_e_NUltra_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_NUltra_And_ec_Z->joinWithAND(ifPitch_E_IsNUltra, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PUltra_Speed = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PUltra_Speed->addOutput(PUltra_SpeedPitch);
  FuzzyRule* fuzzyRule14 = new FuzzyRule(14, ifPitch_e_NUltra_And_ec_Z, Then_Pitch_Speed_PUltra_Speed);
  fuzzy->addFuzzyRule(fuzzyRule14);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule15-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsNMega = new FuzzyRuleAntecedent();
  ifPitch_E_IsNMega->joinSingle(NMega_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_NMega_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_NMega_And_ec_Z->joinWithAND(ifPitch_E_IsNMega, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PMega_Speed = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PMega_Speed->addOutput(PMega_SpeedPitch);
  FuzzyRule* fuzzyRule15 = new FuzzyRule(15, ifPitch_e_NMega_And_ec_Z, Then_Pitch_Speed_PMega_Speed);
  fuzzy->addFuzzyRule(fuzzyRule15);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule16-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsNSB = new FuzzyRuleAntecedent();
  ifPitch_E_IsNSB->joinSingle(NSB_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_NSB_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_NSB_And_ec_Z->joinWithAND(ifPitch_E_IsNSB, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PSB_Speed = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PSB_Speed->addOutput(PSB_SpeedPitch);
  FuzzyRule* fuzzyRule16 = new FuzzyRule(16, ifPitch_e_NSB_And_ec_Z, Then_Pitch_Speed_PSB_Speed);
  fuzzy->addFuzzyRule(fuzzyRule16);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule17-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsNB = new FuzzyRuleAntecedent();
  ifPitch_E_IsNB->joinSingle(NB_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_NB_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_NB_And_ec_Z->joinWithAND(ifPitch_E_IsNB, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PB_Speed = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PB_Speed->addOutput(PB_SpeedPitch);
  FuzzyRule* fuzzyRule17 = new FuzzyRule(17, ifPitch_e_NB_And_ec_Z, Then_Pitch_Speed_PB_Speed);
  fuzzy->addFuzzyRule(fuzzyRule17);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule18-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsNM = new FuzzyRuleAntecedent();
  ifPitch_E_IsNM->joinSingle(NM_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_NM_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_NM_And_ec_Z->joinWithAND(ifPitch_E_IsNM, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PM_Speed = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PM_Speed->addOutput(PM_SpeedPitch);
  FuzzyRule* fuzzyRule18 = new FuzzyRule(18, ifPitch_e_NM_And_ec_Z, Then_Pitch_Speed_PM_Speed);
  fuzzy->addFuzzyRule(fuzzyRule18);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule19-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsNS = new FuzzyRuleAntecedent();
  ifPitch_E_IsNS->joinSingle(NS_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_NS_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_NS_And_ec_Z->joinWithAND(ifPitch_E_IsNS, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PS_Speed = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PS_Speed->addOutput(PS_SpeedPitch);
  FuzzyRule* fuzzyRule19 = new FuzzyRule(19, ifPitch_e_NS_And_ec_Z, Then_Pitch_Speed_PS_Speed);
  fuzzy->addFuzzyRule(fuzzyRule19);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule20-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsZ = new FuzzyRuleAntecedent();
  ifPitch_E_IsZ->joinSingle(Z_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_Z_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_Z_And_ec_Z->joinWithAND(ifPitch_E_IsZ, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_Z_Speed = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_Z_Speed->addOutput(Z_SpeedPitch);
  FuzzyRule* fuzzyRule20 = new FuzzyRule(20, ifPitch_e_Z_And_ec_Z, Then_Pitch_Speed_Z_Speed);
  fuzzy->addFuzzyRule(fuzzyRule20);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule21-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsPUltra = new FuzzyRuleAntecedent();
  ifPitch_E_IsPUltra->joinSingle(PUltra_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_PUltra_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_PUltra_And_ec_Z->joinWithAND(ifPitch_E_IsPUltra, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PUltra_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PUltra_Speed1->addOutput(PUltra_SpeedPitch);
  FuzzyRule* fuzzyRule21 = new FuzzyRule(21, ifPitch_e_PUltra_And_ec_Z, Then_Pitch_Speed_PUltra_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule21);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule22-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsPMega = new FuzzyRuleAntecedent();
  ifPitch_E_IsPMega->joinSingle(PMega_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_PMega_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_PMega_And_ec_Z->joinWithAND(ifPitch_E_IsPMega, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PMega_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PMega_Speed1->addOutput(PMega_SpeedPitch);
  FuzzyRule* fuzzyRule22 = new FuzzyRule(22, ifPitch_e_PMega_And_ec_Z, Then_Pitch_Speed_PMega_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule22);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule23-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsPSB = new FuzzyRuleAntecedent();
  ifPitch_E_IsPSB->joinSingle(PSB_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_PSB_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_PSB_And_ec_Z->joinWithAND(ifPitch_E_IsPSB, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PSB_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PSB_Speed1->addOutput(PSB_SpeedPitch);
  FuzzyRule* fuzzyRule23 = new FuzzyRule(23, ifPitch_e_PSB_And_ec_Z, Then_Pitch_Speed_PSB_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule23);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule024-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsPB = new FuzzyRuleAntecedent();
  ifPitch_E_IsPB->joinSingle(PB_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_PB_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_PB_And_ec_Z->joinWithAND(ifPitch_E_IsPB, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PB_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PB_Speed1->addOutput(PB_SpeedPitch);
  FuzzyRule* fuzzyRule24 = new FuzzyRule(24, ifPitch_e_PB_And_ec_Z, Then_Pitch_Speed_PB_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule24);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule12-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsPM = new FuzzyRuleAntecedent();
  ifPitch_E_IsPM->joinSingle(PM_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_PM_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_PM_And_ec_Z->joinWithAND(ifPitch_E_IsPM, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PM_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PM_Speed1->addOutput(PM_SpeedPitch);
  FuzzyRule* fuzzyRule25 = new FuzzyRule(25, ifPitch_e_PM_And_ec_Z, Then_Pitch_Speed_PM_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule25);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule26-----------------------------------------
  FuzzyRuleAntecedent* ifPitch_E_IsPS = new FuzzyRuleAntecedent();
  ifPitch_E_IsPS->joinSingle(PS_Pitch_E);

  FuzzyRuleAntecedent* ifPitch_e_PS_And_ec_Z = new FuzzyRuleAntecedent();
  ifPitch_e_PS_And_ec_Z->joinWithAND(ifPitch_E_IsPS, ifPitch_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Pitch_Speed_PS_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Pitch_Speed_PS_Speed1->addOutput(PS_SpeedPitch);
  FuzzyRule* fuzzyRule26 = new FuzzyRule(26, ifPitch_e_PS_And_ec_Z, Then_Pitch_Speed_PS_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule26);
  //---------------------------------------------------------------------------------------//

  //End Of Pitch
  //---------start of Yaw ------------------------------
  //fuzzy input for Yaw_Error
  FuzzyInput* Yaw_Error = new FuzzyInput(5);
  Yaw_Error->addFuzzySet(NUltra_Yaw_E);
  Yaw_Error->addFuzzySet(NMega_Yaw_E);
  Yaw_Error->addFuzzySet(NSB_Yaw_E);
  Yaw_Error->addFuzzySet(NB_Yaw_E);
  Yaw_Error->addFuzzySet(NM_Yaw_E);
  Yaw_Error->addFuzzySet(NS_Yaw_E);
  Yaw_Error->addFuzzySet(Z_Yaw_E);
  Yaw_Error->addFuzzySet(PS_Yaw_E);
  Yaw_Error->addFuzzySet(PM_Yaw_E);
  Yaw_Error->addFuzzySet(PB_Yaw_E);
  Yaw_Error->addFuzzySet(PSB_Yaw_E);
  Yaw_Error->addFuzzySet(PMega_Yaw_E);
  Yaw_Error->addFuzzySet(PUltra_Yaw_E);
  fuzzy->addFuzzyInput(Yaw_Error);

  //fuzzy input for roll_error_change
  FuzzyInput* Yaw_Error_change = new FuzzyInput(6);
  Yaw_Error_change->addFuzzySet(Z_Yaw_EC);
  fuzzy->addFuzzyInput(Yaw_Error_change);

  //fuzzy Output for Yaw
  FuzzyOutput* yaw_Out = new FuzzyOutput(3);
  yaw_Out->addFuzzySet(NUltra_SpeedYaw);
  yaw_Out->addFuzzySet(NMega_SpeedYaw);
  yaw_Out->addFuzzySet(NSB_SpeedYaw);
  yaw_Out->addFuzzySet(NB_SpeedYaw);
  yaw_Out->addFuzzySet(NM_SpeedYaw);
  yaw_Out->addFuzzySet(NS_SpeedYaw);
  yaw_Out->addFuzzySet(Z_SpeedYaw);
  yaw_Out->addFuzzySet(PS_SpeedYaw);
  yaw_Out->addFuzzySet(PM_SpeedYaw);
  yaw_Out->addFuzzySet(PB_SpeedYaw);
  yaw_Out->addFuzzySet(PSB_SpeedYaw);
  yaw_Out->addFuzzySet(PMega_SpeedYaw);
  yaw_Out->addFuzzySet(PUltra_SpeedYaw);
  fuzzy->addFuzzyOutput(yaw_Out);

  //fuzzy Output for Yaw tilt for  servo1 & servo2.
  FuzzyOutput* Yaw_Out_tilt = new FuzzyOutput(4);
  Yaw_Out_tilt->addFuzzySet(Z_tilt);
  Yaw_Out_tilt->addFuzzySet(PS_tilt);
  Yaw_Out_tilt->addFuzzySet(PM_tilt);
  Yaw_Out_tilt->addFuzzySet(PB_tilt);
  Yaw_Out_tilt->addFuzzySet(PSB_tilt);
  Yaw_Out_tilt->addFuzzySet(PMega_tilt);
  Yaw_Out_tilt->addFuzzySet(PUltra_tilt);
  Yaw_Out_tilt->addFuzzySet(NS_tilt);
  Yaw_Out_tilt->addFuzzySet(NM_tilt);
  Yaw_Out_tilt->addFuzzySet(NB_tilt);
  Yaw_Out_tilt->addFuzzySet(NSB_tilt);
  Yaw_Out_tilt->addFuzzySet(NMega_tilt);
  Yaw_Out_tilt->addFuzzySet(NUltra_tilt);
  fuzzy->addFuzzyOutput(Yaw_Out_tilt);


  //-------------------------------------Rule27-Yaw----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsNUltra = new FuzzyRuleAntecedent();
  ifYaw_E_IsNUltra->joinSingle(NUltra_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_EC_IsZ = new FuzzyRuleAntecedent();
  ifYaw_EC_IsZ->joinSingle(Z_Yaw_EC);

  FuzzyRuleAntecedent* ifYaw_e_NUltra_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_NUltra_And_ec_Z->joinWithAND(ifYaw_E_IsNUltra, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_NUltra_Speed = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_NUltra_Speed->addOutput(NUltra_SpeedYaw);
  Then_Yaw_Speed_NUltra_Speed->addOutput(PUltra_tilt);
  FuzzyRule* fuzzyRule27 = new FuzzyRule(27, ifYaw_e_NUltra_And_ec_Z, Then_Yaw_Speed_NUltra_Speed);
  fuzzy->addFuzzyRule(fuzzyRule27);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule28-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsNMega = new FuzzyRuleAntecedent();
  ifYaw_E_IsNMega->joinSingle(NMega_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_NMega_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_NMega_And_ec_Z->joinWithAND(ifYaw_E_IsNMega, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_NMega_Speed = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_NMega_Speed->addOutput(NMega_SpeedYaw);
  Then_Yaw_Speed_NMega_Speed->addOutput(PMega_tilt);
  FuzzyRule* fuzzyRule28 = new FuzzyRule(28, ifYaw_e_NMega_And_ec_Z, Then_Yaw_Speed_NMega_Speed);
  fuzzy->addFuzzyRule(fuzzyRule28);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule29-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsNSB = new FuzzyRuleAntecedent();
  ifYaw_E_IsNSB->joinSingle(NSB_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_NSB_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_NSB_And_ec_Z->joinWithAND(ifYaw_E_IsNSB, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_NSB_Speed = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_NSB_Speed->addOutput(NSB_SpeedYaw);
  Then_Yaw_Speed_NSB_Speed->addOutput(PSB_tilt);
  FuzzyRule* fuzzyRule29 = new FuzzyRule(29, ifYaw_e_NSB_And_ec_Z, Then_Yaw_Speed_NSB_Speed);
  fuzzy->addFuzzyRule(fuzzyRule29);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule30-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsNB = new FuzzyRuleAntecedent();
  ifYaw_E_IsNB->joinSingle(NB_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_NB_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_NB_And_ec_Z->joinWithAND(ifYaw_E_IsNB, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_NB_Speed = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_NB_Speed->addOutput(NB_SpeedYaw);
  Then_Yaw_Speed_NB_Speed->addOutput(PB_tilt);
  FuzzyRule* fuzzyRule30 = new FuzzyRule(30, ifYaw_e_NB_And_ec_Z, Then_Yaw_Speed_NB_Speed);
  fuzzy->addFuzzyRule(fuzzyRule30);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule31-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsNM = new FuzzyRuleAntecedent();
  ifYaw_E_IsNM->joinSingle(NM_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_NM_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_NM_And_ec_Z->joinWithAND(ifYaw_E_IsNM, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_NM_Speed = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_NM_Speed->addOutput(NM_SpeedYaw);
  Then_Yaw_Speed_NM_Speed->addOutput(PM_tilt);
  FuzzyRule* fuzzyRule31 = new FuzzyRule(31, ifYaw_e_NM_And_ec_Z, Then_Yaw_Speed_NM_Speed);
  fuzzy->addFuzzyRule(fuzzyRule31);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule32-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsNS = new FuzzyRuleAntecedent();
  ifYaw_E_IsNS->joinSingle(NS_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_NS_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_NS_And_ec_Z->joinWithAND(ifYaw_E_IsNS, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_NS_Speed = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_NS_Speed->addOutput(NS_SpeedYaw);
  Then_Yaw_Speed_NS_Speed->addOutput(PS_tilt);
  FuzzyRule* fuzzyRule32 = new FuzzyRule(32, ifYaw_e_NS_And_ec_Z, Then_Yaw_Speed_NS_Speed);
  fuzzy->addFuzzyRule(fuzzyRule32);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule33-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsZ = new FuzzyRuleAntecedent();
  ifYaw_E_IsZ->joinSingle(Z_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_Z_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_Z_And_ec_Z->joinWithAND(ifYaw_E_IsZ, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_Z_Speed = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_Z_Speed->addOutput(Z_SpeedYaw);
  Then_Yaw_Speed_Z_Speed->addOutput(Z_tilt);
  FuzzyRule* fuzzyRule33 = new FuzzyRule(33, ifYaw_e_Z_And_ec_Z, Then_Yaw_Speed_Z_Speed);
  fuzzy->addFuzzyRule(fuzzyRule33);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule34-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsPUltra = new FuzzyRuleAntecedent();
  ifYaw_E_IsPUltra->joinSingle(PUltra_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_PUltra_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_PUltra_And_ec_Z->joinWithAND(ifYaw_E_IsPUltra, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_PUltra_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_PUltra_Speed1->addOutput(PUltra_SpeedYaw);
  Then_Yaw_Speed_PUltra_Speed1->addOutput(NUltra_tilt);
  FuzzyRule* fuzzyRule34 = new FuzzyRule(34, ifYaw_e_PUltra_And_ec_Z, Then_Yaw_Speed_PUltra_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule34);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule35-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsPMega = new FuzzyRuleAntecedent();
  ifYaw_E_IsPMega->joinSingle(PMega_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_PMega_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_PMega_And_ec_Z->joinWithAND(ifYaw_E_IsPMega, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_PMega_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_PMega_Speed1->addOutput(PMega_SpeedYaw);
  Then_Yaw_Speed_PMega_Speed1->addOutput(NMega_tilt);
  FuzzyRule* fuzzyRule35 = new FuzzyRule(35, ifYaw_e_PMega_And_ec_Z, Then_Yaw_Speed_PMega_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule35);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule36-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsPSB = new FuzzyRuleAntecedent();
  ifYaw_E_IsPSB->joinSingle(PSB_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_PSB_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_PSB_And_ec_Z->joinWithAND(ifYaw_E_IsPSB, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_PSB_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_PSB_Speed1->addOutput(PSB_SpeedYaw);
  Then_Yaw_Speed_PSB_Speed1->addOutput(NSB_tilt);
  FuzzyRule* fuzzyRule36 = new FuzzyRule(36, ifYaw_e_PSB_And_ec_Z, Then_Yaw_Speed_PSB_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule36);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule37-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsPB = new FuzzyRuleAntecedent();
  ifYaw_E_IsPB->joinSingle(PB_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_PB_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_PB_And_ec_Z->joinWithAND(ifYaw_E_IsPB, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_PB_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_PB_Speed1->addOutput(PB_SpeedYaw);
  Then_Yaw_Speed_PB_Speed1->addOutput(NB_tilt);
  FuzzyRule* fuzzyRule37 = new FuzzyRule(37, ifYaw_e_PB_And_ec_Z, Then_Yaw_Speed_PB_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule37);
  //---------------------------------------------------------------------------------------//

  //-------------------------------------Rule38-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsPM = new FuzzyRuleAntecedent();
  ifYaw_E_IsPM->joinSingle(PM_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_PM_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_PM_And_ec_Z->joinWithAND(ifYaw_E_IsPM, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_PM_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_PM_Speed1->addOutput(PM_SpeedYaw);
  Then_Yaw_Speed_PM_Speed1->addOutput(NM_tilt);
  FuzzyRule* fuzzyRule38 = new FuzzyRule(38, ifYaw_e_PM_And_ec_Z, Then_Yaw_Speed_PM_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule38);
  //---------------------------------------------------------------------------------------//
  //-------------------------------------Rule39-----------------------------------------
  FuzzyRuleAntecedent* ifYaw_E_IsPS = new FuzzyRuleAntecedent();
  ifYaw_E_IsPS->joinSingle(PS_Yaw_E);

  FuzzyRuleAntecedent* ifYaw_e_PS_And_ec_Z = new FuzzyRuleAntecedent();
  ifYaw_e_PS_And_ec_Z->joinWithAND(ifYaw_E_IsPS, ifYaw_EC_IsZ);
  //-----------------------OUTPUT-------------------------------------------------------
  FuzzyRuleConsequent* Then_Yaw_Speed_PS_Speed1 = new FuzzyRuleConsequent(); //output
  Then_Yaw_Speed_PS_Speed1->addOutput(PS_SpeedYaw);
  Then_Yaw_Speed_PS_Speed1->addOutput(NS_tilt);
  FuzzyRule* fuzzyRule39 = new FuzzyRule(39, ifYaw_e_PS_And_ec_Z, Then_Yaw_Speed_PS_Speed1);
  fuzzy->addFuzzyRule(fuzzyRule39);
  //---------------------------------------------------------------------------------------//

  //Wait until  the receiver is active and the throttle is set to the lower position.
  Serial.println("Waiting until the receiver is active and the throttle is set to the lower position.");
  //while (throttle_channel_3 < 990 || throttle_channel_3 > 1020 || yaw_channel_4 < 1400) {
  while (throttle_channel_3 < 990 || throttle_channel_3 > 1070) {
    start ++;                                                  //While waiting increment start whith every loop.

    delayMicroseconds(900);
    if (start == 125) {                                       //Every 125 loops (500ms).
      digitalWrite(12, !digitalRead(12));                      //Change the led status.
      start = 0;                                               //Start again at 0.
    }
  }
  start = 0;                                                   //Set start back to 0.
  digitalWrite(12, HIGH);
}

void loop(void) {
  currentTime = millis();

  if (start == 0) {
    //Set all servo to 90 degrees at the start for correct orientation
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);

    esc1.writeMicroseconds(1480);
    esc2.writeMicroseconds(1480);
    esc3.writeMicroseconds(1480);
    esc4.writeMicroseconds(1480);
  }

  //TO-DO: Change initiation process, use the toggle or position switches fro arming.
  //For starting the motors: throttle low and yaw left (step 1).
  if (start == 0 && throttle_channel_3 < 1070 && yaw_channel_4 < 1070) {
    start = 1;
    digitalWrite(12, HIGH);
    Serial.println("First");
  }
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && throttle_channel_3 < 1070 && yaw_channel_4 > 1460) {
    start = 2;
    digitalWrite(12, HIGH);
    Serial.println("Second");
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && throttle_channel_3 < 1070 && yaw_channel_4 > 1870) {
    start = 0;
    digitalWrite(12, LOW);
    Serial.println("Third");

  }

  /*------------Convert signals of RF from 1495-1880, to 0-80 degrees for ROLL---*/
  roll_setpoint = 0;
  if (roll_channel_1 > 1495) { //when roll stick is from center to highest
    if (roll_channel_1 > 1880) roll_channel_1 = 1880;

    roll_setpoint = map(roll_channel_1, 1495, 1880, 90, 180);
  }
  else if (roll_channel_1 < 1455) { //when roll stick is from center to lowest
    if (roll_channel_1 < 1055) roll_channel_1 = 1055;

    roll_setpoint = map(roll_channel_1, 1455, 1055, 90, 0);
  }

  /*------------Convert signals of RF from 1510-1880, to 0-80 degrees for PITCH---*/
  pitch_setpoint = 0;
  if (pitch_channel_2 > 1510) { //when pitch stick is from center to highest
    if (pitch_channel_2 > 1880) pitch_channel_2 = 1880;
    forward_speed = map(pitch_channel_2, 1510, 1880, 0, 500);  // Map 1510-2000 to 0-500 millissecond as the range for forward speed.

    pitch_angle1 = map(pitch_channel_2, 1510, 1880, 90, 0);    //Map 1510-2000 to 90-0 degrees as the output range for servo3.
    pitch_angle2 = map(pitch_channel_2, 1510, 1880, 90, 180);  //Map 1510-2000 to 90-180 degrees as the output range for servo4.

  }
  else if (pitch_channel_2 < 1470) { //when pitch stick is from center to lowest
    if (pitch_channel_2 < 1055) pitch_channel_2 = 1055;
    forward_speed = map(pitch_channel_2, 1470, 1055, 0, 500);// Map 1470-1055 to 0-500 millissecond as the range for forward speed.

    pitch_angle1 = map(pitch_channel_2, 1470, 1055, 90, 180);//Map 1470-1055 to 90-180 degrees as the output range for servo3.
    pitch_angle2 = map(pitch_channel_2, 1470, 1055, 90, 0);//Map 1470-1055 to 90-0 degrees as the output range for servo4.

  } else {
    pitch_angle1 = 90;
    pitch_angle2 = 90;
  }

  /*------------Convert signals from RF from 1490-1880, to 0-180 degrees for YAW---*/
  yaw_setpoint = 0;
  if (yaw_channel_4 > 1490) { //when yaw stick is from center to highest
    if (yaw_channel_4 > 1880) yaw_channel_4 = 1880;

    yaw_setpoint = map(yaw_channel_4, 1490, 1880, 0, 270);

    yaw_angle1 = map(yaw_channel_4, 1490, 1880, 90, 0);//Map 1470-1055 to 90-180 degrees as the output range for servo1.
    yaw_angle2 = map(yaw_channel_4, 1490, 1880, 90, 0);//Map 1470-1055 to 90-0 degrees as the output range for servo2.

  }
  else if (yaw_channel_4 < 1450) { //when yaw stick is from center to lowest
    if (yaw_channel_4 < 1055) yaw_channel_4 = 1055;

    yaw_setpoint = map(yaw_channel_4, 1450, 1055, 360, 90);

    yaw_angle1 = map(yaw_channel_4, 1450, 1055, 90, 180);//Map 1470-1055 to 90-180 degrees as the output range for servo1.
    yaw_angle2 = map(yaw_channel_4, 1450, 1055, 90, 180);//Map 1470-1055 to 90-0 degrees as the output range for servo2.

  } else {
    yaw_setpoint = 360;
    yaw_angle1 = 90;
    yaw_angle2 = 90;
  }

  //print_signals();
  getsignal_in();

  /*----------Get the error and the change of error-----------*/
  roll_error = (roll_setpoint) + (Roll_combined);
  pitch_error = (Pitch_combined);
  yaw_error = (yaw_setpoint) - (Heading_combined);

  roll_error_change = roll_error - previous_error_roll;
  pitch_error_change = pitch_error - previous_error_pitch;
  yaw_error_change = yaw_error - previous_error_yaw;

  /*----------FUZZIFICATION AND DEFFUZIFICATION PROCESSES------*/
  fuzzy->setInput(1, roll_error);
  fuzzy->setInput(2, roll_error_change);
  fuzzy->setInput(3, pitch_error);
  fuzzy->setInput(4, pitch_error_change);
  fuzzy->setInput(5, yaw_error);
  fuzzy->setInput(6, yaw_error_change);
  fuzzy->fuzzify();
  fuzzy_roll_out = fuzzy->defuzzify(1);
  fuzzy_pitch_out = fuzzy->defuzzify(2);
  fuzzy_yaw_out = fuzzy->defuzzify(3);
  fuzzy_yaw_out_tilt = fuzzy->defuzzify(4);

  /*--------Save current error to Previous error for the next calculation---*/
  previous_error_roll = roll_error;
  previous_error_pitch = pitch_error;
  previous_error_yaw = yaw_error;

  if (start == 2) {
    /*--------Get Throttle AND limit it to 1800 only to have an allowance for additional speed---*/
    throttle = throttle_channel_3;

    /*----------YAW MANIPULATION -----------------*/
    if (fuzzy_yaw_out > 0) {
      yaw_out = fuzzy_yaw_out;
    }
    else if (fuzzy_yaw_out < 0) {
      yaw_out = fuzzy_yaw_out * (-1);
    }

    /*----------PITCH MANIPULATION -----------------*/
    //pitchManipulation = yaw_out + fuzzy_pitch_out;
    pitchManipulation = fuzzy_pitch_out;
    if (pitch_error > 2) {
      //if blimp is tilting backwards

      pitchThrustDown = map(pitchManipulation, 0, 1000, 1490, 1880); // thrust downward
      pitchThrustUp = map(pitchManipulation, 0, 1000, 1470, 1070); // thrust upward

      esc1_out = pitchThrustUp;
      esc2_out = pitchThrustDown;
      //esc3_out = rollThrustDown;
      //esc4_out = rollThrustDown;

      if (Pitch_combined < 0) {
        servo3.write(90 + Pitch_combined);
        servo4.write(90 - Pitch_combined);
      } else {
        servo3.write(90 - Pitch_combined);
        servo4.write(90 + Pitch_combined);
      }
    }
    else if (pitch_error < -2) {
      //if blimp is tilting forward

      pitchThrustDown = map(pitchManipulation, 0, 1000, 1490, 1880); // thrust downward
      pitchThrustUp = map(pitchManipulation, 0, 1000, 1470, 1070); // thrust upward

      esc1_out = pitchThrustUp;
      esc2_out = pitchThrustDown;
      //esc3_out = rollThrustDown;
      //esc4_out = rollThrustDown;

      if (Pitch_combined < 0) {
        servo3.write(90 - Pitch_combined);
        servo4.write(90 + Pitch_combined);
      } else {
        servo3.write(90 + Pitch_combined);
        servo4.write(90 - Pitch_combined);
      }
    }
    else {
      esc1_out = 1480; //Motor Stop
      esc2_out = 1480; //Motor Stop
    }

    /*---------------------COMBINED OUTPUT ---------------------*/
    if (forward_speed > 100) {
      //esc3_out = forward_speed;
      //esc4_out = forward_speed;
    }

    if (forward_speed < 100) {
      /*----------ROLL MANIPULATION -----------------*/
      rollManipulation = fuzzy_roll_out;
      if (roll_error > 2) {
        //if blimp is tilting to the right

        rollThrustDown = map(rollManipulation, 0, 1000, 1490, 1880); // thrust downward
        rollThrustUp = map(rollManipulation, 0, 1000, 1470, 1070); // thrust upward

        //esc1_out = pitchThrustDown;
        //esc2_out = pitchThrustDown;
        esc3_out = rollThrustUp;
        esc4_out = rollThrustDown;

        if (Roll_combined < 0) {
          servo1.write(90 + Roll_combined);
          servo2.write(90 - Roll_combined);
        } else {
          servo1.write(90 - Roll_combined);
          servo2.write(90 + Roll_combined);
        }

      }
      else if (roll_error < -2) {
        //if blimp is tilting to the left

        rollThrustDown = map(rollManipulation, 0, 1000, 1490, 1880); // thrust downward
        rollThrustUp = map(rollManipulation, 0, 1000, 1470, 1070); // thrust upward

        //esc1_out = pitchThrustDown;
        //esc2_out = pitchThrustDown;
        esc3_out = rollThrustDown;
        esc4_out = rollThrustUp;

        if (Roll_combined < 0) {
          servo1.write(90 - Roll_combined);
          servo2.write(90 + Roll_combined);
        } else {
          servo1.write(90 + Roll_combined);
          servo2.write(90 - Roll_combined);
        }
      }
      else {
        esc3_out = 1480; //Motor Stop
        esc4_out = 1480; //Motor Stop
      }

    }

    /*----------USE SERVO LIBRARY TO DRIVE THE ESC's AND SERVO's--------*/
    esc1.writeMicroseconds(esc1_out);
    esc2.writeMicroseconds(esc2_out);
    esc3.writeMicroseconds(esc3_out);
    esc4.writeMicroseconds(esc4_out);


    //servo1.write(servo1_out);
    //servo2.write(servo2_out);
    //servo3.write(pitch_angle1);
    //servo4.write(pitch_angle2);

    /*
      Serial.print("\n");
      Serial.print(" Roll Error    :");
      Serial.print(roll_error);
      Serial.print("  Pitch Error    :");
      Serial.print(pitch_error);
      Serial.print("  Yaw Error      :");
      Serial.print(yaw_out);
      Serial.print("  Roll1       :");
      Serial.print(esc1_out);
      Serial.print("  pitch2      :");
      Serial.print(esc2_out);
      Serial.print("  Roll1       :");
      Serial.print(esc3_out);
      Serial.print("  Roll2       :");
      Serial.print(esc4_out);
    */
  }

}
/*-----------------INTERRUPT SUB_ROUTINE TO GET THE RF SIGNAL ----------------*/
void calcRoll() {
  if (digitalRead(52)) timer_1 = micros();
  else roll_channel_1 = micros() - timer_1;
}

void calcPitch() {
  if (digitalRead(50)) timer_2 = micros();
  else pitch_channel_2 = micros() - timer_2;
}

void calcYaw() {
  if (digitalRead(46))timer_4 = micros();
  else yaw_channel_4 = micros() - timer_4;
}

void calcThrottle() {
  if (digitalRead(48))timer_3 = micros();
  else throttle_channel_3 = micros() - timer_3;
}

void calcReverse() {
  if (digitalRead(44))timer_4 = micros();
  else control_channel_5 = micros() - timer_4;
}


/*--------------PRINT SIGNALS FROM RF-----------*/
//TO-DO: check if RF stick is calibrated show this if not and let the user calibrate the stick
void print_signals() {
  Serial.print("Roll:");
  if (roll_channel_1 - 1455 < 0)Serial.print("<-");
  else if (roll_channel_1 - 1495 > 0)Serial.print("->");
  else Serial.print("-+-");
  Serial.print(roll_channel_1);

  Serial.print("  Pitch:");
  if (pitch_channel_2 - 1470 < 0)Serial.print("vvv");
  else if (pitch_channel_2 - 1510 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(pitch_channel_2);

  Serial.print("  Throttle:");
  if (throttle_channel_3 - 1465 < 0)Serial.print("vvv");
  else if (throttle_channel_3 - 1505 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(throttle_channel_3);

  Serial.print("  Yaw:");
  if (yaw_channel_4 - 1450 < 0)Serial.print("<-");
  else if (yaw_channel_4 - 1490 > 0)Serial.print("->");
  else Serial.print("-+-");
  Serial.println(yaw_channel_4);
}

/*---------------------GET SENSOR SIGNAL AND USE COMPLIMENTARY FILTER FOR SMOOTH DATA-----------------*/
void getsignal_in() {

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Roll_combined = euler.z();
  Pitch_combined = euler.y();
  Heading_combined = euler.x();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}


void initSensors() {
  /* Initialise the sensor */
  Serial.begin(115200);
  Serial.println("Checking BNO055..."); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}


//esc_out1 = back motor
//esc_out2 = front motor
//esc_out3 = right motor
//esc_out4 = left motor

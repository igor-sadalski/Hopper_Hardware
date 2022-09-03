#include <Archer_Config.h>
#include <TripENC.h>
#include <ELMO_CANt4.h>
#include <Koios.h>
#include <SPI.h>
//#include <SD.h>
#include <TeensyThreads.h>
#include <ArduinoEigen.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace Archer;
using namespace Eigen;

//==================CONSTANTS
TripENC tENC(trip_CS1,trip_CS2,trip_CS3);
ELMO_CANt4 elmo;

float x_d[7];
#define torque_to_current 1.0/0.083

#define MAX_CURRENT 12  //  15
#define MIN_CURRENT -12 // -15

#define TIMEOUT_INTERVAL 100 // ms to timeout

using vector_3t = Eigen::Matrix<float, 3, 1>;
using vector_4t = Eigen::Matrix<float, 4, 1>;
using matrix_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using matrix_3t = Eigen::Matrix<float, 3, 3>;
using quat_t = Eigen::Quaternion<float>;

//For rotation about x and y
//#define kp_y 15.0
//#define kp_rp 30.0
//#define kd_y 0.7
//#define kd_rp 1.5

#define kp_y 15.0
#define kp_rp 120.0
#define kd_y 1.0
#define kd_rp 2.0
//use volatile if we need to use threading for our robot
volatile float dR = 0;
volatile float dP = 0;
volatile float dY = 0;
volatile float x1;
volatile float v1;
volatile float x2;
volatile float v2;
volatile float x3;
volatile float v3;
volatile float q0;
volatile float q1;
volatile float q2;
volatile float q3;

quat_t quat_init;
quat_t quat_init_inverse;
bool initialized = false;
bool time_initialized = false;

Koios *koios;

int nF;
int rt = 0;

const byte numChars = 128;
char receivedChars[numChars]; //change here to get more charactes in the string
boolean newData = false;
//this is only for the first debugging run
char messageFromRobot[128]; //fill in the exact length of the message you will be sending
char exactMessage[40]; //fill in the exact length of the message you will be sending
float state[10];

//===========SPEEDING UP BABY
char additional_read_buffer[3000]; //this values are out of nowhere
char additional_write_buffer[3000];

//=================SETUP=============

bool exit_state = false;

void setup() {

  //====================WIFI==============
  delay(100); //give time to open and print
  Serial.begin(115200); //this is for the monitor
  Serial7.begin(115200); //baud rates must be the same
  while (!Serial7) {;}
  delay(100);
  
  //start a diode to be sure all is working
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  //================Koios=============
  koios = new Koios(tENC, elmo);
  koios->initKoios1(1);

  // initKoios2
  delay(250);
  koios->STO(1);
  koios->waitSwitch(1); //manual switch on robot
//  koios->setSigB(1);
  delay(250);
  rt = koios->motorsOn();
  delay(5000);
  koios->resetStates();
  koios->setLEDs("0100");
  koios->waitSwitch(0);
//  koios->setSigB(0);
  koios-> setLogo('A');
  delay(5000);
  koios->flashG(2);
  delay(5000);

  
  rt = threads.setSliceMicros(50);
  threads.addThread(imuThread);
  koios->setLEDs("0001");
  koios->setLogo('G');
  delay(2000);
  Serial7.clear();
}

//============FUNCTIONS==========
void delayLoop(uint32_t T1, uint32_t L){
  uint32_t T2 = micros();
  if((T2-T1)<L){
    uint32_t a = L + T1 - T2;
    threads.delay_us(a);
  }
}

void imuThread(){
  while(1){
    if(IMU_PORT.available() > 0){
      uint8_t byt = IMU_PORT.read();
      if(byt == 0xFA){
        threads.delay_us(350);
        for(int i=0;i<3;i++){
          IMU_PORT.read();
        }
        //=========================================================
        union{
          long y;
          float z;
        }Q0;
        uint8_t b0 = IMU_PORT.read();
        uint8_t b1 = IMU_PORT.read();
        uint8_t b2 = IMU_PORT.read();
        uint8_t b3 = IMU_PORT.read();
        uint8_t bQ0[4] = {b0,b1,b2,b3};
        long x = (long)bQ0[3]<<24|(long)bQ0[2]<<16|bQ0[1]<<8|bQ0[0];
        Q0.y = x;
        q0 = Q0.z;
        //=========================================================
        union{
          long y;
          float z;
        }Q1;
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
        uint8_t bQ1[4] = {b0,b1,b2,b3};
        x = (long)bQ1[3]<<24|(long)bQ1[2]<<16|bQ1[1]<<8|bQ1[0];
        Q1.y = x;
        q1 = Q1.z;
        union{
          long y;
          float z;
        }Q2;
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
        uint8_t bQ2[4] = {b0,b1,b2,b3};
        x = (long)bQ2[3]<<24|(long)bQ2[2]<<16|bQ2[1]<<8|bQ2[0];
        Q2.y = x;
        q2 = Q2.z;
        union{
          long y;
          float z;
        }Q3;
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
        uint8_t bQ3[4] = {b0,b1,b2,b3};
        x = (long)bQ3[3]<<24|(long)bQ3[2]<<16|bQ3[1]<<8|bQ3[0];
        Q3.y = x;
        q3 = Q3.z;
        union{
          long y;
          float z;
        }RR;
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
        uint8_t bR[4] = {b0,b1,b2,b3};
        x = (long)bR[3]<<24|(long)bR[2]<<16|bR[1]<<8|bR[0];
        RR.y = x;
        dR = RR.z;
        union{
          long y;
          float z;
        }PP;
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
        uint8_t bP[4] = {b0,b1,b2,b3};
        x = (long)bP[3]<<24|(long)bP[2]<<16|bP[1]<<8|bP[0];
        PP.y = x;
        dP = PP.z;
        union{
          long y;
          float z;
        }YY;
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
        uint8_t bY[4] = {b0,b1,b2,b3};
        x = (long)bY[3]<<24|(long)bY[2]<<16|bY[1]<<8|bY[0];
        YY.y = x;
        dY = YY.z;
        IMU_PORT.read();
        IMU_PORT.read();
        nF++;
      }
    }
    threads.delay_us(100);
  }
}

matrix_3t cross(vector_3t q) {
    matrix_3t c;
    c << 0, -q(2), q(1),
            q(2), 0, -q(0),
            -q(1), q(0), 0;
    return c;
}

unsigned long last_ESP_message;
unsigned long current_ESP_message;
char oneAdded[4];
quat_t quat_a;

void getTorque(float* state, quat_t quat_a, vector_3t &torque) {
    vector_3t delta_omega;
    vector_3t omega_a;
    vector_3t omega_d;
    vector_4t quat_d_vec;
    vector_4t quat_a_vec;

// 1-2-3 -- no
    quat_t quat_actuator = quat_t(0.8806, 0.3646, -0.2795, 0.1160);
// 3-1-2 -- no
//    quat_t quat_actuator = quat_t(0.3400, 0.4249, 0.1758, 0.8204);
// 2-3-1 -- no
//    quat_t quat_actuator = quat_t(0.5405, -0.0607, -0.4558, -0.7053);
// 3-2-1 -- def no
//    quat_t quat_actuator = quat_t(0.6847, 0.2106, 0.0878, -0.1092);
    quat_t quat_d = quat_t(1,0,0,0);
    omega_d << 0,0,0;
    omega_a << state[3], state[4], state[5];

//    quat_d *= quat_actuator;
//    quat_a *= quat_actuator;

    quat_d_vec << quat_d.w(), quat_d.x(), quat_d.y(), quat_d.z();
    quat_a_vec << quat_a.w(), quat_a.x(), quat_a.y(), quat_a.z();

    vector_3t delta_quat;
    delta_quat << quat_a_vec[0] * quat_d_vec.segment(1, 3) - quat_d_vec[0] * quat_a_vec.segment(1, 3) -
                  cross(quat_a_vec.segment(1, 3)) * quat_d_vec.segment(1, 3);
    matrix_3t Kp, Kd;
    Kp.setZero();
    Kd.setZero();
    Kp.diagonal() << kp_rp, kp_rp, kp_y;
    Kd.diagonal() << kd_rp, kd_rp, kd_y;

//    Serial.print(delta_quat(0)); Serial.print(", ");
//    Serial.print(delta_quat(1)); Serial.print(", ");
//    Serial.print(delta_quat(2)); Serial.print(", ");
//    Serial.println();

    vector_3t tau;
    delta_omega = -quat_actuator.inverse()._transformVector(omega_a) - omega_d;
    tau = -quat_actuator.inverse()._transformVector(Kp * delta_quat) - Kd * delta_omega;

    torque << tau*torque_to_current;
}

//==========================LOOP=============
  char receivedCharsESP[34];
  char receivedCharsTeensy[10*sizeof(float)+8+1];

void exitProgram() {
    elmo.cmdTC(0.0,IDX_K1);
    elmo.cmdTC(0.0,IDX_K2);
    elmo.cmdTC(0.0,IDX_K3); 
    koios->motorsOff(0);
    koios->setSigB(1);
    koios->setLEDs("1000");
    koios->setLogo('R');
    while(1) {};
}

void loop() {
  static float Q0 = 0;
  static float Q1 = 0;
  static float Q2 = 0;
  static float Q3 = 0;
  static float DY = 0;
  static float DP = 0;
  static float DR = 0;
  
  //check if imu data is not corrupted
  int rt = koios->checkFrame(q0,q1,q2,q3,dY,dP,dR);
  //based on imu upadate states
  if(rt==1){
    Q0 = q0;
    Q1 = q1;
    Q2 = q2;
    Q3 = q3;
    DY = dY;
    DP = dP;
    DR = dR;
  }
  koios->updateStates(x1,v1,x2,v2,x3,v3);
  // Add step to get leg length from Bia here over serial
  state[0] = v1;
  state[1] = v3;
  state[2] = v2;
  state[3] = DR;
  state[4] = DP;
  state[5] = DY;
  state[6] = Q0;
  state[7] = Q1;
  state[8] = Q2;
  state[9] = Q3;

  static int reset_cmd = 0;

  if (Serial.available() > 0) {
    while (Serial.available() > 0)
      Serial.read();
    reset_cmd = 1;
  };

  vector_4t state_tmp;
  state_tmp << state[6], state[7], state[8], state[9];

  if (!initialized &&
  (state_tmp.norm() > 0.95 && state_tmp.norm() < 1.05 )) {
//       Serial.print(state[6]); Serial.print(", ");
//       Serial.print(state[7]); Serial.print(", ");
//       Serial.print(state[8]); Serial.print(", ");
//       Serial.print(state[9]); Serial.print(", ");
//       Serial.println();
    quat_init = quat_t(state[9], state[6], state[7], state[8]);
    quat_init_inverse = quat_init.inverse();
    initialized = true;
  } 

  if (initialized) {
    quat_a = quat_t(state[9], state[6], state[7], state[8]); // assuming q_w is last.
    quat_a = quat_init_inverse*quat_a;
  
    state[6] = quat_a.w();
    state[7] = quat_a.x();
    state[8] = quat_a.y();
    state[9] = quat_a.z();
  }

  receivedCharsTeensy[0] = 0b11111111;
  receivedCharsTeensy[1] = 0b11111111;
  memcpy(receivedCharsTeensy+2, state, 40);
   
  for (int i = 0; i < 6; i++) {
      byte oneAdded = 0b00000001;
      for (int j = 1; j < 8; j++){
        if (receivedCharsTeensy[i*7+(j-1)+2] == 0b00000000) {
          receivedCharsTeensy[i*7+(j-1)+2] = 0b00000001;
          oneAdded += (1 << (8-j));
        }
      }
      memcpy(&receivedCharsTeensy[40+i+2], &oneAdded, 1);
  }
  receivedCharsTeensy[48] = 0b0;
   
  Serial7.print(receivedCharsTeensy);
  Serial7.flush();

    
  
  int index = 0;
  while(index < 34) {
    if (Serial7.available() > 0) {
      receivedCharsESP[index] = Serial7.read();
      index++;
    }
    if (time_initialized) {
      current_ESP_message = millis();
      if (current_ESP_message - last_ESP_message > TIMEOUT_INTERVAL) {
        exitProgram();
      }
    }
  }

  if (!time_initialized) {
    last_ESP_message = millis();
    time_initialized = true;
  } else {
    last_ESP_message = current_ESP_message;
  }

  ///////////////////Print Binary////////////////////
  //  for (int i = 0; i < 34; i++) {
  //    Serial.print(receivedCharsESP[i], BIN);
  //    Serial.print(" ");
  //  }
  //  Serial.println();

  memcpy(oneAdded, receivedCharsESP+2+28, 4*sizeof(char));
  for (int i = 0; i < 4; i++) {
    for (int j = 1; j < 8; j++) {
      if(oneAdded[i] & (1 << (8-j))) {
        receivedCharsESP[2+i*7+(j-1)] = 0;
      }
    }
  }

  float state_d[7];
  memcpy(state_d, receivedCharsESP+2, 7*4);

  //////////////// Print the desired state ////////////////////////////
  //     Serial.print(state_d[0]); Serial.print(", ");
  //     Serial.print(state_d[1]); Serial.print(", ");
  //     Serial.print(state_d[2]); Serial.print(", ");
  //     Serial.print(state_d[3]); Serial.print(", ");
  //     Serial.print(state_d[4]); Serial.print(", ");
  //     Serial.print(state_d[5]); Serial.print(", ");
  //     Serial.print(state_d[6]);
  //     Serial.println();



  //use for the counication with the wheel motors
  //convert torques to amps with torque / 0.083 = currents [A]
  //for a range of -1.6Nm to 1.6 Nm
  vector_3t current; 
  if (initialized) {
    getTorque(state, quat_a, current);  
  } else {
    current[0] = 0;
    current[1] = 0;
    current[2] = 0;
  }
  
//  Serial.print(current[0]); Serial.print(" ");
//  Serial.print(current[1]); Serial.print(" ");
//  Serial.print(current[2]); Serial.print(" ");
//  Serial.println();

  ////////////////////Safety///////////////////////
  for (int i = 0; i < 3; i++) {
    if (current[i] > MAX_CURRENT) {
      current[i] = MAX_CURRENT;
    } else if (current[i] < MIN_CURRENT) {
      current[i] = MIN_CURRENT;
    }
  }

  if (reset_cmd) {
    elmo.cmdTC(-0.1*v1,IDX_K1);
    elmo.cmdTC(-0.1*v3,IDX_K2);
    elmo.cmdTC(-0.1*v2,IDX_K3); 
    if (abs(v1) < 0.01 && abs(v2) < 0.01 && abs(v3) < 0.01) {
      reset_cmd = 0;
    }
  } else {
    elmo.cmdTC(current[0],IDX_K1);
    elmo.cmdTC(current[1],IDX_K2);
    elmo.cmdTC(current[2],IDX_K3); 
  }

//  elmo.cmdTC(current[0],IDX_K1);
//    elmo.cmdTC(current[1],IDX_K2);
//    elmo.cmdTC(current[2],IDX_K3); 
    

//    elmo.cmdTC(1,IDX_K1);
//    delay(1000);
//    elmo.cmdTC(-1,IDX_K1);
//    delay(1000);
//    elmo.cmdTC(0,IDX_K1);
//    elmo.cmdTC(1,IDX_K2);
//    delay(1000);
//    elmo.cmdTC(-1,IDX_K2);
//    delay(1000);
//    elmo.cmdTC(0,IDX_K2);
//    elmo.cmdTC(1,IDX_K3);
//    delay(1000);
//    elmo.cmdTC(-1,IDX_K3);
//    delay(1000);
//    elmo.cmdTC(0,IDX_K3);

 // send u4 current command to leg over serial RX/TX between teensy boards
 //  delay(1);
}

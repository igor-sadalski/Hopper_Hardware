//code developed with help of Eric 1 August 2022
//MVP for communicating with the robot IMU and sending commands to the encoders

#include <Archer_Config.h>    // Check that this works with latest steps, and with the full cage on ARCHER
#include <TripENC.h>
#include <ELMO_CANt4.h>
#include <Koios.h>
#include <SPI.h>
#include <SD.h>
#include <TeensyThreads.h>

using namespace Archer;

TripENC tENC(trip_CS1,trip_CS2,trip_CS3);
ELMO_CANt4 elmo;
volatile float dR = 0;
volatile float dP = 0;
volatile float dY = 0;
volatile float x1;
volatile float v1;
volatile float x2;
volatile float v2;
volatile float x3;
volatile float v3;

void setup() {
  Koios koios = Koios(tENC, elmo);
  koios.initKoios1(0);
  koios.resetStates();
  T1 = micros();
  rt = threads.setSliceMicros(50);
  threads.addThread(imuThread);
  koios.setLEDs("0001");
  koios.setLogo('G');
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



void loop() {
  static float Q0 = 0;
  static float Q1 = 0;
  static float Q2 = 0;
  static float Q3 = 0;
  static float DY = 0;
  static float DP = 0;
  static float DR = 0;
  
  //check if imu data is not corrupted
  rt = koios.checkFrame(q0,q1,q2,q3,dY,dP,dR);
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
  koios.updateStates(x1,v1,x2,v2,x3,v3);
  // Add step to get leg length from Bia here over serial

  //elmo.cmdTC(u1,IDX_K1);
  //elmo.cmdTC(u2,IDX_K2);
  //elmo.cmdTC(u3,IDX_K3); ,.l;;l

  // send u4 current command to leg over serial
  
  Serial.print(Q0,4);
  Serial.print(',');
  Serial.print(Q1,4);
  Serial.print(',');
  Serial.print(Q2,4);
  Serial.print(',');
  Serial.print(Q3,4);
  Serial.print(',');
  Serial.print(DY,4);
  Serial.print(',');
  Serial.print(DP,4);
  Serial.print(',');
  Serial.print(DR,4);
  Serial.print(',');
  Serial.print(v1,4);
  Serial.print(',');
  Serial.print(v2,4);
  Serial.print(',');
  Serial.println(v3,4);
  delayLoop(T1,1000);
  T1 = micros();
}

void delayLoop(uint32_t T1, uint32_t L){
  uint32_t T2 = micros();
  if((T2-T1)<L){
    uint32_t a = L + T1 - T2;
    threads.delay_us(a);
  }
}

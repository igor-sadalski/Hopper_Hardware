#include <Archer_Config.h>
#include <DualENC.h>
#include <ELMO_CANt4.h>
#include <ControlBia.h>
#include <Bia.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <ArduinoEigen.h>

using namespace Archer;
using namespace Eigen;

#define MAX_HOP_TIMEOUT 1000000000 // 1000000

DualENC dENC(doub_CS1,doub_CS2);
ELMO_CANt4 elmo;
ControlBia cBia(1.0);
Bia bia(dENC,elmo,cBia);

uint32_t T1, Th;
float tau_max = 10;
int numHops = 10;
volatile uint32_t T0,nF,pF,rt;
volatile float rb,wb,xf,vf,u;
float d0 = 0.015; // spring deflection // 0.15
float b0 = 0.75; // deflection to consider impact
float u0 = -9.5; // offset torque
int h = 0;
float rb0,v0;

void exitProgram() {
  rt = elmo.motorOff(IDX_BIA);
  bia.setLEDs("1000");
  while(1) {};
}

void setup() {
  Serial.begin(115200); //this is for the monitor
  delay(500);

  //initBia1
  bia.flashR(1);
  delay(5000);  
  rt = bia.initComm(1);
  delay(1000);
  if(rt>0){
    bia.flashA1(2); }
  else{
    bia.flashR(10); }
  delay(10);
  
  cBia.setTx(tau_max);
  v0 = 100;
  Th = 2000000*numHops + 1000000;

  //initBia2
  delay(250);
  bia.STO(1);
  bia.waitSigK(1);
  delay(250);
  elmo.motorOn(IDX_BIA);
  delay(5000);
  bia.resetState(1);
  bia.resetState(2);
  bia.findZero();
  bia.setLEDs("0100");
  bia.waitSigK(0);
  bia.reverseSig(1);
  bia.setSigK(0);
  delay(5000);
  bia.flashG(2);
  delay(5000);

  
  cBia.getRB0(rb0);
  bia.resetState(1);
  bia.resetState(2);
  T0 = micros();
  T1 = T0;
  bia.setLEDs("0001");
}

void loop() {
  // Do not do leg control
//  exitProgram();
  
  compPhase();    //
  releasePhase(); // control to rb = 0, end at xf = 0
  h++;
  if(h>=numHops){
    exitProgram();
  }
}

void compPhase(){
  uint32_t Tc0 = micros();
  uint32_t Ts0,dTs;
  float x,v,r,w,xfs,U;
  int i = 0;
  Serial.println("------------Comp Phase---------------");
  while(i<4){
        // WIFI ESTOP: 
    if (bia.checkSigK() == 1) {
      exitProgram();
    }
    if((micros()-Tc0)>MAX_HOP_TIMEOUT){ // if in comp phase for too long, move-onto release and end experiment
      h = numHops+1;
      i = 4;
      break;
    }
    bia.updateState(1,r,w);
    bia.trackU0(d0,u0,x,v,U);
    rb = r;
    wb = w;
    xf = x;
    vf = v;
//    Serial.print("Spring Deflection: "); Serial.print(xf);
//    Serial.print("  Spring Velocity: "); Serial.print(vf); Serial.println();
Serial.println(i);
    u  = U;
    if(i==0){                 // Waiting for compression
      if(xf>8.0){             // Check for enough deflection
        i = 1;
      }
    }
    if(i==1){                 // waiting to slow down
      if(abs(vf)<v0){          // Check for slow movement
        i = 2;
        Ts0 = micros();
      }
    }
    if(i==2){                 // waiting for settling
      dTs = micros() - Ts0;
      if(abs(vf)>v0){          // Check if it sped up
        i = 1;
      }
      else if(dTs > 50000){   // Check if settle time reached (50000)
        i   = 3;
        xfs = xf;
      }
    }
    if(i==3){                 // Check if impact occured 
      if((xf-xfs) > b0){
        i = 4;
      }
    }
    delayLoop(T1,1000);
    T1 = micros();
    nF++;
  }
}

void releasePhase(){
  uint32_t Tr0 = micros();
  float r,w,x,v,up,ud;
  int i = 0;
  Serial.println("------------Release Phase---------------");
  while(i<1){
    // WIFI ESTOP: 
    if (bia.checkSigK() == 1) {
      exitProgram();
    }
    if((micros()-Tr0)>MAX_HOP_TIMEOUT){ // if in release phase for too long, exit release phase and end experiment
      h = numHops+1;
      i = 1;
      break;
    }
    bia.testPDb(r,w,x,v,up,ud);
    rb = r;
    wb = w;
    xf = x;
    vf = v;
    u  = up + ud;
    if(xf<0.5){
      i = 1;
    }
    delayLoop(T1,1000);
    T1 = micros();
    nF++;
  }
}

void delayLoop(uint32_t T1,uint32_t dT){
  uint32_t T2 = micros();
  if((T2-T1)<dT){
    uint32_t a = dT + T1 - T2;
    threads.delay_us(a);
  }
}

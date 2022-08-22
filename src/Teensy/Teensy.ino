#include <Archer_Config.h>
#include <TripENC.h>
#include <ELMO_CANt4.h>
#include <Koios.h>
#include <SPI.h>
//#include <SD.h>
#include <TeensyThreads.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace Archer;

//==================CONSTANTS
TripENC tENC(trip_CS1,trip_CS2,trip_CS3);
ELMO_CANt4 elmo;

float currents[4];


#define MAX_CURRENT 15
#define MIN_CURRENT -15

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
  
//  //koios->initKoios2();
//  delay(250);
//  koios->STO(1);
//  koios-> waitSwitch(1); //manual switch on robot
//  delay(250);
//  rt = koios->motorsOn();
//  delay(5000);
//  koios->resetStates();
//  koios->setLEDs("0100");
//  koios->waitSwitch(0);
//  koios-> setLogo('A');
//  delay(5000);
//  koios->flashG(2);
//  delay(5000);
//  rt = threads.setSliceMicros(50);
//  threads.addThread(imuThread);
//  koios->setLEDs("0001");
//  koios->setLogo('G');

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

//CHANGE SERIAL!!!
void ReadMessage() {
/* 
 *  save specific portion of binary stream separated by <...> 
 *  into the receivedChars string (char array)
*/
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '{';
    char endMarker = '}'; //terminating conditions
    char rc; //byte we are reading

    while (Serial7.available() > 0 && newData == false) {
        rc = Serial7.read(); //read in byte

        if (recvInProgress == true) { //means we are in <...>
            if (rc != endMarker) {
                receivedChars[ndx] = rc; //add char to our string
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1; //if outside string size limit go back
                }
            }
            else {
                receivedChars[ndx] = '\0'; //terminate that char
                recvInProgress = false; //what we received is not "<"
                ndx = 0;
                newData = true; //at end marker so finished reading a message
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true; //start logging from the next char
        }
    }
}

void TokenizeStringToFloats(char str[], float currents[]){
    char * pch;
    pch = strtok (str,",");
    int i = 0;
    while (pch != NULL)
    {
        currents[i] = strtof (pch, NULL);
        i++;
        pch = strtok (NULL, ",");
    }
}

//==========================LOOP=============

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

       
  //sensor readings should be given to the Serial.print(reading, 4) accuracy
  //use this for sending actual floats to the robot
  //use <...> to denote how message is beginning/ending
  //for some reason order of sending is important
  
  // String communication
  //sprintf(messageFromRobot,"<%f,%f,%f,%f,%f,%f,%f,%f,%f,%f>",v1,v2,v3,DY,DP,DR,Q0, Q1, Q2, Q3); 
  //Serial7.println(messageFromRobot);

  // float communication
  state[0] = v1;
  state[1] = v2;
  state[2] = v3;
  state[3] = DY;
  state[4] = DP;
  state[5] = DR;
  state[6] = Q0;
  state[7] = Q1;
  state[8] = Q2;
  state[9] = Q3;

  state[0] = 1; // debug
  state[1] = 1; // debug
  state[2] = 1; // debug
  state[3] = 1; // debug
  state[4] = 1; // debug
  state[5] = 1; // debug
  state[6] = 1; // debug
  state[7] = 1; // debug
  state[8] = 1; // debug
  state[9] = 1; // debug
  
  memcpy(&exactMessage[0], state, 10*sizeof(float));
//  exactMessage[0] = '<';
//  exactMessage[41] = '>';
  Serial7.println(exactMessage);
  Serial7.flush();
//  sprintf(messageFromRobot,"<%f,%f,%f,%f,%f,%f,%f,%f,%f,%f>",v1,v2,v3,DY,DP,DR,Q0, Q1, Q2, Q3); 
//  Serial.println(messageFromRobot);
  
  //delayLoop(micros(),100);  
  //Serial7.addMemoryForWrite(additional_write_buffer, sizeof(additional_write_buffer));
  
  //Serial7.flush(); //wait for any transmitted data still in buffers to atransmit
  
  //Increase the amount of buffer memory between reception of bytes by the serial 
  //hardware and the available() and read() functions.
  //Serial7.addMemoryForRead(additional_read_buffer, sizeof(additional_read_buffer));
  ReadMessage();
  Serial7.clear(); //Discard any received data that has not been read.  
  if (newData == true) {
     //Serial.println(receivedChars);
  
    TokenizeStringToFloats(receivedChars, currents);

         //Safety
    for (int i = 0; i < 3; i++) {
      if (currents[i] > MAX_CURRENT) {
        currents[i] = MAX_CURRENT;
      } else if (currents[i] < MIN_CURRENT) {
        currents[i] = MIN_CURRENT;
      }
    }
  
     Serial.println(currents[0]);
     Serial.println(currents[1]);
     Serial.println(currents[2]);
     Serial.println();
     
     newData = false;
  }

   //use for the communication with the wheel motors
  //convert torques to amps with torque / 0.083 = currents [A]
  //for a range of -1.6Nm to 1.6 Nm
  //this is done on the PC

  //2
  elmo.cmdTC(currents[0],IDX_K1);
  elmo.cmdTC(currents[1],IDX_K2);
  elmo.cmdTC(currents[2],IDX_K3); 

  // send u4 current command to leg over serial RX/TX between teensy boards
  delay(1);
}

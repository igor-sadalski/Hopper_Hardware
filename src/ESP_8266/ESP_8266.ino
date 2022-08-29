/*
   working two way communication ESP to PC sending strings
*/

#include <ESP8266WiFi.h> //lib for esp chipp

const char *ssid = "NETGEAR35";
const char *password = "amberlab";

// Set up TCP server and port number
int port = 8888;
WiFiServer server(port);

// Set static IP address, gateway IP address, subnet
IPAddress local_IP(192, 168, 1, 4); //this will be assigned to the ESP
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

const byte numChars = 40; //size of the message we expect to receive
boolean newDataTeensy = false;
boolean newDataPC = false;

String readValue;

void setup() {

  // Begin USB Serial
  delay(100);                    //give time to open serial
  Serial.begin(115200);        //there is only one serial on esp!!!
  // wait for serial port to connect. Needed for native USB
  while (!Serial) {
    ;
  }
  delay(100);                  //baud rates at both ends must match!!

  // configure static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  // Set to Station mode. ESP8266 connects to desired network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  //beging server
  server.begin();

  //start a diode to be sure all is working
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); //to start the diode on ESP turn it to low

  //  Serial.println("STA ready.");
}

union convert {
  float x[10];
  char b[40];
  unsigned long long int i[5];
} val;

void loop() {

  float state[10] = {1.2342, 1.2342, 1.2342, 1.2342, 1.2342, 1.2342, 1.2342, 1.2342, 1.2342, 1.2342};
  
  char receivedCharsTeensy[10 * sizeof(float) + 8 + 6];
  char sendCharsTeensy[34];
  char sendCharsTeensy2[34];
  float tmp2[3] = {1.435, 1.123, 5.025};
  char emptyByte[2] = {0b00000000, 0b00000000};
//    sendCharsTeensy[0] = 0b00000000;
//    sendCharsTeensy[1] = 0b00000000;
    sendCharsTeensy[2] = 0b11111111;
    sendCharsTeensy[3] = 0b11111111;

  WiFiClient client = server.available();

  if (client) {
    for (int i = 0; i < 48 + 6; i++) {
      receivedCharsTeensy[i] = 0b10;
    }
    for (int i = 0; i < 2; i++) {
      receivedCharsTeensy[i] = 0b11111111;
    }
    for (int i = 42; i < 50; i++) {
      receivedCharsTeensy[i] = 0b1;
    }
    client.flush();
    delay(2000);
    while (client.connected()) {

      char sendChars[10 * sizeof(float) + 8];
      int index = 0;
      //      while(index < 48+6) {
      //        if (Serial.available() > 0) {
      //          receivedCharsTeensy[index] = Serial.read();
      //          index++;
      //        }
      //      }
      memcpy(sendChars, receivedCharsTeensy, sizeof(sendChars));
      sendChars[48] = 0b0;
      client.print(sendChars); //turned off for debugging

      // The sendChar array is not altered, so the issue is 
      // with the client buffer somehow.
//      for (int i = 0; i < 80; i++) {
//        Serial.print(sendChars[i], BIN);
//        Serial.print(" ");
//      }
//      Serial.println();
      
      client.flush();

      char receivedCharsPC[34];
      //      index = 0;
      //      if (client.available() > 0) {
      while (index < 34) {
        if (client.available() > 0) {
          byte inByte = client.read();
          receivedCharsPC[index] = inByte;
          index++;
        }
      }
      //      }
      //
      //      memcpy(sendCharsTeensy, receivedCharsPC, sizeof(sendCharsTeensy));

      //      Serial.print(sendCharsTeensy); //turned off for debugging
      //      Serial.flush();
      //      sendCharsTeensy[0] = receivedCharsPC[0];

      //      for (int i = 2; i < 30; i++) {
      //        Serial.print(receivedCharsPC[i], BIN);
      //        Serial.print(" ");
      //      }
      //      Serial.println();
//          for (int i = 0; i < 2; i++) {
//              Serial.print(emptyByte[i], BIN);
//              Serial.print(" ");
//            }
//            Serial.println();
            for (int i = 0; i < 34; i++) {
              Serial.print(receivedCharsPC[i], BIN);
              Serial.print(" ");
            }
            Serial.println();

            

      //delay(20);


      //        ReadMessageFromTeensy();
      //        if (newDataTeensy == true) {
      //          client.println(receivedCharsTeensy); //turned off for debugging
      //          client.flush();
      //          newDataTeensy = false; //this will be overriden by ReadMessageFromTeensy() in future
      //        }
      //
      //        ReadMessageFromPC(client);
      //        if (newDataPC == true) {
      //           Serial.println(receivedCharsPC);
      //           newDataPC = false; //this will be overriden by ReadMessageFromTeensy() in future
      //        }
    }
    client.stop();
    exit(0);
  }
}
//
//this reads the messages that start with "<" and end with ">"
//these two functions look scary but are nearly idential and simple
//void ReadMessageFromTeensy() {
//    static boolean recvInProgress = false;
//    static byte ndx = 0;
//    char startMarker = '<';
//    char endMarker = '>';
//    char rc;
//
//    while (Serial.available() > 0 && newDataTeensy == false) {
//        rc = Serial.read();
//
//        if (recvInProgress == true) {
//            if (rc != endMarker) {
//                receivedCharsTeensy[ndx] = rc;
//                ndx++;
//                if (ndx >= numChars) {
//                    ndx = numChars - 1;
//                }
//            }
//            if (ndx == numChars)  {
////                receivedCharsTeensy[ndx] = '\0'; // terminate the string
//                recvInProgress = false;
//                ndx = 0;
//                newDataTeensy = true;
//            }
//        }
//
//        else if (rc == startMarker) {
//            recvInProgress = true;
//        }
//    }
//}
////ReadMessageFromTeensy differs from ReadMessageFromTeensy
////by using different soure of where to read info from
//void ReadMessageFromPC(WiFiClient client) {
//    static boolean recvInProgress = false;
//    static byte ndx = 0;
//    char startMarker = '<';
//    char endMarker = '>';
//    char rc;
//
//    while (client.available() > 0 && newDataPC == false) {
//        rc = client.read();
//
//        if (recvInProgress == true) {
//            if (rc != endMarker) {
//                receivedCharsPC [ndx] = rc;
//                ndx++;
//                if (ndx >= numChars) {
//                    ndx = numChars - 1;
//                }
//            }
//            else {
//                receivedCharsPC[ndx] = '\0'; // terminate the string
//                recvInProgress = false;
//                ndx = 0;
//                newDataPC = true;
//            }
//        }
//
//        else if (rc == startMarker) {
//            recvInProgress = true;
//        }
//    }
//}

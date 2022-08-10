/*
 * working two way communication ESP to PC sending strings
*/

#include <ESP8266WiFi.h> 

const char *ssid = "NETGEAR35";   
const char *password = "amberlab"; 

// Set up TCP server and port number
int port = 8888;
WiFiServer server(port);

// Set static IP address, gateway IP address, subnet
IPAddress local_IP(192, 168, 1, 2);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

const byte numChars = 32;
char receivedCharsTeensy[numChars]; //change here to get more charactes in the string
char receivedCharsPC[numChars]; //change here to get more charactes in the string
boolean newDataTeensy = false;
boolean newDataPC = false;

void setup() {
  
  // Begin USB Serial
 delay(100);                    //give time to open and print
  Serial.begin(115200);        //this is for the monitor
  Serial1.begin(115200); 
  delay(100);                 //T41-ESP baud rates must be the same

  // configure static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  // Set to Station mode. ESP8266 connects to desired network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  //beging server
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  
  if (client) {
    
    while(client.connected()){     
      
        client.println("From T41 to PC: "); ReadMessageFromTeensy(); 
        
        //send whenready + debugging
        if (newDataTeensy == true) {
           client.print(receivedCharsTeensy); //client.println() is client.write() for strings and with endl
           Serial.println(receivedCharsTeensy); //for debugging
           newDataTeensy = false; //this will be overriden by ReadMessageFromTeensy() in future
        }
        
        client.println("From PC to T41: "); ReadMessageFromPC(client);
        if (newDataPC == true) {
           client.print(receivedCharsPC); //client.println() is client.write() for strings and with endl
           Serial.println(receivedCharsPC); //for debugging
           newDataPC = false; //this will be overriden by ReadMessageFromTeensy() in future
        }
        delay(1000);
        
    }
    client.stop();
  }
}  

//this reads the messages that start with "<" and end with ">"
//these two functions look scary but are nearly idential and simple
void ReadMessageFromTeensy() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial1.available() > 0 && newDataTeensy == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedCharsTeensy[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedCharsTeensy[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newDataTeensy = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//ReadMessageFromTeensy differs from ReadMessageFromTeensy 
//by using different soure of where to read info from
void ReadMessageFromPC(WiFiClient client) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (client.available() > 0 && newDataPC == false) {
        rc = client.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedCharsPC [ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedCharsPC[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newDataPC = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

/*
 * working two way communication ESP to PC sending strings
*/

#include <ESP8266WiFi.h> //lib for esp chipp

const char *ssid = "NETGEAR35";   
const char *password = "amberlab"; 

// Set up TCP server and port number
int port = 8888;
WiFiServer server(port);

// Set static IP address, gateway IP address, subnet
IPAddress local_IP(192, 168, 1, 2); //this will be assigned to the ESP
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

const byte numChars = 32; //size of the message we expect to receive
char receivedCharsTeensy[numChars];
char receivedCharsPC[numChars]; 
boolean newDataTeensy = false;
boolean newDataPC = false;

void setup() {
  
  // Begin USB Serial
  delay(100);                    //give time to open serial
  Serial.begin(115200);        //there is only one serial on esp!!!
  // wait for serial port to connect. Needed for native USB
  while (!Serial) {;}
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
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW); //to start the diode on ESP turn it to low
}

void loop() {
  
  WiFiClient client = server.available();
  
  if (client) {
    
    while(client.connected()){  

        client.print("-------From T41 to PC-------"); ReadMessageFromTeensy();
        client.print("Is serial1 ready:"); client.println(Serial);
        client.print("Is there data in buffor:"); client.println(Serial.available());
        client.print("Is there a MESSEAGE in buffir:"); client.println(newDataTeensy);
        //send when ready
        if (newDataTeensy == true) {
          client.print("message T41 to ESP:"); client.println(receivedCharsTeensy);
           newDataTeensy = false; //this will be overriden by ReadMessageFromTeensy() in future
        }
        
        client.println("-------From PC to T41-------"); ReadMessageFromPC(client);
        client.print("Is client available"); client.println(client.available());
        client.print("new data from PC:"); client.println(newDataPC);
        if (newDataPC == true) {
           client.print("message PC to ESP:"); client.println(receivedCharsPC); //for debugging
           Serial.println(receivedCharsPC);
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

    while (Serial.available() > 0 && newDataTeensy == false) {
        rc = Serial.read();

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



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

void setup() {
  
  // Begin USB Serial
 delay(100);                    //give time to open and write
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
       
      while(Serial1.available() > 0){

        //send string states, ESP8266 ->  PC
        while (Serial1.available() > 0) {
          char c1 = Serial1.read();
          client.write(c1);
        }
        // client.write("from esp"); //erase 1 from all serials

         //send string torques, T41 <- ESP8266
        while (client.available()) {
          char c = client.read();
          Serial1.write(c);
        }
        delay(0); //adjust this during execution
        }
    }
    client.stop();
  }  
}

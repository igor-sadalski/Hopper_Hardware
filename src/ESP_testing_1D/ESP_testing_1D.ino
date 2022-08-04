#include <ESP8266WiFi.h>

#define BAUD 115200 

int port = 8888;  //Port number
WiFiServer server(port);
// Set random static IP address, gateway IP address, subnet
IPAddress local_IP(192, 168, 1, 2); //ip to which pc should connect
IPAddress gateway(192, 168, 1, 1); //randomly allocated
IPAddress subnet(255, 255, 0, 0); //randomly allocated

//Server connect to WiFi Network
const char *ssid = "NETGEAR35";        //Enter your wifi SSID
const char *password = "amberlab";  //Enter your wifi Password

int count=0;

void setup() 
{
  Serial.begin(BAUD); //setup baud rate must be the same as for the pc receiver
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //Connect to wifi
 
  // Wait for connection  
  Serial.println("Connecting to Wifi");
  while (WiFi.status() != WL_CONNECTED) {   
    delay(500);
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); //shouldnt we spacify here address again?
  server.begin();
  Serial.print("Open Telnet and connect to IP:");
  Serial.print(WiFi.localIP());
  Serial.print(" on port ");
  Serial.println(port);
}

void loop() 
{
  //=======================================================
  //                 RX/TX RECEPTION
  //=======================================================
  
  //=======================================================
  //                 WIFI SENDING
  //=======================================================
  WiFiClient client = server.available();
  
  if (client) {
    if(client.connected())
    {
      Serial.println("Client Connected");
    }
    
    while(client.connected()){      
      while(client.available()>0){
        // read data from the connected client
        Serial.write(client.read()); 
      }
      //Send Data to connected client
      while(Serial.available()>0)
      {
        client.write(Serial.read());
      }
    }
    client.stop();
    Serial.println("Client disconnected");    
  }
}

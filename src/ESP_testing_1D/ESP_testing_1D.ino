/*
 * working two way communication ESP to Telnet
 * with additional info about processes in between 
 * two machines
 */

#include <ESP8266WiFi.h> 

// Info for which network ESP8266 will connect to
const char *ssid = "NETGEAR35";        //Enter your wifi SSID
const char *password = "amberlab";  //Enter your wifi Password

// Set up TCP server and port number
int port = 8888;
WiFiServer server(port);

// Set static IP address, gateway IP address, subnet
IPAddress local_IP(192, 168, 1, 2);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

//=====================================================================
//      WiFi Setup
//=====================================================================

void setup() {
  
  // Begin USB Serial
  Serial.begin(115200);

  // configure static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  // Set to Station mode. ESP8266 connects to desired network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait until connection to desired network is confirmed
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to "); 
    Serial.print(ssid); Serial.println("..."); 
    delay(500);
  }

  // Succeeded in connecting to desired network (i.e, local area network LAN)
  Serial.println();
  Serial.println("Success!");
  Serial.print("Conneceted to "); Serial.println(ssid);

  // Turn on built-in LED to show successful connection
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Begin TCP server after succesful connection
  server.begin();

  // Instructions to connect to Wifi
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println("Open Telnet and connect to IP on terminal:");
  Serial.print("$ telnet "); Serial.print(WiFi.localIP());
  Serial.print(" "); Serial.println(port);
}


//=====================================================================
//      Client - Server Interactions
//=====================================================================

void loop() {
  
  // Gets a client that is connected to the server 
  // and has data available for reading
  WiFiClient client = server.available();

  // check if client is connected (your PC)
  if (client) {

    // print confirmation message
    if(client.connected()) {
      Serial.println("Client Connected:");
      Serial.println(client);
    }

    // routine while client is connected (BIDIRECTIONAL COMMS HERE!)
    while(client.connected()){      

      // Receive data from connected client
      // Bytes available coming from client
      while(client.available() > 0) {
        // Serial.write(): send series of bytes to serial port
        // client.read(): read next byte from client data
        Serial.write(client.read()); 
      }
      
      // Send data to connected client
      // Arduino has bytes to send to client
      while(Serial.available() > 0) {
        // client.write(): send series of bytes to server for client to see
        // Serial.read(): read bytes from Serial data
        client.write(Serial1.read());
        delay(1000);
      }
    }
    client.stop();
    Serial.println("Client disconnected");    
  }  
  
}

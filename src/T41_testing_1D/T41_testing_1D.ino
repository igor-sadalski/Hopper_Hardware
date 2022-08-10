const byte numChars = 32;
char receivedChars[numChars]; //change here to get more charactes in the string
boolean newData = false;

void setup() {
  delay(100); //give time to open and print
  Serial.begin(115200); //this is for the monitor
  Serial1.begin(115200); //baud rates must be the same
  while (!Serial1) {;}
  delay(100);
  
  //start a diode to be sure all is working
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
}

void loop() {
    Serial1.println("<teensyteensyteensyteensy>");
    Serial.println("-----Receiving from ESP << PC------ "); 
    ReadMessage(); 
    if (newData == true) {
       Serial.println(receivedChars);
       newData = false;
    }
    
    delay(1000);
}

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

    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read(); //read in byte

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

const byte numChars = 32;
char receivedChars[numChars]; //change here to get more charactes in the string
boolean newData = false;

void setup() {
  delay(100); //give time to open and print
  Serial.begin(115200); //this is for the monitor
  Serial1.begin(115200); //baud rates must be the same
  delay(100);
}

void loop() {
    Serial.println("Sending message to ESP/PC");
    Serial1.println("<from teensy>");
    Serial.println("Receiving from PC/ESP: "); ReadMessage(); 
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
    char startMarker = '{'; //terminating conditions
    char endMarker = '}'; //terminating conditions
    char rc; //byte we are reading

    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read(); //read in byte

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc; //add char that is in <...> to out string
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1; //if outside string size limit go back
                }
            }
            else {
                receivedChars[ndx] = '\0'; //terminate that char
                recvInProgress = false; //what we received is not "<"
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

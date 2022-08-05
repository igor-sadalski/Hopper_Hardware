const unsigned int MAX_MESSAGE_LENGTH = 50;

void setup() {
  delay(100); //give time to open and write
  Serial.begin(115200); //this is for the monitor
  Serial1.begin(115200); //chekc if Serial1 baud is the same!
  delay(100);
}

void loop() {
 double num = 3.141592653589793238;

  // create pointer with 8 bytes of double's memory
  byte * num_bytes = (byte *) &num;

  // for sending a number, select indicators for bytes
  char delimiter = ',';

  // ================================================================================
  //   SENDING DATA
  // ================================================================================
  // Start new double to send

  // Some delimeter to distinguish what part of buffer you are on
  Serial1.write(delimiter);


  // Send all 8 bytes of double to Serial buffer
  int i = 0;
  while (i < sizeof(num)) {
    Serial1.write(num_bytes[i]);
    i++;
  }
}

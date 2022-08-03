/* 
 *  Sergio Esteban (sesteban@cpp.edu)
 *  To be uploaded to Teensy 4.1
 *  Send a double and receive it back
 *  Use RX/TX 1
 *  UPLOAD TO TEENSY
 *  CHANGE: tools>>board>>teensyuduino>>teensy4.1
 *  THIS WILL GO THE THE KOIOS_MVP program
 */

void setup() {
  // Begin the Serial at 115200 Baud
  Serial.begin(115200);
  Serial1.begin (115200);

  // Give time to setup Serial
  delay(100);

  // Turn LED on to indicate that Teensy is on
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop(){
  
  Serial.println("-----------------------------------------");
  
  // choose double to send
  double num = 3.141592653589793238;
  //double num = Serial.read();

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

  Serial.print("Serial 1 just sent double: ");
  Serial.println(num,20);
  
  delay(1000);
  
} 

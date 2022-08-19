# Hopper_Hardware

Dependencies: 
* [Teensyduino](https://www.pjrc.com/teensy/td_download.html)
* [ESP 8266](https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/)
* Run ``sudo usermod -a -G tty yourname``
* Run ``sudo usermod -a -G dialout yourname``
* Reboot the computer
* In arduino IDE, got to File->preferences, change sketchbook location to be ${PATH_TO_REPO}/src

Steps to make this work:
1. Flash the esp with the esp code
* when you are flashing the esp, COMPLETELY UNPLUG IT FROM THE ROBOT 
* plug in the USB to the computer
* Go to Tools->Board->ESP8266->NodeMCU1.0
* Tools->Port->USB0
* Upload
2. Flash the teensy with the teensy code
* Board->Teensyduino->Teensy4.1
* Port USB0
3. Start the server code just before the robot reboots

roslaunch vrpn_client_ros fast.launch server:=192.168.1.3


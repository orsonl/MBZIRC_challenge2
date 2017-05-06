In order to upload the grippper code to the Arbotix board, do the following:

== Full Instructions ==
http://learn.trossenrobotics.com/projects/182-arbotix-getting-started-guide-arduino-ide-1-6-x-setup.html

== Brief Instruction ==
1 - Install Arduino (v1.8.1 works, latest version as of 2017-02-09)

2 - If you're on windows, install the FTDI drivers:
    http://www.ftdichip.com/Support/Documents/InstallGuides.htm

3 - Download the arbotix libraries for Arduino:
    https://github.com/Interbotix/arbotix/archive/arduino-1-6.zip

4 - Unpack them to "~/Documents/Arduino/" (Linux) or "Documents\Arduino\" (Windows)

5 - In the Arduino IDE, select:
    Tools > Board > Arbotix Std
    Tools > Programmer > AVRISP mkII (serial)

6 - Make sure the jumper on the Arbotix board is connected to "USB" to allow uploading, not "Vin"

7 - Once the code is uploaded, move the jumper to "Vin"


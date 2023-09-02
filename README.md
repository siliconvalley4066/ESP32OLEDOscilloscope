# ESP32OLEDOscilloscope
ESP32 Oscilloscope for OLED and wireless WEB display

<img src="DSC00021.jpg">

This displays an oscilloscope screen both on a 128x64 OLED and also on the WEB page simultaneusly.
The settings are controled by four tactile switches and also on the WEB page.
You can view the oscilloscope screen on the WEB browser of the PC or the tablet or the smartphone.

For WEB operations, edit the source code WebTask.ino and replace your Access Point and the password.
<pre>
Edit:
const char* ssid = "XXXX";
const char* pass = "YYYY";
To:
const char* ssid = "Your Access Point";
const char* pass = "Your Password";
</pre>
Develop environment is:<br>
Arduino IDE 1.8.19<br>
ESP32 by Espressif Systemsm version 2.0.11<br>

Libraries:<br>
Adafruit_SSD1306<br>
Adafruit_SH110X<br>
arduinoFFT by Enrique Condes 1.6.1<br>
arduinoWebSockets from https://github.com/Links2004/arduinoWebSockets<br>

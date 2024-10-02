# OttoDIY_ESP32
Arduino IDE sketch for an Otto DIY Humanoid Robot built around the ESP32 SoC.

The standard [Otto DIY robot](https://www.ottodiy.com/) makes use of an Arduino Nano (ATmega328), which is quite limited in terms of program space.
Replacing the nano with an ESP32, we can load more extensive programs and make use of the additional ESP32 features such as the onboard WiFi module for OTA (Over-the-air) updates and the built-in Bluetooth module for controlling the robot via an app.

As an example, this sketch features the following functionalities:
* After an initial upload via USB-C, any subsequent revisions of the sketch can be uploaded wirelessly via WiFi OTA. This is especially handy in case of ESP32 versions which otherwise require an onboard button to be depressed while uploading via USB-C. Note that WiFi is turned off once you press the touch sensor to switch over to one of the following modes:
* App mode: Otto can be fully controlled via bluetooth using the app (see android_app folder).
* Detect mode: Otto makes sound and dances whenever movement is detected.
* Reply mode: Otto answers (with a random sound and mouth animation) whenever sound is detected.
* Force mode: When placing your hand at a short distance in front of Otto, Otto will move towards the hand. If you move too closely, Otto will move backwards.
* Song mode: Otto will sing a random song whenever sound is detected.
* Dance mode: Otto will dance whenever sound is detected.
* Avoid mode: Otto walks forwards until an obstacle is detected and will then move to avoid it.

The Otto Humanoid build process is detailed in [this instructable](https://www.instructables.com/Otto-DIY-Humanoid-Robot/).
Instead of the Arduino Nano, I used a 30-pin (instead of the longer 38-pin) ESP32 SoC on a modified mini breadboard, as explained [here](https://www.pangodream.es/breadboard-adapter-for-esp32-dev-board).
It may also be possible (and easier) to use an [Arduino Nano ESP32](https://store.arduino.cc/products/nano-esp32) on the standard Nano Shield I/O board, though I have not tried this myself.

Before loading the sketch, you will need to update it with the pin numbers your peripherals are connected to and either set your WiFi connection credentials or disable WiFi by commenting out `#define ENABLE_WIFI`.

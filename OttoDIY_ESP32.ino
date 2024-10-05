#define SS_USE_NIMBLE true
#include <vector>
#include <Otto.h>
#include <ESP32Servo.h>
#include <NonBlockingRtttl.h>
#include <ArduinoOTA.h>
#include <SerialCommand.h>
#include <BLESerial.h>

// comment out the line below to disable WiFi OTA updates
#define ENABLE_WIFI
#ifdef ENABLE_WIFI
  #include <WiFi.h>
  // if WiFi enabled, set your network SSID and password (empty if anonymous access allowed)
  const char* ssid = "Belkin54g";
  const char* password = "";
#endif

// update with the pin numbers your peripherals are connected to
#define PIN_SG90_1 2 // left leg pin
#define PIN_SG90_2 33 // right leg pin
#define PIN_SG90_3 4 // left foot pin
#define PIN_SG90_4 5 // right foot pin
#define PIN_SG90_5 26 // left arm pin
#define PIN_SG90_6 27 // right arm pin
#define BUZZER_PIN 13
#define TOUCH_SENSOR_PIN 14
#define SOUND_SENSOR_PIN 23
#define US_ECHO_PIN 21 // ultrasonic sensor echo pin
#define US_TRIGGER_PIN 22 // ultrasonic sensor trigger pin
#define CLK 15 // 8x8 LED Matrix Clock pin
#define CS 16  // 8x8 LED Matrix Chip Select pin
#define DIN 17 // 8x8 LED Matrix Data In pin
#define Orientation 1 // 8x8 LED Matrix orientation  Top  = 1, Bottom = 2, Left = 3, Right = 4

// Bluetooth settings
String device_name = "OTTO";
BLESerial<> SerialBLE;
SerialCommand SCmd(SerialBLE);

Otto Otto;
Servo servo_left_arm;
Servo servo_right_arm;

int T = 1000;
int moveId = 0;
int moveSize = 15;
void receiveStop() 
{ sendAck(); Otto.home(); sendFinalAck(); }
void receiveLED()
{ sendAck(); Otto.home(); unsigned long int matrix; char *arg; char *endstr; arg = SCmd.next(); if (arg != NULL) { matrix = strtoul(arg, &endstr, 2); Otto.putMouth(matrix, false); } else { Otto.putMouth(xMouth); delay(2000); Otto.clearMouth(); } sendFinalAck(); }
void recieveBuzzer() 
{ sendAck(); Otto.home(); bool error = false; int frec; int duration; char *arg; arg = SCmd.next(); if (arg != NULL) frec = atoi(arg); else error = true; arg = SCmd.next(); if (arg != NULL) duration = atoi(arg); else error = true; if (error == true) { Otto.putMouth(xMouth); delay(2000); Otto.clearMouth(); } else Otto._tone(frec, duration, 1); sendFinalAck(); }
void receiveTrims() 
{ sendAck(); Otto.home(); int trim_YL, trim_YR, trim_RL, trim_RR; bool error = false; char *arg; arg = SCmd.next(); if (arg != NULL) trim_YL = atoi(arg); else error = true; arg = SCmd.next(); if (arg != NULL) trim_YR = atoi(arg); else error = true; arg = SCmd.next(); if (arg != NULL) trim_RL = atoi(arg); else error = true; arg = SCmd.next(); if (arg != NULL) trim_RR = atoi(arg); else error = true; if (error == true) { Otto.putMouth(xMouth); delay(2000); Otto.clearMouth(); } else { Otto.setTrims(trim_YL, trim_YR, trim_RL, trim_RR); Otto.saveTrimsOnEEPROM(); } sendFinalAck(); }
void receiveServo() 
{ sendAck(); moveId = 30; bool error = false; char *arg; int servo_YL, servo_YR, servo_RL, servo_RR; arg = SCmd.next(); if (arg != NULL) servo_YL = atoi(arg); else error = true; arg = SCmd.next(); if (arg != NULL) servo_YR = atoi(arg); else error = true; arg = SCmd.next(); if (arg != NULL) servo_RL = atoi(arg); else error = true; arg = SCmd.next(); if (arg != NULL) { servo_RR = atoi(arg); } else error = true; if (error == true) { Otto.putMouth(xMouth); delay(2000); Otto.clearMouth(); } else { int servoPos[4] = {servo_YL, servo_YR, servo_RL, servo_RR}; Otto._moveServos(200, servoPos); } sendFinalAck(); }
void receiveMovement() 
{ sendAck(); if (Otto.getRestState() == true) Otto.setRestState(false); char *arg; arg = SCmd.next(); if (arg != NULL) moveId = atoi(arg); else { Otto.putMouth(xMouth); delay(2000); Otto.clearMouth(); moveId = 0; } arg = SCmd.next(); if (arg != NULL) T = atoi(arg); else T = 1000; arg = SCmd.next(); if (arg != NULL) moveSize = atoi(arg); else moveSize = 15; }
void move(int moveId) 
{ bool manualMode = false; switch (moveId) { case 0: Otto.home(); break; case 1: Otto.walk(1, T, 1); break; case 2: Otto.walk(1, T, -1); break; case 3: Otto.turn(1, T, 1); break; case 4: Otto.turn(1, T, -1); break; case 5: Otto.updown(1, T, moveSize); break; case 6: Otto.moonwalker(1, T, moveSize, 1); break; case 7: Otto.moonwalker(1, T, moveSize, -1); break; case 8: Otto.swing(1, T, moveSize); break; case 9: Otto.crusaito(1, T, moveSize, 1); break; case 10: Otto.crusaito(1, T, moveSize, -1); break; case 11: Otto.jump(1, T); break; case 12: Otto.flapping(1, T, moveSize, 1); break; case 13: Otto.flapping(1, T, moveSize, -1); break; case 14: Otto.tiptoeSwing(1, T, moveSize); break; case 15: Otto.bend(1, T, 1); break; case 16: Otto.bend(1, T, -1); break; case 17: Otto.shakeLeg(1, T, 1); break; case 18: Otto.shakeLeg(1, T, -1); break; case 19: Otto.jitter(1, T, moveSize); break; case 20: Otto.ascendingTurn(1, T, moveSize); break; case 21: servo_right_arm.write(160); servo_left_arm.write(20); delay(1000); servo_right_arm.write(90); servo_left_arm.write(90); break; case 22: servo_right_arm.write(160); delay(1000); servo_right_arm.write(90);break;  case 23: servo_left_arm.write(20); delay(1000); servo_left_arm.write(90);break; default: manualMode = true; break; } if (!manualMode) sendFinalAck(); }
void receiveGesture()
{ sendAck(); Otto.home();  int gesture = 0; char *arg; arg = SCmd.next(); if (arg != NULL) gesture = atoi(arg); else     delay(2000); switch (gesture) { case 1: Otto.playGesture(OttoHappy); break; case 2: Otto.playGesture(OttoSuperHappy); break; case 3: Otto.playGesture(OttoSad); break; case 4: Otto.playGesture(OttoSleeping); break; case 5: Otto.playGesture(OttoFart); break; case 6: Otto.playGesture(OttoConfused); break; case 7: Otto.playGesture(OttoLove); break; case 8: Otto.playGesture(OttoAngry); break; case 9: Otto.playGesture(OttoFretful); break; case 10: Otto.playGesture(OttoMagic); break; case 11: Otto.playGesture(OttoWave); break; case 12: Otto.playGesture(OttoVictory); break; case 13: Otto.playGesture(OttoFail); break; default: break; } sendFinalAck(); }
void receiveSing() 
{ sendAck(); Otto.home(); int sing = 0; char *arg; arg = SCmd.next(); if (arg != NULL) sing = atoi(arg); else     delay(2000); switch (sing) { case 1: Otto.sing(S_connection); break; case 2: Otto.sing(S_disconnection); break; case 3: Otto.sing(S_surprise); break; case 4: Otto.sing(S_OhOoh); break; case 5: Otto.sing(S_OhOoh2); break; case 6: Otto.sing(S_cuddly); break; case 7: Otto.sing(S_sleeping); break; case 8: Otto.sing(S_happy); break; case 9: Otto.sing(S_superHappy); break; case 10: Otto.sing(S_happy_short); break; case 11: Otto.sing(S_sad); break; case 12: Otto.sing(S_confused); break; case 13: Otto.sing(S_fart1); break; case 14: Otto.sing(S_fart2); break; case 15: Otto.sing(S_fart3); break; case 16: Otto.sing(S_mode1); break; case 17: Otto.sing(S_mode2); break; case 18: Otto.sing(S_mode3); break; case 19: Otto.sing(S_buttonPushed); break; default: break; } sendFinalAck(); }
void sendAck() 
{ delay(30); SerialBLE.print(F("&&")); SerialBLE.print(F("A")); SerialBLE.println(F("%%")); SerialBLE.flush(); }
void sendFinalAck() 
{ delay(30); SerialBLE.print(F("&&")); SerialBLE.print(F("F")); SerialBLE.println(F("%%")); SerialBLE.flush(); }

void setup() {
  Serial.begin(115200);
  #ifdef ENABLE_WIFI
    OTASetup();
  #endif
  Otto.init(PIN_SG90_1, PIN_SG90_2, PIN_SG90_3, PIN_SG90_4, true, BUZZER_PIN); //Set the servo pins and Buzzer pin
  pinMode(TOUCH_SENSOR_PIN, INPUT);
  pinMode(US_ECHO_PIN, INPUT);    
  pinMode(US_TRIGGER_PIN, OUTPUT); 
  Otto.initMATRIX( DIN, CS, CLK, Orientation);
  servo_left_arm.attach(PIN_SG90_5);
  servo_right_arm.attach(PIN_SG90_6);
  servo_left_arm.write(90);
  servo_right_arm.write(90);

  Otto.writeText("OTTO", 600);
  Otto.putMouth(bigSurprise);  
  Otto.sing(S_cuddly);
  Otto.clearMouth();
}

int state = -1;
const int num_states = 7;
long last_state_switch = 0;

void loop() {
  if (state == -1) { // allow OTA updates until first touch detection
    #ifdef ENABLE_WIFI
        ArduinoOTA.handle();
    #endif
  } 
  // TOUCH SENSOR  
  int touch_state = digitalRead(TOUCH_SENSOR_PIN);
  bool state_changed = false;
  if (touch_state == 1 && (millis() - last_state_switch)>1000) {
    if (state == -1) { 
      #ifdef ENABLE_WIFI
        // turn off wifi at first touch detection
        WiFi.disconnect(); 
        WiFi.mode(WIFI_OFF);
      #endif

      // enable Bluetooth
      SerialBLE.begin(device_name);
      SCmd.addCommand("S", receiveStop);
      SCmd.addCommand("L", receiveLED);
      SCmd.addCommand("T", recieveBuzzer);
      SCmd.addCommand("M", receiveMovement);
      SCmd.addCommand("H", receiveGesture);
      SCmd.addCommand("K", receiveSing);
      SCmd.addCommand("C", receiveTrims);
      SCmd.addCommand("G", receiveServo);
      SCmd.addDefaultHandler(receiveStop);      
    }
    state++;
    state = state % num_states;
    last_state_switch=millis();
    state_changed=true;
    Otto.sing(S_buttonPushed);
    Otto.home();
    servo_left_arm.write(90);
    servo_right_arm.write(90);
  }

  switch (state) {
    case 0: {
      if (state_changed)
        Otto.writeText("APP", 150);
      appMode();
      break;
    }
    case 1: {
      if (state_changed)
        Otto.writeText("DETECT", 150);
      detectMode();
      break;
    }
    case 2: {
      if (state_changed)
        Otto.writeText("REPLY", 150);
      replyMode();
      break;
    }
    case 3: {
      if (state_changed)
        Otto.writeText("FORCE", 150);
     forceMode();
      break;
    }
    case 4: {
      if (state_changed)
        Otto.writeText("SONG", 150);
      songMode();
      break;
    }
    case 5: {
      if (state_changed)
        Otto.writeText("DANCE", 150);
      danceMode();
      break;
    }
    case 6: {
      if (state_changed)
        Otto.writeText("AVOID", 150);
      avoidMode();
      break;
    }
  }

}

#ifdef ENABLE_WIFI
void OTASetup() {
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
#endif

long ultrasound() {
   long duration, distance;
   digitalWrite(US_TRIGGER_PIN,LOW);
   delayMicroseconds(2);
   digitalWrite(US_TRIGGER_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(US_TRIGGER_PIN, LOW);
   duration = pulseIn(US_ECHO_PIN, HIGH);
   distance = duration/58;
   return distance;
}

long reply_start=0;
void replyMode() {
  if (digitalRead(SOUND_SENSOR_PIN) == 1)
    reply_start=millis() + 600; // reply after delay
  if (reply_start > 0 && reply_start <= millis()) {
    Otto.putMouth(random(10,31));
    Otto.sing(random(6,19));
    Otto.clearMouth();
    reply_start=0;
  } 
}

void appMode() {    
    SCmd.readSerial(); // read Bluetooth command
    if (Otto.getRestState()==false){ move(moveId); }
}

void detectMode() {
  int distance = ultrasound();
  if(distance >0 && distance<15){
    Otto.putMouth(random(10,12));
    Otto.sing(random(11,13));
    Otto.playGesture(OttoVictory);
    Otto.flapping(2, 1000, 15,1);
    Otto.shakeLeg (1,2000, 1);
    Otto.home();
    Otto.clearMouth();
  }
}

 std::vector<char*> songs = {
    "Super Mario - Main Theme:d=4,o=5,b=125:a,8f.,16c,16d,16f,16p,f,16d,16c,16p,16f,16p,16f,16p,8c6,8a.,g,16c,a,8f.,16c,16d,16f,16p,f,16d,16c,16p,16f,16p,16a#,16a,16g,2f,16p,8a.,8f.,8c,8a.,f,16g#,16f,16c,16p,8g#.,2g,8a.,8f.,8c,8a.,f,16g#,16f,8c,2c6",
    "The Simpsons:d=4,o=5,b=160:c.6,e6,f#6,8a6,g.6,e6,c6,8a,8f#,8f#,8f#,2g,8p,8p,8f#,8f#,8f#,8g,a#.,8c6,8c6,8c6,c6",
    "Indiana:d=4,o=5,b=250:e,8p,8f,8g,8p,1c6,8p.,d,8p,8e,1f,p.,g,8p,8a,8b,8p,1f6,p,a,8p,8b,2c6,2d6,2e6,e,8p,8f,8g,8p,1c6,p,d6,8p,8e6,1f.6,g,8p,8g,e.6,8p,d6,8p,8g,e.6,8p,d6,8p,8g,f.6,8p,e6,8p,8d6,2c6",
    "Xfiles:d=4,o=5,b=125:e,b,a,b,d6,2b.,1p,e,b,a,b,e6,2b.,1p,g6,f#6,e6,d6,e6,2b.,1p,g6,f#6,e6,d6,f#6,2b.,1p,e,b,a,b,d6,2b.,1p,e,b,a,b,e6,2b.,1p,e6,2b.",
    "Looney:d=4,o=5,b=140:32p,c6,8f6,8e6,8d6,8c6,a.,8c6,8f6,8e6,8d6,8d#6,e.6,8e6,8e6,8c6,8d6,8c6,8e6,8c6,8d6,8a,8c6,8g,8a#,8a,8f",
    "Bond:d=4,o=5,b=80:32p,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d#6,16d#6,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d6,16c#6,16c#7,c.7,16g#6,16f#6,g#.6",
    "GoodBad:d=4,o=5,b=56:32p,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,d#,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,c#6,32a#,32d#6,32a#,32d#6,8a#.,16f#.,32f.,32d#.,c#,32a#,32d#6,32a#,32d#6,8a#.,16g#.,d#",
    "TopGun:d=4,o=4,b=31:32p,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,16f,d#,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,g#",
    "Imperial:d=4,o=5,b=112:8g,16p,8g,16p,8g,16p,16d#.,32p,32a#.,8g,16p,16d#.,32p,32a#.,g,8p,32p,8d6,16p,8d6,16p,8d6,16p,16d#.6,32p,32a#.,8f#,16p,16d#.,32p,32a#.,g,8p,32p,8g6,16p,16g.,32p,32g.,8g6,16p,16f#.6,32p,32f.6,32e.6,32d#.6,16e6,8p,16g#,32p,8c#6,16p,16c.6,32p,32b.,32a#.,32a.,16a#,8p,16d#,32p,8f#,16p,16d#.,32p,32g.,8a#,16p,16g.,32p,32a#.,d6,8p,32p,8g6,16p,16g.,32p,32g.,8g6,16p,16f#.6,32p,32f.6,32e.6,32d#.6,16e6,8p,16g#,32p,8c#6,16p,16c.6,32p,32b.,32a#.,32a.,16a#,8p,16d#,32p,8f#,16p,16d#.,32p,32g.,8g,16p,16d#.,32p,32a#.,g",
    "AxelF:d=4,o=6,b=125:e5,8g.5,8e5,16e5,8a5,8e5,8d5,e5,8b.5,16e5,8c,8b5,8g5,8e5,8b5,8e,16e5,8d5,16d5,8b5,16f_5,e.5,2p,8g.5,8e5,16e5,8a5,8e5,8d5,e5,8b.5,8e5,16e5,8c,8b5,8g5,8e5,8b5,8e,16e5,8d5,16d5,8b5,8f_5,e.5",
    "Airwolf Theme:d=4,o=6,b=100:e5,16a5,16b5,16d,e,16g,16f_,16d,e,16g,16f_,16d,e,8d,16f_,b5,a5,8g5,16a5,8f_5,16d5,g5,16c,16d,16f,g,16c,16b,16f,g,16c,16b,16f,g,8f,16a,d,c,8b5,16d,8a5,16f5,g5,16c,16d,16f,g,16c,16b,16f",
    "James Bond Theme : d=4,o=5,b=140:8e,16f#,16f#,8f#,f#,8e,8e,8e,8e,16g,16g,8g,g,8f#,8f#,8f#,8e,16f#,16f#,8f#,f#,8e,8e,8e,8e,16g,16g,8g,g,8f#,8f,8e,8d#6,2d.6,8b,8a,1b,",
    "StarWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#6",
    "A-Team:d=8,o=5,b=125:4d#6,a#,2d#6,16p,g#,4a#,4d#.,p,16g,16a#,d#6,a#,f6,2d#6,16p,c#.6,16c6,16a#,g#.,2a#",
    "Flinstones:d=4,o=5,b=40:32p,16f6,16a#,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,d6,16f6,16a#.,16a#6,32g6,16f6,16a#.,32f6,32f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,a#,16a6,16d.6,16a#6,32a6,32a6,32g6,32f#6,32a6,8g6,16g6,16c.6,32a6,32a6,32g6,32g6,32f6,32e6,32g6,8f6,16f6,16a#.,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#6,16c7,8a#.6",
    "Smurfs:d=32,o=5,b=200:4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8b,p,4g#,16p,4c#6,p,16a#,p,8f#,p,8a#,p,4g#,4p,g#,p,a#,p,b,p,c6,p,4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8b,p,4g#,16p,4c#6,p,16a#,p,8b,p,8f,p,4f#",
    "LeisureSuit:d=16,o=6,b=56:f.5,f#.5,g.5,g#5,32a#5,f5,g#.5,a#.5,32f5,g#5,32a#5,g#5,8c#.,a#5,32c#,a5,a#.5,c#.,32a5,a#5,32c#,d#,8e,c#.,f.,f.,f.,f.,f,32e,d#,8d,a#.5,e,32f,e,32f,c#,d#.,c#",
    "MissionImp:d=16,o=6,b=95:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d"
};
int songIndex = random(0,songs.size()); //which song to play when the previous one finishes
void songMode() {
  if ( !rtttl::isPlaying() )
  {
      servo_left_arm.write(90);
      servo_right_arm.write(90);
      Otto.clearMouth();    
      delay(1000);
      Otto.putMouth(bigSurprise);
      servo_left_arm.write(0);
      servo_right_arm.write(180);
      rtttl::begin(BUZZER_PIN, songs[songIndex]);
      songIndex=(songIndex+1) % songs.size(); //ready for next song
  }
  else
  {
    rtttl::play();
  }
}

void avoidMode() {
  int distance = ultrasound();
  if(distance >0 && distance<15){
    Otto.sing(S_surprise);
    Otto.playGesture(OttoConfused);
    Otto.walk(2,1000,-1); // BACKWARD x2
    Otto.turn(3,1000,1); // LEFT x3
    delay(50);
  } else {
    Otto.walk(1,1000,1); // FORWARD x1
  }
}

void forceMode() {
  int distance = ultrasound();
   if (distance > 0 && distance <= 10) {
      Otto.walk(1,1000,-1); // BACKWARD
    }
    if (distance > 10 &&  distance < 15) {
      Otto.home();
    }
    if (distance > 15 &&  distance < 30) {
      Otto.walk(1,1000,1); // FORWARD
    }
    if (distance < 0 || distance > 30) {
      Otto.home();
    }
}

int adj[]={ 0, 0,};
int pos[]={ 90,90};
int shift = 60;
int shift_inc = 10;
int shift_delay = 50;
void move_servo(){ servo_left_arm.write(pos[1]+adj[1]); servo_right_arm.write(pos[2]+adj[2]);}
void danceMode() {
  if (digitalRead(SOUND_SENSOR_PIN) == 1) {
    Otto.moonwalker(1, 1000, 25, 1);
    servo_left_arm.write(160);
    servo_right_arm.write(20);
    delay(shift_delay);Otto.playGesture(OttoMagic);
    Otto.moonwalker(1, 1000, 25, -1);
    servo_left_arm.write(20);
    servo_right_arm.write(160);
    delay(shift_delay);Otto.putMouth(happyOpen);
    Otto.crusaito(1, 1000, 25, 1);
    for(int angle=90; angle<90+shift; angle+=shift_inc){  pos[1] = angle;    move_servo();  delay(shift_delay);}
    for(int angle=90+shift; angle>90-shift; angle-=shift_inc) { pos[1] = angle;  move_servo(); delay(shift_delay); }
    for(int angle=90-shift; angle<90; angle+=shift_inc) {pos[1] = angle;  move_servo();   delay(shift_delay); }
    Otto.putMouth(happyOpen);
    Otto.crusaito(1, 1000, 25, -1);
    for(int angle=90; angle<90+shift; angle+=shift_inc){  pos[2] = angle;    move_servo();  delay(shift_delay);}
    for(int angle=90+shift; angle>90-shift; angle-=shift_inc) { pos[2] = angle;  move_servo(); delay(shift_delay); }
    for(int angle=90-shift; angle<90; angle+=shift_inc) {pos[2] = angle;  move_servo();   delay(shift_delay); }
    Otto.clearMouth();
    Otto.home();
    delay(500);
  }  
}
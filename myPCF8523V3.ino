#define ARDUINO_UNO
#define BUFFER_SIZE 128


#include <Arducam_Mega.h>
#include <SPI.h>
#include <SD.h> 
#include "RTClib.h"
#include <Servo.h>     // for uno  #include <ESP32Servo.h>   // for esp32

Servo myservo;  // create servo object to cHihtrol a servo
Servo evacservo;
// RTC_PCF8523 rtc;
RTC_DS1307 rtc;

//   Variables to control operation  ************************
int startTime1 =13;       // time to start 1st   Will run for 3 hours first round
int startTime2 = 19;      // Time to start   2nd   Will run for 3 hours second round
int minInState1 = 4;      // change to 180  number of minutes in State 1
int nbrPics = 7;          // number of pictures at each half hour Will take pictures at min 29 and 59 of each hour
int nbrPresses = 6;       // number of presses of fluor. powder
int picMinute1 = 29;      // the minute on which to take pictures
int picMinute2 = 59;       // the minute on which to take pictures
// **********************************************************

int minutecnt;
int State;
int hrs;
int mins;
int secs;
int started = 0;

const int arducamCS = 10; 
const int arducamSD = 4;

const uint8_t arducamI2C = 0x78;

Arducam_Mega arducam(arducamCS);

bool sdCardReady = false;


const int funnelFan = 2;   //  tan  12 volt  (YELLOW wire from 12 volt battery)
const int evacFan = 3;     // white 12 volt
const int toMarkFan = A0;   // yellow   12 volt centrigal to bLOW the femals into marking

const int pressor = 6;    //  orange  6 volt (RED wire from battery from 6 volt battery)
const int light = 7;      //  gray  6 volt
const int camera = 8;     //  blue  contact closure
         
//   Vin connects to 6 Volt battery

// servo motor  Connects Yellow - PIN 9    Red - +5V  Brown - gnd
int clse = 90;   // was 0
int opn = 0;     // 90

int evacclse = 90;
int evacopn = 180;

// RTC  Connects Yellow - SDA   Orange - SCL    Red - 3.3v   Brown - GND

void setup () {
  //Serial.begin(115200);
  Serial.begin(19200);

#ifndef ESP8266
  while (!Serial); // wait for serial port to cHihnect. Needed for native USB
#endif

  myservo.attach(9);  // attaches the servo Hih pin 9 for both
  evacservo.attach(5);
  pinMode(arducamCS, OUTPUT);
  pinMode(arducamSD, OUTPUT);
  digitalWrite(arducamCS, HIGH);
  digitalWrite(arducamSD, HIGH);

  pinMode(funnelFan, OUTPUT);
  pinMode(evacFan, OUTPUT);
  pinMode(pressor, OUTPUT);
  pinMode(toMarkFan, OUTPUT);
  pinMode(light, OUTPUT);

  Wire.begin();
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find DS1307 RTC on shield."));
    Serial.flush();
    abort();
  } else {
    Serial.println(F("RTC Initialized."));
    if (! rtc.isrunning()) { // NEW: Check if RTC is running
      Serial.println(F("RTC is NOT running. Setting time to compile time!"));
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  Serial.println(F("RTC Ready."));

  // --- Initialize SPI Bus (Essential for both Arducam and SD) ---
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); 
  Serial.println(F("SPI bus initialized."));


  if (arducam.begin() == CAM_ERR_SUCCESS) {
    Serial.println("Camera initialized");
  } else {
    Serial.println("Camera init failed");
  }


  Serial.print(F("Initializing SD card at CS pin: ")); Serial.println(arducamSD); // Will print 4
  if (!SD.begin(arducamSD)) { // Use the defined SD_CS_PIN (D4)
    Serial.println(F("ERROR: SD card initialization failed! No card detected. Is card inserted and formatted FAT16/FAT32?"));
    sdCardReady = false;
  } else {
    Serial.println(F("SD card initialized successfully."));
    sdCardReady = true;
  }

  

  //delay(10000);
  delay(1000);
}


//main
void loop () {


    //delay(5000);
    delay(1000);
    DateTime now = rtc.now();
    hrs = now.hour();
    mins = now.minute();
    secs = now.second();
    Serial.print(F("State:"));
    Serial.print(State);
    Serial.print(F("    Time: "));
    Serial.print(hrs,DEC);
    Serial.print(F(":"));
    Serial.print(mins,DEC);
    Serial.print(F(":"));
    Serial.print(secs,DEC);
    Serial.println();
    RunCheck();
     
    if (State == 0) {
      minutecnt = 0;
      started = 0;
      myservo.write(clse);  //door closed
      
      evacservo.write(0);
      delay(2000);
      evacservo.write(90);
      digitalWrite(funnelFan, LOW);    
      digitalWrite(evacFan, LOW);
      // digitalWrite(camera, LOW);
      digitalWrite(pressor, LOW);
      digitalWrite(toMarkFan, LOW);

      digitalWrite(light, HIGH);
     }

    if (State == 1) {
      myservo.write(clse);  //door closed
      evacservo.write(evacclse);
      digitalWrite(funnelFan, HIGH);    // Hih = Hih  LOW = LOW
      digitalWrite(evacFan, LOW);
      // digitalWrite(camera, LOW);
      digitalWrite(pressor, LOW);
      digitalWrite(toMarkFan, LOW);
      
      Serial.println(F("TRap Fan On"));
      Serial.println(F("Light and PICS may be On"));
      
      
      RunCheck();     
      if (secs < 5){
        minutecnt = minutecnt + 1;
        Serial.print(minutecnt);
        Serial.println(F(" added a minute"));
      }
      
     chkPics();
     // delay(5000);
     State = 2;

      if (minutecnt > minInState1){                          // 180 minutes
         Serial.print(F("minutecnt="));
         Serial.println(minutecnt);
         minutecnt = 0;
         State = 2;
      }
    }  // end State 1


    if (State == 2){                        // evac
        Serial.println(F("State 2"));
        myservo.write(opn);
        evacservo.write(evacclse);
        delay(1000);
        digitalWrite(funnelFan, HIGH);  
        digitalWrite(evacFan, LOW);
        // digitalWrite(camera, LOW);
        digitalWrite(pressor, LOW);
        digitalWrite(toMarkFan, HIGH);

        Serial.println(F("Door is OPEN"));
        Serial.println(F("Trap Fan On"));
        Serial.println(F("ToMark Fan On"));

        delay(4000);  // this gives time for door to open and marking fan to get to full speed
        digitalWrite(funnelFan, LOW); 
        
        
        
        delay(10000);              //30,000 is 30 secHihds for toMarkFan to run
        
        State = 3;
  }  // end State 2


if (State == 3){               // pressor to spray fluorescent powder
      Serial.println(F("State 3"));
      myservo.write(clse);  
      evacservo.write(evacclse);
      digitalWrite(funnelFan, LOW);
      digitalWrite(evacFan, LOW);
      // digitalWrite(camera, LOW);
      digitalWrite(toMarkFan, LOW);
      delay(5000);
      Serial.println(F("Pressor On"));

      // Turn on the air compressor
      digitalWrite(pressor, HIGH);
      delay(1000); // <-- Adjust this delay: Duration to run the compressor


      // Turn off the air compressor
      digitalWrite(pressor, LOW);
         
      delay(3000);  // hanging around in chamber before being evacuated
      
      State = 4;
  }

    if (State == 4){                   // toMark fan to toMark mosquitoes
        Serial.println(F("State 4"));
        myservo.write(clse);  
        evacservo.write(evacopn);
        digitalWrite(funnelFan, LOW);
        digitalWrite(evacFan, HIGH);
        // digitalWrite(camera, LOW);
        digitalWrite(pressor, LOW);
        digitalWrite(toMarkFan, LOW);

        Serial.println(F("Evac Fan On"));
        Serial.println(minutecnt);
        delay(10000);
        State = 0;
        started = 0;
        digitalWrite(evacFan, LOW);
        evacservo.write(evacclse);
  }
}
 
void RunCheck(){
  
   if (hrs == startTime1) {
      if (started == 0) {
         Serial.println(F("Start Running"));
         State = 1;
      }
      started=1;
   }

   if (hrs == startTime1+1) {
      if (started == 0) {
         Serial.println(F("Start Running"));
         State = 1;
      }
      started=1;
   } 

   if (hrs == startTime1+2) {
      if (started == 0) {
         Serial.println(F("Start Running"));
         State = 1;
      }
      started=1;
   }
 
   if (hrs == startTime2) {
      if (started == 0) {
         Serial.println(F("Start Running"));
         State = 1;
      }
      started=1;
      Serial.println(started);
   }

   if (hrs == startTime2+1) {
      if (started == 0) {
         Serial.println(F("Start Running"));
         State = 1;
      }
      started=1;
   }

   if (hrs == startTime2+2) {
      if (started == 0) {
         Serial.println(F("Start Running"));
         State = 1;
      }
      started=1;
   }
    
 }

void chkPics(){
  Serial.println(F("ChkPics"));
  RunCheck();

  takePics();

  if (mins == picMinute1) {
      if(secs < 10){ // Take pictures early in the minute to avoid multiple triggers
        takePics();
      }
  }

  if (mins == picMinute2) {
      if(secs < 10){ // Take pictures early in the minute
        takePics();
      }
  }
}

void takePics(){
  Serial.println(F("Starting cam"));
  Serial.flush();
  digitalWrite(light, HIGH);

  for (int k = 0; k < nbrPics; k++) {
    delay(10);   // Small delay to allow I2C lines to settle

    // Generate a unique filename based on RTC time (Date_Time_PhotoNum.JPG)
    DateTime now = rtc.now(); // Get current time from the shield's RTC
    char filename[40]; // Sufficient for IMG_YYMMDD_HHMMSS_PNN.JPG (YYMMDD_HHMMSS_P0.JPG - P99.JPG)
    sprintf(filename, "IMG_%02d%02d%02d_%02d%02d%02d_P%02d.JPG",
            now.year() % 100, now.month(), now.day(),
            now.hour(), now.minute(), now.second(), k);

    Serial.print(F("Attempting to capture: ")); Serial.println(filename);

    digitalWrite(arducamSD, HIGH);   // Disable SD card
    digitalWrite(arducamCS, LOW);    // Enable camera
    delay(10);

    // arducam.setAutoFocus(1);
    Serial.println(F("Before takePicture()"));

    CamStatus camStatus = arducam.takePicture(CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);

    digitalWrite(arducamCS, HIGH);    // Disable Arducam
    digitalWrite(arducamSD, LOW);     // Re-enable SD

    Serial.println(F("After takePicture()"));

    if (camStatus != CAM_ERR_SUCCESS) {
      Serial.print(F("ERROR: Failed to start capture: "));
      Serial.println(camStatus); // Print the error status
      continue; // Skip to next picture if capture fails
    }

    uint32_t totalLen = arducam.getReceivedLength();
    if (totalLen == 0) {
      Serial.println(F("ERROR: No image data captured."));
      continue;
    }

    digitalWrite(arducamCS, HIGH);   // Disable camera
    digitalWrite(arducamSD, LOW);    // Enable SD card

    if (sdCardReady) {
      // Open the file on the SD card to write the image
      File imageFile = SD.open(filename, FILE_WRITE);
      if (!imageFile) {
        Serial.println(F("ERROR: Failed to open file for writing to SD card! Check SD card."));
        continue; // Skip if file cannot be opened
      }

      const int bufSize = 256;
      uint8_t buf[bufSize];
      uint32_t bytesRemaining = totalLen;

      while (bytesRemaining > 0) {
        uint32_t toRead = min(bufSize, bytesRemaining);
        arducam.readBuff(buf, toRead);
        imageFile.write(buf, toRead);
        bytesRemaining -= toRead;
      }

      imageFile.close();
      Serial.print(F("Saved: ")); Serial.println(filename);
    } else {
      Serial.println("WARNING: SD not ready.");
    }
    
    delay(1500); // Delay between taking multiple pictures (was 1s)
  }

  digitalWrite(light, LOW);
  Serial.println(F("Finished Arducam capture sequence."));
}
 

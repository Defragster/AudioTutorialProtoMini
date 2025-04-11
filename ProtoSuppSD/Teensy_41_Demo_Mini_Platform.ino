/*
  Mini Platform for Teensy 4.1 Example

  This program does nothing overly useful, but illustrates some of the basic
  features and checks for presence of PSRAM/Flash memory installed, SD card
  installed and the ESP32-C3. Reports the info to the LCD and serial port.

  If SD card installed, the Audio button plays the wave file
  "SDTEST2.WAV" from the Teensy audio tutorial
  https://www.pjrc.com/teensy/td_libs_AudioDataFiles.html

  The Scan button sends a command to the ESP32S requesting a scan of
  available WiFi networks.  When the ESP32S returns the scan results,
  the Teensy 4.1 updates those results on the LCD screen and serial port.

  The System button just rechecks the same system information.  

  The three Teensy 4.1 user buttons simply turn the RGB LED colors ON/OFF 

  The ST7796 LCD uses the ST7796_t3 library branch from:  
  https://github.com/KurtE/ST7735_t3/tree/ST7796
  
  The FT6336 touch overlay uses the Adafruit_FT6206.h library.

  The onboard 16MB NOR Flash uses this SerialFlash library branch so that it 
  will work properly on SPI1.
  https://github.com/KurtE/SerialFlash/tree/use_ptr_not_reference
  
  This example code is in the public domain.
*/
#include <ST7796_t3.h>
#include <st7735_t3_font_Arial.h>

#include <Adafruit_FT6206.h>

#include <SPI.h>
#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <Bounce2.h>
#include <SerialFlash.h>
#include "LittleFS.h"
extern "C" uint8_t external_psram_size;

AudioPlaySdWav playSdWav1;
AudioOutputI2S i2s1;
AudioConnection patchCord1(playSdWav1, 0, i2s1, 0);
AudioConnection patchCord2(playSdWav1, 1, i2s1, 1);
AudioControlSGTL5000 sgtl5000_1;

// Pins used with the Teensy Audio Shield
#define SDCARD_CS_PIN BUILTIN_SDCARD
#define SDCARD_MOSI_PIN 11  //7
#define SDCARD_SCK_PIN 13   //14

// LCD control pins defined by the baseboard
#define TFT_CS 10
#define TFT_DC 9

// Use main SPI bus MOSI=11, MISO=12, SCK=13 with different control pins
ST7796_t3 tft = ST7796_t3(TFT_CS, TFT_DC);

// Touch screen control pins defined by the baseboard
// TIRQ interrupt if used is on pin 2
//#define TIRQ_PIN  2

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ts = Adafruit_FT6206();

#define BTN1_PIN 22
#define BTN2_PIN 32
#define BTN3_PIN 40

// INSTANTIATE 3 Button OBJECTS
Bounce button1 = Bounce();
Bounce button2 = Bounce();
Bounce button3 = Bounce();

#define RGB_R_PIN 4
#define RGB_G_PIN 6
#define RGB_B_PIN 5

// Define Audio button location and size
#define AUDIO_X 10
#define AUDIO_Y 10
#define AUDIO_W 105
#define AUDIO_H 32

// Define Scan button location and size
#define SCAN_X 10
#define SCAN_Y 70
#define SCAN_W 105
#define SCAN_H 32
#define BUTTON_FONT Arial_14
#define TXT_FONT Arial_12

// Define System checks button location and size
#define SYSTEM_X 150
#define SYSTEM_Y 10
#define SYSTEM_W 105
#define SYSTEM_H 32

#define FLASH_CS 37  //128Mb NOR flash chip CS pin

#define ESP32SERIAL Serial7  // ESP32 is attached to Serial7 port
#define ESP32SERIAL_BUFFER_SIZE 1024
unsigned char esp32SerialBuffer[ESP32SERIAL_BUFFER_SIZE];

// Subroutine prototypes
void SetScanButton(boolean);   // Handles Scan button when touched
void SetAudioButton(boolean);  // Handles Audio button when touched
void SetSystemButton();        // Handes System button when touched
void SystemCheck();

// Misc flags to keep track of things
boolean isTouched = false;        // Flag if a touch is in process
boolean scanRequested = false;    // Flag if WiFi scan is in process
boolean sDCardInstalled = false;  // Flag if SD card installed
boolean audioPlaying = false;     // Flag if audio is currently playing
boolean esp32SAttached = false;   // Flag if ESP32S is attached

//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  Serial.begin(115200);       //Initialize USB serial port to computer
  ESP32SERIAL.begin(115200);  //Initialize Seria1 1 connected to ESP32S
  // Setup extra memory for the ESP32 serial buffer to avoid overruns
  ESP32SERIAL.addMemoryForRead(esp32SerialBuffer, ESP32SERIAL_BUFFER_SIZE);

  // Required to get SPI1 to work with NOR Flash chip
  SPI1.setMOSI(26);
  SPI1.setSCK(27);
  SPI1.setMISO(39);
  SPI1.setCS(37);
  SPI1.begin();

  //Setup buttons as inputs with pullup resistors
  button1.attach(BTN1_PIN, INPUT_PULLUP);
  button2.attach(BTN2_PIN, INPUT_PULLUP);
  button3.attach(BTN3_PIN, INPUT_PULLUP);
  // Set debounce interval to 15mSec
  button1.interval(15);
  button2.interval(15);
  button3.interval(15);

  //Setup RGB pins as outputs
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  digitalWrite(RGB_R_PIN, LOW);
  digitalWrite(RGB_G_PIN, LOW);
  digitalWrite(RGB_B_PIN, LOW);

  // Setup LCD screen
  tft.init(320, 480);
  tft.invertDisplay(true);  // Requires colors to be inverted
  tft.setRotation(3);       // Rotates screen to match the baseboard orientation
                            // Setup touch Screen
                            // Touch Screen default touch threshold of 40
                            // Can change threshold to change touch sensitivity
  if (!ts.begin(40)) {
    Serial.println("Unable to start touchscreen.");
  } else {
    Serial.println("Touchscreen started.");
  }

  tft.fillScreen(ST7735_BLUE);
  tft.setCursor(1, 120);  // Set initial cursor position
  tft.setFont(TXT_FONT);  // Set initial font style and size

  // Draw buttons
  SetAudioButton(false);
  SetScanButton(false);
  SetSystemButton();

  // Setup audio
  if (sDCardInstalled) {  // Setup the audio
    AudioMemory(8);
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);
    SPI.setMOSI(SDCARD_MOSI_PIN);
    SPI.setSCK(SDCARD_SCK_PIN);
  } else {  // If no audio, gray out button
    tft.setCursor(AUDIO_X + 8, AUDIO_Y + 8);
    tft.setFont(BUTTON_FONT);
    tft.setTextColor(ST7735_WHITE);
    tft.fillRoundRect(AUDIO_X, AUDIO_Y, AUDIO_W, AUDIO_H, 4, ST7735_BLACK);
    tft.print("No Audio");
  }

  // Setup ESP32
  if (!esp32SAttached) {  // If no ESP32 gray out button
    tft.setCursor(SCAN_X + 8, SCAN_Y + 8);
    tft.setFont(BUTTON_FONT);
    tft.setTextColor(ST7735_WHITE);
    tft.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 4, ST7735_BLACK);
    tft.print("No Scan");
  }
  // Initialize buttons
  button1.update();
  button2.update();
  button3.update();
}
//===============================================================================
//  Main
//===============================================================================
void loop() {
  static boolean red_LED = false;
  static boolean green_LED = false;
  static boolean blue_LED = false;

  // Keep an eye on any audio that may be playing and reset button when it ends
  if (playSdWav1.isStopped() && audioPlaying) {  // Audio finished playing
    SetAudioButton(false);
    Serial.println("Audio finished playing");
  }
  // Check to see if the touch screen has been touched
  if (ts.touched() && isTouched == false) {
    TS_Point p = ts.getPoint();
    Serial.print("x Raw = ");  // Show our touch coordinates for each touch
    Serial.print(p.x);
    Serial.print(", y Raw = ");
    Serial.print(p.y);
    Serial.println();

    // Map the touch point to the LCD screen
    p.x = map(p.x, 0, 320, 0, 320);  // X coordinate not remapped
    p.y = map(p.y, 0, 480, 480, 0);

    isTouched = true;

    // Look for a Scan Button Hit
    if ((p.y > SCAN_X) && (p.y < (SCAN_X + SCAN_W))) {
      if ((p.x > SCAN_Y) && (p.x <= (SCAN_Y + SCAN_H))) {
        Serial.println("Scan Button Hit");
        if (esp32SAttached) SetScanButton(true);
      }
    }
    // Look for an Audio Button Hit
    if ((p.y > AUDIO_X) && (p.y < (AUDIO_X + AUDIO_W))) {
      if ((p.x > AUDIO_Y) && (p.x <= (AUDIO_Y + AUDIO_H))) {
        Serial.println("Audio Button Hit");
        if (sDCardInstalled && !audioPlaying) {
          SetAudioButton(true);
        } else if (sDCardInstalled && audioPlaying) {
          SetAudioButton(false);
        }
      }
    }
    // Look for a System Button Hit
    if ((p.y > SYSTEM_X) && (p.y < (SYSTEM_X + SYSTEM_W))) {
      if ((p.x > SYSTEM_Y) && (p.x <= (SYSTEM_Y + SYSTEM_H))) {
        Serial.println("System Button Hit");
        if (scanRequested == false) SetSystemButton();
      }
    }
    Serial.print("x = ");  // Show our touch coordinates for each touch
    Serial.print(p.x);
    Serial.print(", y = ");
    Serial.print(p.y);
    Serial.println();
    delay(100);  // Debounce touchscreen a bit
  }
  if (!ts.touched() && isTouched) {
    isTouched = false;  // touchscreen is no longer being touched, reset flag
  }
  // If we requested a scan, look for serial data coming back from the ESP32S
  if (scanRequested && ESP32SERIAL.available()) {
    Serial.print("Read incoming data");
    tft.setCursor(10, 120);
    tft.setFont(TXT_FONT);
    while (ESP32SERIAL.available()) {  // Print the scan data to the LCD & USB
      String returnData = ESP32SERIAL.readString();
      tft.println(returnData);
      Serial.println(returnData);
    }
    scanRequested = false;  // Reset the scan flag and button
    SetScanButton(false);
  }
  // Check on buttons and update RGB LED as required.
  button1.update();
  button2.update();
  button3.update();
  if (button1.fell()) {
    red_LED = !red_LED;
    digitalWrite(RGB_R_PIN, red_LED);
  }
  if (button2.fell()) {
    green_LED = !green_LED;
    digitalWrite(RGB_G_PIN, green_LED);
  }
  if (button3.fell()) {
    blue_LED = !blue_LED;
    digitalWrite(RGB_B_PIN, blue_LED);
  }
}
//===============================================================================
//  Routine to draw Audio button current state and control audio playback
//===============================================================================
void SetAudioButton(boolean audio) {
  tft.setCursor(AUDIO_X + 8, AUDIO_Y + 8);
  tft.setFont(BUTTON_FONT);
  tft.setTextColor(ST7735_WHITE);

  if (!audio) {  // button is set inactive, redraw button inactive
    tft.fillRoundRect(AUDIO_X, AUDIO_Y, AUDIO_W, AUDIO_H, 4, ST7735_RED);
    tft.print("Play Audio");
    audioPlaying = false;
    if (playSdWav1.isPlaying()) {  // Stop any audio that is playing
      playSdWav1.stop();
      Serial.println("Audio being stopped");
    }
  } else {  // button is active, redraw button active
    tft.fillRoundRect(AUDIO_X, AUDIO_Y, AUDIO_W, AUDIO_H, 4, ST7735_GREEN);
    tft.print("Playing");
    audioPlaying = true;
    if (sDCardInstalled && !playSdWav1.isPlaying()) {  // Play audio file
      Serial.println("Audio being played");
      playSdWav1.play("SDTEST2.WAV");
      delay(10);  // wait for library to parse WAV info
    }
  }
}
//===============================================================================
//  Routine to draw scan button current state and initiate scan request
//===============================================================================
void SetScanButton(boolean scanning) {
  tft.setCursor(SCAN_X + 8, SCAN_Y + 8);
  tft.setFont(BUTTON_FONT);
  tft.setTextColor(ST7735_WHITE);

  if (!scanning) {  // Button is inactive, redraw button
    tft.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 4, ST7735_RED);
    tft.print("Scan WiFi");
  } else {                                                    // Button is active, redraw button
    tft.fillRect(1, SCAN_Y + SCAN_H, 360, 240, ST7735_BLUE);  // Clear previous scan
    tft.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 4, ST7735_GREEN);
    tft.print("Scanning");
    ESP32SERIAL.println("S");  // Send command to ESP32 to start scan
    scanRequested = true;      // Set flag that we requested scan
    Serial.println("Scan being requested");
  }
}
//===============================================================================
//  Routine to draw system button current state and initiate system checks
//===============================================================================
void SetSystemButton() {
  tft.setCursor(SYSTEM_X + 8, SYSTEM_Y + 8);
  tft.setFont(BUTTON_FONT);
  tft.setTextColor(ST7735_WHITE);

  tft.fillRect(1, SCAN_Y + SCAN_H, 360, 240, ST7735_BLUE);  // Clear previous info
  tft.fillRoundRect(SYSTEM_X, SYSTEM_Y, SYSTEM_W, SYSTEM_H, 4, ST7735_GREEN);
  tft.print("System");
  Serial.println("System check requested");

  // Can't do SystemCheck when audio playing off SD card
  if (audioPlaying == false) SystemCheck();

  tft.setCursor(SYSTEM_X + 8, SYSTEM_Y + 8);
  tft.setFont(BUTTON_FONT);
  tft.fillRoundRect(SYSTEM_X, SYSTEM_Y, SYSTEM_W, SYSTEM_H, 4, ST7735_RED);
  tft.print("System");
}
//===============================================================================
//  Routine to query installed hardware
//===============================================================================
void SystemCheck() {
  tft.setCursor(1, 120);  // Set initial cursor position
  tft.setFont(TXT_FONT);  // Set initial font style and size

  // Check for PSRAM chip installed
  uint8_t size = external_psram_size;

  if (size == 0) {
    Serial.println("No PSRAM Installed");
    tft.println("No PSRAM Installed");
  } else {
    Serial.printf("PSRAM Memory Size = %d Mbyte\n", size);
    tft.printf("PSRAM Memory Size = %d Mbyte\n", size);
  }

  tft.println();

  LittleFS_QSPIFlash myfs_NOR;  // NOR FLASH
  LittleFS_QPINAND myfs_NAND;   // NAND FLASH 1Gb

  // Check for NOR Flash chip installed
  if (myfs_NOR.begin()) {
    Serial.printf("NOR Flash Memory Size = %d Mbyte / ", myfs_NOR.totalSize() / 1048576);
    Serial.printf("%d Mbit\n", myfs_NOR.totalSize() / 131072);
    tft.printf("NOR Flash Memory Size = %d Mbyte / ", myfs_NOR.totalSize() / 1048576);
    tft.printf("%d Mbit\n", myfs_NOR.totalSize() / 131072);
  }
  // Check for NAND Flash chip installed
  else if (myfs_NAND.begin()) {
    Serial.printf("NAND Flash Memory Size =  %d bytes / ", myfs_NAND.totalSize());
    Serial.printf("%d Mbyte / ", myfs_NAND.totalSize() / 1048576);
    Serial.printf("%d Gbit\n", myfs_NAND.totalSize() * 8 / 1000000000);
    tft.print("NAND Flash Memory Size = ");
    //    tft.printf("%d bytes / ", myfs_NAND.totalSize());
    tft.printf("%d Mbyte / ", myfs_NAND.totalSize() / 1048576);
    tft.printf("%d Gbit\n", myfs_NAND.totalSize() * 8 / 1000000000);
  } else {
    Serial.printf("No Flash Installed\n");
    tft.printf("No Flash Installed\n");
  }
  tft.println();

  // Check for SD card installed
  if (!(SD.begin(SDCARD_CS_PIN))) {
    Serial.println("SD card not found");
    tft.println("SD card not found");
    sDCardInstalled = false;
  } else {
    Serial.println("SD card is Inserted");
    tft.println("SD card is Inserted");
    sDCardInstalled = true;
  }
  tft.println();

  // Check for ESP32 installed
  ESP32SERIAL.print("?");         // Ask ESP32 if it is there
  delay(100);                     // Wait a bit for ESP32 to respond
  if (ESP32SERIAL.available()) {  // If there is a response
    String returnData = ESP32SERIAL.readString();
    if (returnData == 'Y') {  // ESP32S responded with Y for Yes, I'm here
      esp32SAttached = true;
      Serial.println("ESP32-C3 was found");
      tft.println("ESP32-C3 was found");
    } else {  // No response or invalid response
      Serial.println("ESP32-C3 not found");
      tft.println("ESP32-C3 not found");
      esp32SAttached = false;
    }
  }
  // Check for NOR Flash on baseboard
  if (!SerialFlash.begin(SPI1, FLASH_CS)) {
    Serial.println(F("Unable to access SPI Flash chip"));
    tft.println("Unable to access SPI Flash Chip");
  } else {
    unsigned char id[5];
    SerialFlash.readID(id);
    Serial.print("ID=");

    Serial.printf("ID: %02X %02X %02X\n", id[0], id[1], id[2]);
    unsigned long sizeFlash = SerialFlash.capacity(id);

    if (sizeFlash > 0) {
      Serial.print("SPI1 NOR Flash Memory has ");
      Serial.print(sizeFlash);
      Serial.println(" bytes");
      tft.println();
      tft.printf("SPI NOR Flash Memory Size = %d Mbyte\n", sizeFlash / 1000000);
    }
    Serial.print(tempmonGetTemp());
    Serial.println("°C");
    tft.println();
    tft.print("CPU Temp: ");
    tft.print(tempmonGetTemp());
    tft.println("°C");
  }
}

/*
  Mini Platform for Teensy 4.1 Example

  This program does nothing overly useful, but illustrates some of the basic
  features and checks for presence of PSRAM/Flash memory installed, SD card
  installed and the ESP32-C3. It also looks for an Ethernet cable connection
  and whether a Flash drive is inderted into the Host USB connector.
  It reports the info out to the LCD and serial port.

  If SD card installed, the Audio button plays the wave file
  "SDTEST2.WAV" from the Teensy audio tutorial
  https://www.pjrc.com/teensy/td_libs_AudioDataFiles.html

  The Scan button sends a command to the ESP32S requesting a scan of
  available WiFi networks.  When the ESP32S returns the scan results,
  the Teensy 4.1 updates those results on the LCD screen and serial port.

  The System button just rechecks the same system information.  Note that the
  System button won't work if audio is playing.

  The three Teensy 4.1 user buttons simply turn the RGB LED colors ON/OFF

  The ST7796 LCD uses the ST7796_t3 library branch from:
  https://github.com/KurtE/ST7735_t3/tree/ST7796

  The FT6336 touch overlay uses the Adafruit_FT6206.h library.

  The 16MB NOR Flash uses this SerialFlash library branch so that it
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
#include <USBHost_t36.h>
#include <QNEthernet.h>
using namespace qindesign::network;
extern "C" uint8_t external_psram_size;

// Setup USBHost_t36 and as many HUB ports as needed.
USBHost myusb;
USBHub hub1(myusb);
// Instances for one drive
USBDrive myDrive(myusb);
// Instances for accessing the files on the drive
USBFilesystem myFiles(myusb);

// Setup audio system
AudioPlaySdWav playSdWav1;
AudioOutputI2S i2s1;
AudioConnection patchCord1(playSdWav1, 0, i2s1, 0);
AudioConnection patchCord2(playSdWav1, 1, i2s1, 1);
AudioControlSGTL5000 sgtl5000_1;

// Pins used with the built-in audio
#define SDCARD_CS_PIN BUILTIN_SDCARD
#define SDCARD_MOSI_PIN 11
#define SDCARD_SCK_PIN 13

// LCD control pins defined by the baseboard
#define TFT_CS 10
#define TFT_DC 9
// Use main SPI bus MOSI=11, MISO=12, SCK=13 with different control pins
ST7796_t3 tft = ST7796_t3(TFT_CS, TFT_DC);

// Touch screen control pins defined by the baseboard
// TIRQ interrupt if used is on pin 36.  We are using polling for touch
//#define TIRQ_PIN  36

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ts = Adafruit_FT6206();

// User buttons on the baseboard
#define BTN1_PIN 22
#define BTN2_PIN 32
#define BTN3_PIN 40

// INSTANTIATE 3 Button OBJECTS.  Using bouncee library to debounce the buttons
Bounce button1 = Bounce();
Bounce button2 = Bounce();
Bounce button3 = Bounce();

// Pins used for RGB LED
#define RGB_R_PIN 4
#define RGB_G_PIN 6
#define RGB_B_PIN 5

// Define Audio button location and size on LCD
#define AUDIO_X 10
#define AUDIO_Y 5
#define AUDIO_W 105
#define AUDIO_H 42

// Define Scan button location and size on LCD
#define SCAN_X 190
#define SCAN_Y 5
#define SCAN_W 105
#define SCAN_H 42
#define BUTTON_FONT Arial_14
#define TXT_FONT Arial_12

// Define System checks button location and size on LCD
#define SYSTEM_X 370
#define SYSTEM_Y 5
#define SYSTEM_W 105
#define SYSTEM_H 42

#define FLASH_CS 37  // Baseboard 16MB/128Mb NOR flash chip CS pin

#define ESP32SERIAL Serial7  // ESP32 is attached to Serial7 port
// Create buffer to hold incoming characters from ESP32-C3
#define ESP32SERIAL_BUFFER_SIZE 1024
unsigned char esp32SerialBuffer[ESP32SERIAL_BUFFER_SIZE];

// Subroutine prototypes
void SetScanButton(boolean);   // Handles Scan button when touched
void SetAudioButton(boolean);  // Handles Audio button when touched
void SetSystemButton();        // Handes System button when touched
void SystemCheck();            // System check routines

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
  ESP32SERIAL.begin(115200);  //Initialize Seria1 7 connected to ESP32S
  // Setup extra memory for the ESP32 serial buffer to avoid overruns
  ESP32SERIAL.addMemoryForRead(esp32SerialBuffer, ESP32SERIAL_BUFFER_SIZE);

  // Start the Ethernet port
  Ethernet.begin();

  // Start the USB Host port
  myusb.begin();

  // Define pins to get SPI1 to work with NOR Flash chip
  SPI1.setMOSI(26);
  SPI1.setSCK(27);
  SPI1.setMISO(39);
  SPI1.setCS(37);
  SPI1.begin();

  //Setup buttons as inputs with pullup resistors
  button1.attach(BTN1_PIN, INPUT_PULLUP);
  button2.attach(BTN2_PIN, INPUT_PULLUP);
  button3.attach(BTN3_PIN, INPUT_PULLUP);
  // Set debounce interval on buttons to 15mSec
  button1.interval(15);
  button2.interval(15);
  button3.interval(15);

  //Setup RGB pins as outputs
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  // Ensure the LEDs are off by driving LOW
  digitalWrite(RGB_R_PIN, LOW);
  digitalWrite(RGB_G_PIN, LOW);
  digitalWrite(RGB_B_PIN, LOW);

  // Setup the LCD screen
  tft.init(320, 480);
  tft.invertDisplay(true);  // LCD requires colors to be inverted
  tft.setRotation(3);       // Rotates screen to match the baseboard orientation

  // Setup touch Screen with touch threshold of 40
  if (!ts.begin(40)) {
    Serial.println("Unable to start touchscreen.");
  } else {
    Serial.println("Touchscreen started.");
  }

  tft.fillScreen(ST7735_BLUE);  // Fill screen with blue
  tft.setCursor(1, 120);        // Set initial cursor position
  tft.setFont(TXT_FONT);        // Set initial font style and size

  // Draw buttons with current state
  SetAudioButton(false);
  SetScanButton(false);
  SetSystemButton();

  // Setup audio
  if (sDCardInstalled) {  // Setup the audio if SD card is installed
    AudioMemory(8);
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);
    SPI.setMOSI(SDCARD_MOSI_PIN);
    SPI.setSCK(SDCARD_SCK_PIN);
  } else {  // If no audio, gray out button
    tft.setCursor(AUDIO_X + 8, AUDIO_Y + 12);
    tft.setFont(BUTTON_FONT);
    tft.setTextColor(ST7735_WHITE);
    tft.fillRoundRect(AUDIO_X, AUDIO_Y, AUDIO_W, AUDIO_H, 4, ST7735_BLACK);
    tft.print("No Audio");
  }

  // Setup ESP32 Scan button
  if (!esp32SAttached) {  // If no ESP320C3 attached, gray out button
    tft.setCursor(SCAN_X + 8, SCAN_Y + 12);
    tft.setFont(BUTTON_FONT);
    tft.setTextColor(ST7735_WHITE);
    tft.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 4, ST7735_BLACK);
    tft.print("No Scan");
  }
  // Initialize User baseboard buttons
  button1.update();
  button2.update();
  button3.update();
}
//===============================================================================
//  Main
//===============================================================================
void loop() {
  static boolean red_LED = false;  // Track state of RGB LEDs
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
    Serial.print("x Raw = ");  // Show our raw touch coordinates for each touch
    Serial.print(p.x);
    Serial.print(", y Raw = ");
    Serial.print(p.y);
    Serial.print("  :  ");

    // Map the touch point to the LCD screen.  X is vertical/Y is horizontal
    p.x = map(p.x, 0, 319, 0, 319);  // X coordinate not remapped
    p.y = map(p.y, 0, 479, 479, 0);  // Y coordinate is flipped

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
    Serial.print("x = ");  // Show our mapped touch coordinates for each touch
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
    tft.setCursor(5, SCAN_Y + SCAN_H + 20);
    tft.setFont(TXT_FONT);
    tft.setTextColor(ST7735_WHITE);
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
  if (button1.fell()) {  // If button was pressed
    red_LED = !red_LED;  // Change LED state
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
  tft.setCursor(AUDIO_X + 8, AUDIO_Y + 12);
  tft.setFont(BUTTON_FONT);

  if (!audio) {  // button is set inactive, redraw button inactive
    tft.fillRoundRect(AUDIO_X, AUDIO_Y, AUDIO_W, AUDIO_H, 4, ST7735_RED);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Play Audio");
    audioPlaying = false;
    if (playSdWav1.isPlaying()) {  // Stop any audio that is playing
      playSdWav1.stop();
      Serial.println("Audio being stopped");
    }
  } else {  // button is active, redraw button active
    tft.fillRoundRect(AUDIO_X, AUDIO_Y, AUDIO_W, AUDIO_H, 4, ST7735_GREEN);
    tft.setTextColor(ST7735_BLACK);
    tft.print("   Playing");
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
  tft.setCursor(SCAN_X + 8, SCAN_Y + 12);
  tft.setFont(BUTTON_FONT);

  if (!scanning) {  // Button is inactive, redraw button
    tft.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 4, ST7735_RED);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Scan WiFi");
  } else {                                                        // Button is active, redraw button
    tft.fillRect(0, SCAN_Y + SCAN_H + 1, 479, 319, ST7735_BLUE);  // Clear previous info
    tft.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 4, ST7735_GREEN);
    tft.setTextColor(ST7735_BLACK);
    tft.print(" Scanning");
    ESP32SERIAL.println("S");  // Send command to ESP32 to start scan
    scanRequested = true;      // Set flag that we requested scan
    Serial.println("Scan being requested");
  }
}
//===============================================================================
//  Routine to draw system button current state and initiate system checks
//===============================================================================
void SetSystemButton() {
  tft.setCursor(SYSTEM_X + 20, SYSTEM_Y + 12);
  tft.setFont(BUTTON_FONT);

  tft.fillRect(0, SCAN_Y + SCAN_H + 1, 479, 319, ST7735_BLUE);  // Clear previous info
  tft.fillRoundRect(SYSTEM_X, SYSTEM_Y, SYSTEM_W, SYSTEM_H, 4, ST7735_GREEN);
  tft.setTextColor(ST7735_WHITE);
  tft.print("System");
  Serial.println("System check requested");

  // Only do SystemCheck when audio is not playing off SD card
  if (audioPlaying == false) {
    tft.setCursor(SYSTEM_X + 16, SYSTEM_Y + 12);
    tft.setFont(BUTTON_FONT);
    tft.fillRoundRect(SYSTEM_X, SYSTEM_Y, SYSTEM_W, SYSTEM_H, 4, ST7735_GREEN);
    tft.setTextColor(ST7735_BLACK);
    tft.print("Checking");
    SystemCheck();
    tft.setCursor(SYSTEM_X + 20, SYSTEM_Y + 12);
    tft.setFont(BUTTON_FONT);
    tft.setTextColor(ST7735_WHITE);
    tft.print("System");
  } else {
    tft.setCursor(5, SYSTEM_Y + SYSTEM_H + 20);  // Set initial cursor position
    tft.setFont(TXT_FONT);                       // Set initial font style and size
    tft.setTextColor(ST7735_WHITE);
    tft.print("System check cannot run with audio playing");
  }

  tft.setCursor(SYSTEM_X + 20, SYSTEM_Y + 12);
  tft.setFont(BUTTON_FONT);
  tft.fillRoundRect(SYSTEM_X, SYSTEM_Y, SYSTEM_W, SYSTEM_H, 4, ST7735_RED);
  tft.print("System");
}
//===============================================================================
//  Routine to query installed hardware
//===============================================================================
void SystemCheck() {

  tft.setCursor(0, SYSTEM_Y + SYSTEM_H + 20);  // Set initial cursor position
  tft.setFont(TXT_FONT);                       // Set initial font style and size
  tft.setTextColor(ST7735_WHITE);

  // Check for PSRAM chip(s) installed on Teensy 4.1
  uint8_t size = external_psram_size;
  if (size == 0) {
    Serial.println("No PSRAM Installed");
    tft.println("No PSRAM Installed");
  } else {
    Serial.printf("PSRAM Memory Size = %d Mbyte\n", size);
    tft.printf("PSRAM Memory Size = %d Mbyte\n", size);
  }
  tft.println();

  // Check for Flash chip installed on Teensy 4.1
  LittleFS_QSPIFlash myfs_NOR;  // NOR FLASH on Teensy
  LittleFS_QPINAND myfs_NAND;   // NAND FLASH 2Gb on Teensy

  // Check for any NOR Flash chip installed.  Normally only NAND Flash is installed
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

  // Check for SD card installed in Teensy 4.1 card slot
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

  // Check for ESP32-C3 installed
  while (ESP32SERIAL.available()) { // Clear ESP32 input buffer before CMD
    (char) ESP32SERIAL.read();
  }
  bool esp32SAttachedLast = esp32SAttached;

  ESP32SERIAL.print("?");                          // Ask ESP32-C3 if it is there
  delay(100);                                      // Wait a bit for ESP32 to respond
  if (ESP32SERIAL.available()) {                   // If there is a response
    String returnData = ESP32SERIAL.readString();  // Read response
    if (returnData == 'Y') {                       // ESP32-C3 responded with 'Y' for Yes, I'm here
      esp32SAttached = true;
      Serial.println("ESP32-C3 was found");
      tft.println("ESP32-C3 was found");
    } else {  // No response or invalid response
      Serial.println("ESP32-C3 not found");
      tft.println("ESP32-C3 not found");
      esp32SAttached = false;
    }
  }
  else {
          esp32SAttached = false;
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

    // Check for Ethernet cable connected
    tft.println();
    bool link = Ethernet.linkState();
    Serial.print("Link State: ");
    if (link == true) {
      Serial.println("ON");
      tft.println("Ethernet cable is connected");
    } else {
      Serial.println("OFF");
      tft.println("Ethernet cable is not connected");
    }
    tft.println();

    // Check for USB Flash Drive attached
    myusb.Task();
    if (!myFiles) {
      Serial.println("USB Flash Drive not connected");
      tft.println("USB Flash Drive is not connected");
    } else {
      Serial.println("USB Flash Drive is connected");
      tft.println("USB Flash Drive is connected");
    }
    // Check CPU internal temperature
    Serial.print(tempmonGetTemp());
    Serial.println("°C");
    tft.setCursor(320, 300);
    tft.print("CPU Temp: ");
    tft.print(tempmonGetTemp());
    tft.println("°C");
  }
  if ( esp32SAttached != esp32SAttachedLast ) { // redraw SCAN if ESP32 changes
    tft.setCursor(SCAN_X + 8, SCAN_Y + 12);
    tft.setFont(BUTTON_FONT);
    if ( esp32SAttached ) {
      tft.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 4, ST7735_RED);
      tft.setTextColor(ST7735_WHITE);
      tft.print("Scan WiFi");
    }
    else {
      tft.setTextColor(ST7735_WHITE);
      tft.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 4, ST7735_BLACK);
      tft.print("No Scan");
    }
  }
}

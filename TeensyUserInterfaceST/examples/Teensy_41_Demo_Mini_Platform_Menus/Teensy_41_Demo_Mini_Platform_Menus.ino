/*
    Mini Platform for Teensy 4.1 Example Software



*/

#include <ST7796_t3.h>
#include <st7735_t3_font_Arial.h>
#include <st7735_t3_font_ArialBold.h>
#include <TeensyUserInterfaceST.h>
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

#define ESP32SERIAL Serial7           // ESP32 is attached to Serial7 port
#define ESP32SERIAL_BUFFER_SIZE 1024  // Create buffer for incoming chars from ESP32-C3
unsigned char esp32SerialBuffer[ESP32SERIAL_BUFFER_SIZE];

boolean sDCardInstalled = false;  // Flag if SD card installed

// create the user interface object
TeensyUserInterface ui;

//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  // pin numbers used in addition to the default SPI pins
  const int LCD_CS_PIN = 10;  //xx 10;
  const int LCD_DC_PIN = 9;
  const int TOUCH_CS_PIN = 41;  //This should be removed from .begin

  // setup the LCD orientation, the default font and initialize the user interface
  ui.begin(LCD_CS_PIN, LCD_DC_PIN, TOUCH_CS_PIN, LCD_ORIENTATION_LANDSCAPE_4PIN_RIGHT, Arial_9_Bold);

  // use a grayscale color palette for main screen
  ui.setColorPaletteGray();
  // Use larger font to make back button easier to hit
  ui.setTitleBarFont(Arial_13_Bold);
  ui.setMenuFont(Arial_12_Bold);

  Serial.begin(115200);       //Initialize USB serial port to computer
  ESP32SERIAL.begin(115200);  //Initialize Seria1 7 connected to ESP32S
  // Setup extra memory for the ESP32 serial buffer to avoid overruns
  ESP32SERIAL.addMemoryForRead(esp32SerialBuffer, ESP32SERIAL_BUFFER_SIZE);

  AudioMemory(8);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);
  sgtl5000_1.unmuteLineout();
  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000_1.lineInLevel(5);
  sgtl5000_1.micGain(36);

  // Check for SD card installed in Teensy 4.1 card slot
  if (!(SD.begin(SDCARD_CS_PIN))) {
    Serial.println("SD card not found");
    sDCardInstalled = false;
  } else {
    Serial.println("SD card is Inserted");
    sDCardInstalled = true;
  }
}

// ---------------------------------------------------------------------------------
//           Define the menus and top level loop, place menus after setup()
// ---------------------------------------------------------------------------------

// for each menu, create a forward declaration with "extern"
extern MENU_ITEM mainMenu[];

// the main menu
MENU_ITEM mainMenu[] = {
  { MENU_ITEM_TYPE_MAIN_MENU_HEADER, "Mini Platform for Teensy 4.1 Demo", MENU_COLUMNS_3, mainMenu },
  { MENU_ITEM_TYPE_COMMAND, "Audio", cmdAudioPage, NULL },
  { MENU_ITEM_TYPE_COMMAND, "Scan WiFi", cmdScanWifiPage, NULL },
  { MENU_ITEM_TYPE_COMMAND, "System Info", cmdSystemInfoPage, NULL },
  { MENU_ITEM_TYPE_COMMAND, "USB Host", cmdUsbHostPage, NULL },
  { MENU_ITEM_TYPE_COMMAND, "Ethernet", cmdEthernetPage, NULL },
  { MENU_ITEM_TYPE_COMMAND, "ESP32-C3", cmdEsp32c3Page, NULL },
  { MENU_ITEM_TYPE_COMMAND, "SD Card", cmdSdCardPage, NULL },
  { MENU_ITEM_TYPE_COMMAND, "RGB LED", cmdRgbLedPage, NULL },
  { MENU_ITEM_TYPE_COMMAND, "Color Picker", cmdMenuColorPickerPage, NULL },
  { MENU_ITEM_TYPE_END_OF_MENU, "", NULL, NULL }
};

//===============================================================================
//  Main
//===============================================================================
void loop() {
  // display the menu, then execute commands selected by the user
  ui.displayAndExecuteMenu(mainMenu);
}

// ---------------------------------------------------------------------------------
//                            Commands executed from the menu
// ---------------------------------------------------------------------------------

//===============================================================================
//  Audio Page
//===============================================================================
// build buttons for Audio Page
BUTTON playSDAudioButton;
BUTTON audioInputButton;

static int wavFileSelection = 1;
static bool audioPlaying = false;
static float headsetVolValue = 0.5;
static int lineOutLevelValue = 20;
static int lineInLevelValue = 5;

void cmdAudioPage(void) {
  
  // Setup audio screen graphical elements
  ui.setColorPaletteBlue();
  ui.drawTitleBarWithBackButton("AUDIO");
  ui.clearDisplaySpace();

  // Headset volume control
  NUMBER_BOX_FLOAT headsetVolBox;
  headsetVolBox.labelText = "Headset Volume";
  headsetVolBox.value = headsetVolValue;
  headsetVolBox.minimumValue = 0.0;
  headsetVolBox.maximumValue = 1.0;
  headsetVolBox.stepAmount = 0.1;
  headsetVolBox.digitsRightOfDecimal = 1;
  headsetVolBox.centerX = ui.displaySpaceCenterX - 150;
  headsetVolBox.centerY = ui.displaySpaceCenterY - 75;
  headsetVolBox.width = 120;
  headsetVolBox.height = 50;
  ui.drawNumberBox(headsetVolBox);

  // Line Out level control
  NUMBER_BOX lineOutLevelBox;
  lineOutLevelBox.labelText = "Line Out Level";
  lineOutLevelBox.value = lineOutLevelValue;
  lineOutLevelBox.minimumValue = 13;
  lineOutLevelBox.maximumValue = 31;
  lineOutLevelBox.stepAmount = 1;
  lineOutLevelBox.centerX = ui.displaySpaceCenterX;
  lineOutLevelBox.centerY = ui.displaySpaceCenterY - 75;
  lineOutLevelBox.width = 120;
  lineOutLevelBox.height = 50;
  ui.drawNumberBox(lineOutLevelBox);

  // Line In Level Control
  NUMBER_BOX lineInLevelBox;
  lineInLevelBox.labelText = "Line In Level";
  lineInLevelBox.value = lineInLevelValue;
  lineInLevelBox.minimumValue = 0;
  lineInLevelBox.maximumValue = 15;
  lineInLevelBox.stepAmount = 1;
  lineInLevelBox.centerX = ui.displaySpaceCenterX + 150;
  lineInLevelBox.centerY = ui.displaySpaceCenterY - 75;
  lineInLevelBox.width = 120;
  lineInLevelBox.height = 50;
  ui.drawNumberBox(lineInLevelBox);

  // Define and draw WAV file selections
  SELECTION_BOX wavFileSelectionBox;
  wavFileSelectionBox.labelText = "WAV File Select";
  wavFileSelectionBox.value = wavFileSelection;
  wavFileSelectionBox.choice0Text = "SDTEST1";
  wavFileSelectionBox.choice1Text = "SDTEST2";
  wavFileSelectionBox.choice2Text = "SDTEST3";
  wavFileSelectionBox.choice3Text = "SDTEST4";
  wavFileSelectionBox.centerX = ui.displaySpaceCenterX;
  wavFileSelectionBox.centerY = 200;
  wavFileSelectionBox.width = 460;
  wavFileSelectionBox.height = 50;
  ui.drawSelectionBox(wavFileSelectionBox);

  // Define and draw Audio Play button
  playSDAudioButton = { "Play WAV File", ui.displaySpaceCenterX-100, ui.displaySpaceBottomY - 40, 150, 50 };
  ui.drawButton(playSDAudioButton);

    // Define and draw Audio Input button
  audioInputButton = { "Input - Line In", ui.displaySpaceCenterX+100, ui.displaySpaceBottomY - 40, 150, 50 };
  ui.drawButton(audioInputButton);

  // Execute Audio Page loop until "Back" button pressed, then return to the main menu
  while (true) {

    // Keep an eye on any audio that may be playing and reset button when it ends
    if (playSdWav1.isStopped() && audioPlaying) {  // Audio finished playing
      playSDAudioButton = { "Play", ui.displaySpaceCenterX-100, ui.displaySpaceBottomY - 40, 150, 50 };
      ui.drawButton(playSDAudioButton);
      audioPlaying = false;
    }
    
    // Update touch events for all objects
    ui.getTouchEvents();  

    // Adjust Headset Volume
    if (ui.checkForNumberBoxTouched(headsetVolBox)) {
      sgtl5000_1.volume(headsetVolBox.value);
      Serial.print("Headset volume = ");
      Serial.println(headsetVolBox.value);
    }
    // Adjust Line Output Level 
    if (ui.checkForNumberBoxTouched(lineOutLevelBox)) {
      sgtl5000_1.lineOutLevel(lineOutLevelBox.value);
      Serial.print("Line Out level = ");
      Serial.println(lineOutLevelBox.value);
    }
    // Adjust Line Input Level
    if (ui.checkForNumberBoxTouched(lineInLevelBox)) {
      sgtl5000_1.lineInLevel(lineInLevelBox.value);
      Serial.print("Line In level = ");
      Serial.println(lineInLevelBox.value);
    }
    // Update WAV file selection
    if (ui.checkForSelectionBoxTouched(wavFileSelectionBox)) {
      if (audioPlaying) {  // Already playing a file, so play new selection
        playSdWav1.stop();
        switch (wavFileSelectionBox.value) {
          case 0:
            playSdWav1.play("SDTEST1.WAV");
            break;
          case 1:
            playSdWav1.play("SDTEST2.WAV");
            break;
          case 2:
            playSdWav1.play("SDTEST3.WAV");
            break;
          case 3:
            playSdWav1.play("SDTEST4.WAV");
            break;
          default:
            break;
        }
        delay(10);
      }
    }

    // Check for play audio button touch
    // If currently playing audio, stop playing audio
    if (ui.checkForButtonClicked(playSDAudioButton) && audioPlaying) {
      playSdWav1.stop();
      playSDAudioButton = { "Play WAV File", ui.displaySpaceCenterX - 100, ui.displaySpaceBottomY - 40, 150, 50 };
      ui.drawButton(playSDAudioButton);
      audioPlaying = false;
    }
    // If not audio playing, play current selection
    else if (ui.checkForButtonClicked(playSDAudioButton) && !audioPlaying) {
      switch (wavFileSelectionBox.value) {
        case 0:
          playSdWav1.play("SDTEST1.WAV");
          break;
        case 1:
          playSdWav1.play("SDTEST2.WAV");
          break;
        case 2:
          playSdWav1.play("SDTEST3.WAV");
          break;
        case 3:
          playSdWav1.play("SDTEST4.WAV");
          break;
        default:
          break;
      }
      delay(10);  // wait for library to parse WAV info
      // Update text on Play WAV file button
      playSDAudioButton = { "Playing", ui.displaySpaceCenterX-100, ui.displaySpaceBottomY - 40, 150, 50 };
      ui.drawButton(playSDAudioButton);
      audioPlaying = true;
    }
    
    // Check for audio input source button touch
    // Toggle between Line IN and Mic inputs
    if (ui.checkForButtonClicked(audioInputButton)) {
      if(audioInputButton.labelText =="Input - Line In") {
        audioInputButton = { "Input - Mic", ui.displaySpaceCenterX+100, ui.displaySpaceBottomY - 40, 150, 50 };
        sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
      }
      else{
        audioInputButton = { "Input - Line In", ui.displaySpaceCenterX+100, ui.displaySpaceBottomY - 40, 150, 50 };
        sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
      }
      ui.drawButton(audioInputButton);
    }
    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {
      if (playSdWav1.isPlaying())
        playSdWav1.stop();
      ui.setColorPaletteGray();
      return;
    }
  }
}


//===============================================================================
//  Scan WiFi Page
//===============================================================================
void cmdScanWifiPage(void) {

  // clear the screen and draw title bar showing without the "Back" button
  ui.drawTitleBarWithBackButton("Scan WiFi");
  ui.clearDisplaySpace();

  // wait for the user to press the "OK" button, then return to the main menu
  //
  while (true) {

    // get touch events on the LCD
    ui.getTouchEvents();

    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {

      return;
    }
  }
}


//===============================================================================
//  System Information Page
//===============================================================================
void cmdSystemInfoPage(void) {
  // clear the screen and draw title bar showing without the "Back" button
  ui.drawTitleBarWithBackButton("System Information");
  ui.clearDisplaySpace();

  // wait for the user to press the "OK" button, then return to the main menu
  //
  while (true) {

    // get touch events on the LCD
    ui.getTouchEvents();

    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {

      return;
    }
  }
}


//===============================================================================
//  USB Host Page
//      Read Flash Drive button
//      Read Mouse button
//===============================================================================
void cmdUsbHostPage(void) {
  // clear the screen and draw title bar showing without the "Back" button
  ui.drawTitleBarWithBackButton("USB Host");
  ui.clearDisplaySpace();

  // wait for the user to press the "OK" button, then return to the main menu
  //
  while (true) {

    // get touch events on the LCD
    ui.getTouchEvents();

    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {

      return;
    }
  }
}

//===============================================================================
//  Wired Ethernet Page
//      Check for connection status
//===============================================================================
void cmdEthernetPage(void) {
  // clear the screen and draw title bar showing without the "Back" button
  ui.drawTitleBarWithBackButton("Wired Ethernet");
  ui.clearDisplaySpace();

  // wait for the user to press the "OK" button, then return to the main menu
  //
  while (true) {

    // get touch events on the LCD
    ui.getTouchEvents();

    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {

      return;
    }
  }
}
//===============================================================================
//  ESP32-C3 Page
//      Send/receive commands from ESP32-C3?
//===============================================================================
void cmdEsp32c3Page(void) {
  // clear the screen and draw title bar showing without the "Back" button
  ui.drawTitleBarWithBackButton("ESP32-C3");
  ui.clearDisplaySpace();

  // wait for the user to press the "OK" button, then return to the main menu
  //
  while (true) {

    // get touch events on the LCD
    ui.getTouchEvents();

    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {

      return;
    }
  }
}

//===============================================================================
//  SD Card Page
//      List files on SD card
//===============================================================================
void cmdSdCardPage(void) {
  // clear the screen and draw title bar showing without the "Back" button
  ui.drawTitleBarWithBackButton("SD Card");
  ui.clearDisplaySpace();

  // wait for the user to press the "OK" button, then return to the main menu
  //
  while (true) {

    // get touch events on the LCD
    ui.getTouchEvents();

    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {

      return;
    }
  }
}

//===============================================================================
//  RGB LED Page
//      Read board buttons and set board RGB LED and on-screen LEDs?
//===============================================================================
void cmdRgbLedPage(void) {
  // clear the screen and draw title bar showing without the "Back" button
  ui.drawTitleBarWithBackButton("RGB LED");
  ui.clearDisplaySpace();

  // wait for the user to press the "OK" button, then return to the main menu
  //
  while (true) {

    // get touch events on the LCD
    ui.getTouchEvents();

    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {

      return;
    }
  }
}

//===============================================================================
//  Menu Color Picker Page
//===============================================================================
void cmdMenuColorPickerPage(void) {
  // clear the screen and draw title bar showing without the "Back" button
  ui.drawTitleBarWithBackButton("Menu Color Picker");
  ui.clearDisplaySpace();

  // wait for the user to press the "OK" button, then return to the main menu
  //
  while (true) {

    // get touch events on the LCD
    ui.getTouchEvents();

    // Do any cleanup required before return to main menu
    if (ui.checkForBackButtonClicked()) {

      return;
    }
  }
}

// Advanced Microcontroller-based Audio Workshop
//
// http://www.pjrc.com/store/audio_tutorial_kit.html
// https://hackaday.io/project/8292-microcontroller-audio-workshop-had-supercon-2015
//
// Part 3-3: Add a TFT Display

// #include <ILI9341_t3.h>
// #include <font_Arial.h> // from ILI9341_t3
#include <ST7796_t3.h>
#include <st7735_t3_font_Arial.h>

#include <Adafruit_FT6206.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

///////////////////////////////////
// copy the Design Tool code here
///////////////////////////////////
// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=136,65
AudioAnalyzePeak         peak2;          //xy=348,219
AudioAnalyzePeak         peak1;          //xy=358,171
AudioOutputI2S           i2s1;           //xy=380,92
AudioConnection          patchCord1(playSdWav1, 0, i2s1, 0);
AudioConnection          patchCord2(playSdWav1, 0, peak1, 0);
AudioConnection          patchCord3(playSdWav1, 1, i2s1, 1);
AudioConnection          patchCord4(playSdWav1, 1, peak2, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=155,192
// GUItool: end automatically generated code


// Use these with the Teensy 4.x and Audio Shield Rev D or D2
#define TFT_DC       9
//#define TFT_CS      22
#define TFT_CS      10
#define TFT_RST    255  // 255 = unused, connect to 3.3V
#define TFT_MOSI    11
#define TFT_SCLK    13
#define TFT_MISO    12

// Use these with the Teensy 3.2 and Audio Shield Rev C
//#define TFT_DC      20
//#define TFT_CS      21
//#define TFT_RST    255  // 255 = unused, connect to 3.3V
//#define TFT_MOSI     7
//#define TFT_SCLK    14
//#define TFT_MISO    12

// ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
ST7796_t3 tft = ST7796_t3(TFT_CS, TFT_DC);


// Use these with the Teensy Audio Shield
//#define SDCARD_CS_PIN    10
//#define SDCARD_MOSI_PIN  7   // Teensy 4 ignores this, uses pin 11
//#define SDCARD_SCK_PIN   14  // Teensy 4 ignores this, uses pin 13

// Use these with the Teensy 3.5 & 3.6 & 4.1 SD card
#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used

// Use these for the SD+Wiz820 or other adaptors
//#define SDCARD_CS_PIN    4
//#define SDCARD_MOSI_PIN  11
//#define SDCARD_SCK_PIN   13

void setup() {
  Serial.begin(9600);
  delay(500);
  //tft.setClock(16000000);
  //tft.begin();
  // Setup the LCD screen
  tft.init(320, 480);
  tft.invertDisplay(true);  // LCD requires colors to be inverted
  tft.setRotation(2);       // Rotates screen to match the baseboard orientation

  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(Arial_24);
  //tft.setTextSize(3);
  tft.setCursor(40, 8);
  tft.println("Peak Meter");

  AudioMemory(10);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.1);
  //  SPI.setMOSI(SDCARD_MOSI_PIN);
  //  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  delay(1000);
}

elapsedMillis msecs;

void loop() {
  if (playSdWav1.isPlaying() == false) {
    Serial.println("Start playing");
    //playSdWav1.play("SDTEST1.WAV");
    //playSdWav1.play("SDTEST2.WAV");
    playSdWav1.play("SDTEST3.WAV");
    //playSdWav1.play("SDTEST4.WAV");
    delay(10); // wait for library to parse WAV info
  }

  if (msecs > 15) {
    if (peak1.available() && peak2.available()) {
      float leftNumber = peak1.read();
      float rightNumber = peak2.read();
      static int leftNumberOld = 0;
      static int rightNumberOld = 0;
      Serial.print(leftNumber);
      Serial.print(", ");
      Serial.print(rightNumber);
      Serial.println();

      // draw the verticle bars
      int height = leftNumber * 380;
      if ( height > leftNumberOld )
        tft.fillRect(60, 420 - height, 40, height, ST7735_GREEN);
      else
        tft.fillRect(60, 420 - 380, 40, 380 - height, ST7735_BLACK);
      leftNumberOld = height;
      height = rightNumber * 380;
      if ( height > rightNumberOld )
        tft.fillRect(140, 420 - height, 40, height, ST7735_GREEN);
      else
        tft.fillRect(140, 420 - 380, 40, 380 - height, ST7735_BLACK);
      rightNumberOld = height;
      // a smarter approach would redraw only the changed portion...
      // draw numbers underneath each bar
      tft.setFont(Arial_14);
      tft.fillRect(60, 444, 40, 16, ST7735_BLACK);
      tft.setCursor(60, 444);
      tft.print(leftNumber);
      tft.fillRect(140, 444, 40, 16, ST7735_BLACK);
      tft.setCursor(140, 444);
      tft.print(rightNumber);
      msecs = 0;
    }
  }
}

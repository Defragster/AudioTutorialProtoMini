// Advanced Microcontroller-based Audio Workshop
//
// http://www.pjrc.com/store/audio_tutorial_kit.html
// https://hackaday.io/project/8292-microcontroller-audio-workshop-had-supercon-2015
//
// Part 3-3: Add a TFT Display
//
// This is modified to work with the Mini Platform for Teensy 4.1 ST7796 LCD

#include <ST7796_t3.h>
#include <st7735_t3_font_Arial.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Bounce.h>

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

// TFT pin definitions
#define TFT_DC       9
#define TFT_CS      10

ST7796_t3 tft = ST7796_t3(TFT_CS, TFT_DC);

#define SDCARD_CS_PIN    BUILTIN_SDCARD

Bounce button0 = Bounce(0, 15);
Bounce button2 = Bounce(2, 15);  // 15 = 15 ms debounce time


void setup() {
  Serial.begin(9600);
  delay(500);
  //tft.setSPISpeed(80000000);

  // Setup the LCD screen
  tft.init(320, 480);
  tft.invertDisplay(true);  // LCD requires colors to be inverted
  tft.setRotation(3);       // Rotates screen to match the baseboard orientation

  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(Arial_24);
  //tft.setTextSize(3);
  tft.setCursor(60, 8);
  tft.println("Peak Meter");

  tft.setFont(Arial_14);
  tft.setCursor(5, 60);
  tft.println("File: ");

  tft.setCursor(5, 100);
  tft.println("Vol: ");

  tft.drawTriangle(40, 240, 60, 260, 80, 240, ST7735_YELLOW);
  tft.drawTriangle(150, 260,  170, 240, 190, 260,ST7735_YELLOW);
  
  tft.setCursor(30,280);
  tft.println("BTN 0           BTN 2");

  AudioMemory(10);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.1);

  if (!(SD.begin(SDCARD_CS_PIN))) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  pinMode(0, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  delay(1000);
}
int filenumber = 0;  // which file to play
// List of files we can play
const char * filelist[4] = {
  "SDTEST1.WAV", "SDTEST2.WAV", "SDTEST3.WAV", "SDTEST4.WAV"
};
int knob = 0;
int knobOld = 128;

elapsedMillis msecs;

void loop() {


  if (playSdWav1.isPlaying() == false) {
    const char *filename = filelist[filenumber];
    filenumber = filenumber + 1;
    if (filenumber >= 4) filenumber = 0;
    Serial.print("Start playing ");
    Serial.println(filename);
    playSdWav1.play(filename);
    tft.fillRect(50, 60, 240, 16, ST7735_BLACK);
    tft.setCursor(50, 60);
    tft.print(filename);
    delay(10); // wait for library to parse WAV info
  }

  if (msecs > 15) {
    if (peak1.available() && peak2.available()) {
      float leftNumber = peak1.read();
      float rightNumber = peak2.read();
      static int leftNumberOld = 0;
      static int rightNumberOld = 0;

      // draw the vertical bars
      int height = leftNumber * 260;
      if ( height > leftNumberOld )
        tft.fillRect(320, 290 - height, 40, height, ST7735_GREEN);
      else
        tft.fillRect(320, 290 - 280, 40, 280 - height, ST7735_BLACK);
      leftNumberOld = height;
      height = rightNumber * 260;
      if ( height > rightNumberOld )
        tft.fillRect(400, 290 - height, 40, height, ST7735_GREEN);
      else
        tft.fillRect(400, 290 - 280, 40, 280 - height, ST7735_BLACK);
      rightNumberOld = height;
      // a smarter approach would redraw only the changed portion...
      // draw numbers underneath each bar
      tft.setFont(Arial_14);
      tft.fillRect(320, 300, 40, 16, ST7735_BLACK);
      tft.setCursor(320, 300);
      tft.print(leftNumber);
      tft.fillRect(400, 300, 40, 16, ST7735_BLACK);
      tft.setCursor(400, 300);
      tft.print(rightNumber);
      msecs = 0;
    }
  }
  // read pushbuttons
  button0.update();
  if (button0.fallingEdge()) {
    tft.fillTriangle(40, 240, 60, 260, 80, 240, ST7735_YELLOW);
    playSdWav1.stop();
    delay(100);
    tft.fillRect(40, 240, 40, 20, ST7735_BLACK);
    tft.drawTriangle(40, 240, 60, 260, 80, 240, ST7735_YELLOW);
  }
  button2.update();
  if (button2.fallingEdge()) {
    tft.fillTriangle(150, 260,  170, 240, 190, 260, ST7735_YELLOW);
    playSdWav1.stop();
    filenumber = filenumber - 2;
    if (filenumber < 0) filenumber = filenumber + 4;
    delay(100);
    tft.fillRect(150, 240, 40, 20, ST7735_BLACK);
    tft.drawTriangle(150, 260,  170, 240, 190, 260, ST7735_YELLOW);
  }
  
  // read the knob position (analog input A1)
  knob = analogRead(A1);
  float vol = (float)knob / 1280.0;
  if(abs(knob-knobOld)>= 20) {
    sgtl5000_1.volume(vol);
    tft.fillRect(50, 100, 40, 16, ST7735_BLACK);
    tft.setCursor(50, 100);
    tft.print(vol);
    knobOld = knob;
  }
}

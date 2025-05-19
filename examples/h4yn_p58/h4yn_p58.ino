// Advanced Microcontroller-based Audio Workshop
//
// http://www.pjrc.com/store/audio_tutorial_kit.html
// https://hackaday.io/project/8292-microcontroller-audio-workshop-had-supercon-2015
//
// Part 3-3: Add a TFT Display

// Pick your TFT here:
//#include <ILI9341_t3.h>
#include <ST7796_t3.h>

/*
 * Pick an update mode:
 * 0 = immediate, 1 = frame buffer
 * 2 = async frame buffer
 */
#define UPDATE_MODE 0

#if defined ST77XX_BLACK // see which TFT we're using
#include <st7735_t3_font_Arial.h>
#else
#include <font_Arial.h> // from ILI9341_t3
#define ST7735_BLACK ILI9341_BLACK
#define ST7735_RED ILI9341_RED
#define ST7735_YELLOW ILI9341_YELLOW
#define ST7735_GREEN ILI9341_GREEN
//#define ST7735_BLACK ILI9341_BLACK
#endif // defined ST77XX_BLACK

//#include <Adafruit_FT6206.h>

#include <Audio.h>

#define COUNT_OF(a) ((int)(sizeof a / sizeof a[0]))

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
// these are h4yn0nnymou5e pin assignments - you may need to change them
#define TFT_DC       9
#define TFT_CS      22
//#define TFT_CS      10
#define TFT_RST    255  // 255 = unused, connect to 3.3V

#define LED_PWM  4 // used to set brightness of LED backlight


#if defined ST77XX_BLACK
ST7796_t3 tft = ST7796_t3(TFT_CS, TFT_DC);
#else
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);//, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
#endif // defined ST77XX_BLACK

// Use this with the Teensy 3.5 & 3.6 & 4.1 SD card
#define SDCARD_CS_PIN    BUILTIN_SDCARD

#define SQ 80
DMAMEM uint16_t save1[SQ*SQ], save2[SQ*SQ];

void makeSave(uint16_t* sv, int n)
{
  uint16_t bg = ST77XX_BLACK;
  const char* msg;
 
  switch (n)
  {
    default:
      tft.setTextColor(ST77XX_RED);
      msg = "on";
      break;

    case 1:
      tft.setTextColor(ST77XX_GREEN);
      bg = ST77XX_BLUE;
      msg = "off";
      break;         
  }

  tft.fillRect(0,0,SQ,SQ,bg);
  tft.setFont(Arial_24);
  tft.setCursor(25,25);
  tft.print(msg);
  tft.readRect(0,0,SQ,SQ,sv);
  //delay(500);
}


void setup() {
  pinMode(LED_PWM, OUTPUT);
  //analogWrite(LED_PWM, 64);
  digitalWrite(LED_PWM,0);
 
  Serial.begin(9600);
  delay(100);
  //tft.setClock(16000000);
    // Setup the LCD screen
#if defined ST77XX_BLACK
  tft.init(320, 480);
  tft.setRotation(3);       // Rotates screen to match the baseboard orientation
#else 
  tft.begin();
  tft.setRotation(1);       // Rotates screen to match the baseboard orientation
#endif // defined ST77XX_BLACK

  //tft.invertDisplay(true);  // LCD requires colors to be inverted

  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(Arial_24);
  //tft.setTextSize(3);
  tft.setCursor(40, 8);
  //tft.println("Peak Meter");

  tft.useFrameBuffer(true);
  makeSave(&save1[0],0);
  makeSave(&save2[0],1);
  tft.useFrameBuffer(false);
  tft.freeFrameBuffer();

 
  AudioMemory(10);
//sgtl5000_1.setAddress(HIGH);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.3);
  while (!(SD.begin(SDCARD_CS_PIN)))
  {
      Serial.println("Unable to access the SD card");
      delay(500);
  }
  //tft.setMaxTransaction(100000);  // deliberately break SD playback!
  tft.setMaxTransaction(2000);      // default is 1000, but this should be OK
  tft.enableYieldInMidTransaction(true); // does a bonus yield() if a mid-transaction break occurs

  switch (UPDATE_MODE)
  {
    default:
      tft.useFrameBuffer(true);
      break;

    case 0:
      break;     
  }
 
  delay(100);
}

elapsedMillis msecs;

/*
 * We need to check a number of functions which can result
 * in long-running transactions
 */
elapsedMicros checkMicros;
uint16_t colours[] = {ST7735_RED, CL(255,128,0), ST7735_YELLOW, ST7735_GREEN, ST7735_BLUE, ST7735_MAGENTA, ST7735_WHITE};

uint16_t nextColour(void)
{
  static int idx;
  uint16_t result = colours[idx];

  if (++idx >= COUNT_OF(colours))
    idx = 0;

  return result;   
}

//========================================================================
uint32_t check_(void)
{
  checkMicros = 0;

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_fillRect(void)
{
  checkMicros = 0;

  tft.fillRect(0,0,SQ,SQ,nextColour());

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_drawChar(void)
{
  tft.fillRect(SQ+96,0,96,SQ,nextColour());
  checkMicros = 0;

  tft.setFont();
  tft.setTextSize(8);
  tft.setTextColor(nextColour());
  tft.setCursor(SQ+96,0);
  tft.print("qb");

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_drawChar_bg(void)
{
  checkMicros = 0;

  tft.setFont();
  tft.setTextSize(8);
  tft.setTextColor(nextColour(),ST77XX_BLACK);
  tft.setCursor(SQ,0);
  tft.print("t8");

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_drawFontChar(void)
{
  tft.fillRect(SQ+96,SQ,96,90,nextColour());
  checkMicros = 0;

  tft.setFont(Arial_60);
  tft.setTextColor(nextColour());
  tft.setCursor(SQ+96,SQ);
  tft.print("qb");

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_drawFontChar_bg(void)
{
  checkMicros = 0;

  tft.setFont(Arial_60);
  tft.setTextColor(nextColour(),0x38E7);
  tft.setCursor(SQ,SQ);
  tft.print(" t8 ");

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_writeRect(void)
{
  checkMicros = 0;
  static int which;

  uint16_t* wr = (which&16)?save1:save2;
  tft.writeRect(0,SQ,SQ,SQ,wr);
  which++;// = !which;

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_writeSubImageRect(void)
{
  tft.fillRect(0,SQ*2,SQ,SQ,nextColour()); // don't count this
 
  checkMicros = 0;
  static int which;
  int off = 8;

  uint16_t* wr = (which&32)?save1:save2;
  tft.writeSubImageRect(off,SQ*2+off,SQ-off*2,SQ-off*2,
                        off, off, SQ, SQ,
                        wr);
  which++;// = !which;

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_writeSubImageRectBytesReversed(void)
{
  tft.fillRect(0,SQ*3,SQ,SQ,nextColour()); // don't count this
 
  checkMicros = 0;
  static int which;
  int off = 8;

  uint16_t* wr = (which&8)?save1:save2;
  tft.writeSubImageRect(off,SQ*3+off,SQ-off*2,SQ-off*2,
                        off, off, SQ, SQ,
                        wr);
  which++;// = !which;

  return checkMicros;
}

//------------------------------------------------------------------------
uint32_t check_updateScreen(void)
{ 
  checkMicros = 0;
  tft.updateScreen();
  return checkMicros;
}

//========================================================================
#define RUN_CHECK(n) Serial.printf("Check " #n ": %d\n", check_##n())

uint32_t lastCheck;
elapsedMicros asyncTime;
bool asyncStarted;

void loop()
{
  if (playSdWav1.isPlaying() == false)
  {
    Serial.println("Start playing");
    //playSdWav1.play("SDTEST1.WAV");
    //playSdWav1.play("SDTEST2.WAV");
    //playSdWav1.play("SDTEST3.WAV");
    playSdWav1.play("SDTEST4.WAV");
    delay(10); // wait for library to parse WAV info
  }

  if (millis() - lastCheck >= 100)
  {
    lastCheck = millis();

    Serial.println("========================");
    RUN_CHECK(fillRect);
    RUN_CHECK(drawChar_bg);
    RUN_CHECK(drawChar);
    RUN_CHECK(drawFontChar_bg);
    RUN_CHECK(drawFontChar);
    RUN_CHECK(writeRect);
    RUN_CHECK(writeSubImageRect);
    RUN_CHECK(writeSubImageRectBytesReversed);
    
    switch (UPDATE_MODE)
    {
      default:
        break;

      case 1:
        RUN_CHECK(updateScreen);
        break;
        
      case 2:
        Serial.printf("Async start was %sOK\n",tft.updateScreenAsync()?"":"not ");
        asyncTime = 0;
        asyncStarted=true;
        break;
    }
  }

  if (asyncStarted && !tft.asyncUpdateActive())
  {
    asyncStarted = false;
    Serial.printf("Async update took %dus\n",(uint32_t) asyncTime);
  }
}

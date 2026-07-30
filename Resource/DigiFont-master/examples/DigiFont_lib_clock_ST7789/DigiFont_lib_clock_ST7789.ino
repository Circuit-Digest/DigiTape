// DigiFont library example
// Digital clock example on ST7789 IPS display
// (c) 2020-24 Pawel A. Hernik

/*
ST7789 240x240 1.3" IPS (without CS pin) - only 4+2 wires required:
 #01 GND -> GND
 #02 VCC -> VCC (3.3V only!)
 #03 SCL -> D13/SCK
 #04 SDA -> D11/MOSI
 #05 RES -> D9 /PA0 or any digital (HW RESET is required to properly initialize LCD without CS)
 #06 DC  -> D10/PA1 or any digital
 #07 BLK -> NC

ST7789 240x280 1.69" IPS - only 4+2 wires required:
 #01 GND -> GND
 #02 VCC -> VCC (3.3V only!)
 #03 SCL -> D13/SCK
 #04 SDA -> D11/MOSI
 #05 RES -> optional
 #06 DC  -> D10 or any digital
 #07 CS  -> D9 or any digital
 #08 BLK -> VCC

ST7789 170x320 1.9" IPS - only 4+2 wires required:
 #01 GND -> GND
 #02 VCC -> VCC (3.3V only!)
 #03 SCL -> D13/SCK
 #04 SDA -> D11/MOSI
 #05 RES -> optional
 #06 DC  -> D10 or any digital
 #07 CS  -> D9 or any digital
 #08 BLK -> VCC

ST7789 240x320 2.0" IPS - only 4+2 wires required:
 #01 GND -> GND
 #02 VCC -> VCC (3.3V only!)
 #03 SCL -> D13/SCK
 #04 SDA -> D11/MOSI
 #05 RES -> optional
 #06 DC  -> D10 or any digital
 #07 CS  -> D9 or any digital
*/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include "ST7789_AVR.h"

#define SCR_WD 240
#define SCR_HT 240
//#define SCR_HT 320

#define TFT_DC   10
#define TFT_CS    9  // with CS
#define TFT_RST  -1  // with CS
ST7789_AVR lcd = ST7789_AVR(TFT_DC, TFT_RST, TFT_CS);

// -----------------
#include "DigiFont.h"
// needed for DigiFont library initialization, define your customLineH, customLineV and customRect
void customLineH(int x0,int x1, int y, int c) { lcd.drawFastHLine(x0,y,x1-x0+1,c); }
void customLineV(int x, int y0,int y1, int c) { lcd.drawFastVLine(x,y0,y1-y0+1,c); } 
void customRect(int x, int y,int w,int h, int c) { lcd.fillRect(x,y,w,h,c); } 
DigiFont font(customLineH,customLineV,customRect);
// -----------------

void setup(void) 
{
  Serial.begin(9600);
  lcd.init();
  lcd.fillScreen(BLACK);
}

unsigned long ms;

void demoClock1(int t)
{
  lcd.fillScreen(BLACK);
  font.setColors(RED,RGBto565(60,0,0));
  int w=(SCR_HT-t-10)/4;
  font.setSize1(w-3,w*2,t);
  int y=(SCR_HT-w*2)/2;
  ms=millis();
  while(millis()-ms<5000) {
    font.drawDigit1(random(0,3),0*w,y);
    font.drawDigit1(random(0,4),1*w,y);
    font.drawDigit1(':',2*w+5-3,y);
    font.drawDigit1(random(0,6),t+10-3+2*w,y);
    font.drawDigit1(random(0,10),t+10-3+3*w,y);
    delay(600);
  }
}

void demoClock2(int t)
{
  lcd.fillScreen(BLACK);
  font.setColors(GREEN,RGBto565(0,40,0));
  int w=(SCR_HT-t-10)/4;
  font.setSize2(w-3,w*2,t);
  int y=(SCR_HT-w*2)/2;
  ms=millis();
  while(millis()-ms<5000) {
    font.drawDigit2(random(0,3),0*w,y);
    font.drawDigit2(random(0,4),1*w,y);
    font.drawDigit2(':',2*w+5-3,y);
    font.drawDigit2(random(0,6),t+10-3+2*w,y);
    font.drawDigit2(random(0,10),t+10-3+3*w,y);
    delay(600);
  }
}

void demoClock2c(int t)
{
  lcd.fillScreen(BLACK);
  font.setColors(CYAN,RGBto565(0,190,190),RGBto565(0,40,40));
  int w=(SCR_HT-t-10)/4;
  font.setSize2(w-3,w*2,t);
  int y=(SCR_HT-w*2)/2;
  ms=millis();
  while(millis()-ms<5000) {
    font.drawDigit2c(random(0,3),0*w,y);
    font.drawDigit2c(random(0,4),1*w,y);
    font.drawDigit2c(':',2*w+5-3,y);
    font.drawDigit2c(random(0,6),t+10-3+2*w,y);
    font.drawDigit2c(random(0,10),t+10-3+3*w,y);
    delay(600);
  }
}

void loop() 
{
  demoClock1(10);
  demoClock2(14);
  demoClock2c(14);
  demoClock2(10);
  demoClock2c(10);
}



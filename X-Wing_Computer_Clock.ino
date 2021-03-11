#include <ESP8266WiFi.h>

#include <SPI.h>
#include "Setup_ILI9486.h"
#include <TFT_eSPI.h>
#include <JPEGDecoder.h>
#include <WiFiUdp.h>
#include <TimeLib.h>

#include "Glaser_Becker_Stencil_Regular24pt7b.h"

#include "screen1.h"
#include "screen2.h"
#include "screen3.h"
#include "screen4.h"

// Return the minimum of two values a and b
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

#define DEBUG_PRINT(s)   \
{ \
  Serial.print(s); \
  if (prevDisplay == -1) { \
    tft.print(s); \
  } \
}

#define DEBUG_PRINTLN(s) \
{ \
  Serial.println(s); \
  if (prevDisplay == -1) { \
    tft.println(s); \
  } \
}

const char* ssid     = "Ruby Komtesa";
const char* password = "Sherlock";     

static const char ntpServerName[] = "pool.ntp.org";
const int timeZone = 1;     // Central European Time

TFT_eSPI tft = TFT_eSPI();

WiFiUDP Udp;
unsigned int localPort = 8888;

int prevDisplay = -1;

byte dstOffset(byte d, byte m, unsigned int y, byte h);
time_t getDstCorrectedTime(void);
time_t getNtpTime(void);
void digitalClockDisplay(void);
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);

//####################################################################################################
// Setup
//####################################################################################################
void setup()
{
  Serial.begin(115200);
  tft.begin();

  tft.setRotation(1);  // portrait
  tft.fillScreen(TFT_BLACK);

  tft.setFreeFont(&FreeMono9pt7b);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  tft.setCursor(0, 20);

  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to ");
  DEBUG_PRINT(ssid);
  DEBUG_PRINTLN(" ...");

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    DEBUG_PRINT(++i); DEBUG_PRINT(' ');
  }

  DEBUG_PRINTLN('\n');
  DEBUG_PRINTLN("Connection established!");  
  DEBUG_PRINT("IP address:\t");
  DEBUG_PRINTLN(WiFi.localIP());

  Udp.begin(localPort);

  setSyncProvider(getDstCorrectedTime);
  setSyncInterval(300);
}

//####################################################################################################
// Main loop
//####################################################################################################
void loop(void)
{
  if (timeStatus() != timeNotSet) {
    int currDisplay = minute();
    if (currDisplay != prevDisplay) { //update the display only if time has changed
      prevDisplay = currDisplay;

      switch (currDisplay % 4) {
        case 0:  drawArrayJpeg(screen1, sizeof(screen1), 0, 0); break;
        case 1:  drawArrayJpeg(screen2, sizeof(screen2), 0, 0); break;
        case 2:  drawArrayJpeg(screen3, sizeof(screen3), 0, 0); break;
        default: drawArrayJpeg(screen4, sizeof(screen4), 0, 0); break;
      }

      digitalClockDisplay();
    }
  }
}

//####################################################################################################
// Display clock
//####################################################################################################
void digitalClockDisplay(void)
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.println();

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(0xD145, 0x0001);
  tft.setFreeFont(&Glaser_Becker_Stencil_Regular24pt7b);

  if (hour() >= 10) {
    tft.drawNumber(hour() / 10,  40, 282);
    
  } else {
    tft.drawNumber(0,  40, 282);
  }
  tft.drawNumber(hour() % 10,  80, 282);

  if (minute() >= 10) {
    tft.drawNumber(minute() / 10, 130, 282);    
  } else {
    tft.drawNumber(0, 130, 282);
  }
  tft.drawNumber(minute() % 10, 170, 282);

  if (day() >= 10) {
    tft.drawNumber(day() / 10, 221, 282);
  } else {
    tft.drawNumber(0, 221, 282);  
  }
  tft.drawNumber(day() % 10, 261, 282);

  if (month() >= 10) {
    tft.drawNumber(month() / 10, 312, 282);
  } else {
    tft.drawNumber(0, 312, 282);    
  }
  tft.drawNumber(month() % 10, 352, 282);

  tft.drawNumber(((year() - 2000) / 10), 403, 282);
  tft.drawNumber(((year() - 2000) % 10), 443, 282);
}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

//####################################################################################################
// Get DST offset
//####################################################################################################
/* This function returns the DST offset for the current UTC time.
 * This is valid for the EU, for other places see
 * http://www.webexhibits.org/daylightsaving/i.html
 *
 * Results have been checked for 2012-2030 (but should work since
 * 1996 to 2099) against the following references:
 * - http://www.uniquevisitor.it/magazine/ora-legale-italia.php
 * - http://www.calendario-365.it/ora-legale-orario-invernale.html
 */
byte dstOffset(byte d, byte m, unsigned int y, byte h)
{
  // Day in March that DST starts on, at 1 am
  byte dstOn = (31 - (5 * y / 4 + 4) % 7);

  // Day in October that DST ends  on, at 2 am
  byte dstOff = (31 - (5 * y / 4 + 1) % 7);

  if ((m > 3 && m < 10) ||
      (m == 3 && (d > dstOn || (d == dstOn && h >= 1))) ||
      (m == 10 && (d < dstOff || (d == dstOff && h <= 1))))
    return 1;
  else
    return 0;
}

//####################################################################################################
// Get DST corrected time
//####################################################################################################
time_t getDstCorrectedTime(void)
{
  time_t t = getNtpTime();

  if (t > 0) {
    TimeElements tm;
    breakTime(t, tm);
    t += (timeZone + dstOffset(tm.Day, tm.Month, tm.Year + 1970, tm.Hour)) * SECS_PER_HOUR;
  }

  return t;
}

//####################################################################################################
// Get NTP time
//####################################################################################################
time_t getNtpTime(void)
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  DEBUG_PRINTLN("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      DEBUG_PRINTLN("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL;
    }
  }
  DEBUG_PRINTLN("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

//####################################################################################################
// Send an NTP request to the time server at the given address
//####################################################################################################
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

//####################################################################################################
// Draw a JPEG on the TFT pulled from a program memory array
//####################################################################################################
void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos)
{
  int x = xpos;
  int y = ypos;

  JpegDec.decodeArray(arrayname, array_size);
  
  jpegInfo(); // Print information from the JPEG file (could comment this line out)
  
  renderJPEG(x, y);
  
  Serial.println("#########################");
}

//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void renderJPEG(int xpos, int ypos)
{
  // retrieve infomration about the image
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
  while (JpegDec.read()) {
	  
    // save a pointer to the image block
    pImg = JpegDec.pImage ;

    // calculate where the image block should be drawn on the screen
    uint32_t mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
    uint32_t mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (uint32_t h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (uint32_t w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    tft.startWrite();

    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= (uint32_t)tft.width() && ( mcu_y + win_h ) <= (uint32_t)tft.height())
    {

      // Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
      tft.setAddrWindow(mcu_x, mcu_y, win_w, win_h);

      // Write all MCU pixels to the TFT window
      while (mcu_pixels--) {
        // Push each pixel to the TFT MCU area
        tft.pushColor(*pImg++);
      }

    }
    else if ( (mcu_y + win_h) >= (uint32_t)tft.height()) JpegDec.abort(); // Image has run off bottom of screen so abort decoding

    tft.endWrite();
  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime;

  // print the results to the serial port
  Serial.print(F(  "Total render time was    : ")); Serial.print(drawTime); Serial.println(F(" ms"));
  Serial.println(F(""));
}

//####################################################################################################
// Print image information to the serial port (optional)
//####################################################################################################
void jpegInfo(void)
{
  Serial.println(F("==============="));
  Serial.println(F("JPEG image info"));
  Serial.println(F("==============="));
  Serial.print(F(  "Width      :")); Serial.println(JpegDec.width);
  Serial.print(F(  "Height     :")); Serial.println(JpegDec.height);
  Serial.print(F(  "Components :")); Serial.println(JpegDec.comps);
  Serial.print(F(  "MCU / row  :")); Serial.println(JpegDec.MCUSPerRow);
  Serial.print(F(  "MCU / col  :")); Serial.println(JpegDec.MCUSPerCol);
  Serial.print(F(  "Scan type  :")); Serial.println(JpegDec.scanType);
  Serial.print(F(  "MCU width  :")); Serial.println(JpegDec.MCUWidth);
  Serial.print(F(  "MCU height :")); Serial.println(JpegDec.MCUHeight);
  Serial.println(F("==============="));
}

//####################################################################################################
// Show the execution time (optional)
//####################################################################################################
// WARNING: for UNO/AVR legacy reasons printing text to the screen with the Mega might not work for
// sketch sizes greater than ~70KBytes because 16 bit address pointers are used in some libraries.

// The Due will work fine with the HX8357_Due library.

void showTime(uint32_t msTime)
{
  Serial.print(F(" JPEG drawn in "));
  Serial.print(msTime);
  Serial.println(F(" ms "));
}


//
//
//    IN "Ethernet.h" CHANGED
// int beginWithDHCP(uint8_t *, unsigned long timeout = 60000, unsigned long responseTimeout = 4000);
//    TO
// int beginWithDHCP(uint8_t *, unsigned long timeout = 3000, unsigned long responseTimeout = 2000);
//
//    AND
//
// static int begin(uint8_t *mac, unsigned long timeout = 60000, unsigned long responseTimeout = 4000);
//    TO
// static int begin(uint8_t *mac, unsigned long timeout = 3000, unsigned long responseTimeout = 2000);

//    "Dhcp.h"
//
//    #define DHCP_TIMEOUT            60000
//    #define DHCP_RESPONSE_TIMEOUT   4000
//
//    #define DHCP_TIMEOUT            3000
//    #define DHCP_RESPONSE_TIMEOUT   2000
//
//
//
//

#include <ESP8266WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// #define UART_DEBUG

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES 10 // Number of snowflakes in the animation example
#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16
static const unsigned char PROGMEM logo_bmp[] =
    {0b00000000, 0b11000000,
     0b00000001, 0b11000000,
     0b00000001, 0b11000000,
     0b00000011, 0b11100000,
     0b11110011, 0b11100000,
     0b11111110, 0b11111000,
     0b01111110, 0b11111111,
     0b00110011, 0b10011111,
     0b00011111, 0b11111100,
     0b00001101, 0b01110000,
     0b00011011, 0b10100000,
     0b00111111, 0b11100000,
     0b00111111, 0b11110000,
     0b01111100, 0b11110000,
     0b01110000, 0b01110000,
     0b00000000, 0b00110000};

// https://www.mischianti.org/images-to-byte-array-online-converter-cpp-arduino/

static const unsigned char PROGMEM PoEBitmap[] = { // 'PoE, 128x64px
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xcf, 0xf3, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x1f, 0x9f, 0x39, 0xf8, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x1f, 0xff, 0x80, 0x00, 0x0f, 0x80, 0x3f, 0x3e, 0x7c, 0xfc, 0x01, 0xf0, 0x00, 0xff, 0xff, 0xf0,
    0x1f, 0xff, 0xf0, 0x00, 0x0f, 0xc0, 0x7e, 0x7c, 0xfc, 0x7e, 0x03, 0xf8, 0x00, 0xff, 0xff, 0xf0,
    0x01, 0xc0, 0x7c, 0x00, 0x1f, 0xc0, 0xfc, 0x79, 0xfe, 0x3f, 0x03, 0xf8, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x1e, 0x00, 0x1f, 0xe0, 0xfc, 0xf3, 0xff, 0x3f, 0x07, 0xfc, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x0e, 0x00, 0x7f, 0xf1, 0xfc, 0xe7, 0xff, 0x3f, 0x8f, 0xfe, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x0f, 0x00, 0x7f, 0xf9, 0xfc, 0xc0, 0x0f, 0x3f, 0x9f, 0xff, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x07, 0x00, 0xff, 0xf9, 0xfc, 0xff, 0x9f, 0x3f, 0x9f, 0xff, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x07, 0x00, 0x7f, 0xf9, 0xfc, 0xff, 0xbf, 0x3f, 0x9f, 0xfe, 0x00, 0x0e, 0x00, 0x00,
    0x01, 0xc0, 0x07, 0x00, 0x7f, 0xf9, 0xfc, 0xff, 0x3f, 0x3f, 0xcf, 0xfe, 0x00, 0x0e, 0x02, 0x00,
    0x01, 0xc0, 0x07, 0x00, 0x3f, 0xf9, 0x9c, 0xf6, 0x7e, 0x3d, 0xcf, 0xfc, 0x00, 0x0e, 0x02, 0x00,
    0x01, 0xc0, 0x0f, 0x00, 0x1f, 0xf9, 0xdc, 0xf4, 0xfe, 0x79, 0xcf, 0xfc, 0x00, 0x0e, 0x02, 0x00,
    0x01, 0xc0, 0x0e, 0x00, 0x00, 0x1b, 0xde, 0x71, 0xfe, 0x79, 0xcc, 0x00, 0x00, 0x0e, 0x02, 0x00,
    0x01, 0xc0, 0x1e, 0x00, 0x00, 0x03, 0xde, 0x33, 0xfc, 0x7b, 0xc0, 0x00, 0x00, 0x0f, 0xfe, 0x00,
    0x01, 0xc0, 0x7c, 0x00, 0x00, 0x03, 0xcf, 0x30, 0xf8, 0xfb, 0xc0, 0x00, 0x00, 0x0f, 0xfe, 0x00,
    0x01, 0xff, 0xf0, 0x00, 0x00, 0x03, 0x9f, 0x9f, 0xe1, 0xf9, 0xc0, 0x00, 0x00, 0x0e, 0x02, 0x00,
    0x01, 0xff, 0x80, 0x00, 0x00, 0x03, 0xbf, 0xc3, 0xc7, 0xfc, 0xc0, 0x00, 0x00, 0x0e, 0x02, 0x00,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x01, 0x3f, 0xf8, 0x1f, 0xfc, 0xc0, 0x00, 0x00, 0x0e, 0x02, 0x00,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x01, 0x30, 0x1f, 0xf8, 0x0e, 0x80, 0x00, 0x00, 0x0e, 0x02, 0x00,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x01, 0x60, 0x0f, 0xf0, 0x06, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x00,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x07, 0xe0, 0x02, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x00,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x03, 0xc0, 0x02, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x03, 0xc0, 0x02, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x03, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xe0, 0x03, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x07, 0x70, 0x03, 0xc0, 0x00, 0x00, 0x0e, 0x00, 0x30,
    0x01, 0xc0, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x0c, 0x30, 0x03, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x30,
    0x1f, 0xfc, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x38, 0x1c, 0x07, 0x80, 0x00, 0x00, 0xff, 0xff, 0xf0,
    0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf8, 0x1f, 0x0f, 0x00, 0x00, 0x00, 0xff, 0xff, 0xf0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x7f, 0xf0, 0x0f, 0xfe, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x1f, 0xf0, 0x0f, 0xfc, 0x7f, 0xfc, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x80, 0xf0, 0x8f, 0x80, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0x79, 0x9f, 0x03, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x7f, 0xff, 0x00, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x3f, 0xfe, 0x00, 0x7f, 0xfe, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x1f, 0xfc, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x0a, 0xa8, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x35, 0x54, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x1f, 0xf8, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*
 *
  ...\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\3.0.2\libraries\lwIP_enc28j60\src\utility\enc28j60.h
  public uint16_t getLinkDual(void);

  ...\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\3.0.2\libraries\lwIP_enc28j60\src\utility\enc28j60.cpp
  #define ENC28J60_MIREGADR 0x14 // bank2
  //#define ENC28J60_MIREGADR   (MII_REG_TYPE | BANK_2 | 0x14) // 0x200 | 0x200 | 0x14
  #define ENC28J60_MICMD 0x12 // bank2
  //#define ENC28J60_MICMD   (MII_REG_TYPE | BANK_2 | 0x12) // 0x2000 | 0x0200 | 0x12
  #define ENC28J60_MICMD_MIIRD   0x01
  #define ENC28J60_MISTAT 0x0A
  //#define ENC28J60_MISTAT   (MII_REG_TYPE | BANK_3 | 0x0A) // 0x2000 | 0x0300 | 0x0A
  #define ENC28J60_MISTAT_BUSY   0x01

  //#define ENC28J60_MIRDH   (MII_REG_TYPE | BANK_2 | 0x19)
  //#define ENC28J60_MIRDL   (MII_REG_TYPE | BANK_2 | 0x18)
  #define ENC28J60_MIRDH 0x19
  #define ENC28J60_MIRDL 0x18

  //#define ENC28J60_PHSTAT1   (PHY_REG_TYPE | 0x01) // 0x3000 | 0x01
  //#define ENC28J60_PHSTAT2   (PHY_REG_TYPE | 0x11) // 0x3000 | 0x11
  #define ENC28J60_PHSTAT1 0x01
  #define ENC28J60_PHSTAT2 0x11



  uint16_t
  ENC28J60::getLinkDual(void)
  {
  auto bankOld = _bank;

  setregbank(0x02);
  writereg(ENC28J60_MIREGADR, ENC28J60_PHSTAT2);

  writereg(ENC28J60_MICMD, ENC28J60_MICMD_MIIRD);
  delay(1);
  setregbank(0x03);
  while((readreg(ENC28J60_MISTAT) & ENC28J60_MISTAT_BUSY) != 0){}

  setregbank(0x02);
  writereg(ENC28J60_MICMD, 0x00);

  uint16_t answ = 0;
  answ = readreg(ENC28J60_MIRDL);
  answ |= readreg(ENC28J60_MIRDH) << 8;

  answ = readreg(ENC28J60_MIRDL);
  answ |= readreg(ENC28J60_MIRDH) << 8;

  setregbank(bankOld);
  return answ;
  }


  ...\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\3.0.2\cores\esp8266\LwipIntfDev.h

template <class RawDev>
class LwipIntfDev: public LwipIntf, public RawDev
{
public:
  void resetNetIf(void);
  void DhcpRelease(void);
  void DhcpStop(void);
  void DhcpStart(void);

  const netif* getNetIf() const
  {
    return &_netif;
  }

  IPAddress dnsServerIP()
  {
    return IPAddress(dns_getserver(0)->addr);
  }
  ...



Outside:
  template <class RawDev>
  void LwipIntfDev<RawDev>::resetNetIf(void){
  dhcp_release(&_netif);
  dhcp_stop(&_netif);
  dhcp_start(&_netif);
  }

  template <class RawDev>
  void LwipIntfDev<RawDev>::DhcpRelease(void){
    dhcp_release(&_netif);
  }
  template <class RawDev>
  void LwipIntfDev<RawDev>::DhcpStop(void){
    dhcp_stop(&_netif);
  }
  template <class RawDev>
  void LwipIntfDev<RawDev>::DhcpStart(void){
    dhcp_start(&_netif);
  }



*/

////#include <SPI.h>
////#include <ESP8266WiFi.h>
// #include <W5500lwIP.h>
// #include <W5100lwIP.h>
#include <ENC28J60lwIP.h>
#include <utility/enc28j60.h>

bool LinkIsUp = false;
uint16_t LinkStatus = 0;

typedef enum
{
  MODE_595 = 0,
  MODE_ENC28J60 = 1,
  MODE_ENC28J60_RESETIF = 2,
  // MODE_ENC28J60_
} MODE;

bool UART_DEBUG = false;

#define Rhigh 220 // ADC high resistance kOhm
#define Rlow 50   // ADC low resistance kOhm

#define CSPIN 10 // SD3
////#define CSPIN 15 // D8
// #define CSPIN 16 // D0
// Wiznet5500lwIP eth(CSPIN);
// Wiznet5100lwIP eth(CSPIN);
ENC28J60lwIP eth(CSPIN);
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xD0};

uint8_t ethStatus = 0;
uint8_t count = 0;
uint16_t ethLink = 0;
uint8_t i = 0;

// #define BTN 9
#define btn_main 0 // D3
// #define btn_primary 9 // SD9
// #define btn_secondary 10 // SD3

// #define _595_CLK 14
// #define _595_DATA 13
#define _595_CS 16 // D0
// #define _595_CS 15 // D8
// #define _595_OE 2

#define POE_input 2 // D4
#define ALARM_PIN 15

//------------  GPIO2  - LED
//  GPIO15 - SCS D8
//  GPIO13 - MOSI D7
//  GPIO12 - MISO D6
//  GPIO14 - SCLK D5
//
//  GPIO5 - SCL D1
//  GPIO4 - SDA D2
//
//
//  GPIO10 - SD3
//  GPIO9  - SD2
//  GPIO1  - TXD0
//  GPIO3  - RXD0
//  GPIO2  - TXD1 D4
//  GPIO0  - FLASH D3
//  GPIO4  - D2
//  GPIO5  - D1
//
//  GPIO12 - MISO D6
//  GPIO13 - MOSI D7
//  GPIO14 - SCLK D5
//  GPIO15 - CS D8
//  GPIO16 - WAKE D0

uint16_t btn_status = 0;
//  0x1000 - btn_main ready
//  0x2000 - btn_primary ready
//  0x4000 - btn_secondary ready
uint16_t btn_delay;
// MODE mode = MODE_ENC28J60; //--//
MODE mode = MODE_ENC28J60_RESETIF; //--//
// uint8_t mode1_bits = 0;
uint8_t mode595_current = 0;
bool mode595_refresh = false;
uint16_t _595data = 0x0000;
// int8_t ethStatus = 0;
bool IPReceived = false;

bool POE_ALERT = false;

void BeepShort1()
{
  digitalWrite(ALARM_PIN, 1);
  delay(100);
  digitalWrite(ALARM_PIN, 0);
  // delay(300);
}

void BeepShort2()
{
  digitalWrite(ALARM_PIN, 1);
  delay(100);
  digitalWrite(ALARM_PIN, 0);
  delay(300);
  digitalWrite(ALARM_PIN, 1);
  delay(100);
  digitalWrite(ALARM_PIN, 0);
  // delay(300);
}

void BeepLong1()
{
  digitalWrite(ALARM_PIN, 1);
  delay(800);
  digitalWrite(ALARM_PIN, 0);
}

void BeepShort1Long1()
{
  digitalWrite(ALARM_PIN, 1);
  delay(100);
  digitalWrite(ALARM_PIN, 0);
  delay(200);
  digitalWrite(ALARM_PIN, 1);
  delay(800);
  digitalWrite(ALARM_PIN, 0);
}

/*
ICACHE_RAM_ATTR void POE_INT() {
  POE_ALERT = true;
}
*/
IRAM_ATTR void POE_INT()
{
  POE_ALERT = true;
}

/// Returns "true" if POE input detected
bool POE_check()
{
  if (POE_ALERT)
  {
    Send_595(0);
    _595data = 0;
    display.clearDisplay();
    display.drawBitmap(0, 0, PoEBitmap, 128, 64, 1);
    display.display();
    digitalWrite(ALARM_PIN, 1);
    while (POE_ALERT)
    {
      if (digitalRead(POE_input) == 1)
        POE_ALERT = false;
      else
      {
        delay(500);
        digitalWrite(ALARM_PIN, 0);
        delay(100);
        digitalWrite(ALARM_PIN, 1);
      }
    }
    digitalWrite(ALARM_PIN, 1);
    delay(1000);
    digitalWrite(ALARM_PIN, 0);
    delay(3000);
    return true;
  }
  else
  {
    return false;
  }
}

/// Returns "true" if POE input detected, "timeout" to wait ms, "stopOnBtn" to exit on Btn status changed
bool POE_check(uint16_t timeout, bool stopOnBtn = false)
{
  for (; timeout > 0; timeout--)
  {
    if (POE_check() || ((btn_status & 0x1000) > 0) & stopOnBtn)
      return true;
    delay(1);
  }
  return false;
}

void LCD_595(char data)
{
  uint8_t x = 4, y = 16;
  for (char i = 0; i < 8; i++)
  {
    if ((data & (1 << i)) > 0)
      display.fillRect(x, y, 12, 20, 1);
    else
      display.drawRect(x, y, 12, 20, 1);
    x += 16;
  }
}

void Send_595(uint16_t data)
{
  digitalWrite(_595_CS, 1);
  delayMicroseconds(100);
  digitalWrite(_595_CS, 0);
  delayMicroseconds(100);

  SPI.transfer(data >> 8);
  SPI.transfer(~(data & 0xff));

  delayMicroseconds(100);
  digitalWrite(_595_CS, 0);
  delayMicroseconds(100);
  digitalWrite(_595_CS, 1);
  delayMicroseconds(100);
}

void RenewLinkStatus()
{
  // ethStatus = Ethernet.linkStatus();
  //=============================================
  LinkStatus = eth.getLinkDual();
  LinkIsUp = ((LinkStatus & 0x0400) == 0x0400);
}

double GetBatteryVoltage()
{
  int adc = analogRead(A0);
  double res = adc * 1.0 / 1023 * (double)(Rhigh + Rlow) / Rlow;
  // double res = adc * 1.0 * 5.53 / 1023; // coef 5.53
  if (UART_DEBUG)
  {
    Serial.println("ADC = " + String(adc) + "  " + String(res) + " v");
    display.setCursor(0, 48);
    display.print("ADC " + String(adc) + " " + String(res) + " v");
  }
  return res;
}

void RenderNotificationArea()
{
  display.setCursor(0, 0);
  RenewLinkStatus();
  if (UART_DEBUG)
    Serial.print(String(count % 1000) + " Link 0x" + String(LinkStatus, HEX));
  display.print(String(count % 1000) + " Link 0x" + String(LinkStatus, HEX));
  if (LinkIsUp)
  {
    display.print(" ON");
    if (UART_DEBUG)
      Serial.println(" ON");
  }
  else
  {
    display.print(" OFF");
    if (UART_DEBUG)
      Serial.println(" OFF");
  }

  display.drawRect(110, 0, 14, 8, 1);
  double voltage = GetBatteryVoltage();
  if (voltage > 3.3)
  {
    display.fillRect(112, 2, (int)(10 * (voltage > 4.2 ? 1 : ((voltage - 3.3) / 0.9))), 4, 1);
  }
  display.fillRect(124, 2, 2, 4, 1);
  // display.display();
}

void setup()
{

  delay(100);

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(400000);
  SPI.begin();

  pinMode(_595_CS, OUTPUT);
  pinMode(POE_input, INPUT);
  pinMode(btn_main, INPUT_PULLUP);
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(_595_CS, 1);
  digitalWrite(ALARM_PIN, 0);
  delay(100);
  Send_595(0);

  Serial.begin(9600);
  Serial.println();
  Serial.print(ESP.getFullVersion());
  Serial.println();

  if (GetBatteryVoltage() < 3.0) //--//
  {                              //--//
    BeepShort1();                //--//
    while (true)                 //--//
    {                            //--//
      ESP.wdtFeed();             //--//
      delay(1000);               //--//
    }                            //--//
  }                              //--//

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    if (UART_DEBUG)
      Serial.println(F("SSD1306 allocation failed"));
    while (true)
      delay(1000); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(500);
  if (digitalRead(btn_main) == 0)
  {
    UART_DEBUG = true;
    BeepShort2();
    Serial.println("Serial Debug Enabled");
  }
  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);

  display.setCursor(0, 0);
  display.println("Init...");
  display.println("SSD1306...OK");
  display.print("Eth...");

  eth.setDefault(); // use ethernet for default route

  //
  //
  //
  //
  //

  int present = eth.begin(mac, 1000);
  // Serial.print("Present: ");
  // Serial.println(present);
  Serial.println("Present " + String(present));
  display.println(String(present));
  display.display();
  if (!present)
  {
    Serial.println("no ethernet hardware present");
    while (1)
      ;
  }
  //
  ///
  //
  //
  //
  //

  display.println("MAC:");
  for (i = 0; i < 6; i++)
  {
    if (UART_DEBUG)
    {
      Serial.print(mac[i], HEX);
      Serial.print((i == 5) ? '\n' : '-');
    }
    display.print(mac[i], HEX);
    display.print((i == 5) ? '\n' : '-');
  }

  display.setCursor(0, 48);
  display.print("github.com/RD-433/\nL1L2-EthTester-v0.9");
  display.display();
  delay(4000);

  noInterrupts();                  // Switch off interrupts whilst they are set up
  timer0_isr_init();               // Initialise Timer0
  timer0_attachInterrupt(ISR_TMR); // Goto the ISR function below when an interrupt occurs
  // timer0_write(ESP.getCycleCount() + 80000000L); // Pre-load timer with a time value (1-second)
  timer0_write(ESP.getCycleCount() + 800000L);                        // Pre-load timer with a time value (10-ms)
  attachInterrupt(digitalPinToInterrupt(POE_input), POE_INT, CHANGE); // CHANGE, FALLING, RISING
  interrupts();                                                       // Switch interrupts back on

  if (GetBatteryVoltage() < 3.4)
  {
    // Send_595(0); //--//
    // display.clearDisplay();
    // if (GetBatteryVoltage() < 3.0) //--//
    // {                              //--//
    //   BeepShort1();                //--//
    //   while (true)                 //--//
    //     ;                          //--//
    // }                              //--//
    display.setCursor(0, 0);
    display.print("Low Battery!\n\nPress any key to\ncontinue");
    display.display();
    BeepShort1Long1();
    btn_status &= 0xEFFF;
    while (!((btn_status & 0x1000) > 0))
    {
      POE_check(1000, true);
    }
    btn_status &= 0xEFFF;
    BeepShort1();
  }
}

void TestLink()
{
  // Serial.print("TESTING ENC28J60 LINK REG BIN PHSTAT2 ----- ");
  // auto link = eth.getLink();
  // Serial.println(link, BIN);
  Serial.print("TESTING ENC28J60 LINK REG BIN PHSTAT2 DUAL----- ");
  auto link = eth.getLinkDual();
  Serial.println(link, BIN);
  Serial.println();
  Serial.println();
}

void loop()
{

  ESP.wdtFeed();
  /*
    #ifdef UART_DEBUG
    Serial.print(count);
    Serial.print(" ");
    #endif
  */
  POE_check();
  display.clearDisplay();
  // display.display();
  if ((btn_status & 0x1000) > 0)
  {
    btn_status &= 0xEFFF;
    BeepShort1();
    switch (mode)
    {
    case MODE::MODE_595:
      if (UART_DEBUG)                       //--//
      {                                     //--//
        mode = MODE::MODE_ENC28J60;         //--//
      }                                     //--//
      else                                  //--//
      {                                     //--//
        mode = MODE::MODE_ENC28J60_RESETIF; //--//
      }                                     //--//
      break;
    case MODE::MODE_ENC28J60:
      mode = MODE::MODE_ENC28J60_RESETIF;
      break;
    case MODE::MODE_ENC28J60_RESETIF:
      mode = MODE::MODE_595;
      break;
    default:
      break;
    }
    Send_595(0);
    _595data = 0;
    display.setCursor(0, 7);
    display.print("Mode: ");
    display.print(mode);
    display.display();
    if (UART_DEBUG)
    {
      Serial.print("Mode: ");
      Serial.println(mode);
    }
    count = 0;
    // delay(1000);
    POE_check(500, true);
    display.clearDisplay();
  }

  RenderNotificationArea();

  if (mode == MODE::MODE_595)
  {
    if (count >= 8)
      count = 0;
    if (count == 0)
      i = 10;
    else
      i = 25;
    _595data = 1 << count;

    display.fillRect(0, 0, 5, 7, 0);
    display.setCursor(0, 0);
    display.print(count + 1);

    mode595_current = 0;
    LCD_595(_595data & 0xff);
    display.display();
    if (POE_check())
      return;
    for (; i < 30; i++)
    {
      if ((count == 0) & (i % 3 == 0))
        _595data ^= 1;
      Send_595(_595data);
      if (POE_check(100, true))
        break;
    }
  }
  else if ((mode == MODE::MODE_ENC28J60) || (mode == MODE::MODE_ENC28J60_RESETIF))
  {
    display.setCursor(0, 8);
    display.display();

    ethStatus = eth.status();
    if (UART_DEBUG)
    {
      Serial.print("Status: ");
      Serial.println(ethStatus);
    }

    if (LinkIsUp)
    {
      if ((ethStatus == 255) || (ethStatus == 3))
      {
        if (eth.connected() && !eth.getLinkDual())
        {
          int present = eth.begin(mac, 1000);
          if (UART_DEBUG)
          {
            TestLink();
            Serial.print("Present: ");
            Serial.println(present);
          }
          if (!present)
          {
            display.print("HW_ERROR");
            Serial.println("no ethernet hardware present");
            display.display();
            while (1)
              ;
          }
        }
      }

      ethStatus = 20;
      while (!eth.connected() && ethStatus > 0)
      {
        if (UART_DEBUG)
          Serial.print('.');
        display.print('.');
        display.display();
        if (POE_check(1000, true))
          break;
        RenewLinkStatus();
        if (!LinkIsUp)
          break;
        ethStatus--;
        if (mode == MODE::MODE_ENC28J60_RESETIF)
        {
          if (ethStatus % 5 == 0)
          {
            eth.resetNetIf();
            // eth.DhcpRelease();
            POE_check(500);
          }
        }
      }

      if (eth.connected())
      {
        BeepShort2();
        if (UART_DEBUG)
        {
          Serial.println();
          Serial.print("DHCP IP:  ");
          Serial.println(eth.localIP());
          Serial.print("Mask:     ");
          Serial.println(eth.subnetMask());
          Serial.print("Gateway:  ");
          Serial.println(eth.gatewayIP());
          Serial.print("DNS:      ");
          Serial.println(eth.dnsServerIP());
        }
        display.setCursor(0, 16);
        display.print("IP:  ");
        display.println(eth.localIP());
        display.print("     ");
        display.println(eth.subnetMask());
        display.print("GW:  ");
        display.println(eth.gatewayIP());
        display.print("DNS: ");
        display.println(eth.dnsServerIP());
        display.display();

        display.fillRect(0, 8, 120, 7, 0); //--//
        display.setCursor(0, 8);
        display.print("Renewing IP in 5 s");
        for (i = 5; i > 0; i--)
        {
          display.fillRect(90, 8, 5, 7, 0);
          display.setCursor(90, 8);
          display.print(i);
          display.display();
          if (POE_check(1000, true))
            break;
          RenewLinkStatus();
          if (!LinkIsUp)
            break;
        }

        // eth.resetNetIf(logger, 0);
        /*
        if (mode == 1)
        {
          eth.resetNetIf();
        }
        else if (mode == 2)
        {
          eth.DhcpRelease();
        }
        else if (mode == 3)
        {
          eth.DhcpStop();
          eth.DhcpStart();
        }
        else
        {
          Serial.println("UNKNOWN MODE!");
        }
        */
        eth.resetNetIf();

        // eth.dhcp_release(eth.getNetIf());
        // eth.dhcp_stop(eth.getNetIf());
        // eth.dhcp_start(eth.getNetIf());
        // delay(1000);
        POE_check(1000, true);
      }
      else
      {
        // TestLink();
        /*
        display.setCursor(0, 16);
        display.print("failed");
        Serial.print("failed");
        Serial.println();
        display.display();
        delay(1000);
*/
        if (UART_DEBUG)
          Serial.println("\nDHCP server is not found/responding.\n");
        // display.fillRect(0, 8, 60, 7, 0); //--//
        display.println("\nDHCP 0 = no response");
        display.display();
        BeepLong1();
        POE_check(1000, true);
        display.print("Trying again in 5 s");
        for (i = 5; i > 0; i--)
        {
          display.fillRect(96, 24, 5, 7, 0);
          display.setCursor(96, 24);
          display.print(i);
          display.display();
          if (POE_check(1000, true))
            break;
          RenewLinkStatus();
          if (!LinkIsUp)
            break;
        }
      }
    }
    else
    {
      POE_check(2000, true);
    }
  }
  else
  {
  }
  count++;
}

ICACHE_RAM_ATTR void ISR_TMR(void)
{
  timer0_write(ESP.getCycleCount() + 800000L);

  if ((btn_status & 0xF) == 0)
  {
    if (digitalRead(btn_main) == 0)
    {
      btn_status |= 1;
      btn_delay = 10;
    }
  }
  else if ((btn_status & 0xF) == 1)
  {
    btn_delay--;
    if (btn_delay == 0)
      btn_status |= 2;
  }
  else if ((btn_status & 0xF) == 3)
  {
    if (digitalRead(btn_main) == 1)
      btn_status &= 0xFFF0;
    else
      btn_status |= 4;
  }
  else if ((btn_status & 0xF) == 7)
  {
    if (digitalRead(btn_main) == 1)
      btn_status |= 8;
  }
  else if ((btn_status & 0xF) == 0xF)
  {
    if (digitalRead(btn_main) == 1)
      btn_status |= 0x1000;
    btn_status &= 0xFFF0;
  }

  if (mode == MODE::MODE_595)
  {
    if (mode595_current >= 3)
      mode595_current = 0;
    else
      mode595_current++;
    if ((_595data & 0xff) == (1 << (mode595_current + 0)))
    {
      if (mode595_current >= 3)
        mode595_current = 0;
      else
        mode595_current++;
    }
    _595data = (_595data & 0xff) | (1 << (mode595_current + 8));
    mode595_refresh = true;
  }

  timer0_write(ESP.getCycleCount() + 800000L);
}

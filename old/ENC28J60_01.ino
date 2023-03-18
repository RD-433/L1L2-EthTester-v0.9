//#include <splash.h>

//  Подключаем стандартную библиотеку для работы с Shield'ом по шине SPI
//#include "SPI.h"
//  Подключаем стандартную библиотеку для работы с Ethernet
//#include "UIPEthernet.h"

//
//
//    IN "Ethernet.h" CHANGED
//int beginWithDHCP(uint8_t *, unsigned long timeout = 60000, unsigned long responseTimeout = 4000);
//    TO
//int beginWithDHCP(uint8_t *, unsigned long timeout = 3000, unsigned long responseTimeout = 2000);
//
//    AND
//
//static int begin(uint8_t *mac, unsigned long timeout = 60000, unsigned long responseTimeout = 4000);
//    TO
//static int begin(uint8_t *mac, unsigned long timeout = 3000, unsigned long responseTimeout = 2000);




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




//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>

#include <ESP8266WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define UART_DEBUG

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
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
  0b00000000, 0b00110000
};




//#include "GyverButton.h"

void Send_595 (uint16_t);
void LCD_595 (char);

/*
  //IPAddress ip(192, 168, 33, 177);
  //IPAddress myDns(192, 168, 33, 1);
  //byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  //EthernetClient client;

  //#include <UIPEthernet.h>
  // The connection_data struct needs to be defined in an external file.
  //#include <UIPServer.h>
  //#include <UIPClient.h>
  //EthernetClient client;

  #include <ENC28J60lwIP.h>
  //#define CSPIN 15
  ENC28J60lwIP eth(10);

  //uint8_t mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xD0 };
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  //uint8_t myIP[4] = { 192, 168, 0, 10 };
  //uint8_t myMASK[4] = { 255, 255, 255, 0 };
  //uint8_t myDNS[4] = { 192, 168, 0, 1 };
  //uint8_t myGW[4] = { 192, 168, 0, 1 };
*/


/*
   typedef enum {
    WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
    WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_WRONG_PASSWORD   = 6,
    WL_DISCONNECTED     = 7
  } wl_status_t;


  ...\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\3.0.2\libraries\lwIP_enc28j60\src\utility\enc28j60.cpp
  ...\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\3.0.2\libraries\lwIP_enc28j60\src\utility\enc28j60.h
  public uint16_t getLinkDual(void);

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

public:
  void resetNetIf(void);
	void DhcpRelease(void);
	void DhcpStop(void);
	void DhcpStart(void);

  const netif* getNetIf() const
  {
    return &_netif;
  }



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
//#include <W5500lwIP.h>
//#include <W5100lwIP.h>
#include <ENC28J60lwIP.h>
#include <utility/enc28j60.h>

#define CSPIN 10 // SD3
////#define CSPIN 15 // D8
//#define CSPIN 16 // D0
//Wiznet5500lwIP eth(CSPIN);
//Wiznet5100lwIP eth(CSPIN);
ENC28J60lwIP eth(CSPIN);
//byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xD0 };

uint8_t ethStatus = 0;
uint8_t count = 0;
uint16_t ethLink = 0;
uint8_t i = 0;

//#define BTN 9
#define btn_main 0 // D3
//#define btn_primary 9 // SD9
//#define btn_secondary 10 // SD3

//#define _595_CLK 14
//#define _595_DATA 13
#define _595_CS 16 // D0
//#define _595_CS 15 // D8
//#define _595_OE 2

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
int mode = 0;
//uint8_t mode1_bits = 0;
uint8_t mode595_current = 0;
bool mode595_refresh = false;
uint16_t _595data = 0x0000;
//int8_t ethStatus = 0;

bool POE_ALERT = false;

// Software SPI (slower updates, more flexible pin options):
// pin 2 - Serial clock out (SCLK)
// pin 3 - Serial data out (DIN)
// pin 4 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(2, 3, 4, 5, 6);

/*
  #define _595_CLK 2
  #define _595_DATA 3
  #define _595_CS 7
  #define _595_OE 8
  // pin 2 - 595 CLK
  // pin 3 - 595 DATA
  // pin 7 - 595 CS
  // pin 8 - 595 OE

  //  pin 10 - SCS
  //  pin 11 - MOSI
  //  pin 12 - MISO
  //  pin 13 - SCLK
*/

/*
ICACHE_RAM_ATTR void POE_INT() {
  //Serial.println("MOTION DETECTED!!!");
  //digitalWrite(led, HIGH);
  //startTimer = true;
  //lastTrigger = millis();
  POE_ALERT = true;
}
*/
IRAM_ATTR void POE_INT() {
  //Serial.println("MOTION DETECTED!!!");
  //digitalWrite(led, HIGH);
  //startTimer = true;
  //lastTrigger = millis();
  POE_ALERT = true;
}

void POE_check(){
  if (POE_ALERT){
    Send_595(0);
    _595data = 0;
    display.clearDisplay();
    display.setCursor(64, 48);
    display.print("POE");
    display.display();
    digitalWrite(ALARM_PIN, 1);
    while (POE_ALERT){
      if (digitalRead(POE_input) == 1) POE_ALERT = false;
      else delay(1000);
    }
    digitalWrite(ALARM_PIN, 0);
    display.setCursor(64, 48);
    display.print("   ");
    display.display();
  }
}

void setup() {
  /*
    //  Инициируем работу с монитором последовательного порта на скорости 9600 бод
    Serial.begin(9600);
    display.begin();
    display.setContrast(55);
    display.display();
    delay(2000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK);
  */

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(400000);

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
  delay(1000);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(1000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);

  

  eth.setDefault(); // use ethernet for default route

  

  int present = eth.begin(mac, 1500);
  Serial.print("present: ");
  Serial.println(present);
  if (!present) {
    Serial.println("no ethernet hardware present");
    while (1);
  }



  noInterrupts();                                // Switch off interrupts whilst they are set up
  timer0_isr_init();                             // Initialise Timer0
  timer0_attachInterrupt(ISR_TMR);                   // Goto the ISR function below when an interrupt occurs
  //timer0_write(ESP.getCycleCount() + 80000000L); // Pre-load timer with a time value (1-second)
  timer0_write(ESP.getCycleCount() + 800000L); // Pre-load timer with a time value (10-ms)
  //
  //attachInterrupt(digitalPinToInterrupt(GPIO), ISR, mode);
  attachInterrupt(digitalPinToInterrupt(POE_input), POE_INT, CHANGE); // CHANGE, FALLING, RISING
  //
  interrupts();                                  // Switch interrupts back on

  

}

void TestLink() {
  //Serial.print("TESTING ENC28J60 LINK REG BIN PHSTAT2 ----- ");
  //auto link = eth.getLink();
  //Serial.println(link, BIN);
  Serial.print("TESTING ENC28J60 LINK REG BIN PHSTAT2 DUAL----- ");
  auto link = eth.getLinkDual();
  Serial.println(link, BIN);
  Serial.println();
  Serial.println();
}

void loop() {

  ESP.wdtFeed();

  #ifdef UART_DEBUG
  Serial.print(count);
  Serial.print(" ");
  #endif

  display.clearDisplay();
  //display.display();
  

  if ((btn_status & 0x1000) > 0) {
    btn_status &= 0xEFFF;
    //mode == 1 ? mode = 0 : mode++;
    if (mode == 3){ 
      mode = 0;
      Send_595(0); 
    }
    else {
      POE_check();
      Send_595(0);
      mode++;
    }
    _595data = 0;
    display.setCursor(0, 7);
    display.print("Mode: ");
    display.print(mode);
    Serial.print("Mode: ");
    Serial.println(mode);
    display.display();
    count = 0;
    delay(1000);
  }




  if (mode != 0) {

    ethStatus = eth.status();
    Serial.print("status: ");
    Serial.println(ethStatus);
    ethLink = eth.getLinkDual();
    Serial.println(String(count % 1000) + " Link 0x" + String(ethLink, HEX));

    display.setCursor(0, 0);
    display.print(String(count % 1000) + " Link 0x" + String(ethLink, HEX));

    if (ethLink > 0) {
      if ((ethLink & 0x0400) != 0) {
        Serial.println("Link ON");
        display.println(" ON");
        display.display();
      }
      if ((ethStatus == 255) || (ethStatus == 3)) {
        if (eth.connected() && !eth.getLink()) {
          #ifdef UART_DEBUG
          TestLink();
          #endif
          int present = eth.begin(mac, 1500);
          Serial.print("present: ");
          Serial.println(present);
          if (!present) {
            display.print("HW_ERROR");
            Serial.println("no ethernet hardware present");
            display.display();
            while (1);
          }
        }
      }

      display.setCursor(0, 8);
      Serial.print("connecting");
      ethStatus = 21;
      while (!eth.connected() && ethStatus > 0) {
        display.print('.');
        Serial.print('.');
        display.display();
        delay(1000);
        if (eth.getLinkDual() != 0x400) { break; }
        ethStatus--;
        if (mode == 1)
        {
          /* code */
        }
        else if (mode == 2)
        {
          eth.resetNetIf();
        }
        else if (mode == 3)
        {
          if (count % 5 == 0)
          {
            eth.resetNetIf();
          }
          //eth.DhcpRelease();
        }
        //else if (mode == 4)
        //{
        //  eth.DhcpStop();
        //  eth.DhcpStart();
        //}
        else
        {
          Serial.println("UNKNOWN MODE!");
        }
      }

      if (!eth.connected()) {
        //TestLink();
        display.setCursor(0, 16);
        display.print("failed");
        Serial.print("failed");
        Serial.println();
        display.display();
        delay(1000);
      }
      else {
        display.setCursor(0, 16);
        Serial.println();
        Serial.print("IP: ");
        display.print("IP: ");
        display.println(eth.localIP());
        Serial.println(eth.localIP());
        Serial.print("SM: ");
        display.print("    ");
        display.println(eth.subnetMask());
        Serial.println(eth.subnetMask());
        Serial.print("GW: ");
        display.print("GW: ");
        display.println(eth.gatewayIP());
        Serial.println(eth.gatewayIP());
        display.display();
        delay(1000);
        //eth.resetNetIf(logger, 0);
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
        
        
        
        //eth.dhcp_release(eth.getNetIf());
        //eth.dhcp_stop(eth.getNetIf());
        //eth.dhcp_start(eth.getNetIf());
        delay(1000);
      }

    }
    else {
      //Serial.println(String(count%1000) + "Link Off");
      display.println(" OFF");
      Serial.println("Link OFF");
      display.display();
      delay(1000);
    }

  }

  
  if (mode == 0) {
    if (count >= 8) count = 0;
    if (count == 0) i = 10;
    else i = 26;
    _595data = 1 << count;

    mode595_current = 0;
    //POE_check();
    //Send_595(_595data);
    LCD_595(_595data & 0xff);
    display.display();
    
    for (; i < 30; i++){
      if ((count == 0) && (i % 3 == 0))
      {
        _595data ^= (1 << count);
      }
      POE_check();
      Send_595(_595data);
      delay(100);
    }
  }
  //if (mode != 0) {
  else {
    for (i = 0; i < 10; i++){
      POE_check();
      delay(100);
    }
  }
  count++;
}

void LCD_595(char data) {
  uint8_t x = 20, y = 16;
  for (char i = 0; i < 8; i++) {
    if ((data & (1 << i)) > 0) display.fillRect(x, y, 8, 8, 1);
    else display.drawRect(x, y, 8, 8, 1);
    x += 12;
  }
}

void Send_595(uint16_t data) {
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




ICACHE_RAM_ATTR void ISR_TMR (void) {                                // Timer reached zero, now re-load it to repeat
  timer0_write(ESP.getCycleCount() + 800000L); // Reset the timer, do this first for timing accuracy
  //LED_state = !LED_state;                        // Toggle the current LED_state
  //digitalWrite(LED_pin, LED_state);              // Flash the LED on or off
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  if ((btn_status & 0xF) == 0) {
    if (digitalRead(btn_main) == 0) {
      btn_status |= 1;
      btn_delay = 10;
    }
  }
  else if ((btn_status & 0xF) == 1) {
    btn_delay--;
    if (btn_delay == 0)btn_status |= 2;
  }
  else if ((btn_status & 0xF) == 3) {
    if (digitalRead(btn_main) == 1) btn_status &= 0xFFF0;
    else btn_status |= 4;
  }
  else if ((btn_status & 0xF) == 7) {
    if (digitalRead(btn_main) == 1) btn_status |= 8;
  }
  else if ((btn_status & 0xF) == 0xF) {
    if (digitalRead(btn_main) == 1) btn_status |= 0x1000;
    btn_status &= 0xFFF0;
  }

  if (mode == 0){
    if (mode595_current >= 3) mode595_current = 0;
    else mode595_current++;
    if ((_595data & 0xff) == (1 << mode595_current)) {
      if (mode595_current >= 3) mode595_current = 0;
      else mode595_current++;
    }
    _595data = (_595data & 0xff) | (1 << (mode595_current + 8));
    mode595_refresh = true;
  }

  timer0_write(ESP.getCycleCount() + 800000L);
}

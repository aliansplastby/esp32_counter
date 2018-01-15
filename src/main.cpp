#include <Arduino.h>
#include <U8g2lib.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "soc/rtc.h"
#include "rom/uart.h"


#include <WiFi.h>

#include <HTTPClient.h>
#include <esp_now.h>

#include <Preferences.h>
Preferences preferences;
#define MOLD_BUTTON_PIN 2
#define WIFI_CLAMP_ON

//#define WIFI_GATEWAY
//#define LORA_GATEWAY

const char* ssid = "ALIANSPLAST";
const char* password =  "300451566";

#ifndef WIFI_GATEWAY
//sendin
IPAddress ip(0,0,0,0);
IPAddress subnet(0,0,0,0);
IPAddress gateway(0,0,0,0);

unsigned int counterId=1;
unsigned int machineId=128;
unsigned int mouldId=64;

//IPAddress ip(192, 168, 100, 150); // this 3 lines for a fix IP-address
//IPAddress gateway(192, 168, 100, 1);
//IPAddress subnet(255, 255, 255, 0);

#include "SPI.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <SPI.h>
#include <LoRa.h>

//#define SOFTWARE_SPI
//#define USE_SPI_LIB

//#include <FS.h>
//#include <SD.h>
#include <mySD.h>

// WIFI_LoRa_32 ports

// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)

#define SS      18
#define RST     14
#define DI0     26
#define BAND    433E6

/*
  U8glib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
*/

//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 15, /* data=*/ 4);   // ESP32 Thing, HW I2C with pin remapping
File root;

unsigned long frame;
unsigned long cycletime;
int64_t moldCycleCounter;
unsigned long cycleStartMillis;
unsigned long unixStartTime;

unsigned int language;
unsigned int machineStatus;

#define MOLD_OPENED 0
#define MOLD_CLOSED 1
#define PRODUCTION_STOPPED 2

char productArticle[100] = "ДВ-1,0/131";
char moldInvId[4] = "963";
char machineNumber[10] = "1.12";
char machineName[32] = "ARBURG630H/2300-800";
char machineStatusText[32] = "Машина работает";
char textCycle[32] = "Цикл:";
char textSec[10] = "сек.";
char textCounter[16] = "Счетчик:";
char pcsText[12]=" шт.";

/* TIME SERVER */
unsigned int localPort = 2390;      // local port to listen for UDP packets
/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
void getTime();
unsigned long sendNTPpacket(IPAddress& address);
/* END TIME SERVER*/


// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 12, /* dc=*/ 4, /* reset=*/ 6);  // Arduboy (Production, Kickstarter Edition)
//U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_SSD1306_128X64_NONAME_F_3W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* reset=*/ 8);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* reset=*/ 8);
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 16, /* data=*/ 17, /* reset=*/ U8X8_PIN_NONE);   // ESP32 Thing, pure SW emulated I2C
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 16, /* data=*/ 17);   // ESP32 Thing, HW I2C with pin remapping
//U8G2_SSD1306_128X64_NONAME_F_6800 u8g2(U8G2_R0, 13, 11, 2, 3, 4, 5, 6, A4, /*enable=*/ 7, /*cs=*/ 10, /*dc=*/ 9, /*reset=*/ 8);
//U8G2_SSD1306_128X64_NONAME_F_8080 u8g2(U8G2_R0, 13, 11, 2, 3, 4, 5, 6, A4, /*enable=*/ 7, /*cs=*/ 10, /*dc=*/ 9, /*reset=*/ 8);
//U8G2_SSD1306_128X64_VCOMH0_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8); // same as the NONAME variant, but maximizes setContrast() range
//U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_VCOMH0_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);    // same as the NONAME variant, but maximizes setContrast() range



void updateStrings(void)
{
//  language = language & 3;

  if (language == 0)
  {
    strcpy(moldInvId, "962");
    strcpy(productArticle, "ДВ-1,0/131");
    strcpy(machineNumber, "1.12");
//    strcpy(machineName, "Arburg720H-2900-1300");
    if (machineStatus == MOLD_OPENED) {
      strcpy(machineStatusText, "Форма Разомкнута");
    }
    if (machineStatus == MOLD_CLOSED) {
      strcpy(machineStatusText, "Форма Сомкнута");
    }
    if (machineStatus == 2) {
      strcpy(machineStatusText, "Прво остановлено");
    }
    strcpy(textCycle, "Цикл:");
    strcpy(textSec, "сек.");
    strcpy(textCounter, "Счетчик:");
    strcpy(pcsText, " шт.");

  }

  if (language == 1)
  {
    strcpy(moldInvId, "962");
    strcpy(productArticle, "PAIL-1,0/131mm");
    strcpy(machineNumber, "1.12");
//    strcpy(machineName, "ARBURG720H-2900-1300");
    if (machineStatus == MOLD_OPENED) {
      strcpy(machineStatusText, "Mold opened");
    }
    if (machineStatus == MOLD_CLOSED) {
      strcpy(machineStatusText, "Mold closed");
    }
    if (machineStatus == 2) {
      strcpy(machineStatusText, "Production stopped!");
    }
    strcpy(textCycle, "Cycle:");
    strcpy(textSec, "sec.");
    strcpy(textCounter, "Counter:");
    strcpy(pcsText, " pcs.");
  }

  if (language == 3)
  {
    strcpy(moldInvId, "962");
    strcpy(productArticle, "PAIL-1,0/131mm");
    strcpy(machineNumber, "1.12");
//    strcpy(machineName, "ARBURG720H-2900-1300");
    if (machineStatus == MOLD_OPENED) {
      strcpy(machineStatusText, "Mold opened");
    }
    if (machineStatus == MOLD_CLOSED) {
      strcpy(machineStatusText, "Mold closed");
    }
    if (machineStatus == 2) {
      strcpy(machineStatusText, "Production stopped!");
    }
    strcpy(textCycle, "Zycluszeit:");
    strcpy(textSec, "sec.");
    strcpy(textCounter, "Zaehler:");
  }

  if (language ==2)
  {
    strcpy(moldInvId, "962");
    strcpy(productArticle, "[6180]1180ml/131mm");
    strcpy(machineNumber, "1.12");
//    strcpy(machineName, "Arburg720H-2900-1300");
    if (machineStatus == MOLD_OPENED) {
      strcpy(machineStatusText, "Kalıp açik");
    }
    if (machineStatus == MOLD_CLOSED) {
      strcpy(machineStatusText, "Kalip kapali");//kapalı
    }
    if (machineStatus == 2) {
      strcpy(machineStatusText, "Uretim durdu");
    }
    strcpy(textCycle, "çevrim:");
    strcpy(textCycle, "");
    strcpy(textSec, "san.");
    strcpy(textCounter, "sayici:");
    strcpy(pcsText, " par.");
  }
}

void switchLanguage(void)
{
  language=0;
  preferences.begin("COUNTER",false);
  language=preferences.getInt("language");
  language++;
  if (language == 3)
  {
    language = 0;
  }
  preferences.putInt("language",language);
  preferences.end();
  Serial.print("Language:");
  Serial.println(language);
  updateStrings();
}

void saveCounter(int64_t cnt)
{
  preferences.begin("COUNTER",false);
  preferences.putLong64("counter",cnt);
  preferences.end();
}

int64_t readCounter()
{
  int64_t l;
  preferences.begin("COUNTER",false);
  l=preferences.getLong64("counter");
  Serial.print("Counter:");
  Serial.println((uint32_t)l);
  preferences.end();
  return l;
}

void WiFiOff()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  esp_wifi_set_mode(WIFI_MODE_NULL);
//  WiFi.forceSleepBegin();
  delay(5);
}

void WiFiConnect()
{
  unsigned long m;
  m=millis();
//  esp_wifi_init(WIFI_INIT_CONFIG_DEFAULT);
//  esp_wifi_set_mode(ESP_ERR_WIFI_NOT_INIT);
//  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);
//  wifi_station_connect();

  if (ip!=0)
  {
    Serial.println("WiFi set config!");
    WiFi.config(ip, gateway, subnet);
  }
  int c = 30;
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && c > 0) {
    Serial.print(".");
    delay(300);
    c--;
  }
  if (c > 0)
  {
    Serial.println(".");
    Serial.print("Connected to the WiFi network:");
    Serial.println(ssid);
    Serial.print("Tooked time:");
    Serial.println((millis()-m)*3);

    ip = WiFi.localIP();
    Serial.print("IP: ");
    Serial.println(ip);

    subnet = WiFi.subnetMask();
    Serial.print("NETMASK: ");
    Serial.println(subnet);

    gateway = WiFi.gatewayIP();
    Serial.print("GATEWAY: ");
    Serial.println(gateway);

//    esp_wifi_set_ps(WIFI_PS_MODEM);
  }
}

extern "C" int rom_phy_get_vdd33();



void setup(void) {

  delay(50);
  Serial.begin(115200);
  delay(50);
/*SD CARD*****************************************/
/*
  Serial.println("Initializing SD card...");
  // initialize SD library with SPI pins
  //uint8_t csPin = SD_CHIP_SELECT_PIN, int8_t mosi = -1, int8_t miso = -1, int8_t sck = -1);

  if (!SD.begin(13, 21, 25, 12)) {
    Serial.println("Failed!");
    return;
  }
  else{Serial.println("OK!");}
*/
  /*SD CARD****************************************/
//  uint8_t Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin, int8_t mosiPin, int8_t misoPin, int8_t clockPin) {
//    rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
/*
    SPI.begin(13,21,25,12);
    while (!SD.begin(13, SPI, 24000000, "/sd"))
    {
      Serial.println("initialization failed. Things to check:");
      delay(200);
    };
*/

/*
  Sd2Card card;
  SdVolume volume;
  //SdFile root;
  while (!card.init(SPI_HALF_SPEED, 13, 21, 25, 12))
  {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card is inserted?");
    Serial.println("* Is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
  }
    // print the type of card
  Serial.print("\nCard type: ");
  switch(card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }
    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print("Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
*/
//  ADC_MODE(ADC_VCC); //vcc read
  /*
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  */

//  if (!LoRa.begin(BAND)) {
//    Serial.println("Starting LoRa failed!");
//    while (1);
//  }

  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);

  pinMode(MOLD_BUTTON_PIN, INPUT_PULLUP);
  /*
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high
  */
  //  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_2M);
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
  //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_240M);
  u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setFlipMode(1);
  u8g2.setContrast(1);
  u8g2.enableUTF8Print();   // enable UTF8 support for the Arduino print() function
  cycleStartMillis = millis();
  machineStatus = 0;

  moldCycleCounter = readCounter();
  WiFiConnect();
  getTime();

  float vdd = rom_phy_get_vdd33() / 1000.0;
  Serial.print("VDD:");
  Serial.println(vdd);

  btStop();

  if (!LoRa.begin(BAND,true)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  #ifndef LORA_GATEWAY
    WiFiOff();
  #endif

  switchLanguage();
}

void loraSendPacket(char* msg)
{
  /*
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  */
  Serial.print("Sending LORA packet: ");
  Serial.println(msg);
  digitalWrite(2, HIGH);
  // send packet
  LoRa.beginPacket();
  LoRa.print(msg);
  //  LoRa.print(counter);
  LoRa.endPacket();
  digitalWrite(2, LOW);
//  LoRa.end();
}

static unsigned short days[4][12] =
{
  {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
  { 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
  { 731, 762, 790, 821, 851, 882, 912, 943, 974, 1004, 1035, 1065},
  {1096, 1127, 1155, 1186, 1216, 1247, 1277, 1308, 1339, 1369, 1400, 1430},
};

char URL[512];
HTTPClient http;
int displayOn;

String* messages[128];
int messagesCnt=0;

void sendClamp(long cycleTime, long counterValue,char* eventType,unsigned long mouldOpenedTime)
{
  unsigned long epoch = (millis() * 3 / 1000) + unixStartTime;
  cycleTime *= 3;

  String et=String(eventType);
  String msg;
  msg=String(String(counterId)+String(";")+
                 String(epoch) + String(";")+
                 String(cycleTime) + String(";") +
                 String(counterValue) +String(";")+
                 String(eventType)+String(";")+
                 String(mouldOpenedTime*3)+String(";")+
                 String(machineId)+String(";")+
                 String(mouldId)+String(";."));
  //Serial.println(msg);
  messages[messagesCnt]=new String(msg);
  messagesCnt++;
  if (messagesCnt==5 || et=="PAUSE_PRODUCTION_START" || et=="PAUSE_PRODUCTION_END")
  {
    unsigned long m=millis();

    if (!LoRa.begin(BAND,true)) {
    Serial.println("Starting LoRa failed!");
    while (1);
    }
    Serial.print("LoRa begin tooked:");
    Serial.println(millis()-m);
    String s2=String("");
    for (int i=0;i<messagesCnt;i++)
    {
//        Serial.println(*messages[i]);
        s2=s2+*messages[i]+String("\r\n");
        delete(messages[i]);
    }
    s2.toCharArray(URL,512);
    loraSendPacket(&URL[0]);

    messagesCnt=0;
    Serial.print("Send tooked:");
    Serial.println((millis()-m)*3);

    LoRa.end();
  }
  /*
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    String URL = String("http://82.209.207.138:88/alians_cms1.php?cmd=execute_sql&sql=INSERT%20INTO%20ap_crm_counters_clamp(counterId,timestamp,cycletime,counterValue,eventType,mouldOpenedDuration,machineId,mouldId)%20values(")+
                 String(counterId)+String(",")+
                 String("FROM_UNIXTIME(") +String(epoch) + String("),") +
                 String(cycleTime) + String(",") +
                 String(counterValue) +String(",'")+
                 String(eventType)+String("',")+
                 String(mouldOpenedTime*3)+String(",")+
                 String(machineId)+String(",")+
                 String(mouldId)+String("")+
                 String(")");
    http.begin(URL);  //Specify destination for HTTP request
    int httpResponseCode = http.GET();   //Send the actual POST request
    #ifdef LOGGING
    Serial.println(URL);
    if (httpResponseCode > 0) {
      String response = http.getString();                       //Get the response to the request
      Serial.print("HTTP:");
      Serial.print(httpResponseCode);   //Print return code
      Serial.print("-");
      Serial.println(response);           //Print request answer
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }
    #endif
    http.end();  //Free resources
  }
  */
}
//7 pixel - u8g2_font_haxrcorp4089_t_cyrillic
//10 pixel - u8g2_font_unifont_t_cyrillic
//12 pixel - u8g2_font_cu12_t_cyrillic
//20 pixel - u8g2_font_10x20_t_cyrillic
unsigned long moldOpenTime;

int displayStatus=0;

char loraPacket[512];

int prevDisplayButtonStatus=1;
int isDisplayOn=1;
int prevCycleTime;

void loop(void) {
    displayStatus = 1;
    #ifdef LORA_GATEWAY
    {
      // try to parse packet
      int packetSize = LoRa.parsePacket();
      if (packetSize)
      {
        // received a packet
        Serial.print("Received packet '");
        // read packet
        int c=0;
        while (LoRa.available()) {
          loraPacket[c]=(char)LoRa.read();
          c++;
//          Serial.print((char)LoRa.read());
      }
      loraPacket[c]=0;
      //Serial.print(loraPacket);
      // print RSSI of packet
      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());

    char *p = &loraPacket[0];
    char *p1;
    char *str;
    char *str1;
    while ((str = strtok_r(p, ".\r\n", &p)) != NULL) // delimiter is the semicolon
    {
//     Serial.print("-");
//     Serial.print(str);
      int ii=0;
      String counterId,timestamp,cycletime,counterVal,eventType,mouldopened,machineId,mouldId;
      while ((str1 = strtok_r(str, ";", &str)) != NULL) // delimiter is the semicolon
      {
       if (ii==0) counterId=String(str1);
       if (ii==1) timestamp=String("FROM_UNIXTIME(")+String(str1)+ String(")");
       if (ii==2) cycletime=String(str1);
       if (ii==3) counterVal=String(str1);
       if (ii==4) eventType=String("'")+String(str1)+String("'");
       if (ii==5) mouldopened=String(str1);
       if (ii==6) machineId=String(str1);
       if (ii==7) mouldId=String(str1);

    if (ii==7 && WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    String URL = String("http://82.209.207.138:88/alians_cms1.php?cmd=execute_sql&sql=INSERT%20INTO%20ap_crm_counters_clamp(counterId,timestamp,cycletime,counterValue,eventType,mouldOpenedDuration,machineId,mouldId)%20values(")+
                 String(counterId)+String(",")+
                 String(timestamp) +String(",")+
                 String(cycletime) + String(",") +
                 String(counterVal) +String(",")+
                 String(eventType)+String(",")+
                 String(mouldopened)+String(",")+
                 String(machineId)+String(",")+
                 String(mouldId)+String("")+
                 String(")");
    http.begin(URL);  //Specify destination for HTTP request
    int httpResponseCode = http.GET();   //Send the actual POST request
    Serial.println(URL);
    if (httpResponseCode > 0) {
      String response = http.getString();                       //Get the response to the request
      Serial.print("HTTP:");
      Serial.print(httpResponseCode);   //Print return code
      Serial.print(" - ");
      Serial.println(response);           //Print request answer
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }
    http.end();  //Free resources
    delay(50);
    }
       Serial.print(str1);
       Serial.print(";");
       ii++;
      }
    }

      u8g2.setPowerSave(0);
      u8g2.setFontDirection(0);
      u8g2.clearBuffer();
      //  if (frame&65535>32768) {u8g2.setDrawColor(0);} else {u8g2.setDrawColor(1);}
      u8g2.setDrawColor(1);

      u8g2.setCursor(1, 9);
      u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
      String s1=String("RSSI:")+String(LoRa.packetRssi());
      String s2=String("P.SIZE:")+String(packetSize);
      u8g2.print(s1);
      u8g2.setCursor(1, 16);
      u8g2.print(s2);
      u8g2.sendBuffer();
    }
//    delay(50);
    return;
    }
    #endif //LORA_GATEWAY
  /*
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // received a packet
      Serial.print("Received packet '");
      // read packet
      while (LoRa.available()) {
        Serial.print((char)LoRa.read());
      }
      // print RSSI of packet
      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());
    }
  */
  u8g2_uint_t w;
  frame++; frame = frame & ((65536 * 2) - 1);
  /*
    if (frame>65000) {
      digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else
  */
  /*
  {
    pinMode(25, OUTPUT); //Send success, LED will bright 1 second
    pinMode(13, OUTPUT); //Send success, LED will bright 1 second
    pinMode(2, OUTPUT); //Send success, LED will bright 1 second
    digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(25, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
  }
  */
  //delay(100);
  int buttonStatus = digitalRead(MOLD_BUTTON_PIN);
  /*
    if (buttonStatus == 1)
    {
      buttonStatus = digitalRead(23);
    }
  */

  //buttonStatus=1 MOLD OPENED
  //buttonStatus=0 MOLD CLOSED
  long currentCycleTime = (millis() - cycleStartMillis) * 3;
  if (buttonStatus == 0)
  {
    /*
          pinMode(16,OUTPUT);
          digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
          delay(50);
          digitalWrite(16, LOW); // while OLED is running, must set GPIO16 to high
    */
    //    u8g2.setPowerSave(0);
    unsigned long m1 = millis();
    if (machineStatus== PRODUCTION_STOPPED)
    {
      sendClamp(m1 - cycleStartMillis, moldCycleCounter,"PAUSE_PRODUCTION_END",m1-moldOpenTime);
      machineStatus = MOLD_OPENED;
      delay(100);
    }
    if (machineStatus== MOLD_OPENED)
    {
      moldCycleCounter++;
      prevCycleTime=(m1 - cycleStartMillis)*3;
      if ((moldCycleCounter & 3) == 0)
      {
        saveCounter(moldCycleCounter);
      }
      sendClamp(m1 - cycleStartMillis, moldCycleCounter,"MOLD_CLOSE",m1-moldOpenTime);
      cycleStartMillis = m1;
      machineStatus = MOLD_CLOSED;
      delay(100);
      return;
    }
   }

  if (buttonStatus == 1 && (machineStatus == MOLD_CLOSED))
  {
    machineStatus = MOLD_OPENED;
    moldOpenTime=millis();
  }

  if (currentCycleTime > 60000 && machineStatus!=PRODUCTION_STOPPED)
  {
    machineStatus = PRODUCTION_STOPPED;
    sendClamp(0, moldCycleCounter,"PAUSE_PRODUCTION_START",0);
    //    u8g2.setPowerSave(0);
  }
  if (machineStatus==PRODUCTION_STOPPED)
  {
    /*
    displayStatus = digitalRead(0);
    displayStatus = 0;
    if (displayStatus==1)
    {
      u8g2.setPowerSave(1);
      delay(500);
      return;
    }
    else
    {
      u8g2.setPowerSave(0);
    }
    */
  }

  int displayButtonStatus=digitalRead(0);
  if (displayButtonStatus!=prevDisplayButtonStatus)
  {
    prevDisplayButtonStatus=displayButtonStatus;
    while (digitalRead(0)!=1)
    {
      delay(100);
    }
    prevDisplayButtonStatus=1;
    isDisplayOn=(isDisplayOn+1)&1;
    if (isDisplayOn==0)
    {
      u8g2.setPowerSave(1);
    }
    else
    {
      u8g2.setPowerSave(0);
    }
  }


  if (isDisplayOn == 0)
  {
    delay(90);
    return;
  }

  updateStrings();
  char machineFullName[100];

  u8g2.setFontDirection(0);

  u8g2.clearBuffer();
  //  if (frame&65535>32768) {u8g2.setDrawColor(0);} else {u8g2.setDrawColor(1);}
  u8g2.setDrawColor(1);
  u8g2.drawRBox(0, 0, 25, 12, 2);
  //  if (frame&65535>32768) {u8g2.setDrawColor(1);} else {u8g2.setDrawColor(0);}
  u8g2.setCursor(1, 9);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  u8g2.print("#");
  u8g2.setCursor(6, 10);
  u8g2.setFont(u8g2_font_crox1tb_tf);
  u8g2.print(moldInvId);

  u8g2.setDrawColor(1);
  u8g2.setCursor(28, 9);
  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  u8g2.print(productArticle);

  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 20);
  u8g2.setFont(u8g2_font_5x7_tf);

  //  strcpy(machineName, "ARBURG 150T");
  sprintf(machineFullName, "%s-%s", machineNumber, machineName);
  u8g2.print(machineFullName);

  if (frame & 65535 > 50000 && machineStatus == 2) {
    u8g2.setDrawColor(0);
  } else {
    u8g2.setDrawColor(1);
  }
  u8g2.drawRBox(0, 22, 128, 15, 0);

  u8g2.setDrawColor(0);
  if (frame & 65535 > 50000 && machineStatus == 2) {
    u8g2.setDrawColor(1);
  } else {
    u8g2.setDrawColor(0);
  }

  if (language == 0) {
    u8g2.setFont(u8g2_font_unifont_t_cyrillic);
  } else {
    u8g2.setFont(u8g2_font_timB10_tf);
  }
  if (language == 2)
  {
    u8g2.setFont(u8g2_font_timB10_tr);
  }
  w = u8g2.getUTF8Width(machineStatusText);
  u8g2.setCursor((128 - w) / 2, 20 + 13);
  u8g2.print(machineStatusText);

  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 32 + 8 + 4 + 6 + 2);
  u8g2.setFont(u8g2_font_10x20_t_cyrillic);
  if (language > 0) {
    u8g2.setFont(u8g2_font_helvB12_te);
  }
  u8g2.print(textCycle);

  int sec = currentCycleTime / 1000;
  int dsec = (currentCycleTime - sec * 1000) / 10;

  u8g2.setFont(u8g2_font_timB14_tn);
  u8g2.print(sec);
  u8g2.print(".");
  u8g2.print(dsec);

  if (dsec/10==0)
  {
    u8g2.print("0");
  }
  u8g2.print("/");
  sec = prevCycleTime / 1000;
  dsec = (prevCycleTime - sec * 1000) / 10;
  u8g2.print(sec);
  u8g2.print(".");
  u8g2.print(dsec);
  if (dsec/10==0)
  {
    u8g2.print("0");
  }
//  u8g2.print("/");



//  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
//  u8g2.print(textSec);

  u8g2.setCursor(0, 63);
  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  if (language==2)
  {
    u8g2.setFont(u8g2_font_lucasfont_alternate_tr);
  }
  u8g2.print(textCounter);

  //  u8g2.setFont(u8g2_font_crox1tb_tf);
  int c1 = moldCycleCounter / 1000000;
  int c2 = (moldCycleCounter - c1 * 1000000) / 1000;
  int c3 = (moldCycleCounter - c1 * 1000000 - c2 * 1000);
  if (c1>0)
  {
  u8g2.print(c1);
  u8g2.print(".");
  }
  if (c2>0)
  {
  u8g2.print(c2);
  u8g2.print(".");
  }
  u8g2.print(c3);
  u8g2.print(pcsText);

  //display time;
  unsigned long epoch = (millis() * 3 / 1000) + unixStartTime;
  epoch += 3 * 3600;

  byte second = epoch % 60; epoch /= 60;
  byte minute = epoch % 60; epoch /= 60;
  byte hour   = epoch % 24; epoch /= 24;

  unsigned int years = epoch / (365 * 4 + 1) * 4; epoch %= 365 * 4 + 1;
  unsigned int year;
  for (year = 3; year > 0; year--)
  {
    if (epoch >= days[year][0])
      break;
  }
  unsigned int month;
  for (month = 11; month > 0; month--)
  {
    if (epoch >= days[year][month])
    {
      break;
    }
  }
  year  = years + year;
  month = month + 1;
  unsigned int day   = epoch - days[year][month] + 1;

  u8g2.setCursor(88, 63);
  //  u8g2.print("");
  u8g2.print(hour);
  u8g2.print(":");
  if (minute < 10)   u8g2.print('0');
  u8g2.print(minute);
  if ((second & 1) == 0) {
    u8g2.print(":");
  } else {
    u8g2.print(" ");
  }
  if (second < 10)   u8g2.print('0');
  u8g2.print(second);


  u8g2.sendBuffer();

  //  esp_deep_sleep(1000000/10);
  delay(30);
}


void getTime()
{
  Serial.println("Starting UDP");
  if (udp.begin(localPort) == 1)
  {
    Serial.println("UDP Started!");
  }
  //  Serial.println(udp.localPort());

  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server

  // wait to see if a reply is available
  int c = 20;
  while ((c--) > 0)
  {
    delay(100);
    int cb = udp.parsePacket();
    if (!cb) {
      Serial.println("no packet yet");
      sendNTPpacket(timeServerIP); // send an NTP packet to a time server
      delay(500);
    }
    else {
      Serial.print("packet received, length=");
      Serial.println(cb);
      // We've received a packet, read the data from it
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:

      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      Serial.print("Seconds since Jan 1 1900 = " );
      Serial.println(secsSince1900);

      // now convert NTP time into everyday time:
      Serial.print("Unix time = ");
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // print Unix time:
      Serial.println(epoch);
      unixStartTime = epoch;
      // print the hour, minute and second:
      Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
      Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
      Serial.print(':');
      if ( ((epoch % 3600) / 60) < 10 ) {
        // In the first 10 minutes of each hour, we'll want a leading '0'
        Serial.print('0');
      }
      Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
      Serial.print(':');
      if ( (epoch % 60) < 10 ) {
        // In the first 10 seconds of each minute, we'll want a leading '0'
        Serial.print('0');
      }
      Serial.println(epoch % 60); // print the second
      break;
    }
  }
}
// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
#else
/**
 * ESP-NOW to Watson IoT Gateway Example
 *
 * This shows how to use an ESP8266/Arduino as a Gateway device on the Watson
 * IoT Platform enabling remote ESP-NOW devices to be Watson IoT Devices.
 *
 * Author: Anthony Elder
 * License: Apache License v2
 */
//#include <ESP8266WiFi.h>
//#include <PubSubClient.h>
//extern "C" {
//  #include <espnow.h>
//  #include "user_interface.h"
//}

//-------- Customise these values -----------
//const char* ssid = "<yourSSID>";
//const char* password = "<yourWifiPassword>";

#define ORG "<yourOrg>"
#define DEVICE_TYPE "<yourGatewayType>"
#define DEVICE_ID "<yourGatewayDevice>"
#define TOKEN "<yourGatewayToken>"
//-------- Customise the above values --------

/* Set a private Mac Address
 *  http://serverfault.com/questions/40712/what-range-of-mac-addresses-can-i-safely-use-for-my-virtual-machines
 * Note: the point of setting a specific MAC is so you can replace this Gateway ESP8266 device with a new one
 * and the new gateway will still pick up the remote sensors which are still sending to the old MAC
 */
uint8_t mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
void initVariant() {
  wifi_set_macaddr(SOFTAP_IF, &mac[0]);
}

WiFiClient wifiClient;
String deviceMac;

volatile boolean haveReading = false;

/* Presently it doesn't seem posible to use both WiFi and ESP-NOW at the
 * same time. This gateway gets around that be starting just ESP-NOW and
 * when a message is received switching on WiFi to sending the MQTT message
 * to Watson, and then restarting the ESP. The restart is necessary as
 * ESP-NOW doesn't seem to work again even after WiFi is disabled.
 * Hopefully this will get fixed in the ESP SDK at some point.
 */

void setup() {
  Serial.begin(115200); Serial.println();
  WiFi.mode(WIFI_STA);
  Serial.print("This node AP mac: "); Serial.println(WiFi.softAPmacAddress());
  Serial.print("This node STA mac: "); Serial.println(WiFi.macAddress());

  initEspNow();
}

int heartBeat;

void loop() {
  if (millis()-heartBeat > 30000) {
    Serial.println("Waiting for ESP-NOW messages...");
    heartBeat = millis();
  }

  if (haveReading) {
    haveReading = false;
    wifiConnect();
//    sendToWatson();
    client.disconnect();
    delay(200);
    ESP.restart(); // <----- Reboots to re-enable ESP-NOW
  }
}

void sendToWatson() {
}

void initEspNow() {
  if (esp_now_init()!=0) {
    Serial.println("*** ESP_Now init failed");
    ESP.restart();
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  esp_now_register_recv_cb([](uint8_t *mac, uint8_t *data, uint8_t len) {

    deviceMac = "";
    deviceMac += String(mac[0], HEX);
    deviceMac += String(mac[1], HEX);
    deviceMac += String(mac[2], HEX);
    deviceMac += String(mac[3], HEX);
    deviceMac += String(mac[4], HEX);
    deviceMac += String(mac[5], HEX);

    memcpy(&sensorData, data, sizeof(sensorData));

    Serial.print("recv_cb, msg from device: "); Serial.print(deviceMac);
//    Serial.printf(" Temp=%0.1f, Hum=%0.0f%%, pressure=%0.0fmb\n",
//       sensorData.temp, sensorData.humidity, sensorData.pressure);
    haveReading = true;
  });
}
lan
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to "); Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
     delay(250);
     Serial.print(".");
  }
  Serial.print("\nWiFi connected, IP address: "); Serial.println(WiFi.localIP());
}


void publishTo(const char* topic, const char* payload) {
  Serial.print("publish ");
  /*
  if (client.publish(topic, payload)) {
    Serial.print(" OK ");
  } else {
    Serial.print(" FAILED ");
  }
  Serial.print(" topic: "); Serial.print(topic);
  Serial.print(" payload: "); Serial.println(payload);
  */
}
#endif

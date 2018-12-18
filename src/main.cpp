#define U8G2_ENABLE //40KB
//#define DHT12_ENABLE
//#define SD_ENABLE //40KB
//#define LORA32_ENABLE //7KB
#define WIFI_ENABLE //430KB
#define MODBUS_ENABLE
#define HTTPCLIENT_ENABLE //JSON+HTTP=5KB
//#defitane SONAR_HC04_ENABLE
#define BLE_ENABLE //750kb
//#define ROTARY_ENABLE //3kb
//#define PREFERENCES_ENABLE //8kb
//#define LORA_GATEWAY

#include <Arduino.h>

/** Build time */
const char compileDate[] = __DATE__ " " __TIME__;
/** Unique device name */
char apName[] = "MOLDMON-xxxxxxxxxxxx";

#define LED_DATA_PIN 2
#define MOLD_BUTTON_PIN 17
#define COUNTER_REMOVE_BUTTON_PIN 0
#define BLUE_LED_PIN 25

static const int rotorButtonPin = 0;	// the number of the pushbutton pin
static const int rotorPinA = 22;	// One quadrature pin
static const int rotorPinB = 2;	// the other quadrature pin

char scrollingString[255];
int scrollingPos;

#ifdef DHT12_ENABLE
#include <DHT12.h>
DHT12 DHT;
#endif
int humidity;
int temperature;

#ifdef SONAR_HC04_ENABLE
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;
// defines variables
int distance;
//#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
//NewPing sonar=NewPing(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // Each sensor's trigger pin, echo pin, and max distance to ping.
#endif

#include <SmartLeds.h>
const int LED_COUNT = 1;
const int CHANNEL = 0;
// SmartLed -> RMT driver (WS2812/WS2812B/SK6812/WS2813)
SmartLed leds( { 350, 1500, 1500, 350, 50000 }, LED_COUNT, LED_DATA_PIN, CHANNEL, DoubleBuffer );
//SmartLed leds1( LED_WS2812B, LED_COUNT, 2, CHANNEL, DoubleBuffer );
uint8_t hue;
uint8_t v;

void showRgb(int r=0,int g=0,int b=0) {
    leds[ 0 ] = Rgb{ (uint8_t)r,(uint8_t)g,(uint8_t)b };
    leds.show();
}

volatile uint8_t led_color = 0;          // a value from 0 to 255 representing the hue
volatile uint8_t led_brightness2= 0;  // 255 is maximum brightness, but can be changed.  Might need 256 for common anode to fully turn off.
volatile uint8_t led_brightness = 255;  // 255 is maximum brightness, but can be changed.  Might need 256 for common anode to fully turn off.
volatile uint32_t R, G, B;           // the Red Green and Blue color components
//#define GREEN_LED 5
//#define RED_LED 18
#define BLUE 1
#define GREEN 2
#define RED 4


#ifdef ROTARY_ENABLE
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter = 0;
#include <Button.h>
#include <TicksPerSecond.h>
#include <RotaryEncoderAcelleration.h>
#endif

#include <NewPing.h>

#ifdef U8G2_ENABLE
#include <U8g2lib.h>
#endif

#include "esp_system.h"
#include "esp_wifi.h"
#include "soc/rtc.h"
#include "rom/uart.h"

#ifdef WIFI_ENABLE
const char* ssid = "ALIANSPLAST";
const char* password =  "300451566";

bool usePrimAP = true;
/** SSIDs of local WiFi networks */
String ssidPrim;
String ssidSec;
/** Password for local WiFi network */
String pwPrim;
String pwSec;

#include <WiFi.h>

#ifdef MODBUS_ENABLE
#include <ModbusIP_ESP8266.h>
//ModbusIP object
ModbusIP mb;
#endif
#endif

#ifdef HTTPCLIENT_ENABLE
#include <HTTPClient.h>
#include <JsonParser.h>
#endif
//#include <esp_now.h>

#ifdef PREFERENCES_ENABLE
#include <Preferences.h>
Preferences preferences;
#endif

#define WIFI_CLAMP_ON

#define LORA_CLK 5   //
#define LORA_MOSI 27 //
#define LORA_MISO 19 //REUSED IN SD/MISO
#define LORA_CS      18 //REUSED IN SD/CLK

#define LORA_RST     14
#define LORA_DI0     26
#define LORA_BAND    433E6

#define SD_CLK 18  //YELLOW
#define SD_MISO 19 //PURPLE(27' on board)
#define SD_MOSI 23 //BLUE
#define SD_CS 22 //GREEN


// Courtesy http://www.instructables.com/id/How-to-Use-an-RGB-LED/?ALLSTEPS
// function to convert a color to its Red, Green, and Blue components.
void hueToRGB(uint8_t hue, uint8_t brightness)
{
    const boolean invert = false; // set true if common anode, false if common cathode
    uint16_t scaledHue = (hue * 6);
    uint8_t segment = scaledHue / 256; // segment 0 to 5 around the
                                            // color wheel
    uint16_t segmentOffset =
      scaledHue - (segment * 256); // position within the segment

    uint8_t complement = 0;
    uint16_t prev = (brightness * ( 255 -  segmentOffset)) / 256;
    uint16_t next = (brightness *  segmentOffset) / 256;

    if(invert)
    {
      brightness = 255 - brightness;
      complement = 255;
      prev = 255 - prev;
      next = 255 - next;
    }

    switch(segment ) {
    case 0:      // red
        R = brightness;
        G = next;
        B = complement;
    break;
    case 1:     // yellow
        R = prev;
        G = brightness;
        B = complement;
    break;
    case 2:     // green
        R = complement;
        G = brightness;
        B = next;
    break;
    case 3:    // cyan
        R = complement;
        G = prev;
        B = brightness;
    break;
    case 4:    // blue
        R = next;
        G = complement;
        B = brightness;
    break;
   case 5:      // magenta
    default:
        R = brightness;
        G = complement;
        B = prev;
    break;
    }
}

//sendin
IPAddress ip(192,168,1,211);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,1,1);

unsigned int counterId=1;
unsigned int machineId=128;
unsigned int mouldId=64;

//IPAddress ip(192, 168, 100, 150); // this 3 lines for a fix IP-address
//IPAddress gateway(192, 168, 100, 1);
//IPAddress subnet(255, 255, 255, 0);

#ifdef U8G2
#include "SPI.h"
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
//#include <Wire.h>
#endif
#endif

#ifdef LORA32_ENABLE
#include <LoRa.h>
#endif

//#define SOFTWARE_SPI
//#define USE_SPI_LIB

#ifdef SD_ENABLE
#include <FS.h>
#include <SD.h>
File root;
#endif

// WIFI_LoRa_32 ports

// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)

#ifdef ROTARY_ENABLE
static Button btn;
static RotaryEncoderAcelleration rotor;
long lastRotor = 0;
#endif

//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
#ifdef U8G2_ENABLE
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 15, /* data=*/ 4);   // ESP32 Thing, HW I2C with pin remapping

//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 15, /* data=*/ 4); //128 bytes buffer
//U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 15, /* data=*/ 4);//256 bytes buffer
#endif


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

char msg[128] = "______";
char productArticle[100] = "ДВ-1,0/131 2.1,2.2";
char moldInvId[4] = "746";
char machineNumber[10] = "1.12";
char machineName[32] = "ARBURG630H/2300-800";
char machineStatusText[64] = "Машина работает";
char textCycle[32] = "Цикл:";
char textSec[10] = "сек.";
char textCounter[16] = "Счетчик:";
char pcsText[12]=" шт.";

/* TIME SERVER */
#ifdef WIFI_ENABLE
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
#endif

unsigned long ledB=LOW;
unsigned long lastBlinkTimeMillis=0;

void BlinkRedLED(unsigned int val=-1)
{
  if (val==-1)
	{
    if (millis()-lastBlinkTimeMillis<500)
      return;
    lastBlinkTimeMillis=millis();
	   ledB=(ledB==LOW?HIGH:LOW);
     leds[ 0 ] = Rgb{ 0,(uint8_t)(ledB*255/4), 0 };
	}
  else
  {
    leds[ 0 ] = Rgb{ (uint8_t)val, 0, 0 };
  }
  leds.show();
  leds.wait();
}

void BlinkGreenLED(unsigned int val=-1)
{
  if (val==-1)
	{
    if (millis()-lastBlinkTimeMillis<500)
      return;
    lastBlinkTimeMillis=millis();
	   ledB=(ledB==LOW?HIGH:LOW);
     leds[ 0 ] = Rgb{ uint8_t(ledB*255/4), 0, 0 };
	}
  else
  {
    leds[ 0 ] = Rgb{ 0, (uint8_t)val, 0 };
  }
  leds.show();
  leds.wait();
}

void BlinkBlueLED(unsigned int val=-1)
{
	if (val==-1)
	{
    if (millis()-lastBlinkTimeMillis<500)
      return;
    lastBlinkTimeMillis=millis();
	  ledB=(ledB==LOW?HIGH:LOW);
    leds[ 0 ] = Rgb{ 0, 0, (uint8_t)(ledB*255/4) };
	}
  else
  {
    leds[ 0 ] = Rgb{ 0, 0, (uint8_t)val };
  }
  leds.show();
  leds.wait();
}

void BlinkPinkLED(unsigned int val=-1)
{
	if (val==-1)
	{
    if (millis()-lastBlinkTimeMillis<500)
      return;
    lastBlinkTimeMillis=millis();
	  ledB=(ledB==LOW?HIGH:LOW);
    leds[ 0 ] = Rgb{ (uint8_t)(0x18*ledB/4),(uint8_t)(0xFF*ledB/4),(uint8_t)(0x94*ledB/4) };
	}
  else
  {
    leds[ 0 ] = Rgb{ 0, 0, 0 };
  }
  leds.show();
  leds.wait();
}

void BlinkYellowLED(unsigned int val=-1)
{
	if (val==-1)
	{
    if (millis()-lastBlinkTimeMillis<500)
      return;
    lastBlinkTimeMillis=millis();
	  ledB=(ledB==LOW?HIGH:LOW);
    leds[ 0 ] = Rgb{ (uint8_t)(0x80*ledB),(uint8_t)(0xFF*ledB),  0 };
	}
  else
  {
    leds[ 0 ] = Rgb{ 0, 0, 0 };
  }
  leds.show();
  leds.wait();
}

#ifdef WIFI_ENABLE
void getTime();
unsigned long sendNTPpacket(IPAddress& address);
#endif
/* END TIME SERVER*/

/*
void loopTask(void *pvParameters)
{
	setup();
	for (;;) {
		loop();
//		vTaskDelay(10);
	}
}
extern "C" void app_main()
{
	initArduino();
	xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, 0);
}
*/

#ifdef ROTARY_ENABLE
IRAM_ATTR void UpdateRotorButton()
{
	btn.update();
}
//Rotary Encoder
IRAM_ATTR void UpdateRotor() {
	noInterrupts();
  portENTER_CRITICAL_ISR(&mux);
	rotor.update();
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&mux);
	interrupts();
}
#endif


void updateStrings(void)
{
//  language = language & 3;
  language=1;
  if (language == 0)
  {
    strcpy(moldInvId, "746");
    strcpy(productArticle, "ДВ-1,0/131  2.1,2.2");
    strcpy(machineNumber, "3.19");
    strcpy(machineName, "Arburg630H-2300-800");
    if (machineStatus == MOLD_OPENED) {
      strcpy(machineStatusText, "Форма Разомкнута");
    }
    if (machineStatus == MOLD_CLOSED) {
      strcpy(machineStatusText, "Форма Сомкнута");
    }
    if (machineStatus == 2) {
      strcpy(machineStatusText, "Прозводство остановлено");
    }
    strcpy(textCycle, "Цикл:");
    strcpy(textSec, "сек.");
    strcpy(textCounter, "Счетчик:");
    strcpy(pcsText, " шт.");

  }

  if (language == 1)
  {
    strcpy(moldInvId, "746");
    strcpy(productArticle, "PAIL-1,0/131mm 2.1,2.2");
    strcpy(machineNumber, "2.5");
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
  #ifdef PREFERENCES_ENABLE
  preferences.begin("COUNTER",false);
  language=preferences.getInt("language");
  language++;
  if (language == 3)
  {
    language = 0;
  }
  preferences.putInt("language",language);
  preferences.end();
  #endif
  Serial.print("Language:");
  Serial.println(language);
  updateStrings();
}

void saveCounter(int64_t cnt)
{
  #ifdef PREFERENCES_ENABLE
  preferences.begin("COUNTER",false);
  preferences.putLong64("counter",cnt);
  preferences.end();
  #endif
}

int64_t readCounter()
{
  int64_t l=0;
  #ifdef PREFERENCES_ENABLE
  preferences.begin("COUNTER",false);
  l=preferences.getLong64("counter");
  Serial.print("Counter:");
  Serial.println((uint32_t)l);
  preferences.end();
  #endif
  return l;
}

void WiFiOff()
{
	#ifdef WIFI_ENABLE
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  esp_wifi_set_mode(WIFI_MODE_NULL);
//  WiFi.forceSleepBegin();
  delay(5);
	#endif
}

void WiFiConnect()
{
	#ifdef WIFI_ENABLE
  unsigned long m;
  m=millis();
//  esp_wifi_init(WIFI_INIT_CONFIG_DEFAULT);
//  esp_wifi_set_mode(ESP_ERR_WIFI_NOT_INIT);
//  WiFi.forceSleepWake();
//  wifi_station_connect();


  WiFi.mode(WIFI_STA);

  if (ip!=0)
  {
    //Serial.println("WiFi set config!");
    //WiFi.config(ip, gateway, subnet);
  }

  int c = 15;
  Serial.println("Connecting to WiFi");
  //WiFi.begin(ssid, password);
  if (usePrimAP) {
    Serial.println(ssidPrim);
    WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
  } else {
    Serial.println(ssidSec);
    WiFi.begin(ssidSec.c_str(), pwSec.c_str());
  }

  #ifdef U8G2_ENABLE
	u8g2.clearBuffer();
	#endif
	strcpy(msg, "Connecting to WiFi ");
  while (WiFi.status() != WL_CONNECTED && c > 0) {
		strcpy(&msg[strlen(msg)], ".");
    Serial.print(".");
    delay(500);
		BlinkBlueLED();
		#ifdef U8G2_ENABLE
//		u8g2.clearBuffer();
		u8g2.setCursor(1, 10);
		u8g2.setDrawColor(1);
		u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
		u8g2.print(msg);
		u8g2.sendBuffer();
		#endif
    yield();
    c--;
  }
  delay(250);
  yield();
  if (c==0)
  {
     Serial.println("WiFi disconnect and try again!");
	   WiFiOff();
	    delay(300);
      if (ip!=0)
      {
        //Serial.println("WiFi set config!");
        //WiFi.config(ip, gateway, subnet);
      }
      if (usePrimAP) {
    		Serial.println(ssidPrim);
    		WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
    	} else {
    		Serial.println(ssidSec);
    		WiFi.begin(ssidSec.c_str(), pwSec.c_str());
    	}
    	c=15;
    	while (WiFi.status() != WL_CONNECTED && c > 0) {
    		strcpy(&msg[strlen(msg)], ".");
        Serial.print(".");
        delay(500);
    		BlinkBlueLED();
    //		u8g2.clearBuffer();
    		#ifdef U8G2_ENABLE
    		u8g2.setCursor(1, 10);
    		u8g2.setDrawColor(1);
    		u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
    		u8g2.print(msg);
    		u8g2.sendBuffer();
    		#endif
        c--;
        yield();
      }
  }


  if (WiFi.status() == WL_CONNECTED )
  {
    Serial.println(".");
    Serial.print("Connected to the WiFi network:");
    Serial.print(ssid);

		strcpy(msg, "Connected to:");
		strcpy(&msg[strlen(msg)], ssid);
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

  	#ifdef U8G2_ENABLE
		u8g2.setCursor(1, 10+8);
		u8g2.print(msg);
		u8g2.sendBuffer();
		#endif

    Serial.printf(" in %i sec\r\n",((int)(millis()-m))/1000);
    //Serial.println((millis()-m)*3);

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
	else
	{
    Serial.println("FAILED TO CONNECT TO WIFI!!");
		strcpy(msg, "Failed to connect to:");
		strcpy(&msg[strlen(msg)], ssid);
		#ifdef U8G2_ENABLE
		u8g2.setCursor(1, 10+8);
		u8g2.print(msg);
		u8g2.sendBuffer();
		delay(1000);
		#endif
	}

  /******************************************JSON TEST************************************************/
  #ifdef HTTPCLIENT_ENABLE
  char json[] = "{\"Name\":\"Blanchon\",\"Skills\":[\"C\",\"C++\",\"C#\"],\"Age\":32,\"Online\":true}";

  JsonParser<32> parser;

  Serial.print("Parse ");
  Serial.println(json);

  JsonHashTable hashTable = parser.parseHashTable(json);

  if (!hashTable.success())
  {
      Serial.println("JsonParser.parseHashTable() failed");
      return;
  }

  char* name = hashTable.getString("Name");
  Serial.print("name=");
  Serial.println(name);

  JsonArray skills = hashTable.getArray("Skills");
  Serial.println("skills:");
  for (int i = 0; i < skills.getLength(); i++)
  {
      char* value = skills.getString(i);
      Serial.print(i);
      Serial.print(" ");
      Serial.println(value);
  }

  int age = hashTable.getLong("Age");
  Serial.print("age=");
  Serial.println(age);

  bool online = hashTable.getBool("Online");
  Serial.print("online=");
  Serial.println(online);
  #endif
  //*****************************
	#endif
}

extern "C" int rom_phy_get_vdd33();

#ifdef SD_ENABLE
void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512*8];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
						BlinkBlueLED();
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
				BlinkBlueLED();
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}
#endif

void readDHT12()
{
#ifdef DHT12_ENABLE
  // READ DATA
Serial.print("DHT12, \t");
int status = DHT.read();
switch (status)
{
case DHT12_OK:
  Serial.print("OK,\t");
  break;
case DHT12_ERROR_CHECKSUM:
  Serial.print("Checksum error,\t");
  break;
case DHT12_ERROR_CONNECT:
  Serial.print("Connect error,\t");
  break;
case DHT12_MISSING_BYTES:
  Serial.print("Missing bytes,\t");
  break;
default:
  Serial.print("Unknown error,\t");
  break;
}
// DISPLAY DATA, sensor has only one decimal.
Serial.print("T:");
Serial.print(DHT.temperature, 1);
Serial.print(",\t H:");
Serial.print(DHT.humidity, 1);
Serial.println("\%");
if (status==DHT12_OK)
{
    temperature=DHT.temperature;
    humidity=DHT.humidity;
}
#endif
}

#ifdef BLE_ENABLE
// Includes for JSON object handling
// Requires ArduinoJson library
// https://arduinojson.org
// https://github.com/bblanchon/ArduinoJson
#include <ArduinoJson.h>
// Includes for BLE
#include <nvs.h>
#include <nvs_flash.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <Preferences.h>

/** Flag if stored AP credentials are available */
bool hasCredentials = false;
/** Connection status */
volatile bool isConnected = false;
/** Connection change status */
bool connStatusChanged = false;

// List of Service and Characteristic UUIDs
#define SERVICE_UUID  "0000aaaa-ead2-11e7-80c1-9a214cf093ae"
#define WIFI_UUID     "00005555-ead2-11e7-80c1-9a214cf093ae"

/** Characteristic for digital output */
BLECharacteristic *pCharacteristicWiFi;
/** BLE Advertiser */
BLEAdvertising* pAdvertising;
/** BLE Service */
BLEService *pService;
/** BLE Server */
BLEServer *pServer;

/** Buffer for JSON string */
// MAx size is 51 bytes for frame:
// {"ssidPrim":"","pwPrim":"","ssidSec":"","pwSec":""}
// + 4 x 32 bytes for 2 SSID's and 2 passwords
StaticJsonBuffer<300> jsonBuffer;

/**
 * Create unique device name from MAC address
 **/
void createName() {
	uint8_t baseMac[6];
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
	// Write unique name into apName
	sprintf(apName, "MOLDMON-%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      Serial.printf("Advertised Device:RSSI:%i %s \n", advertisedDevice.getRSSI(), advertisedDevice.toString().c_str());

    }
};
/**
 * MyServerCallbacks
 * Callbacks for client connection and disconnection
 */
class MyServerCallbacks: public BLEServerCallbacks {
	// TODO this doesn't take into account several clients being connected
	void onConnect(BLEServer* pServer) {
		Serial.println("BLE client connected");
	};

	void onDisconnect(BLEServer* pServer) {
		Serial.println("BLE client disconnected");
		pAdvertising->start();
	}
};

/**
 * MyCallbackHandler
 * Callbacks for BLE client read/write requests
 */
class MyCallbackHandler: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		std::string value = pCharacteristic->getValue();
		if (value.length() == 0) {
			return;
		}
		Serial.println("Received over BLE: " + String((char *)&value[0]));

		// Decode data
		int keyIndex = 0;
		for (int index = 0; index < value.length(); index ++) {
			value[index] = (char) value[index] ^ (char) apName[keyIndex];
			keyIndex++;
			if (keyIndex >= strlen(apName)) keyIndex = 0;
		}

		/** Json object for incoming data */
		JsonObject& jsonIn = jsonBuffer.parseObject((char *)&value[0]);
		if (jsonIn.success()) {
			if (jsonIn.containsKey("ssidPrim") &&
					jsonIn.containsKey("pwPrim") &&
					jsonIn.containsKey("ssidSec") &&
					jsonIn.containsKey("pwSec")) {
				ssidPrim = jsonIn["ssidPrim"].as<String>();
				pwPrim = jsonIn["pwPrim"].as<String>();
				ssidSec = jsonIn["ssidSec"].as<String>();
				pwSec = jsonIn["pwSec"].as<String>();

				Preferences preferences;
				preferences.begin("WiFiCred", false);
				preferences.putString("ssidPrim", ssidPrim);
				preferences.putString("ssidSec", ssidSec);
				preferences.putString("pwPrim", pwPrim);
				preferences.putString("pwSec", pwSec);
				preferences.putBool("valid", true);
				preferences.end();

				Serial.println("Received over bluetooth:");
				Serial.println("primary SSID: "+ssidPrim+" password: "+pwPrim);
				Serial.println("secondary SSID: "+ssidSec+" password: "+pwSec);
				connStatusChanged = true;
				hasCredentials = true;
			} else if (jsonIn.containsKey("erase")) {
				Serial.println("Received erase command");
				Preferences preferences;
				preferences.begin("WiFiCred", false);
				preferences.clear();
				preferences.end();
				connStatusChanged = true;
				hasCredentials = false;
				ssidPrim = "";
				pwPrim = "";
				ssidSec = "";
				pwSec = "";

				int err;
				err=nvs_flash_init();
				Serial.println("nvs_flash_init: " + err);
				err=nvs_flash_erase();
				Serial.println("nvs_flash_erase: " + err);
			} else if (jsonIn.containsKey("reset")) {
				WiFi.disconnect();
				esp_restart();
			}
		} else {
			Serial.println("Received invalid JSON");
		}
		jsonBuffer.clear();
	};

	void onRead(BLECharacteristic *pCharacteristic) {
		Serial.println("BLE onRead request");
		String wifiCredentials;

		/** Json object for outgoing data */
		JsonObject& jsonOut = jsonBuffer.createObject();
		jsonOut["ssidPrim"] = ssidPrim;
		jsonOut["pwPrim"] = pwPrim;
		jsonOut["ssidSec"] = ssidSec;
		jsonOut["pwSec"] = pwSec;
		// Convert JSON object into a string
		jsonOut.printTo(wifiCredentials);

		// encode the data
		int keyIndex = 0;
		Serial.println("Stored settings: " + wifiCredentials);
		for (int index = 0; index < wifiCredentials.length(); index ++) {
			wifiCredentials[index] = (char) wifiCredentials[index] ^ (char) apName[keyIndex];
			keyIndex++;
			if (keyIndex >= strlen(apName)) keyIndex = 0;
		}
		pCharacteristicWiFi->setValue((uint8_t*)&wifiCredentials[0],wifiCredentials.length());
		jsonBuffer.clear();
	}
};

/**
 * initBLE
 * Initialize BLE service and characteristic
 * Start BLE server and service advertising
 */
void initBLE() {
	// Initialize BLE and set output power
	BLEDevice::init(apName);
	BLEDevice::setPower(ESP_PWR_LVL_P7);

	// Create BLE Server
	pServer = BLEDevice::createServer();

	// Set server callbacks
	pServer->setCallbacks(new MyServerCallbacks());

	// Create BLE Service
	pService = pServer->createService(BLEUUID(SERVICE_UUID),20);

	// Create BLE Characteristic for WiFi settings
	pCharacteristicWiFi = pService->createCharacteristic(
		BLEUUID(WIFI_UUID),
		// WIFI_UUID,
		BLECharacteristic::PROPERTY_READ |
		BLECharacteristic::PROPERTY_WRITE
	);
	pCharacteristicWiFi->setCallbacks(new MyCallbackHandler());

	// Start the service
	pService->start();

	// Start advertising
	pAdvertising = pServer->getAdvertising();
	pAdvertising->start();
}

/** Callback for receiving IP address from AP */
void gotIP(system_event_id_t event) {
  Serial.println("Connected to WIFI");
  isConnected = true;
	connStatusChanged = true;
}

/** Callback for connection loss */
void lostCon(system_event_id_t event) {
	isConnected = false;
	connStatusChanged = true;
}

/**
	 scanWiFi
	 Scans for available networks
	 and decides if a switch between
	 allowed networks makes sense

	 @return <code>bool</code>
	        True if at least one allowed network was found
*/
bool scanWiFi() {
	/** RSSI for primary network */
	int8_t rssiPrim;
	/** RSSI for secondary network */
	int8_t rssiSec;
	/** Result of this function */
	bool result = false;

	Serial.println("Start scanning for networks");

	WiFi.disconnect(true);
	WiFi.enableSTA(true);
	WiFi.mode(WIFI_STA);

	// Scan for AP
	int apNum = WiFi.scanNetworks(false,true,false,1000);
	if (apNum == 0) {
		Serial.println("Found no networks?????");
		return false;
	}

	byte foundAP = 0;
	bool foundPrim = false;

	for (int index=0; index<apNum; index++) {
		String ssid = WiFi.SSID(index);
		Serial.println("Found AP: " + ssid + " RSSI: " + WiFi.RSSI(index));
		if (!strcmp((const char*) &ssid[0], (const char*) &ssidPrim[0])) {
			Serial.println("Found primary AP");
			foundAP++;
			foundPrim = true;
			rssiPrim = WiFi.RSSI(index);
      usePrimAP = true;
      result=true;
		}
		if (!strcmp((const char*) &ssid[0], (const char*) &ssidSec[0])) {
			Serial.println("Found secondary AP");
			foundAP++;
			rssiSec = WiFi.RSSI(index);
      usePrimAP = false;
      result=true;
  	}
	}
/*
	switch (foundAP) {
		case 0:
			result = false;
			break;
		case 1:
			if (foundPrim) {
				usePrimAP = true;
			} else {
				usePrimAP = false;
			}
			result = true;
			break;
		default:
			Serial.printf("RSSI Prim: %d Sec: %d\n", rssiPrim, rssiSec);
			if (rssiPrim > rssiSec) {
				usePrimAP = true; // RSSI of primary network is better
			} else {
				usePrimAP = false; // RSSI of secondary network is better
			}
			result = true;
			break;
	}
*/
  return result;
}

/**
 * Start connection to AP
 */
void connectWiFi() {
	// Setup callback function for successful connection
	WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
	// Setup callback function for lost connection
	WiFi.onEvent(lostCon, SYSTEM_EVENT_STA_DISCONNECTED);

	WiFi.disconnect(true);
	WiFi.enableSTA(true);
	WiFi.mode(WIFI_STA);

	Serial.println();
	Serial.print("Start connection to ");
	if (usePrimAP) {
		Serial.println(ssidPrim);
		WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
	} else {
		Serial.println(ssidSec);
		WiFi.begin(ssidSec.c_str(), pwSec.c_str());
	}
//  delay(6000);
}

void BleScanTask(void*){
  // put your main code here, to run repeatedly:
  Serial.println("Starting BLE scan task");
  BLEScan *pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(0x50);
  pBLEScan->setWindow(0x30);
  while (1)
  {
  Serial.println("**************************");
  BLEScanResults foundDevices = pBLEScan->start(30);
  vTaskDelay(1000);
  }
  Serial.println("Ending BLE scan task");
  vTaskDelete( NULL );
}

void ble_setup()
{
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  createName();
  Preferences preferences;
  preferences.begin("WiFiCred", false);
  bool hasPref = preferences.getBool("valid", false);
  if (hasPref) {
    ssidPrim = preferences.getString("ssidPrim","");
    ssidSec = preferences.getString("ssidSec","");
    pwPrim = preferences.getString("pwPrim","");
    pwSec = preferences.getString("pwSec","");

    if (ssidPrim.equals("")
        || pwPrim.equals("")
        || ssidSec.equals("")
        || pwPrim.equals("")) {
      Serial.println("Found preferences but credentials are invalid");
    } else {
      Serial.println("Read from preferences:");
      Serial.println("primary SSID: "+ssidPrim+" password: "+pwPrim);
      Serial.println("secondary SSID: "+ssidSec+" password: "+pwSec);
      hasCredentials = true;
    }
  } else {
    Serial.println("Could not find preferences, need send data over BLE");
  }
  preferences.end();

  // Start BLE server
  initBLE();

    #define SCAN_TIME  3 // seconds
    // put your main code here, to run repeatedly:
    BLEScan *pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(0x50);
    pBLEScan->setWindow(0x30);

  #ifdef SERIAL_PRINT
    Serial.printf("Start BLE scan for %d seconds...\n", SCAN_TIME);
  #endif

    BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME);
    int count = foundDevices.getCount();
//    ss << "[";
    for (int i = 0; i < count; i++)
    {
  //    if (i > 0) {
  //      ss << ",";
  //    }
      BLEAdvertisedDevice d = foundDevices.getDevice(i);
      Serial.print("address:");
      Serial.println(d.getAddress().toString().c_str());
      Serial.print("rssi:");
      Serial.print(d.getRSSI());
      if (d.haveName())
      {
        Serial.print(",name:");
        Serial.println(d.getName().c_str());
      }
      /*
      ss << "{\"Address\":\"" << d.getAddress().toString() << "\",\"Rssi\":" << d.getRSSI();
      if (d.haveName())
      {
        ss << ",\"Name\":\"" << d.getName() << "\"";
      }
      if (d.haveAppearance())
      {
        ss << ",\"Appearance\":" << d.getAppearance();
      }
      if (d.haveManufacturerData())
      {
        std::string md = d.getManufacturerData();
        uint8_t* mdp = (uint8_t*)d.getManufacturerData().data();
        char *pHex = BLEUtils::buildHexData(nullptr, mdp, md.length());
        ss << ",\"ManufacturerData\":\"" << pHex << "\"";
        free(pHex);
      }
      if (d.haveServiceUUID())
      {
        ss << ",\"ServiceUUID\":\"" << d.getServiceUUID().toString() << "\"" ;
      }
      if (d.haveTXPower())
      {
        ss << ",\"TxPower\":" << (int)d.getTXPower();
      }
      ss << "}";
      */
    }
    Serial.println("BLE Scan done!");

  if (hasCredentials) {
    // Check for available AP's
    if (!scanWiFi()) {
      Serial.println("Could not find any AP");
    } else {
      // If AP was found, start connection
      connectWiFi();
    }
  }

  xTaskCreate(
                    BleScanTask,          /* Task function. */
                    "BleScanTask",        /* String with name of task. */
                    5000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
}
#endif

void setup(void) {
  Serial.begin(115200);
//Wire.begin(4,15);
delay(100);

  pinMode(BLUE_LED_PIN, OUTPUT);
//  pinMode(GREEN_LED, OUTPUT);
//  pinMode(RED_LED, OUTPUT);
//  ledcSetup(1, 12000, 8);
//  ledcSetup(2, 12000, 8);
//  ledcSetup(3, 12000, 8);
//  ledcAttachPin(RED_LED, 1);
//  ledcAttachPin(GREEN_LED, 2);
//  ledcAttachPin(BLUE_LED, 3);
Serial.printf("\r\nBoard Started1!\r\n");
Serial.println("Using ESP object:");
Serial.println(ESP.getSdkVersion());
Serial.println("Using lower level function:");
Serial.println(esp_get_idf_version());
// Send some device info
Serial.print("Build: ");
Serial.println(compileDate);

#ifdef BLE_ENABLE
  ble_setup();
#endif
  pinMode(COUNTER_REMOVE_BUTTON_PIN,INPUT_PULLUP);
  pinMode(MOLD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SCL,INPUT_PULLUP);
  pinMode(SDA,INPUT_PULLUP);
//  digitalWrite(BLUE_LED,LOW);
//  digitalWrite(GREEN_LED,LOW);
//  digitalWrite(RED_LED,LOW);
  ledB=LOW;
  digitalWrite(BLUE_LED_PIN,HIGH);

  uint8_t mc[6];
  esp_efuse_mac_get_default(mc);
  Serial.printf("\r\nBoard MAC address: %x:%x:%x:%x:%x:%x\r\n", mc[0],mc[1],mc[2],mc[3],mc[4],mc[5] );
//  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);

/*SD CARD*****************************************/

  // initialize SD library with SPI pins
  //uint8_t csPin = SD_CHIP_SELECT_PIN, int8_t mosi = -1, int8_t miso = -1, int8_t sck = -1);
/*  if (!SD.begin(13, 21, 17, 12)) {
    Serial.println("Failed!");
    return;
  }
  else{Serial.println("OK!");}
*/
  /*SD CARD****************************************/
//  uint8_t Sd2Card::init(uint8_t sckRateID,
// uint8_t chipSelectPin, int8_t mosiPin, int8_t misoPin, int8_t clockPin) {
/*
    SPI.begin(13,21,17,12);
    while (!SD.begin(13, SPI, 24000000, "/sd"))
    {
      Serial.println("initialization failed. Things to check:");
      delay(200);
    };
*/


  //Sd2Card card;
  //SdVolume volume;
  //SdFile root;

//	 void begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
//BLEDevice::init("");
/*
text       data     bss     dec     hex filename
+LORA32 1218006  443552   61992 1723550  1a4c9e .pioenvs/heltec_wifi_lora_32/firmware.elf
text       data     bss     dec     hex filename
NO LORA32 1216310  442520   61424 1720254  1a3fbe .pioenvs/heltec_wifi_lora_32/firmware.elf
text       data     bss     dec     hex filename
552054   142360   35952  730366   b24fe .pioenvs/heltec_wifi_lora_32/firmware.elf
*/
/*
BLEScan* pBLEScan = BLEDevice::getScan(); //create new scan
pBLEScan->setAdvertisedDeviceCallbacks(new MyBLEAdvertisedDeviceCallbacks());
pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
Serial.println("BLE Scan start!");
BLEScanResults foundDevices = pBLEScan->start(BLE_SCAN_TIME);
Serial.print("Devices found: ");
Serial.println(foundDevices.getCount());
Serial.println("Scan done!");
*/
//pinMode(BLUE_LED, OUTPUT);
//digitalWrite(BLUE_LED,HIGH);


#ifdef SD_ENABLE
Serial.println("Initializing SD card...");
cycleStartMillis = millis();
SPIClass spi;
spi.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
//SPI.begin(18,19,23);

//	uint8_t init(uint8_t sckRateID,
//  uint8_t chipSelectPin, int8_t mosiPin = -1, int8_t misoPin = -1, int8_t clockPin = -1);
//  while (!card.init(SPI_HALF_SPEED, 18, 27, 19, 5))

int c=3;

while (!SD.begin(SD_CS,spi,18000000,"/sd"))
//while (!SD.begin(17))
//while (!card.init(SPI_QUARTER_SPEED, 18))
  {
		BlinkBlueLED();
    Serial.println("initialization failed.");
		delay(200);
		c--;
		if (c==0) {break;}
  }
	if (c>0)
	{
	Serial.printf("SD initilized in %i ms.\n", millis()-cycleStartMillis );
	uint8_t cardType = SD.cardType();

	if(cardType == CARD_NONE){
			Serial.println("No SD card attached");
			return;
	}

	Serial.print("SD Card Type: ");
	if(cardType == CARD_MMC){
			Serial.println("MMC");
	} else if(cardType == CARD_SD){
			Serial.println("SDSC");
	} else if(cardType == CARD_SDHC){
			Serial.println("SDHC");
	} else {
			Serial.println("UNKNOWN");
	}

	uint64_t cardSize = SD.cardSize() / (1024 * 1024);
	Serial.printf("SD Card Size: %lluMB\n", cardSize);
	testFileIO(SD, "/test.txt");
	}
	digitalWrite(2,LOW);
	delay(200);

	//listDir(SD, "/", 0);

/*
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
	*/
/*

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
spi.end();
SPI.end();

#endif

#ifdef ROTARY_ENABLE
btn.initialize(rotorButtonPin);
rotor.initialize(rotorPinA, rotorPinB);
pinMode(rotorPinA, INPUT_PULLUP);
pinMode(rotorPinB, INPUT_PULLUP);

rotor.setMinMax(0, 255);
rotor.setPosition(5);
attachInterrupt(digitalPinToInterrupt(rotorPinA), UpdateRotor, CHANGE);
attachInterrupt(digitalPinToInterrupt(rotorPinB), UpdateRotor, CHANGE);
#endif

#ifdef LORA32_ENABLE
SPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI, LORA_CS);
LoRa.setPins(LORA_CS, LORA_RST, LORA_DI0);
#endif
#ifndef LORA32_ENABLE
pinMode(LORA_RST, OUTPUT);
digitalWrite(LORA_RST,LOW);
#endif
  /*
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high
  */
  //  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_2M);
  //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
  //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_240M);
	#ifdef U8G2_ENABLE
	u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setFlipMode(1);
  u8g2.setContrast(1);
  u8g2.enableUTF8Print();   // enable UTF8 support for the Arduino print() function
	#endif
  cycleStartMillis = millis();
  machineStatus = 0;

  moldCycleCounter = readCounter();
	#ifdef WIFI_ENABLE

  WiFiConnect();
  Serial.println(WiFi.status());
	if (WiFi.status() == WL_CONNECTED)
	{
  	getTime();
	}

  #ifdef MODBUS_ENABLE
  mb.begin();
  mb.addIsts(100);
  mb.addHreg(100,moldCycleCounter);
  mb.addHreg(102,0);
  mb.addHreg(103,0);
  mb.addHreg(110,0);
  mb.addHreg(111,0);
  mb.addHreg(112,0);
  mb.addHreg(113,0);
  mb.addHreg(114,0);
  mb.addHreg(115,0);
  mb.addHreg(116,0);
  mb.addHreg(117,0);
  mb.addHreg(118,0);
  mb.addHreg(119,0);
  mb.addHreg(120,0);
  mb.addHreg(121,0);
  mb.addHreg(122,0);
  mb.addHreg(123,0);
  mb.addHreg(124,0);
  mb.addHreg(125,0);
  #endif
	#endif

  float vdd = rom_phy_get_vdd33() / 1000.0;
  Serial.print("VDD:");
  Serial.println(vdd);

	#ifdef LORA32_ENABLE
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
	#endif
//  #ifndef LORA_GATEWAY
//    WiFiOff();
//  #endif

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
	#ifdef LORA32_ENABLE
  Serial.print("Sending LORA packet: ");
	BlinkBlueLED();
  Serial.println(msg);
//  digitalWrite(2, HIGH);
  // send packet
  LoRa.beginPacket();
  LoRa.print(msg);
  //  LoRa.print(counter);
  LoRa.endPacket();
	BlinkBlueLED();
//  digitalWrite(2, LOW);
//  LoRa.end();
	#endif
}

static unsigned short days[4][12] =
{
  {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
  { 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
  { 731, 762, 790, 821, 851, 882, 912, 943, 974, 1004, 1035, 1065},
  {1096, 1127, 1155, 1186, 1216, 1247, 1277, 1308, 1339, 1369, 1400, 1430},
};
#ifdef HTTPCLIENT_ENABLE
HTTPClient http;
#else
#ifdef LORA32_ENABLE
char URL[512];
#endif
#endif

int displayOn;

String* messages[128];
int messagesCnt=0;

void sendClamp(long cycleTime, long counterValue,const char* eventType,unsigned long mouldOpenedTime)
{
	BlinkBlueLED(255);
  unsigned long epoch = (millis() / 1000) + unixStartTime;
  cycleTime *= 3;
  String et=String(eventType);
  String msg;
  msg=String(String(counterId)+String(";")+
                 String(epoch) + String(";")+
                 String(cycleTime) + String(";") +
                 String(counterValue) +String(";")+
                 String(eventType)+String(";")+
                 String(mouldOpenedTime)+String(";")+
                 String(machineId)+String(";")+
                 String(mouldId)+String(";."));
  //Serial.println(msg);
  /**PROBLEM HERE*/
  String smsg=String(msg);
//  String* sm=&smsg;
//  messages[messagesCnt]=sm;
  messagesCnt++;
  if (messagesCnt==5 || et=="PAUSE_PRODUCTION_START" || et=="PAUSE_PRODUCTION_END")
  {
		#ifdef LORA32_ENABLE
    unsigned long m=millis();
    if (!LoRa.begin(LORA_BAND)) {
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
    Serial.println((millis()-m));

    LoRa.end();
		#endif

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
	BlinkBlueLED(0);
}
//7 pixel - u8g2_font_haxrcorp4089_t_cyrillic
//10 pixel - u8g2_font_unifont_t_cyrillic
//12 pixel - u8g2_font_cu12_t_cyrillic
//20 pixel - u8g2_font_10x20_t_cyrillic
unsigned long moldOpenTime;

int displayStatus=0;

#ifdef LORA32_ENABLE
char loraPacket[512];
#endif

int prevDisplayButtonStatus=1;
int isDisplayOn=1;
int prevCycleTime;


void modbusHregSetStringChar16(uint16_t hregNum,wchar_t* str,uint8_t maxlen)
{
  #ifdef MODBUS_ENABLE
  wchar_t *p=str;
  uint16_t i=hregNum;
  uint8_t cnt=0;
  while (1)
  {
    mb.Hreg(i,(uint16_t)p[0]);
    if (p[0]==0) break;
    cnt++;
    i++;
    p++;
    if (cnt==maxlen) break;
  }
  #endif
}

void modbusHregSetStringChar8(uint16_t hregNum,char* str,uint8_t maxlen)
{
  #ifdef MODBUS_ENABLE
  unsigned char *p=(unsigned char*)str;
  uint16_t i=hregNum;
  uint8_t cnt=0;
  while (p[0]!=0)
  {
    mb.Hreg(i, uint16_t(p[1]|(p[0]<<8)));
    if (p[0]==0) break;
    p++;
    if (p[0]==0) break;
    p++;
    cnt+=2;
    if (cnt>=maxlen) break;
    i++;
  }
  #endif
}

void loop() {
//    Serial.println(WiFi.status());
//    BlinkBlueLED();
/*
    if ( millis() % 10000 < 5000 )
        showGradient();
    else
        showRgb();
*/

/*
#ifdef HTTPCLIENT_ENABLE
http.begin("http://82.209.207.138:88/alians_cms1.php?cmd=dumpTable&what=now()&table=ap_crm_machines&where=&orderBy=id&noheaders=1&limit=1");  //Specify destination for HTTP request
int httpResponseCode = http.GET();   //Send the actual POST request
//Serial.println(URL);
if (httpResponseCode > 0) {
  String response = http.getString();                       //Get the response to the request
  Serial.print("HTTP:");
  Serial.print(httpResponseCode);   //Print return code
  Serial.print("-");
  Serial.println(response);           //Print request answer
}
#endif
*/
#ifdef BLE_ENABLE
if (connStatusChanged) {
  if (isConnected) {
    Serial.print("Connected to AP: ");
    Serial.print(WiFi.SSID());
    Serial.print(" with IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" RSSI: ");
    Serial.println(WiFi.RSSI());

    Serial.print("FREE stack: ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
    Serial.print("FREE heap: ");
    Serial.println(esp_get_free_heap_size());
  } else {
    if (hasCredentials) {
      Serial.println("Lost WiFi connection");
      // Received WiFi credentials
      if (!scanWiFi()) { // Check for available AP's
        Serial.println("Could not find any AP");
      } else { // If AP was found, start connection
        connectWiFi();
      }
    }
  }
  connStatusChanged = false;
}
#endif

        if ( millis() % 2000<50 )
          readDHT12();
    delay(75);
    #ifdef MODBUS_ENABLE
    mb.task();
    mb.Ists(100, digitalRead(MOLD_BUTTON_PIN));
    mb.Hreg(100, (uint16_t)moldCycleCounter);
    mb.Hreg(102,(float)((float)(prevCycleTime)/1000.0));
//    modbusHregSetStringChar16(110,L"АЛЬЯНС",30);
    modbusHregSetStringChar8(110,(char*)"АЛЬЯНС",30);
    #endif

    #ifdef SONAR_HC04_ENABLE
    long duration;
    float cm;
    pinMode(TRIG_PIN, OUTPUT);
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);
    delayMicroseconds(100);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    duration = pulseIn(ECHO_PIN, HIGH);
    // convert the time into a distance
    cm =duration*0.034/2;
    Serial.print(duration);
    Serial.print("....");
    Serial.println(cm);
    #endif

	//rotor.update();
  #ifdef ROTARY_ENABLE
  UpdateRotor();
	btn.update();
  if (btn.isPressed()) {
  //  led_brightness++;
  //  led_brightness&=15;
//    BlinkBlueLED();
		Serial.println("Rotary Button Pressed.");
    long pos1 = rotor.getPosition();
    Serial.print("Rotor position:");
    Serial.println(pos1);

//    Serial.print("led_brightness=");
//    Serial.println((led_brightness*16)+15);
  }
  //led_brightness2++;//led_brightness2&=15;
  if(interruptCounter>0){
      portENTER_CRITICAL(&mux);
      interruptCounter--;
      long pos = rotor.getPosition();
      portEXIT_CRITICAL(&mux);
      if (lastRotor != pos) {
        float tps = rotor.tps.getTPS();
        Serial.print("Rotor: ");
        Serial.print(pos);
        Serial.print(" ");
        Serial.println(tps);
        led_color=(uint8_t)pos;
      }
      lastRotor = pos;

//      numberOfInterrupts++;
//      Serial.print("An interrupt has occurred. Total: ");
  }
  #endif

  displayStatus = 1;

	#ifdef U8G2_ENABLE
	u8g2_uint_t w;
	#endif
  frame++; frame = frame & ((65536 * 2) - 1);
  int buttonStatus = digitalRead(MOLD_BUTTON_PIN);
  int removeButtonStatus = digitalRead(COUNTER_REMOVE_BUTTON_PIN);

  //buttonStatus=1 MOLD OPENED
  //buttonStatus=0 MOLD CLOSED
  long currentCycleTime = (millis() - cycleStartMillis) ;
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
      prevCycleTime=(m1 - cycleStartMillis);
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

/*
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
			#ifdef U8G2_ENABLE
      u8g2.setPowerSave(1);
			#endif
    }
    else
    {
			#ifdef U8G2_ENABLE
      u8g2.setPowerSave(0);
			#endif
    }
  }
*/
  if (removeButtonStatus==0)
  {
  if (machineStatus==PRODUCTION_STOPPED)
  {
    BlinkRedLED();
  }
  if (machineStatus==MOLD_CLOSED)
  {
    BlinkGreenLED();
  }
  if (machineStatus==MOLD_OPENED)
  {
    BlinkYellowLED();
  }
  }

  if (isDisplayOn == 0)
  {
		//DISPLAY IS OFF
    //delay(90);
    return;
  }

  updateStrings();
  char machineFullName[100];
	#ifdef U8G2_ENABLE
  u8g2.setFontDirection(0);

  u8g2.clearBuffer();
  //  if (frame&65535>32768) {u8g2.setDrawColor(0);} else {u8g2.setDrawColor(1);}
  u8g2.setDrawColor(1);

/*
  u8g2.setDrawColor(1);
  u8g2.setCursor(28, 9);
  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  u8g2.print(productArticle);
*/
  u8g2.setCursor(0, 12);
  u8g2.setFont(u8g2_font_unifont_t_cyrillic);
//  sprintf(scrollingString, "                           %s                        ","#2092 ДВ-1.0/131 №1,2 #01[ОС],#92[РУЧ.БЕЛ] в КОМПЛ. С #23[ЗЕЛ.] крышкой,IML:#923 \"Майонез сладкий 67\%\" в Гофроящик 600х400х520 по 300 ШТ.");
  sprintf(scrollingString, "                           %s                        ","#2092 PAIL-1.0/131 №2.1,2.2 #01[TRANSP.],#92[WHITE HANDLE] in a SET #23[GREEN] LID,IML:#923 \"SWEET SAUCE 67\%\" CARTOON 600х400х520mm by 300 PCS.");
//  u8g2.print(&scrollingString[scrollingPos]);
  char str1[64];
  strncpy(str1,&scrollingString[scrollingPos],24);
//  str1[17]=0;
  u8g2.drawUTF8(0,12,str1);
  if (millis()%100<=100) scrollingPos++;
  if (scrollingPos>(strlen(scrollingString)-25)) scrollingPos=0;

  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 20);
  //u8g2.setFont(u8g2_font_5x7_tf);
	u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  //sprintf(machineFullName, "M%s", machineNumber);
  sprintf(machineFullName, "%s-%s", machineNumber, machineName);
  u8g2.drawUTF8(0,20,machineFullName);

  u8g2.drawRBox(128-25, 0, 25, 12, 2);
  //  if (frame&65535>32768) {u8g2.setDrawColor(1);} else {u8g2.setDrawColor(0);}
  u8g2.setCursor(128-24, 9);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  u8g2.print("#");
  u8g2.setCursor(128-24+6, 10);
//  u8g2.setFont(u8g2_font_crox1tb_tf);
	u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  u8g2.print(moldInvId);

  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  if (((millis()%500)<250) && machineStatus == 2) {
    u8g2.setDrawColor(0);
    u8g2.drawRBox(0, 22, 128, 15, 4);
    u8g2.setDrawColor(1);
    u8g2.drawRFrame(0, 22, 128, 15, 4);
  } else {
    u8g2.setDrawColor(1);
    u8g2.drawRBox(0, 22, 128, 15, 4);
  }

  u8g2.setDrawColor(0);
  if (((millis()%500)<250) && machineStatus == 2) {
    u8g2.setDrawColor(1);
  } else {
    u8g2.setDrawColor(0);
  }

  if (language == 0) {
		u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
//    u8g2.setFont(u8g2_font_unifont_t_cyrillic);
  } else {
    u8g2.setFont(u8g2_font_timB10_tf);
  }
  if (language == 2)
  {
//    u8g2.setFont(u8g2_font_timB10_tr);
  }
  w = u8g2.getUTF8Width(machineStatusText);
  u8g2.setCursor((128 - w) / 2, 20 + 13);
  u8g2.print(machineStatusText);

  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 32 + 8 + 4 + 6 + 2);
//  u8g2.setFont(u8g2_font_10x20_t_cyrillic);
//	u8g2.setFont(u8g2_font_helvB12_te);
  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  if (language > 0) {
    u8g2.setFont(u8g2_font_helvB12_te);
  }
  u8g2.print(textCycle);

  int sec = currentCycleTime / 1000;
  int dsec = (currentCycleTime - sec * 1000) / 10;
//  u8g2.setFont(u8g2_font_timB14_tn);
	u8g2.setFont(u8g2_font_helvB12_te);
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
//    u8g2.setFont(u8g2_font_lucasfont_alternate_tr);
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
  unsigned long epoch = (millis() / 1000) + unixStartTime;
  epoch += 3600*3; //GMT+2

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
//  unsigned int day   = epoch - days[year][month] + 1;

  u8g2.setCursor(88, 63);
  //  u8g2.print("");
  if ((millis()%10000)<3000)
  {
    u8g2.setCursor(88-7, 63);
    u8g2.setFont(u8g2_font_ncenB08_tn);
    u8g2.print(temperature);
    u8g2.setFont(u8g2_font_7x13_t_symbols);
    u8g2.print("℃");
    u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
    u8g2.print("[");
    u8g2.setFont(u8g2_font_ncenB08_tn);
    u8g2.print(humidity);
    u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
    u8g2.print("%");
    u8g2.print("]");
  }
  else
  {
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

  }

  if (removeButtonStatus==1)
  {
    BlinkPinkLED();
//    u8g2.clearBuffer();
    int c1=(millis()%400)<200?1:0;
//    int c2=(millis()%400)<200?0:1;
    u8g2.setDrawColor(c1);
    if (c1==0)
    {
      u8g2.setDrawColor(1);
      u8g2.drawRBox(0,10,128,50,5);
      u8g2.setDrawColor(0);
    }
    else
    {
      u8g2.setDrawColor(0);
      u8g2.drawBox(0,10,128,50);
      u8g2.setDrawColor(1);
      u8g2.drawRFrame(0,10,128,50,5);
    }
    u8g2.setFont(u8g2_font_9x15_t_cyrillic );
    u8g2.setCursor(36, 25);
    u8g2.print("Counter");
    u8g2.setCursor(36, 40);
    u8g2.print("removed");
    u8g2.setCursor(24, 55);
    u8g2.print("from mold");
//    u8g2.drawDisc(128-7, 64-18, 4, U8G2_DRAW_ALL);
//    u8g2.drawCircle(128-7, 64-18, 6, (millis()%400)<200?U8G2_DRAW_ALL:0);
  }
  u8g2.sendBuffer();
	#endif
  //  esp_deep_sleep(1000000/10);
  //delay(30);
}

void getTime()
{
	#ifdef WIFI_ENABLE
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
	strcpy(msg, "Getting time ");
  while ((c--) > 0)
  {
      BlinkGreenLED();
			strcpy(&msg[strlen(msg)], ".");
			#ifdef U8G2_ENABLE
			u8g2.setCursor(1, 10+8+8);
			u8g2.setDrawColor(1);
			u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
			u8g2.print(msg);
			u8g2.sendBuffer();
			#endif

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
	#endif
}
// send an NTP request to the time server at the given address
#ifdef WIFI_ENABLE
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
  return 0;
}
#endif

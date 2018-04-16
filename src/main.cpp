#define U8G2_ENABLE
//#define SD_ENABLE
//#define LORA32_ENABLE
//#define WIFI_ENABLE
//#define HTTPCLIENT_ENABLE
#define BLE_ENABLE
//#define ROTARY_ENABLE
//#define PREFERENCES_ENABLE

//#define WIFI_GATEWAY
//#define LORA_GATEWAY

#include <Arduino.h>

#ifdef ROTARY_ENABLE
#include <Button.h>
#include <TicksPerSecond.h>
#include <RotaryEncoderAcelleration.h>
#endif

#ifdef U8G2_ENABLE
#include <U8g2lib.h>
#endif

#include "esp_system.h"
#include "esp_wifi.h"
#include "soc/rtc.h"
#include "rom/uart.h"

#ifdef WIFI_ENABLE
#include <WiFi.h>
#endif

#ifdef HTTPCLIENT_ENABLE
#include <HTTPClient.h>
#endif
//#include <esp_now.h>

#ifdef PREFERENCES_ENABLE
#include <Preferences.h>
Preferences preferences;
#endif

static const int buttonPin = 21;	// the number of the pushbutton pin

static const int rotorPinA = 13;	// One quadrature pin
static const int rotorPinB = 12;	// the other quadrature pin

#define WIFI_CLAMP_ON

#define MOLD_BUTTON_PIN 17
#define BLUE_LED 2
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

#ifdef BLE_ENABLE
#define GATTC_TAG "GATTC_DEMO"
#define PROFILE_A_APP_ID 0

#include <nvs_flash.h>
#include <esp_bt.h>            // ESP32 BLE
//#include <esp_bt_device.h>     // ESP32 BLE
#include <esp_bt_main.h>       // ESP32 BLE
#include <esp_gap_ble_api.h>   // ESP32 BLE
//#include <esp_gatts_api.h>     // ESP32 BLE
#include <esp_gattc_api.h>     // ESP32 BLE
//#include <esp_gatt_common_api.h>// ESP32 BLE
#endif

const char* ssid = "ALIANSPLAST1";
const char* password =  "300451566";


#ifdef BLE_ENABLE
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static bool connect    = false;
static bool get_server = false;

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};


/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       // Not get the gatt_if, so initial is ESP_GATT_IF_NONE
    },
};


static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
    esp_err_t scan_ret;
    esp_err_t mtu_ret;
    esp_gatt_srvc_id_t *srvc_id;
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGE(GATTC_TAG, "ESP_GATTC_REG_EVT");
        scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
          ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGE(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        break;
    case ESP_GATTC_OPEN_EVT:
        ESP_LOGE(GATTC_TAG, "ESP_GATTC_OPEN_EVT");
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGE(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT");

        ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
    ESP_LOGE(GATTC_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
    ESP_LOGE(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    ESP_LOGE(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        break;

    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGE(GATTC_TAG, "ESP_GATTC_WRITE_DESCR_EVT");
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
      ESP_LOGE(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT");
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
    ESP_LOGE(GATTC_TAG, "ESP_GATTC_WRITE_CHAR_EVT");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGE(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;

}
}


static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
      ESP_LOGE(GATTC_TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        ESP_LOGE(GATTC_TAG, "ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        ESP_LOGE(GATTC_TAG, "ESP_GAP_BLE_SCAN_RESULT_EVT");

        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGE(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGE(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
            ESP_LOG_BUFFER_CHAR_LEVEL(GATTC_TAG, adv_name, adv_name_len,ESP_LOG_ERROR);
            ESP_LOGE(GATTC_TAG, "\n");
//            if (adv_name != NULL) {
//                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
//                    ESP_LOGI(GATTC_TAG, "searched device %s\n", remote_device_name);
//                    if (connect == false) {
//                        connect = true;
//                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
//                        esp_ble_gap_stop_scanning();
//                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
//                    }
//                }
//            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGE(GATTC_TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT");
            esp_ble_gap_start_scanning(60);

            break;
        default:
        ESP_LOGE(GATTC_TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT DEF_EVT:%d",scan_result->scan_rst.search_evt);
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
    ESP_LOGE(GATTC_TAG, "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT status = %x", param->scan_stop_cmpl.status);
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            break;
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGE(GATTC_TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT status = %x", param->adv_stop_cmpl.status);
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGE(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}
#endif


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

#ifdef U8G2
#include "SPI.h"
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
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
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 15, /* data=*/ 4);   // ESP32 Thing, HW I2C with pin remapping
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
void BlinkBlueLED(unsigned int val=-1)
{
	if (val==-1)
	{
	ledB=(ledB==LOW?HIGH:LOW);
	}
	else
	{
		ledB=val;
	}
  digitalWrite(BLUE_LED,ledB);
}
void getTime();
#ifdef WIFI_ENABLE
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
	rotor.update();
	interrupts();
}
#endif


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
  WiFi.mode(WIFI_STA);
//  wifi_station_connect();

  if (ip!=0)
  {
    Serial.println("WiFi set config!");
    WiFi.config(ip, gateway, subnet);
  }

  int c = 10;
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
	#ifdef U8G2_ENABLE
	u8g2.clearBuffer();
	#endif

	strcpy(msg, "Connecting to WiFi ");
  while (WiFi.status() != WL_CONNECTED && c > 0) {
		strcpy(&msg[strlen(msg)], ".");
    Serial.print(".");
    delay(300);
		BlinkBlueLED();
		#ifdef U8G2_ENABLE
//		u8g2.clearBuffer();
		u8g2.setCursor(1, 10);
		u8g2.setDrawColor(1);
		u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
		u8g2.print(msg);
		u8g2.sendBuffer();
		#endif

    c--;
  }
	WiFiOff();
	delay(100);
	WiFi.begin(ssid, password);
	c=10;
	while (WiFi.status() != WL_CONNECTED && c > 0) {
		strcpy(&msg[strlen(msg)], ".");
    Serial.print(".");
    delay(250);
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
  }

  if (c > 0)
  {
    Serial.println(".");
    Serial.print("Connected to the WiFi network:");
    Serial.println(ssid);

		strcpy(msg, "Connected to:");
		strcpy(&msg[strlen(msg)], ssid);
		#ifdef U8G2_ENABLE
		u8g2.setCursor(1, 10+8);
		u8g2.print(msg);
		u8g2.sendBuffer();
		#endif

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
	else
	{
		strcpy(msg, "Failed to connect to:");
		strcpy(&msg[strlen(msg)], ssid);
		#ifdef U8G2_ENABLE
		u8g2.setCursor(1, 10+8);
		u8g2.print(msg);
		u8g2.sendBuffer();
		delay(1000);
		#endif
	}
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

void setup(void) {

  delay(50);
  Serial.begin(115200);
  delay(200);


#ifdef BLE_ENABLE
esp_log_level_set(GATTC_TAG, ESP_LOG_VERBOSE);
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    nvs_flash_erase();
    ret = nvs_flash_init();
}

ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
ret = esp_bt_controller_init(&bt_cfg);
if (ret) {
    ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
}
ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
if (ret) {
    ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
}
ret = esp_bluedroid_init();
if (ret) {
    ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
}

ret = esp_bluedroid_enable();
if (ret) {
    ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
}
//register the  callback function to the gap module
ret = esp_ble_gap_register_callback(esp_gap_cb);
if (ret){
    ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
    return;
}
//register the callback function to the gattc module
ret = esp_ble_gattc_register_callback(esp_gattc_cb);
if(ret){
    ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
    return;
}

ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
if (ret){
    ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
}
/*
esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
 if (local_mtu_ret) {
     ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
 }
 */
 ESP_LOGE(GATTC_TAG, "BLE Init finished.");
#endif

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
//    rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
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
pinMode(BLUE_LED, OUTPUT);
digitalWrite(BLUE_LED,HIGH);
ledB=HIGH;

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
btn.initialize(buttonPin);
rotor.initialize(rotorPinA, rotorPinB);
rotor.setMinMax(100, 999);
rotor.setPosition(500);
attachInterrupt(digitalPinToInterrupt(rotorPinA), UpdateRotor, CHANGE);
attachInterrupt(digitalPinToInterrupt(rotorPinB), UpdateRotor, CHANGE);
#endif

#ifdef LORA32_ENABLE
SPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI, LORA_CS);
LoRa.setPins(LORA_CS, LORA_RST, LORA_DI0);
#endif

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
	if (WiFi.status() == WL_CONNECTED)
	{
  	getTime();
	}
  btStop();
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
char URL[512];
HTTPClient http;
#endif

int displayOn;

String* messages[128];
int messagesCnt=0;

void sendClamp(long cycleTime, long counterValue,char* eventType,unsigned long mouldOpenedTime)
{
	BlinkBlueLED();
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

		#ifdef LORA32_ENABLE
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
    Serial.println((millis()-m)*3);

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
	BlinkBlueLED();
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

void loop(void) {
	//rotor.update();
  #ifdef ROTARY_ENABLE
  long pos = rotor.getPosition();
	btn.update();
  if (btn.isPressed()) {
		Serial.println("Rotary Button Pressed!");
  }

  if (lastRotor != pos) {
    float tps = rotor.tps.getTPS();
    Serial.print("Rotor: ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.println(tps);
  }
  lastRotor = pos;
  #endif

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

			 #ifdef WIFI_ENABLE
			 if (ii==7 && WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
		#ifdef HTTPCLIENT_ENABLE
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
		#endif
    }
		#endif

       Serial.print(str1);
       Serial.print(";");
       ii++;
      }
    }
		#ifdef U8G2_ENABLE
      u8g2.setPowerSave(0);
      u8g2.setFontDirection(0);
      u8g2.clearBuffer();
      //  if (frame&65535>32768) {u8g2.setDrawColor(0);} else {u8g2.setDrawColor(1);}
      u8g2.setDrawColor(1);

      u8g2.setCursor(1, 9);
      u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
      String s1=String("");
      #ifdef LORA32_ENABLE
      s1=String("RSSI:")+String(LoRa.packetRssi());
      #endif
      String s2=String("P.SIZE:")+String(packetSize);
      u8g2.print(s1);
      u8g2.setCursor(1, 16);
      u8g2.print(s2);
      u8g2.sendBuffer();
	  #endif

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
	#ifdef U8G2_ENABLE
	u8g2_uint_t w;
	#endif
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
  u8g2.drawRBox(0, 0, 25, 12, 2);
  //  if (frame&65535>32768) {u8g2.setDrawColor(1);} else {u8g2.setDrawColor(0);}
  u8g2.setCursor(1, 9);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  u8g2.print("#");
  u8g2.setCursor(6, 10);
//  u8g2.setFont(u8g2_font_crox1tb_tf);
	u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  u8g2.print(moldInvId);

  u8g2.setDrawColor(1);
  u8g2.setCursor(28, 9);
  u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
  u8g2.print(productArticle);

  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 20);
  //u8g2.setFont(u8g2_font_5x7_tf);
	u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);

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
}
#endif

#else
#include <WiFi.h>

#include <esp_bt.h>            // ESP32 BLE
#include <esp_bt_device.h>     // ESP32 BLE
#include <esp_bt_main.h>       // ESP32 BLE
#include <esp_gap_ble_api.h>   // ESP32 BLE
#include <esp_gatts_api.h>     // ESP32 BLE
#include <esp_gattc_api.h>     // ESP32 BLE
#include <esp_gatt_common_api.h>// ESP32 BLE

void setup(void) {
  #ifdef BLE_ENABLE
  esp_log_level_set(GATTC_TAG, ESP_LOG_VERBOSE);
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      nvs_flash_erase();
      ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
      ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
      ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  ret = esp_bluedroid_init();
  if (ret) {
      ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
      ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  //register the  callback function to the gap module
  ret = esp_ble_gap_register_callback(esp_gap_cb);
  if (ret){
      ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
      return;
  }
  //register the callback function to the gattc module
  ret = esp_ble_gattc_register_callback(esp_gattc_cb);
  if(ret){
      ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
      return;
  }

  ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
  if (ret){
      ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
  }
  /*
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
   if (local_mtu_ret) {
       ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
   }
   */
   ESP_LOGE(GATTC_TAG, "BLE Init finished.");
  #endif

  //BLEDevice::init("");

  delay(50);
  Serial.begin(115200);
  delay(0);
//	WiFi.mode(WIFI_STA);
}
void loop(void) {
}
#endif

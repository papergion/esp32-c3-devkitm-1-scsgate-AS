//---------------------------------------------------------------------------------------------------------
#define _FW_NAME     "SCSGATE_32"
#define _FW_VERSION  "VER_9.002 "
#define _ESP_CORE    "esp32-2.5.2"
// ========================================================================================================

/*
da sviluppare:
- gestione dispositivi generici, allarme, termostato, ... in sse HA

test da rifare:
- scrittura messaggi scs -> ok
- aggiornamento sseha    -> luci ok, dimmer ok, termo ok, tappa KO
- scrittura mqtt         -> OK
- lettura mqtt           -> OK
- censimento automatico dispositivi (cmd auto)  - mettere 0 in valore_max -> ok
- censimento automatico IN MQTT -> ok
- read/write da vb6   -> ok
- aggiornamento dispositivi (tap) nel PIC -> ok
- gestione tapparelle a percentuale -> ok
- sistemare /status -> ok
- test funzioni /gate:ok  /request: -> ok
- gestione fauxmo alexa : -> ok
- gestione dispositivi generici
- layout /sselog    -> ok
- layout vari       -> ok



- manuale nuovo -> ok

*/

// ========================================================================================================
// per ESP32-C3 
//              tools/partition-scheme/Minimal SPIFFS  <------------------------x define FFS --------------
//              tools/partition-scheme/No FS 4MB (2MB app x 2) 
//              USB CDC on boot: ENABLED solo se si usa il debug con USB (DEVE essere collegato USB)
// ========================================================================================================

// ========================================================================================================
// Serial  = USB
// Serial1 = UART
// ========================================================================================================
// installed library: 
//     AsyncTCP             1.1.4
//     Effortless-SPIFFS    2.3.0
// ========================================================================================================

#define JMP 7
#define BLINKLED     

const int pinLed = 8;

#define SCS

///#define DEBUG_EVENTITY
///#define TRACE_SSE



//#define DEBUG
//#define VERBOSE

#define MY_DOMOTIC
#define MY_TABELLA
#define TERMO_NEW

#define FFS
#define PIC_REQ 0xA7
//----------------------------------------------------------------------------------
//        ---- attenzione - porta http: 8080 <--se alexaParam=y--------------

// SCS WARNING - LA PUBBLICAZIONE AVVIENE SOLO SE PRIMA E' PERVENUTO UN /cmd qualunque da mqtt !!!!!!!!!!!!!!!
//        finchè non invia un comando che inizia con: PIC_REQ  

//----------------------------------------------------------------------------------
// 
//
// devices type:
// 0x01:1-switch
// 0x03:3-dimmer scs
// 0x04:4-dimmer on/off	        0x15:21-dimmer up/dw		   
// 0x08:8-tapparella stop       0x09:9-tapparella pct stop   
// 0x12:18-tapparella up/dwn    0x13:19-tapparella pct up/dwn (knx)
// 0x0B:11-generic              0x0C:12-generic 
// 0x0E:14-:allarme             0x0F:15-termostato
// 0x30-0x37:rele i2c
// 0x40-0x47:pulsanti i2c


//----------------------------------------------------------------------------------
//

#define UART_W_BUFFER

#define NO_ALEXA_MQTT
//#define NO_ALEXA_UDP
#define USE_TCPSERVER
#define TCP_PORT 5045
// verificare DEBUG_FAUXMO_TCP in fauxmoesp.h

#define USE_OTA  // ==>ATTENZIONE<==  eventuali problemi OTA possono essere risolti disattivando il FIREWALL di rete (windows)

#define BUFNR 2 // nr di buffer tx seriali
/*
  scsknxgate - gateway between KNXgate/SCSgate and ethernet application   UDP / TCP / HTTP / MQTT

  V 4.3 - use DIMMER on KNX
  V 4.2 - use TCP for download and update device tab
  V 4.1 - use OTA for wifi firmware update
  V 4.0 - use fauxmo for direct connection with echo dot devices
  V 3.7 - homeassistant mode parametrico - scrittura UART diretta senza TX queue
  V 3.6 - gestione log mqtt
  V 3.3 - gestione tapparelle percentuale - SOLO SCS
  V 3.2 - MQTT scs & knx
  V 2.0 - risponde anche a comandi HTTP, tipo:
                      http://192.168.2.230/gate.htm?type=12&from=01&to=31&cmd=01&resp=y

          la porta di ricezione e' quella prevista in configurazione
          la pagina di comando si chiama gate.htm
          i parametri possibili sono  type=   12 (comando, byte 4 del telegramma) default 12
                                      from=   NN  mio indirizzo hex - default 01 hex
                                      to=     NN  indirizzo hex del device destinatario
                                      cmd=    NN  hex comando da mandare
                                      resp=   y     "y" request to send a response, otherwise no response

          e' possibile utilizzare una finestra di aiuto con
                      http://192.168.2.230/request

          se si usa l'opzione resp=y attiva il log breve e per ogni telegramma di ritorno (l=4) effettua una chiamata al client:
          http://<ipaddress>:8080/json.htm?type=command&param=udevices&script=scsgate_json.lua&type=<byte[3]>&from=<byte[2]>&to=<byte[1]>&cmd=<byte[4]>
          è parametrizzabile la parte:
                            :8080/json.htm?type=command&param=udevices&script=scsgate_json.lua
                            (esempio conforme a DOMOTICZ e script  SCSGAtE_JSON.LUA

  V 1.6 - reset/restart se non aggancia il router con partenza normale (senza jumper) - OBBLIGATORIO IL JUMPER PER PROGRAMMARE *********************
  V 1.5 - aggiunta parametrizzazione numero porta udp da usare (default 52056)
  V 1.4 - ottimizzazione tempi di attesa
  V 1.3 - in modalita'  di connessione client NON attiva il server http per migliorare i tempi di risposta

  // --------------------------------------------------------------------------------------------------------------------------------------------------
  chiamate http:
    a_server.on ( "/", handleScan );                  // elenco reti wifi <- solo in modalita AP
    a_server.on ( "/", handleRoot );                  // hello <- solo in modalità Wifi CLIENT
    a_server.on ( "/scan", handleScan);               // query connessione <- solo in modalita CLIENT
    a_server.on ("/test", handleTest);                // pagina html/js di test
    a_server.on ("/status", handleStatus);            // status display
    a_server.on ("/help", handleHelp);                // help display
    a_server.on ("/picprog", handlePicProg);          // verify / start PIC firmware programming
    a_server.on ("/setting", handleSetting);          // setup wifi client
    a_server.on ("/reset", handleReset);              // reset app  ?device= <esp>|<pic>
    a_server.on ("/request", handleRequest);          // mappa richiesta comandi scs/knx
    a_server.on ("/gate.htm", handleGate);            // esecuzione comando scs/knx
    a_server.on ("/gate", handleGate);                // esecuzione comando scs/knx
    a_server.on ("/callback", handleCallback);        // richiesta setup callback http
    a_server.on ("/backsetting", handleBackSetting);  // setup callbackhttp
    a_server.on ("/mqttconfig", handleMqttConfig);    // richiesta setup mqtt
    a_server.on ("/mqttcfg", handleMqttCFG);          // setup mqtt
    a_server.on ("/mqttdevices", handleMqttDevices);  // inizio processo di censimento automatico dei devices scs
                                                    // ?request=clear prepare start  stop  term  query resend
    a_server.on ("/devicename", handleDeviceName);    // processo di rinominazione dei devices per alexa
    a_server.onNotFound ( handleNotFound );
  // --------------------------------------------------------------------------------------------------------------------------------------------------
*/

#include <WiFi.h>
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <AsyncJson.h>
//#include <ArduinoJson.h>
//#include <vector>

#include <HTTPClient.h>
//#include <ESP8266httpUpdate.h>

#include <mDNS.h>
#include "PubSubClient.h"
#include <WiFiUdp.h>

#include <Arduino.h>
#include <ArduinoOTA.h>

#include "esp_ota_ops.h"
#include "esp_image_format.h"
#include "esp_app_format.h"

#ifdef FFS
  #define CONFIG_LITTLEFS_FOR_IDF_3_2
  #define CONFIG_LITTLEFS_SPIFFS_COMPAT 1

  #include <FS.h>
  #include "LittleFS.h"

  #define SPIFFS LittleFS
  #define FORMAT_LITTLEFS_IF_FAILED true
#endif

char  flagRefreshDevices =  0;

AsyncEventSource events("/events");

#define MY_ENTITA 128
#define MAX_INFO  16
#define INFO_LEN  18


wifi_power_t TxPower = WIFI_POWER_8_5dBm;
uint8_t  ctrPotenza = 0;
uint8_t  forcePower = 0;

uint16_t delay_internal = 1000;

// =======================================================================================================================
// Struttura dati per il byte di aggiornamento
  typedef union _AGG_VAL
  {
    uint8_t Val;
    struct
    {
        uint8_t cancellata:1;
        uint8_t nuova:1;
        uint8_t modificata:1;
        uint8_t b3:1;
        uint8_t chiave:1;
        uint8_t tipo:1;
        uint8_t descrizione:1;
        uint8_t valore:1;
    } ;
  } AGG_VAL;


// ==========================================================================================================================
// ==========================================================================================================================
// ==========================================================================================================================
// Struttura fissa per il singolo dispositivo SCS (totale 32 byte a record)
struct DispositivoSCS {
    uint8_t  id;               // -1 se lo slot è libero, altrimenti da 0 a 128
    char address[3];       // es. "01" o "FF"
    uint8_t  tipo_codice;  // "LUC", "DIM", "TAP", etc. -  Memorizzato come numero (1, 3, 8, 11, 14, 15, 16)
	char descrizione[25];  // Massimo 24 caratteri + terminatore
    char valore_max[4];    // es. "100" o "ON"
    char alexa_id;         // ID POINTER PER FAUXMO

    char valore_corrente[6];       // Memorizza i valori temporanei (es. "100", "22.5") per tapparelle_pct e dimmer
    int8_t ptr_info;       // pointer a cardinfo (-1 se manca)
    char stato[12];        // Memorizza lo stato corrente (es. "ON", "OFF", "STOP")  
	uint8_t valido = 0;     // slot valido (0) o occupato (1) o temporaneo (2)
    AGG_VAL bits;          // in OR: 0x01=riga cancellata   0x02=riga nuova                0x10=chiave cambiata 
                           //        0x20=tipo aggiornato   0x40=descrizione aggiornata    0x80=valore cambiato   
};

// Struttura globale compressa per la Flash (circa 2.1 KB totali)
struct ConfigurazioneGlobale {
    uint8_t tapparelle_percentuale; // 0 = standard, 1 = percentuale
    DispositivoSCS elenco[MY_ENTITA];
//	DispositivoInfo cardinfo[MAX_INFO];
};

// 🟢 Questa è l'istanza in RAM che viene modificata in tempo reale dalle API
static ConfigurazioneGlobale sistema_scs;

struct IndiceSCS {
    uint8_t pointer;  // Puntatore all'indice dell'array 'sistema_scs.elenco' (0-63). 0xFF = non censito
};
// 🟢 Tabella indice globale ad accesso immediato (O(1)) per i 256 indirizzi possibili (0x00 - 0xFF)
static IndiceSCS tabella_indice[256];
// ==========================================================================================================================
// ==========================================================================================================================
// ==========================================================================================================================




#ifdef MY_DOMOTIC
// devices type:
// 0x01:1-switch
// 0x03:3-dimmer scs
// 0x04:4-dimmer on/off	        0x15:21-dimmer up/dw		   
// 0x08:8-tapparella stop       0x09:9-tapparella pct stop   
// 0x12:18-tapparella up/dwn    0x13:19-tapparella pct up/dwn (knx)
// 0x0B:11-generic              0x0C:12-generic 
// 0x0E:14-:allarme             0x0F:15-termostato
// 0x30-0x37:rele i2c
// 0x40-0x47:pulsanti i2c

#define LUCE			1
#define DIMMER			3
#define TAPPARELLA		8
#define GENERICO		11
#define ALLARME 		14
#define TERMOSTATO		15
#define TAPPARELLA_PCT  9
#define INFO            20
#define LOCALE			51

enum TipoEntita { ENTI_INTERRUTTORE, ENTI_SENSORE, ENTI_SLIDER, ENTI_TAPPARELLA, ENTI_TAPPARELLA_PCT, ENTI_INFO };

// uint8_t corrispondenza[5] = {1,15,3,8,9};
uint8_t corrispondenza[6] = {LUCE,TERMOSTATO,DIMMER,TAPPARELLA,TAPPARELLA_PCT,GENERICO};

int8_t hwType_to_tipo(int8_t hwtype)
{
	if ((hwtype >= 0) &&  (hwtype <= ENTI_INFO))
		return corrispondenza[hwtype];
	else
		return -1;
}

int8_t tipo_to_hwType(uint8_t tipo)
{
	int8_t hwtype = -1;
	switch (tipo)
	{
	case LUCE: hwtype = ENTI_INTERRUTTORE; break;
	case TERMOSTATO: hwtype = ENTI_SENSORE; break;
	case DIMMER: hwtype = ENTI_SLIDER; break;
	case TAPPARELLA: hwtype = ENTI_TAPPARELLA; break;
	case TAPPARELLA_PCT: hwtype = ENTI_TAPPARELLA_PCT; break;
	case GENERICO: hwtype = ENTI_INFO; break;
	case 0: hwtype = ENTI_INFO; break;
	}
	return hwtype;
}


// Struttura fissa per info del singolo dispositivo SCS 
struct DispositivoInfo {
    int id;              // -1 se lo slot è libero, altrimenti puntatore a sistema_scs.elenco[] (indice)
    char info_0[18];     // Memorizza la nota_1 stato corrente  
    char info_1[18];     // Memorizza la nota_1 stato corrente  
    char info_2[18];     // Memorizza la nota_2 stato corrente  
    char info_3[18];     // Memorizza la nota_3 stato corrente  
    char info_4[18];     // Memorizza la nota_4 stato corrente  
};
DispositivoInfo cardinfo[MAX_INFO];
int numeroInfo = 0;

bool autocensimento = false;

// ----------------------------------------------------------------  

AsyncEventSource eventity("/eventity");

#endif

#include "pagine_web.h"

// =======================================================================================================================

#include <EEPROM.h>

#ifdef USE_TCPSERVER
  WiFiServer tcpserver(5045);
  WiFiClient tcpclient;
#endif

// =======================================================================================================================
  typedef union _WORD_VAL
  {
    int  Val;
    char v[2];
    struct
    {
        char LB;
        char HB;
    } byte;
  } WORD_VAL;
// =======================================================================================================================
#define WEB_CLIENT
#define HTTP_PORT 80
unsigned int http_port = 80;
// =======================================================================================================================
#define _MODO "SCS"
#define _modo "scs"
#define ID_MY "interfaccia scs"
// =======================================================================================================================
#define PICPATH "/pic" _modo "gate.bin"

#define MYPFX  _modo

#define DEVICEREQUEST 'D'

#define SUBSCRIBE1 MYPFX "/+/set/+"
#define SUBSCRIBE2 MYPFX "/+/setlevel/+"
#define SUBSCRIBE3 MYPFX "/+/setposition/+"

#define NEW_SWITCH_TOPIC "homeassistant/switch/"
#define NEW_LIGHT_TOPIC  "homeassistant/light/"
#define NEW_COVER_TOPIC  "homeassistant/cover/"
#define NEW_CONFIG_TOPIC "/config"
#define NEW_COVERPCT_TOPIC  "homeassistant/cover/"
#define NEW_SENSOR_TOPIC  "homeassistant/sensor/"
#define NEW_GENERIC_TOPIC "homeassistant/generic/"

#define NEW_DEVICE_NAME  "{\"name\": \""

#define     SWITCH_SET   MYPFX "/switch/set/"
#define NEW_SWITCH_SET   "\",\"command_topic\": \"" SWITCH_SET
#define     SWITCH_STATE MYPFX "/switch/state/"
#define NEW_SWITCH_STATE "\",\"state_topic\": \"" SWITCH_STATE

#define NEW_LIGHT_SET    NEW_SWITCH_SET
#define NEW_LIGHT_STATE  NEW_SWITCH_STATE

#define     BRIGHT_SET   MYPFX "/switch/setlevel/"
#define NEW_BRIGHT_SET   "\",\"brightness_command_topic\": \"" BRIGHT_SET
#define     BRIGHT_STATE MYPFX "/switch/value/"
#define NEW_BRIGHT_STATE "\",\"brightness_state_topic\": \"" BRIGHT_STATE

#define     COVER_SET    MYPFX "/cover/set/"
#define NEW_COVER_SET    "\",\"command_topic\": \"" COVER_SET
#define     COVER_STATE  MYPFX "/cover/state/"
#define NEW_COVER_STATE  "\",\"state_topic\": \"" COVER_STATE

#define     COVERPCT_SET    MYPFX "/cover/setposition/"
#define NEW_COVERPCT_SET    "\",\"set_position_topic\": \"" COVERPCT_SET
#define     COVERPCT_STATE  MYPFX "/cover/value/"
#define NEW_COVERPCT_STATE  "\",\"position_topic\": \"" COVERPCT_STATE

#define     SENSOR_TEMP_STATE MYPFX "/sensor/temp/state/"
#define NEW_SENSOR_TEMP_STATE "\",\"state_topic\": \"" SENSOR_TEMP_STATE
#define NEW_SENSOR_TEMP_UNIT "\",\"unit_of_measurement\": \"°C"

#define     SENSOR_HUMI_STATE MYPFX "/sensor/humi/state/"
#define NEW_SENSOR_HUMI_STATE "\",\"state_topic\": \"" SENSOR_HUMI_STATE
#define NEW_SENSOR_HUMI_UNIT "\",\"unit_of_measurement\": \"%"

#define     SENSOR_PRES_STATE MYPFX "/sensor/pres/state/"
#define NEW_SENSOR_PRES_STATE "\",\"state_topic\": \"" SENSOR_PRES_STATE
#define NEW_SENSOR_PRES_UNIT "\",\"unit_of_measurement\": \"mBar"


#define     ALARM_SWITCH_STATE MYPFX "/alarm/state/"
#define NEW_ALARM_SWITCH_STATE "\",\"state_topic\": \"" ALARM_SWITCH_STATE    // armed or disarmed

#define     ALARM_ZONE_STATE MYPFX "/alarm/zone/state/"
#define NEW_ALARM_ZONE_STATE "\",\"state_topic\": \"" ALARM_ZONE_STATE

// generic:
//  scs/generic/set/<to>      <from><type><cmd>   command to send
//  scs/generic/from/<from>   <to><type><cmd>	  received
//  scs/generic/to/<to>       <from><type><cmd>   received

#define     GENERIC_SET   MYPFX "/generic/set/"
#define NEW_GENERIC_SET   "\",\"command_topic\": \"" GENERIC_SET
#define     GENERIC_FROM MYPFX "/generic/from/"
#define NEW_GENERIC_FROM  "\",\"from_topic\": \"" GENERIC_FROM
#define     GENERIC_TO   MYPFX "/generic/to/"
#define NEW_GENERIC_TO    "\",\"to_topic\": \"" GENERIC_FROM

#define NEW_UNIQUE_ID "\",\"unique_id\": \"scsgate_"
#define NEW_DEVICE_END   "\"}"
// =======================================================================================================================
#include "fauxmoESP.h"
fauxmoESP fauxmo;
unsigned char id_interfaccia_scs_knx = 0;
unsigned char id_fauxmo = 0;
unsigned char ArduinoOTAflag = 0;
#ifdef BLINKLED
char ledCtr = 0;
#endif
// =======================================================================================================================
char serIniOption = 0;
char mqtt_server[32];
char mqtt_user[32];
char mqtt_password[20];
char mqtt_persistence = 0;
char mqtt_connections = 0;
char mqtt_port[6];
char mqtt_retry = 0;
char mqtt_log = 0;
char mqtt_retrylimit = 24; //x 10=240sec (4min) -limite oltre il quale si resetta per broker non disponibile
char domoticMode;      // d=as domoticz      h=as homeassistant      maiuscolo=default switches

char domotic_options;  // 1= knx - indirizzi base dispari   0= base pari   3=multi-address

char alexaParam = 0;
int  countRestart = 0;

WiFiClient espClient;

PubSubClient client(espClient);

unsigned char mqttopen = 0;

char prevDevice;
int  prevcount = 0;

char prevAction;
char internal;
char prev_internal;
unsigned char devIx = 0;
unsigned char devCtr = 0;
char uartSemaphor = 0;
// =======================================================================================================================
enum _PICPROG_SM
{
    PICPROG_FREE = 0,
    PICPROG_START,
    PICPROG_REQUEST_WAIT,
    PICPROG_REQUEST_OK,
    PICPROG_FLASH_BLOCK,
    PICPROG_FLASH_WAIT,
    PICPROG_FLASH_SWAP,    // non usato
    PICPROG_FLASH_END,
    PICPROG_ERROR
};
static enum _PICPROG_SM sm_picprog = PICPROG_FREE;
WORD_VAL prog_address;
int    prog_error;
int    prog_retry = 0;
String prog_msg; 
#define PICBUF 64
char   prog_buffer[16];
char   prog_file_data[PICBUF];
char   prog_mode;
#ifdef FFS
File   picFw;
#endif
// =======================================================================================================================

#define INNERWAIT   90  // inner loop delay
#define OUTERWAIT  120  // outer loop delay

signed int now;
signed int prevTime = 0;  
signed int lastMsg = 0;
signed int lastCheck = 0; 
char       badCheck = 0;  

int  counter;

char picfwVersion[16];

//AsyncWebServer a_server(http_port);
AsyncWebServer* a_server_ptr = nullptr;
#define a_server (*a_server_ptr)

HTTPClient httpClient;



bool testWifi(void);
void setupAP(void);
void launchWeb(int webtype);
void createWebServer(int webtype);

const char* ssid = "ESP_" _MODO "GATE";
const char* passphrase = _modo "gate1";

IPAddress local_ip(0, 0, 0, 0);
IPAddress router_ip(0, 0, 0, 0);
IPAddress udp_remote_ip;
IPAddress tcp_remote_ip;
char connectionType = 255;  // 1=AP    0=router
unsigned int udp_remote_port;
unsigned int tcp_remote_port;
char   webon = 0;
String wifiSignals;
String content;

char   httpCallback[128];
int    statusCode;
char   udpopen = 9;
char   tcpopen = 0;
char   tcpuart = 0;     // 0=ritorno seriale su UDP   1=ritorno seriale su TCP    2=debug messg su TCP
String httpResp = "";   // resp= nessuna risposta      resp=i conferma immediata       resp=a risposta "get" con caratteri SCSGATE      resp=y  entrambe

char udpBuffer[255];
char requestBuffer[36];
unsigned char requestLen = 0;

char serObuffer[BUFNR][32];
char serOlen[BUFNR] = {0,0};
signed char bufSemaphor = -1;

char replyBuffer[255];
unsigned char replyLen;
unsigned char firstTime = 0;

unsigned int udpLocalPort = 52056;
WiFiUDP udpConnection;



// -----------------------------------------------------------------------------------------------------------------------------------------------
#define MAXEEPROM 4096  // 4096 is the MAX eeprom size in esp8266 
#define EESIGNATURE_FROM     0x5A 
#define EESIGNATURE          0x5B 

#define E_SSID   0         // ssid wifi  max 32
#define E_PASSW  32        // passw wifi max 32
#define E_IPADDR 64        // my ipaddr  max 16
#define E_ROUTIP 80        // router ip  max 16
#define E_PORT   96        // udp port   max 6
#define E_CALLBACK    102  // http callb max 98
#define E_EESIGNATURE 200  //            max 1 
#define E_DOMOTIC_OPTIONS 201 // bit 0:  0=dispari 1=pari
#define E_MQTT_BROKER 202  // mqtt_server   max 32
#define E_MQTT_PORT   234  // mqtt_port     max 6
#define E_DOMOTICMODE 240  // domotic mode  max 1
#define E_MQTT_USER   241  // mqtt_user     max 32
#define E_MQTT_PSWD   273  // mqtt_password max 32
#define E_MQTT_LOG    305  // mqtt_log      max 1
#define E_MQTT_PERSISTENCE 306  // mqtt_persistence      max 1
#define E_ALEXA       307		// alexa interface max 1
#define E_FILLER      308		// a disposizione  max 16

// =======================================================================================================================
char alexa_BUS_ix[MY_ENTITA];     // index:id alexa, contenuto: device index
// =======================================================================================================================

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");}


#ifdef VERBOSE
static inline int32_t asm_ccount(void) {

    int32_t r;

    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
}
#endif
// ==============================================================================================================
bool testWifi(void) {
  int c = 0;
#ifdef VERBOSE
  Serial.println("Waiting for Wifi to connect");
#endif
  while ( c < 22 ) {
    if (WiFi.status() == WL_CONNECTED)
    {
#ifdef VERBOSE
      Serial.println("Connected!");
#endif
      return true;
    }
    delay(500);
#ifdef VERBOSE
    Serial.print(WiFi.status());
#endif
    c++;
  }
#ifdef VERBOSE
  Serial.println("");
  Serial.println("Connect timed out, opening AP");
#endif
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------
void launchWeb(int webtype)
{ // webtype=1 : AP       webtype=0 : WIFI connected to router
  connectionType = webtype;
#ifdef VERBOSE
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
#endif


#ifndef WEB_CLIENT
  if (webtype == 1)
#endif
  {
    a_server_ptr = new AsyncWebServer(http_port);

    createWebServer(webtype);
    // Start the a_server


    a_server.begin();
    webon = 1;
#ifdef VERBOSE
    Serial.println("A_server started");
#endif
  }

  /*
    // Add service to MDNS-SD (andrebbe dopo il a_server.begin)
    MDNS.addService("http", "tcp", 80);
  */

}

// -----------------------------------------------------------------------------------------------------------------------------------------------
void setupAP(char dbg)
{
  if (dbg)
  {
     Serial.setDebugOutput(true);  // <------------ WIFI debug on serial
     Serial.println("AP start");
  }
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (dbg)
     Serial.println("now scan");

  WiFi.scanNetworks(true);
  long startScanMillis = millis();
  int n = -1;
  int tryctr = 0;
  #define MAXTRY 200
  while ((n < 0) && (tryctr < MAXTRY)) // 20 sec timeout
  {
     if (dbg)
        Serial.print(".");
     n = WiFi.scanComplete();
     delay(100);
     tryctr++;
  }
  char chn[12];
  int w;
  int ch = 0;
  char hBuffer[64];
  if (dbg)
  {
     Serial.println("now scanned");
     sprintf(hBuffer, "found %d channels",n);
     Serial.println(hBuffer);
  }
  for (w = 0; w < 12; w++)
  {
     chn[w] = 0;
  }  
  wifiSignals = "<ol>";
  for (int i = 0; i < n; ++i)
  {
//    sprintf(hBuffer, "<li>%*s  Ch:%d (%ddBm) %s </li>", 20, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
    sprintf(hBuffer, "<li>%*s  Ch:%d (%ddBm) </li>", 20, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i)); //,  WiFi.encryptionType(i));
	w = WiFi.channel(i);
	chn[w] = 1;
	if (dbg)
       Serial.println(hBuffer);
    wifiSignals += hBuffer;
  }
  if (tryctr >= MAXTRY)
  {
    wifiSignals += "<li>... timeout</li>";
  }
  
  wifiSignals += "</ol>";

//  if (dbg)
//  {
//     if (n == 0)
//       Serial.println("no networks found");
//     else
//       Serial.println(wifiSignals);
//
//     Serial.println("");
//  }

  WiFi.scanDelete();
  delay(100);

  WiFi.mode(WIFI_AP);
  for (w = 1; w < 11; w++)
  {
     if (chn[w] == 0) ch = w;
  }  
  WiFi.softAP(ssid, passphrase, ch);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  if (dbg)
     Serial.println("softap");

  launchWeb(1);

  if (dbg)
  {
     WiFi.printDiag(Serial1);             // wifi print diagnosys
     Serial.println("open UDP port");
  }

  udpopen = udpConnection.begin(udpLocalPort);
  if (dbg)
  {
     if (udpopen == 1)
       Serial.println ( "UDP server started" );
     else
       Serial.println ( "UDP open ERROR" );
  }
}
// =====================================================================================================
// ===============================SCRITTURA UART IMMEDIATA FUORI CICLO=DA REQUESTBUFFER=================
// ====================================non loggata======================================================
void immediateSend(void)
{
  if (requestLen > 0)
  {
    // =========================== send control char and data over serial =============================
    String log = "\r\ntx: ";
    char logCh[4];
    uartSemaphor = 1;
    int s = 0;
    while (s < requestLen)
    {
#ifdef UART_W_BUFFER
          Serial1.write(requestBuffer[s]);    //  scrittura SERIALE 
//          delayMicroseconds(120);
  #ifdef USE_TCPSERVER
          if (tcpuart == 2) 
          {
            sprintf(logCh, "%02X ", requestBuffer[s]);
            log += logCh;
          }
  #endif
#else
      delayMicroseconds(OUTERWAIT);
#ifndef DEBUG
      while (((USS(0) >> USTXC) & 0xff) > 0)     { // aspetta il buffer sia completamente vuoto
        delay(0);
      }

      delayMicroseconds(OUTERWAIT);
      USF(0) = requestBuffer[s];  // scrittura seriale
      delayMicroseconds(OUTERWAIT);

      while (((USS(0) >> USTXC) & 0xff) > 0)     { // aspetta il buffer sia completamente vuoto
        delay(0);
      }
#endif
#ifdef DEBUG
      sprintf(logCh, "%02X ", requestBuffer[s]);
      log += logCh;
#endif
#ifdef USE_TCPSERVER
#ifdef DEBUG_FAUXMO_TCP         
      if (tcpuart == 2) 
      {
        sprintf(logCh, "%02X ", requestBuffer[s]);
        log += logCh;
      }
#endif
#endif
      delayMicroseconds(OUTERWAIT);
#endif // UART_W_BUFFER
      s++;
    }
#ifdef DEBUG
    Serial.println("\r\n" + log);
#endif

#ifdef USE_TCPSERVER
#ifdef DEBUG_FAUXMO_TCP         
        if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
        {
          tcpclient.write((char*)&log[0], log.length());
          tcpclient.flush(); 
        }
#endif
#endif
    requestLen = 0;
    uartSemaphor = 0;
  }
}








// =====================================================================================================
// ================================LETTURA UART IMMEDIATA FUORI CICLO===================================
char immediateReceive(char firstChar)
{
  replyLen = 0;
  int wait = 0;
  
  while ((!Serial1.available() ) && (wait < OUTERWAIT * 100))
  {
    delayMicroseconds(10); // 120000 uS (120mS) timeout
    wait++;
  }

  if (Serial1.available() )
  {
    String log = "\r\nrx: ";
    char logCh[4];
    while (Serial1.available() && (replyLen < 255))
    {
      while (Serial1.available() && (replyLen < 255))
      {
        replyBuffer[replyLen] = Serial1.read();        // receive from serial USB
#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
        if (tcpuart == 2)  
        {
          sprintf(logCh, "%02X ", replyBuffer[replyLen]);
          log += logCh;
        }
  #endif
#endif
        
        if ((replyLen != 0) || (firstChar == 0) || (replyBuffer[0] == firstChar))
          replyLen++;
        delayMicroseconds(INNERWAIT);
      }
//      delayMicroseconds(OUTERWAIT);
      
      wait = 0;
      while ((!Serial1.available() ) && (wait < OUTERWAIT / 5))
      {
        delayMicroseconds(5);
        wait++;
      }
    }
#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
    if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
    {
      tcpclient.write((char*)&log[0], log.length());
      tcpclient.flush(); 
    }
  #endif
#endif
  }
  replyBuffer[replyLen] = 0;
  return replyLen;
}
// =====================================================================================================

// =============================================================================================
void setFirst(void)
{
  uartSemaphor = 1;

//  requestBuffer[requestLen++] = '@';
//  requestBuffer[requestLen++] = 0x17; // new - riassume @MX @Y1 F2 @l - ok per knx

//  requestBuffer[requestLen++] = '@';
  requestBuffer[requestLen++] = PIC_REQ;
  requestBuffer[requestLen++] = 0x15; // evita memo in eeprom (in 0x17)

//  requestBuffer[requestLen++] = '@'; 
  requestBuffer[requestLen++] = PIC_REQ;
  requestBuffer[requestLen++] = 'M'; // (in 0x17)
  requestBuffer[requestLen++] = 'X'; // (in 0x17)

//  requestBuffer[requestLen++] = '@';
  requestBuffer[requestLen++] = PIC_REQ;
  requestBuffer[requestLen++] = 'Y'; // (in 0x17)
  requestBuffer[requestLen++] = '1'; // (in 0x17)

//  requestBuffer[requestLen++] = '@';
  requestBuffer[requestLen++] = PIC_REQ;
  requestBuffer[requestLen++] = 'F'; // (in 0x17)
  requestBuffer[requestLen++] = '3';
  requestBuffer[requestLen++] = PIC_REQ;
  requestBuffer[requestLen++] = 'l'; // (in 0x17)
  uartSemaphor = 0;

  firstTime = 1;
  udp_remote_ip = {0, 0, 0, 0}; // kill udp connection

  immediateSend();   // IMMEDIATA senza log sse

}


// -----------------------------------------------------------------------------------------------------------------------------------------------
void WriteEEP(char stream, int eeaddress)
{
   EEPROM.write(eeaddress, stream);
}
// -----------------------------------------------------------------------------------------------------------------------------------------------
void WriteEEP(char stream, int eeaddress, int len)
{
	while (len--)
	{
	   EEPROM.write(eeaddress++, stream);
	}
}
// -----------------------------------------------------------------------------------------------------------------------------------------------
void WriteEEP(char * stream, int eeaddress, int len)
{
  unsigned char eof = 0;
  while (len) // ((len) && (eof == 0))
  {
    EEPROM.write(eeaddress, *stream);
    eeaddress++;
//    if (*stream == 0)
//      eof = 1;
    stream++;
    len--;
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------
void WriteEEP(String stream, int eeaddress)
{
  unsigned char eof = 0;
  char xs = 0;
  while (eof == 0)
  {
    EEPROM.write(eeaddress, stream[xs]);
    eeaddress++;
    if (stream[xs] == 0)
      eof = 1;
    xs++;
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------
void WriteEEP(String stream, int eeaddress, int maxlen)
{
  unsigned char eof = 0;
  char xs = 0;
  while ((eof == 0) && (xs < maxlen))
  {
    EEPROM.write(eeaddress, stream[xs]);
    eeaddress++;
    if (stream[xs] == 0)
      eof = 1;
    xs++;
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------
char ReadStream(int eeaddress)
{
  return EEPROM.read(eeaddress);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------
String ReadStream(char * stream, int eeaddress, int len, unsigned char tipo) // tipo=0 binary array   1:ascii array   2:ascii string
{
  unsigned char a;
  unsigned char i;
  String s = "";

  while (len)
  {
    a = EEPROM.read(eeaddress++);
    if (tipo > 0)
    {
      //          if (a == 0xFF) a = 0;
      if ((a < 0x20) || (a > 0x7F))   a = 0;   // only ASCII
      if (a == 0) len = 1;
    }
    if ((tipo == 1) || (tipo == 0))
    {
      *stream = a;
      stream++;
    }
    if (tipo == 2) 
    {
      if (a == 0) len = 1;
      else   s += char(a);
    }
      
    len--;
    i++;
  }
  if (tipo == 2)
    return s;
  return "";
}

//--------------------------------------------------------------------

void rigenera_tabella_indice() {
    // 1. Resetta completamente l'indice impostando tutti i 256 slot a 0xFF (non censito)
    for (int i = 0; i < 256; i++) {
        tabella_indice[i].pointer = 0xFF;
    }

    // 2. Mappa i dispositivi attivi presenti in RAM
    for (int i = 0; i < MY_ENTITA; i++) {
        if (sistema_scs.elenco[i].id != 0) {
            // Estrae il valore numerico dell'indirizzo memorizzato come stringa
            uint8_t addr_num = (uint8_t)strtol(sistema_scs.elenco[i].address, NULL, 16);
            
            // Assegna la posizione i dell'array a quel byte indice
            tabella_indice[addr_num].pointer = i;
        }
    }
    ESP_LOGI("SCS_INDEX", "⚡ Tabella indice diretta rigenerata con successo.");
}


void set_device_default(int i, uint8_t nuovo)  // imposta valori di default nuovo dispositivo
{
    if ((i < MY_ENTITA) && (i >= 0))
    {
        strcpy(sistema_scs.elenco[i].valore_corrente, "0");
        sistema_scs.elenco[i].ptr_info = -1;
        sistema_scs.elenco[i].valido = 1;
        sistema_scs.elenco[i].bits.Val = 0;
        if (nuovo)
        {
            uint8_t addr_num = sistema_scs.elenco[i].id;
            sprintf(sistema_scs.elenco[i].address,"%02X",addr_num);
            sistema_scs.elenco[i].alexa_id = 0;
            sistema_scs.elenco[i].bits.nuova = 1; 
            // Assegna la posizione i dell'array a quel byte indice
            tabella_indice[addr_num].pointer = i;
        }
        if (nuovo == 2)
        {
            sistema_scs.elenco[i].tipo_codice = LUCE;
            strcpy(sistema_scs.elenco[i].valore_max, "0");
        }
        if (sistema_scs.elenco[i].tipo_codice == TAPPARELLA || sistema_scs.elenco[i].tipo_codice == TAPPARELLA_PCT) {
            strcpy(sistema_scs.elenco[i].stato, "STOP");
        } else if (sistema_scs.elenco[i].tipo_codice == TERMOSTATO) {
            strcpy(sistema_scs.elenco[i].stato, " ");
        } else {
            strcpy(sistema_scs.elenco[i].stato, "OFF");
        }
    }
}

// 🟢 Variante 1: Ricerca immediata tramite Byte numerico (es. 0x32 o 0xA1)
uint8_t CercaAddress(uint8_t address) {
	if (address > 0xAF)
		return 0xFF;
	else
	    return tabella_indice[address].pointer;
}

uint8_t CercaAddressNew(uint8_t address) {
	if (address > 0xAF)
		return 0xFF;
	else
    if (tabella_indice[address].pointer != 0xFF)
        return tabella_indice[address].pointer;
    else
    {
        for(int i = 0; i < MY_ENTITA; i++) 
        {
          if(sistema_scs.elenco[i].valido == 0) 
          {
            sistema_scs.elenco[i].id = address;
            set_device_default(i, 2);
            return i;
            break;
          }
        } // end for
        return 0xFF;
    }
}

// 🟢 Variante 2: Ricerca tramite Stringa Esadecimale (es. "32" o "A1")
uint8_t CercaAddress(const char *address_str) {
    if (address_str == NULL || strlen(address_str) == 0) return 0xFF;
    
    // Converte al volo la stringa esadecimale nel rispettivo byte numerico
    uint8_t addr_num = (uint8_t)strtol(address_str, NULL, 16);
    return tabella_indice[addr_num].pointer;
}
uint8_t CercaAddressNew(const char *address_str) {
    if (address_str == NULL || strlen(address_str) == 0) return 0xFF;
    
    // Converte al volo la stringa esadecimale nel rispettivo byte numerico
    uint8_t addr_num = (uint8_t)strtol(address_str, NULL, 16);
    return CercaAddressNew(addr_num);
}



// Traduce il testo dell'HTML nel codice numerico durante il POST /add
uint8_t decodifica_tipo_a_codice(const char *tipo_str, uint8_t flag_tapparelle) {
    if (strcmp(tipo_str, "LUC") == 0) return LUCE;
    if (strcmp(tipo_str, "DIM") == 0) return DIMMER;
    if (strcmp(tipo_str, "TAP") == 0) return (flag_tapparelle == 1) ? TAPPARELLA_PCT : TAPPARELLA;
    if (strcmp(tipo_str, "TPC") == 0) return (flag_tapparelle == 1) ? TAPPARELLA_PCT : TAPPARELLA;
    if (strcmp(tipo_str, "GEN") == 0) return GENERICO;
    if (strcmp(tipo_str, "ALA") == 0) return ALLARME;
    if (strcmp(tipo_str, "TER") == 0) return TERMOSTATO;
    if (strcmp(tipo_str, "INF") == 0) return INFO;
    if (strcmp(tipo_str, "LOC") == 0) return LOCALE;
    return LUCE; // Default di sicurezza
}

// Traduce il codice numerico nel testo da inviare alla tabella HTML (GET /data)
const char* codifica_codice_a_tipo(uint8_t codice, uint8_t forza=0) {
    switch (codice) {
        case LUCE:			  return "LUC";
        case DIMMER:	    return "DIM";
        case TAPPARELLA:	
        case TAPPARELLA_PCT: 
            if ((!sistema_scs.tapparelle_percentuale) || (forza))
                return "TAP";
            else
                return "TPC";
        case GENERICO:		return "GEN";
        case ALLARME:		  return "ALA";
        case TERMOSTATO:	return "TER";
        case INFO:			  return "INF";
        case LOCALE:		  return "LOC";
        default: return "LUC";
    }
}


#include <Preferences.h>

void salva_dispositivi_preferences() {
    Preferences prefs;
	uint8_t counter = 0;
    // Apre il namespace "scs_data" (il flag false significa "modalità scrittura")
    if (prefs.begin("scs_data", false)) {
        
        prefs.putUChar("tap_pct", sistema_scs.tapparelle_percentuale);
        
        for (int i = 0; i < MY_ENTITA; i++) {
            String chiave = "d" + String(i);
            
            if ((sistema_scs.elenco[i].id != 0) && (sistema_scs.elenco[i].valido == 1)) {
                char buffer_csv[128];
                snprintf(buffer_csv, sizeof(buffer_csv), "%d,%s,%d,%s,%s,%d",
                         sistema_scs.elenco[i].id,
                         sistema_scs.elenco[i].address,
                         sistema_scs.elenco[i].tipo_codice,
                         sistema_scs.elenco[i].descrizione,
                         sistema_scs.elenco[i].valore_max,
                         sistema_scs.elenco[i].alexa_id);
                         
                prefs.putString(chiave.c_str(), buffer_csv);
				counter++;
            } else {
                // Rimuove la chiave se lo slot è vuoto (gestisce l'assenza internamente senza errori)
                prefs.remove(chiave.c_str());
            }
        }
        prefs.end(); // Esegue il commit e chiude automaticamente

        if (events.count() != 0) 
        {
          char buflog[120];
          sprintf(buflog,"[%d] [CMD] Salvataggio NVS completato con Preferences! %d elementi",(millis()/100), counter);
          events.send(buflog, "log", millis());
        }
  
		ESP_LOGI("NVS_SCS", "💾 Salvataggio completato con Preferences!");
    }
}



void inizializza_e_leggi_dispositivi_preferences() {
    // Azzera preventivamente tutta la memoria RAM per partire da una situazione immacolata
    memset(&sistema_scs, 0, sizeof(ConfigurazioneGlobale));
    for (int i = 0; i < MY_ENTITA; i++) {
        sistema_scs.elenco[i].id = 0; // Slot vuoti di base
        strcpy(sistema_scs.elenco[i].stato, "OFF");
        strcpy(sistema_scs.elenco[i].valore_corrente, "0");
        sistema_scs.elenco[i].ptr_info = -1;
        sistema_scs.elenco[i].valido = 0;
        sistema_scs.elenco[i].bits.Val = 0;
        sistema_scs.elenco[i].alexa_id = 0;
    }
    sistema_scs.tapparelle_percentuale = 0;

    for (int i = 0; i < MAX_INFO; i++) {
        cardinfo[i].id = -1; // Slot vuoti di base
        memset(cardinfo[i].info_0, 0, sizeof(cardinfo[i].info_0));
    }

    Preferences prefs;
	uint8_t counter = 0;

	// Apriamo il namespace "scs_data" in modalità sola lettura (true)
    if (prefs.begin("scs_data", true)) {
        
        // Legge il flag tapparelle (se non esiste nella NVS, restituisce il valore di default 0)
        sistema_scs.tapparelle_percentuale = prefs.getUChar("tap_pct", 0);

        // Ricostruisce l'array leggendo le singole stringhe CSV memorizzate
        for (int i = 0; i < MY_ENTITA; i++) {
            // CORRETTO: Portato a 16 byte per evitare stack overflow con indici alti (es. d10, d100)
            char chiave[16];
            snprintf(chiave, sizeof(chiave), "d%d", i);

            // Verifichiamo se la chiave esiste prima di allocare o leggere
            if (prefs.isKey(chiave)) {
                // Leggiamo la stringa direttamente dalla NVS
                String csv_string = prefs.getString(chiave, "");
                
                if (csv_string.length() > 0) {
                    int parsed_id = 0;
                    int parsed_tipo = 1;
                    char parsed_addr[4] = "";
                    char parsed_desc[25] = "";
                    char parsed_val_max[8] = "";
                    int parsed_alexa_id = 0;

                    // Parsing sicuro estraendo il puntatore char* della stringa Arduino (.c_str())
                    sscanf(csv_string.c_str(), "%d,%[^,],%d,%[^,],%s", 
                           &parsed_id, parsed_addr, &parsed_tipo, parsed_desc, parsed_val_max, &parsed_alexa_id);

                    sistema_scs.elenco[i].id = parsed_id;
                    sistema_scs.elenco[i].tipo_codice = (uint8_t)parsed_tipo;
                    strcpy(sistema_scs.elenco[i].address, parsed_addr);
                    strcpy(sistema_scs.elenco[i].descrizione, parsed_desc);
                    strcpy(sistema_scs.elenco[i].valore_max, parsed_val_max);
                    sistema_scs.elenco[i].alexa_id = parsed_alexa_id;
                    
                    set_device_default(i, 0);

					counter++;
                }
            }
        }
        // Chiudiamo il database per liberare le risorse hardware della NVS
        prefs.end();
        if (events.count() != 0) 
        {
            char buflog[120];
            sprintf(buflog,"[%d] [CMD] Rilettura NVS completata con Preferences! %d elementi",(millis()/100), counter);
            events.send(buflog, "log", millis());
        }
    }
  rigenera_tabella_indice();
}

void cancella_vecchia_eeprom() {
    Preferences prefs;
    
    // La libreria EEPROM dell'ESP32 usa internamente il namespace "eeprom"
    if (prefs.begin("eeprom", false)) {
        
        // Cancella TUTTI i dati memorizzati dentro questo specifico namespace
        if (prefs.clear()) {
            Serial.println("🧹 Vecchi dati EEPROM rimossi con successo dalla NVS!");
        } else {
            Serial.println("❌ Errore durante la cancellazione della EEPROM.");
        }
        
        prefs.end();
    }
}


// Funzione helper inline per leggere 2 caratteri esadecimali senza modificare il buffer originale
// Legge 2 caratteri esadecimali da un puntatore const char*
// Converte un singolo carattere ASCII esadecimale ('0'-'9', 'A'-'F', 'a'-'f') nel suo valore 0-15
inline uint8_t hexCharToVal(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 0;
}

// Legge 2 caratteri esadecimali da un puntatore senza usare array né strtoul
inline uint8_t parseHexByte(const char* ptr) {
  return (hexCharToVal(*ptr) << 4) | hexCharToVal(*(ptr + 1));
}
// =============================================================================================
inline uint8_t aConvert(const char* aData) {
  if (!aData || !aData[0] || !aData[1]) return 0; // Protezione da stringhe corte o nulle
  return (hexCharToVal(aData[0]) << 4) | hexCharToVal(aData[1]);
}
inline uint8_t aConvert(const String& aData) {
    return aConvert(aData.c_str());
}
// =============================================================================================
inline uint16_t wConvert(const char* aData) {
  // Verifica che ci siano almeno 4 caratteri validi prima del terminatore '\0'
  if (!aData || !(*aData) || !(*(aData + 1)) || !(*(aData + 2)) || !(*(aData + 3))) {
    return 0;
  }

  // Legge i primi 2 caratteri (byte alto) e i successivi 2 caratteri (byte basso)
  return ((uint16_t)aConvert(aData) << 8) | (uint16_t)aConvert(aData + 2);
}

inline uint16_t wConvert(const String& aData) {
  return wConvert(aData.c_str());
}
// =============================================================================================
// devtype: 1=SWITCH, 3=LIGHT, 4=LIGHT, 8=COVER, 11=GENERIC, 14=ALARM BOARD, 15=termostato, 18=COVER, 19=COVER PCT
size_t MQTTnewdiscover(char devtype, const char* addrDevice, const char* nomeDevice, char* logBuffer, size_t logMaxLen)
{
  constexpr size_t TOPIC_MAX_LEN   = 128;
  constexpr size_t PAYLOAD_MAX_LEN = 512;
  constexpr size_t LOG_MAX_LEN     = TOPIC_MAX_LEN + PAYLOAD_MAX_LEN;

  char topic[TOPIC_MAX_LEN];
  char payload[PAYLOAD_MAX_LEN];
  int len = 0;
  char rc = 0;

  if (devtype == 1)  // D=define new device SWITCH
  {
    rc = 1;
    snprintf(topic, sizeof(topic), "%s%s%s", NEW_SWITCH_TOPIC, addrDevice, NEW_CONFIG_TOPIC);
    len = snprintf(payload, sizeof(payload), "%s%s%s%s%s",
                   NEW_DEVICE_NAME, nomeDevice,
                   NEW_SWITCH_SET, addrDevice,
                   NEW_SWITCH_STATE);
  }
  else if ((devtype == 3) || (devtype == 4)) // D=define new device DIMMER
  {
    rc = 1;
    snprintf(topic, sizeof(topic), "%s%s%s", NEW_LIGHT_TOPIC, addrDevice, NEW_CONFIG_TOPIC);
    len = snprintf(payload, sizeof(payload), "%s%s%s%s%s%s%s%s%s",
                   NEW_DEVICE_NAME, nomeDevice,
                   NEW_LIGHT_SET, addrDevice,
                   NEW_LIGHT_STATE, addrDevice,
                   NEW_BRIGHT_SET, addrDevice,
                   NEW_BRIGHT_STATE);
  }
  else if ((devtype == 8) || (devtype == 18)) // D=define new device COVER
  {
    rc = 1;
    snprintf(topic, sizeof(topic), "%s%s%s", NEW_COVER_TOPIC, addrDevice, NEW_CONFIG_TOPIC);
    len = snprintf(payload, sizeof(payload), "%s%s%s%s%s",
                   NEW_DEVICE_NAME, nomeDevice,
                   NEW_COVER_SET, addrDevice,
                   NEW_COVER_STATE);
  }
  else if (devtype == 11) // D=define new device GENERIC
  {
    rc = 1;
    snprintf(topic, sizeof(topic), "%s%s%s", NEW_GENERIC_TOPIC, addrDevice, NEW_CONFIG_TOPIC);
    len = snprintf(payload, sizeof(payload), "%s%s%s%s%s%s%s",
                   NEW_DEVICE_NAME, nomeDevice,
                   NEW_GENERIC_SET, addrDevice,
                   NEW_GENERIC_FROM, addrDevice,
                   NEW_GENERIC_TO);
  }
  else if ((devtype == 9) || (devtype == 19)) // D=define new device COVER PCT
  {
    rc = 1;
    snprintf(topic, sizeof(topic), "%s%s%s", NEW_COVERPCT_TOPIC, addrDevice, NEW_CONFIG_TOPIC);
    len = snprintf(payload, sizeof(payload), "%s%s%s%s%s%s%s",
                   NEW_DEVICE_NAME, nomeDevice,
                   NEW_COVER_SET, addrDevice,
                   NEW_COVERPCT_SET, addrDevice,
                   NEW_COVERPCT_STATE);
  }
  else if (devtype == 14) // 0x0E) D=define new device ALARM BOARD
  {
  }

  if (rc == 1 && len > 0 && (size_t)len < sizeof(payload))
  {
    // Aggiunge la parte finale comune direttamente in coda a 'payload'
    snprintf(payload + len, sizeof(payload) - len, "%s%s%s%s",
             addrDevice, NEW_UNIQUE_ID, addrDevice, NEW_DEVICE_END);

    client.publish(topic, payload, false);

    if (logBuffer != nullptr && logMaxLen > 0)
    {
      int written = snprintf(logBuffer, logMaxLen, "%s%s\n", topic, payload);
      if (written > 0) {
        return ((size_t)written < logMaxLen) ? (size_t)written : (logMaxLen - 1);
      }
    }
  }

  return rc;
}

// Overload per mantenere la compatibilità con chiamate esistenti che passano una String
//inline size_t MQTTnewdiscover(char devtype, const char* addrDevice, const String& nomeDevice, char* logBuffer, size_t logMaxLen)
//{
//  return MQTTnewdiscover(devtype, addrDevice, nomeDevice.c_str(), logBuffer, logMaxLen);
//}



// =============================================================================================
void WriteError(const char *msgLog)
{
  char Log[250];
  snprintf_P(Log, sizeof(Log), "%lu - %s", millis(), msgLog);
  if (mqttopen == 3)
    client.publish(_MODO "ERROR", Log, 0);
}
void WriteError(String msgLog)
{
  char Log[250];
  snprintf_P(Log, sizeof(Log), "%lu - %s", millis(), msgLog.c_str());
  if (mqttopen == 3)
    client.publish(_MODO "ERROR", Log, 0);
}
// =============================================================================================
char reconnect()
{
  {
    char rc;
    if ((mqtt_user[0] != 0x00) && (mqtt_password[0] != 0x00))
    {
      rc = client.connect("ESP" _modo "gate" _FW_VERSION, mqtt_user, mqtt_password);
    }
    else
    {
      rc = client.connect("ESP" _modo "gate" _FW_VERSION);
    }

    if (rc)
      return 2;
    else
    {
      client.disconnect();
      return 1;
    }
  }
}









//==============================================================================================================
void MqttCallback(char* topic, byte* payload, unsigned int length)
// =============================================================================================================
// ================================ messaggio MQTT in arrivo (comando web) =====================================
// =============================================================================================================
{
  unsigned char devtype = 0;
  char packetBuffer[255];

  unsigned char device = 0xFF;
  unsigned char reply = 0;
  unsigned char command = 0xFF;
  const char* rtopicBase = nullptr;

  // Copia sicura del payload con terminatore nullo
  size_t copyLen = (length < sizeof(packetBuffer) - 1) ? length : (sizeof(packetBuffer) - 1);
  memcpy(packetBuffer, payload, copyLen);
  packetBuffer[copyLen] = '\0';

#ifdef USE_TCPSERVER
  if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
  {
    tcpclient.print("\r\nsub: ");
    tcpclient.print(topic);
    tcpclient.print(" ");
    tcpclient.println(packetBuffer);
    tcpclient.flush(); 
  }
#endif

  if (firstTime == 0) 
  {
    setFirst();
    internal = 1;
  }

  constexpr size_t lenSwitch   = sizeof(SWITCH_SET) - 1;
  constexpr size_t lenBright   = sizeof(BRIGHT_SET) - 1;
  constexpr size_t lenCoverPct = sizeof(COVERPCT_SET) - 1;
  constexpr size_t lenGeneric  = sizeof(GENERIC_SET) - 1;
  constexpr size_t lenCover    = sizeof(COVER_SET) - 1;

  // --------------------------------------- SWITCHES -------------------------------------------
  if (strncmp(topic, SWITCH_SET, lenSwitch) == 0)
  {
    devtype = 1; // switch
    rtopicBase = SWITCH_STATE;
    reply = 1;
    device = parseHexByte(topic + lenSwitch);

    if (strncmp(packetBuffer, "ON", 2) == 0) {
      command = 0x00;
    } else if (strncmp(packetBuffer, "OFF", 3) == 0) {
      command = 0x01;
    }
  }
  // --------------------------------------- LIGHTS DIMM ------------------------------------------
  else if (strncmp(topic, BRIGHT_SET, lenBright) == 0)
  {
#ifdef DEBUG
    rtopicBase = BRIGHT_STATE;
    Serial.println("dimmer");
#endif
    device = parseHexByte(topic + lenBright);
    devtype = 3; // light

    if (strncmp(packetBuffer, "ON", 2) == 0) {
      command = 0x00; // on/up
    } else if (strncmp(packetBuffer, "OFF", 3) == 0) {
      command = 0x01; // off/down
    } else {
      int pct = atoi(packetBuffer);
      
      if ((domoticMode == 'h') || (domoticMode == 'H')) {
        pct = (pct * 100) / 255;
      }
      
      pct = (pct + 5) / 10;
      if (pct > 9) pct = 9;
      if (pct == 0) pct = 1;
      
      command = (unsigned char)((pct << 4) | 0x0D);

#ifdef DEBUG
      unsigned char action = (command >> 4) * 10;
      snprintf(packetBuffer, sizeof(packetBuffer), "%02u", action);
      Serial.print(" payload ");
      Serial.println(packetBuffer);
#endif
    }
  }
  // --------------------------------------- COVERPCT ------------------------------------------------
  else if (strncmp(topic, COVERPCT_SET, lenCoverPct) == 0)
  {
#ifdef DEBUG
    Serial.println("cover%");
#endif
    rtopicBase = COVERPCT_STATE;
    device = parseHexByte(topic + lenCoverPct);
    devtype = 9; // coverpct

    if (strncmp(packetBuffer, "STOP", 4) == 0) {
      command = 0;
    } else if (strncmp(packetBuffer, "ON", 2) == 0 || strncmp(packetBuffer, "CLOSE", 5) == 0) {
      command = 2; // discesa - chiudi
    } else if (strncmp(packetBuffer, "OFF", 3) == 0 || strncmp(packetBuffer, "OPEN", 4) == 0) {
      command = 1; // salita - apri
    } else {
      command = (unsigned char)atoi(packetBuffer);
    }
  }
  // --------------------------------------- GENERIC -------------------------------------------
  else if (strncmp(topic, GENERIC_SET, lenGeneric) == 0)
  {
    devtype = 11; // generic
    rtopicBase = GENERIC_TO;
    reply = 0;
    device = parseHexByte(topic + lenGeneric);

    // packet: ffttcc -> <from> <type> <command>
    char from = parseHexByte(packetBuffer);
    char type = parseHexByte(packetBuffer + 2);
    command   = parseHexByte(packetBuffer + 4);

    requestBuffer[requestLen++] = PIC_REQ;
    requestBuffer[requestLen++] = 'y';     // 0x79
    requestBuffer[requestLen++] = device;  // to device
    requestBuffer[requestLen++] = from;    // from device
    requestBuffer[requestLen++] = type;    // type:command
    requestBuffer[requestLen++] = command; // command
  }
  // --------------------------------------- COVER ------------------------------------------------
  else if (strncmp(topic, COVER_SET, lenCover) == 0)
  {
#ifdef DEBUG
    Serial.println("cover");
#endif
    rtopicBase = COVER_STATE;
    reply = 1;
    devtype = 8; // cover
    device = parseHexByte(topic + lenCover);

    if (strncmp(packetBuffer, "ON", 2) == 0 || strncmp(packetBuffer, "CLOSE", 5) == 0) {
      command = 0x09; // discesa - chiudi
    } else if (strncmp(packetBuffer, "OFF", 3) == 0 || strncmp(packetBuffer, "OPEN", 4) == 0) {
      command = 0x08; // salita - apri
    } else if (strncmp(packetBuffer, "STOP", 4) == 0) {
      command = 0x0A;
    }
  }

  // --------------------------------------- REPLY MQTT -------------------------------------------
  if (reply == 1 && rtopicBase != nullptr)
  {
    const char* outPayload = packetBuffer;

    if ((domoticMode == 'h') || (domoticMode == 'H'))
    {
      if (strncmp(packetBuffer, "CLOSE", 5) == 0) {
        outPayload = "closed";
      } else if (strncmp(packetBuffer, "OPEN", 4) == 0) {
        outPayload = "open";
      }
    }

    char fullReplyTopic[64];
    // Formatta direttamente il valore esadecimale di 'device' a 2 cifre (es. 0A)
    snprintf(fullReplyTopic, sizeof(fullReplyTopic), "%s%02X", rtopicBase, device);
    
    client.publish(fullReplyTopic, outPayload, mqtt_persistence);
  }




  // as gate.htm   handleGate -     firsttime ?

  if (command != 0xFF)
  {
    if (uartSemaphor == 1)
    {
      char log[80];
      sprintf(log, "UART buffer collision ERROR !");
      WriteError(log);
    }

    if ((devtype == 9) || (devtype == 19))	// <====================COVERPCT=========================
    {
      requestBuffer[requestLen++] = PIC_REQ;
      requestBuffer[requestLen++] = 'u';
      requestBuffer[requestLen++] = device;		// to   device
      requestBuffer[requestLen++] = command;    // command (%)
    }
    else
    if (devtype == 4)	// <====================DIMMER KNX=======================
    {
      requestBuffer[requestLen++] = PIC_REQ;
      requestBuffer[requestLen++] = 'm';
      requestBuffer[requestLen++] = highByte(device); // to   device
      requestBuffer[requestLen++] = lowByte(device); // to   device
      requestBuffer[requestLen++] = command;    // command (%)
    }
    else
    if (devtype != 11)	// <====================not GENERIC=======================
    {
      requestBuffer[requestLen++] = PIC_REQ;
      requestBuffer[requestLen++] = 'y';    // 0x79 (@y: invia a pic da MQTT cmd standard da inviare sul bus)
      requestBuffer[requestLen++] = device; // to   device
      requestBuffer[requestLen++] = 0x00;   // from device
      requestBuffer[requestLen++] = 0x12;   // type:command
      requestBuffer[requestLen++] = command;// command
    }
  }
}





void udpTrace(String  data)
{
  if ((udpopen == 1) && (udp_remote_ip))
  {
    int success;
    do  {
      success =  udpConnection.beginPacket(udp_remote_ip, udp_remote_port);
    }   while (!success);

    const char*  datastr = data.c_str();
    int n = 0;
    while (n < data.length())
    {
      udpConnection.write(datastr[n++]);     // UDP reply
    }
    success = udpConnection.endPacket();
  } // udp_remote_ip
}
// =============================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------------------
void scanProcess()
// -----------------------------------------------------------------------------------------------------------------------------------------------
{
    String sTemp;

    if ( connectionType == 1 ) // server web AP
      sTemp = WiFi.softAPIP().toString();
    else
      sTemp = WiFi.localIP().toString();
      
    content = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION " at ";
    content += sTemp;
    content += "<p>";
    content += wifiSignals;

    content += "</p><form method='get' action='setting'><label>SSID: </label>";
    content += "<input name='ssid' maxlength=32 value='";
    sTemp = ReadStream(&sTemp[0], E_SSID, 32, 2);  // tipo=0 binary array   1:ascii array   2:ascii string
    content += sTemp;
    content += "' style='width:150px'><label> PSW: </label><input name='pass' maxlength=64 value='";
    sTemp = ReadStream(&sTemp[0], E_PASSW, 32, 2);  // tipo=0 binary array   1:ascii array   2:ascii string
    content += sTemp;
    content += "' style='width:150px'><label> IP address:</label><input name='ip' maxlength=16 value='";
    sTemp = ReadStream(&sTemp[0], E_IPADDR, 16, 2);  // tipo=0 binary   1:ascii   2=string
    content += sTemp;
    content += "' style='width:100px'><label> Gateway IP</label><input name='rip' maxlength=16 value='";
    sTemp = ReadStream(&sTemp[0], E_ROUTIP, 16, 2);  // tipo=0 binary   1:ascii  2=string
    content += sTemp;
    content += "' style='width:100px'><label>UDP port</label><input name='uport' maxlength=5 value='";
    sTemp = ReadStream(&sTemp[0], E_PORT, 6, 2);  // tipo=0 binary   1:ascii   2=string
    content += sTemp;
    content += "' style='width:50px'><input type='submit'></form></html>";
    wifiSignals = "";
}







#ifdef MY_TABELLA

// -----------------------------------------------------------------------------------------------------------------------------------------------
void uploadDevices(char loadAll)
// bottone Aggiorna di /dispositivi
//                  aggiorna il PIC e salva su NVS
// -----------------------------------------------------------------------------------------------------------------------------------------------
{

// devtype: 1=SWITCH, 3=LIGHT, 4=LIGHT, 8=COVER, 11=GENERIC, 14=ALARM BOARD, 15=termostato, 18=COVER, 19=COVER PCT
//            LUC       DIM      DIM      TAP       GEN         ALA             TER            TAP       TAP 
    WORD_VAL maxp;
    maxp.Val = 0;

    //      if (sistema_scs.tapparelle_percentuale == false)
      {
        requestBuffer[requestLen++] = PIC_REQ;
        requestBuffer[requestLen++] = 'U';
        requestBuffer[requestLen++] = '9';  // pulisce tabella % su pic
        immediateSend();
        delay(300);
        immediateReceive('k');
        delay(100);
      }  // cover == "false"

#ifdef DEBUG_EVENTITY
  char buflog[120];
  sprintf(buflog,"[%d] [CMD] clr ",(millis()/100));
  events.send(buflog, "log", millis());
#endif
//    }


// **************************************************************************************************************

    for(int i = 0; i < MY_ENTITA; i++) 
    {
      if (sistema_scs.elenco[i].valido == 1)
      {
          if ((sistema_scs.elenco[i].tipo_codice == TAPPARELLA) || (sistema_scs.elenco[i].tipo_codice == TAPPARELLA_PCT))
          {
              if (sistema_scs.tapparelle_percentuale == false)
                  sistema_scs.elenco[i].tipo_codice = TAPPARELLA;
              else
              {
                  sistema_scs.elenco[i].tipo_codice = TAPPARELLA_PCT;
                  char *ch;
                  maxp.Val = (int)strtoul(sistema_scs.elenco[i].valore_max, &ch, 10);
                  requestBuffer[requestLen++] = PIC_REQ;
                  requestBuffer[requestLen++] = 'U';
                  requestBuffer[requestLen++] = '8';        // tapparella pct
                  requestBuffer[requestLen++] = sistema_scs.elenco[i].id;     // device id
                  requestBuffer[requestLen++] = sistema_scs.elenco[i].tipo_codice;    // device type
                  requestBuffer[requestLen++] = maxp.byte.HB;    // max position H
                  requestBuffer[requestLen++] = maxp.byte.LB;    // max position L
                  immediateSend();
                  immediateReceive('k');
                  delay(50);
              }
          }
      }
#ifdef DEBUG_EVENTITY
  char buflog[120];
  sprintf(buflog,"[%d] [CMD] new: %s %s %s : %02X, [%d]",(millis()/100), sistema_scs.elenco[i].address, sistema_scs.elenco[i].tipo_codice, sistema_scs.elenco[i].descrizione, i);
  events.send(buflog, "log", millis());
#endif
    }
    rigenera_tabella_indice();
    salva_dispositivi_preferences();
}
#endif



// -----------------------------------------------------------------------------------------------------------------------------------------------
int rimuovi_spazi_string(String &str) {
    int len = 0;
    int src = 0;
    int dst = 0;
    int str_len = str.length();

    while (src < str_len) {
        // Se incontra '%', salta i due caratteri successivi (mantenendo la tua logica C)
        if (str[src] == '%') {
            src += 3; // Salta il '%' e i successivi 2 caratteri
            continue;
        }
        
        if (str[src] != ' ') {
            str[dst] = str[src];
            dst++;
            len++;
        }
        src++;
    }
    
    // Ridimensiona la stringa per tagliare la parte rimanente vecchia
    str.remove(dst); 
    return len;
}
// -----------------------------------------------------------------------------------------------------------------------------------------------

// =====================================================================================================
// =====================================================================================================
/*
WiFi.begin(ssid, password);

// Scegli una delle seguenti opzioni prima che il Wi-Fi completi la connessione:
Serial.printf("Potenza Wi-Fi attuale: %.2f dBm\n", TxPower * 0.25);
TxPower 
WiFi.setTxPower(WIFI_POWER_15dBm);  // 15 dBm (Consigliato standard)
WiFi.setTxPower(WIFI_POWER_13dBm);  // 13 dBm (Ottimo intermedio)
WiFi.setTxPower(WIFI_POWER_11dBm);  // 11 dBm (Bilanciato per micro-schede)
WiFi.setTxPower(WIFI_POWER_8_5dBm); // 8.5 dBm (Massima stabilità/basso consumo)
*/


void gestisciPotenzaDinamica(uint8_t force=0) {
  // Gestione del timer basata sui cicli di chiamata (presuppone loop di 1 secondo)
  if ((ctrPotenza++ < 30) && (force == 0)) 
      return;
  ctrPotenza = 0;

  if (force)
      forcePower = force;
  else
  if (forcePower)
      force = forcePower;

  if (WiFi.status() == WL_CONNECTED) {
    // Se forziamo manualmente, simuliamo un RSSI neutro per non interferire con i blocchi successivi
    long rssi = (force > 0) ? -70 : WiFi.RSSI(); 
    wifi_power_t nuovaPotenza = TxPower;


    // LOGICA DI CONTROLLO CON TETTO MASSIMO A 13 dBm
    if ((rssi < -75) || (force == 3)) {
      // Anche con segnale debole, NON superiamo i 13 dBm per evitare auto-interferenze termiche/radio
      nuovaPotenza = WIFI_POWER_13dBm; 
    }
    else if ((rssi >= -75 && rssi <= -55) || (force == 2)) {
      // Segnale intermedio/buono: 11 dBm è il punto di massima stabilità
      nuovaPotenza = WIFI_POWER_11dBm;
    }
    else if ((rssi > -55) || (force == 1)) {
      // Segnale eccellente: scendiamo a 8.5 dBm
      nuovaPotenza = WIFI_POWER_8_5dBm;
    }


    // Applica il cambio solo se la potenza calcolata è DIVERSA da quella attuale
    if (nuovaPotenza != TxPower) {
      TxPower = nuovaPotenza;
      WiFi.setTxPower(TxPower);
//      Serial.printf("[Wi-Fi] TX Power aggiornato a: %.2f dBm (RSSI: %ld)\n", TxPower * 0.25, rssi);
    }
  }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------

String getWiFiAuthMode() {
  if (WiFi.status() != WL_CONNECTED) {
    return "Non connesso";
  }

  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    switch (ap_info.authmode) {
      case WIFI_AUTH_OPEN:            return "OPEN";
      case WIFI_AUTH_WEP:             return "WEP";
      case WIFI_AUTH_WPA_PSK:         return "WPA-PSK";
      case WIFI_AUTH_WPA2_PSK:        return "WPA2-PSK";
      case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2-PSK";
      case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
      case WIFI_AUTH_WPA3_PSK:        return "WPA3-SAE";
      case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3-PSK";
      case WIFI_AUTH_WAPI_PSK:        return "WAPI-PSK";
      default:                        return "Sconosciuto (" + String(ap_info.authmode) + ")";
    }
  }
  return "Errore lettura AP";
}

// -----------------------------------------------------------------------------------------------------------------------------------------------
void createWebServer(int webtype)
// -----------------------------------------------------------------------------------------------------------------------------------------------
{


  if ( webtype == 0 ) // server web  on NETWORK
  {

#ifdef MY_TABELLA
    // Servizio Pagina Principale HTML
    a_server.on("/dispositivi", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/html", index_html_TAB);
    });

      // API per inviare la lista in formato CSV (senza JSON)

a_server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    
    // Avviamo una risposta in streaming a piccoli pezzi (chunk) per salvaguardare la RAM
    AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        
        // Indice statico per mantenere la posizione del ciclo tra le varie chiamate del server
        static int i_disp = 0;

        // Se index è 0 significa che è una nuova richiesta HTTP: azzeriamo il contatore
        if (index == 0) {
            i_disp = 0;
        }

        String chunk = "";

        // Cicla l'elenco dei dispositivi finché c'è spazio nel buffer di rete corrente
        while (i_disp < MY_ENTITA && chunk.length() < (maxLen - 128)) {
            
            // Filtro: processa solo gli slot realmente occupati
            if ((sistema_scs.elenco[i_disp].id != 0) && (sistema_scs.elenco[i_disp].valido == 1)) {
                
				String tipo_str = codifica_codice_a_tipo(sistema_scs.elenco[i_disp].tipo_codice, 1); 
                // Costruisce la riga CSV identica a prima: id,address,tipo,descrizione,valore
                chunk += String(i_disp) + "," + 
                         String(sistema_scs.elenco[i_disp].address) + "," + 
                         tipo_str + "," + 
                         String(sistema_scs.elenco[i_disp].descrizione) + "," + 
                         String(sistema_scs.elenco[i_disp].valore_max) + "\n";
            }
            i_disp++;
        }

        // Se ci sono dati nel chunk attuale, copiali nel buffer dell'interfaccia di rete
        if (chunk.length() > 0) {
            memcpy(buffer, chunk.c_str(), chunk.length());
            return chunk.length();
        }

        return 0; // Fine dello streaming dei dati CSV
    });

    request->send(response);
});


      // Ricezione dati tramite POST Form standard e salvataggio in RAM
    a_server.on("/add", HTTP_POST, [](AsyncWebServerRequest *request){
      // Legge l'ID inviato dal form (-1 = NUOVO, >=0 = MODIFICA)
      int idModifica = request->hasParam("id", true) ? request->getParam("id", true)->value().toInt() : -1;
      int targetSlot = -1;

      // 1. Estrae i parametri dai campi di testo del form
      String addr = request->hasParam("address", true) ? request->getParam("address", true)->value() : "";
      String tipo = request->hasParam("tipo", true) ? request->getParam("tipo", true)->value() : "";
      String desc = request->hasParam("desc", true) ? request->getParam("desc", true)->value() : "";
      String val  = request->hasParam("valore", true) ? request->getParam("valore", true)->value() : "";

      // Rimuove eventuali spazi bianchi prima e dopo per evitare falsi duplicati (es. " 1" vs "1")
      addr.trim();

      // 2. CONTROLLO UNICITÀ DELL'ADDRESS
      // Cerchiamo se l'indirizzo inserito esiste già in memoria
      for(int i = 0; i < MY_ENTITA; i++) {
        if ((sistema_scs.elenco[i].valido == 1) && addr.equals(String(sistema_scs.elenco[i].address))) {
          // Se stiamo inserendo un NUOVO elemento, blocca subito l'azione
          // Se stiamo MODIFICANDO, blocca solo se il duplicato si trova in uno slot DIVERSO da quello che stiamo modificando

          if(idModifica == -1 || idModifica != i) {
			request->send(400, "text/plain", "L'indirizzo (Address) inserito è già presente in lista!");
            return;
          }
        }
      } // end for

      // 3. DETERMINAZIONE DELLO SLOT DI DESTINAZIONE
      if (idModifica == -1) 
      {
        // NUOVO INSERIMENTO
//        if(conteggioElementi >= MY_ENTITA) {
//			request->send(400, "text/plain", "ID non valido o spazio in memoria esaurito (Max 128 dispositivi).");
//			return;
//        }

        // Trova il primo slot vuoto
        for(int i = 0; i < MY_ENTITA; i++) 
        {
          if(sistema_scs.elenco[i].valido == 0) 
          {
            targetSlot = i;
            break;
          }
        } // end for
      } else {	// else idModifica
        // MODIFICA DI UN ELEMENTO ESISTENTE
        if (idModifica >= 0 && idModifica < MY_ENTITA && (sistema_scs.elenco[idModifica].valido == 1)) {

          targetSlot = idModifica;            // <-----------------------------------
          sistema_scs.elenco[targetSlot].bits.modificata = 1;
          uint8_t addr_num = sistema_scs.elenco[targetSlot].id;
          // Toglie la posizione i dell'array a quel byte indice
          tabella_indice[addr_num].pointer = -1;
        }
      } // end idModifica



      // 4. SALVATAGGIO DEI DATI IN RAM
      if(targetSlot != -1) {
        char temp[28];

		// Copia protetta nei buffer di array di caratteri (C-strings)

        strncpy(temp,addr.c_str(), 2);
        temp[2] = '\0'; // Garantisce la terminazione corretta del buffer di 3 byte
        
        uint8_t addr_num = (uint8_t)strtol(temp, NULL, 16);
        sistema_scs.elenco[targetSlot].id = addr_num; // address

        // Aggiunge la posizione i dell'array a quel byte indice
        tabella_indice[addr_num].pointer = targetSlot;
        if (strcmp(sistema_scs.elenco[targetSlot].address, temp) != 0)
        {
            sistema_scs.elenco[targetSlot].bits.chiave = 1;
            strcpy(sistema_scs.elenco[targetSlot].address, temp);
        }

        strncpy(temp,tipo.c_str(), 3);
        temp[3] = '\0'; // Garantisce la terminazione corretta del buffer
        if (sistema_scs.elenco[targetSlot].tipo_codice != decodifica_tipo_a_codice(temp,sistema_scs.tapparelle_percentuale));
        {
            sistema_scs.elenco[targetSlot].bits.tipo = 1;
            sistema_scs.elenco[targetSlot].tipo_codice = decodifica_tipo_a_codice(temp,sistema_scs.tapparelle_percentuale);
        }

        strncpy(temp,desc.c_str(), 25);
        temp[25] = '\0'; // Garantisce la terminazione corretta del buffer
        if (strcmp(sistema_scs.elenco[targetSlot].descrizione, temp) != 0)
        {
            sistema_scs.elenco[targetSlot].bits.descrizione = 1;
            strcpy(sistema_scs.elenco[targetSlot].descrizione, temp);
        }

        strncpy(temp,val.c_str(), 3);
        temp[3] = '\0'; // Garantisce la terminazione corretta del buffer
        if (strcmp(sistema_scs.elenco[targetSlot].valore_max, temp) != 0)
        {
            sistema_scs.elenco[targetSlot].bits.valore = 1;
            strcpy(sistema_scs.elenco[targetSlot].valore_max, temp);
        }
      
        if (idModifica == -1)   // nuovo
            set_device_default(targetSlot, 1);


#ifdef DEBUG_EVENTITY
  char buflog[120];
  char binlog[9];
  for (unsigned i = 0; i < 8; ++i) binlog[i] = 0x30 | ((sistema_scs.elenco[targetSlot].bits.Val >> (7 - i)) & 0x01);
  binlog[8] = 0;
  sprintf(buflog,"[%d] [CMD] xxx: %s %s %s : 0b%s, [%d]",targetSlot, sistema_scs.elenco[targetSlot].address, sistema_scs.elenco[targetSlot].tipo, sistema_scs.elenco[targetSlot].descrizione, binlog, targetSlot);
  events.send(buflog, "log", millis());
#endif
        
        request->redirect("/dispositivi");
      } else {  //  // else targeSlot SALVATAGGIO IN RAM

//        request->send(400, "text/plain", "Errore: ID non valido o slot non allocabile.");

		  String htmlErrore = "<html><head><script>"
							  "alert('Errore: ID non valido o slot non allocabile.');"
							  "window.history.back();"
							  "</script></head><body></body></html>";
		  request->send(200, "text/html", htmlErrore);	  
	  
	  }
    });

      // Cancellazione di un elemento tramite ID (Indice dell'array)
    a_server.on("/delete", HTTP_POST, [](AsyncWebServerRequest *request){
        if(request->hasParam("id")) {
          int i_disp = request->getParam("id")->value().toInt();

          if(i_disp >= 0 && i_disp < MY_ENTITA && (sistema_scs.elenco[i_disp].valido == 1)) {
            sistema_scs.elenco[i_disp].valido = 0;
            sistema_scs.elenco[i_disp].bits.cancellata = 1;
///			rigenera_tabella_indice();
            uint8_t addr_num = sistema_scs.elenco[i_disp].id;
            // Toglie la posizione i dell'array a quel byte indice
            tabella_indice[addr_num].pointer = -1;

#ifdef DEBUG_EVENTITY
    char buflog[120];
    char binlog[9];
    for (unsigned i = 0; i < 8; ++i) binlog[i] = 0x30 | ((sistema_scs.elenco[i_disp].bits.Val >> (7 - i)) & 0x01);
    binlog[8] = 0;
    sprintf(buflog,"[%d] [CMD] dxx: %s %s %s : 0b%s, [%d]",id, sistema_scs.elenco[id].address, sistema_scs.elenco[id].tipo, sistema_scs.elenco[id].descrizione, binlog, id);
    events.send(buflog, "log", millis());
#endif
            request->send(200, "text/plain", "OK");
            return;
          }
        }
        request->send(400, "text/plain", "ID Non valido");
      });


  // Endpoint per la memorizzazione che riceve in input lo stato della checkbox
  a_server.on("/refresh", HTTP_POST, [](AsyncWebServerRequest *request){
    // Legge il parametro inviato in input dall'URL della richiesta POST
    if(request->hasParam("tapparelle")) {
      String val = request->getParam("tapparelle")->value();
      sistema_scs.tapparelle_percentuale = (val == "1");
      uploadDevices(sistema_scs.tapparelle_percentuale);

   //      Serial.printf("Pulsante Memorizza premuto. Stato checkbox ricevuto: %s\n", sistema_scs.tapparelle_percentuale ? "ATTIVO" : "DISATTIVO");

      flagRefreshDevices = 1;
    }

    // Risposta di conferma per il browser
    request->send(200, "text/plain", "OK");
  });

  // Endpoint per dire al browser se la checkbox deve essere attiva o no
  a_server.on("/get-config", HTTP_GET, [](AsyncWebServerRequest *request){
    String stato = sistema_scs.tapparelle_percentuale ? "1" : "0";
    request->send(200, "text/plain", stato);
  });


#endif


#ifdef MY_DOMOTIC

    a_server.on("/sseha", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", index_html_HA);
    });



a_server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    
    // Creiamo una risposta in streaming con Content-Type JSON
    AsyncWebServerResponse *response = request->beginChunkedResponse("application/json", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        
        // Struttura statica per tracciare lo stato dell'invio tra un chunk e l'altro
        static int fase = 0; // 0 = Inizio, 1 = Dispositivi, 2 = Info, 3 = Fine
        static int i_entita = 0;
        static int i_info = 0;

        // Se l'indice è 0, resettiamo i contatori per una nuova richiesta HTTP
        if (index == 0) {
            fase = 0;
            i_entita = 0;
            i_info = 0;
        }

        String chunk = "";

        // FASE 0: Inizio del JSON
        if (fase == 0) {
            chunk += "[";
            fase = 1; // Passa alla scansione dei dispositivi
        }

        // FASE 1: Invio dei dispositivi (Adattato alla tua nuova struttura 'sistema_scs.elenco')
        if (fase == 1) {
            while (i_entita < MY_ENTITA && chunk.length() < (maxLen - 256)) {
                // Filtro: invia solo i dispositivi validi/censiti
                if ((sistema_scs.elenco[i_entita].id != 0) && (sistema_scs.elenco[i_entita].valido == 1) 
                && (sistema_scs.elenco[i_entita].tipo_codice != GENERICO) && (sistema_scs.elenco[i_entita].tipo_codice != ALLARME))
                {    
                    if (i_entita > 0 || index > 1) chunk += ",";
                    int8_t hwtype = tipo_to_hwType(sistema_scs.elenco[i_entita].tipo_codice);
                    chunk += "{";
                    chunk += "\"id\":\"" + String(sistema_scs.elenco[i_entita].address) + "\",";
                    chunk += "\"nome\":\"" + String(sistema_scs.elenco[i_entita].descrizione) + "\",";
                    chunk += "\"tipo\":" + String(hwtype) + ",";
                    
                    if (hwtype == ENTI_TAPPARELLA_PCT) { 
                        chunk += "\"stato\":\"" + String(sistema_scs.elenco[i_entita].valore_corrente) + "|" + String(sistema_scs.elenco[i_entita].stato) + "\"";
                    } else if (hwtype == ENTI_SLIDER) { 
                        chunk += "\"stato\":\"" + String(sistema_scs.elenco[i_entita].stato) + "|" + String(sistema_scs.elenco[i_entita].valore_corrente) + "\"";
                    } else {
                        chunk += "\"stato\":\"" + String(sistema_scs.elenco[i_entita].stato) + "\"";
                    }
                    chunk += "}";
                }
                i_entita++;
            }
            
            if (i_entita >= MY_ENTITA) {
                fase = 2; // Dispositivi finiti, passa alle Info
            }
        }

        // FASE 2: Invio delle Card Info (Adattato a 'sistema_scs.cardinfo')
        if (fase == 2) {
            while (i_info < numeroInfo && chunk.length() < (maxLen - 256)) {
                if (cardinfo[i_info].id != -1) {
                    
                    // Aggiunge la virgola se abbiamo già inviato qualcosa (o dispositivi o info precedenti)
                    chunk += ",";

                    char info_combinato[128] = {0};
                    snprintf(info_combinato, sizeof(info_combinato), "%s|%s|%s|%s|%s",
                             cardinfo[i_info].info_0, 
                             cardinfo[i_info].info_1, 
                             cardinfo[i_info].info_2, 
                             cardinfo[i_info].info_3,
                             cardinfo[i_info].info_4);

                    // Trova l'indice del dispositivo associato per prenderne il nome
//                    int p_id = sistema_scs.cardinfo[i_info].id;
                    int p_id = cardinfo[i_info].id;
                    String nome_disp = "Info";
                    nome_disp += String(sistema_scs.elenco[p_id].descrizione);
                    String id_disp = String(sistema_scs.elenco[p_id].address);

                    chunk += "{";
                    chunk += "\"id\":\"" + id_disp + "_INF\",";
                    chunk += "\"nome\":\"" + nome_disp + " (info)\",";
                    chunk += "\"tipo\":5,";
                    chunk += "\"stato\":\"" + String(info_combinato) + "\"";
                    chunk += "}";
                }
                i_info++;
            }
            
            if (i_info >= numeroInfo) {
                fase = 3; // Info finite, chiudi l'array
            }
        }

        // FASE 3: Chiusura finale del JSON
        if (fase == 3) {
            chunk += "]";
            fase = 4; // Fine della trasmissione
// mio log
//          events.send(chunk.c_str(), "log", millis());
        }

        // Copia il chunk generato nel buffer di rete dell'ESP32
        if (chunk.length() > 0) {
            memcpy(buffer, chunk.c_str(), chunk.length());
            return chunk.length();
        }

        return 0; // Trasmissione completata con successo
    });

    request->send(response);
});







    // 3. Endpoint API universale per i pulsanti
    a_server.on("/api/toggle", HTTP_POST, [](AsyncWebServerRequest *request) {

// SWITCH_SET
        if (request->hasParam("id")) {
            unsigned char command = 0;
            unsigned char device = 0;
            String idRichiesto = request->getParam("id")->value();

            uint8_t i = CercaAddress(idRichiesto.c_str());
            if (i != 0xFF)   
            {
//                if ((sistema_scs.elenco[i].id == idRichiesto) && 
                if  ((tipo_to_hwType(sistema_scs.elenco[i].tipo_codice) == ENTI_INTERRUTTORE) || (tipo_to_hwType(sistema_scs.elenco[i].tipo_codice) == ENTI_SLIDER))
                {

                  // Toggle dello stato dell'entità - comando SCS <******************************************************+

                    if (strcmp(sistema_scs.elenco[i].stato, "ON") == 0) {
                        strcpy(sistema_scs.elenco[i].stato, "OFF");
                    } else {
                        strcpy(sistema_scs.elenco[i].stato, "ON");
                    }
                    
                    device = (uint8_t)strtol(sistema_scs.elenco[i].address, NULL, 16);
                    if (strcmp(sistema_scs.elenco[i].stato, "ON") == 0)
                      command = 0x00;
                    else 
                      command = 0x01;
                    requestLen = 0;
                    requestBuffer[requestLen++] = PIC_REQ;
                    requestBuffer[requestLen++] = 'y';    // 0x79 (@y: invia a pic da MQTT cmd standard da inviare sul bus)
                    requestBuffer[requestLen++] = device; // to   device
                    requestBuffer[requestLen++] = 0x00;   // from device
                    requestBuffer[requestLen++] = 0x12;   // type:command
                    requestBuffer[requestLen++] = command;// command

                    // Notifica SSE usando l'ID entità come evento
                    eventity.send(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address, millis());
                }
            }
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Bad Request");
        }
    });



    // LIGHT_SET
    // 3. Endpoint API universale per i dimmer (slider)
    a_server.on("/api/set", HTTP_POST, [](AsyncWebServerRequest *request) {
      if (request->hasParam("id") && request->hasParam("val")) {
          String idRichiesto = request->getParam("id")->value();
          String valoreRicevuto = request->getParam("val")->value();
          unsigned char command = 0;
          unsigned char device = 0;
          
          uint8_t i = CercaAddress(idRichiesto.c_str());
          if (i != 0xFF)   
          {
			  if  ((tipo_to_hwType(sistema_scs.elenco[i].tipo_codice) == ENTI_SLIDER)) {
                  device = (uint8_t)strtol(sistema_scs.elenco[i].address, NULL, 16);
                  
                // comando SCS <******************************************************+
                  command = 0x00;  // on/up
                  int pct = valoreRicevuto.toInt();    // percentuale
                  strcpy(sistema_scs.elenco[i].valore_corrente, valoreRicevuto.c_str());
                // --------------- percentuale da 1 a 255 (home assistant) -----------
        //        if ((domoticMode == 'h') || (domoticMode == 'H'))   // h=as homeassistant
        //        { // percentuale 0-255
        //          pct *= 100;                  // da 0 a 25500
        //          pct /= 255;                  // da 0 a 100
        //        }
                // --------------- percentuale da 1 a 100 (domoticz) -----------------
                  // percentuale 0-100
                  pct += 5;                    // arrotondamento
                  pct /= 10;                   // 0-10
                  if (pct > 9) pct = 9;
                  if (pct == 0) pct = 1;		    // 1-9
                  pct *= 16;                   // hex high nibble
                  pct += 0x0D;                 // hex low  nibble
                  command = (unsigned char) pct;

                  requestLen = 0;
                  requestBuffer[requestLen++] = PIC_REQ;
                  requestBuffer[requestLen++] = 'y';    // 0x79 (@y: invia a pic da MQTT cmd standard da inviare sul bus)
                  requestBuffer[requestLen++] = device; // to   device
                  requestBuffer[requestLen++] = 0x00;   // from device
                  requestBuffer[requestLen++] = 0x12;   // type:command
                  requestBuffer[requestLen++] = command;// command

                  // Notifica istantaneamente via SSE tutti gli altri browser connessi
                  eventity.send(String(sistema_scs.elenco[i].valore_corrente) + sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address, millis());
              }
          }
          request->send(200, "text/plain", "OK");
      } else {
          request->send(400, "text/plain", "Bad Request");
      }
  });

// COVER_SET
    // 3. Endpoint API universale per tapparelle (up/down/stop)
      a_server.on("/api/tapparella", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("id") && request->hasParam("cmd")) {
        String idRichiesto = request->getParam("id")->value();
        String comando = request->getParam("cmd")->value(); // "SALITA", "DISCESA", "STOP"
        unsigned char command = 0;
        unsigned char device = 0;
        
        uint8_t i = CercaAddress(idRichiesto.c_str());
        if (i != 0xFF)   
        {
			  if  ((tipo_to_hwType(sistema_scs.elenco[i].tipo_codice) == ENTI_TAPPARELLA)) 
			  {
                device = (uint8_t)strtol(sistema_scs.elenco[i].address, NULL, 16);
                if (comando == "SALITA") //  && sistema_scs.elenco[i].pin != -1) 
                {
                    strcpy (sistema_scs.elenco[i].stato, "SALITA");
                    command = 0x08;
                } 
                else if (comando == "DISCESA") // && sistema_scs.elenco[i].pin2 != -1) 
                {
                    strcpy (sistema_scs.elenco[i].stato, "DISCESA");
                    command = 0x09;
                } 
                else 
                {
                    strcpy (sistema_scs.elenco[i].stato, "STOP");
                    command = 0x0A;
                }
                requestLen = 0;
                requestBuffer[requestLen++] = PIC_REQ;
                requestBuffer[requestLen++] = 'y';    // 0x79 (@y: invia a pic da MQTT cmd standard da inviare sul bus)
                requestBuffer[requestLen++] = device; // to   device
                requestBuffer[requestLen++] = 0x00;   // from device
                requestBuffer[requestLen++] = 0x12;   // type:command
                requestBuffer[requestLen++] = command;// command

                // Invia l'aggiornamento SSE alla pagina Home
                eventity.send(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address, millis());
            }
        }
        request->send(200, "text/plain", "OK");
    } else {
        request->send(400, "text/plain", "Bad Request");
    }
});


// COVERPCT_SET
    // 3. Endpoint API universale per tapparelle_pct (up/down/stop/%)
      a_server.on("/api/tapparella_pct", HTTP_POST, [](AsyncWebServerRequest *request) {
      if (request->hasParam("id") && request->hasParam("target")) {
        String idRichiesto = request->getParam("id")->value();
        String comando = request->getParam("target")->value(); // "SALITA", "DISCESA", "STOP", %
        unsigned char command = 0;  
        unsigned char device = 0;
        
        uint8_t i = CercaAddress(idRichiesto.c_str());
        if (i != 0xFF)   
        {
            if  (tipo_to_hwType(sistema_scs.elenco[i].tipo_codice) == ENTI_TAPPARELLA_PCT) 
			{
                device = (uint8_t)strtol(sistema_scs.elenco[i].address, NULL, 16);
// 0=stop   1=open    2=close   xxx=pct
                if (comando == "SALITA") //  && sistema_scs.elenco[i].pin != -1) 
                {
                    strcpy (sistema_scs.elenco[i].stato, "SALITA");
                    command = 0x08;
                } 
                else if (comando == "DISCESA") // && sistema_scs.elenco[i].pin2 != -1) 
                {
                    strcpy (sistema_scs.elenco[i].stato, "DISCESA");
                    command = 0x09;
                } 
                else if (comando == "STOP") // && sistema_scs.elenco[i].pin2 != -1) 
                {
                    strcpy (sistema_scs.elenco[i].stato, "STOP");
                    command = 0x0A;
                } 
                else 
                {
                    int pct = comando.toInt();    // percentuale
                    if (pct == 0)
                    {
                    strcpy (sistema_scs.elenco[i].stato, "DISCESA");
                    }
                    else
                    if (pct == 100)
                    {
                    strcpy (sistema_scs.elenco[i].stato, "DISCESA");
                    }
                    requestBuffer[requestLen++] = PIC_REQ;
                    requestBuffer[requestLen++] = 'u';
                    requestBuffer[requestLen++] = device;		// to   device
                    requestBuffer[requestLen++] = (unsigned char) pct;    // command (%)
                    String payload = String(comando) + "|" + String(sistema_scs.elenco[i].stato);
                    eventity.send(payload.c_str(), sistema_scs.elenco[i].address, millis());

#ifdef DEBUG_EVENTITY
          String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + sistema_scs.elenco[i].id + " : " + payload;
          events.send(rispostaLog.c_str(), "log", millis());
#endif
                }
                if (command != 0)
                {
                    requestLen = 0;
                    requestBuffer[requestLen++] = PIC_REQ;
                    requestBuffer[requestLen++] = 'y';    // 0x79 (@y: invia a pic da MQTT cmd standard da inviare sul bus)
                    requestBuffer[requestLen++] = device; // to   device
                    requestBuffer[requestLen++] = 0x00;   // from device
                    requestBuffer[requestLen++] = 0x12;   // type:command
                    requestBuffer[requestLen++] = command;// command
                    // Invia l'aggiornamento SSE alla pagina Home
                    eventity.send(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address, millis());
                }
#ifdef DEBUG_EVENTITY
          String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + sistema_scs.elenco[i].address + " : " + sistema_scs.elenco[i].stato;
          events.send(rispostaLog.c_str(), "log", millis());
#endif
            }
        }
        request->send(200, "text/plain", "OK");
    } else {
        request->send(400, "text/plain", "Bad Request");
    }
});

    a_server.addHandler(&eventity);    

#endif







    // GESTIONE DEL FORM DI INPUT (POST)
    a_server.on("/invia", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasParam("testo", true)) {
            uint8_t result = 0; // 1=eseguito   2=a pic   3=rifiutato
            String comandoRicevuto = request->getParam("testo", true)->value();

			rimuovi_spazi_string(comandoRicevuto);
			if (comandoRicevuto[0] == '*')
				comandoRicevuto[0] = '@';
            
            // Stampa sulla seriale dell'ESP32 il comando ricevuto
//            Serial.printf("Comando utente ricevuto: %s\n", comandoRicevuto.c_str());
            
            // Genera immediatamente un log confermando la ricezione
//            String rispostaLog = "[" + String(millis()/100) + "s] [CMD] Eseguito: " + comandoRicevuto;
//            events.send(rispostaLog.c_str(), "log", millis());

			// Esempio di invio dello stato (Max 32 caratteri gestiti in sicurezza lato client)
//			events.send("In funzione - OK", "stato", millis());

		    prev_internal = 3;
            if ((comandoRicevuto == "@MA") || (comandoRicevuto == "@o"))
            {
				internal = 0;
            }

            if ((comandoRicevuto == "stop") || (comandoRicevuto == "STOP"))
            {
				autocensimento = false;
                events.send("stop", "chiudi_connessione");
                setFirst();
				internal = 1;
                result = 1;
//				request->send(200, "text/plain", "OK");
            }
            else
            if (comandoRicevuto == "auto")
			{
	            events.send("Auto censimento attivo - 'off'  per terminare ", "log", millis());
				autocensimento = true;
                setFirst();
				internal = 1;
                result = 1;
//				request->send(200, "text/plain", "OK");
			}
			else
            if (comandoRicevuto == "off")
			{
	            events.send("Auto censimento terminato ", "log", millis());
				autocensimento = false;
                result = 1;
//				request->send(200, "text/plain", "OK");
			}
			else
            if (comandoRicevuto == "more")
			{
	            events.send("Media potenza wifi ", "log", millis());
                result = 1;
                gestisciPotenzaDinamica(2);
			}
			else
            if (comandoRicevuto == "high")
			{
	            events.send("Top potenza wifi ", "log", millis());
                result = 1;
                gestisciPotenzaDinamica(3);
			}
			else
            if (comandoRicevuto == "less")
			{
	            events.send("Bassa potenza wifi ", "log", millis());
                result = 1;
                gestisciPotenzaDinamica(1);
			}
			else
            if (comandoRicevuto == "display")
			{
    // Creiamo un buffer grande per contenere molte righe di testo insieme
    // 4096 byte bastano per circa 30-40 dispositivi a blocco
                char *grandi_log = (char *)malloc(4096);
                if (grandi_log != NULL) {
                    grandi_log[0] = '\0'; // Inizializza la stringa vuota
                    int contatore_righe = 0;

                    for (int i=0; i<MY_ENTITA; i++)
                    {
                       if (sistema_scs.elenco[i].valido)
                       {
                          char buflog[128];
                          String tipo_str = codifica_codice_a_tipo(sistema_scs.elenco[i].tipo_codice); 
                          
                          if (alexaParam == 'y') {
                              snprintf(buflog, sizeof(buflog), "[%d] [scs] %d, id: %02X, %s, %s, st: %s, AX=%02X\n",
                                (millis()/100), i, sistema_scs.elenco[i].id, tipo_str.c_str(), 
                                sistema_scs.elenco[i].descrizione, sistema_scs.elenco[i].stato, sistema_scs.elenco[i].alexa_id);
                          } else {
                              snprintf(buflog, sizeof(buflog), "[%d] [scs] %d, id: %02X, %s, %s, st: %s\n",
                                (millis()/100), i, sistema_scs.elenco[i].id, tipo_str.c_str(), 
                                sistema_scs.elenco[i].descrizione, sistema_scs.elenco[i].stato);
                          }

                          // Concatena la riga nel mega buffer
                          strcat(grandi_log, buflog);
                          contatore_righe++;

                          // Per sicurezza, se accumuliamo 15 righe, le spediamo per non riempire il buffer da 4100 byte
                          if (contatore_righe >= 15) {
                              events.send(grandi_log, "log", millis());
                              grandi_log[0] = '\0'; // Svuota il buffer per le prossime righe
                              contatore_righe = 0;
                              vTaskDelay(pdMS_TO_TICKS(50)); // Qui il delay funziona perché la coda è libera
                          }
                       }
                    }

                    // Invia il residuo rimasto nel buffer se rimangono dispositivi
                    if (strlen(grandi_log) > 0) {
                        events.send(grandi_log, "log", millis());
                    }

                    free(grandi_log); // LIBERA SEMPRE LA MEMORIA!
                }

                result = 1;
//                request->send(200, "text/plain", "OK");
			}
			else
            if (comandoRicevuto == "mqttnew")
			{
              if (mqttopen == 3)
              {
                 char *grandi_log = (char *)malloc(4096);
                 if (grandi_log != NULL) 
                 {
                    grandi_log[0] = '\0'; // Inizializza la stringa vuota
                    int contatore_righe = 0;
                    size_t offset = 0;
                    for (int i=0; i<MY_ENTITA; i++)
                    {
                       if (sistema_scs.elenco[i].valido)
                       {
                          size_t added = MQTTnewdiscover(sistema_scs.elenco[i].tipo_codice, sistema_scs.elenco[i].address, sistema_scs.elenco[i].descrizione, grandi_log+offset, 4096-offset);
                          contatore_righe++;
                          offset += added;
                          // Per sicurezza, se accumuliamo 15 righe, le spediamo per non riempire il buffer da 4100 byte
                          if (contatore_righe >= 15) {
                              events.send(grandi_log, "log", millis());
                              grandi_log[0] = '\0'; // Svuota il buffer per le prossime righe
                              contatore_righe = 0;
                              offset = 0;
                              vTaskDelay(pdMS_TO_TICKS(50)); // Qui il delay funziona perché la coda è libera
                          }
                       }
                    }
                    result = 1;
                    // Invia il residuo rimasto nel buffer se rimangono dispositivi
                    if (strlen(grandi_log) > 0) {
                        events.send(grandi_log, "log", millis());
                    }

                    free(grandi_log); // LIBERA SEMPRE LA MEMORIA!

                }  // log != NULL


              } // mqttopen
              else
              {
    	         events.send("MQTT non è attivo ", "log", millis());
                 result = 3;
              } // mqttopen
			} // mqttnew
			else
            if ((comandoRicevuto == "internal") || (comandoRicevuto == "server"))
            {
                events.send("ripristino","log", millis());
                setFirst();
				internal = 1;
                delay(5);
                result = 1;
//				request->send(200, "text/plain", "OK");
            }
            else
            if ((comandoRicevuto == "reload") || (comandoRicevuto == "RELOAD"))
            {
#ifdef MY_TABELLA
			  inizializza_e_leggi_dispositivi_preferences();
              events.send("Entità ricaricate - fai refresh pagina /dispositivi ", "log", millis());
              delay(5);
#endif
              result = 1;
// 			  request->send(200, "text/plain", "OK");
            }
            else
    		if ((comandoRicevuto[0] == '@') || (comandoRicevuto[0] == PIC_REQ))
            {
	            requestLen = comandoRicevuto.length();
		        comandoRicevuto.toCharArray(requestBuffer, requestLen+1);
			    immediateSend();
                result = 2;
//				request->send(200, "text/plain", "OK");
            }
            else
            {
              result = 4;
//                request->send(200, "text/plain", "Comando sconosciuto");
            }
            if (result == 1)
            {
                String rispostaLog = "[" + String(millis()/100) + "s] [CMD] Eseguito: " + comandoRicevuto;
                events.send(rispostaLog.c_str(), "log", millis());
            }
            else
            if (result == 2)
            {
                String rispostaLog = "[" + String(millis()/100) + "s] [CMD] pic: " + comandoRicevuto;
                events.send(rispostaLog.c_str(), "log", millis());
            }
            else
            if (result == 3)
            {
                String rispostaLog = "[" + String(millis()/100) + "s] [CMD] Rifiutato: " + comandoRicevuto;
                events.send(rispostaLog.c_str(), "log", millis());
            }
            else
            if (result == 4)
            {
                String rispostaLog = "[" + String(millis()/100) + "s] [CMD] Sconosciuto: " + comandoRicevuto;
                events.send(rispostaLog.c_str(), "log", millis());
            }
			request->send(200, "text/plain", "OK");

//            else
//				request->send(200, "text/plain", "KO");

		} else {
            request->send(400, "text/plain", "Parametro mancante");
        }

    });



// =============================================================================================
    // Rotta Principale: serve la pagina HTML di controllo
    a_server.on("/sselog", HTTP_GET, [](AsyncWebServerRequest *request){
        setFirst();
		internal = 1;
        request->send(200, "text/html", index_html_LOG);
		prev_internal = 5;
        delay_internal = 300;
    });

    // Registrazione del gestore SSE sul server
    a_server.addHandler(&events);    
// =============================================================================================



// =============================================================================================
    a_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
// =============================================================================================
      content = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION " at ";
      content += WiFi.localIP().toString();
      content += "<p><ol>";
      content += "<li>/scan      (query connection)</li>";
//    content += "<li>/setting   (setup connection)</li>"; - NASCOSTO
      content += "<li>/status    (display status)</li>";
      content += "<li>/help      (PIC status)</li>";
      content += "<li>/reset     (device=ESP | PIC)</li>";
#ifdef FFS
      content += "<li>/picprog   (verify/start PIC firmware programming)</li>";
#endif
      content += "<li>/request   (command request)</li>";
      content += "<li>/gate      (command request)</li>";        
      content += "<li>/callback  (setup http callback)</li>";
//    content += "<li>/backsetting  (setup http callback)</li>";  - NASCOSTO
      content += "<li>/mqttconfig (setup mqtt broker)</li>";
//      content += "<li>/mqttcfg   (setup mqtt broker)</li>";     - NASCOSTO
//      content += "<li>/mqttdevices (mqtt devices discover)</li>";
//      content += "<li>/devicename (mqtt devices names)</li>";
      content += "<li>/sselog      (SCS LOG)</li>";
      content += "<li>/dispositivi (CENSIMENTO)</li>";
      content += "<li>/sseha       (DOMOTIC)</li>";
      content += "</ol>";
      content += "</form></html>";
      request->send(200, "text/html", content);
      content = "";
    });
// =============================================================================================
  }
  else
  {
// =============================================================================================
    a_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
   {
// =============================================================================================
      scanProcess();
      request->send(200, "text/html", content);
      content = "";
    });
// =============================================================================================
  }


// =============================================================================================
    a_server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request)
   {
// =============================================================================================
      scanProcess();
      request->send(200, "text/html", content);
      content = "";
    });
// =============================================================================================



// =============================================================================================
    a_server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
   {
// =============================================================================================
      setFirst();
      internal = 1;

      String param = request->arg("device");

      if (param == "mqtt")
      {
        if (!client.connected())
          client.disconnect();
        mqttopen = 0;

        mqtt_retry = 0;
        if ((mqtt_server[0] >= '0') && (mqtt_server[0] <= '9'))
        {
          mqttopen = 1;
          client.setServer(mqtt_server, atoi(mqtt_port));    // Configure MQTT connexion
          client.setCallback(MqttCallback);                  // callback function to execute when a MQTT message
          content = "{\"status\":\"OK\"}";
        }
        else
        {
          content = "{\"status\":\"ERROR - missing broker IPaddress\"}";
        }
      }
      else if (param == "esp")
      {
        countRestart = 200;
        content = "{\"status\":\"OK - resetting...\"}";
      }
      else if (param == "pic")
      {
        uartSemaphor = 1;
        requestBuffer[requestLen++] = PIC_REQ;
//        requestBuffer[requestLen++] = '@';
        requestBuffer[requestLen++] = '|';
        requestBuffer[requestLen++] = '|';
        content = "{\"status\":\"OK\"}";
        uartSemaphor = 0;
      }
      else if (param == "all")
      {
        uartSemaphor = 1;
        requestBuffer[requestLen++] = PIC_REQ;
//        requestBuffer[requestLen++] = '@';
        requestBuffer[requestLen++] = '|';
        requestBuffer[requestLen++] = '|';
        uartSemaphor = 0;
        countRestart = 200;
        content = "{\"status\":\"OK - resetting...\"}";
      }
      else if (events.count() != 0)
      {
        content = "{\"status\":\"SSE closed\"}";
        if (events.count() != 0) 
        {
            events.send("stop", "chiudi_connessione");
        }
        delay(50); // Piccolo delay per dare tempo alla rete di trasmettere
        events.close();

      }
      else
      {
        content = "{\"invreq  use device=esp  or  device=pic  or  device=mqtt\"}";
      }
// =============================================================================================
      request->send(200, "text/html", content);
      content = "";
    });
// =============================================================================================





// =============================================================================================
    a_server.on("/setting", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
      String qsid = request->arg("ssid");
      String qpass = request->arg("pass");
      String qip = request->arg("ip");
      String qrip = request->arg("rip");
      String qport = request->arg("uport");

      local_ip.fromString(qip);
      router_ip.fromString(qrip);
      udpLocalPort = qport.toInt();
      if (udpLocalPort == 0)
      {
        udpLocalPort = 52056;
        qport = "52056";
      }

      WriteEEP(qsid, E_SSID);
      WriteEEP(qpass, E_PASSW);
      WriteEEP(qip, E_IPADDR);
      WriteEEP(qrip, E_ROUTIP);
      WriteEEP(qport, E_PORT);
      
      EEPROM.commit();

      content = "{\"Success\":\"saved to eeprom... reset to boot into new wifi\"}";
      statusCode = 200;
// =============================================================================================
      request->send(statusCode, "application/json", content);
      content = "";
    });
// =============================================================================================


// =============================================================================================
    a_server.on("/status", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
        if (firstTime == 0) setFirst();
        char temp[38];
        content = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION;
        content += " at ";
        
        content += WiFi.localIP().toString();


        content += "<pre style='background: #ffffff; padding: 10px; border: 1px solid #ccc; border-radius: 4px; color: #000000; font-family: monospace;'>";


//        content += "<p><ol>";

        if (connectionType == 1) // web server AP
        {
//          Serial1.write('@');   // set led lamps
          Serial1.write(PIC_REQ);   // set led lamps
          Serial1.write(0xF2);  // set led lamps high-freq (AP mode)
          content += "<li>";
          content += "Working as AP ";
          content += "</li>";
        }
        else
        {
//          Serial1.write('@');   // set led lamps
          Serial1.write(PIC_REQ);   // set led lamps
          Serial1.write(0xF1);  // set led lamps std-freq (client mode)
          content += "<li>";
          content += "System frequency (Mhz): ";
          unsigned char frq = ESP.getCpuFreqMHz(); // returns the CPU frequency in MHz as an unsigned 8-bit integer
          content += String(frq);
          content += "</li>";

          content += "<li>";
          content += "Wifi ssid: ";
          char hBuffer[64];
          sprintf(hBuffer, "%s - Ch:%d - (%ddBm)", WiFi.SSID().c_str(), WiFi.channel(), WiFi.RSSI());
          content += hBuffer;
          content += " - Router: ";
          content += WiFi.gatewayIP().toString();
          sprintf(hBuffer, " - Potenza Wi-Fi: %.2f dBm", TxPower * 0.25);
          content += hBuffer;
          sprintf(hBuffer, " - Sicurezza: ");
          content += hBuffer;
          content += getWiFiAuthMode();
          content += "</li>";
        }

        content += "<li>";
        if ((udpopen == 1) && (udpLocalPort > 0))
        {
          content += "UDP hearing on port ";
          content += String(udpLocalPort);
          if (udp_remote_ip)
          {
            content += " connected ip: ";
            content += udp_remote_ip.toString();
            content += " port ";
            content += String(udp_remote_port);
          }
          else
            content += " now unconnected";
        }
        else
          content += "UDP is closed ";
        content += "</li>";
        
      #ifdef USE_TCPSERVER
          content += "<li>";
          if (tcpopen)
          {
            content += "TCP hearing on port ";
            content += String(TCP_PORT);
            
            if (tcpclient && tcpclient.connected())
            {
                content += " connected ip: ";
                content += tcpclient.remoteIP().toString();
            }
          }
          else
            content += "TCP not opened ";

          content += "</li>";
      #endif

        content += "<li>";
        content += "SSE CLIENTS: ";
        content += String(events.count());

        char hBuf[4];
        content += " - Domotic options: ";
        sprintf(hBuf, "%02X", domotic_options);
        content += hBuf;
        content += "</li>";

//        if (ArduinoOTAflag > 0)
//        {
//          content += "<li>OTA update ready</li>";  
//        }
        content += "<li>";
        content += "Http callback setup = ";
        if (httpCallback[0] == ':')
          content += "YES";
        else
          content += "NO";
        content += "</li>";

        content += "<li>";
        if (mqttopen == 0)
          content += "MQTT connection is CLOSED ";
        else if (mqttopen == 1)
          content += "MQTT connection is WAITING ";
        else if (mqttopen == 2)
          content += "MQTT connection TRY to CONNECT ";
        else if (mqttopen == 3)
          content += "MQTT connection is OPEN ";


        if (alexaParam  == 'y')
        {
          content += "</li>";
          content += "<li>fauxmo ECHO DOT direct connection</li>";
          sprintf(temp, "<li>  %d devices added - ", id_fauxmo + 1);
          content += temp;
          sprintf(temp, "  %d devices discovered</li>", fauxmo.discovered());
          content += temp;
        }
        else
        {
          content += " - broker is: ";
          content += mqtt_server;
          content += "   port: ";
          content += mqtt_port;

          content += "  - ";
          content += " conn.retry: ";
          content += String(mqtt_connections,DEC);
          content += "</li>";
        }
//        content += "</li>";

        if (mqttopen > 0)
        {
          content += " - ";
          if (domoticMode == 'd')
            content += " mode: DOMOTICZ";
          else if (domoticMode == 'h')
            content += " mode: HOMEASSISTANT";
          else if (domoticMode == 'D')
            content += " mode: DOMOTICZ 1-2";
          else if (domoticMode == 'H')
            content += " mode: HOMEASSISTANT 1-2";
          else
            content += "mqtt model: GENERIC";
          content += "</li>";
        }

        if ((mqttopen > 0) || (alexaParam == 'y'))
        {
          content += "<li>known devices in eeprom: ";
          for (uint8_t i = 0; i < MY_ENTITA; i++)
          {
              if (sistema_scs.elenco[i].valido == 1)
              {
                  content += String(sistema_scs.elenco[i].address);
                  content += " ";
            }
          }
          content += "</li>";
        }

//        content += "<li>";
//        content += "ESP mode: " _MODO;
//        content += "</li>";

//        sprintf(temp, "<li>Free heap: %d bytes</li>", ESP.getFreeHeap());
//        content += temp;

        content += "<li>";
        content += "PIC fw version: ";
        content += picfwVersion;
        content += "</li>";

//        content += "<li>";
//        content += "INFO numero: ";
//        content += String(numeroInfo,DEC);
//        content += "</li>";

      // @Qr
//        Serial1.write(PIC_REQ);    // command mode 
//        Serial1.write('Q');    // command: query 
//        Serial1.write('r');    // query PIC reset nrs
//        delay(2);            // wait 2ms
//        char nrs = 0;
//        nrs = Serial1.read();        // receive from serial USB
//        content += "<li>";
//        content += "ESP resets: ";
//        content += String(nrs,DEC);
//        content += "</li>";

//        content += "<li>";
//        content += "END of report ";
//        content += "</li>";

//        content += "</ol>";



    content += "</pre>"; // Chiudiamo il blocco preformattato



    // --- 🟢 CHUNK 3: DIAGNOSTICA MEMORIA INTERNA ---
    uint32_t flash_size_bytes = 0;
    esp_flash_get_size(NULL, &flash_size_bytes);
    
    size_t total_sram = heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t watermark_sram = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t max_block_sram = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    char *p_buf = (char *)malloc(1024);

    // Nota i modificatori %zu per le variabili size_t e la correzione per la Flash
    int len = snprintf(p_buf, 1024,
        "- Flash Fisica Rilevata: %lu MB (%lu Byte)\r\n"
        "- RAM Interna Totale Heap...: %zu Byte\r\n"
        "- RAM Interna (SRAM) Libera : %zu Byte\r\n"
        "- RAM Interna Minima Storica: %zu Byte (Watermark)\r\n"
        "- RAM Interna Blocco Max    : %zu Byte (Allocabile contiguo)\r\n",
        (unsigned long)(flash_size_bytes / (1024 * 1024)), 
        (unsigned long)flash_size_bytes, 
        total_sram, 
        free_sram, 
        watermark_sram, 
        max_block_sram);

    content += "<h3>Mappa della memoria:</h3>";
//    content += "<pre style='background:#222; padding:10px; border-radius:4px; color:#fff;'>";
    content += "<pre style='background: #ffffff; padding: 10px; border: 1px solid #ccc; border-radius: 4px; color: #000000; font-family: monospace;'>";
	content += p_buf;
    content += "</pre>"; // Chiudiamo il blocco preformattato



    // --- 🟢 CHUNK 4: SCANSIONE DINAMICA DELLE PARTIZIONI ATTIVE ---
    // Apriamo un tag <pre> per mantenere gli spaziamenti e i caratteri a larghezza fissa (stile console)
    content += "<h3>Mappa delle Partizioni Flash:</h3>";
//    content += "<pre style='background:#222; padding:10px; border-radius:4px; color:#fff;'>";
    content += "<pre style='background: #ffffff; padding: 10px; border: 1px solid #ccc; border-radius: 4px; color: #000000; font-family: monospace;'>";

    // Otteniamo l'iteratore iniziale per tutte le partizioni
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    
    // Verifichiamo che l'iteratore sia valido e che il tuo buffer allocato con malloc esista
    if (it != NULL && p_buf != NULL) {
        
        while (it != NULL) {
            const esp_partition_t *part = esp_partition_get(it);
            
            if (part != NULL) {
                const char* tipo_str = (part->type == ESP_PARTITION_TYPE_APP) ? "APP " : "DATA";
                
                // Utilizziamo in sicurezza il tuo char *p_buf allocato tramite malloc(1024)
                snprintf(p_buf, 1024,
                    "  - Label: %-10s | Tipo: %s | Offset: 0x%08X | Dim: %7u Byte (%u KB)\n",
                    part->label, 
                    tipo_str, 
                    (unsigned int)part->address, 
                    (unsigned int)part->size, 
                    (unsigned int)(part->size / 1024));
                    
                // Accumuliamo i dati nella stringa HTML complessiva
                content += p_buf;
            }
            
            // Avanziamo l'iteratore alla partizione successiva
            it = esp_partition_next(it);
        }
        
        // NOTA: esp_partition_next rilascia automaticamente l'iteratore quando raggiunge la fine (diventando NULL).
        // Non serve chiamare esp_partition_iterator_release(it) se 'it' è già diventato NULL, 
        // poiché non farebbe nulla o potrebbe invalidare puntatori di sistema.
        
    } else {
        content += "  [Errore: Impossibile leggere la tabella delle partizioni o memoria insufficiente]\n";
    }
    content += "</pre>"; // Chiudiamo il blocco preformattato





    // --- 🟢 CHUNK 5: CALCOLO METRICHE FIRMWARE E CHIUSURA PAGINA ---
    const esp_partition_t *running_part = esp_ota_get_running_partition();
    
    uint32_t app_bin_size = 0;
    uint32_t free_partition_space = 0;
    float app_percentage_occupied = 0.0f;

    if (running_part != NULL) {
        esp_image_metadata_t img_data;
        const esp_partition_pos_t part_pos = {
            .offset = running_part->address,
            .size = running_part->size,
        };
        
        // Analizza i segmenti hardware e ottiene l'esatta lunghezza del firmware binario
        if (esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &part_pos, &img_data) == ESP_OK) {
            app_bin_size = img_data.image_len;
            free_partition_space = running_part->size - app_bin_size;
            app_percentage_occupied = ((float)app_bin_size / (float)running_part->size) * 100.0f;
        }
    }

    // Buffer locale temporaneo per formattare la sezione delle metriche
//    char metriche_buf[512];
    snprintf(p_buf, 1024,
        "<h3>Metriche Memoria Applicazione:</h3>"
        "<pre style='background:#222; padding:10px; border-radius:4px; color:#ffb300; font-family:monospace;'>"
        "- Dimensione Reale Firmware : %u Byte (%u KB) [%.2f%% della partizione]\n"
        "- Spazio Libero in Partizione: %u Byte (%u KB)\n"
        "</pre>",
        (unsigned int)app_bin_size,
        (unsigned int)(app_bin_size / 1024),
        app_percentage_occupied,
        (unsigned int)free_partition_space,
        (unsigned int)(free_partition_space / 1024)
    );


    // Accodiamo le metriche calcolate alla stringa principale della pagina
    content += p_buf;
    content += "</pre>"; // Chiudiamo il blocco preformattato








    free(p_buf);
    content += "</form></html>";
    request->send(200, "text/html", content);
    content = "";




// =============================================================================================
    });




// =============================================================================================
a_server.on("/help", HTTP_GET, [] (AsyncWebServerRequest *request) 
{
// =============================================================================================
    AsyncWebServerResponse *response = request->beginChunkedResponse("text/html", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t 
    {
      char temp[255];
      int bytesread = 1;
      String to_send = "";

      to_send = "<HTML><BODY>\r\nHello from ESP32_" _MODO "GATE " _FW_VERSION;
      to_send += " at ";
      to_send += WiFi.localIP().toString();
      to_send += "<p><ul>";

      if (connectionType == 1) // web server AP
      {
//        Serial1.write('@');   // set led lamps
        Serial1.write(PIC_REQ);   // set led lamps
        Serial1.write(0xF2);  // set led lamps high-freq (AP mode)
        to_send += "<li>";
        to_send += "Working as AP ";
        to_send += "</li>";
      }
      else
      {
//      Serial1.write('@');   // set led lamps
        Serial1.write(PIC_REQ);   // set led lamps
        Serial1.write(0xF1);  // set led lamps std-freq (client mode)
        to_send += "<li>";
        to_send += "System frequency (Mhz): ";
        unsigned char frq = ESP.getCpuFreqMHz(); // returns the CPU frequency in MHz as an unsigned 8-bit integer
        to_send += String(frq);
        to_send += "</li>";

        to_send += "<li>";
        to_send += "Wifi connection ssid: ";
        char hBuffer[64];
        sprintf(hBuffer, "%s - Ch:%d - (%ddBm)", WiFi.SSID().c_str(), WiFi.channel(), WiFi.RSSI());
        to_send += hBuffer;
        to_send += "</li>";

        to_send += "<li>";
        to_send += "Router IP: ";
        to_send += WiFi.gatewayIP().toString();
        to_send += "</li>";
      }
      to_send += "<li>---------PIC---------------</li>";

    // @QH
      Serial1.write(PIC_REQ);    // command mode 
      Serial1.write('Q');    // command: query 
      Serial1.write('H');    // query PIC fw version

      delay(5);            // wait 5ms
      char nrs = 0;

    //  while (bytesread> 0)
      while (nrs < 35)
      {
        temp[0] = '\0';
        bytesread = Serial1.readBytesUntil('\n', temp, 255);        // receive from serial USB
        if (bytesread > 1)
        {
          temp[bytesread] = '\0';
          to_send += "<li>";
          to_send += temp;
          to_send += "</li>";
        }
        delayMicroseconds(INNERWAIT);
//        delay(2);            // wait 2ms
        nrs++;
      }
      to_send += "</ul></p>";
      to_send += "</BODY></HTML>\r\n";

//      to_send.getBytes(buffer, maxLen); 
//      return to_send.length();

      size_t len = min(maxLen, to_send.length() - index);
//      Serial.printf("Sending %u bytes\n", len);
      memcpy(buffer, to_send.c_str() + index, len);
      return len;
    });

    request->send(response);
//    request->send("");
  });
// =============================================================================================




// =============================================================================================
    a_server.on("/request", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================

    IPAddress ip = WiFi.softAPIP();

    content = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION;
    content += "<p>";
    content += "</p><form method='get' action='gate'>";
    content += "<label>Req Type: </label><input name='type' maxlength=2 value='12' style='width:25px'>";
    content += "<label>From: </label><input name='from' maxlength=2 value='01' style='width:25px'>";
    content += "<label>to:</label><input name='to' maxlength=2 style='width:25px'>";
//    content += "<label>to:</label><input name='to' maxlength=4 style='width:50px'>";
    content += "<label>command</label><input name='cmd' maxlength=2 style='width:25px'>\
      <label>reply</label><input name='resp' maxlength=1 value='y' style='width:15px'>\
      <input type='submit'></form>";

    content += "</html>";
    if (firstTime == 0) setFirst();

    request->send(200, "text/html", content);
    content = "";
  });
// =============================================================================================






// =============================================================================================
    a_server.on("/gate", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
    if (firstTime == 0) setFirst();

    //    if(!server.authenticate(www_username, www_password))
    //        return server.requestAuthentication();

    String from;
    char  cfrom;
    String to;
    String type;
    char  ctype;
    char  cto;
    String cmd;
    char  ccmd;

//    WiFiClient client = request->client();
    //    if ((client.available()) && (client.connected()))
//    if (client.connected())
    {
  #ifdef DEBUG
      Serial.println("");
      Serial.print("New client:");
      Serial.print(client.remoteIP());
      Serial.print(" port:");
      Serial.println(client.remotePort());
  #endif
      tcp_remote_ip = request->url().toInt();
//      tcp_remote_ip = client.remoteIP();
//      tcp_remote_port = client.remotePort();
    }

    if (request->hasArg("type"))
    {
      type = request->arg("type");
      ctype = aConvert(type);
      if (ctype == 0) ctype = 0x12;
    }
    else
      ctype = 0x12;

    if (request->hasArg("from"))
    {
      from = request->arg("from");
      cfrom = aConvert(from);
    }
    else
      cfrom = 0x00;
    if (request->hasArg("to"))
    {
      to = request->arg("to");
      cto = aConvert(to);
    }
    else
      cto = 0x00;
    if (request->hasArg("cmd"))
    {
      cmd = request->arg("cmd");
      ccmd = aConvert(cmd);
    }
    else
      ccmd = 0x01;


    if (request->hasArg("resp"))
    {
      httpResp = request->arg("resp");
    }
    else
      httpResp = "";


  #ifdef DEBUG
    Serial.println(" ");
    Serial.print(" t=");
    if (server.hasArg("type"))
    {
      Serial.print(type);
      Serial.print(":");
    }
    Serial.println(ctype, HEX);

    Serial.print(" f=");
    if (server.hasArg("from"))
    {
      Serial.print(from);
      Serial.print(":");
    }
    Serial.println(cfrom, HEX);

    Serial.print(" a=");
    if (server.hasArg("to"))
    {
      Serial.print(to);
      Serial.print(":");
    }
    Serial.println(cto, HEX);

    Serial.print(" c=");
    if (server.hasArg("cmd"))
    {
      Serial.print(cmd);
      Serial.print(":");
    }
    Serial.println(ccmd, HEX);
  #endif
    {
      // tapparelle percentuale
      if (cfrom == 0xFE) // <================COVERPCT==================
      {
        uartSemaphor = 1;
        requestBuffer[requestLen++] = PIC_REQ;
        requestBuffer[requestLen++] = 'u';
        requestBuffer[requestLen++] = cto;
        uartSemaphor = 0;
      }
      else
      {
        uartSemaphor = 1;
        requestBuffer[requestLen++] = PIC_REQ;
        requestBuffer[requestLen++] = 'y';
        requestBuffer[requestLen++] = cto;
        requestBuffer[requestLen++] = cfrom;
        requestBuffer[requestLen++] = ctype;
        requestBuffer[requestLen++] = ccmd;
      }
      if ((httpResp == "y") || (httpResp == "i"))
      {
        content = "{\"status\":\"OK ";
        content += "\"}";
        statusCode = 200;
        request->send(statusCode, "text/html", content);
      }
    }
// =============================================================================================

    content = "";
  });
// =============================================================================================




// =============================================================================================
    a_server.on("/mqttconfig", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
        if (firstTime == 0) setFirst();
        IPAddress ip = WiFi.softAPIP();
        content = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION;
        content += "<p>";
        content += "</p><form method='get' action='mqttcfg'>\
          <label>MQTT broker: </label><input name='broker' maxlength=32 value='";
        content += String(mqtt_server);
        content += "' style='width:130px'>\
          <label>port: </label><input name='port' maxlength=5 value='";
        content += String(mqtt_port);
        content += "' style='width:50px'> ";

        content += "<label>user:</label><input name='user' maxlength=32 value='";
        content += String(mqtt_user);
        content += "' style='width:160px'>\
          <label>password: </label><input name='pswd' maxlength=20 value='";
        content += String(mqtt_password);
        content += "' style='width:160px'>\
          <label>domotic(h):</label><input name='dom' maxlength=1 value='";
        content += String(domoticMode);
        content += "' style='width:20px'>\
            <label>options(xx):</label><input name='dopt' maxlength=2 value='";
      //  content += String(domotic_options);
        char hBuf[4];
        sprintf(hBuf, "%02X", domotic_options);
        content += hBuf;
        content += "' style='width:20px'>\
            <label>log(y):</label><input name='log' maxlength=1 value='";
        content += String(mqtt_log);
        content += "' style='width:20px'>\
          <label>persistence(y):</label><input name='persistence' maxlength=1 value='";
        if (mqtt_persistence == 1)
          content += "y";
        else
          content += "n";
        content += "' style='width:20px'>\
          <label>alexa(y):</label><input name='alexa' maxlength=1 value='";
        if ( alexaParam == 'y')
          content += "y";
        else
          content += "n";

        content += "' style='width:20px'><input type='submit'></form> </html>";
        request->send(200, "text/html", content);
        content = "";
// =============================================================================================
    });





// =============================================================================================
    a_server.on("/mqttcfg", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
        IPAddress ip = WiFi.softAPIP();
        if (firstTime == 0) setFirst();

        int i;
        String broker = request->arg("broker");
        String port = request->arg("port");
        String user = request->arg("user");
        String pswd = request->arg("pswd");
        String log  = request->arg("log");
        String dom  = request->arg("dom");
        String dopt  = request->arg("dopt");
        String pers  = request->arg("persistence");
        String alex  = request->arg("alexa");
        if (log[0] == 'Y') log[0] = 'y';
        if (pers[0] == 'Y') pers[0] = 'y';
        if (alex[0] == 'Y') alex[0] = 'y';
        
      #ifdef NO_ALEXA_MQTT
        if ((broker.length() > 0) && (alex[0] == 'y'))
        {
          content = "{\"ERROR\":\"ALEXA option invalid in MQTT mode\"}";
          statusCode = 300;
        }
        else
      #endif
        {
          broker.toCharArray(mqtt_server, sizeof(mqtt_server));
          port.toCharArray(mqtt_port, sizeof(mqtt_port));
          user.toCharArray(mqtt_user, sizeof(mqtt_user));
          pswd.toCharArray(mqtt_password, sizeof(mqtt_password));

          char cdopt[4];
          dopt.toCharArray(cdopt, 4);
          char *ch;
          domotic_options = (char)strtoul(&dopt[0], &ch, 16);

          if (dom.length() == 0) dom = "d";
          domoticMode = dom.charAt(0);  // minuscolo
          if (log.length() == 0) log = "n";
          mqtt_log = log.charAt(0);
          if (pers.length() == 0) pers = "n";
          if (pers.charAt(0) == 'y')
            mqtt_persistence = 1;
          else
            mqtt_persistence = 0;
          alexaParam = alex[0];

          WriteEEP(broker, E_MQTT_BROKER);
          WriteEEP(port, E_MQTT_PORT);
          WriteEEP(dom[0], E_DOMOTICMODE);
          WriteEEP(domotic_options, E_DOMOTIC_OPTIONS);
          WriteEEP(user, E_MQTT_USER);
          WriteEEP(pswd, E_MQTT_PSWD);
          WriteEEP(log[0], E_MQTT_LOG);
          WriteEEP(pers[0], E_MQTT_PERSISTENCE);
          WriteEEP(alex[0], E_ALEXA);
          content = "{\"Success\":\"saved to eeprom ";

          EEPROM.commit();
          content += " ... please use .../reset?device=esp    to boot into new mqtt broker\"}";
          statusCode = 200;
        }
        request->send(statusCode, "text/html", content);
        content = "";
// =============================================================================================
    });






// =============================================================================================
    a_server.on("/callback", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
      content = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION "<p>";
      content += "</p><form method='get' action='backsetting'><label>Callback request: </label><input name='callback' length=128 maxlength=128 value='";
      content += httpCallback;
      content += "' style='width:800px'> <input type='submit'></form>";
      content += "</html>";
      request->send(200, "text/html", content);
      content = "";
// =============================================================================================
    });



    


// =============================================================================================
    a_server.on("/backsetting", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
      String httpback = request->arg("callback");
      httpback.toCharArray(httpCallback, sizeof(httpCallback));

      WriteEEP(httpCallback, E_CALLBACK);
    //  WriteStream(httpCallback, E_CALLBACK, 98);
      EEPROM.commit();

      content = "{\"Status\":\"OK\"}";
      statusCode = 200;

      request->send(statusCode, "application/json", content);
      content = "";
// =============================================================================================
    });






#ifdef FFS
// =============================================================================================
    a_server.on("/picprog", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
      // ?program= <Y>|<T>

      content = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION;
      content += " at ";
      content += WiFi.localIP().toString();
      content += "<p><ol>";

      String param = request->arg("program");
      if (param == "Y")
      {
        prog_mode = 2;  // programmazione vera
        sm_picprog = PICPROG_START;
      }
      if (param == "T")
      {
        prog_mode = 3;  // programmazione di prova
        sm_picprog = PICPROG_START;
      }

      if (sm_picprog == PICPROG_FREE)
      {
        Serial1.flush();
        Serial1.write(PIC_REQ);    // 
        Serial1.write('q');    // query PIC fw version
        delay(50);            // wait 50ms
        char sl = 0;
        if (Serial1.available() )
        {
          while (Serial1.available() && (sl < 15))
          {
            picfwVersion[sl++] = Serial1.read();        // receive from serial USB
            delayMicroseconds(INNERWAIT);
          }
          picfwVersion[0] = '>';
          picfwVersion[sl] = 0;
        }
      }

      content += "<li>";
      content += "PIC fw version: ";
      content += picfwVersion;
      content += "</li>";
      String s = "working...";

      if (sm_picprog == PICPROG_FREE)
      {
        s = "nothing";
        if (SPIFFS.begin(FORMAT_LITTLEFS_IF_FAILED)) 
        {
          String path = "/version.txt";

          if (SPIFFS.exists(path)) 
          {
            File f = SPIFFS.open(path, "r");
            if (f) 
            {
              while (f.position()<f.size())
              {
                s=f.readStringUntil('\n');
                s.trim();
              } 
              f.close();
            }
          }
        }

        content += "<li>";
        content += "NEW fw version: ";
        content += s;
        content += "</li>";
        content += "<li>";
        content += "last fw update rc: ";
      }
      else
      {
        content += "<li>";
        content += "current update: ";
      }
      
      content += prog_msg;
      content += " - retry: ";
      content += String(prog_retry,DEC);
      content += "</li>";

      content += "</ol>";
      content += "</form></html>";

      request->send(200, "text/html", content);
      content = "";
// =============================================================================================
    });
#endif


// =============================================================================================
    a_server.onNotFound(notFound);
// =============================================================================================
}
// -----------------------------------------------------------------------------------------------------------------------------------------------
// =====================================================================================================
#ifdef DEBUG
void manualInput(char prefix)
{
/*
  switch (prefix)
  {
    case 'O':
      fauxmo.setState(1, 1, 128);
      Serial.println("device 1: ON, 128");
      break;
    case 'o':
      fauxmo.setState(1, 0, 100);
      Serial.println("device 1: OFF, 100");
      break;
    case '1':  // switch
    case '3': // dimmer
    case '4': // dimmer
    case '8': // cover
    case '9': // coverpct
      EEPROM.write(0x11 + E_MQTT_TABDEVICES, prefix - '0'); // cambia devicetype di scs 0x11
      EEPROM.commit();
      break;
  }
*/
}
#endif



// =====================================================================================================
// =====================================================================================================
char BufferSearch(void)
{
   char bufNr = 0;
   do
   {
     if (serOlen[bufNr] == 0)  return bufNr;
     bufNr++;
   } while (bufNr < BUFNR);
   return 255;
}
// =====================================================================================================
// ===============================SCRITTURA UART DAL DOPPIO BUFFER======================================
// ==================================COMANDI ALEXA==non loggata=========================================
char SendToPIC(char bufNr)
{
   char len = serOlen[bufNr];
   // =============================CICLO SCRITTURA BUFFER UART =========================================
   if (len > 0)
   {
     bufSemaphor = bufNr;
     // =========================== send control char and data over serial =============================
     String log = "\r\ntx: ";
     char logCh[4];
     int s = 0;
     while (s < len)
     {
#ifdef UART_W_BUFFER
          Serial1.write(serObuffer[bufNr][s]);    //  scrittura SERIALE 
//          delayMicroseconds(120);
  #ifdef USE_TCPSERVER
          if (tcpuart == 2) 
          {
            sprintf(logCh, "%02X ", serObuffer[bufNr][s]);
            log += logCh;
          }
  #endif
#else
    
       // USS = uart register 1C-19 (32 bit)  bit 16-23=data nr in tx fifo   (USTXC = 16)
       // UART STATUS Registers Bits
       // USTX    31 //TX PIN Level
       // USRTS   30 //RTS PIN Level
       // USDTR   39 //DTR PIN Level
       // USTXC   16 //TX FIFO COUNT (8bit)
       // USRXD   15 //RX PIN Level
       // USCTS   14 //CTS PIN Level
       // USDSR   13 //DSR PIN Level
       // USRXC    0 //RX FIFO COUNT (8bit)

#ifdef DEBUG
#else
       while (((USS(0) >> USTXC) & 0xff) > 0)     
       {	// aspetta il buffer sia completamente vuoto
          delay(0);
       }
       delayMicroseconds(OUTERWAIT);
       USF(0) = serObuffer[bufNr][s];    // scrittura SERIALE
       while (((USS(0) >> USTXC) & 0xff) > 0)     
       {	// aspetta il buffer sia completamente vuoto
          delay(0);
       }
#endif
#ifdef DEBUG
       sprintf(logCh, "%02X ", serObuffer[bufNr][s]);
       log += logCh;
#endif
#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
       if (tcpuart == 2) 
       {
         sprintf(logCh, "%02X ", serObuffer[bufNr][s]);
         log += logCh;
       }
  #endif
#endif
       if (s <= len)
          delayMicroseconds(OUTERWAIT); // 100uS
#endif // UART_W_BUFFER
       s++;
     }
#ifdef DEBUG
     Serial.println("\r\n" + log);
#endif

#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
     if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
     {
        tcpclient.write((char*)&log[0], log.length());
        tcpclient.flush(); 
     }
  #endif
#endif
     serOlen[bufNr] = 0;
     bufSemaphor = -1;
   } // len > 0
   return 0;

}  


// =====================================================================================================
// =====================================================================================================
void ConnectToRouter(String esid, String epass)
{
// =========================================================================================
//        CONNESSIONE  AL  ROUTER 
// =========================================================================================

  WiFi.disconnect(false, true); // Pulisce eventuali sessioni salvate corrotte
  WiFi.mode(WIFI_STA);

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  // 2. Evita blocchi nella negoziazione WPA2/WPA3
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);

  // 3. Configura PMF (Protected Management Frames) come opzionale (non obbligatorio)
  wifi_config_t wifi_config;
  esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
  wifi_config.sta.pmf_cfg.capable = true;
  wifi_config.sta.pmf_cfg.required = false;
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  WiFi.begin(esid.c_str(), epass.c_str());   

  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  // =========================================================================================

    if ((local_ip[0] > 0) && (local_ip[0] < 255))
    {
#ifdef VERBOSE
      Serial.println(" ");
      Serial.println("static IP");
#endif
      WiFi.config(local_ip, router_ip, IPAddress(255, 255, 255, 0));
    }
#ifdef VERBOSE
    else
    {
      Serial.println(" ");
      Serial.println("dynamic IP");
    }
#endif

    if (testWifi())   // wifi connesso
    {
      Serial1.write(PIC_REQ);   // set led lamps
//      Serial1.write('@');   // set led lamps
      Serial1.write(0xF1);  // set led lamps std-freq (client mode)

      WiFi.mode(WIFI_STA);
      //      WiFi.disconnect();
      launchWeb(0);

#ifdef VERBOSE
      Serial.println("Ready");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
#endif

      // ==================================== TCP server ====================================================
#ifdef USE_TCPSERVER
      tcpserver.begin();
      tcpopen = 1;
#endif 

      // ==================================== alexa - fauxmo ================================================

      if (alexaParam == 'y' )
      {

        // By default, fauxmoESP creates it's own webserver on the defined port
        // The TCP port must be 80 for gen3 devices (default is 1901)
        // This has to be done before the call to enable()

        fauxmo.createServer(true); // not needed, this is the default value
        fauxmo.setPort(80); // This is required for gen3 devices

        // You have to call enable(true) once you have a WiFi connection
        // You can enable or disable the library at any moment
        // Disabling it will prevent the devices from being discovered and switched

        fauxmo.enable(true);
#ifdef DEBUG
        Serial.println("- fauxmo enabled");
        Serial.printf("[START] Free heap: %d bytes\r\n", ESP.getFreeHeap());
#endif
        // fauxmo ALEXA add devices <=============================================================================================

        id_interfaccia_scs_knx = fauxmo.addDevice(ID_MY, 0);
        
        uint8_t devtype;
        uint8_t devx = 1;
        for (devx = 0; devx < MY_ENTITA; devx++)
        {
          if (sistema_scs.elenco[devx].valido == 1)
          {
              devtype = sistema_scs.elenco[devx].tipo_codice;
              if ((devtype > 0) && (devtype != 11) && (devtype < 18))  // W6 - in alexa censire solo indirizzi base
              {
                  if (devtype == 9)
                    id_fauxmo = fauxmo.addDevice(sistema_scs.elenco[devx].descrizione, 1);
                  else
                    id_fauxmo = fauxmo.addDevice(sistema_scs.elenco[devx].descrizione, 128);
                
                  alexa_BUS_ix[id_fauxmo] = devx;  // index: device id alexa max MY_ENTITA devices    data: ID bus reale
                  sistema_scs.elenco[devx].alexa_id = id_fauxmo; 
/*
                  if (events.count() != 0) 
                  {
                      char buflog[128];
              		  String tipo_str = codifica_codice_a_tipo(sistema_scs.elenco[i].tipo_codice); 
                      sprintf(buflog,"[%d] [scs] %d, id: %02X, %s, %s, st: %s, val: %d, AX=%02X",
                        (millis()/100), i, sistema_scs.elenco[i].id, tipo_str, 
                        sistema_scs.elenco[i].descrizione, sistema_scs.elenco[i].stato, sistema_scs.elenco[i].valido, sistema_scs.elenco[i].alexa_id);
                      events.send(buflog, "log", millis());
                  }
*/
              }
          }
        }
        // ----------------------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------------------


#ifdef VERBOSE
        Serial.printf("[START] %d fauxmo devices added\r\n", id_fauxmo + 1);
        Serial.printf("[START] Free heap: %d bytes\r\n", ESP.getFreeHeap());
#endif



        // fauxmo ALEXA callback <=============================================================================================
#ifdef DEBUG_FAUXMO_TCP         
        fauxmo.onSetState([](unsigned char alexa_id, const char * device_name, char alexacommand, unsigned char value, const char * body, const char * response)
#else
        fauxmo.onSetState([](unsigned char alexa_id, const char * device_name, char alexacommand, unsigned char value)
#endif
        { // callback start

          // Callback when a command from Alexa is received.
          // You can use alexa_id or device_name to choose the element to perform an action onto (relay, LED,...)
          //// old // State is a bool (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
          // command is a char (1=ON 2=OFF 3=bright equal  4=bright+   5=bright- )
          // value is a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
          // Just remember not to delay too much here, this is a callback, exit as soon as possible.
          // If you have to do something more involved here set a flag and process it in your main loop.

#ifdef VERBOSE
          Serial.printf("\r\n- Device #%d (%s) command: %d value: %d ", alexa_id, device_name, alexacommand, value);
#endif

          char bufNr = BufferSearch();

//        if (bufSemaphor == -1)  bufNr = 0;
//        else                    bufNr = 1;


#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
          if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
          {
            char tcpbuffer[55];
            int  buflen;
            sprintf(tcpbuffer,"\r\n- Device #%d (%s) command: %d value: %d       ", alexa_id, device_name, alexacommand, value);
            tcpclient.write(tcpbuffer, 47);
            tcpclient.write("\r\n  req: ", 9);
            tcpclient.write(body, strlen(body));
            tcpclient.write("\r\n  res: ", 9);
            tcpclient.write(response, strlen(response));
            sprintf(tcpbuffer,"\r\n- UART semaphor %d   bufnr: %d   len 0-1: %d- %d       ", bufSemaphor, bufNr, serOlen[0], serOlen[1]);
            tcpclient.write(tcpbuffer, 50);

            tcpclient.flush(); 
          }
  #endif
#endif

          // Checking for alexa_id is simpler if you are certain about the order they are loaded and it does not change.
          // Otherwise comparing the device_name is safer.

          //      if (strcmp(device_name, ID_MY)==0)
          
          if (alexa_id == id_interfaccia_scs_knx)
          {
#ifdef DEBUG
            Serial.println(ID_MY);
#endif
//            if (alexacommand == 1) // accendi <----------------
//              system_update_cpu_freq(160);
//            else if (alexacommand == 2) // spegni <----------------
//              system_update_cpu_freq(80);
          }
          else // an scs/knx device
          {
            char devix = alexa_BUS_ix[alexa_id];
            char devtype = sistema_scs.elenco[devix].tipo_codice;
#ifdef DEBUG
            Serial.printf("\r\n-       device " _MODO " %02X - type: %02X ", de[devix].address, devtype);
#endif

            // If your device state is changed by any other means (MQTT, physical button,...)
            // you can instruct the library to report the new state to Alexa on next request:
            // fauxmo.setState(ID_YELLOW, true, 255);

            {
              if (firstTime == 0) setFirst();  // DEVONO essere attivi @MX  e @l
              unsigned char command;
              int pct;
              char stato = fauxmo.getState(alexa_id);
              char device = sistema_scs.elenco[devix].id;
              switch (devtype)
              {
                // --------------------------------------- SWITCHES -------------------------------------------
                case 1:
                  if (alexacommand == 1) // accendi <----------------
                  {
                    command = 0;
                    fauxmo.setState(alexa_id, stato | 1, 128);
                  }
                  else if (alexacommand == 2) // spegni  <----------------
                  {
                    command = 1;
                    fauxmo.setState(alexa_id, stato & 0xFE, 128);
                  }
                  else
                    break;
                    
                  serObuffer[bufNr][serOlen[bufNr]++] = PIC_REQ;
                  serObuffer[bufNr][serOlen[bufNr]++] = 'y';  // 0x79 (@y: invia a pic da ALEXA cmd standard da inviare sul bus)

// comando §y<destaddress><source><type><command>
                  serObuffer[bufNr][serOlen[bufNr]++] = sistema_scs.elenco[devix].id;     // to   device address
                  serObuffer[bufNr][serOlen[bufNr]++] = 0x00;       // from device
                  serObuffer[bufNr][serOlen[bufNr]++] = 0x12;       // command type
                  serObuffer[bufNr][serOlen[bufNr]++] = command;    // command char
                  break;


                case 3:
                  // --------------------------------------- LIGHTS DIMM ------------------------------------------
                  if (alexacommand == 1) // accendi  <----------------
                  {
                    command = 0;
                    fauxmo.setState(alexa_id, stato | 1, 0);
                  }
                  else if (alexacommand == 2) // spegni <----------------
                  {
                    command = 1;
                    fauxmo.setState(alexa_id, stato & 0xFE, 0);
                  }
                  else if ((alexacommand == 3) || (alexacommand == 4) || (alexacommand == 5)) // cambia il valore  <----------------
                  {
                    fauxmo.setState(alexa_id, 1, value);

                    // trasformare da % a 1D-9D <--------------------------------------------------------------
                    pct = value;    // percentuale da 0 a 255
                    pct *= 100;                  // da 0 a 25500
                    pct /= 255;                  // da 0 a 100
                    pct += 5;                    // arrotondamento
                    pct /= 10;                   // 0-10
                    if (pct > 9) pct = 9;
                    if (pct == 0) pct = 1;	   // 1-9
                    pct *= 16;                   // hex high nibble
                    pct += 0x0D;                 // hex low  nibble
                    command = (unsigned char) pct;
                  }
                  else
                    break;

                  serObuffer[bufNr][serOlen[bufNr]++] = PIC_REQ;
                  serObuffer[bufNr][serOlen[bufNr]++] = 'y';        // 0x79 (@y: invia a pic da ALEXA cmd standard da inviare sul bus)
                  serObuffer[bufNr][serOlen[bufNr]++] = device;     // to   device address
                  serObuffer[bufNr][serOlen[bufNr]++] = 0x00;       // from device
                  serObuffer[bufNr][serOlen[bufNr]++] = 0x12;       // command type
                  serObuffer[bufNr][serOlen[bufNr]++] = command;    // command char
                  break;


                case 4:
                  break;

                case 8:
//              case 18:
                  // --------------------------------------- COVER ---------------------------------------------------
                  if ((alexacommand == 2) || (alexacommand == 1)) // spegni / ferma  <--oppure accendi------
                  {
                    command = 0x0A;
                    fauxmo.setState(alexa_id, stato | 0xC1, value); // 0xc1: dara' errore ma almeno evita il blocco
                  }
                  else if (alexacommand == 4) // alza  <----------------
                  {
                    command = 0x08;
                    fauxmo.setState(alexa_id, stato | 0xC0, value); // 0xc0: dopo aver inviato lo stato setta value a 128
                  }
                  else if (alexacommand == 5) // abbassa  <----------------
                  {
                    command = 0x09;
                    fauxmo.setState(alexa_id, stato | 0xC0, value); // 0xc0: dopo aver inviato lo stato setta value a 128
                  }
                  else
                    break;

                  serObuffer[bufNr][serOlen[bufNr]++] = PIC_REQ;
                  serObuffer[bufNr][serOlen[bufNr]++] = 'y';        // 0x79 (@y: invia a pic da ALEXA cmd standard da inviare sul bus)

// comando §y<destaddress><source><type><command>
                  serObuffer[bufNr][serOlen[bufNr]++] = device;     // to   device address
                  serObuffer[bufNr][serOlen[bufNr]++] = 0x00;       // from device
                  serObuffer[bufNr][serOlen[bufNr]++] = 0x12;       // command type
                  serObuffer[bufNr][serOlen[bufNr]++] = command;    // command char
                  break;

                case 9:
//              case 19:
                  // --------------------------------------- COVERPCT alexa------------------------------------------------
                  fauxmo.setState(alexa_id, 1, value);  // coverpct - lo stato deve sempre essere ON
                  pct = value;
                  pct *= 100;
                  pct /= 255;

                  if (alexacommand == 2) // spegni / ferma  <----------------
                  {
                    command = 0;
                    if (value < 5)
                      fauxmo.setState(alexa_id, 0, value);
                  }
                  else if (alexacommand == 1) // accendi (SU) <----------
                  {
                    command = 1;
                  }
                  else if (alexacommand == 4) // alza  <----------------
                  {
                    command = 1;
                    command = pct;
                  }
                  else if (alexacommand == 5) // abbassa  <----------------
                  {
                    command = 2;
                    command = pct;
                  }
                  else // (alexacommand == 3) // non cambia  <----------------
                    break;


                  serObuffer[bufNr][serOlen[bufNr]++] = PIC_REQ;
                  serObuffer[bufNr][serOlen[bufNr]++] = 'u';
                  serObuffer[bufNr][serOlen[bufNr]++] = device;     // to   device address
                  serObuffer[bufNr][serOlen[bufNr]++] = command;        // %
                  break;
              } // end switch

            } // end uartsemaphor
          } // end else   an scs device
        });
#ifdef NO_ALEXA_UDP
		udpopen = 0;
#else
		udpopen = 9;  // udp request
#endif
      }  // end if alexa

      
      
      if (udpopen == 9)
      {
#ifdef DEBUG
        Serial.println("open UDP port");
#endif
        udpopen = udpConnection.begin(udpLocalPort);
#ifdef DEBUG
        if (udpopen == 1)
          Serial.println ( "UDP server started" );
        else
          Serial.println ( "UDP open ERROR" );
#endif
      }
      Serial1.write(PIC_REQ);   // set led lamps
//      Serial1.write('@');   // set led lamps
      Serial1.write(0xF1);  // set led lamps std-freq (client mode)


      // =================================================================================================

      if ((mqtt_server[0] >= '0') && (mqtt_server[0] <= '9'))
      {
#if defined DEBUG
        Serial.print("MQTT set server ");
        Serial.println(mqtt_server);
#endif
        mqttopen = 1;
        client.setServer(mqtt_server, atoi(mqtt_port));    // Configure MQTT connexion
        client.setCallback(MqttCallback);           // callback function to execute when a MQTT message
      }
      // =================================================================================================
      if ((alexaParam == 'y') && (firstTime == 0))
        setFirst();  // DEVONO essere attivi @MX  e @l
    }
    else
    {
      // =================================================================================================
      // CONNESSIONE WIFI FALLITA . REBOOT
      // =================================================================================================
      ESP.restart();
    }
}


// =====================================================================================================
void setup() {
// =====================================================================================================

  char forceAP = 0;
  char jumperOpen;
  prog_msg = "no info";

  // esp_netif_set_hostname( _modo "gate" );
  
// pin alternativi s3 rx/tx
  pinMode(5, INPUT); // tx
  pinMode(6, INPUT); // rx

// initialize uart and uart2
#if defined(DEBUG) || defined(VERBOSE)
  Serial.begin(115200);		// debug monitor USB
  delay(500);
  while (!Serial) {
    ;
  }
  Serial.flush();
#endif

//  Serial1.setPins(RX1, TX1);
  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N2, 20, 21);
//  Serial1.setHwFlowCtrlMode(HW_FLOWCTRL_DISABLE);

  while (!Serial1) {
    ;
  }
  Serial1.flush();

  // aspetta 15 secondi perche' con l'assorbimento iniziale di corrente esp8266 fa disconnettere l'adattatore seriale
  int pauses = 0;
  
#ifdef BLINKLED
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH); // spento
#endif
  while (pauses < 150) // 15 secondi wait
  {
      pauses++;
      delay(100);            // wait 100ms
  }
  Serial1.write('@');    // 
  Serial1.write(0xF0);   // set led lamps low-freq (in progress mode)
  delay(10);            // wait 10ms
#ifdef BLINKLED
  digitalWrite(pinLed, HIGH); // spento
#endif

//===============================jumper test========================================
#ifdef JMP
  pinMode(JMP, OUTPUT);
  digitalWrite(JMP, HIGH); // OUTPUT ALTO
  delay(20);
  pinMode(JMP, INPUT);
  jumperOpen = digitalRead(JMP); //  == 1 jumper aperto
 #ifdef VERBOSE
  if (jumperOpen == 1)
	Serial.println("jumper OPEN");
  else
	Serial.println("jumper CLOSED");
 #endif
  pinMode(JMP, INPUT);
#endif
   
  EEPROM.begin(MAXEEPROM);
  
  char eeSignature = EEPROM.read(E_EESIGNATURE);
  if (eeSignature == EESIGNATURE_FROM)
  {
	EEPROM.write(E_EESIGNATURE, EESIGNATURE);
	EEPROM.write(E_DOMOTIC_OPTIONS, 0x01);
  }
  else
  if (eeSignature != EESIGNATURE)
  {
#ifdef DEBUG
    Serial.println("eeinit");
#endif
    int we = 0;
    while (we < 4096)
    {
      EEPROM.write(we, 0);
      we++;
    }
	EEPROM.write(E_EESIGNATURE, EESIGNATURE);
	EEPROM.write(E_DOMOTICMODE, 'H');
	EEPROM.write(E_DOMOTIC_OPTIONS, 0x01);
	EEPROM.write(E_MQTT_LOG, 'n');
	EEPROM.write(E_MQTT_PERSISTENCE, 'y');
	EEPROM.write(E_ALEXA, 'n');
	
    EEPROM.commit();  
  }
//===============================================================================


#ifdef VERBOSE
// ----------------------------------------- DEBUG -------------------------------------------
  Serial.println();
  Serial.println();
  Serial.println("Startup");

  Serial.println("Press  'a' for start as AP, 'r' for router client");
#ifdef FFS
  Serial.println("Press  's' for speed test,  'f' for spiffs test");
#endif
  Serial.setTimeout(5000);
  Serial.readBytes(&serIniOption, 1);
  Serial.setTimeout(100);
#endif

// ------------- default dipende dal jumper ---------------------------
  if (serIniOption == 0)
  {
	if (jumperOpen == 0)
		serIniOption = 'a';
	else
		serIniOption = 'r';
  }

// ------------- 'e' : EEPROM init ------------------------------------
  if (serIniOption == 'e')
  {
    int we = 0;
    while (we < 4096)
    {
      EEPROM.write(we, 0);
      we++;
    }
    EEPROM.commit();
#ifdef VERBOSE
    Serial.println("e: eeprom init OK");
    Serial.println("Forced AP mode");
#endif
    forceAP = 1;
  }



// ------------- 'a' : AP mode ----------------------------------------
  if ((serIniOption == 'a') || (serIniOption == 'A'))
  {
    forceAP = 1;
#ifdef VERBOSE
    Serial.println("a: AP mode");
#endif
  }
  
// ------------- 'r' : ROUTER (client) mode ---------------------------
  if (serIniOption == 'r')
  {
    forceAP = 0;
#ifdef VERBOSE
    Serial.println("r: router (client) mode");
#endif
  }
// -----------------------------------------------------------------------------------------------------------

  Serial1.write('@');    // 
  Serial1.write(0xF0);   // set led lamps low-freq (in progress mode)
  delay(10);            // wait 10ms

  String esid;
  esid = ReadStream(&esid[0], E_SSID, 32, 2);  // tipo=0 binary array   1:ascii array   2:ascii string
  if ((esid[0] == 0) || (esid[0] == 0xFF))
  {
    forceAP = 1;
#ifdef DEBUG
    Serial.println("- AP mode");
#endif
  }
  
#ifdef VERBOSE
  Serial.println(esid);
  Serial.print("Read EEPROM pass: ");
#endif

  String epass = "";
  epass = ReadStream(&epass[0], E_PASSW, 32, 2);  // tipo=0 binary array   1:ascii array   2:ascii string
#ifdef DEBUG
  Serial.println(epass);
#endif

  ReadStream(httpCallback, E_CALLBACK, sizeof(httpCallback), 1);  // tipo=0 binary array   1:ascii array   2:ascii string
#ifdef DEBUG
  Serial.print("callback=");
  Serial.println(httpCallback);
#endif

  ReadStream(mqtt_server, E_MQTT_BROKER, sizeof(mqtt_server), 1);  // tipo=0 binary   1:ascii
#ifdef DEBUG
  Serial.print("broker=");
  Serial.println(mqtt_server);
#endif

  ReadStream(mqtt_port, E_MQTT_PORT, sizeof(mqtt_port), 1);  // tipo=0 binary   1:ascii
#ifdef DEBUG
  Serial.print("port=");
  Serial.println(mqtt_port);
#endif

  domoticMode = ReadStream(E_DOMOTICMODE);
#ifdef DEBUG
  Serial.print("domoticMode=");
  Serial.println(domoticMode);
#endif

  domotic_options = ReadStream(E_DOMOTIC_OPTIONS);  // tipo=0 binary   1:ascii

  ReadStream(mqtt_user, E_MQTT_USER, sizeof(mqtt_user), 1);  // tipo=0 binary   1:ascii
#ifdef DEBUG
  Serial.print("user=");
  Serial.println(mqtt_user);
#endif

  ReadStream(mqtt_password, E_MQTT_PSWD, sizeof(mqtt_password), 1);  // tipo=0 binary   1:ascii
#ifdef DEBUG
  Serial.print("password=");
  Serial.println(mqtt_password);
#endif

  mqtt_log = ReadStream(E_MQTT_LOG);
#ifdef DEBUG
  Serial.print("log=");
  Serial.println(mqtt_log);
#endif

  alexaParam = ReadStream(E_ALEXA);
#ifdef DEBUG
  Serial.print("alexa=");
  Serial.println(alexaParam);
  if (alexaParam == 'y')
    Serial.println("HTTP_PORT = 8080");
#endif
  if (alexaParam == 'y')
  {
    http_port = 8080;  // 8080
  }

  mqtt_persistence = ReadStream(E_MQTT_PERSISTENCE);
  if (mqtt_persistence == 'y')
    mqtt_persistence = 1;
  else
    mqtt_persistence = 0;

#ifdef DEBUG
  Serial.print("pers=");
  Serial.println(mqtt_persistence);
#endif

  Serial1.write('@');    // 
  Serial1.write(0xF0);   // set led lamps low-freq (in progress mode)
  delay(10);            // wait 10ms
  Serial1.flush();
  Serial1.write('@');    // 
  Serial1.write('q');    // query PIC fw version
  delay(50);            // wait 50ms
  char sl = 0;
  if (Serial1.available() )
  {
    while (Serial1.available() && (sl < 15))
    {
      picfwVersion[sl++] = Serial1.read();        // receive from serial USB
      //    delayMicroseconds(INNERWAIT);
    }
    picfwVersion[0] = '>';
    picfwVersion[sl] = 0;
  }
#ifdef VERBOSE
  Serial.print(picfwVersion);
  Serial.println(">");
#endif

  setFirst();
  internal = 1;
  delay(50);            // wait 50ms

  Serial1.write(PIC_REQ);   // set led lamps
  Serial1.write('b');   // clear SCSgate/KNXgate buffer
  Serial1.setTimeout(10); // timeout is 10mS
  delay(50);           // wait 50ms
  Serial1.flush();

  String qip = ReadStream(&qip[0], E_IPADDR, 16, 2);  // tipo=0 binary   1:ascii   2=string
  local_ip.fromString(qip);

#ifdef VERBOSE
  if (serIniOption == 'a')
  {
    Serial.println("");
    Serial.print("local ip=");
    Serial.println(local_ip);
  }
#endif
  String qrip = ReadStream(&qrip[0], E_ROUTIP, 16, 2);  // tipo=0 binary   1:ascii  2=string
  router_ip.fromString(qrip);

#ifdef VERBOSE
  if (serIniOption == 'r')
  {
  Serial.print("router ip=");
  Serial.println(router_ip);
  }
#endif

  String qport = ReadStream(&qport[0], E_PORT, 6, 2);  // tipo=0 binary   1:ascii   2=string
  udpLocalPort = qport.toInt();
  if ((udpLocalPort == 0) || (udpLocalPort == 0xFFFF))
  {
    udpLocalPort = 52056;
  }
#ifdef VERBOSE
  Serial.print("local port=");
  Serial.println(udpLocalPort);
#endif

  inizializza_e_leggi_dispositivi_preferences();

  if ((serIniOption != 'a') && (esid.length() > 1 ))
// =========================================================================================
//        CONNESSIONE  AL  ROUTER 
// =========================================================================================
  {
	  ConnectToRouter(esid, epass);
      setFirst();
      internal = 1;
      prev_internal = 5;
  }
  else
  {
//    Serial1.write('@');   // set led lamps
    Serial1.write(PIC_REQ);   // set led lamps
    Serial1.write(0xF2);  // set led lamps high-freq (AP mode)
    // =================================================================================================
    // access point mode
    // =================================================================================================
#ifdef DEBUG
    if (serIniOption == 'A')
      setupAP(1);
    else
#endif
    setupAP(0);
    Serial1.write(PIC_REQ);   // set led lamps
//    Serial1.write('@');   // set led lamps
    Serial1.write(0xF2);  // set led lamps high-freq (AP mode)
  }


      // ==================================== OTA startup ================================================
#ifdef USE_OTA
      // Port defaults to 8266
      // ArduinoOTA.setPort(8266);

      // Hostname defaults to esp8266-[ChipID]
      // ArduinoOTA.setHostname("myesp8266");

      // No authentication by default
      // ArduinoOTA.setPassword("admin");

      // Password can be set with it's md5 value as well
      // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
      // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

      ArduinoOTA.onStart([]() {

#ifdef DEBUG
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()

        Serial.println("Start updating " + type);
#endif

        requestBuffer[requestLen++] = '@';
        requestBuffer[requestLen++] = '|'; // reset PIC
        requestBuffer[requestLen++] = '|';
        immediateSend();

        ArduinoOTAflag = 1;
        EEPROM.commit();
      });

      ArduinoOTA.onEnd([]() {
#ifdef DEBUG
        Serial.println("\nOTA update END");
        Serial.println("");
#endif
      });

      ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
#ifdef DEBUG
        /*    static unsigned int prev_progress;
            if (progress != prev_progress)
            {
              prev_progress = progress;
              Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
            }
        */
        Serial.printf(".");
#endif
      });

      ArduinoOTA.onError([](ota_error_t error) {
#ifdef DEBUG
        ArduinoOTAflag = 0;
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
#endif
      });

      ArduinoOTAflag = 1;
      ArduinoOTA.begin();
#ifdef DEBUG
      Serial.println("OTA begin OK");
#endif
#endif


  
}
// -----------------------------------------------------------------------------------------------------------------------------------------------
String tcpJarg(char * mybuffer, char * argument)
{
  String val;

  char* p1 = strstr(mybuffer, argument); // cerca l'argomento
  if (p1)
  {
//  char* p2 = strstr(p1, ":");          // cerca successivo :
    char* p2 = strchr(p1, ':');          // cerca successivo :
    if (p2)
    {
//    char* p3 = strstr(p2, "\"");       // cerca successivo "
      char* p3 = strchr(p2, '\"');       // cerca successivo "
      if (p3)
      {
        p3++;
        char l = 0;
        while ((*p3 != '\"') && (l < 120))
        {
          val = val + *p3;
          p3++;
          l++;
        }
      }
    }
  }

  return val;
}
// -----------------------------------------------------------------------------------------------------------------------------------------------

#ifdef FFS
// =====================================================================================================
// PicProg
// =====================================================================================================
void PicProg(void)
{
   String path = PICPATH;
   int s, l, progptr;
   char bLenght;
   char sbuf[8];
   WORD_VAL block_check;
   
   switch (sm_picprog)
   {
    case PICPROG_FREE:
	  break;
//------------------------------------------------------
    case PICPROG_START:
      if (!SPIFFS.begin(FORMAT_LITTLEFS_IF_FAILED)) 
      {
         sm_picprog = PICPROG_ERROR;
         prog_error = 1;
         prog_msg = "SPIFFS error";
      } else 
      {
         prog_error = 0;
         prog_retry = 0;
         prog_address.Val = 0;
         prog_msg = "running";
      }

      if (SPIFFS.exists(path)) 
      {
         picFw = SPIFFS.open(path, "r");
         if (!picFw) 
         {
           sm_picprog = PICPROG_ERROR;;
           prog_error = 11;
           prog_msg = PICPATH " not exists";
         } else 
         {
           Serial1.write('@');
           Serial1.write(0x11);   // autoprogram request
           sm_picprog = PICPROG_REQUEST_WAIT;
           Serial1.flush();
         }
      }
      else 
      {
         sm_picprog = PICPROG_ERROR;
         prog_error = 12;
         prog_msg = PICPATH " not found";
      }
	  break;

//------------------------------------------------------
    case PICPROG_REQUEST_WAIT:
      Serial1.setTimeout(100); // timeout calcolato per 80bytes uart (tx/rx) + program time (?)
      l = Serial1.readBytes(&bLenght, 1);
      if (l == 1)
      {
          l = Serial1.readBytes(sbuf, bLenght);
          if (l == 4)
          {
				//   04 10 07 00 00   (l=4, destination=10, format=07, data=00 00      
             if ( (sbuf[0] == 0x10) && (sbuf[1] == 0x07))   
                 sm_picprog = PICPROG_REQUEST_OK;
             else
             {
               sm_picprog = PICPROG_ERROR;
               prog_error = 21;
               prog_msg = "PIC answer error at @0x11";
             }
          }
          else      
          {
            sm_picprog = PICPROG_ERROR;
            prog_error = 22;
            prog_msg = "PIC answer wrong at @0x11";
          }
      }
      else 
      {
         sm_picprog = PICPROG_ERROR;
         prog_error = 23;
         prog_msg = "PIC don't answer at @0x11";
      }
	  break;

//------------------------------------------------------
    case PICPROG_REQUEST_OK:
      l=picFw.readBytes(prog_file_data, PICBUF);
      sm_picprog = PICPROG_FLASH_BLOCK;
      while (l < PICBUF)
      {
         prog_file_data[l++] = 0xFF;
      }
	  break;

//------------------------------------------------------
    case PICPROG_FLASH_BLOCK:
//    vb6:  WriteBuf (Chr(Len(sWrite) + 2) + Chr(10) + Chr(7) + sWrite)

// compute checksum      
      block_check.Val = 0;
      for (s=0; s < PICBUF; s++)
      {
         block_check.Val += prog_file_data[s];
      }
      if  (prog_address.byte.HB == 0xF0)
//    if ((prog_address.byte.HB == 0xF0) || (block_check.Val == PICBUF * 255))
      {
         sm_picprog = PICPROG_FLASH_END;
         break;
      }

      // block 1: firmware block address
      //   06 10 07 01 00 00 40 
      prog_buffer[0] = 6;     // data length
      prog_buffer[1] = 0x10;  // from
      prog_buffer[2] = 0x07;  // format
      prog_buffer[3] = 0x01;  // command
      
      prog_buffer[4] = prog_address.byte.LB;  // address LB
      prog_buffer[5] = prog_address.byte.HB;  // address HB
      prog_buffer[6] = 64;    // data block length (command 0x40)
      Serial1.write(prog_buffer, 7);
//      delayMicroseconds(120);

      
      // block 2-3-4-5-6-7-8-9: firmware block data
      //   10 10 07 xx xx xx xx xx xx xx xx 

      progptr = 0;
      for (s=0; s < 8; s++)
      {
        prog_buffer[0] = 10;     // data length
        prog_buffer[1] = 0x10;  // from
        prog_buffer[2] = 0x07;  // format
        
        prog_buffer[3] = prog_file_data[progptr++];
        prog_buffer[4] = prog_file_data[progptr++];
        prog_buffer[5] = prog_file_data[progptr++];
        prog_buffer[6] = prog_file_data[progptr++];
        prog_buffer[7] = prog_file_data[progptr++];
        prog_buffer[8] = prog_file_data[progptr++];
        prog_buffer[9] = prog_file_data[progptr++];
        prog_buffer[10] = prog_file_data[progptr++];
        Serial1.write(prog_buffer, 11);
//        delayMicroseconds(120);
     }

      // block 10: firmware block end
      //   05 10 07 02 ck ck
      prog_buffer[0] = 5;     // data length
      prog_buffer[1] = 0x10;  // from
      prog_buffer[2] = 0x07;  // format
      prog_buffer[3] = prog_mode;  // command 2: true write    3: test
      
      prog_buffer[4] = block_check.byte.LB;
      prog_buffer[5] = block_check.byte.HB;
      Serial1.write(prog_buffer, 6);
//      delayMicroseconds(120);

      sm_picprog = PICPROG_FLASH_WAIT;
	  break;

    case PICPROG_FLASH_WAIT:
      l = Serial1.readBytes(&bLenght, 1);
      if (l == 1)
      {
          l = Serial1.readBytes(sbuf, bLenght);
          if (l == 8)
          {
             //   08 10 07 02 00 F1 1D F1 1D  (l=8, destin=10, fmt=07, req=02 write - 03=test )
             //               ^^ d(1)=00 ok   01 protect   >0xF0 error
             if ((sbuf[3] >= 0xF0) && (sbuf[3] != 0xFE)) 
             {  
                sm_picprog = PICPROG_FLASH_BLOCK;
                if (++prog_retry > 100)
                {
                  sm_picprog = PICPROG_ERROR;
                  prog_error = 24;
                  prog_msg = "PIC flash too retry: ";
                  sprintf(prog_buffer, "%02X%02X %02X%02X", sbuf[4], sbuf[5], sbuf[6], sbuf[7]);
                  prog_msg += prog_buffer;
                }
             }
             else
             {  // ok
                prog_address.Val += PICBUF;
                sm_picprog = PICPROG_REQUEST_OK;
             }
          }
          else      
          {
            sm_picprog = PICPROG_ERROR;
            prog_error = 24;
            prog_msg = "PIC answer wrong at flashW";
          }
      }
      else 
      {
         sm_picprog = PICPROG_ERROR;
         prog_error = 25;
         prog_msg = "PIC don't answer at flashW";
      }
	  break;

    case PICPROG_ERROR:
// prog_error 1-9 nothing to close     10-20 close spifs     20-30 close file
      if (prog_error >= 19)
          picFw.close();
      if (prog_error >= 9)
          SPIFFS.end();
      sm_picprog = PICPROG_FREE;
      char hBuffer[20];
      sprintf(hBuffer, " - last addr: %02X%02X", prog_address.byte.HB, prog_address.byte.LB);
      prog_msg += hBuffer;
	  break;
	  
    case PICPROG_FLASH_SWAP:
//    Serial1.write(0x61);  // flash programming end
      //   03 10 07 61
      prog_buffer[0] = 3;     // data length
      prog_buffer[1] = 0x10;  // from
      prog_buffer[2] = 0x07;  // format
      prog_buffer[3] = 0x61;  // command
      Serial1.write(prog_buffer, 4);
//      delayMicroseconds(500);

      prog_address.Val += PICBUF;
      sm_picprog = PICPROG_REQUEST_OK;
	  break;
	  
    case PICPROG_FLASH_END:
//    Serial1.write(0x80);  // flash programming end
      
      //   03 10 07 80
      prog_buffer[0] = 3;     // data length
      prog_buffer[1] = 0x10;  // from
      prog_buffer[2] = 0x07;  // format
      prog_buffer[3] = 0x80;  // command
      Serial1.write(prog_buffer, 4);
//      delayMicroseconds(500);

      prog_msg = "PIC flash OK";
      picFw.close();
      SPIFFS.end();
      sm_picprog = PICPROG_FREE;
	  break;
   }
}
#endif







// **************************************************************************************************++


uint8_t scs_tratta_messaggi_per_mqtt(void)
{
  //    [F3] u <address> <position%>
  if ((replyLen == 4) && (replyBuffer[0] == 0xF3))    
  { // replyLen==4 && replyBuffer == 0xF3 'u'
    uint8_t device = replyBuffer[2];
    int pct = replyBuffer[3];
//    uint8_t devx = CercaAddress(device);
   
#ifdef TRACE_SSE
  char buflog[120];
  sprintf(buflog, "[TRC} tap/dimm mqtt:  %c ", replyBuffer[1]);
  events.send(buflog, "log", millis());
#endif
    
	if (replyBuffer[1] == 'u')    //'u': posizione tapparelle//
		// ----------------uab-(address position)--- PUBBLICAZIONE STATO COVERPCT -------------------------------------------------------
	{ // pubblicazione posizione coverpct   
		char actionc[6];

		sprintf(actionc, "%03u", pct);     // position
		char nomeDevice[5];
		sprintf(nomeDevice, "%02X", device);  // device
		String topic = COVERPCT_STATE;
		topic += nomeDevice;
		const char* cTopic = topic.c_str();
		client.publish(cTopic, actionc, mqtt_persistence);
	}  // 'u'
	else
    if (replyBuffer[1] == 'm')    //'m': posizione dimmer//
    {
#ifdef TRACE_SSE
  char buflog[120];
  sprintf(buflog, "[TRC} dimmer position");
  events.send(buflog, "log", millis());
#endif
    // ----------------uab-(address position)--- PUBBLICAZIONE STATO DIMMER -------------------------------------------------------
        char actionc[6];

        sprintf(actionc, "%03u", pct);     // position
        char nomeDevice[5];
        sprintf(nomeDevice, "%02X", device);  // device
        String topic = BRIGHT_STATE;
        topic += nomeDevice;
        const char* cTopic = topic.c_str();
        client.publish(cTopic, actionc, mqtt_persistence);
    }    
	return 0;    
  } // replyLen==4 - 5  && replyBuffer == 0xF3-F4
    
  else
  // --------SCS-----4abcd-(from-to-type-cmd)------- PUBBLICAZIONE STATO DEVICES -------------------------------------------------------

  if ((replyBuffer[1] == 'y') && ((replyBuffer[0] == 0xF5) || (replyBuffer[0] == 0xF6)))
  { // START pubblicazione stato device        [0xF5] [y] 32 00 12 01
#ifdef TRACE_SSE
  char buflog[120];
  sprintf(buflog, "[TRC} mqtt state %d  - cmd %02X", mqttopen, replyBuffer[4]);
  events.send(buflog, "log", millis());
#endif
    char devtype;
    char action;
    String topic = "NOTOPIC";
    String payload = "";

    // ================================ C O M A N D I     S C S ===========================================

    // SCS intero   [7] A8 32 00 12 01 21 A3
    // SCS ridotto [0xF5] [y] 32 00 12 01
    //  termostato [0xF5] [y] B4 01 30 F4      @yb46530f4  SENSORE

    uint8_t devx = 0;
    uint8_t device;
    if (replyBuffer[4] == 0x30)  // <-termostato-----------------------
    {
		if ((replyBuffer[2] == 0xB4) || (replyBuffer[2] == 0xB5))  // <-temperatura rilevata-----B4 zz 30 tt-----B5 zz 30 tt-----------
		{
			device = replyBuffer[3];  
//			devtype = 15;
			devtype = 0;

			const char* cPayload;
			const char* cTopic;
			char nomeDevice[5];
			sprintf(nomeDevice, "%02X", device);  // to
			topic = SENSOR_TEMP_STATE;
			topic += nomeDevice;

			char actionc[5];
			if (replyBuffer[2] == 0xB5)
			  sprintf(actionc, "%03u", (255 + replyBuffer[5])); 
			else
			  sprintf(actionc, "%03u", replyBuffer[5]); 
			actionc[4]=0;
			actionc[3]= actionc[2];
			actionc[2]=',';
			payload = String(actionc);
	        
			cPayload = payload.c_str();
			cTopic = topic.c_str();
			client.publish(cTopic, cPayload, mqtt_persistence);
		}
#ifdef TERMO_NEW
		else
		if ((replyBuffer[2] == 0x90) || (replyBuffer[2] == 0x91)) // setup temperatura --estate/inverno---- 90/91 0z 30 tt  (x5)
		{																   
			device = replyBuffer[3];  
			devtype = 0;

			const char* cPayload;
			const char* cTopic;
			char nomeDevice[5];
			sprintf(nomeDevice, "%02X", device);  // to
			topic = "scs/sensor/temp/setup/";
			topic += nomeDevice;

			char actionc[5];
			sprintf(actionc, "%03u", (5 * replyBuffer[5])); 
			actionc[4]=0;
			actionc[3]= actionc[2];
			actionc[2]=',';
			payload = String(actionc);
	        
			cPayload = payload.c_str();
			cTopic = topic.c_str();
			client.publish(cTopic, cPayload, mqtt_persistence);
		}
		else
		if (replyBuffer[2] == 0xB7)  // stato sonda ------ B7 zz 30 Fx
		{
			device = replyBuffer[3];  
			devtype = 0;

			const char* cPayload;
			const char* cTopic;
			char nomeDevice[5];
			sprintf(nomeDevice, "%02X", device);  // to
			topic = "scs/sonda/state/";
			topic += nomeDevice;

			char actionc[20];
			switch (replyBuffer[5])
			{
// es. device	04 : zona 4	stato sonda    f0	(riscaldamento)	f1	(condizionamento)	f2	(antigelo)	f3	(protezione termica)	ff	(off)
			case 0xf0:
				payload = "riscaldamento";
				break;
			case 0xf1:
				payload = "condizionamento";
				break;
			case 0xf2:
				payload = "antigelo";
				break;
			case 0xf3:
				payload = "protezione";
				break;
			case 0xff:
				payload = "off";
				break;
			default:
				payload = "";
				break;
			}
			if (payload[0] != 0)
			{
				cPayload = payload.c_str();
				cTopic = topic.c_str();
				client.publish(cTopic, cPayload, mqtt_persistence);
			}
		}
#endif
	}
#ifdef TERMO_NEW
	else
	if ((replyBuffer[2] == 0xB7) && (replyBuffer[4] == 0x31))          // stato attuatore --inverno---- B7 8z 31 0x
	{																   // stato fancoil   --inverno---- B7 8z 31 2x
		device = replyBuffer[3];  
		device &= 0x0F;
		devtype = 0;

		const char* cPayload;
		const char* cTopic;
		char nomeDevice[5];
		sprintf(nomeDevice, "%02X", device);  // to
		topic = "scs/actuator/state/";
		topic += nomeDevice;

		char actionc[32];
		switch (replyBuffer[5])
		{
		case 0x00:
			payload = "inverno - acceso";
			break;
		case 0x01:
			payload = "inverno - spento";
			break;
		default:  
			payload = "";
			break;
		}
		if (payload[0] != 0)
		{
			cPayload = payload.c_str();
			cTopic = topic.c_str();
			client.publish(cTopic, cPayload, mqtt_persistence);
		}
		else
		{
			topic = "scs/fancoil/state/";
			topic += nomeDevice;
			switch (replyBuffer[5])
			{
			case 0x20:
				payload = "inverno - spento";
				break;
			case 0x21:
				payload = "inverno - vel. 1";
				break;
			case 0x22:
				payload = "inverno - vel. 2";
				break;
			case 0x23:
				payload = "inverno - vel. 3";
				break;
			default:
				payload = "off";
				break;
			}
			if (payload[0] != 0)
			{
				cPayload = payload.c_str();
				cTopic = topic.c_str();
				client.publish(cTopic, cPayload, mqtt_persistence);
			}
		}
	}
	else
	if ((replyBuffer[2] == 0xB7) && (replyBuffer[4] == 0x32))          // stato attuatore --estate---- B7 8z 31 0x
	{																   // stato fancoil   --estate---- B7 8z 31 2x
		device = replyBuffer[3];  
		device &= 0x0F;
		devtype = 0;

		const char* cPayload;
		const char* cTopic;
		char nomeDevice[5];
		sprintf(nomeDevice, "%02X", device);  // to
		topic = "scs/actuator/state/";
		topic += nomeDevice;

		char actionc[32];
		switch (replyBuffer[5])
		{
		case 0x00:
			payload = "estate - acceso";
			break;
		case 0x01:
			payload = "estate - spento";
			break;
		default:
			payload = "";
			break;
		}
		if (payload[0] != 0)
		{
			cPayload = payload.c_str();
			cTopic = topic.c_str();
			client.publish(cTopic, cPayload, mqtt_persistence);
		}
		else
		{
			topic = "scs/fancoil/state/";
			topic += nomeDevice;

			switch (replyBuffer[5])
			{
			case 0x20:
				payload = "estate - spento";
				break;
			case 0x21:
				payload = "estate - vel. 1";
				break;
			case 0x22:
				payload = "estate - vel. 2";
				break;
			case 0x23:
				payload = "estate - vel. 3";
				break;
			default:
				payload = "off";
				break;
			}
			if (payload[0] != 0)
			{
				cPayload = payload.c_str();
				cTopic = topic.c_str();
				client.publish(cTopic, cPayload, mqtt_persistence);
			}
		}
	}
#endif
    else
    if (replyBuffer[4] == 0x12)  // <-comando----------------------------
    {
      action = replyBuffer[5];
      if (replyBuffer[2] == 0xB1)   // <------------ indirizzato a TUTTI i devices
      {
        char nomeDevice[4];
        String topicPfx;
        const char* cPayload;
        const char* cTopic;
        char typeAll = 99;
        switch (action)
        {
          case 0:
            typeAll = 1;
            payload = "ON";
            topicPfx = SWITCH_STATE;
            break;
          case 1:
            typeAll = 1;
            payload = "OFF";
            topicPfx = SWITCH_STATE;
            break;
          case 8:
            typeAll = 8;
            if ((domoticMode == 'h') || (domoticMode == 'H'))
                payload = "open";
            else
                payload = "OFF";
            topicPfx = COVER_STATE;
            break;
          case 9:
            typeAll = 8;
            if ((domoticMode == 'h') || (domoticMode == 'H'))
                payload = "closed";
            else
                payload = "ON";
            topicPfx = COVER_STATE;
            break;
          case 0x0A:
            typeAll = 8;
            payload = "STOP";
            topicPfx = COVER_STATE;
            break;
        }  // end switch 
        for (devx = 1; devx < MY_ENTITA; devx++)
        {
            if (sistema_scs.elenco[devx].valido == 1)
            {
              devtype = sistema_scs.elenco[devx].tipo_codice;
              if (((typeAll == 1) && (devtype == DIMMER))  // dimmer
                || (typeAll == devtype))  // switch - cover
              {
                strncpy(nomeDevice, sistema_scs.elenco[devx].address, 2);  // to
                topic = topicPfx;
                topic += nomeDevice;
            
#ifdef USE_TCPSERVER
                if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
                {
                  String log = "\r\npub: ";
                  log += topic;
                  log += " ";
                  log += payload;
                  tcpclient.write((char*)&log[0], log.length());
                  tcpclient.flush(); 
                }
    //          else
#endif
                {
                  cPayload = payload.c_str();
                  cTopic = topic.c_str();
                  client.publish(cTopic, cPayload, mqtt_persistence);
                }
              }     //endif dimmer cover
            }   // end valido
        }   // end for devx
        device = 0;
        devtype = 0;
      }                            // <------------ indirizzato a TUTTI i devices
      else
      {
        if (replyBuffer[2] < 0xB0)
          device = replyBuffer[2];  // to
        else
          device = replyBuffer[3];  // from
      }



      if ((action == 0) || (action == 1))       // switch
        devtype = LUCE;
      else if ((action == 3) || (action == 4))      // dimmer
        devtype = 0; // sono solo richieste di +/- , poi arrivera' l'intensita'
      else if ((action == 0x08) || (action == 0x09) || (action == 0x0A))   // cover
        devtype = TAPPARELLA;
      else if ((action & 0x0F) == 0x0D) // da 0x1D a 0x9D
      {
        devtype = DIMMER;
        action >>= 4;
        action *= 10;	// percentuale 10-90

        // --------------- percentuale da 1 a 255 (home assistant) ----------------
        //         if (domoticMode == 'h')   // h=as homeassistant
        {
          int pct = action;
          pct *= 255;      // da 0 a 25500
          pct /= 100;      // da 0 a 100
          action = (char) pct;
        }
      }
      else
        devtype = 0;





      if ((device != 0) && (devtype != 0))
      //      if ((device != 0) && (devtype != 0) && ((device != prevDevice) || (action != prevAction)))
      { // device valido & evitare doppioni
        prevDevice = device;
        prevAction = action;

        char nomeDevice[5];
        sprintf(nomeDevice, "%02X", device);  // to

      // ----------------------------------------- STATO SWITCH --------------------------------------------
        if (devtype == LUCE)
        {
          if (action == 0)
          {
            payload = "ON";
          }
          else if (action == 1)
          {
            payload = "OFF";
          }
          topic = SWITCH_STATE;
          topic += nomeDevice;
        }
        else
        // ----------------------------------------- STATO DIMMER --------------------------------------------
        if ((devtype == DIMMER) ||(devtype == 4))
        {
          char actionc[4];
          sprintf(actionc, "%02u", action);
          payload = String(actionc);
          topic = BRIGHT_STATE;
          topic += nomeDevice;
        }
        else
        // ----------------------------------------- STATO COVER --------------------------------------------
        if ((devtype == TAPPARELLA) || (devtype == 18))
        {
          if (action == 0x08)
          {
            if ((domoticMode == 'h') || (domoticMode == 'H'))
              payload = "open";
            else
              payload = "OFF";
          }
          else if (action == 0x09)
          {
            if ((domoticMode == 'h') || (domoticMode == 'H'))
              payload = "closed";
            else
              payload = "ON";
          }
          else if (action == 0x0A)
          {
            payload = "STOP";
          }

          topic = COVER_STATE;
          topic += nomeDevice;
        } // devtype=8
  // ----------------------------------------------------------------------------------------------------


    // ======================================== P U B B L I C A Z I O N E ===========================================
          const char* cTopic = topic.c_str();
          
          
#ifdef USE_TCPSERVER
          if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
          {
            String log = "\r\npub: ";
            log += topic;
            log += " ";
            log += payload;
            tcpclient.write((char*)&log[0], log.length());
            tcpclient.flush(); 
          }
#endif
          
          if (payload == "")
          {
              char cPayload[24];
              sprintf(cPayload, "UNKNOWN: %02X %02X %02X %02X", replyBuffer[1], replyBuffer[2], replyBuffer[3], replyBuffer[4]);  // to
              client.publish(cTopic, cPayload, 0);
          }
          else
          {
              const char* cPayload = payload.c_str();
              client.publish(cTopic, cPayload, mqtt_persistence);
          }
      }       // evitare doppioni & device valido
    } // <-replyBuffer[4] == 0x12---------------------comando----------------------------
  // ----------------------------------------------------------------------------------------------------
    else
      
      
      
  // ----------------------------------pubblicazione stati GENERIC device SCS (to & from)------------------------
      { // generic device

          device = replyBuffer[2];            // TO
          devx = CercaAddress(device);
          
          if ((devx != 0xFF) && (sistema_scs.elenco[devx].tipo_codice == GENERICO))  
          {	          // device generic censito
            topic = GENERIC_TO;
            topic += sistema_scs.elenco[devx].address;
            const char* cTopic = topic.c_str();
            char cPayload[8];
          
#ifdef USE_TCPSERVER
            if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
            {
              String log = "\r\npub: ";
              log += topic;
              log += " ";
              log += payload;
              tcpclient.write((char*)&log[0], log.length());
              tcpclient.flush(); 
            }
#endif
          
            sprintf(cPayload, "%02X%02X%02X", replyBuffer[3], replyBuffer[4], replyBuffer[5]);
            client.publish(cTopic, cPayload, mqtt_persistence);
          }

          device = replyBuffer[3];            // FROM
          devx = CercaAddress(device);
          if ((devx != 0xFF) && (sistema_scs.elenco[devx].tipo_codice == GENERICO))  
          {	          // device generic censito
            topic = GENERIC_FROM;
            topic += sistema_scs.elenco[devx].address;
            const char* cTopic = topic.c_str();
            char cPayload[24];
            sprintf(cPayload, "%02X%02X%02X", replyBuffer[2], replyBuffer[4], replyBuffer[5]);
            client.publish(cTopic, cPayload, mqtt_persistence);
          }

      } // generic device
	return 1;
}
	return 1;
}







// =====================================================================================================


#ifdef MY_DOMOTIC
// MY_DOMOTIC_NEW


void scs_tratta_messaggi_per_my_domotic_B1(uint8_t des, uint8_t cmd, uint8_t arg)
{
    char actionc[16];
    String payload = "";
    int valore = -1;
/*
    uint8_t des  = replyBuffer[2]; // Destinazione (es. 0x32, 0x91, 0xB4, 0xB5...)
    uint8_t prov = replyBuffer[3]; // Provenienza / Zona (es. 0x01, 0x02...)
    uint8_t cmd  = replyBuffer[4]; // Tipo di comando (es. 0x12 o 0x30)
    uint8_t arg  = replyBuffer[5]; // Argomento reale (es. 0x00, 0x01, 0xF4...)
*/
    if ((des == 0xB1) && (cmd == 0x12))   // <------------ indirizzato a TUTTI i devices
    {
      char typeAll = 0;
      switch (arg)
      {
        case 0:
          typeAll = 1;
          payload = "ON";
          break;
        case 1:
          typeAll = 1;
          payload = "OFF";
          break;
        case 8:
          typeAll = 8;
          payload = "SALITA";
          break;
        case 9:
          typeAll = 8;
          payload = "DISCESA";
          break;
        case 0x0A:
          typeAll = 8;
          payload = "STOP";
          break;
        default:
          if ((arg & 0x0F) == 0x0D) // da 0x1D a 0x9D
          {
            typeAll = 3;
            arg >>= 4;
            arg *= 10;	// percentuale 10-90
            sprintf(actionc, "%02u", arg);
            valore = arg;
          }
          break;
      }
//uint8_t corrispondenza[5] = {LUCE,TERMOSTATO,DIMMER,TAPPARELLA,TAPPARELLA_PCT};
//enum TipoEntita { ENTI_INTERRUTTORE, ENTI_SENSORE, ENTI_SLIDER, ENTI_TAPPARELLA, ENTI_TAPPARELLA_PCT };
#ifdef TRACE_SSE
    char buflog[120];
	  sprintf(buflog, "[TRC} domotic new  - msg B1: typeAll %d", typeAll);
	  events.send(buflog, "log", millis());
#endif

      for (int i = 0; i < MY_ENTITA; i++) 
      {
          if (((typeAll == 1)  // on/off
                && (sistema_scs.elenco[i].tipo_codice == LUCE))
          ||  ((typeAll == 8)  // up/down/stop
                && ((sistema_scs.elenco[i].tipo_codice == TAPPARELLA) || (sistema_scs.elenco[i].tipo_codice == TAPPARELLA_PCT))))
          {                
             strcpy(sistema_scs.elenco[i].stato, payload.c_str());
			       if (eventity.count() != 0)
	               eventity.send(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address, millis());
#ifdef DEBUG_EVENTITY
  String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + String(sistema_scs.elenco[i].address) + " : " + payload;
  events.send(rispostaLog.c_str(), "log", millis());
#endif
          }
          else
          if ((typeAll == 1)  // on/off
                && (sistema_scs.elenco[i].tipo_codice == DIMMER))
          {                
            strcpy (sistema_scs.elenco[i].stato, payload.c_str());
			if (eventity.count() != 0)
	            eventity.send(String(sistema_scs.elenco[i].stato) + "|" + String(sistema_scs.elenco[i].valore_corrente), String(sistema_scs.elenco[i].address), millis());

#ifdef DEBUG_EVENTITY
  String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + sistema_scs.elenco[i].id + " : " + payload;
  events.send(rispostaLog.c_str(), "log", millis());
#endif
          }
        }
	}
}









// ----------------------------------------------------------------------------------------------------------------------------------------------
void sse_ha_update(const char * stato, const char * id)
{
#ifdef DEBUG_EVENTITY
    String rispostaLog = "[" + String(millis()/100) + "s] [CMD] === id: " + id + " : " + stato;
    events.send(rispostaLog.c_str(), "log", millis());
#endif
	if (eventity.count() != 0)
		eventity.send(stato, id, millis());
}
// ----------------------------------------------------------------------------------------------------------------------------------------------
void invia_nuova_card_info_sse(int indx) {
	if (eventity.count() == 0) return;

    int8_t p_info = sistema_scs.elenco[indx].ptr_info;
    if (p_info < 0 || p_info >= MAX_INFO) return;

	char payload[256];
	snprintf(payload, sizeof(payload), 
		"{\"id\": \"%s_INF\",\"nome\": \"%s (info)\",\"tipo\":5,\"stato\": \"%s|%s|%s|%s|%s\"}", 
         sistema_scs.elenco[indx].address,
		 sistema_scs.elenco[indx].descrizione,
         cardinfo[p_info].info_0,
         cardinfo[p_info].info_1,
         cardinfo[p_info].info_2,
         cardinfo[p_info].info_3,
         cardinfo[p_info].info_4);
		 
#ifdef DEBUG_EVENTITY
    String rispostaLog = "[" + String(millis()/100) + "s] [CMD] ==NEW_CARD== " + payload;
    events.send(rispostaLog.c_str(), "log", millis());
#endif
	if (eventity.count() != 0)
	{
		eventity.send(payload, "NEW_CARD", millis());
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}
// ----------------------------------------------------------------------------------------------------------------------------------------------
void invia_aggiornamento_info_sse(int indx, int rownr) {
	if (eventity.count() == 0) return;

    int8_t p_info = sistema_scs.elenco[indx].ptr_info;
    if (p_info < 0 || p_info >= MAX_INFO) return;

    const char *testo_riga = NULL;
    switch (rownr) {
        case 0: testo_riga = cardinfo[p_info].info_0; break;
        case 1: testo_riga = cardinfo[p_info].info_1; break;
        case 2: testo_riga = cardinfo[p_info].info_2; break;
        case 3: testo_riga = cardinfo[p_info].info_3; break;
        case 4: testo_riga = cardinfo[p_info].info_4; break;
    }

	char id[16];
	snprintf(id, sizeof(id),"%s_INF", sistema_scs.elenco[indx].address);

	char payload[64]; // Bastano pochi byte per una riga singola
	snprintf(payload, sizeof(payload), "%d:%s", rownr, testo_riga);

#ifdef DEBUG_EVENTITY
    String rispostaLog = "[" + String(millis()/100) + "s] [CMD] ==UPD_CARD== : " + id + " -> " + payload;
    events.send(rispostaLog.c_str(), "log", millis());
#endif
	if (eventity.count() != 0)
		eventity.send(payload, id, millis());
}
// ----------------------------------------------------------------------------------------------------------------------------------------------



// ----------------------------------------------------------------------------------------------------------------------------------------------
uint8_t scs_tratta_messaggi_dispositivi_ignoti(uint8_t addr_num, uint8_t des, uint8_t cmd, uint8_t arg)
{
/*

// Struttura dati per il singolo elemento della lista con i limiti richiesti
struct Elemento {
    char address[3];     // 2 caratteri + terminatore null
    char tipo[4];        // 3 caratteri + terminatore null
    char descrizione[21];// 20 caratteri + terminatore null
    char valore[4];      // 3 caratteri + terminatore null
    bool valido = false; // Flag per verificare se lo slot è occupato    
    AGG_VAL bits;        // in OR: 0x01=riga cancellata   0x02=riga nuova                0x10=chiave cambiata 
                         //        0x20=tipo aggiornato   0x40=descrizione aggiornata    0x80=valore cambiato   
};
Elemento sistema_scs.elenco[MY_ENTITA];
bool sistema_scs.tapparelle_percentuale = false; // Stato memorizzato in RAM
// ----------------------------------------------------------------------------------------------------------------

*/




    uint8_t t = 0; // Tipo di dispositivo da censire (0 = non censito, 1 = luce, 2 = dimmer, 3 = tapparella, 4 = termostato)
    if (cmd == 0x12) 
	{ 
        if ((arg == 0x00) || (arg == 0x01))
			t = LUCE;
		else
		if ((arg & 0xF) == 0xD)
			t = DIMMER;
		else
        if ((arg == 0x08) || (arg == 0x09) || (arg == 0x0A))
			t = (sistema_scs.tapparelle_percentuale == true) ? TAPPARELLA_PCT : TAPPARELLA;
	}
	else
	if ((cmd == 0x30) && ((des == 0xB4) || (des == 0xB5))) 
			t = TERMOSTATO;


	if (t != 0)
	{
// dispositivi ignoti - caricarli sulla "sistema_scs.elenco[]"

		// --- AGGIUNGI NUOVO ELEMENTO IN RAM ---

		int i = MY_ENTITA-1;
        // Trova il primo slot vuoto
        for(int ix = 0; ix < MY_ENTITA; ix++) 
        {
          if(sistema_scs.elenco[ix].valido == 0) 
          {
            i = ix;
            sistema_scs.elenco[i].id = addr_num;
            sistema_scs.elenco[i].bits.Val = 0;
            sistema_scs.elenco[i].bits.nuova = 1;  // <-----------------------------------
            break;
          }
        }

		sprintf(sistema_scs.elenco[i].address, "%02X", addr_num);

        sprintf(sistema_scs.elenco[i].descrizione, "dispositivo scs %02X", addr_num);  // device
        sistema_scs.elenco[i].tipo_codice = t;

//#ifdef TRACE_SSE
        if (events.count() != 0) 
        {
	      char buflog[64];
	      sprintf(buflog, "[TRC} == auto -> %02X, tipo %s", addr_num, codifica_codice_a_tipo(t));
	      events.send(buflog, "log", millis());
        }
//#endif

        set_device_default(i, 1);
		if (t == TAPPARELLA_PCT)
            strcpy(sistema_scs.elenco[i].valore_max, "100");
        else
            strcpy(sistema_scs.elenco[i].valore_max, "0");

//	    rigenera_tabella_indice();

        i = CercaAddress(addr_num);

        return i;
	}
    else
    {
        if (events.count() != 0) 
        {
	      char buflog[64];
          sprintf(buflog, "[TRC} == auto unknown -> %02X, to %02X : %02X %02X", addr_num, des, cmd, arg);
	      events.send(buflog, "log", millis());
        }
    }
    return 0xFF;
}
// ----------------------------------------------------------------------------------------------------------------------------------------------



// ----------------------------------------------------------------------------------------------------------------------------------------------
void scs_tratta_messaggi_per_my_domotic_new(void)
{
    // ================================ C O M A N D I     S C S ===========================================

    // SCS intero   [7] A8 32 00 12 01 21 A3
    // SCS ridotto [0xF5] [y] 32 00 12 01
    //  termostato [0xF5] [y] B4 01 30 F4

#ifdef TRACE_SSE
  char buflog[120];
  sprintf(buflog, "[TRC} = domotic SSE %d  - cmd %02X %c", eventity.count(), replyBuffer[0], replyBuffer[1]);
  events.send(buflog, "log", millis());
#endif

  bool modificato = false;

  // ---------------------u-posizione tapparelle o dimmer %---------------------------------------------------------------------
  if (replyBuffer[0] == 0xF3)    
  { 
    char device = replyBuffer[2];
    int pct = replyBuffer[3];
    if (replyBuffer[1] == 'u')    //'u': posizione tapparelle//
    {
    // ----------------uab-(address position)--- PUBBLICAZIONE STATO COVERPCT -------------------------------------------------------
      int i = CercaAddress(device);
      if (i != 0xFF) {
        sprintf(sistema_scs.elenco[i].valore_corrente, "%d", pct);
	    if (eventity.count() != 0)
        {
         char stato[32];
         sprintf(stato, "%d|%s", pct, sistema_scs.elenco[i].stato);
		 sse_ha_update(stato, sistema_scs.elenco[i].address);
        }
  	  }
    }  
	return;
  }  


//  if ((eventity.count() != 0) && (replyBuffer[1] == 'y') && ((replyBuffer[0] == 0xF5) || (replyBuffer[0] == 0xF6)))
  if ((replyBuffer[1] == 'y') && ((replyBuffer[0] == 0xF5) || (replyBuffer[0] == 0xF6)))
  { 
    uint8_t des  = replyBuffer[2]; // Destinazione (es. 0x32, 0x91, 0xB4, 0xB5...)
    uint8_t prov = replyBuffer[3]; // Provenienza / Zona (es. 0x01, 0x02...)
    uint8_t cmd  = replyBuffer[4]; // Tipo di comando (es. 0x12 o 0x30)
    uint8_t arg  = replyBuffer[5]; // Argomento reale (es. 0x00, 0x01, 0xF4...)

	uint8_t addr_num = (des < prov) ? des : prov;  //04
    if (addr_num == 0)
        addr_num = (des < prov) ? prov : des;

	uint8_t i = CercaAddress(addr_num); //ff

//    if ((cmd == 0x30) && ((des == 0xB4) || (des == 0xB5))) 

	if ((i == 0xFF ) && ((cmd == 0x30) || (cmd == 0x31) || (cmd == 0x32))) //ok
	{
		prov &= 0x0F;   //04
		addr_num = prov; //04
		i = CercaAddress(addr_num); //ff
	}
	else
	if (des == 0xB1)
	{
		scs_tratta_messaggi_per_my_domotic_B1(des, cmd, arg);
		return;
	}
#ifdef TRACE_SSE
    if (events.count() != 0) 
    {
      char buflog[64];
      sprintf(buflog, "[TRC} == auto? i: %02x -> %02X, to %02X : %02X %02X", i, addr_num, des, cmd, arg);
      events.send(buflog, "log", millis());
    }
#endif
// ------------------------------------------------------------------------------------
	if ((i == 0xFF) && (autocensimento) && (addr_num != 0) && (addr_num < 0xB0))
	{
		i = scs_tratta_messaggi_dispositivi_ignoti(addr_num, des, cmd, arg); // 04 b5 30 1c
	}
	if (i == 0xFF) 
		return;
// ------------------------------------------------------------------------------------



    uint8_t t = sistema_scs.elenco[i].tipo_codice;
    int8_t  p_info = sistema_scs.elenco[i].ptr_info;
            
    char addr_cercato[3];
    snprintf(addr_cercato, sizeof(addr_cercato), "%02X", addr_num);

#ifdef TRACE_SSE
	  char buflog[64];
	  sprintf(buflog, "[TRC} == cmd %02X - des %02X - idx %d - tipo %d", cmd, des, i, t);
	  events.send(buflog, "log", millis());
#endif

  // =================================================================
    // 🟢 FLUSSO 1: AUTOMAZIONE LUCI E TAPPARELLE (CMD 0x12)
    // =================================================================
    if (cmd == 0x12)
    {
        // --- INTERPRETAZIONE AUTOMAZIONE LUCI / DIMMER ---
        if (t == LUCE || t == DIMMER) { 
            if (arg == 0x00) {
                strcpy(sistema_scs.elenco[i].stato,"ON");
                if (eventity.count() != 0)
                {
                    sse_ha_update(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address);
                }
            } else if (arg == 0x01) {
                strcpy(sistema_scs.elenco[i].stato,"OFF");
        		if (eventity.count() != 0)
				{
				    sse_ha_update(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address);
				}
            } else if (t == DIMMER && arg >= 0x1D && arg <= 0x9D) {
                int step = ((arg >> 4) & 0x0F) - 1; 
                int pct = step * 12;
                if (pct > 100) pct = 100;
                strcpy(sistema_scs.elenco[i].stato,"ON");
                sprintf(sistema_scs.elenco[i].valore_corrente, "%d", pct);
                if (eventity.count() != 0)
                {
                  char stato[32];
                  sprintf(stato, "%s|%d", sistema_scs.elenco[i].stato, pct);
                  sse_ha_update(stato, sistema_scs.elenco[i].address);
                }
        	}
        }
        // --- INTERPRETAZIONE TAPPARELLE ---
        else if (t == TAPPARELLA || t == TAPPARELLA_PCT) 
	    {
          int target;
          int corrente = atoi(sistema_scs.elenco[i].valore_corrente);
          if (arg == 0x08) {
              // Eseguiamo l'azione solo se non eravamo già in SALITA
              if  (strcmp(sistema_scs.elenco[i].stato, "SALITA") != 0) 
              {
                  strcpy(sistema_scs.elenco[i].stato,"SALITA");
                  if (eventity.count() != 0)
                  {
                     sse_ha_update(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address);
                  }
              }
              target = 100;
        } else if (arg == 0x09) {
            // Eseguiamo l'azione solo se non eravamo già in DISCESA
              if  (strcmp(sistema_scs.elenco[i].stato, "DISCESA") != 0) 
              {
                  strcpy(sistema_scs.elenco[i].stato,"DISCESA");
                  if (eventity.count() != 0)
                  {
                    sse_ha_update(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address);
                  }
               }
               target = 0;
        } else if (arg == 0x0A) {
            // 🟢 FILTRO IDEMPOTENZA (STOP LOOP): 
            // Se lo stato è già "STOP" (perché impostato precedentemente a fine corsa),
            // ignoriamo l'eco del comando 0x0A. 'modificato' rimarrà false e il blocco non verrà rieseguito!
            if  (strcmp(sistema_scs.elenco[i].stato, "STOP") != 0) 
            {
                strcpy(sistema_scs.elenco[i].stato,"STOP");
                if (eventity.count() != 0)
                {
                  if (t == TAPPARELLA_PCT)
                  {
                      char stato[32];
                      sprintf(stato, "%s|%s", sistema_scs.elenco[i].valore_corrente, sistema_scs.elenco[i].stato);
                      sse_ha_update(stato, sistema_scs.elenco[i].address);
                  }
                  else
                      sse_ha_update(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address);
                }
            }
            target = corrente;
		 }
       }
	}
	// =================================================================
	// 🟢 FLUSSO 2: TERMOREGOLAZIONE (CMD 0x30)
	// =================================================================
	else if (cmd == 0x30) 
	{
#ifdef TRACE_SSE
	  char buflog[64];
	  sprintf(buflog, "[TRC} == TT cmd %02X - des %02X - idx %d - tipo %d", cmd, des, i, t);
	  events.send(buflog, "log", millis());
#endif
		if (t == TERMOSTATO)  
		{
			// 🌡️ Risposta Temperatura <= 25.5°C
			if (des == 0xB4) {
				double temp = (double)arg / 10.0;
				char nuova_temp_str[8];
				snprintf(nuova_temp_str, sizeof(nuova_temp_str), "%.1f", temp);
				if (strcmp(sistema_scs.elenco[i].stato,nuova_temp_str) != 0) {
					strcpy(sistema_scs.elenco[i].stato,nuova_temp_str);
					if (eventity.count() != 0)
					{
					   sse_ha_update(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address);
					}
				}
			}
			// 🌡️ Risposta Temperatura > 25.5°C
			else if (des == 0xB5) 
			{
				double temp = 25.5 + ((double)arg / 10.0);
				char nuova_temp_str[8];
				snprintf(nuova_temp_str, sizeof(nuova_temp_str), "%.1f", temp);
				if (strcmp(sistema_scs.elenco[i].stato,nuova_temp_str) != 0) {
					strcpy(sistema_scs.elenco[i].stato,nuova_temp_str);
					if (eventity.count() != 0)
					{
					   sse_ha_update(sistema_scs.elenco[i].stato, sistema_scs.elenco[i].address);
					}
				}
			}
			else
			{
				if (p_info == -1)  
				{
					if ((numeroInfo < MAX_INFO) && (cardinfo[numeroInfo].id < 0)) 
					{
						cardinfo[numeroInfo].id = i;
						sistema_scs.elenco[i].ptr_info = numeroInfo;
						p_info = numeroInfo;
						invia_nuova_card_info_sse(i);
						numeroInfo++;
					}
				}
				if ((des == 0x90) || (des == 0x91)) // setup temperatura --estate/inverno---- 90/91 0z 30 tt  (x5)
				{
					double temp = ((double)arg /2.0);
#ifdef INFO_LABEL
					snprintf(cardinfo[p_info].info_0, INFO_LEN, "setup %.1f °C", temp);
#else
					snprintf(cardinfo[p_info].info_0, INFO_LEN, "%.1f °C", temp);
#endif
					invia_aggiornamento_info_sse(i, 0);
				}
				else
				if (des == 0xB7)  // stato sonda ------ B7 zz 30 Fx
				{
					switch (arg)
					{
		// es. device	04 : zona 4	stato sonda    f0	(riscaldamento)	f1	(condizionamento)	f2	(antigelo)	f3	(protezione termica)	ff	(off)
					case 0xf0:
						snprintf(cardinfo[p_info].info_1, INFO_LEN, "riscaldamento");
						modificato = true;
						break;
					case 0xf1:
						snprintf(cardinfo[p_info].info_1, INFO_LEN,"condizionamento");
						modificato = true;
						break;
					case 0xf2:
						snprintf(cardinfo[p_info].info_1, INFO_LEN, "antigelo");
						modificato = true;
						break;
					case 0xf3:
						snprintf(cardinfo[p_info].info_1, INFO_LEN, "protezione");
						modificato = true;
						break;
					case 0xff:
						snprintf(cardinfo[p_info].info_1, INFO_LEN, "OFF");
						modificato = true;
						break;
					}
					if (modificato)
					{
						invia_aggiornamento_info_sse(i, 1);
						modificato = false;
					}
				}
			}
		}
	}


	else if ((cmd == 0x31) && (des == 0xB7) && (t == TERMOSTATO))
	{
		if (p_info == -1) 
		{
				if ((numeroInfo < MAX_INFO) && (cardinfo[numeroInfo].id < 0)) 
				{
					cardinfo[numeroInfo].id = i;
					sistema_scs.elenco[i].ptr_info = numeroInfo;
					p_info = numeroInfo;
					invia_nuova_card_info_sse(i);
					numeroInfo++;
				}
		}
		switch (arg)
		{
		  // stato attuatore --inverno---- B7 8z 31 0x
		case 0x00:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "att. inverno ON");
#else
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "inverno ON");
#endif
			modificato = true;
			break;
		case 0x01:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "att. inverno OFF");
#else
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "inverno OFF");
#endif
			modificato = true;
			break;
		}
		if (modificato)
		{
			invia_aggiornamento_info_sse(i, 2);
			modificato = false;
		}
		switch (arg)
		{
		  // stato fancoil   --inverno---- B7 8z 31 2x
		case 0x20:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil inv. OFF");
#else
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "inverno OFF");
#endif
			modificato = true;
			break;
		case 0x21:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil inv. V 1");
#else
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "inverno Vel. 1");
#endif
			modificato = true;
			break;
		case 0x22:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil inv. V 2");
#else
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "inverno Vel. 2");
#endif
			modificato = true;
			break;
		case 0x23:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil inv. V 3");
#else
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "inverno Vel. 3");
#endif
			modificato = true;
			break;
		default:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil OFF");
#else
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "OFF");
#endif
			modificato = true;
			break;
		}
		if (modificato)
		{
			invia_aggiornamento_info_sse(i, 3);
			modificato = false;
		}
	}
	else if ((cmd == 0x32) && (des == 0xB7) && (t == TERMOSTATO))
	{
		if (p_info == -1) 
		{
				if ((numeroInfo < MAX_INFO) && (cardinfo[numeroInfo].id < 0)) 
				{
					cardinfo[numeroInfo].id = i;
					sistema_scs.elenco[i].ptr_info = numeroInfo;
					p_info = numeroInfo;
					invia_nuova_card_info_sse(i);
					numeroInfo++;
				}
		}
		switch (arg)
		{
		  // stato attuatore --estate---- B7 8z 32 0x
		case 0x00:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "att. estate ON");
#else
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "estate ON");
#endif
			modificato = true;
			break;
		case 0x01:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "att. estate OFF");
#else
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "estate OFF");
#endif
			modificato = true;
			break;
		}
		if (modificato)
		{
			invia_aggiornamento_info_sse(i, 2);
			modificato = false;
		}
		switch (arg)
		{
		  // stato fancoil   --estate---- B7 8z 32 2x
		case 0x20:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil est. OFF");
#else
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "estate OFF");
#endif
			modificato = true;
			break;
		case 0x21:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil est. V 1");
#else
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "estate Vel. 1");
#endif
			modificato = true;
			break;
		case 0x22:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil est. V 2");
#else
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "estate Vel. 2");
#endif
			modificato = true;
			break;
		case 0x23:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil est. V 3");
#else
			snprintf(cardinfo[p_info].info_2, INFO_LEN, "estate Vel. 3");
#endif
			modificato = true;
			break;
		default:
#ifdef INFO_LABEL
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "fancoil OFF");
#else
			snprintf(cardinfo[p_info].info_3, INFO_LEN, "OFF");
#endif
			modificato = true;
			break;
		}
		if (modificato)
		{
			invia_aggiornamento_info_sse(i, 3);
			modificato = false;
		}
	}
 } // <-replyBuffer[4] == 0x12---------------------comando----------------------------
  // ----------------------------------------------------------------------------------------------------


/* dispositivi generici non trattati per ora

  else
      
  // ----------------------------------pubblicazione stati GENERIC device SCS (to & from)------------------------
      { // generic device

          device = replyBuffer[2];            // TO
          devx = ixOfDevice(device);
          
          if ((devx) && (device_BUS_id[devx].deviceType == 11))  
          {	          // device generic censito
            char nomeDevice[3];
            sprintf(nomeDevice, "%02X", device);  // to
            topic = GENERIC_TO;
            topic += nomeDevice;
            const char* cTopic = topic.c_str();
            char cPayload[8];
          
          
            sprintf(cPayload, "%02X%02X%02X", replyBuffer[3], replyBuffer[4], replyBuffer[5]);
            client.publish(cTopic, cPayload, mqtt_persistence);
          }

          device = replyBuffer[3];            // FROM
          devx = ixOfDevice(device);
          if ((devx) && (device_BUS_id[devx].deviceType == 11))  
          {	          // device generic censito
            char nomeDevice[3];
            sprintf(nomeDevice, "%02X", device);  // from
            topic = GENERIC_FROM;
            topic += nomeDevice;
            const char* cTopic = topic.c_str();
            char cPayload[24];
            sprintf(cPayload, "%02X%02X%02X", replyBuffer[2], replyBuffer[4], replyBuffer[5]);
            client.publish(cTopic, cPayload, mqtt_persistence);
          }

      } // generic device
       */

  // ----------------------------------------------------------------------------------------------------
}










#endif



// -----------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  now = (signed int)millis();

#ifdef DEBUG
  counter++;
  counter = 0;
#endif

  if  (countRestart > 0)
  {
    countRestart--;
    if (countRestart == 0)  ESP.restart();
  }

if (sm_picprog == PICPROG_FREE) {
// ========================================== NORMAL RUN ====================================================

#ifdef MY_TABELLA
    if (flagRefreshDevices == 1)
    {
        if (events.count() != 0) 
            events.send("Entità ricaricate - fai refresh pagina /dispositivi ", "log", millis());
        delay(5);
    }
#endif
    flagRefreshDevices =  0;
    if (delay_internal)
        delay_internal--;
    if ((events.count() != 0) && (internal != prev_internal) && (delay_internal == 0)) //  && (replyBuffer[1] |= 0))
	{
		prev_internal = internal;
		if (internal)
			events.send("In funzione - modalità server - OK", "stato", millis());
		else
			events.send("In funzione - modalità TEST - OK", "stato", millis());
	}

      // ==================================== TCP server ====================================================
#ifdef USE_TCPSERVER
   if (tcpopen) {

   if (tcpserver.hasClient()) {
    // client is connected
    if (!tcpclient || !tcpclient.connected())
    {
      if(tcpclient) tcpclient.stop();          // client disconnected
      tcpclient = tcpserver.available(); // ready for new client
#ifdef DEBUG
      Serial.println("/n TCP CON "); // connesso !
#endif
    } else 
    {
      tcpserver.available().stop();  // have client, block new conections
#ifdef DEBUG
      Serial.println("/nTCP STOP ");
#endif
    }
   }
  
   if (tcpclient && tcpclient.connected())
    tcpopen = 2;
   else
    tcpopen = 1;
  
  
   if (tcpclient && tcpclient.connected() && tcpclient.available())
   {
    char tcpBuffer[255];
    int  buflen;
    int  pos = 0;
    WORD_VAL maxp;
    maxp.Val = 0;

    char device  = 0;
    char devtype = 0;
    char deviceX = 0;
    char nomeDevice[6];
    char AlexaDescr[21];

    // client input processing
    while (tcpclient.available())
    {
#ifdef DEBUG
      Serial.print(".");
#endif
      tcpBuffer[pos] = (uint8_t)tcpclient.read();
      pos++;
    }
    buflen = pos;
    tcpBuffer[pos] = 0;
    
#ifdef DEBUG 
    Serial.print("\r\ntcp rx: ");
    Serial.write(&tcpBuffer[0], buflen);
#endif

// ------------------------------------------------------------------------------------------------------
    if (memcmp(tcpBuffer, "#request",8) == 0)
// ------------------------------------------------------------------------------------------------------
    {
      String busid = tcpJarg(tcpBuffer,(char *) "\"device\""); // bus id
      deviceX = CercaAddress(busid.c_str()); // pointer device reale o temporaneo
      if (deviceX == 0xFF)
      {
          device  = atoi(busid.c_str());
          devtype = LUCE;
      }
      else
      {
          device  = sistema_scs.elenco[deviceX].id;
          devtype = sistema_scs.elenco[deviceX].tipo_codice;
      }
      
      String sreq = tcpJarg(tcpBuffer,(char *) "\"request\""); // on-off-up-down-stop-nn%
      String scmd = tcpJarg(tcpBuffer,(char *) "\"command\""); // 0xnn

      if (scmd != "")
      {
        char command = aConvert(scmd);
        requestBuffer[requestLen++] = PIC_REQ;
        requestBuffer[requestLen++] = 'y';   // 0x79 (@y: invia a pic da tcp #request cmd standard da inviare sul bus)

// comando §y<destaddress><source><type><command>
        requestBuffer[requestLen++] = device; // to   device
        requestBuffer[requestLen++] = 0x00;   // from device
        requestBuffer[requestLen++] = 0x12;   // type:command
        requestBuffer[requestLen++] = command;// command
      }
    }	// #request
    else
// ------------------------------------------------------------------------------------------------------
    if (memcmp(tcpBuffer, "#putdevice",10) == 0)   // upload device
// ------------------------------------------------------------------------------------------------------
    {
      String busid = tcpJarg(tcpBuffer,(char *) "\"device\"");
      if (busid != "")
      {
        busid += "  ";
        deviceX = CercaAddressNew(busid.c_str()); // pointer device esistente o nuovo
        if (deviceX != 0xFF)
        {
          device  = sistema_scs.elenco[deviceX].id;

          String alexadescr = tcpJarg(tcpBuffer,(char *) "\"descr\"");
          strcpy(sistema_scs.elenco[deviceX].descrizione, alexadescr.c_str());
        
          String stype = tcpJarg(tcpBuffer,(char *) "\"type\"");
          devtype = (char) stype.toInt();
/*
//          if ((devtype == 8) && (domotic_options == 0x03))
//             devtype = 18;          
//          if ((devtype == 9) && (domotic_options == 0x03))
//             devtype = 19;          
*/
          sistema_scs.elenco[deviceX].tipo_codice = devtype;
             
          if ((alexaParam == 'y' ) && (devtype < 18))		// w6 - alexa non ha bisogno dei types 18 19
          {
///            fauxmo.renameDevice(&edesc[0], alexadescr.c_str());  <----- ???
          }
          
          if ((devtype == 9) || (devtype == 19))			// w6 - aggiorna tapparelle pct su pic
          {
            String smaxpos = tcpJarg(tcpBuffer,(char *) "\"maxp\"");
            char *ch;
            maxp.Val = (int)strtoul(&smaxpos[0], &ch, 10);
            strcpy(sistema_scs.elenco[deviceX].valore_max, smaxpos.c_str());
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '8';
            requestBuffer[requestLen++] = device;     // device id
            requestBuffer[requestLen++] = devtype;    // device type
            requestBuffer[requestLen++] = maxp.byte.HB;    // max position H
            requestBuffer[requestLen++] = maxp.byte.LB;    // max position L
            immediateSend();
            immediateReceive('k');
            delay(100);
          }
          else
            maxp.Val = 0;
            

#ifdef DEBUG 
          sprintf(tcpBuffer, "{\"device\":\"%02X\",\"type\":\"%d\",\"maxp\":\"%d\"\
                            ,\"descr\":\"%s\"}",device,devtype,maxp.Val,AlexaDescr);
#endif
        } // deviceX > 0
      }  // busid != ""
///      else
///         sprintf(tcpBuffer, "#ok");
      
      String cover = tcpJarg(tcpBuffer,(char *) "\"coverpct\"");
      if (cover == "false")
      {
        requestBuffer[requestLen++] = PIC_REQ;
        requestBuffer[requestLen++] = 'U';
        requestBuffer[requestLen++] = '9';
        immediateSend();
        delay(300);
        immediateReceive('k');
        delay(100);
        sistema_scs.tapparelle_percentuale = false;
      }  // cover == "false"
      else
      if (cover == "true")
      {
        sistema_scs.tapparelle_percentuale = true;
      }  // cover == "false"

      String devclear = tcpJarg(tcpBuffer,(char *) "\"devclear\"");
      if (devclear == "true")
      {
/*
        for (int i = 0; i < MY_ENTITA; ++i)
        {
          EEPROM.write((int) i * E_ALEXA_DESC_LEN + E_ALEXA_DESC_DEVICE, 0);
  	      device_BUS_id[i].Val = 0;
        }
        WriteEEP((char) 0, (int) E_MQTT_TABDEVICES, (int) MY_ENTITA * E_MQTT_TABLEN);
*/
      }  // devclear == "true"
      

      sprintf(tcpBuffer, "#ok");
      buflen = 0;
      while (tcpBuffer[buflen]) buflen++;
      tcpclient.write(tcpBuffer, buflen);
      
//    EEPROM.commit();
      tcpclient.flush();
    }	// #putdevice
    else
// ------------------------------------------------------------------------------------------------------
    if (memcmp(tcpBuffer, "#getdevall",10) == 0) // download devices
// ------------------------------------------------------------------------------------------------------
    {
      deviceX = 1;
      sistema_scs.tapparelle_percentuale = false;
      for (deviceX = 0; deviceX < MY_ENTITA; deviceX++)
      {
        if (sistema_scs.elenco[deviceX].valido == 1)
        {
          device = sistema_scs.elenco[deviceX].id;
          devtype = sistema_scs.elenco[deviceX].tipo_codice;
          maxp.Val = 0;
          if ((devtype == 9) || (devtype == 19))
          {
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '6';
            requestBuffer[requestLen++] = device;
            immediateSend();
            char m = immediateReceive('[');
            sistema_scs.tapparelle_percentuale = true;
            if (m > 8)
            {
              maxp.Val = replyBuffer[2] | replyBuffer[1] << 8;
            }
          }  // devtype == 9
          sprintf(tcpBuffer, "{\"device\":\"%s\",\"type\":\"%d\",\"maxp\":\"%d\",\"descr\":\"%s\"}",sistema_scs.elenco[deviceX].address,devtype,maxp.Val,sistema_scs.elenco[deviceX].descrizione);
          buflen = 0;
          while (tcpBuffer[buflen]) buflen++;
          tcpclient.write(tcpBuffer, buflen);
          tcpclient.flush();
        } // valido
        deviceX++;
      } // for devicex < MY_ENTITA
     
      sprintf(tcpBuffer, "#eof");
      buflen = 0;
      while (tcpBuffer[buflen]) buflen++;
      tcpclient.write(tcpBuffer, buflen);
      tcpclient.flush();
    } // received "#getdevices"
    else
// ------------------------------------------------------------------------------------------------------
    if (memcmp(tcpBuffer, "#getdevice",10) == 0)  // download devices
// ------------------------------------------------------------------------------------------------------
    {
      deviceX = 1;

      String busid = tcpJarg(tcpBuffer,(char *) "\"device\"");
      if (busid != "")
      {
        deviceX = CercaAddress(busid.c_str()); // pointer device reale o temporaneo
      }
      else
      busid = tcpJarg(tcpBuffer,(char *) "\"afterdev\"");
      if (busid != "")
      {
        deviceX = CercaAddress(busid.c_str()); // pointer device reale o temporaneo
        if (deviceX != 0xFF)
            deviceX++;
      }
      else
      busid = tcpJarg(tcpBuffer,(char *) "\"devnum\"");
      if (busid != "")
      {
        deviceX = (char) busid.toInt();
        deviceX--;
      }

      if (deviceX == 0xFF)
      {
        sprintf(tcpBuffer, "#eof");
        buflen = 0;
        while (tcpBuffer[buflen]) buflen++;
        tcpclient.write(tcpBuffer, buflen);
        tcpclient.flush();
      }
      else
      {         // testare valido
          device = sistema_scs.elenco[deviceX].id;
          devtype = sistema_scs.elenco[deviceX].tipo_codice;
          maxp.Val = 0;
          if ((devtype == 9) || (devtype == 19))
          {
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '6';
            requestBuffer[requestLen++] = device;
            immediateSend();
            char m = immediateReceive('[');
            
            if (m > 8)
            {
              maxp.Val = replyBuffer[2] | replyBuffer[1] << 8;
            }
            
          }  // devtype == 9
//          sprintf(tcpBuffer, "{\"device\":\"%s\",\"type\":\"%d\",\"maxp\":\"%d\",\"descr\":\"%s\"}",nomeDevice,devtype,maxp.Val,AlexaDescr);
          uint8_t devX = deviceX - 1; 
          sprintf(tcpBuffer, "{\"device\":\"%s\",\"type\":\"%d\",\"maxp\":\"%d\",\"descr\":\"%s\"}",sistema_scs.elenco[deviceX].address,devtype,maxp.Val,sistema_scs.elenco[deviceX].descrizione);
//          sprintf(tcpBuffer, "{\"device\":\"%s\",\"type\":\"%d\",\"maxp\":\"%d\",\"descr\":\"%s\"}",sistema_scs.elenco[devX].address,devtype,maxp.Val,sistema_scs.elenco[devX].descrizione);
          buflen = 0;
          while (tcpBuffer[buflen]) buflen++;
          tcpclient.write(tcpBuffer, buflen);
          tcpclient.flush();
      } // devtype > 0
    } // received "#getdevices"
    else
// ------------------------------------------------------------------------------------------------------
    if (memcmp(tcpBuffer, "#setup ",7) == 0)
// ------------------------------------------------------------------------------------------------------
    {
      String debug = tcpJarg(tcpBuffer,(char *) "\"debug\"");
      if (debug == "tcp")  
      {
        tcpuart = 2;
      }
      else if (debug == "no")  
      {
        tcpuart = 0;
      }

      String uart = tcpJarg(tcpBuffer,(char *) "\"uart\"");
      if ((uart == "tcp")  && (tcpuart == 0))
      {
        tcpuart = 1;
#ifdef DEBUG 
	    Serial.print("\r\ntcp uart accepted ");
#endif
      }

      String ecommit = tcpJarg(tcpBuffer,(char *) "\"commit\"");
      if (ecommit == "true")
      {
        EEPROM.commit();
      }

//      String freq = tcpJarg(tcpBuffer,"\"frequency\"");
//      if (freq == "160")  
//      {
//        system_update_cpu_freq(160);
//      }
//      else
//      if (freq == "80")
//      {
//        system_update_cpu_freq(80);
//      }

      sprintf(tcpBuffer, "#ok");
      buflen = 0;
      while (tcpBuffer[buflen]) buflen++;
      tcpclient.write(tcpBuffer, buflen);
      tcpclient.flush();
    }
// ------------------------------------------------------------------------------------------------------
    else
    {
      if (tcpuart == 1)
      {
        firstTime = 0;  // msg udp: ora i comandi MQTT vanno fatti precedere dal setup
        int s = 0;
        while (s < buflen)
        {
          Serial1.write(tcpBuffer[s]);   // write on serial KNXgate/SCSgate
          delayMicroseconds(120);
          s++;
        }
        
//      if (replyLen > 0)
//      {
//        tcpclient.write(replyBuffer, replyLen);
//      }
      }
      else
      {
#ifdef DEBUG
        Serial.print("error");
#endif
        tcpclient.write(tcpBuffer, buflen);
//        tcpclient.flush(); 
        tcpclient.write("#err", 4);
//        tcpclient.flush(); 
      }
    }
   } // tcpclient connected and available
  } // tcpopen
  // =========================================== FINE TCP ==========================================
#endif







  // ================================================ OTA ==========================================
#ifdef USE_OTA
  if (ArduinoOTAflag == 1)
  {
    if (now - prevTime > 29)  // 30mS
    {
#ifdef BLINKLED
      digitalWrite(pinLed, HIGH); // spento
#endif
    }
    if (now - prevTime > 99)  // 100mS
    {
#ifdef BLINKLED
      digitalWrite(pinLed, HIGH); // spento
#endif
      prevTime = now;
      ArduinoOTA.handle();
   #ifdef BLINKLED
      ledCtr++;
      if (connectionType == 0)  // router - 1sec
      {
        if (ledCtr > 9)
        {
          ledCtr = 0;
          digitalWrite(pinLed, LOW); // acceso
          gestisciPotenzaDinamica();
        }
      }
      else    // AP - 0.3sec
      {
        if (ledCtr > 2)
        {
          ledCtr = 0;
          digitalWrite(pinLed, LOW); // acceso
        }
      }
   #endif
    }
  }
#endif



  // ======================================= WIFI KEEP ALIVE ===========================================

  if (connectionType == 0) // wifi router mode
  {
    if (WiFi.status() == WL_CONNECTED)
    {
        lastCheck = now; 
        badCheck = 0;    
    }
    else
    {
#ifdef BLINKLED
      digitalWrite(pinLed, LOW); // acceso
#endif
      if ((now - lastCheck) > 10000)
      { // controllo connessione wifi  ogni 10 secondi
        lastCheck = now; 
        badCheck++;  
//      if (badCheck > 3)    // 30 secondi 
        if (badCheck > 30)   // 5 minuti 
        { 
          Serial1.write('@');    // 
          Serial1.write(0xFF);   // set led lamps zero
          delay(100);           // wait 100ms
          Serial1.write('@');    // 
          Serial1.write('|');    // 
          Serial1.write('|');    // 
          delay(100);           // wait 100ms
          countRestart = 2000;
          badCheck = 0;
        }
      }
    }
  }    
    

  // ======================================= MQTT KEEP ALIVE ===========================================
  if (mqttopen != 0)
  {

    if (mqttopen == 3)
    {
      client.loop();
      if (!client.connected())
      {
        mqttopen = 1;
        lastMsg  = now;
      }
    }

    if (mqttopen == 2)
    {
      client.loop();
      client.subscribe(SUBSCRIBE1);
      client.subscribe(SUBSCRIBE2);
      client.subscribe(SUBSCRIBE3);
      mqttopen = 3;
    }

    if (mqttopen == 1)
    {
      if ((now - lastMsg) > 10000)
      { // nuova versione - controllo connessione mqtt solo ogni 10 secondi
        lastMsg = now;         //                per evitare overload
#ifdef DEBUG
        Serial.println("mqtt not connected");
#endif
        mqttopen = reconnect();
        mqtt_connections++;
#ifdef DEBUG
        if (mqttopen == 2)
          Serial.println("ok ready");
        else
          Serial.println("ko");
#endif
        if (mqttopen == 1) // connection failed
        {
          mqtt_retry++;
          if ((mqtt_retrylimit > 0) && (mqtt_retry > mqtt_retrylimit))
          {
              ESP.restart();
//            mqttopen = 0;   
          }
        }
        else
          mqtt_retry = 0;
      }
    }
  }
  // ===================================== END MQTT KEEP ALIVE ===========================================



  if (udpopen == 1)
  {
    // ====================================== receive from UDP - send to SERIAL =====================
    int packetSize = udpConnection.parsePacket();

    if (packetSize)
    {
#ifdef DEBUG
      Serial.println ( "UDP packet received" );
#endif
      int len = udpConnection.read(udpBuffer, 255);
      packetSize = len;
      udp_remote_ip = udpConnection.remoteIP();
      udp_remote_port = udpConnection.remotePort();
      udpBuffer[len] = 0;

      httpResp = "";

      /*--------------------------------------------------------------------------
          if (strcmp(packetBuffer,"@V") == 0) // query version
          {
            int s;
            for (s=0; s<sizeof(VERSION); s++)
            {
              Serial1.write(VERSION[s]);   // write on serial KNXgate/SCSgate
              delayMicroseconds(50);
            }
          }
          else
      --------------------------------------------------------------------------*/
      
      if (strcmp(udpBuffer, "@Keep_alive") == 0)
      {
      }
      else if (strcmp(udpBuffer, "@Kill") == 0)
      {
        udp_remote_ip = {0, 0, 0, 0};
      }
      else
      {
        firstTime = 0;  // messaggio udp arrivato: ora i comandi MQTT vanno fatti precedere dal setup
        int s = 0;
        while (s < len)
        {
          Serial1.write(udpBuffer[s]);   // write on serial KNXgate/SCSgate
//          delayMicroseconds(120);
          s++;
        }
        tcpuart = 0;
#ifdef DEBUG 
	    Serial.print("\r\ntcp uart closed ");
#endif
      }
    } // packetsize
  }   // udpopen == 1



  //    delay(1);  // wait 1 mS (!)
  delayMicroseconds(500);


  // ====================================== receive from SERIAL - send to UDP =====================
  // rx: 04 31 00 12 00 


  replyLen = 0;
  if (Serial1.available() )
  {
    internal = 0;
    String log = "\r\ns2 rx: ";
    char logCh[4];
    char lmax;
    char prefix = Serial1.read();        // receive from serial USB
#ifdef DEBUG
//    manualInput(prefix);
//    prefix = 0;
#endif
    if ((prefix > 0xF0) && (prefix < 0xFF)) // 0xf5 y aa bb cc dd 
    {
      internal = 1;
      lmax = (prefix & 0x0F);
      lmax++;
#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
      if (tcpuart == 2)  
      {
        sprintf(logCh, "%02X > ", prefix);
        log += logCh;
      }
  #endif
#endif
      replyBuffer[replyLen++] = prefix;
    }
    else
    {
  // rx: 04 31 00 12 00 
      internal = 0;
      replyBuffer[replyLen++] = prefix;
      lmax = 255;

#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
      if (tcpuart == 2)  
      {
        sprintf(logCh, "%02X ", prefix);
        log += logCh;
      }
  #endif
#endif
    }
    while (Serial1.available() && (replyLen < lmax))
    {
      while (Serial1.available() && (replyLen < lmax))
      {
        replyBuffer[replyLen] = Serial1.read();        // receive from serial USB
#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
        if (tcpuart == 2)  
        {
          sprintf(logCh, "%02X ", replyBuffer[replyLen]);
          log += logCh;
        }
  #endif
#endif
        replyLen++;
        delayMicroseconds(INNERWAIT); // 90
      }
      delayMicroseconds(OUTERWAIT); // 120
    }
    replyBuffer[replyLen] = 0;        // aggiunge 0x00


#ifdef USE_TCPSERVER
  #ifdef DEBUG_FAUXMO_TCP         
    if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
    {
      tcpclient.write((char*)&log[0], log.length());
      tcpclient.flush(); 
    }
  #endif
#endif
  } // serial_available

  replyBuffer[replyLen] = 0;





// EVENTS SSE LOG

  if ((events.count() != 0) && (replyLen > 1)) //  && (replyBuffer[1] |= 0))
  {
/*
		if (internal != prev_internal)
		{
			prev_internal = internal;
			if (internal)
				events.send("In funzione - modalità server - OK", "stato", millis());
			else
				events.send("In funzione - modalità TEST - OK", "stato", millis());
		}
*/
      String log;
      char logCh[256];
      int xr = 0;
	  bool asc = false;
      if (internal == 1) // modalita' interna - ok per tutto - caratteri in modalità interna [F3 ...
      {
//        sprintf(logCh, "[%04d] SCS RX: {%02X %02X} -> ", (now/100), replyBuffer[xr++], replyBuffer[xr++]);
        sprintf(logCh, "SCS RX (int): {%02X %02X} -> ", replyBuffer[xr++], replyBuffer[xr++]);
        log = logCh;
        while (xr < replyLen)
        {
          sprintf(logCh, "%02X ", replyBuffer[xr++]);
          log += logCh;
        }
        if (events.count() != 0) 
            events.send(log.c_str(), "log", millis());
      }
      else
      {				// modalità ascii oppure binaria
//		log = "";
        if ((replyBuffer[1] == '\r') || ( replyBuffer[1] == '\n'))
		{
          log = &replyBuffer[2];
		  asc = true;
		}
        else
        if ((replyBuffer[0] == '\r') || ( replyBuffer[0] == '\n'))
		{
          log = &replyBuffer[1];
		  asc = true;
		}
        else
          log = replyBuffer;

		if ((replyBuffer[1] >= 0x20) && (replyBuffer[1] <= 0x7E) || ((replyBuffer[1] == '\n') || (replyBuffer[1] == '\r')))  // CARATTERI NON STAMPABILI - modo binario
			asc = true;

		if (!asc)  // CARATTERI NON STAMPABILI - modo binario
		{
			sprintf(logCh, "SCS rx (ext): -> {%02x} ", replyBuffer[xr++]);
			log = logCh;
			while (xr < replyLen)
			{
			  sprintf(logCh, "%02X ", replyBuffer[xr++]);
			  log += logCh;
			}
		}
		else
		{
	        log.replace("\r", ""); // Rimuove il Carriage Return
		    log.replace("\n", "\\n"); // Sostituisce il New Line con un a capo HTML
		}
        if (events.count() != 0) 
        	events.send(log.c_str(), "log", millis());
      }
  }




  // --------------------- RISPOSTA UDP se richiesto -------------------------------------------


  if (tcpuart == 0)
  {
    if ((udpopen == 1) && (udp_remote_ip) && (replyLen) && (internal == 0))
    {
      if (httpResp == "")
      {
        int success;
        do  {
          success =  udpConnection.beginPacket(udp_remote_ip, udp_remote_port);
        }   while (!success);

        int n = 0;
        while (n < replyLen)
        {
          udpConnection.write(replyBuffer[n++]);     // UDP reply
        }
        success = udpConnection.endPacket();
      }
    } // udp_remote_ip
  }


  // --------------------- RISPOSTA TCP se richiesto -------------------------------------------


#ifdef USE_TCPSERVER
  else
  if ((tcpuart == 1) && (replyLen > 0) && (tcpclient) && (tcpclient.connected())) 
  {
#ifdef DEBUG 
    Serial.print("\r\ntcp write ");
    Serial.write(replyBuffer, replyLen);
#endif
    tcpclient.write(replyBuffer, replyLen);
//  tcpclient.flush(); 
  }
#endif


  // --------------------- RISPOSTA HTTP se richiesto -------------------------------------------


  if (((httpResp == "a") || (httpResp == "y")) && (httpCallback[0] == ':'))
  {
    if ((replyBuffer[0] == 0xF5) && (replyBuffer[1] == 'y') && (replyLen == 6))   // solo se stringa=   [0xF5] [y] 32 00 12 01
    {
      content = "http://";
      content += tcp_remote_ip.toString();
      content += httpCallback;
      char hBuffer[12];

      // intero  [7] A8 32 00 12 01 21 A3
      // ridotto [0xF5] [y] 32 00 12 01
      // ----------------1---2--3--4--5--
      sprintf(hBuffer, "&type=%02X", replyBuffer[4]);
      content += hBuffer;
      sprintf(hBuffer, "&from=%02X", replyBuffer[3]);
      content += hBuffer;
      sprintf(hBuffer, "&to=%02X", replyBuffer[2]);
      content += hBuffer;
      sprintf(hBuffer, "&cmd=%02X", replyBuffer[5]);
      content += hBuffer;
      httpClient.begin(content);
      int httpCode = httpClient.GET();                                  //Send the request
      httpClient.end();   //Close connection
      content = "";
    }
  } // httpResp == "a"
  // --------------------- FINE RISPOSTA HTTP -------------------------------------------


  else

  // ----------------------------------------- ALEXA STATO DEVICES ---------------------------------------------------------
    


  // ---------------------u-posizione tapparelle o dimmer %---------------------------------------------------------------------
  //    [F3] u <address> <position%>
  if ((replyLen == 4) && (replyBuffer[0] == 0xF3) && (replyBuffer[1] == 'u') &&  (alexaParam == 'y'))  //'u': posizione tapparelle//
  { // replyLen==4 && replyBuffer == 0xF3 'u'
    uint8_t device = replyBuffer[2];
    int pct = replyBuffer[3];
    uint8_t devx = CercaAddress(device);
   
#ifdef TRACE_SSE
  char buflog[120];
  sprintf(buflog, "[TRC} alexa tapparelle ", replyBuffer[1]);
  events.send(buflog, "log", millis());
#endif
    if (devx != 0xFF)
    {
       // aggiornamento posizione coverpct fauxmo   
        uint8_t id_alexa = sistema_scs.elenco[devx].alexa_id;
        if (id_alexa)
        {
          pct *= 255;
          pct /= 100;
          if (pct == 0)  pct = 1;
          if (pct > 255) pct = 255;
          fauxmo.setState(id_alexa, 0xFF, pct);
	    }    
    }
  }






#ifdef MY_DOMOTIC
  // ================================== P U B B L I C A Z I O N E    S S E =========================================
	if (replyLen > 1)
		scs_tratta_messaggi_per_my_domotic_new();
#endif
  // ===============================================================================================================

    // ----------------------------------------- ALEXA STATO DEVICES END----------------------------------------------------
	if ((replyLen > 1) && (mqttopen == 3) && (replyBuffer[1] != DEVICEREQUEST))
    {
		if (scs_tratta_messaggi_per_mqtt() == 0)
			replyLen = 0; //  impedire pubblicazione UDP messaggi m e u
	} // END pubblicazione stato devices
  // ----------------------------------------------------------------------------------------------------

  // ================================ F I N E   C O M A N D I     S C S ===========================================







      // =====================================fauxmo handle (ALEXA) ====================================================
      if (alexaParam == 'y' )
      {
         // fauxmoESP uses an async TCP server but a sync UDP server
        // Therefore, we have to manually poll for UDP packets
        fauxmo.handle();

#ifdef DEBUG
        // This is a sample code to output free heap every 15 seconds
        // This is a cheap way to detect memory leaks
        //      static unsigned long last = millis();
        //      if (millis() - last > 15000) {
        //      last = millis();
        //      Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
        //      }
#endif

        // If your device state is changed by any other means (MQTT, physical button,...)
        // you can instruct the library to report the new state to Alexa on next request:
        // fauxmo.setState(ID_YELLOW, true, 255);
      }  // alexaParam == 'y'
      // ===============================================================================================================




      // ===================================CICLO SCRITTURA BUFFER UART=DA REQUESTBUFFER===LOGGATA======================
      if (requestLen > 0)
      {
        uartSemaphor = 1;
        // =========================== send control char and data over serial =============================
        String log = "\r\ntx: ";
        char logCh[4];
        int s = 0;
        while (s < requestLen)
        {
#ifdef UART_W_BUFFER
          Serial1.write(requestBuffer[s]);    //  scrittura SERIALE 
//          delayMicroseconds(120);
  #ifdef USE_TCPSERVER
          if (tcpuart == 2) 
          {
            sprintf(logCh, "%02X ", requestBuffer[s]);
            log += logCh;
          }
  #endif
#else
          //       Serial1.flush();
          //       Serial1.write(requestBuffer[s]);   // write on serial KNXgate/SCSgate - 90uS


          // USS = uart register 1C-19 (32 bit)  bit 16-23=data nr in tx fifo   (USTXC = 16)
          // UART STATUS Registers Bits
          // USTX    31 //TX PIN Level
          // USRTS   30 //RTS PIN Level
          // USDTR   39 //DTR PIN Level
          // USTXC   16 //TX FIFO COUNT (8bit)
          // USRXD   15 //RX PIN Level
          // USCTS   14 //CTS PIN Level
          // USDSR   13 //DSR PIN Level
          // USRXC    0 //RX FIFO COUNT (8bit)

#ifdef DEBUG
#else
          while (((USS(0) >> USTXC) & 0xff) > 0)     {	// aspetta il buffer sia completamente vuoto
            delay(0);
          }

          delayMicroseconds(OUTERWAIT);
          USF(0) = requestBuffer[s];    // scrittura SERIALE

          while (((USS(0) >> USTXC) & 0xff) > 0)     {	// aspetta il buffer sia completamente vuoto
            delay(0);
          }
#endif
#ifdef DEBUG
          sprintf(logCh, "%02X ", requestBuffer[s]);
          log += logCh;
#endif
#ifdef USE_TCPSERVER
#ifdef DEBUG_FAUXMO_TCP         
          if (tcpuart == 2) 
          {
            sprintf(logCh, "%02X ", requestBuffer[s]);
            log += logCh;
          }
#endif
#endif
          delayMicroseconds(OUTERWAIT); // 100uS
#endif // UART_W_BUFFER
          s++;
        }
#ifdef DEBUG
        Serial.println("\r\n" + log);
#endif

#ifdef USE_TCPSERVER
#ifdef DEBUG_FAUXMO_TCP         
        if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
        {
          tcpclient.write((char*)&log[0], log.length());
          tcpclient.flush(); 
        }
#endif
#endif

//  SSE EVENTS LOG

        if ((events.count() != 0) && (requestLen > 1)) //  && (requestBuffer[1] |= 0))
        {
            String log;
            char logCh[32];
            int xr = 0;
//            sprintf(logCh, "[%04d] SCS TX: ->{%c %c} -> ", (now/100), requestBuffer[xr++], requestBuffer[xr++]);
            if (requestBuffer[0] == PIC_REQ)
            {
              xr++;
              sprintf(logCh, "SCS TX: ->{&#xA7 %c} -> ", requestBuffer[xr++]);
            }
            else
            if (requestBuffer[0] == 0x40)
            {
              xr++;
              sprintf(logCh, "SCS TX: ->{&#x40 %c} -> ", requestBuffer[xr++]);
            }
            else
              sprintf(logCh, "SCS TX: ->{%c %c} -> ", requestBuffer[xr++], requestBuffer[xr++]);
            log = logCh;

            while (xr < requestLen)
            {
              sprintf(logCh, "%02X ", requestBuffer[xr++]);
              log += logCh;
            }
//            log.replace("§", "&#xA7"); // § 0xA7
//            log.replace("@", "&#x40"); // @ 0x40

            if (events.count() != 0) 
                events.send(log.c_str(), "log", millis());
        }

        requestLen = 0;
        uartSemaphor = 0;
      } // requestlen > 0
      






      // ============================CICLO SCRITTURA BUFFER UART ASYNCH=================================================
      char bufNr = 0;
      while (bufNr < BUFNR)
      {
        if (serOlen[bufNr]) SendToPIC(bufNr);
        bufNr++;
      }
      // ===============================================================================================================

	  if (events.count() != prevcount)
	  {
		if ((events.count() == 0) && (prevcount)) // se ho chiuso tutte le sessioni aperte
		{
			setFirst();
			internal = 1;
		}
		prevcount = events.count();
	  }

// =======================================end NORMAL RUN ====================================================
}
#ifdef FFS
else
{      
// ========================================== PICPROG RUN ====================================================
   PicProg();
// =======================================end PICPROG RUN ====================================================
}
#endif      
      // =====================================================================================================
      // =====================================================================================================
////      if (webon == 1)
////        a_server.handleClient();
      // =====================================================================================================
      // =====================================================================================================
}

// =====================================================================================================
/*
// =====================================================================================================
void SerialSetup(void)
{
	String stmp;
    Serial.setTimeout(15000); // 15 sec timeout
    Serial.println("S: _________serial setup________");
    
    SerialSetupParam("enter wifi SSID [", E_SSID);
    SerialSetupParam("enter wifi password [", E_PASSW);
    SerialSetupParam("enter ESP ipaddress [", E_IPADDR);
    SerialSetupParam("enter ROUTER ipaddress [", E_ROUTIP);
    SerialSetupParam("enter UDP port [", E_PORT);
 
    EEPROM.commit();

    Serial.println("ok...");
    Serial.setTimeout(1000); // 1 sec timeout
}
// =====================================================================================================
void SerialSetupParam(char* title, int eAdress)
{
    String stmp;
    Serial.print(title);
    stmp = "";
    stmp = ReadStream(&stmp[0], eAdress, 32, 2); 
    if ((stmp != "") && (stmp[0] > 0) && (stmp[0] < 0xFF)) 
      Serial.print(stmp);
    Serial.print("] : ");
    stmp = SerialRead(20000);
    if (stmp != "")
    {
      Serial.println("");
      Serial.println(stmp);
      if (stmp == "0")  
          stmp = "";
      WriteEEP(stmp, eAdress);
    }
    Serial.println("");
}
// =====================================================================================================
String SerialRead(int tout)
{
    String stmp;
    char c;
    int timeo = 0;
    do
    {
      timeo++;
      if (Serial.available())
      {  
         c=Serial.read();
         if ((c == '\n') || (c == '\r')|| (c == 0))
             return stmp;
         if ((c == '\b') && (stmp != ""))
             stmp.remove(stmp.length()-1);
         else
             stmp += char(c);
      }
      delay(1);
    } while(timeo < tout);
    return stmp;
}
// =====================================================================================================
*/


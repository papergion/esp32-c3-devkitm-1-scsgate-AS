//---------------------------------------------------------------------------------------------------------
#define _FW_NAME     "SCSGATE_32"
#define _FW_VERSION  "VER_8.008 "
#define _ESP_CORE    "esp32-2.5.2"
// ========================================================================================================

/*
da sviluppare:
- gestione dispositivi generici, allarme, termostato, ... in sse HA
- censimento automatico dispositivi
- trattamento % dimmer 'm'
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
#define LOWPOWER

const int pinLed = 8;

#define SCS

//#define DEBUG_EVENTITY
//#define DEBUG
//#define VERBOSE

#define MY_DOMOTIC
#define MY_TABELLA

// #define FFS
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

#define MY_ENTITA 64

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

enum TipoEntita { ENTI_INTERRUTTORE, ENTI_SENSORE, ENTI_SLIDER, ENTI_TAPPARELLA, ENTI_TAPPARELLA_PCT };

uint8_t corrispondenza[5] = {1,15,3,8,9};

struct Entita {
    String id;
    String nome;
    TipoEntita tipo;
    uint8_t pin;    // SCS address
    String stato;
    uint8_t valore; 
};

Entita elencoEntita[MY_ENTITA];
int numeroEntita = 0;

void aggiungiEntita(String id, String nome, TipoEntita tipo, uint8_t pin, String statoIniziale, uint8_t valore) {
    if (numeroEntita < MY_ENTITA) {
//        String statoEffettivo = statoInizialeDefault;
//        if (tipo == ENTI_INTERRUTTORE || tipo == ENTI_SLIDER || tipo == ENTI_TAPPARELLA) {
//            statoEffettivo = leggiStatoSalvato(id, statoInizialeDefault);
//        }
//        if (pin != -1 && tipo == ENTI_INTERRUTTORE) {
//            pinMode(pin, OUTPUT);
//            digitalWrite(pin, statoIniziale == "ON" ? HIGH : LOW);
//        }
        elencoEntita[numeroEntita] = {id, nome, tipo, pin, statoIniziale, valore};
        numeroEntita++;
    }
}
// ----------------------------------------------------------------  
int cercaEntitaScs(int pin) {
    for (int i = 0; i < numeroEntita; i++) {
        if (elencoEntita[i].pin == pin) {
            return i;
        }
    }
    return -1;
}

AsyncEventSource eventity("/eventity");






const char index_html_HA[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="it">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-C3 JSON Dashboard</title>
    <style>
        body { font-family: 'Segoe UI', sans-serif; background: #f4f7f6; color: #333; text-align: center; padding: 20px; }
        .grid { display: flex; flex-wrap: wrap; justify-content: center; max-width: 1200px; margin: 0 auto; }
        .card { background: white; padding: 20px; border-radius: 12px; box-shadow: 0 4px 8px rgba(0,0,0,0.05); width: 180px; margin: 10px; text-align: left; }
        h1 { color: #007bff; }
        h2 { margin-top: 0; font-size: 1.3em; color: #444; }
        .status-badge { padding: 4px 10px; border-radius: 20px; font-weight: bold; font-size: 0.85em; float: right; }
        .ON { background: #d4edda; color: #155724; }
        .OFF { background: #f8d7da; color: #721c24; }
        .VAL { background: #e2e3e5; color: #383d41; }
        .btn { border: none; padding: 10px 0; width: 100%; font-size: 14px; border-radius: 6px; cursor: pointer; transition: 0.2s; font-weight: bold; margin-top: 20px; background: #007bff; color: white; }
        .btn:hover { background: #0056b3; }
        .btn-tapp { flex: 1; border: none; padding: 10px 0; font-weight: bold; border-radius: 6px; cursor: pointer; background: #e2e3e5; color: #333; transition: 0.2s; }
        .active-up { background: #28a745; color: white; }
        .active-stop { background: #dc3545; color: white; }
        .active-down { background: #ffc107; color: black; }
    </style>
</head>
<body>
    <h1>ESP32-C3 Client-Side Dashboard</h1>
    <div id="dashboardGrid" class="grid">
        <!-- Le card verranno inserite dinamicamente qui dal ciclo JS -->
    </div>

<script>
    let sliderInUso = {};

    async function inizializzaDashboard() {
        try {
            const response = await fetch('/api/config');
            const entitaElenco = await response.json();
            const grid = document.getElementById('dashboardGrid');
            
            entitaElenco.forEach(entita => {
                const card = document.createElement('div');
                card.className = 'card';
                
                if (entita.tipo === 0) { // ENTI_INTERRUTTORE
                    card.innerHTML = `
                        <h2>${entita.nome}</h2>
                        <p>Stato: <span id="badge_${entita.id}" class="status-badge ${entita.stato}">${entita.stato}</span></p>
                        <button class="btn" onclick="inviaComando('${entita.id}')">AZIONA</button>
                    `;
                } else if (entita.tipo === 1) { // ENTI_SENSORE
                    card.innerHTML = `
                        <h2>${entita.nome}</h2>
                        <div style="display: flex; align-items: center; justify-content: space-between; margin-top: 15px;">
                            <span style="font-size: 2rem; margin-right: 10px;">🌡️</span>
                            <div style="text-align: right; font-size: 1.6rem; font-weight: bold; color: #007bff;">
                                <span id="badge_${entita.id}">${entita.stato}</span>°C
                            </div>
                        </div>
                    `;
                } else if (entita.tipo === 2) { // ENTI_SLIDER
                    const parti = entita.stato.split('|');
                    const acceso = parti[0];
                    const livello = parti[1];
                    card.innerHTML = `
                        <h2>${entita.nome}</h2>
                        <p>Stato: <span id="badge_${entita.id}" class="status-badge ${acceso}">${acceso}</span> (<b id="valore_${entita.id}">${livello}</b>)</p>
                        <input type="range" min="0" max="100" value="${livello}" class="slider" 
                            id="slider_${entita.id}" 
                            oninput="aggiornaSliderLocale('${entita.id}', this.value)" 
                            onchange="inviaValoreSlider('${entita.id}', this.value)">
                        <div style="display: flex; gap: 5px; margin-top: 15px;">
                            <button class="btn" onclick="inviaComando('${entita.id}')">AZIONA</button>
                        </div>
                    `;
                    sliderInUso[entita.id] = false;
                } else if (entita.tipo === 3) { // ENTI_TAPPARELLA
                      card.innerHTML = `
                          <h2>${entita.nome}</h2>
                          <p>Stato: <b id="testo_${entita.id}">${entita.stato}</b></p>
                          <div style="display: flex; gap: 5px; margin-top: 15px;">
                              <button id="btn_UP_${entita.id}" class="btn-tapp" onclick="inviaComandoTapparella('${entita.id}', 'SALITA')">▲ SU</button>
                              <button id="btn_STOP_${entita.id}" class="btn-tapp" onclick="inviaComandoTapparella('${entita.id}', 'STOP')">■ STOP</button>
                              <button id="btn_DOWN_${entita.id}" class="btn-tapp" onclick="inviaComandoTapparella('${entita.id}', 'DISCESA')">▼ GIÙ</button>
                          </div>
                      `;
                } else if (entita.tipo === 4) { // ENTI_TAPPARELLA_PCT
                      const partiTapp = entita.stato.split('|');
                      const pct = partiTapp[0];
                      const direzione = partiTapp[1];
                      card.innerHTML = `
                          <h2>${entita.nome}</h2>
                          <p>Posizione: <b id="testo_pct_${entita.id}">${pct}</b>% (<span id="testo_dir_${entita.id}">${direzione}</span>)</p>
                          <input type="range" min="0" max="100" value="${pct}" class="slider" 
                              id="slider_pct_${entita.id}" 
                              oninput="sliderInUso['${entita.id}'] = true; document.getElementById('testo_pct_${entita.id}').innerText = this.value" 
                              onchange="inviaTargetTapparella('${entita.id}', this.value)">
                          <div style="display: flex; gap: 5px; margin-top: 15px;">
                              <button id="btn_UP_${entita.id}" class="btn-tapp" onclick="inviaTargetTapparella('${entita.id}', '100')">▲ APRI</button>
                              <button id="btn_STOP_${entita.id}" class="btn-tapp" onclick="inviaTargetTapparella('${entita.id}', 'STOP')">■ STOP</button>
                              <button id="btn_DOWN_${entita.id}" class="btn-tapp" onclick="inviaTargetTapparella('${entita.id}', '0')">▼ CHIUDI</button>
                          </div>
                      `;
                      sliderInUso[entita.id] = false;
                      setTimeout(function() { aggiornaGraficaTapparellaPct(entita.id, pct, direzione); }, 10);
                }
                grid.appendChild(card);
                connettiSSE(entita.id, entita.tipo);
                if(entita.tipo === 3) aggiornaGraficaTapparella(entita.id, entita.stato);
            });
        } catch (error) {
            console.error("Errore nel caricamento della configurazione JSON:", error);
        }
    }

    function inviaComando(idEntita) {
        const badge = document.getElementById('badge_' + idEntita);
        if (badge) {
            const nuovoStato = (badge.innerText === 'ON') ? 'OFF' : 'ON';
            badge.innerText = nuovoStato;
            badge.className = 'status-badge ' + nuovoStato;
        }
        fetch('/api/toggle?id=' + idEntita, { method: 'POST' })
        .catch(err => console.error("Errore comando:", err));
    }

    function aggiornaSliderLocale(idEntita, valore) {
        sliderInUso[idEntita] = true;
        document.getElementById('valore_' + idEntita).innerText = valore;
    }

    function inviaValoreSlider(idEntita, valore) {
        fetch('/api/set?id=' + idEntita + '&val=' + valore, { method: 'POST' })
        .then(function() {
            setTimeout(function() { sliderInUso[idEntita] = false; }, 300);
        })
        .catch(err => {
            console.error(err);
            sliderInUso[idEntita] = false;
        });
    }

    function inviaComandoTapparella(idEntita, comando) {
        fetch('/api/tapparella?id=' + idEntita + '&cmd=' + comando, { method: 'POST' })
        .catch(err => console.error("Errore tapparella:", err));
    }

    function aggiornaGraficaTapparella(id, stato) {
        const btnUp = document.getElementById('btn_UP_' + id);
        const btnStop = document.getElementById('btn_STOP_' + id);
        const btnDown = document.getElementById('btn_DOWN_' + id);
        const testo = document.getElementById('testo_' + id);
        
        if(!btnUp || !btnStop || !btnDown) return;
        
        testo.innerText = stato;
        btnUp.className = "btn-tapp";
        btnStop.className = "btn-tapp";
        btnDown.className = "btn-tapp";
        
        if (stato === "SALITA") btnUp.classList.add("active-up");
        else if (stato === "STOP") btnStop.classList.add("active-stop");
        else if (stato === "DISCESA") btnDown.classList.add("active-down");
    }

function inviaTargetTapparella(idEntita, target) {
    fetch('/api/tapparella_pct?id=' + idEntita + '&target=' + target, { method: 'POST' })
    .catch(err => console.error(err));
}

function aggiornaGraficaTapparellaPct(id, pct, direzione) {
    const inputSlider = document.getElementById('slider_pct_' + id);
    const testoPct = document.getElementById('testo_pct_' + id);
    const testoDir = document.getElementById('testo_dir_' + id);
    const btnUp = document.getElementById('btn_UP_' + id);
    const btnStop = document.getElementById('btn_STOP_' + id);
    const btnDown = document.getElementById('btn_DOWN_' + id);
    
    if (!inputSlider) return;

    if (!sliderInUso[id]) {
        inputSlider.value = pct;
        testoPct.innerText = pct;
    }
    testoDir.innerText = direzione;
    btnUp.className = "btn-tapp";
    btnStop.className = "btn-tapp";
    btnDown.className = "btn-tapp";

    if (direzione === "SALITA") btnUp.classList.add("active-up");
    else if (direzione === "STOP") btnStop.classList.add("active-stop");
    else if (direzione === "DISCESA") btnDown.classList.add("active-down");
}

function connettiSSE(idEntita, tipoEntita) {
        if (!window.sseInizializzato) {
            window.source = new EventSource('/eventity');
            window.sseInizializzato = true;
        }
        
        window.source.addEventListener(idEntita, function(e) {
            if (tipoEntita === 2 && sliderInUso[idEntita]) return; 

            if (tipoEntita === 0) {
                const badge = document.getElementById('badge_' + idEntita);
                if (badge) {
                    badge.innerText = e.data;
                    if (e.data === 'ON') badge.className = 'status-badge ON';
                    else if (e.data === 'OFF') badge.className = 'status-badge OFF';
                    else badge.className = 'status-badge VAL';
                }
            } else if (tipoEntita === 1) {
                const badge = document.getElementById('badge_' + idEntita);
                if (badge) {
                    badge.innerText = e.data.replace('°C', '').trim();
                }
            } else if (tipoEntita === 2) {
                const badge = document.getElementById('badge_' + idEntita);
                const testoValore = document.getElementById('valore_' + idEntita);
                const inputSlider = document.getElementById('slider_' + idEntita);
                
                if (e.data.includes('|')) {
                    const partiRemote = e.data.split('|');
                    const acceso = partiRemote[0];
                    const livello = partiRemote[1];
                    if (badge) {
                        badge.innerText = acceso;
                        badge.className = 'status-badge ' + acceso;
                    }
                    if (testoValore) testoValore.innerText = livello;
                    if (inputSlider) inputSlider.value = livello;
                } else {
                    if (testoValore) testoValore.innerText = e.data;
                    if (inputSlider) inputSlider.value = e.data;
                }
            }
            else if (tipoEntita === 3) {
                aggiornaGraficaTapparella(idEntita, e.data);
            }
            else if (tipoEntita === 4) {
                const partiTappRemote = e.data.split('|');
                const stringPct = partiTappRemote[0];
                const stringDirezione = partiTappRemote[1];
                
                if(stringDirezione === "STOP") {
                    sliderInUso[idEntita] = false;
                }
                
                aggiornaGraficaTapparellaPct(idEntita, stringPct, stringDirezione);
            }
        });
    }

    window.onload = inizializzaDashboard;
</script>
</body>
</html>
)rawliteral";



// Funzione C++ per convertire l'array di strutture in un formato stringa JSON valido
String generaConfigurazioneJSON() {

    String json = "[";
    for (int i = 0; i < numeroEntita; i++) {
        json += "{";
        json += "\"id\":\"" + elencoEntita[i].id + "\",";
        json += "\"nome\":\"" + elencoEntita[i].nome + "\",";
        json += "\"tipo\":" + String(elencoEntita[i].tipo) + ",";
        if (elencoEntita[i].tipo == ENTI_TAPPARELLA_PCT) 
          json += "\"stato\":\"" + String(elencoEntita[i].valore) + "|" + elencoEntita[i].stato + "\"";
        else
        if (elencoEntita[i].tipo == ENTI_SLIDER)
          json += "\"stato\":\"" + String(elencoEntita[i].stato) + "|" + elencoEntita[i].valore + "\"";
        else
          json += "\"stato\":\"" + elencoEntita[i].stato + "\"";
        json += "}";
        if (i < numeroEntita - 1) json += ",";
    }
    json += "]";
#ifdef DEBUG_EVENTITY
  events.send(json, "log", millis());
#endif
    return json;
}


#endif



#ifdef MY_TABELLA

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


// =======================================================================================================================


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

Elemento lista[MY_ENTITA];
int conteggioElementi = 0;
bool tapparellePercentuale = false; // Stato memorizzato in RAM

// Pagina HTML + JavaScript integrata
const char index_html_TAB[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="it">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Dispositivi SCS</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f9; color: #333; }
        table { width: 100%; border-collapse: collapse; margin-top: 20px; background: white; }
        th { border: 1px solid #ccc; padding: 6px 10px; text-align: left; background-color: #007bff; color: white; }
        td { border: 1px solid #ccc; padding: 4px 10px; text-align: left; }
        
        form { background: white; padding: 15px; border-radius: 5px; box-shadow: 0 0 10px rgba(0,0,0,0.1); display: flex; gap: 10px; flex-wrap: wrap; align-items: center; }
        input, select, button { padding: 6px; border: 1px solid #ccc; border-radius: 4px; background: white; }
        
        .divider { width: 1px; height: 26px; background-color: #ccc; margin: 0 5px; }
        
        .config-inline { display: flex; align-items: center; gap: 6px; font-weight: bold; font-size: 14px; }
        .config-inline input[type="checkbox"] { width: 16px; height: 16px; cursor: pointer; margin: 0; }
        
        button { background-color: #28a745; color: white; cursor: pointer; border: none; padding: 5px 12px; }
        button:hover { background-color: #218838; }
        
        .btn-refresh { background-color: #17a2b8; font-weight: bold; }
        .btn-refresh:hover { background-color: #138496; }
        .btn-edit { background-color: #ffc107; color: #212529; }
        .btn-edit:hover { background-color: #e0a800; }
        .btn-del { background-color: #dc3545; }
        .btn-del:hover { background-color: #c82333; }
        .btn-cancel { background-color: #6c757d; }
        .btn-cancel:hover { background-color: #5a6268; }
    </style>
</head>
<body>
    <h2>Dispositivi SCS (<span id="count">0</span>/50)</h2>
    
    <form action="/add" method="POST" id="main-form">
        <input type="hidden" name="id" id="form-id" value="-1">
        <input type="text" name="address" id="form-address" placeholder="Addr (2)" maxlength="2" required style="width: 60px;" pattern="^[0-9A-Fa-f]{2}$" title="Inserire un valore esadecimale di 2 cifre (es. 01 o FF)" oninput="this.value = this.value.toUpperCase();">
        
        <select name="tipo" id="form-tipo" style="width: 80px;" required>
            <option value="LUC">LUC</option>
            <option value="DIM">DIM</option>
            <option value="TAP">TAP</option>
            <option value="GEN">GEN</option>
            <option value="ALA">ALA</option>
            <option value="TER">TER</option>
        </select>
        
        <input type="text" name="desc" id="form-desc" placeholder="Descrizione (20)" maxlength="20" required style="width: 180px;">
        <input type="text" name="valore" id="form-valore" placeholder="Val (3)" maxlength="3" required style="width: 60px;">
        
        <button type="submit" id="btn-submit">Aggiungi</button>
        <button type="button" id="btn-cancel" class="btn-cancel" style="display:none;" onclick="annullaModifica()">Annulla</button>
        
        <div class="divider"></div>
        
        <div class="config-inline">
            <input type="checkbox" id="chk-tapparelle">
            <label for="chk-tapparelle">Tapparelle a percentuale</label>
        </div>
        
        <button type="button" class="btn-refresh" onclick="salvaInEEPROM()">Aggiorna</button>
    </form>

    <table>
        <thead>
            <tr>
                <th>Address</th>
                <th>Tipo</th>
                <th>Descrizione</th>
                <th>Valore</th>
                <th>Azioni</th>
            </tr>
        </thead>
        <tbody id="tabella-dati">
            <!-- Dati inseriti dinamicamente via script -->
        </tbody>
    </table>

    <script>
        window.addEventListener('DOMContentLoaded', () => {
            caricaConfig();
            caricaDati(); 
        });

        function caricaDati() {
            fetch('/data')
                .then(response => response.text())
                .then(txt => {
                    const tbody = document.getElementById('tabella-dati');
                    tbody.innerHTML = '';
                    if(!txt.trim()) {
                        document.getElementById('count').innerText = '0';
                        return;
                    }
                    const linee = txt.split('\n');
                    let count = 0;
                    
                    linee.forEach(linea => {
                        if(!linea.trim()) return;
                        const token = linea.split(',');
                        const id = token[0];
                        const addr = token[1];
                        const tipo = token[2];
                        const desc = token[3];
                        const val = token[4];
                        
                        const row = document.createElement('tr');
                        row.innerHTML = `
                            <td>${addr}</td>
                            <td>${tipo}</td>
                            <td>${desc}</td>
                            <td>${val}</td>
                            <td>
                                <button type="button" class="btn-edit" onclick="preparaModifica(${id}, '${addr}', '${tipo}', '${desc}', '${val}')">Modifica</button>
                                <button type="button" class="btn-del" onclick="elimina(${id})">Elimina</button>
                            </td>
                        `;
                        tbody.appendChild(row);
                        count++;
                    });
                    document.getElementById('count').innerText = count;
                });
        }

        function caricaConfig() {
            fetch('/get-config')
                .then(response => response.text())
                .then(stato => {
                    document.getElementById('chk-tapparelle').checked = (stato.trim() === '1');
                });
        }

        function salvaInEEPROM() {
            const btn = document.querySelector('.btn-refresh');
            const testoOriginale = btn.innerText;
            const chkTapparelle = document.getElementById('chk-tapparelle').checked ? '1' : '0';
            
            btn.innerText = 'Memorizzazione...';
            btn.disabled = true;

            fetch(`/refresh?tapparelle=${chkTapparelle}`, { method: 'POST' })
                .then(response => response.text())
                .then(risposta => {
                    if(risposta.trim() === "OK") {
                        alert('Logica di memorizzazione eseguita con successo!');
                    } else {
                        alert('Errore restituito dall\'ESP32.');
                    }
                    btn.innerText = testoOriginale;
                    btn.disabled = false;
                })
                .catch(err => {
                    alert('Errore di connessione con l\'ESP32');
                    btn.innerText = testoOriginale;
                    btn.disabled = false;
                });
        }

        document.getElementById('main-form').addEventListener('submit', function(e) {
            const addrVal = document.getElementById('form-address').value;
            if (addrVal === '00') {
                alert('L\'indirizzo 00 non è valido. Inserire un valore da 01 a FF.');
                e.preventDefault();
            }
        });

        function preparaModifica(id, addr, tipo, desc, val) {
            document.getElementById('form-id').value = id;
            document.getElementById('form-address').value = addr;
            document.getElementById('form-tipo').value = tipo;
            document.getElementById('form-desc').value = desc;
            document.getElementById('form-valore').value = val;
            
            document.getElementById('btn-submit').innerText = 'Salva';
            document.getElementById('btn-submit').style.backgroundColor = '#007bff';
            document.getElementById('btn-cancel').style.display = 'inline-block';
            window.scrollTo(0, 0);
        }

        function annullaModifica() {
            document.getElementById('form-id').value = '-1';
            document.getElementById('main-form').reset();
            document.getElementById('btn-submit').innerText = 'Aggiungi';
            document.getElementById('btn-submit').style.backgroundColor = '#28a745';
            document.getElementById('btn-cancel').style.display = 'none';
        }

        function elimina(id) {
            if(confirm('Eliminare questo elemento?')) {
                fetch(`/delete?id=${id}`, { method: 'POST' })
                    .then(() => caricaDati());
            }
        }
    </script>
</body>
</html>
)rawhtml";

#endif

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
#define DEV_NR            128 // numero massimo di devices - da 00 a A8 per scs
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

#define NEW_SWITCH_TOPIC "homeassistant/switch/";
#define NEW_LIGHT_TOPIC  "homeassistant/light/";
#define NEW_COVER_TOPIC  "homeassistant/cover/";
#define NEW_CONFIG_TOPIC "/config";
#define NEW_COVERPCT_TOPIC  "homeassistant/cover/";
#define NEW_SENSOR_TOPIC  "homeassistant/sensor/";
#define NEW_GENERIC_TOPIC "homeassistant/generic/";

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
#define NEW_DEVICE_END   "\"}";
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

char prevAction;
char internal;
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

AsyncWebServer a_server(http_port);

HTTPClient httpClient;

// Interfaccia HTML con layout a due colonne (Split Screen)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-C3 Control Panel</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 0; background-color: #1a1a1a; color: #f0f0f0; height: 100vh; display: flex; flex-direction: column; }
        h2 { text-align: center; margin: 15px 0; color: #00ff66; }
        
        /* Layout principale a due colonne */
        .container { display: flex; flex: 1; height: calc(100vh - 60px); overflow: hidden; }
        
        /* Colonna di Sinistra: Log */
        .col-left { width: 50%; padding: 20px; border-right: 2px solid #333; display: flex; flex-direction: column; }
        #log-container { 
            flex: 1; background-color: #0c0c0c; border: 1px solid #444; border-radius: 6px; padding: 15px; 
//            font-family: 'Courier New', Courier, monospace; overflow-y: auto; display: flex; flex-direction: column-reverse; 
            font-family: 'Courier New', Courier, monospace; overflow-y: auto; display: flex; flex-direction: column; 
        }
//        .log-entry { margin: 4px 0; font-size: 14px; line-height: 1.4; color: #80ea6e; }
        .log-entry { margin: 4px 0; line-height: 1.4; font-size: 14px; }
        .info { color: #80ea6e; }
        .warn { color: #ffb300; }
        .error { color: #ff5252; }
        
        /* Colonna di Destra: Input */
        .col-right { width: 50%; padding: 20px; display: flex; flex-direction: column; justify-content: center; align-items: center; }
        .card-input { background-color: #262626; border: 1px solid #444; border-radius: 8px; padding: 30px; width: 80%; max-width: 400px; box-shadow: 0 4px 10px rgba(0,0,0,0.3); }
        .card-input h3 { margin-top: 0; color: #00bfff; }
        input[type="text"] { width: 100%; padding: 12px; margin: 15px 0; border: 1px solid #555; border-radius: 4px; background-color: #333; color: #fff; font-size: 16px; }
        input[type="text"]:focus { border-color: #00bfff; outline: none; }
        button { width: 100%; padding: 12px; background-color: #00bfff; border: none; border-radius: 4px; color: #1a1a1a; font-size: 16px; font-weight: bold; cursor: pointer; transition: background 0.2s; }
        button:hover { background-color: #0099cc; }
        #status-msg { margin-top: 10px; font-size: 14px; text-align: center; }
    </style>
</head>
<body>
    <h2>Pannello di Controllo ESP32-C3</h2>
    
    <div class="container">
        <!-- COLONNA SINISTRA: LOG -->
        <div class="col-left">
            <div id="log-container">
                <div class="log-entry" style="color: #888;">In attesa di log SCS...</div>
            </div>
        </div>
        
        <!-- COLONNA DESTRA: INPUT -->
        <div class="col-right">
            <div class="card-input">
                <h3>Invia Comando</h3>
                <input type="text" id="comando-input" placeholder="Scrivi qui il comando..." maxlength="50">
                <button onclick="inviaDati()">Esegui</button>
                <div id="status-msg"></div>
            </div>
        </div>
    </div>

    <script>
        // Gestione SSE (Log a sinistra)
        if (!!window.EventSource) {
            var source = new EventSource('/events');
            var container = document.getElementById('log-container');

            source.addEventListener('log', function(e) {
                if (container.children.length === 1 && container.children[0].style.color === 'rgb(136, 136, 136)') {
                    container.innerHTML = '';
                }
                var entry = document.createElement('div');
                entry.className = 'log-entry';

  // Formatta il testo (es. colora in base al tipo di log)
                if (e.data.includes(' TX:')) entry.classList.add('error');
                else if (e.data.includes('[CMD]')) entry.classList.add('warn');
                else entry.classList.add('info');

  // Ripristina i veri a capo convertendo "\\n" in "\n"
                var logFormattato = e.data.replace(/\\n/g, '\n');

//                entry.innerHTML = e.data;   
                entry.innerHTML = logFormattato.replace(/\n/g, '<br>'); // Sostituisce i veri a capo con <br> per HTML
                container.appendChild(entry); 

//                newLog.innerHTML = `<span>[${new Date().toLocaleTimeString()}]</span> ${e.data}`; // <----------------------------
//                container.appendChild(newLog); // <----------------------------

                if (container.children.length > 20) {
                    container.removeChild(container.firstChild);
                }
            }, false);

            source.addEventListener('open', function(e) {
                console.log('Flusso log aperto.');
            }, false);

            source.addEventListener('chiudi_connessione', function(e) {
                container.innerHTML = 'Connessione chiusa.';
                console.log("Il server mi ha chiesto di chiudere.");
                source.close(); // Questo blocca definitivamente i tentativi di riconnessione automatica del browser
            });


        }

        // Invio Input asincrono (Destra) senza ricaricare la pagina
        function inviaDati() {
            var inputEl = document.getElementById('comando-input');
            var statusEl = document.getElementById('status-msg');
            var valore = inputEl.value.trim();
            
            if (valore === "") return;

            statusEl.innerHTML = "Invio in corso...";
            statusEl.style.color = "#aaa";

            // Utilizziamo Fetch API per fare un POST asincrono all'ESP32
            fetch('/invia', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: 'testo=' + encodeURIComponent(valore)
            })
            .then(response => {
                if (response.ok) {
                    statusEl.innerHTML = "Inviato con successo!";
                    statusEl.style.color = "#00ff66";
                    inputEl.value = ""; // Svuota il campo di testo
                } else {
                    statusEl.innerHTML = "Errore server.";
                    statusEl.style.color = "#ff5252";
                }
            })
            .catch(error => {
                statusEl.innerHTML = "Errore di rete.";
                statusEl.style.color = "#ff5252";
            });
        }
    </script>
</body>
</html>
)rawliteral";



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

#define E_MQTT_TABDEVICES 324	// mqtt_device max 128x4=512 -> 836
#define E_MQTT_TABLEN       4	// 
// ==============================================================================================================
typedef union _DEVICE    {
  struct {
        char address;	      
        char deviceType;		      
        char alexa_id;		      
        };
  struct {
        char addressW;			      
        char DeviceType;		      
        char Alexa_id;		      
        };
  struct {
        long int  Val;			      
        };
} DEVADDR;
// ==============================================================================================================
DEVADDR device_BUS_id[DEV_NR]; // index:device index,  contenuto: device address reale (scs o knx) e tipo
char alexa_BUS_ix[DEV_NR];     // index:id alexa, contenuto: device index

// ==============================================================================================================
#define E_ALEXA_DESC_DEVICE 836 // device description max DEV_NR x E_ALEXA_DESC_LEN char 
#define E_ALEXA_DESC_LEN     20 // device description max length - top 
// 836 + 128*20 = 3396
// ==============================================================================================================

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
    createWebServer(webtype);
    // Start the a_server
//    a_server.begin(HTTP_PORT);
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
#ifdef LOWPOWER
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
#endif

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
#ifdef LOWPOWER
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
#endif
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
// ===============================SCRITTURA UART IMMEDIATA FUORI CICLO==================================
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

  requestBuffer[requestLen++] = '@';
  requestBuffer[requestLen++] = 0x15; // evita memo in eeprom (in 0x17)

  requestBuffer[requestLen++] = '@'; 
  requestBuffer[requestLen++] = 'M'; // (in 0x17)
  requestBuffer[requestLen++] = 'X'; // (in 0x17)

  requestBuffer[requestLen++] = '@';
  requestBuffer[requestLen++] = 'Y'; // (in 0x17)
  requestBuffer[requestLen++] = '1'; // (in 0x17)

  requestBuffer[requestLen++] = '@';
  requestBuffer[requestLen++] = 'F'; // (in 0x17)
  requestBuffer[requestLen++] = '3';
  requestBuffer[requestLen++] = PIC_REQ;
  requestBuffer[requestLen++] = 'l'; // (in 0x17)
  uartSemaphor = 0;

  firstTime = 1;
  udp_remote_ip = {0, 0, 0, 0}; // kill udp connection
}
/*
// -----------------------------------------------------------------------------------------------------------------------------------------------
String ReadEEP(int eeaddress) 
{
  unsigned char a = 1;
  unsigned char i;
  String s = "";

  while (a)
  {
    a = EEPROM.read(eeaddress++);
    if (i++ > 128) a = 0;
    if ((a < 0x20) || (a > 0x7F))   a = 0;   // only ASCII
    if (a)  s += char(a);
  }
  return s;
}
// -----------------------------------------------------------------------------------------------------------------------------------------------
char ReadEEP(int eeaddress) 
{
    return = EEPROM.read(eeaddress);
}
*/
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
char WriteDescrOfIx(String edesc, char scsdevX)
{
  if (scsdevX < DEV_NR)
  {
      WriteEEP(edesc, scsdevX * E_ALEXA_DESC_LEN + E_ALEXA_DESC_DEVICE, E_ALEXA_DESC_LEN);
      return 1;
  }
  return 0;
}
//--------------------------------------------------------------------




//--------------------------------------------------------------------
char ixOfDeviceNew(char device)
{
  char x = 1;
  if (device == 0)  return 0;
  while (x < DEV_NR)
  {
    if (device_BUS_id[x].address == device)
        return x;
    else
    if (device_BUS_id[x].address == 0)
    { 
        device_BUS_id[x].address = device;
        device_BUS_id[x].deviceType = 1;
        device_BUS_id[x].alexa_id = 0;
        return x;
    }
	x++;
  }
  return 0;
}
//--------------------------------------------------------------------
char ixOfDeviceNew(char * scsdevice)
{
  char *ch;
  char x = 1;
  
  *(scsdevice+2) = 0; // 2 caratteri
  char device = (char)strtoul(scsdevice, &ch, 16);

  if (device == 0)  return 0;

  while (x < DEV_NR)
  {
    if (device_BUS_id[x].address == device)
        return x;
    else
    if (device_BUS_id[x].address == 0)
    { 
        device_BUS_id[x].address = device;
        device_BUS_id[x].deviceType = 1;
        device_BUS_id[x].alexa_id = 0;
        return x;
    }
	x++;
  }
  return 0;
}
//--------------------------------------------------------------------
char ixOfDevice(char * scsdevice)
{
  char *ch;
  *(scsdevice+2) = 0; // 2 caratteri
  char device = (char)strtoul(scsdevice, &ch, 16);
  if (device == 0)  return 0;

  char x = 1;
  while (x < DEV_NR)
  {
    if (device_BUS_id[x].address == device)
        return x;
    else
    if (device_BUS_id[x].address == 0)
		return 0;
	x++;
  }
  return 0;
}
//--------------------------------------------------------------------
char ixOfDevice(DEVADDR device)
{
  char x = 1;
  if (device.address == 0)  return 0;
  while (x < DEV_NR)
  {
    if (device_BUS_id[x].address == device.address)
        return x;
    else
    if (device_BUS_id[x].address == 0)
    return 0;
  x++;
  }
  return 0;
}
//--------------------------------------------------------------------
char ixOfDevice(char device)
{
  char x = 1;
  if (device == 0)  return 0;
  while (x < DEV_NR)
  {
    if (device_BUS_id[x].address == device)
        return x;
    else
    if (device_BUS_id[x].address == 0)
    return 0;
  x++;
  }
  return 0;
}
//--------------------------------------------------------------------
char DeviceOfIx(char ixdevice)
{
  return device_BUS_id[ixdevice].address;
}
//--------------------------------------------------------------------
char DeviceOfIx(char ixdevice, char * busname)
{
  sprintf(busname, "%02X", device_BUS_id[ixdevice].address);
  return device_BUS_id[ixdevice].address;
}
//--------------------------------------------------------------------
char DeviceOfIxPlus(char ixdevice, char * busname)
{
  sprintf(busname, "%02X", device_BUS_id[ixdevice].address);
  return device_BUS_id[ixdevice].address;
}
//--------------------------------------------------------------------
String descrOfIx(char scsdevX)
{
  if (scsdevX < DEV_NR)
  {
      String edes = ReadStream(&edes[0], (int)E_ALEXA_DESC_DEVICE + (scsdevX * E_ALEXA_DESC_LEN), E_ALEXA_DESC_LEN, 2); // tipo=0 binary array   1:ascii array   2:ascii string
      return edes;
  }
  else
  {
      char cdes[E_ALEXA_DESC_LEN];
      sprintf(cdes, "dispositivo scs %02X", devIx);  // device
      String edes(cdes);
      return edes;
  }
}
/*
//--------------------------------------------------------------------
void notFound(AsyncWebServerRequest *request) {
//    request->send(404, "text/plain", "Not found");
  if (firstTime == 0) setFirst();
  String message = "File non trovato\n\n";
  
  message += "URI: ";
  message += a_server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
    
  request->send(404, "text/plain", message);
  WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
  EEPROM.commit();
}
*/

// =============================================================================================
char aConvert(String aData)
{
  char str[4];
  char *ptr;
  long ret;
  aData.toCharArray(str, 4);
  str[2] = 0;
  ret = strtoul(str, &ptr, 16);
  return (char) ret;
}
// =============================================================================================
word wConvert(String aData)
{
  char str[6];
  char *ptr;
  long ret;
  aData.toCharArray(str, 5);
  str[4] = 0;
  ret = strtoul(str, &ptr, 16);
  return (word) ret;
}

// devtype: 1=SWITCH, 3=LIGHT, 4=LIGHT, 8=COVER, 11=GENERIC, 14=ALARM BOARD, 15=termostato, 18=COVER, 19=COVER PCT

// -----------------------------------------------------------------------------------------------------------------------------------------------
char  MQTTnewdiscover(char devtype, char * addrDevice, String nomeDevice)
{
  char rc = 0;
  String topic;
  String payload;
  if (devtype == 1)  // D=define new device SWITCH <<<-----------------------------------------------
  {
    rc = 1;
    topic   = NEW_SWITCH_TOPIC;
    topic  += addrDevice;
    topic  += NEW_CONFIG_TOPIC;

    payload = NEW_DEVICE_NAME;
    payload += nomeDevice;
    payload += NEW_SWITCH_SET;
    payload += addrDevice;
    payload += NEW_SWITCH_STATE;
//    payload += addrDevice;
//    payload += NEW_DEVICE_END;
  }
  else if ((devtype == 3) || (devtype == 4)) // D=define new device DIMMER <<<-----------------------------------------------
  {
    rc = 1;
    topic   = NEW_LIGHT_TOPIC;
    topic  += addrDevice;
    topic  += NEW_CONFIG_TOPIC;

    payload = NEW_DEVICE_NAME;
    payload += nomeDevice;
    payload += NEW_LIGHT_SET;
    payload += addrDevice;
    payload += NEW_LIGHT_STATE;
    payload += addrDevice;
    payload += NEW_BRIGHT_SET;
    payload += addrDevice;
    payload += NEW_BRIGHT_STATE;
//    payload += addrDevice;
//    payload += NEW_DEVICE_END;
  }
  else if ((devtype == 8) || (devtype == 18)) // D=define new device COVER <<<-----------------------------------------------
  {
    rc = 1;
    topic   = NEW_COVER_TOPIC;
    topic  += addrDevice;
    topic  += NEW_CONFIG_TOPIC;

    payload = NEW_DEVICE_NAME;
    payload += nomeDevice;
    payload += NEW_COVER_SET;
    payload += addrDevice;
    payload += NEW_COVER_STATE;
//    payload += addrDevice;
//    payload += NEW_DEVICE_END;
  }
  else if (devtype == 11) // D=define new device GENERIC <<<-----------------------------------------------
  {
    rc = 1;
    topic   = NEW_GENERIC_TOPIC;
    topic  += addrDevice;
    topic  += NEW_CONFIG_TOPIC;

    payload = NEW_DEVICE_NAME;
    payload += nomeDevice;
    payload += NEW_GENERIC_SET;
    payload += addrDevice;
    payload += NEW_GENERIC_FROM;
    payload += addrDevice;
    payload += NEW_GENERIC_TO;
//    payload += addrDevice;
//    payload += NEW_DEVICE_END;
  }
  else if ((devtype == 9) || (devtype == 19)) // D=define new device COVER PCT<<<-----------------------------------------------
  {
    rc = 1;
    topic   = NEW_COVERPCT_TOPIC;
    topic  += addrDevice;
    topic  += NEW_CONFIG_TOPIC;

    payload  = NEW_DEVICE_NAME;
    payload += nomeDevice;
    payload += NEW_COVER_SET;
    payload += addrDevice;
    payload += NEW_COVERPCT_SET;
    payload += addrDevice;
    ///      payload += NEW_COVER_STATE;  // cover percentuale, meglio non trattare lo stato
    ///      payload += addrDevice;       //                    ma solo la posizione
    payload += NEW_COVERPCT_STATE;
//    payload += addrDevice;
//    payload += NEW_DEVICE_END;
  }
  else if (devtype == 14) // 0x0E) // D=define new device ALARM BOARD<<<-----------------------------------------------
  {
  }
  if (rc == 1)
  {
    payload += addrDevice;
    payload += NEW_UNIQUE_ID;
    payload += addrDevice;
    payload += NEW_DEVICE_END;
    const char* cPayload = payload.c_str();
    const char* cTopic = topic.c_str();
    client.publish(cTopic, cPayload, false);
  }
  return rc;
}


// -----------------------------------------------------------------------------------------------------------------------------------------------
char  MQTTnewdevice(char devIx, char* nomedevice)  // KNX
{
// DEVADDR device_BUS_id[DEV_NR]; // pointer: eventuale id alexa - contenuto: device address reale (scs o knx) e tipo

  char devicename[6];
  DeviceOfIx(devIx, devicename);

  if (nomedevice) memcpy(nomedevice,devicename,6);

  String edesc = descrOfIx(devIx);
  if (mqttopen == 3)
  {
    if ((edesc[0] == 0) || (edesc[0] == 0xFF) || (edesc == ""))
      MQTTnewdiscover(device_BUS_id[devIx].deviceType, devicename, devicename);
    else
      MQTTnewdiscover(device_BUS_id[devIx].deviceType, devicename, edesc);
  }
  return devIx;
}

// =====================================================================================================
void newTest()
{
  char apice = 39;
  char linea = 10;

  content = R"=====(
<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8">
        <meta http-equiv="X-UA-Compatible" content="IE=edge">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Home Controller</title>
        <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.5.1/jquery.min.js"></script> 
        <link rel="stylesheet" href="https://stackpath.bootstrapcdn.com/bootstrap/4.4.1/css/bootstrap.min.css" integrity="sha384-Vkoo8x4CGsO3+Hhxv8T/Q5PaXtkKtu6ug5TOeNV6gBiFeWPGFN9MuhOf23Q9Ifjh" crossorigin="anonymous">
    </head>
    <body style="padding-left: 10px">
        <h2 style="text-align: center; margin-top: 0px">LUCI</h2>
        <br/>
)=====";

// ------------------------------ luci --------------------------------------------------------

        char devtype;
        char nomeDevice[6];
        char devx = 1;
        while ((devx < DEV_NR) && (device_BUS_id[devx].addressW))
        {
          devtype = device_BUS_id[devx].deviceType;
          if ((devtype == 1) || (devtype == 4))
          {
            DeviceOfIx(devx, (char *)nomeDevice);
            content += " <div class=\"row\">";
            content += " <div class=\"col-6\">";
            content += descrOfIx(devx);

            content += "</div><div class=\"col-2\" style=\"color: green\" onclick=\"callAPI2(";
            content += apice;
            content += nomeDevice;
            content += apice;
            content += ",\'00\')\">ON</div>";

            content += "<div class=\"col-2\" style=\"color: red\" onclick=\"callAPI2(";
            content += apice;
            content += nomeDevice;
            content += apice;
            content += ",\'01\')\">OFF</div></div>";
            content += R"=====(
)=====";
            
          } // devtype == 1
          devx++;
        }  // while

        content += R"=====(
        <br/>
        <h2 style="text-align: center; margin-top: 0px">TAPPARELLE</h2>
        <br/>
)=====";

// ------------------------------ tapparelle --------------------------------------------------

        char nomeDevicePlus[6];
        devx = 1;
        while ((devx < DEV_NR) && (device_BUS_id[devx].addressW))
        {
          devtype = device_BUS_id[devx].deviceType;
          if ((devtype == 8) || (devtype == 9) || (devtype == 18) || (devtype == 19))
          {
            DeviceOfIx(devx, (char *)nomeDevice);
            DeviceOfIxPlus(devx, (char *)nomeDevicePlus);
            content += " <div class=\"row\">";
            content += " <div class=\"col-5\">";
            content += descrOfIx(devx);

            content += "</div><div class=\"col-2\" style=\"color: green\" onclick=\"callAPI2(";
            content += apice;
            content += nomeDevicePlus;
            content += apice;
            content += ",\'08\')\">UP</div>";
            content += "<div class=\"col-3\" style=\"color: red\" onclick=\"callAPI2(";
            content += apice;
            content += nomeDevicePlus;
            content += apice;
            content += ",\'09\')\">DOWN</div>";
            content += "<div class=\"col-2\" style=\"color: blue\" onclick=\"callAPI2(";
            content += apice;
            content += nomeDevice;
            content += apice;
            content += ",\'0A\')\">STOP</div></div>";
            content += R"=====(
)=====";
            
          } // devtype == 1
          devx++;
        }  // while



  content += R"=====(
        <br/>
    </body>
    <script>
        function callAPI2(to, cmd) {
            var request = new XMLHttpRequest()
)=====";


  content += "                request.open('GET', 'http://";
  content += WiFi.localIP().toString();
  if (alexaParam == 'y')
      content += ":8080";
  
  content += "/gate?to=";
  content += apice;
  
  content += " + to + '&cmd=' + cmd + '&resp=y', true)";  // non si riesce a mettere un apice dopo 0C

  content += R"=====(
            request.send()
        }
        function callAPI(to,cmd) {
            callAPI2('0C' + to, cmd);
        }
    </script>
</html>
)=====";
  
}
/*
// =============================================================================================

void handleClear()
{
  content = "<!DOCTYPE HTML>\r\n<html>";
  content += "<p>Clearing the EEPROM - reboot.</p></html>";
  server.send(200, "text/html", content);
  content = "";
  for (int i = 0; i < 4096; ++i) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  WiFi.disconnect();
}
*/

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

// ===============================================================================================
// MQTT callback function  - comando proveniente da MQTT:  desktop o pulsante fisico
// ===============================================================================================
void MqttCallback(char* topic, byte* payload, unsigned int length)
// =============================================================================================================
// ================================ messaggio MQTT in arrivo (comando web) =====================================
// =============================================================================================================
{
  int i = 0;
  char dev[5];
  char *ch;
  unsigned char devtype = 0;
  char packetBuffer[255];

  unsigned char device = 0xFF;

  unsigned char reply = 0;
  unsigned char command = 0xFF;
  String rtopic;


  // create character buffer with ending null terminator (string)
  for (i = 0; i < length; i++) {
    packetBuffer[i] = payload[i];
  }
  packetBuffer[i] = '\0';

  String payloads = String(packetBuffer);
  String topicString = String(topic);
          
#ifdef USE_TCPSERVER
  if ((tcpuart == 2) && (tcpclient) && (tcpclient.connected())) 
  {
    String log = "\r\nsub: ";
    log += topicString;
    log += " ";
    log += payloads;
    tcpclient.write((char*)&log[0], log.length());
    tcpclient.flush(); 
  }
#endif
          

  if (firstTime == 0) setFirst();

  // --------------------------------------- SWITCHES -------------------------------------------
  if (topicString.substring(0, sizeof(SWITCH_SET) - 1) == SWITCH_SET)
  {
    devtype = 1; // switch
    rtopic = SWITCH_STATE;
    reply = 1;
    dev[0] = *(topic + sizeof(SWITCH_SET) - 1);
    dev[1] = *(topic + sizeof(SWITCH_SET));
    dev[2] = 0;
    device = (char)strtoul(dev, &ch, 16);
    if (payloads.substring(0, 2) == "ON")
      command = 0x00;
    else if (payloads.substring(0, 3) == "OFF")
      command = 0x01;
  }
  // ----------------------------------------------------------------------------------------------
  else

    // --------------------------------------- LIGHTS DIMM ------------------------------------------
    if (topicString.substring(0, sizeof(BRIGHT_SET) - 1) == BRIGHT_SET)
    {
#ifdef DEBUG
      rtopic = BRIGHT_STATE;
      Serial.println("dimmer");
#endif
      dev[0] = *(topic + sizeof(BRIGHT_SET) - 1);
      dev[1] = *(topic + sizeof(BRIGHT_SET));
      dev[2] = 0;
      device = (char)strtoul(dev, &ch, 16);
      devtype = 3; // light
      if (payloads.substring(0, 2) == "ON")  // trasformare da % a 1D-9D <--------------------------------------------------------------
        command = 0x00;  // on/up
      else if (payloads.substring(0, 3) == "OFF")
        command = 0x01; // off/down
      else
      {
        int pct = atoi(packetBuffer);    // percentuale
        // --------------- percentuale da 1 a 255 (home assistant) -----------
        if ((domoticMode == 'h') || (domoticMode == 'H'))   // h=as homeassistant
        { // percentuale 0-255
          pct *= 100;                  // da 0 a 25500
          pct /= 255;                  // da 0 a 100
        }
        // --------------- percentuale da 1 a 100 (domoticz) -----------------
        // percentuale 0-100
        pct += 5;                    // arrotondamento
        pct /= 10;                   // 0-10
        if (pct > 9) pct = 9;
        if (pct == 0) pct = 1;		    // 1-9
        pct *= 16;                   // hex high nibble
        pct += 0x0D;                 // hex low  nibble
        command = (unsigned char) pct;

#ifdef DEBUG
        Serial.println(" payload " + content);
        //     char hBuffer[4];
        //     sprintf(hBuffer, "%02X", pct);
        // ricalcolare payloads per debug
        char action = pct;
        action >>= 4;
        action *= 10;  // percentuale 10-90
        char actionc[4];
        sprintf(actionc, "%02u", action);
        payloads = String(actionc);
#endif
      }
    }
  // ----------------------------------------------------------------------------------------------
    else





      // --------------------------------------- COVERPCT ------------------------------------------------
      if (topicString.substring(0, sizeof(COVERPCT_SET) - 1) == COVERPCT_SET)
      {
#ifdef DEBUG
        Serial.println("cover%");
#endif
        rtopic = COVERPCT_STATE;
        //    reply = 1;  // TEST
        dev[0] = *(topic + sizeof(COVERPCT_SET) - 1);
        dev[1] = *(topic + sizeof(COVERPCT_SET));
        dev[2] = 0;
        device = (word)strtoul(dev, &ch, 16);
        devtype = 9; // coverpct

        if (payloads.substring(0, 4) == "STOP")
        {
          command = 0;
        }
        else if (payloads.substring(0, 2) == "ON")  // discesa - chiudi
        {
          command = 2;
        }
        else if (payloads.substring(0, 5) == "CLOSE")  // discesa - chiudi
        {
          command = 2;
        }
        else if (payloads.substring(0, 3) == "OFF") // salita - apri
        {
          command = 1;
// in domoticz 
        }
        else if (payloads.substring(0, 4) == "OPEN") // salita - apri
        {
          command = 1;
        }
        else
        {
          int pct = atoi(packetBuffer);    // percentuale
          command = (unsigned char) pct;
        }
      }
  // ----------------------------------------------------------------------------------------------
      else


  // --------------------------------------- GENERIC  -------------------------------------------
  if (topicString.substring(0, sizeof(GENERIC_SET) - 1) == GENERIC_SET)
  {
    devtype = 11; // generic
    rtopic = GENERIC_TO;
    reply = 0;
    dev[0] = *(topic + sizeof(GENERIC_SET) - 1);
    dev[1] = *(topic + sizeof(GENERIC_SET));
    
    requestBuffer[requestLen++] = PIC_REQ;

	char from;
	char type;
	
    requestBuffer[requestLen++] = 'y';		// 0x79 (@y: invia a pic da MQTT cmd standard da inviare sul bus)

    dev[2] = 0;
    device = (char)strtoul(dev, &ch, 16);

    packetBuffer[6] = 0;        // packet: ffttcc ->  <from> <type> <command>
    command = (char)strtoul(&packetBuffer[4], &ch, 16);
    packetBuffer[4] = 0;
    type = (char)strtoul(&packetBuffer[2], &ch, 16);
    packetBuffer[2] = 0;
    from = (char)strtoul(&packetBuffer[0], &ch, 16);

    requestBuffer[requestLen++] = device; // to   device
    requestBuffer[requestLen++] = from;   // from device
    requestBuffer[requestLen++] = type;   // type:command
    requestBuffer[requestLen++] = command;// command
  }
  // ----------------------------------------------------------------------------------------------
  else








        // --------------------------------------- COVER ------------------------------------------------
        if (topicString.substring(0, sizeof(COVER_SET) - 1) == COVER_SET)
        {
#ifdef DEBUG
          Serial.println("cover");
#endif
          rtopic = COVER_STATE;
          reply = 1;
          devtype = 8; // cover
          dev[0] = *(topic + sizeof(COVER_SET) - 1);
          dev[1] = *(topic + sizeof(COVER_SET));
          dev[2] = 0;
          device = (char)strtoul(dev, &ch, 16);
          if (payloads.substring(0, 2) == "ON")   // discesa - chiudi
            command = 0x09;
          else if (payloads.substring(0, 5) == "CLOSE")  // discesa - chiudi
          {
            command = 0x09;
          }
          else if (payloads.substring(0, 3) == "OFF") // salita - apri
            command = 0x08;
          else if (payloads.substring(0, 4) == "OPEN") // salita - apri
          {
            command = 0x08;
          }
          else if (payloads.substring(0, 4) == "STOP")
            command = 0x0A;
        }
  // ----------------------------------------------------------------------------------------------

  if (reply == 1)
  {
    if ((domoticMode == 'h') || (domoticMode == 'H')) // home assistant
    {
      if (payloads.substring(0, 5) == "CLOSE")   // discesa - chiudi
        payloads = "closed";
      else if (payloads.substring(0, 4) == "OPEN")  // salita - apri
        payloads = "open";
    }

    rtopic += dev;
    const char* cContent = payloads.c_str();
    const char* cTopic = rtopic.c_str();
    client.publish(cTopic, cContent, mqtt_persistence);
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






#ifdef MY_DOMOTIC


/*
// -----------------------------------------------------------------------------------------------------------------------------------------------
void reloadEntityOld(char firstload)
// -----------------------------------------------------------------------------------------------------------------------------------------------
{
// devtype: 1=SWITCH, 3=LIGHT, 4=LIGHT, 8=COVER, 11=GENERIC, 14=ALARM BOARD, 15=termostato, 18=COVER, 19=COVER PCT

    WORD_VAL maxp;
    maxp.Val = 0;

    uint8_t device  = 0;
    uint8_t devtype = 0;
    uint8_t deviceX = 0;
    char nomeDevice[6];
    char AlexaDescr[21];

    deviceX = 1;
    tapparellePercentuale = false;

    while (device_BUS_id[deviceX].addressW)
    {
      devtype = device_BUS_id[deviceX].deviceType;
      if (device_BUS_id[deviceX].addressW)
      {
        device = DeviceOfIx(deviceX, nomeDevice);
        String alexadescr = descrOfIx(deviceX);
        alexadescr.toCharArray(AlexaDescr, 21);  
        maxp.Val = 0;
/*
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
  */
 /*
        switch(devtype)
        {
          case 1:  // SWITCH
            aggiungiEntita(nomeDevice, AlexaDescr, ENTI_INTERRUTTORE, device, "OFF",0);
            break;
          case 3:  // DIMMER
          case 4:
            aggiungiEntita(nomeDevice, AlexaDescr, ENTI_SLIDER, device, "0",0);
            break;
          case 8:  // COVER
          case 18:
            aggiungiEntita(nomeDevice, AlexaDescr, ENTI_TAPPARELLA, device, "STOP",0);
            break;
          case 9:  // COVER PCT
          case 19:
            aggiungiEntita(nomeDevice, AlexaDescr, ENTI_TAPPARELLA_PCT, device, "0",0);
            tapparellePercentuale = true;
            break;
          case 11:  // GENERIC
            break;
          case 14:  // ALARM
            break;
          case 15:  // TERMO
            aggiungiEntita(nomeDevice, AlexaDescr, ENTI_SENSORE, device, "...",0);
            break;
        }
//        sprintf(tcpBuffer, "{\"device\":\"%s\",\"type\":\"%d\",\"maxp\":\"%d\",\"descr\":\"%s\"}",nomeDevice,devtype,maxp.Val,AlexaDescr);
      } // devtype > 0
      deviceX++;
    } // while devicex < DEV_NR
}
*/

// -----------------------------------------------------------------------------------------------------------------------------------------------
void reloadEntity(char firstload)
// -----------------------------------------------------------------------------------------------------------------------------------------------
{

Entita tempEntita[MY_ENTITA];
/*
struct Entita {
    String id;
    String nome;
    TipoEntita tipo;
    uint8_t pin;    // SCS address
    String stato;
    uint8_t valore;
};
*/
int numeroTemp = 0;

// devtype: 1=SWITCH, 3=LIGHT, 4=LIGHT, 8=COVER, 11=GENERIC, 14=ALARM BOARD, 15=termostato, 18=COVER, 19=COVER PCT

    WORD_VAL maxp;
    maxp.Val = 0;

    uint8_t device  = 0;
    uint8_t devtype = 0;
    uint8_t deviceX = 0;
    char nomeDevice[6];
    char AlexaDescr[21];

    deviceX = 1;
    tapparellePercentuale = false;

    while (device_BUS_id[deviceX].addressW)
    {
      devtype = device_BUS_id[deviceX].deviceType;
      if (device_BUS_id[deviceX].addressW)
      {
        device = DeviceOfIx(deviceX, nomeDevice);
        String alexadescr = descrOfIx(deviceX);
        alexadescr.toCharArray(AlexaDescr, 21);  
        maxp.Val = 0;
/*
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
  */
        String stato = "";
        uint8_t valore = 0;
        for (int i=0; i< numeroEntita; i++)
        {
            if ((elencoEntita[i].id == nomeDevice)
            &&  (corrispondenza[elencoEntita[i].tipo] == devtype))
            {
              stato = elencoEntita[i].stato;
              valore = elencoEntita[i].valore;
            }
        }        

        switch(devtype)
        {
          case 1:  // SWITCH
            if (stato == "")  stato = "OFF";
            tempEntita[numeroTemp] = {nomeDevice, AlexaDescr, ENTI_INTERRUTTORE, device, stato, valore};
            break;
          case 3:  // DIMMER
          case 4:
            if (stato == "")  stato = "OFF";
            tempEntita[numeroTemp] = {nomeDevice, AlexaDescr, ENTI_SLIDER, device, stato, valore};
            break;
          case 8:  // COVER
          case 18:
            if (stato == "")  stato = "STOP";
            tempEntita[numeroTemp] = {nomeDevice, AlexaDescr, ENTI_TAPPARELLA, device, stato, valore};
            break;
          case 9:  // COVER PCT
          case 19:
            if (stato == "")  stato = "STOP";
            tempEntita[numeroTemp] = {nomeDevice, AlexaDescr, ENTI_TAPPARELLA_PCT, device, stato, valore};
            tapparellePercentuale = true;
            break;
          case 11:  // GENERIC
            break;
          case 14:  // ALARM
            break;
          case 15:  // TERMO
            if (stato == "")  stato = "...";
            tempEntita[numeroTemp] = {nomeDevice, AlexaDescr, ENTI_SENSORE, device, stato, valore};
            break;
        }
//        sprintf(tcpBuffer, "{\"device\":\"%s\",\"type\":\"%d\",\"maxp\":\"%d\",\"descr\":\"%s\"}",nomeDevice,devtype,maxp.Val,AlexaDescr);
        numeroTemp++;
      } // devtype > 0
      deviceX++;
    } // while devicex < DEV_NR

    numeroEntita = 0;
    for (int i=0; i< numeroTemp; i++)
    {
      aggiungiEntita(tempEntita[i].id, tempEntita[i].nome, tempEntita[i].tipo, tempEntita[i].pin, tempEntita[i].stato, tempEntita[i].valore);
    }        
}


#endif

#ifdef MY_TABELLA

// -----------------------------------------------------------------------------------------------------------------------------------------------
void reloadDevices(void)
// -----------------------------------------------------------------------------------------------------------------------------------------------
{

    conteggioElementi = 0;

// devtype: 1=SWITCH, 3=LIGHT, 4=LIGHT, 8=COVER, 11=GENERIC, 14=ALARM BOARD, 15=termostato, 18=COVER, 19=COVER PCT
//            LUC       DIM      DIM      TAP       GEN         ALA             TER            TAP       TAP 
    WORD_VAL maxp;
    maxp.Val = 0;

    char device  = 0;
    char devtype = 0;
    char deviceX = 0;
    char nomeDevice[6];
    char AlexaDescr[21];

    deviceX = 1;
    while (device_BUS_id[deviceX].addressW)
    {
      devtype = device_BUS_id[deviceX].deviceType;
      if (device_BUS_id[deviceX].addressW)
      {
        device = DeviceOfIx(deviceX, nomeDevice);
        String alexadescr = descrOfIx(deviceX);
        alexadescr.toCharArray(AlexaDescr, 21);  
        maxp.Val = 0;
        strcpy(lista[conteggioElementi].address, nomeDevice);
        strcpy(lista[conteggioElementi].descrizione, AlexaDescr);
        lista[conteggioElementi].valido = true;
        sprintf(lista[conteggioElementi].tipo,"%02X", devtype);
        strcpy(lista[conteggioElementi].valore, "0");
        lista[conteggioElementi].bits.Val = 0;
        switch(devtype)
        {
          case 1:  // SWITCH
            strcpy(lista[conteggioElementi].tipo,"LUC");
            break;
          case 3:  // DIMMER
          case 4:
            strcpy(lista[conteggioElementi].tipo,"DIM");
            break;
          case 8:  // COVER
          case 18:
            strcpy(lista[conteggioElementi].tipo,"TAP");
            break;
          case 11:  // GENERIC
            strcpy(lista[conteggioElementi].tipo,"GEN");
            break;
          case 14:  // ALARM
            strcpy(lista[conteggioElementi].tipo,"ALA");
            break;
          case 15:  // TERMO
            strcpy(lista[conteggioElementi].tipo,"TER");
            break;
          case 9:  // COVER PCT
          case 19:
            tapparellePercentuale = true;
            strcpy(lista[conteggioElementi].tipo,"TAP");
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '6';
            requestBuffer[requestLen++] = device;
            immediateSend();
            delay(5);
            char m = immediateReceive('[');
            if (m > 8)
            {
              maxp.Val = replyBuffer[2] | replyBuffer[1] << 8;
            }
            sprintf(lista[conteggioElementi].valore,"%d", maxp.Val);
            break;
      }
      if (conteggioElementi<(MY_ENTITA - 1))
      {
        conteggioElementi++;
        lista[conteggioElementi].valido = false;
      }
    } // while devicex < DEV_NR
    deviceX++;
  }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------
void uploadDevices(char loadAll)
// -----------------------------------------------------------------------------------------------------------------------------------------------
{

//    int conteggioElementi = 0;

// devtype: 1=SWITCH, 3=LIGHT, 4=LIGHT, 8=COVER, 11=GENERIC, 14=ALARM BOARD, 15=termostato, 18=COVER, 19=COVER PCT
//            LUC       DIM      DIM      TAP       GEN         ALA             TER            TAP       TAP 
    WORD_VAL maxp;
    maxp.Val = 0;
    char *ch;

//    char busid  = 0;
    char device  = 0;
    char devtype = 0;
    char deviceX = 0;

    for(int i = 0; i < MY_ENTITA; i++) 
    {
      if (((!lista[i].valido) && (lista[i].bits.cancellata) && (!lista[i].bits.nuova) )
      ||  ((lista[i].valido) && (lista[i].bits.chiave) && (!lista[i].bits.nuova)))
      {
          loadAll = 1;
      }
    }

    if (loadAll == 1)
// prima bisogna cancellare tutto
    {
      for(int i = 0; i < MY_ENTITA; i++) 
      {
        if ((!lista[i].valido) && (lista[i].bits.cancellata))
            lista[i].bits.Val = 0;
        else
        if (lista[i].valido)
        {
          lista[i].bits.Val = 0;
          lista[i].bits.nuova = 1;
        }
      }

      if (tapparellePercentuale == false)
      {
        requestBuffer[requestLen++] = PIC_REQ;
        requestBuffer[requestLen++] = 'U';
        requestBuffer[requestLen++] = '9';
        immediateSend();
        delay(300);
        immediateReceive('k');
        delay(100);
      }  // cover == "false"

      for (int i = 0; i < DEV_NR; ++i)
      {
        EEPROM.write((int) i * E_ALEXA_DESC_LEN + E_ALEXA_DESC_DEVICE, 0);
	      device_BUS_id[i].Val = 0;
      }

      WriteEEP((char) 0, (int) E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
      WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
//    EEPROM.commit();
#ifdef DEBUG_EVENTITY
  char buflog[120];
  sprintf(buflog,"[%d] [CMD] clr ",(millis()/100));
  events.send(buflog, "log", millis());
#endif
    }


// **************************************************************************************************************

    for(int i = 0; i < MY_ENTITA; i++) 
    {
/*
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
    } bits;
*/

      if ((!lista[i].valido) && (lista[i].bits.cancellata))
      {
        deviceX = ixOfDevice(&lista[i].address[0]);
        if (deviceX)
        {
          device_BUS_id[deviceX].Val = 0;

// la pulizia nel PIC e' gia' stata fatta          

        }
      }
      else
      if(lista[i].bits.nuova) 
      {
//DEVADDR device_BUS_id[DEV_NR]; // index:device index,  contenuto: device address reale (scs o knx) e tipo
        deviceX = ixOfDeviceNew(&lista[i].address[0]);
        device = device_BUS_id[deviceX].address;
        if (deviceX)
        {
          device_BUS_id[deviceX].address = device; // (char)strtoul(lista[i].address, &ch, 16);
          device_BUS_id[deviceX].alexa_id = 0;        // rivedere
          if (strcmp(lista[i].tipo, "LUC") == 0)
              device_BUS_id[deviceX].deviceType = 1;
          else 
          if (strcmp(lista[i].tipo, "DIM") == 0)
              device_BUS_id[deviceX].deviceType = 3;
          else 
          if (strcmp(lista[i].tipo, "GEN") == 0)
              device_BUS_id[deviceX].deviceType = 11;
          else 
          if (strcmp(lista[i].tipo, "ALA") == 0)
              device_BUS_id[deviceX].deviceType = 14;
          else 
          if (strcmp(lista[i].tipo, "TER") == 0)
              device_BUS_id[deviceX].deviceType = 15;
          else 
          if (strcmp(lista[i].tipo, "TAP") == 0)
          {
              if (tapparellePercentuale == false)
                  device_BUS_id[deviceX].deviceType = 8;
              else
              {
                  device_BUS_id[deviceX].deviceType = 9;
                  char *ch;
                  maxp.Val = (int)strtoul(lista[i].valore, &ch, 10);
                  requestBuffer[requestLen++] = PIC_REQ;
                  requestBuffer[requestLen++] = 'U';
                  requestBuffer[requestLen++] = '8';
                  requestBuffer[requestLen++] = device;     // device id
                  requestBuffer[requestLen++] = device_BUS_id[deviceX].deviceType;    // device type
                  requestBuffer[requestLen++] = maxp.byte.HB;    // max position H
                  requestBuffer[requestLen++] = maxp.byte.LB;    // max position L
                  immediateSend();
                  immediateReceive('k');
                  delay(50);
              }
            }
            WriteDescrOfIx(lista[i].descrizione, deviceX);
          }
#ifdef DEBUG_EVENTITY
  char buflog[120];
  sprintf(buflog,"[%d] [CMD] new: %s %s %s : %02X, %d, [%d]",(millis()/100), lista[i].address, lista[i].tipo, lista[i].descrizione, device,device_BUS_id[deviceX].deviceType,deviceX);
  events.send(buflog, "log", millis());
#endif
      }
      else
      if(lista[i].bits.modificata) 
      {
  //DEVADDR device_BUS_id[DEV_NR]; // index:device index,  contenuto: device address reale (scs o knx) e tipo
          deviceX = ixOfDeviceNew(&lista[i].address[0]);
          device = device_BUS_id[deviceX].address;
          if (deviceX)
          {
  //          device_BUS_id[deviceX].address = device; // (char)strtoul(lista[i].address, &ch, 16);
  //          device_BUS_id[deviceX].alexa_id = 0;        // rivedere
            if (strcmp(lista[i].tipo, "LUC") == 0)
                device_BUS_id[deviceX].deviceType = 1;
            else 
            if (strcmp(lista[i].tipo, "DIM") == 0)
                device_BUS_id[deviceX].deviceType = 3;
            else 
            if (strcmp(lista[i].tipo, "GEN") == 0)
                device_BUS_id[deviceX].deviceType = 11;
            else 
            if (strcmp(lista[i].tipo, "ALA") == 0)
                device_BUS_id[deviceX].deviceType = 14;
            else 
            if (strcmp(lista[i].tipo, "TER") == 0)
                device_BUS_id[deviceX].deviceType = 15;
            else 
            if (strcmp(lista[i].tipo, "TAP") == 0)
            {
                if (tapparellePercentuale == false)
                    device_BUS_id[deviceX].deviceType = 8;
                else
                if (strcmp(lista[i].tipo, "TAP") == 0)
                {
                    if (tapparellePercentuale == false)
                        device_BUS_id[deviceX].deviceType = 8;
                    else
                    {
                        device_BUS_id[deviceX].deviceType = 9;
                        if(lista[i].bits.valore)
                        { 
                          char *ch;
                          maxp.Val = (int)strtoul(lista[i].valore, &ch, 10);
                          requestBuffer[requestLen++] = PIC_REQ;
                          requestBuffer[requestLen++] = 'U';
                          requestBuffer[requestLen++] = '8';
                          requestBuffer[requestLen++] = device;     // device id
                          requestBuffer[requestLen++] = device_BUS_id[deviceX].deviceType;    // device type
                          requestBuffer[requestLen++] = maxp.byte.HB;    // max position H
                          requestBuffer[requestLen++] = maxp.byte.LB;    // max position L
                          immediateSend();
                          immediateReceive('k');
                          delay(50);
                        }
                    }
                }
            }
         else 
            device_BUS_id[deviceX].deviceType = 1;
            if(lista[i].bits.descrizione) 
              WriteDescrOfIx(lista[i].descrizione, deviceX);
          }
#ifdef DEBUG_EVENTITY
  char buflog[120];
  sprintf(buflog,"[%d] [CMD] agg: %s %s %s : %02X, %d, [%d]",(millis()/100), lista[i].address, lista[i].tipo, lista[i].descrizione, device,device_BUS_id[deviceX].deviceType,deviceX);
  events.send(buflog, "log", millis());
#endif

      }
    }




}
#endif
// -----------------------------------------------------------------------------------------------------------------------------------------------
void createWebServer(int webtype)
// -----------------------------------------------------------------------------------------------------------------------------------------------
{


  if ( webtype == 0 ) // server web  on NETWORK
  {

#ifdef MY_TABELLA
      reloadDevices();

    // Servizio Pagina Principale HTML
    a_server.on("/dispositivi", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/html", index_html_TAB);
    });

      // API per inviare la lista in formato CSV (senza JSON)
    a_server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){

        String output = "";
        for(int i = 0; i < MY_ENTITA; i++) {
          if(lista[i].valido) {
            output += String(i) + "," + 
                      String(lista[i].address) + "," + 
                      String(lista[i].tipo) + "," + 
                      String(lista[i].descrizione) + "," + 
                      String(lista[i].valore) + "\n";
          }
        }
        request->send(200, "text/plain", output);
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
        if(lista[i].valido && addr.equals(String(lista[i].address))) {
          // Se stiamo inserendo un NUOVO elemento, blocca subito l'azione
          // Se stiamo MODIFICANDO, blocca solo se il duplicato si trova in uno slot DIVERSO da quello che stiamo modificando
          if(idModifica == -1 || idModifica != i) {
            request->send(400, "text/plain", "Errore: L'indirizzo (Address) inserito e' gia' presente in lista!");
            return;
          }
        }
      }
/*
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
    } bits;
*/
      // 3. DETERMINAZIONE DELLO SLOT DI DESTINAZIONE
      if (idModifica == -1) 
      {
        // NUOVO INSERIMENTO
        if(conteggioElementi >= MY_ENTITA) {
          request->send(400, "text/plain", "Errore: Lista piena (Max 50 elementi)");
          return;
        }
        // Trova il primo slot vuoto
        for(int i = 0; i < MY_ENTITA; i++) 
        {
          if(!lista[i].valido) 
          {
            targetSlot = i;
            lista[targetSlot].bits.Val = 0;
            lista[targetSlot].bits.nuova = 1;  // <-----------------------------------
            break;
          }
        }
      } else {
        // MODIFICA DI UN ELEMENTO ESISTENTE
        if (idModifica >= 0 && idModifica < MY_ENTITA && lista[idModifica].valido) {
          targetSlot = idModifica;            // <-----------------------------------
          lista[targetSlot].bits.modificata = 1;
        }
      }

      // 4. SALVATAGGIO DEI DATI IN RAM
      if(targetSlot != -1) {
        char temp[22];
        // Copia protetta nei buffer di array di caratteri (C-strings)

        strncpy(temp,addr.c_str(), 2);
        temp[2] = '\0'; // Garantisce la terminazione corretta del buffer di 3 byte
        if (strcmp(lista[targetSlot].address, temp) != 0)
        {
            lista[targetSlot].bits.chiave = 1;
            strcpy(lista[targetSlot].address, temp);
        }

        strncpy(temp,tipo.c_str(), 3);
        temp[3] = '\0'; // Garantisce la terminazione corretta del buffer
        if (strcmp(lista[targetSlot].address, temp) != 0)
        {
            lista[targetSlot].bits.tipo = 1;
            strcpy(lista[targetSlot].tipo, temp);
        }

        strncpy(temp,desc.c_str(), 20);
        temp[20] = '\0'; // Garantisce la terminazione corretta del buffer
        if (strcmp(lista[targetSlot].descrizione, temp) != 0)
        {
            lista[targetSlot].bits.descrizione = 1;
            strcpy(lista[targetSlot].descrizione, temp);
        }

        strncpy(temp,val.c_str(), 3);
        temp[3] = '\0'; // Garantisce la terminazione corretta del buffer
        if (strcmp(lista[targetSlot].valore, temp) != 0)
        {
            lista[targetSlot].bits.valore = 1;
            strcpy(lista[targetSlot].valore, temp);
        }

/*
        strncpy(lista[targetSlot].address, addr.c_str(), 2);
        lista[targetSlot].address[2] = '\0'; // Garantisce la terminazione corretta del buffer di 3 byte

        strncpy(lista[targetSlot].tipo, tipo.c_str(), 3);
        lista[targetSlot].tipo[3] = '\0';

        strncpy(lista[targetSlot].descrizione, desc.c_str(), 20);
        lista[targetSlot].descrizione[20] = '\0';

        strncpy(lista[targetSlot].valore, val.c_str(), 3);
        lista[targetSlot].valore[3] = '\0';
*/
        // Se era un nuovo elemento incrementa il contatore, se era una modifica resta invariato
        if (!lista[targetSlot].valido) {
          lista[targetSlot].valido = true;
          conteggioElementi++;
        }
#ifdef DEBUG_EVENTITY
  char buflog[120];
  char binlog[9];
  for (unsigned i = 0; i < 8; ++i) binlog[i] = 0x30 | ((lista[targetSlot].bits.Val >> (7 - i)) & 0x01);
  binlog[8] = 0;
  sprintf(buflog,"[%d] [CMD] xxx: %s %s %s : 0b%s, [%d]",conteggioElementi, lista[targetSlot].address, lista[targetSlot].tipo, lista[targetSlot].descrizione, binlog, targetSlot);
  events.send(buflog, "log", millis());
#endif
        
        request->redirect("/dispositivi");
      } else {
        request->send(400, "text/plain", "Errore: ID non valido o slot non allocabile.");
      }
    });

      // Cancellazione di un elemento tramite ID (Indice dell'array)
    a_server.on("/delete", HTTP_POST, [](AsyncWebServerRequest *request){
        if(request->hasParam("id")) {
          int id = request->getParam("id")->value().toInt();
          if(id >= 0 && id < MY_ENTITA && lista[id].valido) {
            lista[id].valido = false;
            lista[id].bits.cancellata = 1;
            conteggioElementi--;
#ifdef DEBUG_EVENTITY
    char buflog[120];
    char binlog[9];
    for (unsigned i = 0; i < 8; ++i) binlog[i] = 0x30 | ((lista[id].bits.Val >> (7 - i)) & 0x01);
    binlog[8] = 0;
    sprintf(buflog,"[%d] [CMD] dxx: %s %s %s : 0b%s, [%d]",conteggioElementi, lista[id].address, lista[id].tipo, lista[id].descrizione, binlog, id);
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
      bool newtapparellePercentuale = (val == "1");
      if (tapparellePercentuale == newtapparellePercentuale)
      {
        uploadDevices(0);
      }
      else
      {
        tapparellePercentuale = newtapparellePercentuale;
        uploadDevices(1);
      }

//      Serial.printf("Pulsante Memorizza premuto. Stato checkbox ricevuto: %s\n", tapparellePercentuale ? "ATTIVO" : "DISATTIVO");

      flagRefreshDevices = 1;
    }

    // Risposta di conferma per il browser
    request->send(200, "text/plain", "OK");
  });

  // Endpoint per dire al browser se la checkbox deve essere attiva o no
  a_server.on("/get-config", HTTP_GET, [](AsyncWebServerRequest *request){
    String stato = tapparellePercentuale ? "1" : "0";
    request->send(200, "text/plain", stato);
  });


#endif


#ifdef MY_DOMOTIC

    reloadEntity(1);

    a_server.on("/sseha", HTTP_GET, [](AsyncWebServerRequest *request){
        reloadEntity(0);
        request->send(200, "text/html", index_html_HA);
    });



   // 2. Endpoint JSON che JavaScript leggerà all'avvio per montare la pagina
    a_server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", generaConfigurazioneJSON());
    });

    // 3. Endpoint API universale per i pulsanti
    a_server.on("/api/toggle", HTTP_POST, [](AsyncWebServerRequest *request) {

// SWITCH_SET
        if (request->hasParam("id")) {
            unsigned char command = 0;
            unsigned char device = 0;
            String idRichiesto = request->getParam("id")->value();
            for (int i = 0; i < numeroEntita; i++) {
                if ((elencoEntita[i].id == idRichiesto) && 
                    (elencoEntita[i].tipo == ENTI_INTERRUTTORE) || (elencoEntita[i].tipo == ENTI_SLIDER)) 
                {

                  // Toggle dello stato dell'entità - comando SCS <******************************************************+

                    elencoEntita[i].stato = (elencoEntita[i].stato == "ON") ? "OFF" : "ON";
                    device = elencoEntita[i].pin;
                    if (elencoEntita[i].stato == "ON")
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
                    eventity.send(elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
                    break;
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
          
          for (int i = 0; i < numeroEntita; i++) {
              if (elencoEntita[i].id == idRichiesto && elencoEntita[i].tipo == ENTI_SLIDER) {
                  device = elencoEntita[i].pin;
//                  elencoEntita[i].stato = valoreRicevuto;
                  
                // comando SCS <******************************************************+
                  command = 0x00;  // on/up
                  int pct = valoreRicevuto.toInt();    // percentuale
                  elencoEntita[i].valore = pct;
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
                  eventity.send(String(elencoEntita[i].valore) + elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
                  break;
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
        
        for (int i = 0; i < numeroEntita; i++) {
            if (elencoEntita[i].id == idRichiesto && elencoEntita[i].tipo == ENTI_TAPPARELLA) {
                device = elencoEntita[i].pin;
                if (comando == "SALITA") //  && elencoEntita[i].pin != -1) 
                {
                    elencoEntita[i].stato = "SALITA";
                    command = 0x08;
                } 
                else if (comando == "DISCESA") // && elencoEntita[i].pin2 != -1) 
                {
                    elencoEntita[i].stato = "DISCESA";
                    command = 0x09;
                } 
                else 
                {
                    elencoEntita[i].stato = "STOP";
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
                eventity.send(elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
                
                // Salva lo stato su LittleFS
//                salvaStatiSuFile();
                break;
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
        
        for (int i = 0; i < numeroEntita; i++) {
            if (elencoEntita[i].id == idRichiesto && elencoEntita[i].tipo == ENTI_TAPPARELLA_PCT) {
                device = elencoEntita[i].pin;
// 0=stop   1=open    2=close   xxx=pct
                if (comando == "SALITA") //  && elencoEntita[i].pin != -1) 
                {
                    elencoEntita[i].stato = "SALITA";
                    command = 0x08;
//                    command = 1;
                } 
                else if (comando == "DISCESA") // && elencoEntita[i].pin2 != -1) 
                {
                    elencoEntita[i].stato = "DISCESA";
                    command = 0x09;
//                    command = 2;
                } 
                else if (comando == "STOP") // && elencoEntita[i].pin2 != -1) 
                {
                    elencoEntita[i].stato = "STOP";
                    command = 0x0A;
//                    command = 0;
                } 
                else 
                {
                    int pct = comando.toInt();    // percentuale
                    if (pct == 0)
                    {
                      elencoEntita[i].stato = "DISCESA";
                    }
                    else
                    if (pct == 100)
                    {
                      elencoEntita[i].stato = "SALITA";
                    }
                    requestBuffer[requestLen++] = PIC_REQ;
                    requestBuffer[requestLen++] = 'u';
                    requestBuffer[requestLen++] = device;		// to   device
                    requestBuffer[requestLen++] = (unsigned char) pct;    // command (%)
                    String payload = String(comando) + "|" + elencoEntita[i].stato;
                    eventity.send(payload.c_str(), elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
          String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + payload;
          events.send(rispostaLog.c_str(), "log", millis());
#endif
                    break;
                }

                requestLen = 0;
                requestBuffer[requestLen++] = PIC_REQ;
                requestBuffer[requestLen++] = 'y';    // 0x79 (@y: invia a pic da MQTT cmd standard da inviare sul bus)
                requestBuffer[requestLen++] = device; // to   device
                requestBuffer[requestLen++] = 0x00;   // from device
                requestBuffer[requestLen++] = 0x12;   // type:command
                requestBuffer[requestLen++] = command;// command
                // Invia l'aggiornamento SSE alla pagina Home
                eventity.send(elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
          String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + elencoEntita[i].stato;
          events.send(rispostaLog.c_str(), "log", millis());
#endif

                break;
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
            String comandoRicevuto = request->getParam("testo", true)->value();
            
            // Stampa sulla seriale dell'ESP32 il comando ricevuto
//            Serial.printf("Comando utente ricevuto: %s\n", comandoRicevuto.c_str());
            
            // Genera immediatamente un log confermando la ricezione
            String rispostaLog = "[" + String(millis()/100) + "s] [CMD] Eseguito: " + comandoRicevuto;
            events.send(rispostaLog.c_str(), "log", millis());

            requestLen = comandoRicevuto.length();
            comandoRicevuto.toCharArray(requestBuffer, requestLen+1);

            if ((comandoRicevuto == "stop") || (comandoRicevuto == "STOP"))
            {
                events.send("stop", "chiudi_connessione");
                setFirst();
            }
            else
            if ((comandoRicevuto == "reload") || (comandoRicevuto == "RELOAD"))
            {
#ifdef MY_DOMOTIC
              events.send("Entità ricaricate - fai refresh pagina /sseha ", "log", millis());
              reloadEntity(0);
              delay(5);
#endif
#ifdef MY_TABELLA
              events.send("Entità ricaricate - fai refresh pagina /devices ", "log", millis());
              reloadDevices();
              delay(5);
#endif
            }
            else
            {
              immediateSend();
              request->send(200, "text/plain", "OK");
            }
          } else {
            request->send(400, "text/plain", "Parametro mancante");
        }
    });



// =============================================================================================
    // Rotta Principale: serve la pagina HTML di controllo
    a_server.on("/sselog", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", index_html);
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
      content += "<li>/mqttdevices (mqtt devices discover)</li>";
      content += "<li>/devicename (mqtt devices names)</li>";
      content += "<li>/sselog     (SCS LOG)</li>";
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
/*
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
*/
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
      WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
      EEPROM.commit();
      setFirst();

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
        requestBuffer[requestLen++] = '@';
        requestBuffer[requestLen++] = '|';
        requestBuffer[requestLen++] = '|';
        content = "{\"status\":\"OK\"}";
        uartSemaphor = 0;
      }
      else if (param == "all")
      {
        uartSemaphor = 1;
        requestBuffer[requestLen++] = '@';
        requestBuffer[requestLen++] = '|';
        requestBuffer[requestLen++] = '|';
        uartSemaphor = 0;
        countRestart = 200;
        content = "{\"status\":\"OK - resetting...\"}";
      }
      else if (events.count() != 0)
      {
        content = "{\"status\":\"SSE closed\"}";

        events.send("stop", "chiudi_connessione");
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
        content += "<p><ol>";

        if (connectionType == 1) // web server AP
        {
          Serial1.write('@');   // set led lamps
          Serial1.write(0xF2);  // set led lamps high-freq (AP mode)
          content += "<li>";
          content += "Working as AP ";
          content += "</li>";
        }
        else
        {
          Serial1.write('@');   // set led lamps
          Serial1.write(0xF1);  // set led lamps std-freq (client mode)
          content += "<li>";
          content += "System frequency (Mhz): ";
          unsigned char frq = ESP.getCpuFreqMHz(); // returns the CPU frequency in MHz as an unsigned 8-bit integer
          content += String(frq);
          content += "</li>";

          content += "<li>";
          content += "Wifi connection ssid: ";
          char hBuffer[64];
          sprintf(hBuffer, "%s - Ch:%d - (%ddBm)", WiFi.SSID().c_str(), WiFi.channel(), WiFi.RSSI());
          content += hBuffer;
          content += "</li>";

          content += "<li>";
          content += "Router IP: ";
          content += WiFi.gatewayIP().toString();
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
        content += "</li>";

        if (ArduinoOTAflag > 0)
        {
          content += "<li>OTA update ready</li>";  
        }
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

        content += "</li>";

        if (alexaParam  == 'y')
        {
          content += "<li>fauxmo ECHO DOT direct connection</li>";
          sprintf(temp, "<li>  %d devices added</li>", id_fauxmo + 1);
          content += temp;
          sprintf(temp, "<li>  %d devices discovered</li>", fauxmo.discovered());
          content += temp;
        }
        else
        {
          content += "<li>MQTT broker is: ";
          content += mqtt_server;
          content += "   port: ";
          content += mqtt_port;
        }
        content += "</li>";

        if (mqttopen > 0)
        {
          content += "<li>";
          if (domoticMode == 'd')
            content += "mqtt model: DOMOTICZ";
          else if (domoticMode == 'h')
            content += "mqtt model: HOMEASSISTANT";
          else if (domoticMode == 'D')
            content += "mqtt model: DOMOTICZ 1-2";
          else if (domoticMode == 'H')
            content += "mqtt model: HOMEASSISTANT 1-2";
          else
            content += "mqtt model: GENERIC";
          content += "</li>";
        }
        char hBuf[4];
        content += "<li>Domotic options: ";
        sprintf(hBuf, "%02X", domotic_options);
        content += hBuf;
        content += "</li>";

        if ((mqttopen > 0) || (alexaParam == 'y'))
        {
          content += "<li>known devices in eeprom: ";
          char devtype;
          char devNr = 0;
          char nomeDevice[6];
          char device;
          char devx = 1;
          while ((devx < DEV_NR) && (device_BUS_id[devx].addressW))
          {
            devtype = device_BUS_id[devx].deviceType;
            if ((devtype > 0) && (devtype < 32))
            {
              devNr++;
              DeviceOfIx(devx, nomeDevice);
              content += nomeDevice;
              content += " ";
            }
            devx++;
          }
          content += "</li>";
        }

        content += "<li>";
        content += "ESP mode: " _MODO;
        content += "</li>";

        sprintf(temp, "<li>Free heap: %d bytes</li>", ESP.getFreeHeap());
        content += temp;

        content += "<li>";
        content += "PIC fw version: ";
        content += picfwVersion;
        content += "</li>";

        content += "<li>";
        content += "MQTT connection retries: ";
        content += String(mqtt_connections,DEC);
        content += "</li>";

      // @Qr
        Serial1.write(PIC_REQ);    // command mode 
        Serial1.write('Q');    // command: query 
        Serial1.write('r');    // query PIC reset nrs
        delay(2);            // wait 2ms
        char nrs = 0;
        nrs = Serial1.read();        // receive from serial USB
        content += "<li>";
        content += "ESP resets: ";
        content += String(nrs,DEC);
        content += "</li>";

        content += "<li>";
        content += "</li>";

        content += "<li>";
        content += "END of report ";
        content += "</li>";

        content += "</ol>";
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

//      to_send = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION;
      to_send = "<HTML><BODY>\r\nHello from ESP32_" _MODO "GATE " _FW_VERSION;
      to_send += " at ";
      to_send += WiFi.localIP().toString();
      to_send += "<p><ul>";

      if (connectionType == 1) // web server AP
      {
        Serial1.write('@');   // set led lamps
        Serial1.write(0xF2);  // set led lamps high-freq (AP mode)
        to_send += "<li>";
        to_send += "Working as AP ";
        to_send += "</li>";
      }
      else
      {
        Serial1.write('@');   // set led lamps
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

          WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
          EEPROM.commit();
          content += " ... please use .../reset?device=esp    to boot into new mqtt broker\"}";
          statusCode = 200;
        }
        request->send(statusCode, "text/html", content);
        content = "";
// =============================================================================================
    });





// =============================================================================================
    a_server.on("/mqttdevices", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
        String richiesta;
        richiesta = request->arg("request");

        uartSemaphor = 1;
        if (firstTime == 0) setFirst();  // DEVONO essere attivi @MX  e @l

        if (richiesta == "clear")
        {
          content = "{\"status\":\"OK - tab cleared...\"}";
          statusCode = 200;
          for (int i = 0; i < DEV_NR; ++i)
          {
            EEPROM.write((int) i * E_ALEXA_DESC_LEN + E_ALEXA_DESC_DEVICE, 0);
          device_BUS_id[i].Val = 0;
          }
          WriteEEP((char) 0, (int) E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
          EEPROM.commit();
          requestBuffer[requestLen++] = PIC_REQ;
          requestBuffer[requestLen++] = 'U';
          requestBuffer[requestLen++] = '9';
        }
        else

          if (richiesta == "prepare")
          {
            content = "{\"status\":\"OK - proceed\" ";
            if (mqttopen == 3)
              content += ", \"mqtt\":\"ready\"}";
            else
              content += ", \"mqtt\":\"CLOSED\"}";

            statusCode = 200;
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = DEVICEREQUEST;
            requestBuffer[requestLen++] = 0xFF;
            immediateSend();
            delay(200);       // wait 200mS
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '1';
          }


          else if (richiesta == "start")
          {
            content = "{\"status\":\"OK - started\" ";
            if (mqttopen == 3)
              content += ", \"mqtt\":\"ready\"}";
            else
              content += ", \"mqtt\":\"CLOSED\"}";
            statusCode = 200;

            devIx = 1;
            devCtr = 0;
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '2'; // fine censimento
            immediateSend();
            delay(600);       // wait 600mS due to pic eeprom write

            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = DEVICEREQUEST;
            requestBuffer[requestLen++] = devIx;
          }


        /*
          else
          if (richiesta == "term")
          {
            content = "{\"status\":\"OK - terminated...\"}";
            statusCode = 200;
            devIx = 0;
            WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
            EEPROM.commit();
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '3';
            immediateSend();
            delay(200); // wait 100mS due to eeprom write
          }
        */


          else if (richiesta == "stop")
          {
            content = "{\"status\":\"OK - stopped...\"}";
            statusCode = 200;
            devIx = 0;
            WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
            EEPROM.commit();
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '3';
            immediateSend();
            delay(200); // wait 100mS due to eeprom write
          }
          else if (richiesta == "resend")
          {
            if (mqttopen == 3)
            {
              content = "<!DOCTYPE HTML>\r\n<html>Hello from ESP32_" _MODO "GATE " _FW_VERSION;
              content += "<p><ol>";
              statusCode = 200;

              char devtype;
              char nomeDevice[6];
              char devx = 1;
              while ((devx < DEV_NR) && (device_BUS_id[devx].addressW))
              {
                devtype = device_BUS_id[devx].deviceType;
                if ((devtype > 0) && (devtype < 32))
                {
                  MQTTnewdevice(devx, nomeDevice);
                  content += "<li>";
                  content += nomeDevice;
                  if (devtype == 1)
                    content += "switch";
                  else if (devtype == 3)
                    content += "dimmer";
                  else if (devtype == 8)
                    content += "cover";
                  else if (devtype == 9)
                    content += "coverpct";
                  else if (devtype == 11)
                    content += "generic";
                  else if (devtype == 14) // 0x0E)
                    content += "alarm board";
                  else if (devtype == 15) // 0x0F)
                    content += "termo";
                  else if (devtype == 18)
                    content += "cover1";
                  else if (devtype == 19)
                    content += "coverpct1";

                  content += "</li>";
                }
                devx++;
              }
              content += "</ol>";
              content += "</form></html>";
            }
            else
            {
              content = "{\"status\":\"KO - MQTT service not available...\"}";
              statusCode = 400;
            }
          }


          else
          { // query
            if (devIx == 0)
            {
              content = "<!DOCTYPE HTML>\r\n<html>OK - list of discovered devices:";
              content += "<p><ol>";
              statusCode = 200;

              char devtype;
              char nomeDevice[6];
          char devx = 1;
              while ((devx < DEV_NR) && (device_BUS_id[devx].addressW))
              {
                devtype = device_BUS_id[devx].deviceType;
                if ((devtype > 0) && (devtype < 32))
                {
                  DeviceOfIx(devx, (char *)nomeDevice);
                  content += "<li>";
                  content += nomeDevice;
                  if (devtype == 1)
                    content += "switch";
                  else if (devtype == 3)
                    content += "dimmer";
                  else if (devtype == 8)
                    content += "cover";
                  else if (devtype == 18)
                    content += "cover U";
                  else if (devtype == 11)
                    content += "generic";
                  else if (devtype == 14) // 0x0E)
                    content += "alarm board";
                  else if (devtype == 15) // 0x0F)
                    content += "termo";
                  else if (devtype == 9)
                    content += "coverpct";
                  else if (devtype == 19)
                    content += "coverpct U";
                  
                  if ((devtype == 9) || (devtype == 19))
                  {
                    requestBuffer[requestLen++] = PIC_REQ;
                    requestBuffer[requestLen++] = 'U';
                    requestBuffer[requestLen++] = '6';
                    requestBuffer[requestLen++] = device_BUS_id[devx].address;
                    immediateSend();
                    char m = immediateReceive('[');
                    if (m > 8)
                    {
                      char hBuffer[32];
                      sprintf(hBuffer, " %02X%02X %02X%02X %02X %02X%02X %02X", replyBuffer[1], replyBuffer[2], replyBuffer[3], replyBuffer[4], replyBuffer[5], replyBuffer[6], replyBuffer[7], replyBuffer[8]);
                      content += hBuffer;
                    }
                    else
                    {
                      //              content += " ?parms? ";
                      char hBuffer[24];
                      sprintf(hBuffer, " (%d) ", m);
                      content += hBuffer;
                      char n = 1;
                      while (n < m)
                      {
                        sprintf(hBuffer, "%02X ", replyBuffer[n]);
                        content += hBuffer;
                        n++;
                      }
                    }
                  } // devtype = 9

                  content += "  ";
                  content += descrOfIx(devx);
                  content += "</li>";
                } // devtype > 0
                devx++;
              }  // while
              content += "</ol>";
              content += "</form></html>";
            } // devIx == 0
            else
            {
              content = "{\"status\":\"waiting query 0x";
              char hBuffer[4];
              sprintf(hBuffer, "%02X", devIx);
              content += hBuffer;
              content += " \"}";
              statusCode = 200;
            }
          }
        uartSemaphor = 0;

        request->send(statusCode, "text/html", content);
        content = "";
// =============================================================================================
    });



 



// =============================================================================================
    a_server.on("/devicename", HTTP_GET, [] (AsyncWebServerRequest *request) 
    {
// =============================================================================================
  
        // denominazione manuale
        char device  = 0;
        char devtype = 0;
        char deviceX = 0;
        char nomeDevice[6];
        char temp[32];
        char *ch;
        WORD_VAL maxp;
        
        String AlexaDescr = "";
        String TypeDescr = "";
        String EndMsg = "";

        nomeDevice[0] = 0;
        temp[0] = 0;

        if (firstTime == 0) setFirst();  // DEVONO essere attivi @MX  e @l



        // -------------- manual update start -------------------

        if (request->hasArg("busid"))
        {
          String busid = request->arg("busid");
          if ((busid != "") 
          &&  (busid.length() > 1))
          {
            deviceX = ixOfDeviceNew(&busid[0]);
            if (deviceX > 0) 
            {
              if (request->hasArg("devname"))
              {
                String edesc = descrOfIx(deviceX);
                AlexaDescr = request->arg("devname");
                if (AlexaDescr != edesc)
                {
                  if (alexaParam == 'y' )
                  {
                    fauxmo.renameDevice(&edesc[0], &AlexaDescr[0]);
                  }
                  WriteDescrOfIx(AlexaDescr, deviceX);
                  EndMsg = "<li>UPDATED !!!</li>";
                }
              }
              if (request->hasArg("maxpos"))
              {
                String maxpos = request->arg("maxpos");
                maxp.Val = (int)strtoul(&maxpos[0], &ch, 10);
              }
              else
                maxp.Val = 0;
                
              if (request->hasArg("type"))
              {
                String stype = request->arg("type");
                char type = (char) stype.toInt();
                if (type == 0) type = 1;
                devtype = device_BUS_id[deviceX].deviceType;
                if (type != devtype)
                {
                  device_BUS_id[deviceX].deviceType = type;
                  devtype = type;
                  EndMsg = "<li>UPDATED !!!</li>";
                }
              }

              if (EndMsg != "")
              {
                  WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
                  EEPROM.commit();
              }
              AlexaDescr = "";
              if ((devtype == 9) || (devtype == 19))
              {
                requestBuffer[requestLen++] = PIC_REQ;
                requestBuffer[requestLen++] = 'U';
                requestBuffer[requestLen++] = '8';
                requestBuffer[requestLen++] = device_BUS_id[deviceX].address;     // device id
                requestBuffer[requestLen++] = devtype;    // device type
                requestBuffer[requestLen++] = maxp.byte.HB;    // max position H
                requestBuffer[requestLen++] = maxp.byte.LB;    // max position L
                immediateSend();
                devtype = 0;
                immediateReceive('k');
              } // devtype == 9 || 19
            } // ((deviceX > 0) && (devAddress < DEV_NR))
          } // (request->hasArg("busid"))
        } // (busid != "")
        
        
        // -------------- end of update -------------------
        
        
        deviceX++;
      //  if (device_BUS_id[deviceX].address == 0)
      //      deviceX = 1;
        devtype = device_BUS_id[deviceX].deviceType;
        
        if ((deviceX >= DEV_NR)  || (device_BUS_id[deviceX].addressW == 0)) devtype=0;

        if (device_BUS_id[deviceX].addressW > 0)
        {
          device = DeviceOfIx(deviceX, nomeDevice);
          AlexaDescr = descrOfIx(deviceX);
          if (devtype == 1)
            TypeDescr = "1 switch";
          else if (devtype == 2)
            TypeDescr = "2 ";
          else if (devtype == 3)
            TypeDescr = "3 dimmer";
          else if (devtype == 4)
            TypeDescr = "4 dimmer";
          else if (devtype == 8)
            TypeDescr = "8 cover";
          else if (devtype == 18)
            TypeDescr = "18 cover1";
          else if (devtype == 11)
            TypeDescr = "11 generic";
          else if (devtype == 14) // 0x0E)
            TypeDescr = "E alarm board";
          else if (devtype == 15) // 0x0F)
            TypeDescr = "F termo";
          else if (devtype == 9)
            TypeDescr = "9 coverpct";
          else if (devtype == 19)
            TypeDescr = "19 coverpct1";
          
          if ((devtype == 9) || (devtype == 19))
          {
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '6';
            requestBuffer[requestLen++] = device_BUS_id[deviceX].address;
            immediateSend();
            char m = immediateReceive('[');
            //    replyBuffer[1], replyBuffer[2]  -  maxposition H-L
            if (m > 8)
            {
              int c = replyBuffer[2] | replyBuffer[1] << 8;
              sprintf(temp, "%d", c);
            }
            else
              sprintf(temp, "nul");
          }
          else
            TypeDescr = String((char)devtype,10);
        }

        content = "<!DOCTYPE HTML>\r\n<html>Discovered device:";
        content += "<p>";
        content += "</p><form method='get' action='devicename'>";
        content += "<label>SCS address: </label><input name='busid' maxlength=2 value='";
        content += String(nomeDevice);
        content += "' style='width:40px'>\
            <label>  type: </label><input name='type' maxlength=10 value='";
        content += TypeDescr;
        content += "' style='width:80px'> ";

        content += "<label> max position (0,1 sec):</label><input name='maxpos' maxlength=6 value='";
        content += temp;
        content += "' style='width:45px'>";

        content += "<label> name:</label><input name='devname' maxlength=20 value='";
        content += AlexaDescr;
        content += "' style='width:200px'>";

        content += "<label> alexa ref.:</label><input name='alexaind' maxlength=3 value='";


      //  content += ...............    da fauxmo.addDevice

        char sAlex[4];
        if (device_BUS_id[deviceX].alexa_id)
        {
          sprintf(sAlex, "%d", device_BUS_id[deviceX].alexa_id);
          content += sAlex;
        }
        else
          content += "no";
          
        content += "' style='width:40px'>";

        content += "<input type='submit'>";
        content += EndMsg;

        if ((devtype <= 0) || (devtype > 31))
        {
          sprintf(temp, "<li>END OF %d discovered devices</li>", deviceX);
          content += temp;
          content += "</li>";
          device = 0;
          deviceX = 0;
          WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
          EEPROM.commit();
        }
        content += "</form> </html>";

        request->send(200, "text/html", content);
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
// =====================================================================================================
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
#ifdef LOWPOWER
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
#endif
WiFi.begin(esid.c_str(), epass.c_str());   
#ifdef LOWPOWER
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
#endif
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
      Serial1.write('@');   // set led lamps
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
        
        char devtype;
        char devx = 1;
        while ((devx < DEV_NR) && (device_BUS_id[devx].addressW))
        {
          devtype = device_BUS_id[devx].deviceType;
          if ((devtype > 0) && (devtype != 11) && (devtype < 18))  // W6 - in alexa censire solo indirizzi base
          {
            String edesc = descrOfIx(devx);
            if ((edesc != "") && (edesc[0] > ' '))
            {
              if (devtype == 9)
                id_fauxmo = fauxmo.addDevice(&edesc[0], 1);
              else
                id_fauxmo = fauxmo.addDevice(&edesc[0], 128);
                
              alexa_BUS_ix[id_fauxmo] = devx;  // index: device id alexa max DEV_NR devices    data: ID bus reale
              device_BUS_id[devx].alexa_id = id_fauxmo; 
            }
            devx++;
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
            char devtype = device_BUS_id[devix].deviceType;
//          char device = alexa_BUS_id[alexa_id];
            
#ifdef DEBUG
            Serial.printf("\r\n-       device " _MODO " %02X - type: %02X ", device_BUS_id[devix].address, devtype);
#endif

            // If your device state is changed by any other means (MQTT, physical button,...)
            // you can instruct the library to report the new state to Alexa on next request:
            // fauxmo.setState(ID_YELLOW, true, 255);

            {
              if (firstTime == 0) setFirst();  // DEVONO essere attivi @MX  e @l
              unsigned char command;
              int pct;
              char stato = fauxmo.getState(alexa_id);
              char device = device_BUS_id[devix].address;
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
                  serObuffer[bufNr][serOlen[bufNr]++] = device_BUS_id[devix].address;     // to   device address
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
      Serial1.write('@');   // set led lamps
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
// =====================================================================================================
// -----------------------------------------------------------------------------------------------------------------------------------------------
void setup() {

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

/*
// ------------- 's' : SPEED test  ------------------------------------
  if (serIniOption == 's')
  {
    int32_t time1;
//  char ixOfDevice(DEVADDR device)
//    unsigned long last = millis();
     
    DEVADDR dtest;
    dtest.linesector = 0x44;
    dtest.address = 0x55;

    char xa = 0;
    while (xa < DEV_NR)
    {
      device_BUS_id[xa].linesector = 11;
      device_BUS_id[xa].address = 22;
      device_BUS_id[xa].deviceType = 1;
      xa++;
    }
    // test search table speed with ixofdevices()
    time1 = asm_ccount();
    int nts = 0;
    int lups =0;
    char tst;
    while (nts < 1000)
    {
      tst = ixOfDevice((DEVADDR)dtest);
      nts++;
    }
    int32_t time2;
    lups = nts;
    time2 = asm_ccount();
//    Serial.printf("\r\n- table search (1000 devcs): %d mS\r\n", (millis() - last)); // 
    Serial.printf("\r\ns: - table search (%d devcs): %d tcks\r\n", lups, time2 - time1); //     
    Serial.printf(" - %d uS\r\n", (time2 - time1)/80); //     
    time2 = asm_ccount();
    Serial.printf("\r\n- table search (%d devcs): %d tcks\r\n", lups, time2 - time1); //     
    Serial.printf(" - %d uS\r\n", (time2 - time1)/80); //     
  }
*/


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
#ifdef FFS
/*  --------------------------------------------------------- test spiffs --------------------------------
#ifdef VERBOSE
  if (serIniOption == 'f')
  {
    Serial.println("f: test spiffs");
    if (SPIFFS.begin()) 
    {
      Serial.println("SPIFFS Active");
      Serial.println();

    } else 
    {
      Serial.println("Unable to activate SPIFFS");
    }

    String str = "";
    Dir dir = SPIFFS.openDir("/");   
    while (dir.next()) {
      str += dir.fileName();
      str += " / ";
      str += dir.fileSize();
      str += "\r\n";
    }
    Serial.print(str);

    String path = "/version.txt";
    Serial.println("handleFileRead: " + path);

    if (SPIFFS.exists(path)) 
    {
      File f = SPIFFS.open(path, "r");
      if (!f) {
        Serial.print("Unable To Open '");
        Serial.print(path);
        Serial.println("' for Reading");
        Serial.println();
      } else {
        String s;
        Serial.print("Contents of file '");
        Serial.print(path);
        Serial.println("'");
        Serial.println();
        while (f.position()<f.size())
        {
          s=f.readStringUntil('\n');
          s.trim();
          Serial.println(s);
        } 
        f.close();
      }
      Serial.println();
    }
    else 
      Serial.println("/version.txt not found");
    
    path = PICPATH;
    Serial.println("handleFileRead: " + path);

    if (SPIFFS.exists(path)) 
    {
      File f = SPIFFS.open(path, "r");
      if (!f) {
        Serial.print("Unable To Open '");
        Serial.print(path);
        Serial.println("' for Reading");
        Serial.println();
      } else {
        String s;
        Serial.print("Contents of file '");
        Serial.print(path);
        Serial.println("'");
        Serial.println();
        
        char buffer[20];
        s=f.readBytes(buffer, 16);
        Serial.printf("%02X %02X %02X %02X  %02X %02X %02X %02X", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
        
        f.close();
      }
      Serial.println();
    }
    else 
      Serial.println(PICPATH " not found");
        
    SPIFFS.end();
  }
#endif
*/  
#endif
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

  ReadStream((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN, 0);  // tipo=0 binary   1:ascii


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
  delay(50);            // wait 50ms

  Serial1.write("@b");   // clear SCSgate/KNXgate buffer
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



  if ((serIniOption != 'a') && (esid.length() > 1 ))
// =========================================================================================
//        CONNESSIONE  AL  ROUTER 
// =========================================================================================
  {
	  ConnectToRouter(esid, epass);
  }
  else
  {
    Serial1.write('@');   // set led lamps
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
    Serial1.write('@');   // set led lamps
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
        WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
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
// =====================================================================================================
// DEVADDR device_BUS_id[DEV_NR]; // pointer: eventuale id alexa - contenuto: device address reale (scs o knx) e tipo
// =====================================================================================================





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

//  delay(100);
//  return;
  

if (sm_picprog == PICPROG_FREE)
{
// ========================================== NORMAL RUN ====================================================

#ifdef MY_DOMOTIC
    if (flagRefreshDevices == 1)
    {
        events.send("Entità ricaricate - fai refresh pagina /sseha ", "log", millis());
//        reloadEntity(0);
        delay(5);
    }
#endif
#ifdef MY_TABELLA
    if (flagRefreshDevices == 1)
    {
        events.send("Entità ricaricate - fai refresh pagina /devices ", "log", millis());
        reloadDevices();
        delay(5);
    }
#endif
    flagRefreshDevices =  0;



      // ==================================== TCP server ====================================================
#ifdef USE_TCPSERVER
  if (tcpopen)
  {
   if (tcpserver.hasClient())
   {
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
      deviceX = ixOfDevice(&busid[0]);
      device = device_BUS_id[deviceX].address;
      devtype = device_BUS_id[deviceX].deviceType;
      
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
        deviceX = ixOfDeviceNew(&busid[0]);
        device = device_BUS_id[deviceX].address;

        if (deviceX)
        {
          String alexadescr = tcpJarg(tcpBuffer,(char *) "\"descr\"");
          alexadescr.toCharArray(AlexaDescr, 21);  

          WriteDescrOfIx(AlexaDescr, deviceX);
        
          String stype = tcpJarg(tcpBuffer,(char *) "\"type\"");
          devtype = (char) stype.toInt();
/*
//          if ((devtype == 8) && (domotic_options == 0x03))
//             devtype = 18;          
//          if ((devtype == 9) && (domotic_options == 0x03))
//             devtype = 19;          
*/
          device_BUS_id[deviceX].deviceType = devtype;
             
          if ((alexaParam == 'y' ) && (devtype < 18))		// w6 - alexa non ha bisogno dei types 18 19
          {
            String edesc = descrOfIx(deviceX);
            fauxmo.renameDevice(&edesc[0], &alexadescr[0]);
          }
          
          if ((devtype == 9) || (devtype == 19))			// w6 - aggiorna tapparelle pct su pic
          {
            String smaxpos = tcpJarg(tcpBuffer,(char *) "\"maxp\"");
            char *ch;
            maxp.Val = (int)strtoul(&smaxpos[0], &ch, 10);
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
        tapparellePercentuale = false;
      }  // cover == "false"
      else
      if (cover == "true")
      {
        tapparellePercentuale = true;
      }  // cover == "false"

      String devclear = tcpJarg(tcpBuffer,(char *) "\"devclear\"");
      if (devclear == "true")
      {
        for (int i = 0; i < DEV_NR; ++i)
        {
          EEPROM.write((int) i * E_ALEXA_DESC_LEN + E_ALEXA_DESC_DEVICE, 0);
  	      device_BUS_id[i].Val = 0;
        }
        WriteEEP((char) 0, (int) E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
      }  // devclear == "true"
      

      sprintf(tcpBuffer, "#ok");
      buflen = 0;
      while (tcpBuffer[buflen]) buflen++;
      tcpclient.write(tcpBuffer, buflen);
      
      WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
//    EEPROM.commit();
      tcpclient.flush();
    }	// #putdevice
    else
// ------------------------------------------------------------------------------------------------------
    if (memcmp(tcpBuffer, "#getdevall",10) == 0) // download devices
// ------------------------------------------------------------------------------------------------------
    {
      deviceX = 1;
      tapparellePercentuale = false;
      while (device_BUS_id[deviceX].addressW)
      {
        devtype = device_BUS_id[deviceX].deviceType;
        if (device_BUS_id[deviceX].addressW)
        {
          device = DeviceOfIx(deviceX, nomeDevice);
          String alexadescr = descrOfIx(deviceX);
          alexadescr.toCharArray(AlexaDescr, 21);  
          maxp.Val = 0;
          if ((devtype == 9) || (devtype == 19))
          {
            requestBuffer[requestLen++] = PIC_REQ;
            requestBuffer[requestLen++] = 'U';
            requestBuffer[requestLen++] = '6';
            requestBuffer[requestLen++] = device;
            immediateSend();
            char m = immediateReceive('[');
            tapparellePercentuale = true;
            
            if (m > 8)
            {
              maxp.Val = replyBuffer[2] | replyBuffer[1] << 8;
            }
            
          }  // devtype == 9
          sprintf(tcpBuffer, "{\"device\":\"%s\",\"type\":\"%d\",\"maxp\":\"%d\",\"descr\":\"%s\"}",nomeDevice,devtype,maxp.Val,AlexaDescr);
          buflen = 0;
          while (tcpBuffer[buflen]) buflen++;
          tcpclient.write(tcpBuffer, buflen);
          tcpclient.flush();
        } // devtype > 0
        deviceX++;
      } // while devicex < DEV_NR
     
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
//      deviceX = (char) busid.toInt();
        deviceX = ixOfDevice(&busid[0]);
      }
      else
      busid = tcpJarg(tcpBuffer,(char *) "\"afterdev\"");
      if (busid != "")
      {
//      deviceX = (char) busid.toInt();
        deviceX = ixOfDevice(&busid[0]);
        deviceX++;
      }
      else
      busid = tcpJarg(tcpBuffer,(char *) "\"devnum\"");
      if (busid != "")
      {
        deviceX = (char) busid.toInt();
      }

      devtype = device_BUS_id[deviceX].deviceType;
      if (device_BUS_id[deviceX].addressW)
      {
          device = DeviceOfIx(deviceX, nomeDevice);
          String alexadescr = descrOfIx(deviceX);
          alexadescr.toCharArray(AlexaDescr, 21);  
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
          sprintf(tcpBuffer, "{\"device\":\"%s\",\"type\":\"%d\",\"maxp\":\"%d\",\"descr\":\"%s\"}",nomeDevice,devtype,maxp.Val,AlexaDescr);
          buflen = 0;
          while (tcpBuffer[buflen]) buflen++;
          tcpclient.write(tcpBuffer, buflen);
          tcpclient.flush();
      } // devtype > 0
      else
      {
        sprintf(tcpBuffer, "#eof");
        buflen = 0;
        while (tcpBuffer[buflen]) buflen++;
        tcpclient.write(tcpBuffer, buflen);
        tcpclient.flush();
      }
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
  internal = 0;
  if (Serial1.available() )
  {
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
        delayMicroseconds(INNERWAIT);
      }
      delayMicroseconds(OUTERWAIT);
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
      String log;
      char logCh[256];
      int xr = 0;
      if (internal == 1)
      {
//        sprintf(logCh, "[%04d] SCS RX: {%02X %02X} -> ", (now/100), replyBuffer[xr++], replyBuffer[xr++]);
        sprintf(logCh, "SCS RX: {%02X %02X} -> ", replyBuffer[xr++], replyBuffer[xr++]);
        log = logCh;
        while (xr < replyLen)
        {
          sprintf(logCh, "%02X ", replyBuffer[xr++]);
          log += logCh;
        }
        events.send(log.c_str(), "log", millis());
      }
      else
      {
        log = "";
//        sprintf(logCh, "[%04d] -> ", (now/100));
//        log += logCh;
        if ( replyBuffer[1] == '\n')
          log = &replyBuffer[2];
        else
        if ( replyBuffer[0] == '\r')
          log = &replyBuffer[1];
        else
          log = replyBuffer;
        
//        log += replyBuffer;
        log.replace("\r", ""); // Rimuove il Carriage Return
        log.replace("\n", "\\n"); // Sostituisce il New Line con un a capo HTML
        events.send(log.c_str(), "log", millis());

        /*
        char newLine = 0;
        sprintf(logCh, "[%04d] --- RX: ", (now/100));
        log += logCh;

        while (xr < replyLen)
        {
          if (replyBuffer[xr++] == 0x0d)
          {
            newLine = 1;
            if (replyBuffer[xr] == 0x0a)
                xr++;
          }

          if (newLine == 1)
          {
            events.send(log.c_str(), "log", millis());
            sprintf(logCh, "[%04d] ------: ", (now/100));
            log = logCh;
          }
          newLine = 2;
          sprintf(logCh, "%c", replyBuffer[xr++]);
          log += logCh;
//          logCh[0] = replyBuffer[xr++];
//          logCh[1] = 0;
//          log += logCh;
        }

        if (newLine == 2)
        {
          events.send(log.c_str(), "log", millis());
        }
        */
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





  // --------------------- RISPOSTA HTTP di prova -------------------------------------------


  if ((httpResp == "W") && (httpCallback[0] != 0))
  {
    if ((replyBuffer[0] == 0xF5) && (replyBuffer[1] == 'y') && (replyLen == 6))   // solo se stringa=   [0xF5] [y] 32 00 12 01
    {
      content = "http://";
//      content += tcp_remote_ip.toString();
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
  } // httpResp == "W"
  // --------------------- FINE RISPOSTA HTTP -------------------------------------------






  // =========================================== M Q T T  PUBLISH=================================================

  // ------------[0xF3] [D] <index/device> <type> ----------------------- CENSIMENTO DEVICES -------------
  if ((replyLen == 4) && (devIx > 0) && (replyBuffer[0] == 0xF3) && (replyBuffer[1] == DEVICEREQUEST)) // started from handleMqttDevices
  {
    devIx   = replyBuffer[2];
    char device = replyBuffer[2];
    char devtype = replyBuffer[3];
#ifdef SCS_XX
  // ------------[0xF4] [D] <index> <device> <type> ----------------------- CENSIMENTO DEVICES -------------
  if ((replyLen == 5) && (devIx > 0) && (replyBuffer[0] == 0xF4) && (replyBuffer[1] == DEVICEREQUEST)) // started from handleMqttDevices
  {
    devIx   = replyBuffer[2];
    char device = replyBuffer[3];
    char devtype = replyBuffer[4];
#endif
    if ((devtype == 0xFF))  // || ((devCall > 1) && (devIx < devCall)))
    {
      WriteEEP((char*)&device_BUS_id[0], E_MQTT_TABDEVICES, (int) DEV_NR * E_MQTT_TABLEN);
      EEPROM.commit();
      if (firstTime == 0) setFirst();  // DEVONO essere attivi @MX  e §l
      devIx = 0;
      uartSemaphor = 1;
      requestBuffer[requestLen++] = PIC_REQ;
      requestBuffer[requestLen++] = 'U';
      requestBuffer[requestLen++] = '7'; // chiede di ripubblicare tutte le posizioni
      uartSemaphor = 0;
    }
    else if (devIx == 0)
    {
      device_BUS_id[devIx].Val = 0;
      devIx++;
      uartSemaphor = 1;
      requestBuffer[requestLen++] = PIC_REQ;
      requestBuffer[requestLen++] = DEVICEREQUEST;
      requestBuffer[requestLen++] = devIx;
      uartSemaphor = 0;
    }
    else
    {
      char dvx = ixOfDeviceNew(device);
      MQTTnewdevice(dvx, 0);
      devCtr++;
      device_BUS_id[dvx].deviceType = devtype;
      String dsc = descrOfIx(dvx);

      if ((dsc[0] == 0) || (dsc[0] == 0xFF) || (dsc == ""))
      {
        // descrizione standard per ALEXA 
        char AlexaDescr[E_ALEXA_DESC_LEN];
        sprintf(AlexaDescr, "dispositivo scs %02X", devIx);  // device
        WriteDescrOfIx(AlexaDescr, devIx);
      }

      devIx++;
      uartSemaphor = 1;
      requestBuffer[requestLen++] = PIC_REQ;
      requestBuffer[requestLen++] = DEVICEREQUEST;
      requestBuffer[requestLen++] = devIx;
      uartSemaphor = 0;
    }
  } //  replyLen == 4...   fine censimento

  // ----------------------------------------- FINE CENSIMENTO DEVICES -------------------------------------------------------
  else





  // ----------------------------------------- ALEXA STATO DEVICES ---------------------------------------------------------
    

  // ---------------------u-posizione tapparelle o dimmer %---------------------------------------------------------------------
  //    [F3] u <address> <position%>
  if ((replyLen == 4) && (replyBuffer[0] == 0xF3))    
  { // replyLen==4 && replyBuffer == 0xF3 'u'
    char device = replyBuffer[2];
    int pct = replyBuffer[3];
    char devx = ixOfDevice(device);
   
    
    if (replyBuffer[1] == 'u')    //'u': posizione tapparelle//
    {
      if (alexaParam == 'y')      //u//
      { // aggiornamento posizione coverpct fauxmo   
        char id_alexa = device_BUS_id[devx].alexa_id;
        if (id_alexa)
        {
          pct *= 255;
          pct /= 100;
          if (pct == 0)  pct = 1;
          if (pct > 255) pct = 255;
          fauxmo.setState(id_alexa, 0xFF, pct);
        }
      } // alexaParam == 'y'
    
#ifdef NO_ALEXA_MQTT
    else
#endif    

    // ----------------------------------------- ALEXA STATO DEVICES END----------------------------------------------------



    // ----------------uab-(address position)--- PUBBLICAZIONE STATO COVERPCT -------------------------------------------------------
      if (mqttopen == 3)    //u//
      { // pubblicazione posizione coverpct   
        char actionc[6];

        sprintf(actionc, "%03u", pct);     // position
        char nomeDevice[5];
        sprintf(nomeDevice, "%02X", device);  // device
        String topic = COVERPCT_STATE;
        topic += nomeDevice;
        const char* cTopic = topic.c_str();
        client.publish(cTopic, actionc, mqtt_persistence);
      } // mqttopen == 3
    }  // 'u'
    
    else
    
    
    if (replyBuffer[1] == 'm')    //'m': posizione dimmer//
    {
    // ----------------uab-(address position)--- PUBBLICAZIONE STATO DIMMER -------------------------------------------------------
      if (mqttopen == 3)    //u//
      { // pubblicazione posizione dimmer   [0xF3] [m] 32 00
        char actionc[6];

        sprintf(actionc, "%03u", pct);     // position
        char nomeDevice[5];
        sprintf(nomeDevice, "%02X", device);  // device
        String topic = BRIGHT_STATE;
        topic += nomeDevice;
        const char* cTopic = topic.c_str();
        client.publish(cTopic, actionc, mqtt_persistence);
      } // mqttopen == 3
    }    
    
    replyLen = 0; // per impedire pubblicazione UDP
  } // replyLen==4 - 5  && replyBuffer == 0xF3-F4
    
  else




  // --------SCS-----4abcd-(from-to-type-cmd)------- PUBBLICAZIONE STATO DEVICES -------------------------------------------------------
  // --------KNX-----4abcd-(from-to-cmd)------------ PUBBLICAZIONE STATO DEVICES -------------------------------------------------------

  if ((mqttopen == 3) && (replyBuffer[1] == 'y') && ((replyBuffer[0] == 0xF5) || (replyBuffer[0] == 0xF6)))
  { // START pubblicazione stato device        [0xF5] [y] 32 00 12 01
    char devtype;
    char action;
    String topic = "NOTOPIC";
    String payload = "";

    // ================================ C O M A N D I     S C S ===========================================

    // SCS intero   [7] A8 32 00 12 01 21 A3
    // SCS ridotto [0xF5] [y] 32 00 12 01
    //  termostato [0xF5] [y] B4 01 30 F4      @yb46530f4  SENSORE

    char devx = 0;
    char device;
    if ((replyBuffer[4] == 0x30) && ((replyBuffer[2] == 0xB4) || (replyBuffer[2] == 0xB5)))  // <-termostato----------------------------
    {
        device = replyBuffer[3];  
        devtype = 15;

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
        }
        devx = 1;
        while ((devx < DEV_NR) && (device_BUS_id[devx].addressW))
        {
          devtype = device_BUS_id[devx].deviceType;
          if (((typeAll == 1) && (devtype == 3))  // dimmer
            || (typeAll == devtype))  // switch - cover
          {
            sprintf(nomeDevice, "%02X", device_BUS_id[devx].address);  // to
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
          }
          devx++;
        }
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
        devtype = 1;
      else if ((action == 3) || (action == 4))      // dimmer
        devtype = 0; // sono solo richieste di +/- , poi arrivera' l'intensita'
      else if ((action == 0x08) || (action == 0x09) || (action == 0x0A))   // cover
        devtype = 8;
      else if ((action & 0x0F) == 0x0D) // da 0x1D a 0x9D
      {
        devtype = 3;
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
        if (devtype == 1)
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
        if ((devtype == 3) ||(devtype == 4))
        {
          char actionc[4];
          sprintf(actionc, "%02u", action);
          payload = String(actionc);
          topic = BRIGHT_STATE;
          topic += nomeDevice;
        }
        else
        // ----------------------------------------- STATO COVER --------------------------------------------
        if ((devtype == 8) || (devtype == 18))
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
          devx = ixOfDevice(device);
          
          if ((devx) && (device_BUS_id[devx].deviceType == 11))  
          {	          // device generic censito
            char nomeDevice[3];
            sprintf(nomeDevice, "%02X", device);  // to
            topic = GENERIC_TO;
            topic += nomeDevice;
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
  // ----------------------------------------------------------------------------------------------------
    } // END pubblicazione stato devices

  // ================================ F I N E   C O M A N D I     S C S ===========================================










#ifdef MY_DOMOTIC
  // ================================== P U B B L I C A Z I O N E    S S E =========================================


  // ---------------------u-posizione tapparelle o dimmer %---------------------------------------------------------------------
  //    [F3] u <address> <position%>
//  if ((eventity.count() != 0) && (replyLen == 4) && (replyBuffer[0] == 0xF3))    
  if ((eventity.count() != 0) && (replyBuffer[0] == 0xF3))    
  { // replyLen==4 && replyBuffer == 0xF3 'u'
    char device = replyBuffer[2];
    int pct = replyBuffer[3];
   
    if (replyBuffer[1] == 'u')    //'u': posizione tapparelle//
    {
    // ----------------uab-(address position)--- PUBBLICAZIONE STATO COVERPCT -------------------------------------------------------
      int i = cercaEntitaScs(device);

      if (i>=0) {
//        char actionc[6];
//        sprintf(actionc, "%3u", pct);     // position
//        elencoEntita[i].valore = atoi(actionc);
        elencoEntita[i].valore = pct;
        eventity.send(String(elencoEntita[i].valore) + "|" + elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
        String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + payload;
        events.send(rispostaLog.c_str(), "log", millis());
#endif
      }
    }  // 'u'
  }  

/*
    else
    
    if (replyBuffer[1] == 'm')    //'m': posizione dimmer//
    {
    // ----------------uab-(address position)--- PUBBLICAZIONE STATO DIMMER -------------------------------------------------------
      if (mqttopen == 3)    //u//
      { // pubblicazione posizione dimmer   [0xF3] [m] 32 00
        char actionc[6];

        sprintf(actionc, "%03u", pct);     // position
        char nomeDevice[5];
        sprintf(nomeDevice, "%02X", device);  // device
        String topic = BRIGHT_STATE;
        topic += nomeDevice;
        const char* cTopic = topic.c_str();
        client.publish(cTopic, actionc, mqtt_persistence);
      } // mqttopen == 3
    }    
    
    replyLen = 0; // per impedire pubblicazione UDP
  } // replyLen==4 - 5  && replyBuffer == 0xF3-F4

else

*/



  // --------SCS-----4abcd-(from-to-type-cmd)------- PUBBLICAZIONE STATO DEVICES -------------------------------------------------------
  // --------KNX-----4abcd-(from-to-cmd)------------ PUBBLICAZIONE STATO DEVICES -------------------------------------------------------

  if ((eventity.count() != 0) && (replyBuffer[1] == 'y') && ((replyBuffer[0] == 0xF5) || (replyBuffer[0] == 0xF6)))
  { // START pubblicazione stato device        [0xF5] [y] 32 00 12 01
    char devtype;
    char action;
    char actionc[16];
    String payload = "";
    char device;
    int i;
    int valore = -1;
    // ================================ C O M A N D I     S C S ===========================================

    // SCS intero   [7] A8 32 00 12 01 21 A3
    // SCS ridotto [0xF5] [y] 32 00 12 01
    //  termostato [0xF5] [y] B4 01 30 F4

    if ((replyBuffer[4] == 0x30) && ((replyBuffer[2] == 0xB4) || (replyBuffer[2] == 0xB5)))  // <-termostato----------------------------
    {
      device = replyBuffer[3];  
      devtype = 15;

      if (replyBuffer[2] == 0xB5)
        sprintf(actionc, "%03u", (255 + replyBuffer[5])); 
      else
        sprintf(actionc, "%03u", replyBuffer[5]); 
      actionc[4]=0;
      actionc[3]= actionc[2];
      actionc[2]=',';
      payload = String(actionc);
      
      i = cercaEntitaScs(device);

      if (i>=0) {
        elencoEntita[i].stato = payload;
        eventity.send(elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
    String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + payload;
    events.send(rispostaLog.c_str(), "log", millis());
#endif
      }
    }
    else

    // enum TipoEntita { ENTI_INTERRUTTORE, ENTI_SENSORE, ENTI_SLIDER, ENTI_TAPPARELLA };

    if (replyBuffer[4] == 0x12)  // <-comando----------------------------
    {
      char typeAll = 0;
      action = replyBuffer[5];
      switch (action)
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
          if ((action & 0x0F) == 0x0D) // da 0x1D a 0x9D
          {
            typeAll = 3;
            action >>= 4;
            action *= 10;	// percentuale 10-90
            sprintf(actionc, "%02u", action);
            valore = action;
          }
          break;
      }



      if (replyBuffer[2] == 0xB1)   // <------------ indirizzato a TUTTI i devices
      {
        for (int i = 0; i < numeroEntita; i++) 
        {
          if (((typeAll == 1)  // on/off
                && (elencoEntita[i].tipo == ENTI_INTERRUTTORE))
          ||  ((typeAll == 8)  // up/down/stop
                && ((elencoEntita[i].tipo == ENTI_TAPPARELLA) || (elencoEntita[i].tipo == ENTI_TAPPARELLA_PCT))))
          {                
            elencoEntita[i].stato = payload;
            eventity.send(elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
  String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + payload;
  events.send(rispostaLog.c_str(), "log", millis());
#endif
          }
          else
          if ((typeAll == 1)  // on/off
                && (elencoEntita[i].tipo == ENTI_SLIDER))
          {                
            elencoEntita[i].stato = payload;
            eventity.send(String(elencoEntita[i].stato.c_str()) + "|" + elencoEntita[i].valore, elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
  String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + payload;
  events.send(rispostaLog.c_str(), "log", millis());
#endif
          }
        }
      }                            
      else
      {                           // <------------ indirizzato a UN SOLO device
        if (replyBuffer[2] < 0xB0)
          device = replyBuffer[2];  // to
        else
          device = replyBuffer[3];  // from
        i = cercaEntitaScs(device); 
        if (i>=0) 
        {
          if (elencoEntita[i].tipo == ENTI_TAPPARELLA_PCT)
          {
            elencoEntita[i].stato = payload;
            eventity.send(String(elencoEntita[i].valore) + "|" + elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
  String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + payload + "|" + String(elencoEntita[i].valore);
  events.send(rispostaLog.c_str(), "log", millis());
#endif
          }
          else
          if (elencoEntita[i].tipo == ENTI_SLIDER)
          {
            if (valore >= 0)
                elencoEntita[i].valore = valore;
            if (payload != "")
                elencoEntita[i].stato = payload;
            eventity.send(String(elencoEntita[i].stato.c_str()) + "|" + elencoEntita[i].valore, elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
  String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + String(elencoEntita[i].valore) + "|" + elencoEntita[i].stato;
  events.send(rispostaLog.c_str(), "log", millis());
#endif
          }
          else
          {
            elencoEntita[i].stato = payload;
            eventity.send(elencoEntita[i].stato.c_str(), elencoEntita[i].id.c_str(), millis());
#ifdef DEBUG_EVENTITY
  String rispostaLog = "[" + String(millis()/100) + "s] [CMD] id: " + elencoEntita[i].id + " : " + payload;
  events.send(rispostaLog.c_str(), "log", millis());
#endif
          }
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
    } // END pubblicazione stato devices

  // ================================ F I N E   C O M A N D I     S C S ===========================================




#endif
  // ===============================================================================================================



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




      // ===================================CICLO SCRITTURA BUFFER UART=================================================
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


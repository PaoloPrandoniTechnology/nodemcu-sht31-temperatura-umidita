# DOCUMENTAZIONE DETTAGLIATA PROGETTO FIRMWARE NODEMCU
## Sensore Temperatura & Umidità — SHT31-D I²C

---

## 1. PANORAMICA GENERALE

### 1.1 Descrizione Progetto
Firmware per **Wemos D1 Mini (ESP8266)** dedicato al monitoraggio ambientale ad alta precisione.
Il progetto acquisisce dati da un sensore **SHT31-D** (temperatura + umidità relativa) tramite bus I²C,
calcola grandezze derivate (punto di rugiada, indice di calore, umidità assoluta), e mette a
disposizione i dati attraverso molteplici canali di output.

### 1.2 Stack Tecnologico
| Componente | Versione / Dettagli |
|---|---|
| **Piattaforma** | PlatformIO `espressif8266` / Arduino Core |
| **MCU Target** | ESP8266 (Wemos D1 Mini) |
| **Sensore** | SHT31-D I²C @ 0x44 (ADDR pin LOW) |
| **Sensore Libreria** | Wire (I²C standard Arduino, NO wrapper terze parti) |
| **Librerie Esterne** | `WebSockets@^2.4.1`, `PubSubClient@^2.8` |
| **Build flags** | `MQTT_MAX_PACKET_SIZE=1024`, `MQTT_KEEPALIVE=30` |
| **Upload Speed** | 921600 baud |
| **Monitor Seriale** | 115200 baud |

### 1.3 Canali di Output Disponibili
1. **Seriale CSV** @ 115200 baud (1 Hz)
2. **Pagina Web HTTP** su porta 80 (dashboard live + configurazione)
3. **WebSocket Server** su porta 81 (broadcast binario ogni 10 ms)
4. **MQTT** (Home Assistant Discovery auto-configurante)

### 1.4 Grandezze Fisiche Acquisite / Calcolate
| Grandezza | Formula / Fonte | Range tipico |
|---|---|---|
| Temperatura (T) | SHT31-D raw → `-45 + 175 × raw/65535` | -40°C ~ +125°C |
| Umidità Relativa (RH) | SHT31-D raw → `100 × raw/65535` | 0% ~ 100% |
| Punto di Rugiada (DP) | Magnus-Tetens: `b·α/(a−α)` con `α = a·T/(b+T) + ln(RH/100)` | - |
| Indice di Calore (HI) | Rothfusz NOAA (solo per T ≥ 26.7°C) | - |
| Umidità Assoluta (AH) | Buck vapour pressure → `216.7·e/(T+273.15)` in g/m³ | - |
| Stato Heater SHT31 | Bit di stato | ON/OFF |
| Bitmask diagnostica | 6-bit: T_OK, RH_OK, CRC_OK, I2C_OK, HEAT_ON, ALERT | 0x00 ~ 0x3F |

---

## 2. HARDWARE & WIRING

### 2.1 Assegnazione Pin
```
SHT31-D  →  Wemos D1 Mini (ESP8266)
───────────────────────────────────────
VCC      →  3.3V  (MAX 3.3V! NON 5V)
GND      →  GND
SDA      →  D2   (GPIO4)   Attenzione: Wire.begin(SDA, SCL) vuole SDA PRIMA!
SCL      →  D1   (GPIO5)
ADDR     →  GND  (indirizzo I²C = 0x44)
ALERT    →  (non collegato, letto via registro status)
RESET    →  (non collegato, soft reset via comando 0x30A2)
```

### 2.2 Parametri Sensore
- **Modalità**: Single-Shot HIGH Accuracy No-Stretch Clock
- **Tempo conversione**: ~16 ms (HIGH accuracy)
- **Frequenza bus I²C**: 400 kHz (Fast Mode)
- **Campionamento effettivo**: 1 Hz
- **Heater integrato**: Gestito automaticamente via isteresi RH

---

## 3. CRASH LOGGER & DIAGNOSTICA RESET

### 3.1 Architettura RTC User Memory
Il firmware implementa un **Crash Logger avanzato** basato sulla **RTC User Memory** (512 byte dell'ESP8266 che sopravvivono a SW reset, WDT reset, Exception Panic, Deep Sleep wake — ma NON a power-cycle o reset pin).

**Layout RTC Words (48-127)**:
```
Words 48-63 :  HEARTBEAT PRE-CRASH (aggiornato ogni 500ms)
                 Magic + CRC + uptime + heap + stack + WDT feeds
               + WiFi/MQTT cached status + supervisore counters
               + sensor state bitmask
               → Cattura lo stato ESATTAMENTE prima di un crash
               anche se l'handler panic non riesce a scriverlo.

Words 64-89 :  CRASH CONTEXT (26 words, CRC32-validato)
  64 : Magic_1 = 0xCA55CADE
  65 : Magic_2 = ~Magic_1 (complemento)
  66 : VERSIONE = 1
  67 : Reset Reason enum (RstReasonCode)
  68 : uptime_ms al crash
  69 : free heap bytes
  70 : min heap storico
  71 : heap fragmentation %
  72 : stack watermark usato (byte)
  73 : WDT feed count
  74 : MQTT conn (bit16) + WiFi.status (bits 0-7)  [CACHED — NO API]
  75 : sensor_status_bits (bits16-23) + ds18_conv (bit0)  [CACHED]
  76 : epoch sec (NTP se disponibile, altrimenti uptime)
  77-89 : reserved (zero)
  90 : CRC32 di words 64..89

Words 91-127 :  Riservati per futuri SDK panic dump
```

### 3.2 Reset Reason Enum Normalizzato
Valori dello stato `RstReasonCode`:
| Valore | Enum | Causa |
|---|---|---|
| 0 | `RST_POWERON` | Power-On Reset (accensione) |
| 1 | `RST_EXTERNAL_PIN` | Pin RST tirato LOW esternamente |
| 2 | `RST_HW_WATCHDOG` | Hardware Watchdog timeout |
| 3 | `RST_SW_WATCHDOG` | Software Watchdog timeout |
| 4 | `RST_EXCEPTION_PANIC` | Exception CPU (LoadStoreError, ecc.) |
| 5 | `RST_SOFTWARE_RESTART` | `ESP.restart()` da software |
| 6 | `RST_DEEPSLEEP_WAKE` | Wake da Deep Sleep |
| 7 | `RST_BROWNOUT_DETECT` | Brown-out detector (tensione troppo bassa) |
| 99 | `RST_UNKNOWN` | Sconosciuto |

### 3.3 Handler Panic Custom — Regole CRITICHE
La funzione `capturePanicContext()` esegue durante l'exception panic con interrupt DISABILITATI.
Per evitare crash ricorsivi (LoadStoreErrorBus che impedisce la scrittura RTC), **NESSUNA chiamata a oggetti C++ o API lwIP è consentita**:
- ❌ `g_mqtt.connected()` → dereferenzia interno PubSubClient, potrebbe essere NULL
- ❌ `WiFi.status()` → lwIP stack potenzialmente inconsistente
- ❌ `String()` → chiama `malloc()` → heap potenzialmente corrotto
- ✅ Solo variabili globali `g_diag_lastMqttWifiWord` e `g_diag_lastSensorWord` (cached dall'heartbeat)
- ✅ `ESP.getFreeHeap()` → contatori UMM interni, safe
- ✅ `millis()` → counter tick, safe

### 3.4 Stack Watermark Monitoring
L'ESP8266 ha uno stack del task principale di ~4096 byte. Il firmware calcola:
- **Baseline SP** (catturato all'inizio di `setup()`, stack quasi vuoto)
- **Lowest SP** (SP minimo mai rilevato = massimo utilizzo)
- **Stack available** = 3968 − used (con margine safety 128B per interrupt frame)
- **Allerta** stack overflow → warning seriale ogni 5s se disponibili < 256 byte

---

## 4. GESTIONE WIFI: CONNESSIONE ROBUSTA & FALLBACK

### 4.1 Credenziali Default
```
SSID STA   : "YOUR_WIFI_SSID"
Password   : "YOUR_WIFI_PASSWORD"
SSID AP    : "TempHumid-Node"
Pass AP    : "CHANGE_ME_AP_PASS"
IP AP      : 192.168.4.1
```

### 4.2 Strategia Connessione STA (11 Tentativi)
Funzione `netStartSTAWithRetries()`:
```
TENTATIVI TOTALI = 1 + WIFI_STA_RETRY_COUNT (10) = 11

PER OGNI tentativo i=1..11:
  ├─ i>1 ? → ESEGUI HARD RESET WIFI + pausa 2000ms tra tentativi
  │    ├─ WiFi.disconnect(true)
  │    ├─ WiFi.mode(WIFI_OFF)
  │    ├─ Attendi WIFI_STA_HARD_RESET_OFF_MS = 500ms (WDT feed attivo!)
  │    ├─ WiFi.persistent(false)
  │    └─ WiFi.setAutoReconnect(false)
  │
  └─ netTrySTAOnce(i, true):
       ├─ WiFi.mode(WIFI_STA)
       ├─ WiFi.begin(SSID, PASS)
       └─ ATTENDI fino a WL_CONNECTED oppure WIFI_STA_PER_TRY_TIMEOUT_MS = 15000ms
            (250ms per tick, massimo 60 tick = 15s)

Dopo TUTTI i 11 fallimenti:
  → Attiva FALLBACK MODE (AP+STA contemporaneamente)
  → STA continuerà a ritentare OGNI 30s in background via netFallbackPollRetry()
```

### 4.3 Modalità Fallback AP+STA
Se tutti gli 11 tentativi falliscono:
```
g_net_fallback_mode = true

Ogni WIFI_FALLBACK_POLL_PERIOD_MS = 30000ms (30 s):
  ├─ Se già connesso: disabilita fallback, torna a STA normale
  ├─ Altrimenti:
  │    ├─ WiFi.mode(WIFI_OFF) se necessario
  │    ├─ WiFi.mode(WIFI_AP_STA)
  │    ├─ WiFi.softAP(AP_SSID, AP_PASSWORD)
  │    └─ WiFi.begin(WIFI_SSID, WIFI_PASSWORD)
```
L'utente può sempre connettersi all'AP `192.168.4.1` per accedere alla pagina di configurazione.

### 4.4 Riconnessione Automatica STA
Quando WiFi è attivo in modalità normale (non fallback):
- `WiFi.setAutoReconnect(true)` → SDK ESP8266 gestisce riconnessioni micro-interruzioni
- Transizione `WL_CONNECTED → WL_DISCONNECTED` viene rilevata in `loop()` e attiva re-try

---

## 5. SISTEMA DI CONFIGURAZIONE EEPROM

### 5.1 Layout EEPROM (4096 byte)
Struttura `Config_t` (offset 0, validata da CRC32):
```
Magic    (4B) : 0x55AA55AA
Version  (1B) : 2 (attuale, con migrazione automatica v1 → v2)
Reserved (3B) : padding

device_name        (40B) : Nome dispositivo (es. "Sensori Cantina")
sensor_sht31_name  (40B) : Nome sensore SHT31 (es. "Ambiente_SHT31")
reserved_sensor_2  (40B) : futura espansione
reserved_sensor_3  (40B) : futura espansione
reserved_sensor_4  (40B) : futura espansione

mqtt_server   (64B) : IP/hostname broker MQTT
mqtt_port     (2B)  : porta TCP (default 1883)
mqtt_user     (48B) : username
mqtt_password (64B) : password
mqtt_device_id(64B) : identificativo univoco MQTT (per topic)

CRC32        (4B) : CRC di TUTTO il blocco escluso se stesso
```
*Totale ~460B, assolutamente entro 4096B.*

### 5.2 Flusso Caricamento Configurazione
```
configLoadFromEeprom():
  1. Legge sizeof(Config) byte da EEPROM[0..N]
  2. Verifica magic + version:
       ├─ MAGIC OK + VERSION 2 + CRC OK → usa direttamente
       ├─ MAGIC OK + VERSION 1          → MIGRAZIONE SILENTE v1 → v2
       │    ├─ copia campi esistenti in temp
       │    ├─ applica defaults per nuovi campi
       │    ├─ calcola CRC v2
       │    └─ SALVA SUBITO in EEPROM
       └─ ALTRO → CARICA DEFAULTS + salva in EEPROM
  3. Sempre esegue sanitizeConfig(): validazione UTF-8 printable + range
```

### 5.3 Sanitizzazione Config
- Nulli o non-UTF8-printable `< 0x20 || 0x7F` → revert a default
- Porta `0` o `>65535` → default 1883
- Garantisce **ZERO crash** dovuti a EEPROM corrotta

### 5.4 Valori Default
| Campo | Valore |
|---|---|
| Device Name | `"Sensori Temp/Umid"` |
| Sensore SHT31 | `"Ambiente_SHT31"` |
| MQTT Server | `"192.168.1.100"` |
| MQTT Port | `1883` |
| MQTT User | `"mqtt_user"` |
| MQTT Password | `"mqtt_password"` |
| MQTT Device ID | `"sensori_temperatura_umidita"` |

---

## 6. PAGINA WEB DI CONFIGURAZIONE (HTTP PORTA 80)

### 6.1 Endpoint HTTP
| Metodo | URL | Descrizione |
|---|---|---|
| GET | `/` | Dashboard + UI configurazione (SPA, INDEX_HTML in PROGMEM) |
| GET | `/api/labels` | JSON: nomi dispositivo + sensore per UI |
| GET | `/api/config` | JSON: configurazione completa corrente |
| POST/PUT | `/api/config` | Salva nuova configurazione in EEPROM |
| POST | `/api/restart` | Riavvia ESP8266 (conferma JS lato client) |

### 6.2 Dashboard Live (Tab "Dashboard")
- **Connessione WebSocket** verso `ws://<IP>:81/`
- Dati ricevuti in **formato binario (ArrayBuffer)** di 28 byte:
  ```
  Offset 0-3   : float32 T(°C)     little-endian
  Offset 4-7   : float32 RH(%)
  Offset 8-11  : float32 DewPoint(°C)
  Offset 12-15 : float32 HeatIndex(°C)
  Offset 16-19 : float32 AbsHumidity(g/m³)
  Offset 20-23 : float32 status_bits (uint8 promosso)
  Offset 24-27 : float32 heater_on (1.0=ON 0.0=OFF)
  ```
  **Fallback**: anche parsing stringa CSV (`T,RH,DP,HI,AH,status,heat`) è supportato lato client.

- **Smoothing lato client**: EMA (Exponential Moving Average) con α=0.15 per evitare jitter visuale
- **Severity semantiche colorate**:
  - Verde OK / Rosso NC (dati non validi) / Giallo warning
- **Controlli aggiuntivi**:
  - Pacchetti/Hz ricevuti (FPS stimato)
  - Età ultimo dato in ms
  - Timestamp locale PC
  - Bitmask diagnostica HEX + binary
  - Stato sensore formattato `[T:OK RH:OK CRC:OK I2C:OK HEAT]`

### 6.3 Tab Configurazione
**Sezione 1 — Identificativo Dispositivo**:
- Nome scheda/ambiente (max 39 char)

**Sezione 2 — Nome Sensore**:
- Etichetta SHT31-D I²C (max 39 char)

**Sezione 3 — Connettività MQTT**:
- Server/IP broker (max 63)
- Porta (1-65535)
- Username (max 47)
- Password (max 63, tipo `password`)
- Device ID (regex: `[a-zA-Z0-9_-]+`, max 63)
- **Anteprima topic MQTT** live durante editing

**Sezione 4 — Azioni**:
| Pulsante | Comportamento |
|---|---|
| 💾 Salva | POST `/api/config` → valida server-side + EEPROM commit |
| ↺ Ricarica | GET `/api/config` → ripristina valori correnti |
| ⚠ Ripristina Default | Conferma JS → POST con valori hardcoded |
| 🔄 Riavvia dispositivo | POST `/api/restart` + reload pagina dopo 4.5s |

**Comportamento salvataggio**:
- Se MQTT cambiato (server/porta/user/pass/devid): `g_mqtt_config_changed = true` → riconnessione immediata al prossimo loop
- Se solo nomi cambiati: `g_mqtt_discovery_sent = false` → nuovo discovery MQTT a prossima connessione
- I nomi cambiati sono visibili alla UI dopo `loadAndApplyAll()` (300ms dopo salvataggio)

### 6.4 Meccanismi Anti-Zombie WebSocket (Lato Client)
- **Silence Detector**: 30s senza dati → `ws.close()` + riconnessione
- **Exponential Backoff**: 1.5s → 3s → 6s → ... → 15s max, con jitter casuale ±300ms
- **Consec Fail Counter**: per logging/debug

---

## 7. DRIVER SHT31-D I²C

### 7.1 Inizializzazione `sht31_init()`
```
Wire.begin(SDA=D2, SCL=D1)   → Attenzione ordine! ESP8266 vuole (SDA,SCL)
Wire.setClock(400000)        → 400 kHz Fast Mode
Prova ACK @ 0x44
↓
CMD_SOFT_RESET (0x30A2) → delay 2ms
CMD_CLEAR_STATUS (0x3041)
↓
READ_STATUS_REGISTER (0xF32D):
  ├─ se heater era ON da stato precedente → forza CMD_HEATER_OFF
↓
Prova misura singola SINGLE_HIGH_NOSTRETCH (0x2400):
  ├─ attesa 16ms
  ├─ requestFrom(6 byte)
  ├─ CRC check per T[0:2] e RH[3:5] (polinomio CRC-8 0x31)
  ├─ calcolo raw → fisico
  └─ set STATUS bits se tutto OK
```

### 7.2 Polling State Machine `sht31_poll()` — 1 Hz
```
STATO: SHT31_POLL_IDLE
  ↓ if (now - lastSample >= 1000ms)
  ├─ Invia CMD 0x2400 (Single-Shot HIGH No-Stretch)
  │    ├─ FALLITO: azzera T, RH, CRC, I2C bits → dati = NAN
  │    └─ OK: g_status_bits |= I2C_OK
  ├─ convPending = true, convStart_ms = now
  └─ → SHT31_POLL_WAIT

STATO: SHT31_POLL_WAIT
  ↓ if (now - convStart >= 16ms)
  ├─ Wire.requestFrom(6 byte)
  │    └─ FALLITO (≠6 byte): azzera tutti i bits → NAN → torna IDLE
  ├─ Verifica CRC8 su buf[0:1] vs buf[2], buf[3:4] vs buf[5]
  ├─ Conversione T: raw uint16 →  -45 + 175*(raw/65535) °C
  ├─ Conversione RH: raw uint16 →  100*(raw/65535) %
  │    └─ Range check: T in [-45, 130] °C, RH in [0, 100] %
  ├─ Grandezze derivate (solo se T e RH validi):
  │    ├─ DewPoint = Magnus-Tetens
  │    ├─ HeatIndex = Rothfusz NOAA (T ≥ 26.7°C, altrimenti passthrough T)
  │    └─ AbsHumidity = Buck vapour pressure formula
  ├─ Heater auto Hysteresis (solo se RH valido):
  │    ├─ RH > 95% per ≥ 30s → ACCENDI HEATER (0x306D)
  │    ├─ RH < 90%           → SPEGNI HEATER (0x3066)
  │    └─ RH ≤ 95% non acceso → reset timer
  ├─ lastSample_ms = now
  └─ → SHT31_POLL_IDLE
```

### 7.3 Bitmask Stato Sensore `g_status_bits`
```
Bit 0 : STATUS_SHT31_T_OK     (0x01) → Temperatura valida
Bit 1 : STATUS_SHT31_RH_OK    (0x02) → Umidità valida
Bit 2 : STATUS_SHT31_CRC_OK   (0x04) → CRC lettura OK
Bit 3 : STATUS_SHT31_I2C_OK   (0x08) → Ultimo comando I²C ACK ricevuto
Bit 4 : STATUS_SHT31_HEAT_ON  (0x10) → Heater attualmente attivo
Bit 5 : STATUS_SHT31_ALERT    (0x20) → Alert flag da status register
```

---

## 8. MQTT & INTEGRAZIONE HOME ASSISTANT

### 8.1 Parametri di Rete MQTT
| Parametro | Valore | Spiegazione |
|---|---|---|
| KeepAlive | 10 s | Invia ping ogni 10s; timeout dopo 5s socket |
| Socket Timeout | 5 s | lwIP read/write timeout |
| Buffer Size | 2048 B | PubSubClient MQTT buffer |
| Max Packet Size | 1024 B | Build flag (payload massimo singolo) |
| Pub Period | 1000 ms (1 Hz) | Cadenza publish dati ambientali |
| Online Refresh | 30 s | Riscrivi availability "online" retain QoS1 |
| **TCP KeepAlive** | **idle=5s, interval=3s, count=3** | **Rileva socket zombie entro 5 + 3×3 = 14s** |

⚠️ **NOTA CRITICA `WiFiClient::keepAlive()`**: Non chiamare PRIMA della connessione TCP!
Il metodo dereferenzia `_client` interno SENZA null-check. Prima della `connect()`, `_client=nullptr`
→ **LoadStoreErrorBus panic (exccause=28, excvaddr=0x0)**. Deve essere chiamato **SUBITO DOPO**
`g_mqtt.connect()` OK, quando `ClientContext` è già creato.

### 8.2 Topic MQTT — Struttura
```
BASE = "homeassistant/sensor/<mqtt_device_id>/"

Availability:          BASE + "availability"           retain, QoS1
                          "online"  /  "offline" (Last Will)

Sensori (state):       BASE + "<sensor_id>/state"      retain, QoS1
  sht31_temp_c           → T °C (3 decimali)
  sht31_hum_rh           → RH % (2 decimali)
  sht31_dew_c            → Punto Rugiada °C
  sht31_heat_index_c     → Indice Calore °C
  sht31_abs_hum_gm3      → Umidità Assoluta g/m³
  sht31_heater           → 1 = ON, 0 = OFF
  sensor_status_bitmask  → "0x00" ~ "0x3F"
  sensor_status_hexbin   → "0x3F  b111111"
  sensor_status_list     → "Ambiente_SHT31 [T:OK RH:OK ...]"
  supv_*                 → Entità supervisore
  diag_*                 → Entità diagnostica (heap, stack, RSSI, ...)

Discovery HA Config:  BASE + "<sensor_id>/config"      retain, QoS1
                          JSON conforme schema MQTT Discovery
```

### 8.3 MQTT Home Assistant Discovery
Alla prima connessione MQTT riuscita, pubblica un `config` JSON per OGNI entità:
```json
{
  "name": "Ambiente_SHT31 Temperatura",
  "unique_id": "sensori_temperatura_umidita_sht31_temp_c",
  "state_topic": "homeassistant/sensor/.../sht31_temp_c/state",
  "availability_topic": "homeassistant/sensor/.../availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "device_class": "temperature",
  "unit_of_measurement": "°C",
  "icon": "mdi:thermometer",
  "state_class": "measurement",
  "device": {
    "identifiers": ["sensori_temperatura_umidita"],
    "name": "Sensori Temp/Umid",
    "manufacturer": "Paolo Prandoni",
    "model": "NodeMCU 1xSHT31-D I2C (High Accuracy)",
    "sw_version": "<ESP8266 Core Version>"
  }
}
```
Home Assistant creerà **AUTOMATICAMENTE** 22+ entità senza configurazione manuale.

### 8.4 Strategia Riconnessione MQTT Robusta
Funzione `mqttEnsureConnected()` chiamata **ogni loop**:

**CASO A — Siamo già connessi**:
Esegue 4 Sanity Check prima di dichiarare "tutto ok":
1. `g_wifiClient.connected()` → socket TCP realmente viva? (se false → ZOMBIE)
2. `delta_lastGoodPubMs >= SUPV_PUB_STALL_MS (10s)` anche se connected() → ZOMBIE CATCH-ALL
3. `g_mqtt_pub_consec_fail >= 3` publish fallite consecutive CON connected()=true → ZOMBIE
4. Refresh availability "online" ogni 30s (se fallisce → ZOMBIE)

Se ZOMBIE rilevato:
- Pubblica availability "offline"
- `g_mqtt.disconnect()`
- Azzera tutti i counter
- Torna a stato "non connesso"

**CASO B — Non siamo connessi**:
```
WiFi.status ≠ WL_CONNECTED → return false (attendi WiFi)

Calcolo BACKOFF:
  • Se stavamo connessi PRIMA (transizione persa): backoff = 0  (SUBITO)
  • Altrimenti:
      base = 250ms << (failures-1)  [clamp a 8 iterazioni = 32s max]
      + jitter 0-127ms casuale
      clamp a MQTT_CONNECT_BACKOFF_MAX_MS = 2000ms
      Se failures ≥ 5: minimo 1000ms

Se non ancora scaduto backoff: return false

TENTATIVO connect():
  clientId = "<mqtt_device_id>_<ESP_CHIP_ID_HEX>" (univoco per evitare sessioni duplicate)
  LWT Topic = availability topic  QoS1  retain  payload="offline"
  User + Pass auth MQTT

SUCCESSO:
  ├─ g_mqtt_was_connected = true
  ├─ Azzera tutti i failures counters
  ├─ TCP KeepAlive(5s, 3s, 3)  ← QUI, NON PRIMA!
  ├─ Publish "online" su availability
  └─ SE discovery non ancora inviato → mqttPublishAllDiscovery()

FALLIMENTO:
  ├─ g_mqtt_consec_failures++
  ├─ Stampa motivo (TIMEOUT, BAD_CREDENTIALS, UNAUTHORIZED, ...)
  └─ Backoff incrementato al prossimo tentativo
```

### 8.5 Transizioni Loop
Il `loop()` rileva autonomamente la transizione `CONNESSO → DISCONNESSO` DOPO `g_mqtt.loop()`:
```cpp
bool nowConn = g_mqtt.connected();
if (g_mqtt_was_connected && !nowConn) {
    // FORZA RICONNESSIONE IMMEDIATA: backoff = 0
    g_mqtt_was_connected = false;
    g_mqtt_lastAttemptMs = 0;
    g_mqtt_consec_failures = 0;
    // → prossima mqttEnsureConnected() tenta SUBITO
}
```

---

## 9. SUPERVISORE MQTT 4 LIVELLI + HARDWARE WATCHDOG

### 9.1 Architettura di Sicurezza
Il Supervisore monitora lo stato delle comunicazioni e applica una **escalation di recovery** se lo stall persiste.
In parallelo, l'**Hardware Watchdog** (ESP8266 System on Chip) armato a **8000 ms (8s)** protegge da loop infiniti.

### 9.2 Trigger Stall
`supervisorIsStalled()` ritorna TRUE se UNA delle seguenti si verifica (con WiFi WL_CONNECTED):
- **Trigger A (Conn Stall)**: MQTT `!connected()` AND `now - lastGoodConn ≥ 10s`
- **Trigger B (Pub Stall)**: MQTT `connected()` AND `now - lastGoodPub ≥ 10s`
- **Trigger C (Fallback Online)**: MQTT `connected()` AND `now - lastOnlineRefresh ≥ 60s` (2× il periodo)

⚠️ **Comportamento con WiFi assente**: Se `WiFi.status() ≠ WL_CONNECTED`, il rilevamento è SOSPESO
ma **il timer L4 (120s dal trip iniziale) CONTINUA a decorrere**. Questo evita che brevi glitch WiFi
ripetuti a < 120s l'uno dall'altro impediscano PER SEMPRE l'escalation a L4 (Reset Hardware).

### 9.3 Escalation 4 Livelli
```
Tempo 0  : TRIP (stall rilevato) → g_supvTripped = true, g_supvTripMs = now
           → IDLE
               ↓ subito
           → L1: g_supvL1Count++
              forza bypass backoff con g_mqtt_lastAttemptMs = 0
              chiama mqttEnsureConnected()
               ↓ aspetta SUPV_L1_COOLDOWN_MS = 3000ms
               ↓ ripeti L1 ogni 2500ms in attesa cooldown
           → L2: SUPV_L2_COOLDOWN_MS = 5000ms dopo L1
              g_supvL2Count++
              ├─ Sincrono: publica "offline" → disconnect()
              ├─ Azzera discovery
              ├─ → L2S_WAIT_COOLDOWN state machine per 250ms
              └─ → mqttEnsureConnected()

           → L3: SUPV_L3_COOLDOWN_MS = 8000ms dopo L2
              g_supvL3Count++
              ├─ Publish "offline" → MQTT disconnect()
              ├─ WiFi.disconnect(true) → WiFi.mode(WIFI_OFF)
              ├─ Attendi SUPV_L3_OFF_MS = 1200ms  (power-cycle radio PHY)
              ├─ WiFi.mode(WIFI_STA) → WiFi.begin(...) → WiFi.mode(WIFI_AP_STA)
              ├─ softAP(AP_SSID, AP_PASS)
              ├─ Attendi SUPV_L3_POST_MS = 100ms
              └─ Dopo cooldown 8s → TORNA A L1 (ciclo continuo)

           → L4 HARDWARE RESET (DEADLINE ASSOLUTA):
              Dopo SUPV_L4_RESET_MS = 120000ms (2 MIN) dal PRIMO TRIP iniziale:
              ├─ SENZA ECCEZIONI, anche se L2/L3/L1 stanno ancora provando
              ├─ anche se c'è stato un glitch WiFi nel mezzo
              ├─ Serial.flush() → delay(400) → ESP.restart()
              └─ g_supvResetCount++ (perso via power-cycle, usato finché RTC vive)
```

### 9.4 Clear Trip (Recupero)
Il trip viene cancellato solo se:
```
g_mqtt.connected()
AND now - lastGoodConn < 3000ms  (connessione stabile almeno 3s)
AND now - lastGoodPub  < 3000ms  (pubblicazioni funzionanti almeno 3s)
```

### 9.5 Watchdog Hardware
```cpp
ESP.wdtEnable(SUPV_WDT_HW_MS = 8000);
```
Il WDT deve essere "nutrito" con `ESP.wdtFeed()` (chiamato anche tramite `supervisorWatchdogFeed()`)
almeno OGNI 8000 ms. TUTTI i loop lunghi e le operazioni potenzialmente bloccanti contengono
watchdog feed **in linea**:
- Scrittura EEPROM (ogni 64 byte)
- Hard reset WiFi (ogni 50ms di delay)
- Attesa connessione STA (ogni 250ms)
- Attesa conversione SHT31 (ogni 1ms)
- Lettura body HTTP POST (ogni 32 tick)
- Tutte le chiamate alle sottosistemi in `loop()`

---

## 10. OUTPUT SERIALE CSV

### 10.1 Formato
Stampato a **1 Hz** (stesso rate SHT31):
```
ts_ms,SHT31_T_C,SHT31_RH_PCT,SHT31_DEW_C,SHT31_HI_C,SHT31_AH_GM3,status_hex,heater,status_list
12345,24.123,55.67,14.82,24.12,12.345,0x3F,0,Ambiente_SHT31 [T:OK RH:OK CRC:OK HEATER]
```
Valori non validi → `NaN`.

---

## 11. WEBSOCKET SERVER PORTA 81

### 11.1 Configurazione
```
Porta           : 81
Modalità        : Server (broadcast a tutti i client connessi)
Frequenza       : SAMPLE_PERIOD_MS = 10ms (100 Hz)
Formato payload : BINARIO 28 byte = 7 × float32 little-endian
Heartbeat WS    : ping 15s, pong timeout 6s, chiudi dopo 2 ping falliti
                  → Elimina zombie NAT/router che droppano TCP silenziosamente
```

### 11.2 Callback & Eventi
```cpp
g_ws.enableHeartbeat(15000, 6000, 2);  // SDK WebSockets gestisce automaticamente
```

---

## 12. SETUP() FLOW DI INIZIALIZZAZIONE

```cpp
Serial.begin(115200)
  ↓
diagInitAndPrintResetReport() → Crash Logger, decode ESP rst_info,
  │                              stampa report completo (reset reason,
  │                              ultimo crash valido, ultimo heartbeat,
  │                              heap, stack, chip/flash info)
  ↓
g_diag_bootCount++
  ↓
configLoadFromEeprom(&g_config)  → migrazione, sanitization, fallback defaults
  ↓
sht31_init()  → I²C begin, soft reset, clear status, test measure
  ↓
netStartSTAorAP()  → 11 tentativi STA → fallback AP+STA → softAP start
  ↓
g_http.on("/", ...)
g_http.on("/api/labels", ...)
g_http.on("/api/config", ...)
g_http.on("/api/restart", ...)
g_http.onNotFound(...)
g_http.begin()
  ↓
g_ws.begin()
g_ws.enableHeartbeat(15000, 6000, 2)
  ↓
g_wifiClient.setNoDelay(true)  // Nagle OFF per bassa latenza
g_mqtt.setServer/setKeepAlive/setSocketTimeout/setBufferSize
  (keepAlive TCP NON chiamato qui — crash potenziale!)
  ↓
supervisorInit()  → timer L1/L2/L3/L4 + WDT HW ARMATO a 8s
  ↓
FINE setup()
```

---

## 13. LOOP() FLOW ESECUZIONE

```
Ogni iterazione loop() ~ SAMPLE_PERIOD_MS = 10ms (non bloccante, tutti i subsystem
sono state-machine asincrone. Nessun delay() nel loop, solo millis()/differenze.)

  ┌──────────────────────────────────────────────────────────────┐
  │ 1. crashHeartbeatSave(now)   → RTC words 48..63 ogni 500ms   │
  │ 2. DIAG Stack overflow?      → warning seriale ogni 5s       │
  │ 3. DIAG Heap critico?        → warning seriale ogni 5s       │
  │    (free<4096 OR maxBlock<2048)                               │
  │                                                              │
  │ 4. WDT FEED                                                  │
  │ 5. g_http.handleClient()                                     │
  │    WDT FEED                                                  │
  │ 6. g_ws.loop()                                               │
  │    g_mqtt.loop()                                             │
  │    WDT FEED                                                  │
  │ 7. netFallbackPollRetry()   → ogni 30s se in fallback mode  │
  │    WDT FEED                                                  │
  │                                                              │
  │ 8. RILEVAZIONE TRANSIZIONE MQTT CONN → DISCONN              │
  │    → forza backoff=0 per riconnessione IMMEDIATA             │
  │                                                              │
  │ 9. mqttEnsureConnected()                                     │
  │    WDT FEED                                                  │
  │ 10. sht31_poll()           → 1 Hz state machine             │
  │     WDT FEED                                                 │
  │ 11. serialPrintCsv()      → 1 Hz output seriale             │
  │     WDT FEED                                                 │
  │ 12. wsBroadcast()        → 100 Hz broadcast binario         │
  │     WDT FEED                                                 │
  │ 13. mqttPublishAllData()  → 1 Hz se connesso                │
  │     WDT FEED                                                 │
  │ 14. supervisorPoll()     → L1/L2/L3/L4 watchdog + recovery  │
  │     WDT FEED                                                 │
  └──────────────────────────────────────────────────────────────┘
```

---

## 14. ELENCO COSTANTI PRINCIPALI (RIFERIMENTO RAPIDO)

### 14.1 SHT31
| Define | Valore | Note |
|---|---|---|
| `SHT31_I2C_ADDR` | 0x44 | ADDR pin LOW |
| `SHT31_I2C_FREQ` | 400 kHz | Fast Mode |
| `SHT31_SAMPLE_INTERVAL_MS` | 1000 | 1 Hz |
| `SHT31_TCONV_HIGH_MS` | 16 | HIGH accuracy |
| `SHT31_HEATER_ON_RH_PCT` | 95% | Soglia accensione heater |
| `SHT31_HEATER_OFF_RH_PCT` | 90% | Isteresi spegnimento |
| `SHT31_HEATER_MIN_ON_MS` | 30000 | Minimo acceso prima di accendere |

### 14.2 WiFi
| Define | Valore |
|---|---|
| `WIFI_STA_RETRY_COUNT` | 10 (totale 11 con il primo) |
| `WIFI_STA_PER_TRY_TIMEOUT_MS` | 15000 |
| `WIFI_STA_INTER_RETRY_DELAY_MS` | 2000 |
| `WIFI_STA_HARD_RESET_OFF_MS` | 500 |
| `WIFI_FALLBACK_POLL_PERIOD_MS` | 30000 |

### 14.3 MQTT
| Define | Valore |
|---|---|
| `MQTT_PUB_PERIOD_MS` | 1000 |
| `MQTT_CONNECT_BACKOFF_MIN_MS` | 250 |
| `MQTT_CONNECT_BACKOFF_MAX_MS` | 2000 |
| `MQTT_KEEPALIVE_S` | 10 |
| `MQTT_SOCKET_TIMEOUT_S` | 5 |
| `MQTT_CLIENT_BUFFER_SIZE` | 2048 |
| `MQTT_ONLINE_REFRESH_MS` | 30000 |
| `TCP_KEEPALIVE_IDLE_SEC` | 5 |
| `TCP_KEEPALIVE_INTERVAL_SEC` | 3 |
| `TCP_KEEPALIVE_COUNT` | 3 |

### 14.4 Supervisore / WDT
| Define | Valore |
|---|---|
| `SUPV_WDT_HW_MS` | 8000 |
| `SUPV_CONN_STALL_MS` | 10000 |
| `SUPV_PUB_STALL_MS` | 10000 |
| `SUPV_L1_COOLDOWN_MS` | 3000 |
| `SUPV_L2_COOLDOWN_MS` | 5000 |
| `SUPV_L3_COOLDOWN_MS` | 8000 |
| `SUPV_L4_RESET_MS` | 120000 |

---

## 15. RIFERIMENTI FILE DI PROGETTO

| File | Scopo |
|---|---|
| [platformio.ini](platformio.ini) | Configurazione build, board, dipendenze, build flags |
| [src/main.cpp](src/main.cpp) | Firmware completo (unico file sorgente ~3700 righe) |
| [.pio/build/d1_mini/firmware.bin](.pio/build/d1_mini/firmware.bin) | Ultimo firmware compilato (flash diretto) |
| [ha_dashboard_stanze_v2.yaml](ha_dashboard_stanze_v2.yaml) | Dashboard Home Assistant |

---

## 16. NOTE FINALI DI SICUREZZA E BEST PRACTICE

1. **Power SHT31-D**: Alimentare SEMPRE a 3.3V. Il sensore **non tollera 5V** su VCC.
2. **Pull-up I²C**: SDA e SCL necessitano di pull-up a 3.3V (4.7kΩ ciascuno). Il modulo SHT31 breakout
   spesso li integra, ma verificarlo.
3. **Heater SHT31**: Non abusare del riscaldatore integrato. Può alterare la temperatura letta
   di alcuni gradi. L'isteresi attuale (95% ON / 90% OFF, min 30s) è conservativa per condensa
   in ambienti molto umidi.
4. **Watchdog Feed**: Non rimuovere MAI i `wdtFeed()` da loop lunghi. Il WDT a 8s armato
   causerebbe reset hardware continui.
5. **EEPROM Write Limite**: 4096 byte, ~100,000 cicli scrittura. Salvataggi solo su richiesta
   utente, nessuna scrittura periodica.
6. **MQTT Credenziali**: Inserite hardcoded come default, ma sovrascrivibili via pagina web.
   Il salvataggio è in EEPROM (non crittografato — sicurezza fisica device).
7. **Last Will MQTT**: Se la connessione cade improvvisamente, il broker MQTT pubblica AUTOMATICAMENTE
   `"offline"` sul topic availability dopo 1.5× KeepAlive = 15s.
8. **Crash Context RTC**: Non sopravvive a power-cycle. Per diagnosi persistente servirebbe un file system SPIFFS,
   ma aumenterebbe la complessità e i tempi di write.
9. **Buffer Overflow MQTT**: Se aumentare `MQTT_CLIENT_BUFFER_SIZE > 2048`, aggiornare anche
   `-D MQTT_MAX_PACKET_SIZE` in platformio.ini (sono due distinte!).

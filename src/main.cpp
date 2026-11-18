#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#include <Updater.h>
#include <user_interface.h>
#include <ets_sys.h>

// ============================================================================
// CRASH LOGGER - Diagnostica Reset Continui (Fase 1)
// ============================================================================
// Usa la RTC USER MEMORY (512 byte max) che SOPRAVVIVE a:
//   - Software Reset (ESP.restart / WDT SW)
//   - Hardware WDT reset
//   - Exception Panic reset
//   - Deep Sleep wake
// NON sopravvive a:
//   - Power cycle (PowerOn Reset)
//   - Reset pin RST pulled LOW (External reset)
//
// Layout:  words 64..127 (ultimi 256 byte della RTC user memory)
//   Word 64 : MAGIC_1  (0xCA55CADE)
//   Word 65 : MAGIC_2  complement checked
//   Word 66 : VERSION  (1)
//   Word 67 : last_crash_reset_reason_enum
//   Word 68 : last_crash_uptime_ms_u32
//   Word 69 : last_crash_heap_free_u32
//   Word 70 : last_crash_heap_min_u32
//   Word 71 : last_crash_heap_frag_pct_u32
//   Word 72 : last_crash_stack_wm_bytes_u32
//   Word 73 : last_crash_wdt_feed_count_u32
//   Word 74 : last_crash_mqtt_conn (1=yes 0=no) + wifi_st (low 8)
//   Word 75 : last_crash_ds18_conv_pend + status_bits
//   Word 76 : last_crash_timestamp_sec (epoch if NTP available else uptime)
//   Words 77-89 : reserved (zero)
//   Word 90 : CRC32  (words 64..89)
//   Words 91-127: Reserved for future SDK panic postmortem dump copy
//
// Words 48..63 : Pre-crash heartbeat (aggiornato ogni 500ms nel loop)
//   Se avviene un crash, l'ultimo heartbeat valido indica lo stato STATO
//   del sistema subito prima del crash, anche se l'handler eccezione
//   non riusciamo a salvarne uno nuovo.
// ============================================================================
#define CRASHLOG_MAGIC_1          0xCA55CADEu
#define CRASHLOG_MAGIC_2          (~0xCA55CADEu)
#define CRASHLOG_VERSION          1u
#define CRASHLOG_RTC_MAGIC        64u
#define CRASHLOG_RTC_VERSION      66u
#define CRASHLOG_RTC_RSTREASON    67u
#define CRASHLOG_RTC_UPTIME       68u
#define CRASHLOG_RTC_HEAP_FREE    69u
#define CRASHLOG_RTC_HEAP_MIN     70u
#define CRASHLOG_RTC_HEAP_FRAG    71u
#define CRASHLOG_RTC_STACK_WM     72u
#define CRASHLOG_RTC_WDT_FEED     73u
#define CRASHLOG_RTC_MQTT_WIFI    74u
#define CRASHLOG_RTC_SENSORS      75u
#define CRASHLOG_RTC_EPOCHSEC     76u
#define CRASHLOG_RTC_CRC32        90u
#define CRASHLOG_RTC_WORDS_START  64u
#define CRASHLOG_RTC_WORDS_PAYLD  26u  // words 64..89 inclusive
#define HEARTBEAT_RTC_START       48u
#define HEARTBEAT_RTC_WORDS       16u

// Reset reason enum (normalizzato ESP8266)
typedef enum {
    RST_POWERON = 0,
    RST_EXTERNAL_PIN,
    RST_HW_WATCHDOG,
    RST_SW_WATCHDOG,
    RST_EXCEPTION_PANIC,
    RST_SOFTWARE_RESTART,
    RST_DEEPSLEEP_WAKE,
    RST_BROWNOUT_DETECT,
    RST_UNKNOWN = 99
} RstReasonCode;

typedef struct {
    uint32_t      magic1;
    uint32_t      magic2;
    uint32_t      version;
    RstReasonCode rstReason;
    uint32_t      uptimeMs;
    uint32_t      heapFree;
    uint32_t      heapMin;
    uint32_t      heapFragPct;
    uint32_t      stackWmBytes;
    uint32_t      wdtFeedCount;
    uint32_t      mqttConnWifiWord;
    uint32_t      sensorStateWord;
    uint32_t      epochSec;
    uint32_t      crc32;
} CrashContext;

static_assert(sizeof(CrashContext) <= (128 - 64) * 4, "CrashContext overflow RTC budget");

// ---- Telemetria runtime condivisa tra loop e heartbeat ----
static uint32_t g_diag_minHeapEver = 0xFFFFFFFFu;
static uint32_t g_diag_stackWmBytes = 0xFFFFFFFFu;
static uint32_t g_diag_wdtFeedCount = 0;
static uint32_t g_diag_lastHeartbeatMs = 0;
static RstReasonCode g_rstReasonCached = RST_UNKNOWN;
static uint8_t       g_rstExccauseCached = 0;
static uint32_t      g_rstExccauseCached32 = 0;
static uint32_t      g_rstEpc1Cached = 0;
static uint32_t      g_rstEpc2Cached = 0;
static uint32_t      g_rstEpc3Cached = 0;
static uint32_t      g_rstExcvaddrCached = 0;
static uint32_t      g_diag_bootCount = 0;    // salvato in EEPROM? No: è il boot count da ultimo power cycle

// ---- Stati WiFi/MQTT "cached" raw (aggiornati in heartbeat) ----
//  Per leggere lo stato in capturePanicContext DURANTE l'eccezione SENZA
//  dover chiamare g_mqtt.connected() / WiFi.status(): quelle API colpiscono
//  oggetti C++ e lwIP interni e durante l'exception panic possono causare
//  LoadStoreErrorBus ricorsivo (NULL/inconsistente) → il primo crash non
//  viene mai salvato e la prossima boot vede RTC invalido.
static uint32_t g_diag_lastMqttWifiWord = 0; // bit16 = mqttConnected, bits0-7 = WiFi.status()
static uint32_t g_diag_lastSensorWord = 0;   // bits16-23 = status_bits, bit0 = ds18_conv_pending

// ---- Stack watermark (ESP8266 cont_stack task principale: ~4096 byte, conservativo 3840) ----
//
//  Nota sul core ESP8266 3.1.2: il campo `g_pcont->stack_end` non ha semantica
//  affidabile cross-versione (in alcuni core punta all'END basso, in altri all'INIZIO alto
//  della regione stack). Confrontare SP attuale con `stack_end` produce quindi spesso
//  0 anche quando stack e' OK → falsi allarmi stack_available_now=0 e
//  stack_used_wm=0 all'avvio (report ricevuto).
//
//  SOLUZIONE ROBUSTA: usiamo solo la baseline SP (catturata in diagStackCanaryInit
//  all'inizio di setup, quando stack e' quasi vuoto) vs SP attuale (cresce verso il
//  BASSO su Xtensa). I valori sono "used from baseline" e "available" calcolato su
//  dimensione stimata conservativa.
#define TASK_STACK_SIZE_ESTIMATE_BYTES 3968u   // < 4KB, safety margine 128 byte per interrupt frame

static uint32_t g_diag_stackBaselineSp = 0;   // SP alto inizio setup (stack vuoto)
static uint32_t g_diag_stackLowestSp   = 0xFFFFFFFFu;  // SP piu' basso mai visto (max usato)
static uint32_t g_diag_stackMaxUsed     = 0;

static size_t stackUsedBytes(void)
{
    register uint32_t sp asm("a1");
    if (g_diag_stackBaselineSp == 0u) return 0;
    if (sp >= g_diag_stackBaselineSp) return 0;
    return (size_t)(g_diag_stackBaselineSp - sp);
}

static size_t stackAvailableBytes(void)
{
    size_t used = stackUsedBytes();
    if (used >= TASK_STACK_SIZE_ESTIMATE_BYTES) return 0;
    return (size_t)(TASK_STACK_SIZE_ESTIMATE_BYTES - used);
}

static uint32_t stackWatermarkBytes(void)
{
    register uint32_t sp asm("a1");
    if (g_diag_stackBaselineSp == 0u) {
        g_diag_stackBaselineSp = sp;
    }
    if (sp < g_diag_stackBaselineSp) {
        if (sp < g_diag_stackLowestSp) {
            g_diag_stackLowestSp = sp;
            uint32_t used = g_diag_stackBaselineSp - sp;
            if (used > g_diag_stackMaxUsed) g_diag_stackMaxUsed = used;
        }
    }
    return g_diag_stackMaxUsed;
}

static void diagStackCanaryInit(void)
{
    register uint32_t sp asm("a1");
    g_diag_stackBaselineSp = sp;
    g_diag_stackLowestSp   = sp;
    g_diag_stackMaxUsed    = 0u;
}

static bool diagStackOverflowDetected(void)
{
    size_t used = stackUsedBytes();
    size_t av   = (used >= TASK_STACK_SIZE_ESTIMATE_BYTES) ? 0u : (TASK_STACK_SIZE_ESTIMATE_BYTES - used);
    if (av < 256u) return true;
    return false;
}

// ---- RTC user memory read/write wrapper ----
static bool crashRtcReadWords(uint32_t idx, uint32_t* dst, uint16_t count)
{
    return ESP.rtcUserMemoryRead(idx, dst, count);
}
static bool crashRtcWriteWords(uint32_t idx, const uint32_t* src, uint16_t count)
{
    return ESP.rtcUserMemoryWrite(idx, const_cast<uint32_t*>(src), count);
}

// ---- CRC32 per validazione RTC ----
static uint32_t crashCrc32(const uint32_t* words, uint16_t nWords)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint16_t w = 0; w < nWords; w++)
    {
        uint32_t v = words[w];
        for (uint8_t b = 0; b < 4; b++)
        {
            uint8_t byte = (uint8_t)(v & 0xFFu);
            crc ^= byte;
            for (uint8_t k = 0; k < 8; k++)
            {
                uint32_t mask = -(crc & 1u);
                crc = (crc >> 1) ^ (0xEDB88320u & mask);
            }
            v >>= 8;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

// ---- Decodifica reset reason ----
static RstReasonCode decodeEspResetReason(const char* reasonString,
                                          const rst_info* ri)
{
    // Prima: usa rst_info struct dal SDK (piu' preciso)
    if (ri != nullptr)
    {
        switch (ri->reason)
        {
            case REASON_DEFAULT_RST:      return RST_POWERON;
            case REASON_WDT_RST:          return RST_HW_WATCHDOG;
            case REASON_EXCEPTION_RST:    return RST_EXCEPTION_PANIC;
            case REASON_SOFT_WDT_RST:     return RST_SW_WATCHDOG;
            case REASON_SOFT_RESTART:     return RST_SOFTWARE_RESTART;
            case REASON_DEEP_SLEEP_AWAKE: return RST_DEEPSLEEP_WAKE;
            case REASON_EXT_SYS_RST:      return RST_EXTERNAL_PIN;
        }
    }
    // Fallback stringa Arduino
    if (reasonString == nullptr) return RST_UNKNOWN;
    String s(reasonString);
    if      (s.startsWith("Power on"))                  return RST_POWERON;
    else if (s.indexOf("External") >= 0)                 return RST_EXTERNAL_PIN;
    else if (s.indexOf("Hardware Watchdog") >= 0)        return RST_HW_WATCHDOG;
    else if (s.indexOf("Software Watchdog") >= 0)        return RST_SW_WATCHDOG;
    else if (s.indexOf("Exception") >= 0)                return RST_EXCEPTION_PANIC;
    else if (s.indexOf("Software restart") >= 0)         return RST_SOFTWARE_RESTART;
    else if (s.indexOf("Deep-Sleep") >= 0)               return RST_DEEPSLEEP_WAKE;
    else if (s.indexOf("Brownout") >= 0 || s.indexOf("BOD") >= 0) return RST_BROWNOUT_DETECT;
    return RST_UNKNOWN;
}

static const char* rstReasonStr(RstReasonCode c)
{
    switch (c)
    {
        case RST_POWERON:          return "POWERON_RESET";
        case RST_EXTERNAL_PIN:     return "EXT_PIN_RESET";
        case RST_HW_WATCHDOG:      return "HW_WATCHDOG_RESET";
        case RST_SW_WATCHDOG:      return "SW_WATCHDOG_RESET";
        case RST_EXCEPTION_PANIC:  return "EXCEPTION_PANIC_RESET";
        case RST_SOFTWARE_RESTART: return "SOFTWARE_RESTART";
        case RST_DEEPSLEEP_WAKE:   return "DEEPSLEEP_WAKE";
        case RST_BROWNOUT_DETECT:  return "BROWNOUT_BOD_RESET";
        default:                   return "UNKNOWN_RESET";
    }
}

// ---- Salva crash context o heartbeat ----
static void crashSaveContext(const CrashContext* ctx)
{
    uint32_t buf[CRASHLOG_RTC_WORDS_PAYLD];
    memset(buf, 0, sizeof(buf));
    buf[0] = ctx->magic1;
    buf[1] = ctx->magic2;
    buf[2] = ctx->version;
    buf[3] = (uint32_t)ctx->rstReason;
    buf[4] = ctx->uptimeMs;
    buf[5] = ctx->heapFree;
    buf[6] = ctx->heapMin;
    buf[7] = ctx->heapFragPct;
    buf[8] = ctx->stackWmBytes;
    buf[9] = ctx->wdtFeedCount;
    buf[10] = ctx->mqttConnWifiWord;
    buf[11] = ctx->sensorStateWord;
    buf[12] = ctx->epochSec;
    // words 13..25 reserved = 0
    uint32_t crc = crashCrc32(buf, CRASHLOG_RTC_WORDS_PAYLD - 1);  // CRC esclusa da se stessa
    buf[25] = crc;
    crashRtcWriteWords(CRASHLOG_RTC_WORDS_START, buf, CRASHLOG_RTC_WORDS_PAYLD);
    (void)crc;
}

static bool crashLoadContext(CrashContext* out)
{
    uint32_t buf[CRASHLOG_RTC_WORDS_PAYLD];
    memset(out, 0, sizeof(*out));
    if (!crashRtcReadWords(CRASHLOG_RTC_WORDS_START, buf, CRASHLOG_RTC_WORDS_PAYLD)) return false;
    if (buf[0] != CRASHLOG_MAGIC_1 || buf[1] != CRASHLOG_MAGIC_2) return false;
    uint32_t crc = crashCrc32(buf, CRASHLOG_RTC_WORDS_PAYLD - 1);
    if (crc != buf[25]) return false;
    out->magic1        = buf[0];
    out->magic2        = buf[1];
    out->version       = buf[2];
    out->rstReason     = (RstReasonCode)buf[3];
    out->uptimeMs      = buf[4];
    out->heapFree      = buf[5];
    out->heapMin       = buf[6];
    out->heapFragPct   = buf[7];
    out->stackWmBytes  = buf[8];
    out->wdtFeedCount  = buf[9];
    out->mqttConnWifiWord = buf[10];
    out->sensorStateWord = buf[11];
    out->epochSec      = buf[12];
    out->crc32         = buf[25];
    return true;
}

static void crashClearContext(void)
{
    uint32_t zero[CRASHLOG_RTC_WORDS_PAYLD];
    memset(zero, 0, sizeof(zero));
    crashRtcWriteWords(CRASHLOG_RTC_WORDS_START, zero, CRASHLOG_RTC_WORDS_PAYLD);
}

// ---- Heartbeat pre-crash (ogni ~500ms) ----
// IMPLEMENTAZIONE POSTICIPATA dopo dichiarazione globali statici (vedi ~riga 1580)
static void crashHeartbeatSave(uint32_t nowMs);
static bool crashHeartbeatLoad(uint32_t out[HEARTBEAT_RTC_WORDS])
{
    if (!crashRtcReadWords(HEARTBEAT_RTC_START, out, HEARTBEAT_RTC_WORDS)) return false;
    if (out[0] != 0x42484541u || out[1] != 0xA55A0001u) return false;
    uint32_t crc = crashCrc32(out, HEARTBEAT_RTC_WORDS - 1);
    return (crc == out[15]);
}

// ---- Panic handler custom (sostituisce handler di default Arduino) ----
extern "C" {
    void custom_crash_callback(struct rst_info* rstInfo, uint32_t stackStart, uint32_t stackEnd);
    void custom_pre_sleep_callback(uint8_t wakeup_opt, uint32_t sleep_ms);
}
static CrashContext g_panicCtx;
static void capturePanicContext(struct rst_info* ri);   // implementation postponed

// ============================================================================
// Versioning firmware
// ============================================================================
#define FW_VERSION_STR        "1.0.0"
#define FW_BUILD_DATE         __DATE__ " " __TIME__

// ============================================================================
// OTA sicurezza costanti
// ============================================================================
#define OTA_ESP_MAGIC_BYTE         0xE9u
#define OTA_MAX_SKETCH_BYTES       1000000UL
#define OTA_WDT_FEED_CHUNK_BYTES   4096UL
#define OTA_SOFT_TIMEOUT_MS        60000UL
#define OTA_HARD_TIMEOUT_MS        120000UL
#define OTA_MIN_FREE_HEAP_BYTES    10240UL


static RstReasonCode diagInitAndPrintResetReport(void)
{
    Serial.println();
    Serial.println(F("=============================================================="));
    Serial.println(F(" CRASH LOGGER — REPORT RESET / DIAGNOSTICA"));
    Serial.println(F("=============================================================="));

    diagStackCanaryInit();
    g_diag_stackWmBytes = stackWatermarkBytes();
    g_diag_minHeapEver = (uint32_t)ESP.getFreeHeap();
    g_diag_wdtFeedCount = 0;

    // ---- Decodifica reset corrente ----
    String reasonStr = ESP.getResetReason();
    String infoStr   = ESP.getResetInfo();
    const rst_info* ri = ESP.getResetInfoPtr();
    RstReasonCode rr = decodeEspResetReason(reasonStr.c_str(), ri);
    g_rstReasonCached = rr;
    if (ri != nullptr)
    {
        g_rstExccauseCached32 = (uint32_t)ri->exccause;
        g_rstExccauseCached = (uint8_t)ri->exccause;
        g_rstEpc1Cached = ri->epc1;
        g_rstEpc2Cached = ri->epc2;
        g_rstEpc3Cached = ri->epc3;
        g_rstExcvaddrCached = ri->excvaddr;
    }

    Serial.print  (F(" RESET REASON (SDK rst_info->reason byte 0): "));
    Serial.println(rstReasonStr(rr));
    Serial.print  (F("  [Arduino String]: ")); Serial.println(reasonStr);
    Serial.print  (F("  [SDK rst_info full hex dump]: ")); Serial.println(infoStr);
    if (ri != nullptr)
    {
        Serial.print(F("  rtc_info.reason     (hex) = 0x")); Serial.println(ri->reason, HEX);
        if (ri->reason == REASON_EXCEPTION_RST || ri->reason == REASON_SOFT_WDT_RST)
        {
            Serial.print(F("  exccause  = ")); Serial.print(ri->exccause, DEC);
            Serial.print(F(" (0x")); Serial.print((uint32_t)ri->exccause, HEX); Serial.println(F(")"));
            Serial.print(F("  epc1=0x")); Serial.print(ri->epc1, HEX);
            Serial.print(F("  epc2=0x")); Serial.print(ri->epc2, HEX);
            Serial.print(F("  epc3=0x")); Serial.println(ri->epc3, HEX);
            Serial.print(F("  excvaddr=0x")); Serial.println(ri->excvaddr, HEX);
            Serial.print(F("  depc=0x"));    Serial.println(ri->depc, HEX);
        }
    }
    Serial.print(F("  CPU Freq: ")); Serial.print(ESP.getCpuFreqMHz()); Serial.println(F(" MHz"));
    Serial.print(F("  Core Ver: ")); Serial.println(ESP.getCoreVersion());
    Serial.print(F("  FW Ver  : ")); Serial.println(FW_VERSION_STR);
    Serial.print(F("  FW Build: ")); Serial.println(FW_BUILD_DATE);
    Serial.print(F("  Boot Ver: ")); Serial.println(ESP.getBootVersion());
    Serial.print(F("  Boot Mode:")); Serial.println(ESP.getBootMode());

    // ---- Load ultimo crash salvato ----
    CrashContext cc;
    if (crashLoadContext(&cc))
    {
        Serial.println(F("--------------------------------------------------------------"));
        Serial.println(F(" ULTIMO CRASH SALVATO (RTC user memory, CRC OK):"));
        Serial.print  (F("   Causa       : ")); Serial.println(rstReasonStr(cc.rstReason));
        Serial.print  (F("   Uptime crash: ")); Serial.print(cc.uptimeMs); Serial.println(F(" ms"));
        Serial.print  (F("   Heap free   : ")); Serial.print(cc.heapFree); Serial.println(F(" bytes"));
        Serial.print  (F("   Heap min    : ")); Serial.print(cc.heapMin);  Serial.println(F(" bytes"));
        Serial.print  (F("   Heap frag   : ")); Serial.print(cc.heapFragPct); Serial.println(F(" %"));
        Serial.print  (F("   Stack used  : ")); Serial.print(cc.stackWmBytes); Serial.println(F(" bytes"));
        Serial.print  (F("   WDT feeds   : ")); Serial.println(cc.wdtFeedCount);
        Serial.print  (F("   MQTT conn   : "));
        Serial.print(((cc.mqttConnWifiWord >> 16) & 1u) ? F("SI") : F("NO"));
        Serial.print(F("   WiFi.status=0x"));
        Serial.println(cc.mqttConnWifiWord & 0xFFu, HEX);
        Serial.print  (F("   Sensors bitmask=0x")); Serial.print((cc.sensorStateWord >> 16) & 0xFFu, HEX);
        Serial.print  (F("  ds18_conv_pending=")); Serial.println(cc.sensorStateWord & 1u);

        // Snapshot di questo boot come "nuovo crash" solo se reset non è PowerOn.
        // PowerOn = RTC ancora valida da boot precedente ma non è un crash.
        if (rr == RST_POWERON)
        {
            Serial.println(F("   (questo boot è POWER ON: RTC crash da avvio precedente, NON un crash)"));
            crashClearContext();
        }
        else
        {
            // Verifica se il crash è appena successo in questo boot
            Serial.println(F("   >>> QUESTO È UN CRASH RECENTE. Context caricato."));
        }
    }
    else
    {
        Serial.println(F(" RTC crash: NESSUN crash context valido trovato (primo boot o power cycle)"));
    }

    // ---- Load ultimo heartbeat se crash context vuoto ma abbiamo heartbeat ----
    uint32_t hb[HEARTBEAT_RTC_WORDS];
    if (crashHeartbeatLoad(hb))
    {
        Serial.println(F("--------------------------------------------------------------"));
        Serial.println(F(" ULTIMO HEARTBEAT PRE-CRASH valido (RTC words 48..63):"));
        Serial.print  (F("   uptime_beat_ms   = ")); Serial.println(hb[2]);
        Serial.print  (F("   free_heap        = ")); Serial.print(hb[3]); Serial.println(F(" B"));
        Serial.print  (F("   min_heap_ever    = ")); Serial.print(hb[4]); Serial.println(F(" B"));
        Serial.print  (F("   heap_frag        = ")); Serial.print(hb[5]); Serial.println(F(" %"));
        Serial.print  (F("   max_free_block   = ")); Serial.print(hb[6]); Serial.println(F(" B"));
        Serial.print  (F("   stack_used_wm    = ")); Serial.print(hb[7]); Serial.println(F(" B"));
        Serial.print  (F("   wdt_feed_count   = ")); Serial.println(hb[8]);
        Serial.print  (F("   mqtt_conn        = ")); Serial.print(((hb[9] >> 16) & 1u) ? F("SI") : F("NO"));
        Serial.print  (F("   wifi_st          = 0x")); Serial.println(hb[9] & 0xFFu, HEX);
        Serial.print  (F("   status_bits      = 0x")); Serial.print((hb[10] >> 16) & 0xFFu, HEX);
        Serial.print  (F("  ds18_conv_pending = ")); Serial.println(hb[10] & 1u);
        Serial.print  (F("   mqtt_consec_fail = ")); Serial.println(hb[11]);
        Serial.print  (F("   supv L1/L2/L3/RST= ")); Serial.print(hb[12] & 0xFFu);
        Serial.print('/'); Serial.print((hb[12] >> 8) & 0xFFu);
        Serial.print('/'); Serial.print((hb[12] >> 16) & 0xFFu);
        Serial.print('/'); Serial.println((hb[12] >> 24) & 0xFFu);
        Serial.print  (F("   boot_count       = ")); Serial.println(hb[13]);
    }

    // ---- Stato attuale ----
    Serial.println(F("--------------------------------------------------------------"));
    Serial.print  (F(" STATO ATTUALE boot: free_heap="));
    Serial.print(ESP.getFreeHeap());
    Serial.print(F("  max_block="));
    Serial.print(ESP.getMaxFreeBlockSize());
    Serial.print(F("  frag="));
    Serial.print(ESP.getHeapFragmentation());
    Serial.println(F("%"));
    Serial.print  (F("   stack_available_now=")); Serial.print(stackAvailableBytes());
    Serial.print(F("  stack_used_wm=")); Serial.println(g_diag_stackWmBytes);
    Serial.print  (F("   chip_id=0x")); Serial.print(ESP.getChipId(), HEX);
    Serial.print  (F("   flash_id=0x")); Serial.println(ESP.getFlashChipId(), HEX);
    Serial.print  (F("   flash_size=")); Serial.print(ESP.getFlashChipRealSize());
    Serial.print(F("  speed=")); Serial.print(ESP.getFlashChipSpeed());
    Serial.print(F("Hz  mode=")); Serial.println(ESP.getFlashChipMode());
    Serial.println(F("=============================================================="));
    Serial.flush();
    return rr;
}

// ============================================================================
// Pin assegnazione SHT31-D - Wemos D1 Mini (ESP8266)
//   D1 (GPIO5) = SCL, D2 (GPIO4) = SDA
//   ATTENZIONE: Wire.begin(sda, scl) su ESP8266 vuole SDA POI SCL
// ============================================================================
#define SHT31_I2C_ADDR                 0x44
#define SHT31_I2C_FREQ                 400000UL
#define SHT31_SAMPLE_INTERVAL_MS       1000
#define SHT31_TCONV_HIGH_MS            16
#define SHT31_CONV_TIME_MS             SHT31_TCONV_HIGH_MS
#define SHT31_CMD_SINGLE_HIGH_NOSTRETCH 0x2400
#define SHT31_CMD_SOFT_RESET           0x30A2
#define SHT31_CMD_HEATER_ON            0x306D
#define SHT31_CMD_HEATER_OFF           0x3066
#define SHT31_CMD_READ_STATUS          0xF32D
#define SHT31_CMD_CLEAR_STATUS         0x3041
#define SHT31_STATUS_HEATER_BIT        0x2000
#define SHT31_HEATER_ON_RH_PCT         95.0f
#define SHT31_HEATER_OFF_RH_PCT        90.0f
#define SHT31_HEATER_MIN_ON_MS         30000UL
#define SHT31_T_MIN_C                  -45.0f
#define SHT31_T_MAX_C                 130.0f
#define SHT31_RH_MIN_PCT                0.0f
#define SHT31_RH_MAX_PCT              100.0f

// ============================================================================
// Parametri di sistema
// ============================================================================
#define SERIAL_BAUDRATE     115200
#define SAMPLE_PERIOD_MS    10
#define HTTP_PORT            80
#define WS_PORT              81

// ============================================================================
// Credenziali WiFi
// ============================================================================
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define AP_SSID       "TempHumid-Node"
#define AP_PASSWORD   "CHANGE_ME_AP_PASS"

// ============================================================================
// Parametri RICONNESSIONE WiFi Robusta
//   Primo tentativo + 10 retry = 11 tentativi TOTALI prima dell'AP fallback.
//   Tra ogni tentativo fallito: hard reset completo del modulo WiFi + delay 2s.
//   Dopo 11 fallimenti: modalità AP+STA, STA continua a ritentare ogni 30s.
// ============================================================================
#define WIFI_STA_RETRY_COUNT          10U     // retry DOPO il primo tentativo (totale 11)
#define WIFI_STA_PER_TRY_TIMEOUT_MS   15000UL // timeout per OGNI singolo begin()
#define WIFI_STA_INTER_RETRY_DELAY_MS 2000UL  // pausa TRA un fallimento e il prossimo begin
#define WIFI_STA_HARD_RESET_OFF_MS    500UL   // durata WIFI_OFF nel reset tra retry
#define WIFI_FALLBACK_POLL_PERIOD_MS  30000UL // cadenza retry STA in modalità fallback AP+STA

// ============================================================================
// Configurazione MQTT DEFAULT (usate se EEPROM vuota o corrotta)
// ============================================================================
#define MQTT_BROKER_IP_DEFAULT        "192.168.1.100"
#define MQTT_BROKER_PORT_DEFAULT      1883
#define MQTT_USER_DEFAULT             "mqtt_user"
#define MQTT_PASSWORD_DEFAULT         "mqtt_password"
#define MQTT_DEVICE_ID_DEFAULT        "sensori_temperatura_umidita"
#define MQTT_DEVICE_NAME_DEFAULT      "Sensori Temperatura e Umidita"
#define MQTT_MANUFACTURER             "Paolo Prandoni"
#define MQTT_MODEL                    "NodeMCU 1xSHT31-D I2C (High Accuracy)"
#define MQTT_PUB_PERIOD_MS            1000
#define MQTT_CONNECT_BACKOFF_MIN_MS   250U
#define MQTT_CONNECT_BACKOFF_MAX_MS   2000U
#define MQTT_KEEPALIVE_S              10U
#define MQTT_SOCKET_TIMEOUT_S         5U
#define MQTT_CLIENT_BUFFER_SIZE       2048U
#define MQTT_ONLINE_REFRESH_MS        30000UL

// ============================================================================
// Parametri OTTIMIZZAZIONE TRAFFICO MQTT DEFAULT
// (usati se EEPROM vuota o corrotta; poi configurabili via Web UI in EEPROM v3)
// ============================================================================
#define THROTTLE_TIMER_DATA_BASE_MS_DEFAULT        5000UL     // TIMER B: base scan sensori (on-change avviene in questo loop)
#define THROTTLE_SAFETY_MAX_INTERVAL_MS_DEFAULT    60000UL    // Safety heartbeat: pubblica TUTTI i dati anche senza cambiamenti
#define THROTTLE_SUPV_FORCE_INTERVAL_MS_DEFAULT    10000UL    // Forza publish supv delta/last ogni N ms (se non gia' on-change)
#define THROTTLE_THRESH_TEMP_C_DEFAULT             0.1f       // Soglia Temperatura: 0.1°C
#define THROTTLE_THRESH_RH_PCT_DEFAULT             0.5f       // Soglia Umidita: 0.5%
#define THROTTLE_THRESH_DEW_C_DEFAULT              0.1f       // Soglia Dew Point: 0.1°C
#define THROTTLE_THRESH_HI_C_DEFAULT               0.1f       // Soglia Heat Index: 0.1°C
#define THROTTLE_THRESH_AH_GM3_DEFAULT             0.05f      // Soglia Abs Humidity: 0.05 g/m³
#define THROTTLE_THRESH_RSSI_DBM_DEFAULT           2          // Soglia RSSI: 2 dBm
#define THROTTLE_THRESH_FRAG_PCT_DEFAULT           1.0f       // Soglia Heap Frag: 1.0%

// Range safety min/max (hard-enforced in sanitizeConfig e Web UI input)
#define THROTTLE_TIMER_DATA_MIN_MS                 500UL
#define THROTTLE_TIMER_DATA_MAX_MS                 60000UL
#define THROTTLE_SAFETY_MIN_MS                     5000UL
#define THROTTLE_SAFETY_MAX_MS                     300000UL
#define THROTTLE_SUPV_FORCE_MIN_MS                 2000UL
#define THROTTLE_SUPV_FORCE_MAX_MS                 60000UL
#define THROTTLE_THRESH_T_MIN                      0.01f
#define THROTTLE_THRESH_T_MAX                      2.00f
#define THROTTLE_THRESH_RH_MIN                     0.1f
#define THROTTLE_THRESH_RH_MAX                     10.0f
#define THROTTLE_THRESH_AH_MIN                     0.01f
#define THROTTLE_THRESH_AH_MAX                     1.00f
#define THROTTLE_THRESH_RSSI_MIN                   1
#define THROTTLE_THRESH_RSSI_MAX                   10
#define THROTTLE_THRESH_FRAG_MIN                   0.5f
#define THROTTLE_THRESH_FRAG_MAX                   20.0f

// ============================================================================
// TCP KeepAlive livello trasporto (ESP8266 lwIP).
// Valori aggressivi per rilevare velocemente socket zombie half-open.
// Firma core ESP8266 3.1.2: keepAlive(uint16_t idle_sec, uint16_t intv_sec, uint8_t count)
// Massimo tempo per rilevare una connessione TCP morta silenziosamente:
//   IDLE + INTERVAL * COUNT = 5 + 3*3 = 14 sec
// (default lwIP 7200 + 75*9 = 7875 s ≈ 2h10m, inutile per MQTT 10s KA)
// ============================================================================
#define TCP_KEEPALIVE_IDLE_SEC         ((uint16_t)5)
#define TCP_KEEPALIVE_INTERVAL_SEC     ((uint16_t)3)
#define TCP_KEEPALIVE_COUNT            ((uint8_t)3)

// ============================================================================
// Supervisore MQTT + Watchdog
//   Soglie aggressive dopo riconnessione immediata:
//   trigger dopo 10s di stall, escalation L1-L4 rapida, reset completo a 120s.
// ============================================================================
#define SUPV_WDT_HW_MS                8000U
#define SUPV_CONN_STALL_MS            10000UL
#define SUPV_PUB_STALL_MS             10000UL
#define SUPV_L1_COOLDOWN_MS           3000UL
#define SUPV_L2_COOLDOWN_MS           5000UL
#define SUPV_L3_COOLDOWN_MS           8000UL
#define SUPV_L4_RESET_MS              120000UL

// ============================================================================
// Nomi sonde DEFAULT (usati se EEPROM vuota o corrotta)
// ============================================================================
#define DEVICE_NAME_DEFAULT           "Sensori Temp/Umid"
#define SENSOR_SHT31_NAME_DEFAULT     "Ambiente_SHT31"

// ============================================================================
// Layout EEPROM
// ============================================================================
#define EEPROM_OFFSET_CONFIG  0
#define EEPROM_SIZE_BYTES     4096
#define CONFIG_MAGIC_1        0x55AA55AAu
#define CONFIG_VERSION        4
#define MAX_LEN_DEVICE_NAME   40
#define MAX_LEN_SENSOR_NAME   40
#define MAX_LEN_MQTT_SERVER   64
#define MAX_LEN_MQTT_USER     48
#define MAX_LEN_MQTT_PASS     64
#define MAX_LEN_MQTT_DEVID    64
#define MAX_LEN_WIFI_SSID     33
#define MAX_LEN_WIFI_PASS     64
#define MAX_LEN_AP_SSID       33
#define MAX_LEN_AP_PASS       64

typedef struct Config_t {
    uint32_t magic;
    uint8_t  version;
    uint8_t  reserved1;
    uint16_t reserved2;

    char     device_name[MAX_LEN_DEVICE_NAME];

    char     sensor_sht31_name[MAX_LEN_SENSOR_NAME];
    char     reserved_sensor_2[MAX_LEN_SENSOR_NAME];
    char     reserved_sensor_3[MAX_LEN_SENSOR_NAME];
    char     reserved_sensor_4[MAX_LEN_SENSOR_NAME];

    char     mqtt_server[MAX_LEN_MQTT_SERVER];
    uint16_t mqtt_port;
    char     mqtt_user[MAX_LEN_MQTT_USER];
    char     mqtt_password[MAX_LEN_MQTT_PASS];
    char     mqtt_device_id[MAX_LEN_MQTT_DEVID];

    // --- Nuovi in v3: Ottimizzazione traffico MQTT ---
    uint32_t throttle_timer_data_base_ms;
    uint32_t throttle_safety_max_interval_ms;
    uint32_t throttle_supv_force_interval_ms;
    float    throttle_thresh_temp_c;
    float    throttle_thresh_rh_pct;
    float    throttle_thresh_dew_c;
    float    throttle_thresh_hi_c;
    float    throttle_thresh_ah_gm3;
    uint8_t  throttle_thresh_rssi_dbm;
    uint8_t  throttle_padding_1;
    uint16_t throttle_padding_2;
    float    throttle_thresh_frag_pct;

    // --- Nuovi in v4: Credenziali WiFi STA e AP ---
    char     wifi_ssid[MAX_LEN_WIFI_SSID];
    char     wifi_password[MAX_LEN_WIFI_PASS];
    char     ap_ssid[MAX_LEN_AP_SSID];
    char     ap_password[MAX_LEN_AP_PASS];

    uint32_t crc32;
} Config;

static_assert(sizeof(Config) <= (EEPROM_SIZE_BYTES - 32), "Config troppo grande per EEPROM");

static Config g_config;

// ============================================================================
// CRC-32 (ISO 3309 / zlib)
// ============================================================================
static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len)
{
    crc ^= 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t k = 0; k < 8; k++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static uint32_t configComputeCrc(const Config* cfg)
{
    const uint8_t* p = (const uint8_t*)cfg;
    const size_t payloadSize = sizeof(Config) - sizeof(uint32_t);
    return crc32Update(0, p, payloadSize);
}

static void safeStrncpy(char* dst, const char* src, size_t dstSize)
{
    if (!dst || dstSize == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    size_t i = 0;
    while (i + 1 < dstSize && src[i] != '\0') {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static bool isValidUtf8Printable(const char* s, size_t maxLen)
{
    if (!s) return false;
    size_t i = 0;
    while (s[i] != '\0') {
        if (i >= maxLen) return false;
        uint8_t c = (uint8_t)s[i];
        if (c < 0x20 || c == 0x7F) return false;
        i++;
    }
    return true;
}

static void sanitizeConfig(Config* cfg)
{
    if (!isValidUtf8Printable(cfg->device_name, sizeof(cfg->device_name))) {
        safeStrncpy(cfg->device_name, DEVICE_NAME_DEFAULT, sizeof(cfg->device_name));
    }
    if (!isValidUtf8Printable(cfg->sensor_sht31_name, sizeof(cfg->sensor_sht31_name))) {
        safeStrncpy(cfg->sensor_sht31_name, SENSOR_SHT31_NAME_DEFAULT, sizeof(cfg->sensor_sht31_name));
    }
    if (!isValidUtf8Printable(cfg->reserved_sensor_2, sizeof(cfg->reserved_sensor_2))) {
        memset(cfg->reserved_sensor_2, 0, sizeof(cfg->reserved_sensor_2));
    }
    if (!isValidUtf8Printable(cfg->reserved_sensor_3, sizeof(cfg->reserved_sensor_3))) {
        memset(cfg->reserved_sensor_3, 0, sizeof(cfg->reserved_sensor_3));
    }
    if (!isValidUtf8Printable(cfg->reserved_sensor_4, sizeof(cfg->reserved_sensor_4))) {
        memset(cfg->reserved_sensor_4, 0, sizeof(cfg->reserved_sensor_4));
    }
    if (!isValidUtf8Printable(cfg->mqtt_server, sizeof(cfg->mqtt_server))) {
        safeStrncpy(cfg->mqtt_server, MQTT_BROKER_IP_DEFAULT, sizeof(cfg->mqtt_server));
    }
    if (!isValidUtf8Printable(cfg->mqtt_user, sizeof(cfg->mqtt_user))) {
        safeStrncpy(cfg->mqtt_user, MQTT_USER_DEFAULT, sizeof(cfg->mqtt_user));
    }
    if (!isValidUtf8Printable(cfg->mqtt_password, sizeof(cfg->mqtt_password))) {
        safeStrncpy(cfg->mqtt_password, MQTT_PASSWORD_DEFAULT, sizeof(cfg->mqtt_password));
    }
    if (!isValidUtf8Printable(cfg->mqtt_device_id, sizeof(cfg->mqtt_device_id))) {
        safeStrncpy(cfg->mqtt_device_id, MQTT_DEVICE_ID_DEFAULT, sizeof(cfg->mqtt_device_id));
    }
    if (cfg->mqtt_port == 0 || cfg->mqtt_port > 65535) {
        cfg->mqtt_port = MQTT_BROKER_PORT_DEFAULT;
    }

    // --- Nuovi in v3: Clamp throttling params entro safety ranges ---
    #define CLAMP_U32(v, mn, mx) do { if ((v) < (mn)) (v) = (mn); else if ((v) > (mx)) (v) = (mx); } while(0)
    #define CLAMP_FLT(v, mn, mx) do { if (!isfinite(v) || (v) < (mn)) (v) = (mn); else if ((v) > (mx)) (v) = (mx); } while(0)
    #define CLAMP_U8(v, mn, mx)  do { if ((v) < (mn)) (v) = (mn); else if ((v) > (mx)) (v) = (mx); } while(0)

    CLAMP_U32(cfg->throttle_timer_data_base_ms,     THROTTLE_TIMER_DATA_MIN_MS,  THROTTLE_TIMER_DATA_MAX_MS);
    CLAMP_U32(cfg->throttle_safety_max_interval_ms, THROTTLE_SAFETY_MIN_MS,      THROTTLE_SAFETY_MAX_MS);
    CLAMP_U32(cfg->throttle_supv_force_interval_ms, THROTTLE_SUPV_FORCE_MIN_MS,  THROTTLE_SUPV_FORCE_MAX_MS);
    CLAMP_FLT(cfg->throttle_thresh_temp_c,  THROTTLE_THRESH_T_MIN,  THROTTLE_THRESH_T_MAX);
    CLAMP_FLT(cfg->throttle_thresh_rh_pct,  THROTTLE_THRESH_RH_MIN, THROTTLE_THRESH_RH_MAX);
    CLAMP_FLT(cfg->throttle_thresh_dew_c,   THROTTLE_THRESH_T_MIN,  THROTTLE_THRESH_T_MAX);
    CLAMP_FLT(cfg->throttle_thresh_hi_c,    THROTTLE_THRESH_T_MIN,  THROTTLE_THRESH_T_MAX);
    CLAMP_FLT(cfg->throttle_thresh_ah_gm3,  THROTTLE_THRESH_AH_MIN, THROTTLE_THRESH_AH_MAX);
    CLAMP_U8 (cfg->throttle_thresh_rssi_dbm,THROTTLE_THRESH_RSSI_MIN, THROTTLE_THRESH_RSSI_MAX);
    CLAMP_FLT(cfg->throttle_thresh_frag_pct,THROTTLE_THRESH_FRAG_MIN, THROTTLE_THRESH_FRAG_MAX);

    cfg->throttle_padding_1 = 0;
    cfg->throttle_padding_2 = 0;

    // --- Nuovi in v4: Sanitize credenziali WiFi STA e AP ---
    if (!isValidUtf8Printable(cfg->wifi_ssid, sizeof(cfg->wifi_ssid))) {
        safeStrncpy(cfg->wifi_ssid, WIFI_SSID, sizeof(cfg->wifi_ssid));
    }
    if (!isValidUtf8Printable(cfg->wifi_password, sizeof(cfg->wifi_password))) {
        safeStrncpy(cfg->wifi_password, WIFI_PASSWORD, sizeof(cfg->wifi_password));
    }
    if (!isValidUtf8Printable(cfg->ap_ssid, sizeof(cfg->ap_ssid))) {
        safeStrncpy(cfg->ap_ssid, AP_SSID, sizeof(cfg->ap_ssid));
    }
    if (!isValidUtf8Printable(cfg->ap_password, sizeof(cfg->ap_password))) {
        safeStrncpy(cfg->ap_password, AP_PASSWORD, sizeof(cfg->ap_password));
    }
    // WPA2 password AP: min 8 char, oppure vuota (AP aperta)
    size_t apPwdLen = strlen(cfg->ap_password);
    if (apPwdLen > 0 && apPwdLen < 8) {
        safeStrncpy(cfg->ap_password, AP_PASSWORD, sizeof(cfg->ap_password));
    }

    #undef CLAMP_U32
    #undef CLAMP_FLT
    #undef CLAMP_U8
}

static void configLoadDefaults(Config* cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = CONFIG_MAGIC_1;
    cfg->version = CONFIG_VERSION;
    safeStrncpy(cfg->device_name, DEVICE_NAME_DEFAULT, sizeof(cfg->device_name));
    safeStrncpy(cfg->sensor_sht31_name, SENSOR_SHT31_NAME_DEFAULT, sizeof(cfg->sensor_sht31_name));
    safeStrncpy(cfg->mqtt_server, MQTT_BROKER_IP_DEFAULT, sizeof(cfg->mqtt_server));
    cfg->mqtt_port = MQTT_BROKER_PORT_DEFAULT;
    safeStrncpy(cfg->mqtt_user, MQTT_USER_DEFAULT, sizeof(cfg->mqtt_user));
    safeStrncpy(cfg->mqtt_password, MQTT_PASSWORD_DEFAULT, sizeof(cfg->mqtt_password));
    safeStrncpy(cfg->mqtt_device_id, MQTT_DEVICE_ID_DEFAULT, sizeof(cfg->mqtt_device_id));

    // --- Nuovi in v3: valori default throttling ---
    cfg->throttle_timer_data_base_ms     = THROTTLE_TIMER_DATA_BASE_MS_DEFAULT;
    cfg->throttle_safety_max_interval_ms = THROTTLE_SAFETY_MAX_INTERVAL_MS_DEFAULT;
    cfg->throttle_supv_force_interval_ms = THROTTLE_SUPV_FORCE_INTERVAL_MS_DEFAULT;
    cfg->throttle_thresh_temp_c          = THROTTLE_THRESH_TEMP_C_DEFAULT;
    cfg->throttle_thresh_rh_pct          = THROTTLE_THRESH_RH_PCT_DEFAULT;
    cfg->throttle_thresh_dew_c           = THROTTLE_THRESH_DEW_C_DEFAULT;
    cfg->throttle_thresh_hi_c            = THROTTLE_THRESH_HI_C_DEFAULT;
    cfg->throttle_thresh_ah_gm3          = THROTTLE_THRESH_AH_GM3_DEFAULT;
    cfg->throttle_thresh_rssi_dbm        = THROTTLE_THRESH_RSSI_DBM_DEFAULT;
    cfg->throttle_thresh_frag_pct        = THROTTLE_THRESH_FRAG_PCT_DEFAULT;

    // --- Nuovi in v4: valori default credenziali WiFi STA e AP ---
    safeStrncpy(cfg->wifi_ssid,     WIFI_SSID,     sizeof(cfg->wifi_ssid));
    safeStrncpy(cfg->wifi_password, WIFI_PASSWORD, sizeof(cfg->wifi_password));
    safeStrncpy(cfg->ap_ssid,       AP_SSID,       sizeof(cfg->ap_ssid));
    safeStrncpy(cfg->ap_password,   AP_PASSWORD,   sizeof(cfg->ap_password));

    sanitizeConfig(cfg);
    cfg->crc32 = configComputeCrc(cfg);
}

static bool configSaveToEeprom(const Config* cfg);

static bool configLoadFromEeprom(Config* cfg)
{
    EEPROM.begin(EEPROM_SIZE_BYTES);
    uint8_t* p = (uint8_t*)cfg;
    for (size_t i = 0; i < sizeof(Config); i++) {
        p[i] = EEPROM.read(EEPROM_OFFSET_CONFIG + i);
    }
    bool valid = false;
    if (cfg->magic == CONFIG_MAGIC_1 && cfg->version == CONFIG_VERSION) {
        uint32_t expected = configComputeCrc(cfg);
        valid = (expected == cfg->crc32);
    }
    if (!valid && cfg->magic == CONFIG_MAGIC_1 && cfg->version == 1) {
        Serial.println(F("EEPROM: CONFIG v1 trovata -> MIGRAZIONE SILENTE a v4 ..."));
        char tmp_device[MAX_LEN_DEVICE_NAME];
        char tmp_mqtt_srv[MAX_LEN_MQTT_SERVER];
        uint16_t tmp_mqtt_port;
        char tmp_mqtt_user[MAX_LEN_MQTT_USER];
        char tmp_mqtt_pass[MAX_LEN_MQTT_PASS];
        char tmp_mqtt_devid[MAX_LEN_MQTT_DEVID];
        safeStrncpy(tmp_device, cfg->device_name, sizeof(tmp_device));
        safeStrncpy(tmp_mqtt_srv, cfg->mqtt_server, sizeof(tmp_mqtt_srv));
        tmp_mqtt_port = cfg->mqtt_port;
        safeStrncpy(tmp_mqtt_user, cfg->mqtt_user, sizeof(tmp_mqtt_user));
        safeStrncpy(tmp_mqtt_pass, cfg->mqtt_password, sizeof(tmp_mqtt_pass));
        safeStrncpy(tmp_mqtt_devid, cfg->mqtt_device_id, sizeof(tmp_mqtt_devid));
        configLoadDefaults(cfg);
        safeStrncpy(cfg->device_name, tmp_device, sizeof(cfg->device_name));
        safeStrncpy(cfg->mqtt_server, tmp_mqtt_srv, sizeof(cfg->mqtt_server));
        cfg->mqtt_port = tmp_mqtt_port;
        safeStrncpy(cfg->mqtt_user, tmp_mqtt_user, sizeof(cfg->mqtt_user));
        safeStrncpy(cfg->mqtt_password, tmp_mqtt_pass, sizeof(cfg->mqtt_password));
        safeStrncpy(cfg->mqtt_device_id, tmp_mqtt_devid, sizeof(cfg->mqtt_device_id));
        cfg->version = CONFIG_VERSION;
        sanitizeConfig(cfg);
        cfg->crc32 = configComputeCrc(cfg);
        (void)configSaveToEeprom(cfg);
        valid = true;
        Serial.println(F("EEPROM: migrazione v1 -> v4 COMPLETATA e salvata."));
    }
    if (!valid && cfg->magic == CONFIG_MAGIC_1 && cfg->version == 2) {
        Serial.println(F("EEPROM: CONFIG v2 trovata -> MIGRAZIONE SILENTE a v4 ..."));
        char tmp_device[MAX_LEN_DEVICE_NAME];
        char tmp_sht31[MAX_LEN_SENSOR_NAME];
        char tmp_mqtt_srv[MAX_LEN_MQTT_SERVER];
        uint16_t tmp_mqtt_port;
        char tmp_mqtt_user[MAX_LEN_MQTT_USER];
        char tmp_mqtt_pass[MAX_LEN_MQTT_PASS];
        char tmp_mqtt_devid[MAX_LEN_MQTT_DEVID];
        safeStrncpy(tmp_device, cfg->device_name, sizeof(tmp_device));
        safeStrncpy(tmp_sht31, cfg->sensor_sht31_name, sizeof(tmp_sht31));
        safeStrncpy(tmp_mqtt_srv, cfg->mqtt_server, sizeof(tmp_mqtt_srv));
        tmp_mqtt_port = cfg->mqtt_port;
        safeStrncpy(tmp_mqtt_user, cfg->mqtt_user, sizeof(tmp_mqtt_user));
        safeStrncpy(tmp_mqtt_pass, cfg->mqtt_password, sizeof(tmp_mqtt_pass));
        safeStrncpy(tmp_mqtt_devid, cfg->mqtt_device_id, sizeof(tmp_mqtt_devid));
        configLoadDefaults(cfg);
        safeStrncpy(cfg->device_name, tmp_device, sizeof(cfg->device_name));
        safeStrncpy(cfg->sensor_sht31_name, tmp_sht31, sizeof(cfg->sensor_sht31_name));
        safeStrncpy(cfg->mqtt_server, tmp_mqtt_srv, sizeof(cfg->mqtt_server));
        cfg->mqtt_port = tmp_mqtt_port;
        safeStrncpy(cfg->mqtt_user, tmp_mqtt_user, sizeof(cfg->mqtt_user));
        safeStrncpy(cfg->mqtt_password, tmp_mqtt_pass, sizeof(cfg->mqtt_password));
        safeStrncpy(cfg->mqtt_device_id, tmp_mqtt_devid, sizeof(cfg->mqtt_device_id));
        cfg->version = CONFIG_VERSION;
        sanitizeConfig(cfg);
        cfg->crc32 = configComputeCrc(cfg);
        (void)configSaveToEeprom(cfg);
        valid = true;
        Serial.println(F("EEPROM: migrazione v2 -> v4 COMPLETATA e salvata."));
    }
    if (!valid && cfg->magic == CONFIG_MAGIC_1 && cfg->version == 3) {
        Serial.println(F("EEPROM: CONFIG v3 trovata -> MIGRAZIONE SILENTE a v4 ..."));
        char tmp_device[MAX_LEN_DEVICE_NAME];
        char tmp_sht31[MAX_LEN_SENSOR_NAME];
        char tmp_mqtt_srv[MAX_LEN_MQTT_SERVER];
        uint16_t tmp_mqtt_port;
        char tmp_mqtt_user[MAX_LEN_MQTT_USER];
        char tmp_mqtt_pass[MAX_LEN_MQTT_PASS];
        char tmp_mqtt_devid[MAX_LEN_MQTT_DEVID];
        uint32_t tmp_throttle_timer, tmp_throttle_safety, tmp_throttle_supv;
        float    tmp_th_t, tmp_th_rh, tmp_th_dw, tmp_th_hi, tmp_th_ah, tmp_th_frag;
        uint8_t  tmp_th_rssi;
        safeStrncpy(tmp_device, cfg->device_name, sizeof(tmp_device));
        safeStrncpy(tmp_sht31, cfg->sensor_sht31_name, sizeof(tmp_sht31));
        safeStrncpy(tmp_mqtt_srv, cfg->mqtt_server, sizeof(tmp_mqtt_srv));
        tmp_mqtt_port = cfg->mqtt_port;
        safeStrncpy(tmp_mqtt_user, cfg->mqtt_user, sizeof(tmp_mqtt_user));
        safeStrncpy(tmp_mqtt_pass, cfg->mqtt_password, sizeof(tmp_mqtt_pass));
        safeStrncpy(tmp_mqtt_devid, cfg->mqtt_device_id, sizeof(tmp_mqtt_devid));
        tmp_throttle_timer  = cfg->throttle_timer_data_base_ms;
        tmp_throttle_safety = cfg->throttle_safety_max_interval_ms;
        tmp_throttle_supv   = cfg->throttle_supv_force_interval_ms;
        tmp_th_t    = cfg->throttle_thresh_temp_c;
        tmp_th_rh   = cfg->throttle_thresh_rh_pct;
        tmp_th_dw   = cfg->throttle_thresh_dew_c;
        tmp_th_hi   = cfg->throttle_thresh_hi_c;
        tmp_th_ah   = cfg->throttle_thresh_ah_gm3;
        tmp_th_rssi = cfg->throttle_thresh_rssi_dbm;
        tmp_th_frag = cfg->throttle_thresh_frag_pct;
        configLoadDefaults(cfg);
        safeStrncpy(cfg->device_name, tmp_device, sizeof(cfg->device_name));
        safeStrncpy(cfg->sensor_sht31_name, tmp_sht31, sizeof(cfg->sensor_sht31_name));
        safeStrncpy(cfg->mqtt_server, tmp_mqtt_srv, sizeof(cfg->mqtt_server));
        cfg->mqtt_port = tmp_mqtt_port;
        safeStrncpy(cfg->mqtt_user, tmp_mqtt_user, sizeof(cfg->mqtt_user));
        safeStrncpy(cfg->mqtt_password, tmp_mqtt_pass, sizeof(cfg->mqtt_password));
        safeStrncpy(cfg->mqtt_device_id, tmp_mqtt_devid, sizeof(cfg->mqtt_device_id));
        cfg->throttle_timer_data_base_ms     = tmp_throttle_timer;
        cfg->throttle_safety_max_interval_ms = tmp_throttle_safety;
        cfg->throttle_supv_force_interval_ms = tmp_throttle_supv;
        cfg->throttle_thresh_temp_c   = tmp_th_t;
        cfg->throttle_thresh_rh_pct   = tmp_th_rh;
        cfg->throttle_thresh_dew_c    = tmp_th_dw;
        cfg->throttle_thresh_hi_c     = tmp_th_hi;
        cfg->throttle_thresh_ah_gm3   = tmp_th_ah;
        cfg->throttle_thresh_rssi_dbm = tmp_th_rssi;
        cfg->throttle_thresh_frag_pct = tmp_th_frag;
        cfg->version = CONFIG_VERSION;
        sanitizeConfig(cfg);
        cfg->crc32 = configComputeCrc(cfg);
        (void)configSaveToEeprom(cfg);
        valid = true;
        Serial.println(F("EEPROM: migrazione v3 -> v4 COMPLETATA e salvata."));
    }
    EEPROM.end();
    if (valid) {
        sanitizeConfig(cfg);
    } else {
        configLoadDefaults(cfg);
    }
    return valid;
}

static bool configSaveToEeprom(const Config* cfg)
{
    sanitizeConfig((Config*)cfg);
    Config tmp = *cfg;
    tmp.crc32 = configComputeCrc(&tmp);

    EEPROM.begin(EEPROM_SIZE_BYTES);
    const uint8_t* p = (const uint8_t*)&tmp;
    for (size_t i = 0; i < sizeof(Config); i++) {
        EEPROM.write(EEPROM_OFFSET_CONFIG + i, p[i]);
        if ((i & 0x3Fu) == 0u) {
            ESP.wdtFeed();
            g_diag_wdtFeedCount++;
        }
    }
    ESP.wdtFeed();
    bool ok = EEPROM.commit();
    ESP.wdtFeed();
    EEPROM.end();
    if (ok) {
        memcpy(&g_config, &tmp, sizeof(Config));
    }
    return ok;
}

// ============================================================================
// Stato sensori (bitmask per diagnostica) SHT31
// ============================================================================
#define STATUS_SHT31_T_OK     0x01
#define STATUS_SHT31_RH_OK    0x02
#define STATUS_SHT31_CRC_OK   0x04
#define STATUS_SHT31_I2C_OK   0x08
#define STATUS_SHT31_HEAT_ON  0x10
#define STATUS_SHT31_ALERT    0x20
#define STATUS_MASK_ALL       0x3F

// ============================================================================
// Forward declarations (chiamate a monte delle definizioni reali)
// ============================================================================
static inline void supervisorWatchdogFeed(void);
static void supervisorPoll(void);

// ============================================================================
// MQTT THROTTLING: ultimo stato pubblicato (per dead-band on-change detection)
// ============================================================================
typedef struct {
    // --- Valori sensori ambientali (confronto con soglie) ---
    float    sht31_t;
    float    sht31_rh;
    float    sht31_dp;
    float    sht31_hi;
    float    sht31_ah;
    bool     sht31_heater;

    // --- Stato sensore: qualsiasi cambiamento = publish ---
    uint8_t  sensor_status_bits;

    // --- Supervisore: level/counters on-change; trip/last_good forza ogni 10s ---
    uint8_t  supv_level;
    uint32_t supv_l1_count;
    uint32_t supv_l2_count;
    uint32_t supv_l3_count;
    uint32_t supv_reset_count;

    // --- Diagnostica statica (on-connect + on-change) ---
    uint8_t  diag_reset_reason_id;
    uint32_t diag_boot_count;
    uint32_t diag_min_heap_B;
    uint32_t diag_stack_wm_B;

    // --- Diagnostica dinamica (confronto con soglie) ---
    int8_t   diag_wifi_rssi_dbm;   // signed: confronto assoluto
    float    diag_heap_frag_pct;
    uint32_t diag_wifi_sta_ip_v4;  // IP STA (WiFi.localIP() come uint32 network-order: 0xC0A80037)
} LastPublishedState;

static LastPublishedState g_lastPub;
static bool g_lastPubValid = false;   // true dopo almeno 1 publish riuscita

// ============================================================================
// MQTT THROTTLING: timer separati (TIMER A 1s / TIMER B 5s + safety + supv 10s)
// ============================================================================
static unsigned long g_throttle_lastTimerA_ms = 0;   // uptime + wdt: 1s invariato
static unsigned long g_throttle_lastTimerB_ms = 0;   // dati ambientali/supv: base 5s
static unsigned long g_throttle_lastSafety_ms = 0;   // safety max interval: forza publish tutti
static unsigned long g_throttle_lastSupvForce_ms = 0;// forza supv delta/last ogni 10s
static unsigned long g_throttle_lastStaticPublish_ms = 0;  // quando abbiamo inviato i statici (on-connect)
static bool          g_throttle_needStaticOnConnect = true; // vera dopo riconnessione MQTT

// ============================================================================
// Nuove forward declarations throttling
// ============================================================================
static void throttleResetAllState(void);
static bool throttleExceedsThreshFloat(float cur, float last, float thresh);
static bool throttleExceedsThreshAbsInt(int cur, int last, uint8_t thresh);
static void mqttPublishUptimeWdtOnly(void);
static void mqttPublishStaticOnConnect(void);
static void mqttPublishAllDataSmart(void);
static void handleApiFwInfo(void);
static void handleUploadChunk(void);
static void handleUploadFinish(void);

// ============================================================================
// SHT31-D globali (HIGH accuracy, single-shot 1Hz) — dichiarate QUI per essere
// visibili in sht31_send_cmd / sht31_init / sht31_poll definiti in seguito
// ============================================================================
static float         g_sht31_t  = NAN;
static float         g_sht31_rh = NAN;
static float         g_sht31_dp = NAN;
static float         g_sht31_hi = NAN;
static float         g_sht31_ah = NAN;
static bool          g_sht31_heater = false;
static unsigned long g_sht31_heater_start_ms = 0;
static bool          g_sht31_convPending = false;
static unsigned long g_sht31_convStart_ms = 0;
static unsigned long g_sht31_lastSample_ms = 0;
static uint8_t       g_status_bits = 0;

// ============================================================================
// Front-end inline: dashboard + configurazione
// ============================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="it">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Sensori T/U · Configurabile</title>
<style>
  :root{
    --bg:#0e0f11;
    --panel:#15171b;
    --panel-h:#1c1f25;
    --border:#2a2e36;
    --ink:#e6e8ec;
    --ink-dim:#a7adb6;
    --ink-more:#7b828c;
    --ok:#66bb6a;
    --warn:#ffb300;
    --err:#ef5350;
    --accent:#42a5f5;
    --accent-h:#1e88e5;
    --mono:"Courier New", ui-monospace, monospace;
  }
  *{box-sizing:border-box}
  html,body{margin:0;padding:0;min-height:100%;background:var(--bg);color:var(--ink);
    font-family:ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial;
  }
  .wrap{padding:18px;max-width:980px;margin:0 auto}
  header{display:flex;align-items:center;justify-content:space-between;gap:14px;margin-bottom:16px;flex-wrap:wrap}
  header h1{margin:0;font-size:17px;letter-spacing:.3px;font-weight:600;color:#e9ecef}
  header h1 .devname{color:var(--accent);font-weight:700}
  header .sub{color:var(--ink-more);font-size:12px;margin-top:2px}
  .status{display:flex;align-items:center;gap:8px;color:var(--ink-dim);font-size:12px;font-family:var(--mono)}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--err);box-shadow:0 0 8px var(--err)}
  .dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok)}
  .tabs{display:flex;gap:6px;margin-bottom:14px;flex-wrap:wrap}
  .tab{padding:8px 14px;background:var(--panel);border:1px solid var(--border);border-radius:8px;
    color:var(--ink-dim);cursor:pointer;font-size:13px;font-weight:600;user-select:none;transition:all .15s}
  .tab:hover{border-color:var(--accent);color:var(--ink)}
  .tab.active{background:linear-gradient(180deg,var(--accent-h),#1565c0);color:#fff;border-color:#0d47a1;box-shadow:0 2px 8px rgba(66,165,245,.3)}
  .view{display:none}
  .view.active{display:block}
  table.dati{
    width:100%;border-collapse:collapse;background:var(--panel);
    border:1px solid var(--border);border-radius:10px;overflow:hidden;
    box-shadow:0 8px 28px rgba(0,0,0,.35);
  }
  table.dati thead th{
    background:linear-gradient(180deg,var(--panel-h),#14161a);
    color:#dde2e8;text-align:left;font-weight:600;font-size:11px;letter-spacing:1px;
    text-transform:uppercase;padding:10px 14px;border-bottom:1px solid var(--border);
    position:sticky;top:0;z-index:2;
  }
  table.dati thead th.right{text-align:right}
  table.dati tbody td{padding:8px 14px;border-bottom:1px solid #1e2128;font-size:13px;vertical-align:middle}
  table.dati tbody tr:last-child td{border-bottom:none}
  table.dati tbody tr.cat td{
    background:linear-gradient(180deg,#242830,#1b1e25);
    color:#f4f6f9;font-weight:700;font-size:12px;letter-spacing:1px;
    text-transform:uppercase;padding:9px 14px;border-bottom:1px solid var(--border);
    border-top:1px solid #0a0b0d;
  }
  table.dati tbody tr.cat td:first-child{padding-left:14px}
  table.dati .k{color:var(--ink-dim)}
  table.dati .v{color:#f6f8fa;font-family:var(--mono);font-size:14px;text-align:right;font-variant-numeric:tabular-nums}
  table.dati .u{color:var(--ink-more);font-family:var(--mono);font-size:11px;padding-left:6px}
  table.dati .note{color:var(--ink-more);font-size:11px}
  table.dati .v.bad{color:var(--err)}
  table.dati .v.warn{color:var(--warn)}
  table.dati .v.ok{color:var(--ok)}
  .footer{margin-top:14px;color:var(--ink-more);font-size:11px;display:flex;justify-content:space-between;flex-wrap:wrap;gap:6px;font-family:var(--mono)}
  .card{background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:16px;box-shadow:0 8px 28px rgba(0,0,0,.3);margin-bottom:14px}
  .card h2{margin:0 0 12px 0;font-size:14px;font-weight:700;letter-spacing:.5px;color:#dde2e8;text-transform:uppercase}
  .card h3{margin:14px 0 8px 0;font-size:12px;font-weight:700;color:var(--accent);letter-spacing:.8px;text-transform:uppercase}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px 18px}
  label.f{display:block;font-size:12px;color:var(--ink-dim);margin-bottom:4px;font-weight:600;letter-spacing:.3px}
  input[type="text"],input[type="password"],input[type="number"]{
    width:100%;padding:9px 11px;background:#0a0b0d;color:var(--ink);
    border:1px solid var(--border);border-radius:7px;font-size:13px;font-family:inherit;
    transition:border-color .15s,box-shadow .15s;
  }
  input[type="text"]:focus,input[type="password"]:focus,input[type="number"]:focus{
    outline:none;border-color:var(--accent);box-shadow:0 0 0 3px rgba(66,165,245,.15);
  }
  .actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:14px}
  button.btn{
    padding:9px 16px;border-radius:7px;font-size:13px;font-weight:700;cursor:pointer;
    border:1px solid transparent;transition:all .15s;font-family:inherit;letter-spacing:.2px;
  }
  button.btn.primary{background:linear-gradient(180deg,var(--accent),var(--accent-h));color:#fff;border-color:#0d47a1;box-shadow:0 2px 8px rgba(66,165,245,.25)}
  button.btn.primary:hover{filter:brightness(1.08)}
  button.btn.primary:disabled{filter:grayscale(.5);opacity:.6;cursor:not-allowed}
  button.btn.danger{background:linear-gradient(180deg,#ef5350,#c62828);color:#fff;border-color:#b71c1c;box-shadow:0 2px 8px rgba(239,83,80,.2)}
  button.btn.ghost{background:transparent;color:var(--ink-dim);border-color:var(--border)}
  button.btn.ghost:hover{color:var(--ink);border-color:var(--ink-dim)}
  .toast{position:fixed;top:16px;left:50%;transform:translateX(-50%);z-index:999;
    padding:10px 16px;border-radius:8px;font-size:13px;font-weight:600;opacity:0;pointer-events:none;transition:opacity .25s,transform .25s;
    box-shadow:0 8px 24px rgba(0,0,0,.5);max-width:90vw;
  }
  .toast.show{opacity:1;transform:translateX(-50%) translateY(4px)}
  .toast.ok{background:#2e7d32;color:#fff;border:1px solid #1b5e20}
  .toast.err{background:#c62828;color:#fff;border:1px solid #b71c1c}
  .toast.warn{background:#ef6c00;color:#fff;border:1px solid #e65100}
  .info-block{background:#0a0b0d;border:1px solid var(--border);border-radius:7px;padding:10px 12px;font-family:var(--mono);font-size:12px;color:var(--ink-dim);word-break:break-all;margin-top:4px}
  .sec-hint{color:var(--ink-more);font-size:11px;margin-top:3px}
  .warn-box{background:rgba(239,83,80,.08);border:1px solid rgba(239,83,80,.35);border-radius:8px;padding:11px 14px;color:#ffab91;font-size:12.5px;font-weight:600;line-height:1.45;margin-bottom:12px}
  .warn-box b{color:#ffccbc}
  .fw-log{background:#05060a;border:1px solid var(--border);border-radius:7px;padding:10px 12px;max-height:220px;overflow-y:auto;font-family:var(--mono);font-size:11.5px;line-height:1.55;color:#b5bac4;margin-top:8px}
  .fw-log .l-time{color:#5c6370;margin-right:6px;font-variant-numeric:tabular-nums}
  .fw-log .l-ok{color:#66bb6a}
  .fw-log .l-err{color:#ef5350}
  .fw-log .l-warn{color:#ffb300}
  .fw-log .l-info{color:#42a5f5}
  .progress-wrap{margin-top:6px;background:#0a0b0d;border:1px solid var(--border);border-radius:7px;overflow:hidden;height:24px;position:relative}
  .progress-wrap progress{width:100%;height:100%;appearance:none;-webkit-appearance:none;border:0;background:transparent}
  .progress-wrap progress::-webkit-progress-bar{background:#0a0b0d}
  .progress-wrap progress::-webkit-progress-value{background:linear-gradient(90deg,#42a5f5,#1e88e5)}
  .progress-wrap progress::-moz-progress-bar{background:linear-gradient(90deg,#42a5f5,#1e88e5)}
  .progress-wrap .p-text{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-family:var(--mono);font-size:12px;color:#e9ecef;font-weight:700;letter-spacing:.3px;text-shadow:0 1px 2px rgba(0,0,0,.6)}
  .file-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:8px}
  .file-row input[type=file]{color:var(--ink-dim);font-size:12.5px;max-width:100%}
  .file-meta{font-family:var(--mono);font-size:11.5px;color:var(--ink-more);margin-left:auto}
  @media(max-width:640px){
    table.dati thead th,table.dati tbody td{padding:7px 8px;font-size:12px}
    table.dati .v{font-size:12px}
    header h1{font-size:15px}
    .card{padding:12px}
  }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div>
      <h1>Sensori Temperatura · Umidità · <span class="devname" id="hdr-name">...</span></h1>
      <div class="sub" id="hdr-sub">SHT31-D I²C (D1=SCL · D2=SDA) · HIGH accuracy 1Hz · HeatIndex · AssHum · DewPoint</div>
    </div>
    <div class="status">
      <span id="dot" class="dot"></span>
      <span id="stat">connessione...</span>
    </div>
  </header>

  <div class="tabs" id="tabs">
    <div class="tab active" data-view="view-dash">Dashboard</div>
    <div class="tab" data-view="view-cfg">⚙ Configurazione</div>
    <div class="tab" data-view="view-fw">🔧 Firmware</div>
  </div>

  <div id="view-dash" class="view active">
    <table class="dati" id="tab">
      <thead>
        <tr>
          <th style="width:38%">Parametro</th>
          <th class="right" style="width:34%">Valore</th>
          <th style="width:28%">Note / Unità</th>
        </tr>
      </thead>
      <tbody id="rows"></tbody>
    </table>
    <div class="footer">
      <div id="f-host"></div>
      <div id="f-ts"></div>
    </div>
  </div>

  <div id="view-cfg" class="view">
    <div class="card">
      <h2>Identificativo Dispositivo</h2>
      <div class="grid">
        <div><label class="f">Nome scheda / Ambiente</label>
          <input type="text" id="cfg-device-name" maxlength="39" placeholder="es. Soggiorno, Cantina, Camera da letto">
          <div class="sec-hint">Nome univoco per identificare questo dispositivo in casa</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Nome Sensore Ambientale</h2>
      <div class="grid">
        <div><label class="f">SHT31-D I²C (Temperatura + Umidità)</label><input type="text" id="cfg-s1" maxlength="39"></div>
      </div>
    </div>

    <div class="card">
      <h2>WiFi — Rete Casa (Modalità Station)</h2>
      <div class="sec-hint" style="margin-bottom:12px">Credenziali per la connessione alla rete WiFi domestica. Dopo la modifica è necessario <b>riavviare il dispositivo</b>.</div>
      <div class="grid">
        <div>
          <label class="f">SSID Rete</label>
          <input type="text" id="cfg-wifi-ssid" maxlength="32" placeholder="es. MiaWiFi_2.4GHz">
          <div class="sec-hint">Nome della rete WiFi (max 32 caratteri)</div>
        </div>
        <div>
          <label class="f">Password Rete</label>
          <input type="password" id="cfg-wifi-pass" maxlength="63" placeholder="lascia vuoto se rete aperta">
          <div class="sec-hint">Password WPA2 (max 63 caratteri). Lasciare vuoto solo per reti aperte.</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>WiFi — Access Point (Modalità Hotspot)</h2>
      <div class="sec-hint" style="margin-bottom:12px">Credenziali per la rete Access Point creata dal dispositivo. Usata per configurazione iniziale o fallback se la rete casa non è disponibile. Dopo modifica: <b>riavvio necessario</b>.</div>
      <div class="grid">
        <div>
          <label class="f">SSID Access Point</label>
          <input type="text" id="cfg-ap-ssid" maxlength="32" placeholder="es. TempHumid-Node">
          <div class="sec-hint">Nome dell'AP emesso dal dispositivo (max 32 caratteri). Default IP: 192.168.4.1</div>
        </div>
        <div>
          <label class="f">Password Access Point</label>
          <input type="password" id="cfg-ap-pass" maxlength="63" placeholder="min 8 char oppure vuoto per AP aperto">
          <div class="sec-hint">Password WPA2 min 8 caratteri, oppure lascia vuoto per Access Point aperto (non protetto).</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Connettività MQTT</h2>
      <div class="grid">
        <div><label class="f">Server / IP Broker</label><input type="text" id="cfg-mqtt-srv" maxlength="63"></div>
        <div><label class="f">Porta</label><input type="number" id="cfg-mqtt-port" min="1" max="65535" value="1883"></div>
        <div><label class="f">Username</label><input type="text" id="cfg-mqtt-user" maxlength="47"></div>
        <div><label class="f">Password</label><input type="password" id="cfg-mqtt-pass" maxlength="63"></div>
        <div style="grid-column: 1 / -1;">
          <label class="f">MQTT Device ID (univoco per ciascun dispositivo)</label>
          <input type="text" id="cfg-mqtt-devid" maxlength="63">
          <div class="sec-hint">Usato come parte dei topic MQTT. Es. "sensori_soggiorno", "sensori_cantina"</div>
          <div class="info-block" id="mqtt-topic-preview"></div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Ottimizzazione Traffico MQTT</h2>
      <div class="sec-hint" style="margin-bottom:12px">Parametri di throttling e dead-band per ridurre il traffico di rete. Applicati a runtime senza riavvio.</div>
      <h3>⏱ Frequenze</h3>
      <div class="grid">
        <div>
          <label class="f">Periodo base scan dati (ms)</label>
          <input type="number" id="cfg-throttle-timer-base" min="500" max="60000" step="500">
          <div class="sec-hint">TIMER B: frequenza base di scan. Range: 500-60000 ms. Default: 5000 ms.</div>
        </div>
        <div>
          <label class="f">Safety heartbeat publish tutti (ms)</label>
          <input type="number" id="cfg-throttle-safety" min="5000" max="300000" step="5000">
          <div class="sec-hint">Forza publish TUTTI i dati anche senza cambiamenti. Range 5-300 s. Default 60000 ms.</div>
        </div>
        <div>
          <label class="f">Forza supv delta/last ogni (ms)</label>
          <input type="number" id="cfg-throttle-supv" min="2000" max="60000" step="1000">
          <div class="sec-hint">Forza publish supervisore last_good/trip. Range: 2-60 s. Default 10000 ms.</div>
        </div>
      </div>
      <h3>📏 Soglie minime cambiamento (dead-band)</h3>
      <div class="grid">
        <div>
          <label class="f">Temperatura (°C)</label>
          <input type="number" id="cfg-thresh-t" min="0.01" max="2.00" step="0.01">
          <div class="sec-hint">Range: 0.01 - 2.00 °C. Default: 0.10 °C.</div>
        </div>
        <div>
          <label class="f">Umidità Relativa (%)</label>
          <input type="number" id="cfg-thresh-rh" min="0.1" max="10.0" step="0.1">
          <div class="sec-hint">Range: 0.1 - 10.0 %. Default: 0.5 %.</div>
        </div>
        <div>
          <label class="f">Dew Point (°C)</label>
          <input type="number" id="cfg-thresh-dew" min="0.01" max="2.00" step="0.01">
          <div class="sec-hint">Range: 0.01 - 2.00 °C. Default: 0.10 °C.</div>
        </div>
        <div>
          <label class="f">Heat Index (°C)</label>
          <input type="number" id="cfg-thresh-hi" min="0.01" max="2.00" step="0.01">
          <div class="sec-hint">Range: 0.01 - 2.00 °C. Default: 0.10 °C.</div>
        </div>
        <div>
          <label class="f">Umidità Assoluta (g/m³)</label>
          <input type="number" id="cfg-thresh-ah" min="0.01" max="1.00" step="0.01">
          <div class="sec-hint">Range: 0.01 - 1.00 g/m³. Default: 0.05 g/m³.</div>
        </div>
        <div>
          <label class="f">RSSI WiFi (dBm)</label>
          <input type="number" id="cfg-thresh-rssi" min="1" max="10" step="1">
          <div class="sec-hint">Range: 1 - 10 dBm (differenza minima). Default: 2 dBm.</div>
        </div>
        <div>
          <label class="f">Heap Fragmentation (%)</label>
          <input type="number" id="cfg-thresh-frag" min="0.5" max="20.0" step="0.5">
          <div class="sec-hint">Range: 0.5 - 20.0 %. Default: 1.0 %.</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Azioni</h2>
      <div class="actions">
        <button class="btn primary" id="btn-save">💾 Salva Configurazione</button>
        <button class="btn ghost" id="btn-load">↺ Ricarica valori correnti</button>
        <button class="btn danger" id="btn-reset">⚠ Ripristina default</button>
        <button class="btn ghost" id="btn-restart">🔄 Riavvia dispositivo</button>
      </div>
      <div class="sec-hint" style="margin-top:10px">⚠ Dopo il salvataggio MQTT, la riconnessione avviene automaticamente. Per rendere effettive le modifiche ai nomi, ricaricare la pagina.</div>
    </div>
  </div>

  <div id="view-fw" class="view">
    <div class="card">
      <h2>📋 Info Dispositivo</h2>
      <table class="dati" id="fw-tab" style="box-shadow:none;border:1px solid var(--border);margin-bottom:0">
        <thead>
          <tr>
            <th style="width:38%">Proprietà</th>
            <th class="right" style="width:34%">Valore</th>
            <th style="width:28%">Note</th>
          </tr>
        </thead>
        <tbody>
          <tr><td class="k">Versione Firmware</td><td class="v" id="fw-ver" style="text-align:right">...</td><td class="note"><span class="u">FW_VERSION</span></td></tr>
          <tr><td class="k">Build / Compilazione</td><td class="v" id="fw-build" style="text-align:right">...</td><td class="note"><span class="u">__DATE__ __TIME__</span></td></tr>
          <tr><td class="k">Chip ID</td><td class="v" id="fw-chip" style="text-align:right">...</td><td class="note"><span class="u">ESP.getChipId()</span></td></tr>
          <tr><td class="k">Flash Size</td><td class="v" id="fw-flash" style="text-align:right">...</td><td class="note"><span class="u">totale / reale</span></td></tr>
          <tr><td class="k">Max Sketch Size</td><td class="v" id="fw-maxsk" style="text-align:right">...</td><td class="note"><span class="u">OTA max bytes</span></td></tr>
          <tr><td class="k">Free Heap</td><td class="v" id="fw-heap" style="text-align:right">...</td><td class="note"><span class="u">min 16 KB richiesti</span></td></tr>
          <tr><td class="k">Max Free Block</td><td class="v" id="fw-mblk" style="text-align:right">...</td><td class="note"><span class="u">contiguo</span></td></tr>
          <tr><td class="k">Heap Fragm.</td><td class="v" id="fw-frag" style="text-align:right">...</td><td class="note"><span class="u">%</span></td></tr>
          <tr><td class="k">SDK Core Version</td><td class="v" id="fw-sdk" style="text-align:right">...</td><td class="note"><span class="u">ESP8266 Arduino</span></td></tr>
          <tr><td class="k">Ultimo Reset Reason</td><td class="v" id="fw-rst" style="text-align:right">...</td><td class="note"><span class="u">RTC</span></td></tr>
        </tbody>
      </table>
      <div class="sec-hint" style="margin-top:10px">Aggiornato automaticamente ogni 5 secondi. Free Heap troppo basso? Attendi un minuto o riavvia prima di aggiornare.</div>
    </div>

    <div class="card">
      <h2>⬆ Aggiornamento Firmware (OTA)</h2>
      <div class="warn-box">
        <b>⚠ OPERAZIONE CRITICA — LEGGERE ATTENTAMENTE</b><br>
        • Caricare solo file <b>.bin</b> generati da PlatformIO per <b>Wemos D1 Mini (ESP8266, 4MB flash)</b>.<br>
        • <b>NON</b> chiudere la pagina, <b>NON</b> spegnere o disconnettere l'alimentazione durante l'aggiornamento.<br>
        • Se l'aggiornamento fallisce, sarà necessario ripristinare via <b>USB seriale</b> (cavo).<br>
        • L'aggiornamento sovrascrive <b>solo il firmware</b>; configurazione EEPROM e parametri salvati <b>non vengono toccati</b>.
      </div>
      <div class="grid" style="margin-bottom:12px">
        <div>
          <label class="f">File Firmware (.bin)</label>
          <div class="file-row">
            <input type="file" id="fw-file" accept=".bin,application/octet-stream">
            <span class="file-meta" id="fw-meta">nessun file selezionato</span>
          </div>
          <div class="sec-hint">Dimensione massima: 1.000.000 byte (1 MB). File con magic 0xE9 non valido verranno rifiutati dal dispositivo.</div>
        </div>
      </div>
      <div class="progress-wrap" id="fw-pwrap" style="display:none">
        <progress id="fw-prog" value="0" max="100"></progress>
        <div class="p-text" id="fw-ptext">0 %</div>
      </div>
      <div class="actions" style="margin-top:14px">
        <button class="btn primary" id="btn-fw-upload" disabled>🚀 Aggiorna Firmware</button>
        <button class="btn ghost" id="btn-fw-refresh">↻ Aggiorna info</button>
      </div>
      <div class="sec-hint" style="margin-top:10px" id="fw-hint">Seleziona un file .bin per abilitare il pulsante.</div>
      <div class="fw-log" id="fw-log"></div>
    </div>
  </div>

</div>

<div id="toast" class="toast"></div>

<script>
(function(){
  "use strict";

  // --- Gestione tab ---
  const tabs = document.querySelectorAll(".tab");
  const views = document.querySelectorAll(".view");
  tabs.forEach(t=>t.addEventListener("click",()=>{
    tabs.forEach(x=>x.classList.remove("active"));
    views.forEach(x=>x.classList.remove("active"));
    t.classList.add("active");
    document.getElementById(t.dataset.view).classList.add("active");
  }));

  // --- Toast ---
  const toastEl = document.getElementById("toast");
  let toastT=null;
  function toast(msg,type){
    toastEl.className="toast "+(type||"");
    toastEl.textContent=msg;
    requestAnimationFrame(()=>toastEl.classList.add("show"));
    if(toastT)clearTimeout(toastT);
    toastT=setTimeout(()=>toastEl.classList.remove("show"),2800);
  }

  // --- Config API ---
  async function apiGetConfig(){
    const r=await fetch("/api/config",{cache:"no-store"});
    if(!r.ok)throw new Error("GET config HTTP "+r.status);
    return r.json();
  }
  async function apiSetConfig(payload){
    const r=await fetch("/api/config",{
      method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify(payload)
    });
    const txt=await r.text();
    let ok=r.ok, msg=txt||"";
    try{ const j=JSON.parse(txt); if(j.ok!==undefined)ok=!!j.ok; if(j.message)msg=j.message; }catch(_){}
    return {ok,msg};
  }
  async function apiRestart(){
    const r=await fetch("/api/restart",{method:"POST"});
    return r.ok;
  }
  async function apiLabels(){
    try{
      const r=await fetch("/api/labels",{cache:"no-store"});
      if(r.ok) return r.json();
    }catch(_){}
    return null;
  }

  // --- Valori di default (allineati al firmware C) ---
  const CFG_DEFAULTS={
    device_name:"Sensori Temp/Umid",
    sensor_sht31_name:"Ambiente_SHT31",
    wifi_ssid:"YOUR_WIFI_SSID",
    wifi_password:"YOUR_WIFI_PASSWORD",
    ap_ssid:"TempHumid-Node",
    ap_password:"CHANGE_ME_AP_PASS",
    mqtt_server:"192.168.1.100",
    mqtt_port:1883,
    mqtt_user:"mqtt_user",
    mqtt_password:"mqtt_password",
    mqtt_device_id:"sensori_temperatura_umidita",
    throttle_timer_data_base_ms:5000,
    throttle_safety_max_interval_ms:60000,
    throttle_supv_force_interval_ms:10000,
    throttle_thresh_temp_c:0.10,
    throttle_thresh_rh_pct:0.5,
    throttle_thresh_dew_c:0.10,
    throttle_thresh_hi_c:0.10,
    throttle_thresh_ah_gm3:0.05,
    throttle_thresh_rssi_dbm:2,
    throttle_thresh_frag_pct:1.0
  };
  const DEFAULT_LABELS={
    device_name:CFG_DEFAULTS.device_name,
    sht31:CFG_DEFAULTS.sensor_sht31_name
  };
  let LABELS=Object.assign({},DEFAULT_LABELS);

  function applyLabels(){
    document.getElementById("hdr-name").textContent=LABELS.device_name||DEFAULT_LABELS.device_name;
    document.title="Sensori T/U · "+(LABELS.device_name||"NodeMCU");
    ROWS[0][2]="SHT31-D · "+(LABELS.sht31||DEFAULT_LABELS.sht31)+"  (I²C @0x44 · HIGH accuracy)";
    rebuildRows();
  }

  // --- Definizione righe tabella dashboard ---
  const ROWS = [
    ["cat-sht31","cat","SHT31-D · Ambiente_SHT31  (I²C @0x44 · HIGH accuracy)"],
    ["sht-t","row","Temperatura Ambiente","°C","SHT31 ±0.2°C · 16ms conv"],
    ["sht-rh","row","Umidità Relativa","%","SHT31 ±2%RH · 0-100%"],
    ["sht-dp","row","Punto di Rugiada","°C","Magnus-Tetens · T,RH derivato"],
    ["sht-hi","row","Indice di Calore","°C","NOAA Rothfusz · HI T,RH"],
    ["sht-ah","row","Umidità Assoluta","g/m³","Buck vapour pressure"],
    ["sht-heat","row","Heater integrato","","Anti condensa · auto RH>95% 30s"],
    ["cat-sys","cat","STATO SISTEMA"],
    ["sys-ws","row","Connessione WS","","stato socket"],
    ["sys-hz","row","Frequenza pacchetti","Hz","RX rate lato client"],
    ["sys-age","row","Età ultimo dato","ms","interarrivo stimato"],
    ["sys-ts","row","Timestamp locale","","data/ora PC"],
    ["sys-flag","row","Stato sensori","","flags T:RH:CRC:I2C:HEAT:ALERT"],
    ["sys-sensors","row","Sensore collegato","","bitmask 6-bit diagnostica"]
  ];

  const tbody = document.getElementById("rows");
  const dot   = document.getElementById("dot");
  const stat  = document.getElementById("stat");
  const fHost = document.getElementById("f-host");
  const fTs   = document.getElementById("f-ts");
  const ref = {};
  function rebuildRows(){
    tbody.innerHTML=""; ref.length=0;
    Object.keys(ref).forEach(k=>delete ref[k]);
    for(const r of ROWS){
      const tr = document.createElement("tr");
      if(r[1]==="cat"){
        tr.classList.add("cat");
        const td = document.createElement("td");
        td.colSpan = 3;
        td.textContent = r[2];
        tr.appendChild(td);
      } else {
        const a = document.createElement("td"); a.className="k"; a.textContent=r[2];
        const b = document.createElement("td"); b.className="v"; b.textContent="—"; b.id="v-"+r[0]; b.style.textAlign="right";
        const c = document.createElement("td");
        if(r[3]){ const u = document.createElement("span"); u.className="u"; u.textContent=r[3]; c.appendChild(u); }
        if(r[4]){ const n = document.createElement("div"); n.className="note"; n.textContent=r[4]; c.appendChild(n); }
        tr.appendChild(a); tr.appendChild(b); tr.appendChild(c);
        ref[r[0]] = b;
      }
      tbody.appendChild(tr);
    }
  }
  rebuildRows();

  function setv(id, val, cls){
    const el = ref[id]; if(!el) return;
    el.textContent = val;
    el.className = "v" + (cls ? " "+cls : "");
  }
  function setStatus(ok, text){
    dot.className = "dot" + (ok ? " ok" : "");
    stat.textContent = text;
  }
  function clamp(v,a,b){return Math.max(a,Math.min(b,v));}
  function sensorList(bits){
    const n1=(LABELS.sht31||"Ambiente_SHT31").substring(0,14);
    const tOk  = (bits & 0x01) !== 0;
    const rhOk = (bits & 0x02) !== 0;
    const crcOk= (bits & 0x04) !== 0;
    const i2cOk= (bits & 0x08) !== 0;
    const heat = (bits & 0x10) !== 0;
    const flag = (tOk?"T:OK":"T:NC")+" · "+(rhOk?"RH:OK":"RH:NC")+" · "+
                 (crcOk?"CRC:OK":"CRC:ERR")+" · "+(i2cOk?"I2C:OK":"I2C:NC")+
                 (heat?" · HEAT":"");
    return n1+"  ["+flag+"]";
  }

  let lastTs = 0, fpsCount = 0, fpsLastReport = performance.now(), fpsValue = 0;
  let lastAge = 0;
  const EMA_A = 0.15;
  let sT=null, sRH=null, sDP=null, sHI=null, sAH=null;
  function ema(prev,val,alpha){ if(prev===null) return val; return prev + alpha*(val-prev); }

  function renderAll(r){
    const host = (location.hostname||"192.168.4.1")+":"+81;
    fHost.textContent = "ws://"+host;
    const now = new Date();
    fTs.textContent = "dati ricevuti "+now.toLocaleTimeString("it-IT")+"."+String(now.getMilliseconds()).padStart(3,"0");

    const sb = Number.isFinite(r.status) ? Math.round(r.status) : 0;
    const tOk  = (sb & 0x01) !== 0;
    const rhOk = (sb & 0x02) !== 0;
    const i2cOk= (sb & 0x08) !== 0;
    const allData = tOk && rhOk && i2cOk;

    if(tOk && Number.isFinite(r.tC))  sT  = ema(sT,  r.tC,  EMA_A);
    setv("sht-t",  tOk ? sT.toFixed(3)  : "NC", tOk ? "ok" : "bad");

    if(rhOk && Number.isFinite(r.rhP)) sRH = ema(sRH, r.rhP, EMA_A);
    setv("sht-rh", rhOk ? sRH.toFixed(2) : "NC", rhOk ? "ok" : "bad");

    if(allData && Number.isFinite(r.dpC)) sDP = ema(sDP, r.dpC, EMA_A);
    setv("sht-dp", Number.isFinite(sDP)  ? sDP.toFixed(2)  : "—");

    if(allData && Number.isFinite(r.hiC)) sHI = ema(sHI, r.hiC, EMA_A);
    setv("sht-hi", Number.isFinite(sHI)  ? sHI.toFixed(2)  : "—");

    if(allData && Number.isFinite(r.ahG)) sAH = ema(sAH, r.ahG, EMA_A);
    setv("sht-ah", Number.isFinite(sAH)  ? sAH.toFixed(3)  : "—", allData ? "" : "warn");

    const heatOn = Number.isFinite(r.heat) && Math.round(r.heat) !== 0;
    setv("sht-heat", heatOn ? "ATTIVO" : "spento", heatOn ? "warn" : "");

    setv("sys-hz", fpsValue + " Hz");
    setv("sys-age", lastAge + " ms");
    const n = new Date();
    setv("sys-ts", n.toLocaleString("it-IT")+"."+String(n.getMilliseconds()).padStart(3,"0"));
    setv("sys-flag", "0x"+sb.toString(16).toUpperCase().padStart(2,"0")+
      "  b"+(sb>>>0).toString(2).padStart(6,"0"));
    setv("sys-sensors", sensorList(sb));
    setv("sys-ws", "Attiva", "ok");
  }

  let ws=null, retryTimer=null, reconDelay=1500;
  let wsSilenceTimer=null, wsConsecFails=0;
  const WS_SILENCE_MS = 30000;
  const WS_BACKOFF_MIN = 1500;
  const WS_BACKOFF_MAX = 15000;

  function wsSilenceStart(){
    if(wsSilenceTimer) clearTimeout(wsSilenceTimer);
    wsSilenceTimer = setTimeout(()=>{
      console.warn("WS: silenzio >"+(WS_SILENCE_MS/1000)+"s -> close for reconnect");
      setStatus(false,"silenzio dati -> riconnetto");
      setv("sys-ws","STALLATA","bad");
      try{ ws && ws.close(); }catch(_){}
    }, WS_SILENCE_MS);
  }
  function wsSilenceStop(){
    if(wsSilenceTimer){ clearTimeout(wsSilenceTimer); wsSilenceTimer=null; }
  }

  function onMessage(ev){
    wsSilenceStart();
    let tC=NaN,rhP=NaN,dpC=NaN,hiC=NaN,ahG=NaN,st=NaN,heat=NaN;
    if(typeof ev.data === "string"){
      const parts = String(ev.data).trim().split(/[,\s]+/);
      tC   = parts[0] ? parseFloat(parts[0]) : NaN;
      rhP  = parts[1] ? parseFloat(parts[1]) : NaN;
      dpC  = parts[2] ? parseFloat(parts[2]) : NaN;
      hiC  = parts[3] ? parseFloat(parts[3]) : NaN;
      ahG  = parts[4] ? parseFloat(parts[4]) : NaN;
      st   = parts[5] ? parseFloat(parts[5]) : NaN;
      heat = parts[6] ? parseFloat(parts[6]) : NaN;
    } else if(ev.data instanceof ArrayBuffer){
      const v = new DataView(ev.data);
      if(v.byteLength>=4)  tC   = v.getFloat32(0,true);
      if(v.byteLength>=8)  rhP  = v.getFloat32(4,true);
      if(v.byteLength>=12) dpC  = v.getFloat32(8,true);
      if(v.byteLength>=16) hiC  = v.getFloat32(12,true);
      if(v.byteLength>=20) ahG  = v.getFloat32(16,true);
      if(v.byteLength>=24) st   = v.getFloat32(20,true);
      if(v.byteLength>=28) heat = v.getFloat32(24,true);
    }
    const now = performance.now();
    if(lastTs>0) lastAge = Math.round(now - lastTs);
    lastTs = now; fpsCount++;
    if(now - fpsLastReport >= 900){
      fpsValue = Math.round((fpsCount*1000)/(now - fpsLastReport));
      fpsCount = 0; fpsLastReport = now;
    }
    renderAll({tC,rhP,dpC,hiC,ahG,status:st,heat:heat});
  }
  function reconnect(){
    if(retryTimer) return;
    const delay = reconDelay;
    wsConsecFails++;
    reconDelay = Math.min(WS_BACKOFF_MAX, reconDelay * 2 + ((Math.random()*300)|0));
    retryTimer = setTimeout(()=>{ retryTimer=null; connect(); }, delay);
  }
  function connect(){
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const host = location.hostname || "192.168.4.1";
    const port = (location.port && location.port!=="80" && location.port!=="443")
      ? (parseInt(location.port,10)) : 81;
    const url = proto+"://"+host+":81/";
    setStatus(false, "connessione a "+host+":81 ... (tentativo "+(wsConsecFails+1)+", backoff "+reconDelay+"ms)");
    setv("sys-ws","In attesa","warn");
    try{
      if(ws){ try{ ws.onopen=ws.onclose=ws.onerror=ws.onmessage=null; ws.close(); }catch(_){} ws=null; }
      ws = new WebSocket(url);
    }catch(e){ setStatus(false,"URL non valida: "+e.message); reconnect(); return; }
    ws.binaryType = "arraybuffer";
    ws.onopen = ()=>{
      wsConsecFails = 0;
      reconDelay = WS_BACKOFF_MIN;
      setStatus(true,"connesso "+host+":81");
      setv("sys-ws","Attiva","ok");
      wsSilenceStart();
    };
    ws.onclose = (e)=>{
      wsSilenceStop();
      let reason="";
      if(e && typeof e.code!=="undefined") reason = " (code="+e.code+")";
      setStatus(false,"disconnesso"+reason+" - riconnessione...");
      setv("sys-ws","Interrotta","bad");
      reconnect();
    };
    ws.onerror = ()=>{
      try{ ws && ws.close(); }catch(_){}
    };
    ws.onmessage = onMessage;
  }
  connect();

  // --- Form configurazione ---
  const fIds={
    dn:"cfg-device-name",
    s1:"cfg-s1",
    wss:"cfg-wifi-ssid", wsp:"cfg-wifi-pass",
    aps:"cfg-ap-ssid",   app:"cfg-ap-pass",
    ms:"cfg-mqtt-srv", mp:"cfg-mqtt-port",
    mu:"cfg-mqtt-user", mpa:"cfg-mqtt-pass",
    mdi:"cfg-mqtt-devid",
    tth:"cfg-throttle-timer-base",
    tsf:"cfg-throttle-safety",
    tsp:"cfg-throttle-supv",
    tht:"cfg-thresh-t",
    thr:"cfg-thresh-rh",
    thd:"cfg-thresh-dew",
    thh:"cfg-thresh-hi",
    tha:"cfg-thresh-ah",
    thrssi:"cfg-thresh-rssi",
    thf:"cfg-thresh-frag"
  };
  function $(id){return document.getElementById(id);}

  function updateTopicPreview(){
    const devid=$(fIds.mdi).value.trim() || "sensori_xxx";
    const srv=$(fIds.ms).value.trim() || "broker";
    const port=$(fIds.mp).value || "1883";
    const preview=
      "Broker: "+srv+":"+port+"\n"+
      "Availability: homeassistant/sensor/"+devid+"/availability\n"+
      "Dati:  homeassistant/sensor/"+devid+"/sht31_<grandezza>/state\n"+
      "Config: homeassistant/sensor/"+devid+"/sht31_<grandezza>/config";
    $("mqtt-topic-preview").textContent=preview;
  }
  [fIds.ms,fIds.mp,fIds.mdi].forEach(id=>$(id).addEventListener("input",updateTopicPreview));

  function fillForm(cfg){
    $(fIds.dn).value  = cfg.device_name         || CFG_DEFAULTS.device_name;
    $(fIds.s1).value  = cfg.sensor_sht31_name   || CFG_DEFAULTS.sensor_sht31_name;
    $(fIds.wss).value = cfg.wifi_ssid           || CFG_DEFAULTS.wifi_ssid;
    $(fIds.wsp).value = cfg.wifi_password       || CFG_DEFAULTS.wifi_password;
    $(fIds.aps).value = cfg.ap_ssid             || CFG_DEFAULTS.ap_ssid;
    $(fIds.app).value = cfg.ap_password         || CFG_DEFAULTS.ap_password;
    $(fIds.ms).value  = cfg.mqtt_server         || CFG_DEFAULTS.mqtt_server;
    $(fIds.mp).value  = cfg.mqtt_port           || CFG_DEFAULTS.mqtt_port;
    $(fIds.mu).value  = cfg.mqtt_user           || CFG_DEFAULTS.mqtt_user;
    $(fIds.mpa).value = cfg.mqtt_password       || CFG_DEFAULTS.mqtt_password;
    $(fIds.mdi).value = cfg.mqtt_device_id      || CFG_DEFAULTS.mqtt_device_id;
    $(fIds.tth).value = cfg.throttle_timer_data_base_ms     ?? CFG_DEFAULTS.throttle_timer_data_base_ms;
    $(fIds.tsf).value = cfg.throttle_safety_max_interval_ms ?? CFG_DEFAULTS.throttle_safety_max_interval_ms;
    $(fIds.tsp).value = cfg.throttle_supv_force_interval_ms ?? CFG_DEFAULTS.throttle_supv_force_interval_ms;
    $(fIds.tht).value = cfg.throttle_thresh_temp_c          ?? CFG_DEFAULTS.throttle_thresh_temp_c;
    $(fIds.thr).value = cfg.throttle_thresh_rh_pct          ?? CFG_DEFAULTS.throttle_thresh_rh_pct;
    $(fIds.thd).value = cfg.throttle_thresh_dew_c           ?? CFG_DEFAULTS.throttle_thresh_dew_c;
    $(fIds.thh).value = cfg.throttle_thresh_hi_c            ?? CFG_DEFAULTS.throttle_thresh_hi_c;
    $(fIds.tha).value = cfg.throttle_thresh_ah_gm3          ?? CFG_DEFAULTS.throttle_thresh_ah_gm3;
    $(fIds.thrssi).value = cfg.throttle_thresh_rssi_dbm     ?? CFG_DEFAULTS.throttle_thresh_rssi_dbm;
    $(fIds.thf).value = cfg.throttle_thresh_frag_pct        ?? CFG_DEFAULTS.throttle_thresh_frag_pct;
    updateTopicPreview();
  }

  function readForm(){
    return {
      device_name: $(fIds.dn).value.trim(),
      sensor_sht31_name: $(fIds.s1).value.trim(),
      wifi_ssid: $(fIds.wss).value.trim(),
      wifi_password: $(fIds.wsp).value,
      ap_ssid: $(fIds.aps).value.trim(),
      ap_password: $(fIds.app).value,
      mqtt_server: $(fIds.ms).value.trim(),
      mqtt_port: parseInt($(fIds.mp).value,10) || 1883,
      mqtt_user: $(fIds.mu).value,
      mqtt_password: $(fIds.mpa).value,
      mqtt_device_id: $(fIds.mdi).value.trim(),
      throttle_timer_data_base_ms: parseInt($(fIds.tth).value,10) || CFG_DEFAULTS.throttle_timer_data_base_ms,
      throttle_safety_max_interval_ms: parseInt($(fIds.tsf).value,10) || CFG_DEFAULTS.throttle_safety_max_interval_ms,
      throttle_supv_force_interval_ms: parseInt($(fIds.tsp).value,10) || CFG_DEFAULTS.throttle_supv_force_interval_ms,
      throttle_thresh_temp_c: parseFloat($(fIds.tht).value),
      throttle_thresh_rh_pct: parseFloat($(fIds.thr).value),
      throttle_thresh_dew_c: parseFloat($(fIds.thd).value),
      throttle_thresh_hi_c: parseFloat($(fIds.thh).value),
      throttle_thresh_ah_gm3: parseFloat($(fIds.tha).value),
      throttle_thresh_rssi_dbm: parseInt($(fIds.thrssi).value,10) || CFG_DEFAULTS.throttle_thresh_rssi_dbm,
      throttle_thresh_frag_pct: parseFloat($(fIds.thf).value)
    };
  }

  async function loadAndApplyAll(){
    try{
      const [cfg, labels] = await Promise.all([apiGetConfig(), apiLabels()]);
      fillForm(cfg);
      if(labels){
        LABELS.device_name = labels.device_name || DEFAULT_LABELS.device_name;
        LABELS.sht31       = labels.sht31       || DEFAULT_LABELS.sht31;
      } else {
        LABELS.device_name = cfg.device_name       || CFG_DEFAULTS.device_name;
        LABELS.sht31       = cfg.sensor_sht31_name || CFG_DEFAULTS.sensor_sht31_name;
      }
      applyLabels();
    }catch(e){
      toast("Errore caricamento config: "+e.message,"err");
    }
  }

  $("btn-save").addEventListener("click",async ()=>{
    const b=$("btn-save"); b.disabled=true;
    const orig=b.textContent; b.textContent="Salvataggio...";
    try{
      const payload=readForm();
      if(!payload.device_name) throw new Error("Nome scheda obbligatorio");
      // --- Validazioni WiFi ---
      if(!payload.wifi_ssid) throw new Error("SSID Rete Casa obbligatorio");
      if(payload.wifi_ssid.length>32) throw new Error("SSID Rete Casa troppo lungo (max 32)");
      if(payload.wifi_password.length>63) throw new Error("Password Rete Casa troppo lunga (max 63)");
      if(!payload.ap_ssid) throw new Error("SSID Access Point obbligatorio");
      if(payload.ap_ssid.length>32) throw new Error("SSID Access Point troppo lungo (max 32)");
      if(payload.ap_password.length>0 && payload.ap_password.length<8) throw new Error("Password Access Point: min 8 caratteri WPA2 oppure lasciare vuoto per AP aperto");
      if(payload.ap_password.length>63) throw new Error("Password Access Point troppo lunga (max 63)");
      // --- Fine Validazioni WiFi ---
      if(!payload.mqtt_server) throw new Error("Server MQTT obbligatorio");
      if(!payload.mqtt_device_id) throw new Error("Device ID MQTT obbligatorio");
      if(!/^[a-zA-Z0-9_\-]+$/.test(payload.mqtt_device_id)) throw new Error("Device ID: solo lettere, numeri, _ e -");
      if(payload.mqtt_port<1||payload.mqtt_port>65535) throw new Error("Porta MQTT non valida (1-65535)");
      // --- Validazioni HARD-BLOCK throttling range (allineate a safety ranges firmware C + HTML min/max) ---
      if(!Number.isFinite(payload.throttle_timer_data_base_ms)||payload.throttle_timer_data_base_ms<500||payload.throttle_timer_data_base_ms>60000)
        throw new Error("Throttling: Timer base dati fuori range (500 - 60000 ms)");
      if(!Number.isFinite(payload.throttle_safety_max_interval_ms)||payload.throttle_safety_max_interval_ms<5000||payload.throttle_safety_max_interval_ms>300000)
        throw new Error("Throttling: Safety Max Interval fuori range (5000 - 300000 ms)");
      if(!Number.isFinite(payload.throttle_supv_force_interval_ms)||payload.throttle_supv_force_interval_ms<2000||payload.throttle_supv_force_interval_ms>60000)
        throw new Error("Throttling: Forza Supervisore fuori range (2000 - 60000 ms)");
      if(!Number.isFinite(payload.throttle_thresh_temp_c)||payload.throttle_thresh_temp_c<0.01||payload.throttle_thresh_temp_c>2.0)
        throw new Error("Soglia Temperatura fuori range (0.01 - 2.00 °C)");
      if(!Number.isFinite(payload.throttle_thresh_rh_pct)||payload.throttle_thresh_rh_pct<0.1||payload.throttle_thresh_rh_pct>10.0)
        throw new Error("Soglia Umidità fuori range (0.1 - 10.0 %)");
      if(!Number.isFinite(payload.throttle_thresh_dew_c)||payload.throttle_thresh_dew_c<0.01||payload.throttle_thresh_dew_c>2.0)
        throw new Error("Soglia Punto Rugiada fuori range (0.01 - 2.00 °C)");
      if(!Number.isFinite(payload.throttle_thresh_hi_c)||payload.throttle_thresh_hi_c<0.01||payload.throttle_thresh_hi_c>2.0)
        throw new Error("Soglia Indice Calore fuori range (0.01 - 2.00 °C)");
      if(!Number.isFinite(payload.throttle_thresh_ah_gm3)||payload.throttle_thresh_ah_gm3<0.01||payload.throttle_thresh_ah_gm3>1.0)
        throw new Error("Soglia Umidità Assoluta fuori range (0.01 - 1.00 g/m³)");
      if(!Number.isFinite(payload.throttle_thresh_rssi_dbm)||payload.throttle_thresh_rssi_dbm<1||payload.throttle_thresh_rssi_dbm>10)
        throw new Error("Soglia RSSI fuori range (1 - 10 dBm)");
      if(!Number.isFinite(payload.throttle_thresh_frag_pct)||payload.throttle_thresh_frag_pct<0.5||payload.throttle_thresh_frag_pct>20.0)
        throw new Error("Soglia Heap Frag fuori range (0.5 - 20.0 %)");
      const r=await apiSetConfig(payload);
      if(r.ok){
        toast("Configurazione salvata! "+(r.msg||""),"ok");
        setTimeout(loadAndApplyAll, 300);
      } else {
        toast("Salvataggio fallito: "+(r.msg||"errore sconosciuto"),"err");
      }
    }catch(e){
      toast(e.message,"warn");
    }finally{
      b.disabled=false; b.textContent=orig;
    }
  });

  $("btn-load").addEventListener("click",()=>{
    loadAndApplyAll(); toast("Ricaricato","ok");
  });

  $("btn-reset").addEventListener("click",async ()=>{
    if(!confirm("Ripristinare TUTTI i valori di default?\n(Nome, sensore, WiFi, MQTT, Ottimizzazione Traffico)")) return;
    const payload={
      device_name:""+DEVICE_NAME_DEFAULT,
      sensor_sht31_name:""+SENSOR_SHT31_DEFAULT,
      wifi_ssid:""+WIFI_SSID_DEFAULT,
      wifi_password:""+WIFI_PASS_DEFAULT,
      ap_ssid:""+AP_SSID_DEFAULT,
      ap_password:""+AP_PASS_DEFAULT,
      mqtt_server:""+MQTT_SRV_DEFAULT,
      mqtt_port:MQTT_PORT_DEFAULT,
      mqtt_user:""+MQTT_USER_DEFAULT,
      mqtt_password:""+MQTT_PASS_DEFAULT,
      mqtt_device_id:""+MQTT_DEVID_DEFAULT,
      throttle_timer_data_base_ms:THROTTLE_TIMER_BASE_DEFAULT,
      throttle_safety_max_interval_ms:THROTTLE_SAFETY_DEFAULT,
      throttle_supv_force_interval_ms:THROTTLE_SUPV_DEFAULT,
      throttle_thresh_temp_c:THROTTLE_THRESH_T_DEFAULT,
      throttle_thresh_rh_pct:THROTTLE_THRESH_RH_DEFAULT,
      throttle_thresh_dew_c:THROTTLE_THRESH_DEW_DEFAULT,
      throttle_thresh_hi_c:THROTTLE_THRESH_HI_DEFAULT,
      throttle_thresh_ah_gm3:THROTTLE_THRESH_AH_DEFAULT,
      throttle_thresh_rssi_dbm:THROTTLE_THRESH_RSSI_DEFAULT,
      throttle_thresh_frag_pct:THROTTLE_THRESH_FRAG_DEFAULT
    };
    const r=await apiSetConfig(payload);
    if(r.ok){ toast("Default ripristinati","ok"); loadAndApplyAll(); }
    else toast("Errore: "+(r.msg||""),"err");
  });

  $("btn-restart").addEventListener("click",async ()=>{
    if(!confirm("Riavviare il dispositivo ora?")) return;
    try{
      await apiRestart();
      toast("Riavvio in corso... attendere","warn");
      setTimeout(()=>location.reload(), 4500);
    }catch(e){ toast("Errore riavvio","err"); }
  });

  // --- Alias costanti fallback per reset default (derivati da CFG_DEFAULTS) ---
  const DEVICE_NAME_DEFAULT   = CFG_DEFAULTS.device_name;
  const SENSOR_SHT31_DEFAULT  = CFG_DEFAULTS.sensor_sht31_name;
  const WIFI_SSID_DEFAULT     = CFG_DEFAULTS.wifi_ssid;
  const WIFI_PASS_DEFAULT     = CFG_DEFAULTS.wifi_password;
  const AP_SSID_DEFAULT       = CFG_DEFAULTS.ap_ssid;
  const AP_PASS_DEFAULT       = CFG_DEFAULTS.ap_password;
  const MQTT_SRV_DEFAULT      = CFG_DEFAULTS.mqtt_server;
  const MQTT_USER_DEFAULT     = CFG_DEFAULTS.mqtt_user;
  const MQTT_PASS_DEFAULT     = CFG_DEFAULTS.mqtt_password;
  const MQTT_DEVID_DEFAULT    = CFG_DEFAULTS.mqtt_device_id;
  const MQTT_PORT_DEFAULT     = CFG_DEFAULTS.mqtt_port;
  const THROTTLE_TIMER_BASE_DEFAULT  = CFG_DEFAULTS.throttle_timer_data_base_ms;
  const THROTTLE_SAFETY_DEFAULT      = CFG_DEFAULTS.throttle_safety_max_interval_ms;
  const THROTTLE_SUPV_DEFAULT        = CFG_DEFAULTS.throttle_supv_force_interval_ms;
  const THROTTLE_THRESH_T_DEFAULT    = CFG_DEFAULTS.throttle_thresh_temp_c;
  const THROTTLE_THRESH_RH_DEFAULT   = CFG_DEFAULTS.throttle_thresh_rh_pct;
  const THROTTLE_THRESH_DEW_DEFAULT  = CFG_DEFAULTS.throttle_thresh_dew_c;
  const THROTTLE_THRESH_HI_DEFAULT   = CFG_DEFAULTS.throttle_thresh_hi_c;
  const THROTTLE_THRESH_AH_DEFAULT   = CFG_DEFAULTS.throttle_thresh_ah_gm3;
  const THROTTLE_THRESH_RSSI_DEFAULT = CFG_DEFAULTS.throttle_thresh_rssi_dbm;
  const THROTTLE_THRESH_FRAG_DEFAULT = CFG_DEFAULTS.throttle_thresh_frag_pct;

  // ======================================================================
  // Tab 🔧 Firmware: aggiornamento OTA + info device
  // ======================================================================
  const OTA_MAX_BYTES = 1000000;
  const fwIds = {
    file:   "fw-file",
    meta:   "fw-meta",
    btn:    "btn-fw-upload",
    btnRf:  "btn-fw-refresh",
    prog:   "fw-prog",
    pwrap:  "fw-pwrap",
    ptext:  "fw-ptext",
    hint:   "fw-hint",
    log:    "fw-log",
    ver:    "fw-ver",
    build:  "fw-build",
    chip:   "fw-chip",
    flash:  "fw-flash",
    maxsk:  "fw-maxsk",
    heap:   "fw-heap",
    mblk:   "fw-mblk",
    frag:   "fw-frag",
    sdk:    "fw-sdk",
    rst:    "fw-rst"
  };
  function fw$(id){return document.getElementById(fwIds[id]);}

  function fwLog(msg, cls) {
    const log = fw$("log"); if (!log) return;
    const now = new Date();
    const ts = String(now.getHours()).padStart(2,"0")+":"+
               String(now.getMinutes()).padStart(2,"0")+":"+
               String(now.getSeconds()).padStart(2,"0")+"."+
               String(now.getMilliseconds()).padStart(3,"0");
    const li = document.createElement("div");
    const t = document.createElement("span"); t.className="l-time"; t.textContent = ts;
    const m = document.createElement("span");
    m.className = cls ? "l-"+cls : "";
    m.textContent = msg;
    li.appendChild(t); li.appendChild(m);
    log.appendChild(li);
    log.scrollTop = log.scrollHeight;
  }

  function fwSetProgress(cur, tot) {
    const pw = fw$("pwrap"); if (!pw) return;
    const p  = fw$("prog");
    const pt = fw$("ptext");
    if (tot > 0) {
      pw.style.display = "block";
      const pct = Math.round((cur / tot) * 100);
      if (p) { p.value = Math.max(0, Math.min(100, pct)); p.max = 100; }
      if (pt) pt.textContent = pct + " %  (" + cur.toLocaleString("it-IT") + " / " + tot.toLocaleString("it-IT") + " B)";
    } else {
      pw.style.display = "none";
      if (p) p.value = 0;
      if (pt) pt.textContent = "";
    }
  }

  function renderFwInfo(info) {
    if (!info) return;
    function setVal(id, v, cls) {
      const el = fw$(id);
      if (!el) return;
      el.textContent = v;
      if (cls) { el.className = "v " + cls; } else { el.className = "v"; }
    }
    setVal("ver",   info.fw_version || "—");
    setVal("build", info.fw_build || "—");
    setVal("chip",  info.chip_id || "—");
    if (info.flash_size != null) {
      setVal("flash", (info.flash_size/1048576).toFixed(2) + " MB / " + ((info.flash_real_size||0)/1048576).toFixed(2) + " MB reale");
    } else {
      setVal("flash", "—");
    }
    if (info.max_sketch_bytes != null) {
      setVal("maxsk", (info.max_sketch_bytes/1048576).toFixed(3) + " MB  (" + info.max_sketch_bytes.toLocaleString("it-IT") + " B)");
    }
    if (info.free_heap != null) {
      const kb = info.free_heap / 1024;
      let cls = "ok";
      if (info.free_heap < 32768) cls = "warn";
      if (info.free_heap < 24576) cls = "bad";
      setVal("heap", kb.toFixed(1) + " KB  (" + info.free_heap.toLocaleString("it-IT") + " B)", cls);
    }
    if (info.max_free_block != null) {
      const kb = info.max_free_block / 1024;
      let cls = "";
      if (info.max_free_block < 16384) cls = "warn";
      setVal("mblk", kb.toFixed(1) + " KB", cls);
    }
    if (info.heap_frag_pct != null) {
      let cls = "ok";
      if (info.heap_frag_pct >= 40) cls = "warn";
      if (info.heap_frag_pct >= 65) cls = "bad";
      setVal("frag", info.heap_frag_pct + " %", cls);
    }
    setVal("sdk",   info.sdk_version || "—");
    setVal("rst",   info.reset_reason || "—");
  }

  async function apiGetFwInfo() {
    try {
      const r = await fetch("/api/fw_info", {cache:"no-store"});
      if (!r.ok) throw new Error("HTTP " + r.status);
      return await r.json();
    } catch (e) {
      console.warn("get fw info failed:", e);
      return null;
    }
  }

  let FW_FILE = null;
  fw$("file").addEventListener("change", e => {
    const f = e.target.files && e.target.files[0];
    FW_FILE = f || null;
    const meta = fw$("meta");
    const btn  = fw$("btn");
    const hint = fw$("hint");
    if (!FW_FILE) {
      meta.textContent = "nessun file selezionato";
      btn.disabled = true;
      hint.textContent = "Seleziona un file .bin per abilitare il pulsante.";
      hint.style.color = "";
      return;
    }
    let bad = null;
    if (FW_FILE.size <= 0)           bad = "File vuoto!";
    else if (FW_FILE.size > OTA_MAX_BYTES) bad = "File troppo grande: " + FW_FILE.size + " > " + OTA_MAX_BYTES + " byte";
    meta.textContent = FW_FILE.name + " · " + FW_FILE.size.toLocaleString("it-IT") + " byte";
    if (bad) {
      btn.disabled = true;
      hint.textContent = "⚠ " + bad;
      hint.style.color = "var(--err)";
    } else {
      btn.disabled = false;
      hint.textContent = "File pronto. Clicca \"Aggiorna Firmware\" per iniziare (conferma richiesta).";
      hint.style.color = "";
    }
  });

  fw$("btn").addEventListener("click", async () => {
    if (!FW_FILE) { toast("Seleziona prima un file .bin","warn"); return; }
    if (FW_FILE.size > OTA_MAX_BYTES) { toast("File troppo grande (> "+OTA_MAX_BYTES+" B)","err"); return; }

    const confirmMsg =
      "Aggiornare il firmware del dispositivo?\n\n" +
      "File: " + FW_FILE.name + "\n" +
      "Dimensione: " + FW_FILE.size.toLocaleString("it-IT") + " byte\n\n" +
      "⚠ NON CHIUDERE LA PAGINA E NON SCOLLEGARE ALIMENTAZIONE\n" +
      "   finché non viene completato l'aggiornamento.";
    if (!confirm(confirmMsg)) return;

    const btn = fw$("btn");
    const fileInput = fw$("file");
    const btnRf = fw$("btnRf");
    btn.disabled = true; fileInput.disabled = true; btnRf.disabled = true;
    fwLog("Inizio aggiornamento firmware: " + FW_FILE.name + " (" + FW_FILE.size + " B)", "info");
    fwSetProgress(0, FW_FILE.size);

    try {
      const info = await apiGetFwInfo();
      if (info && info.free_heap != null && info.free_heap < 12288) {
        fwLog("ATTENZIONE: free heap " + info.free_heap + " B < min 12288 B. Rischio fallimento.", "warn");
      }
    } catch (_) {}

    const fd = new FormData();
    fd.append("firmware", FW_FILE, FW_FILE.name);

    await new Promise((resolve, reject) => {
      const xhr = new XMLHttpRequest();
      xhr.open("POST", "/update", true);
      xhr.upload.onprogress = ev => {
        if (ev.lengthComputable) fwSetProgress(ev.loaded, ev.total);
      };
      xhr.onload = () => {
        let resp = null;
        try { resp = JSON.parse(xhr.responseText || "{}"); } catch(e) { resp = null; }
        const ok = xhr.status >= 200 && xhr.status < 300 && resp && resp.ok;
        fwLog("Risposta server: HTTP " + xhr.status + "  →  " + (xhr.responseText || "(vuoto)").substring(0, 240),
              ok ? "info" : "err");
        if (ok) {
          fwSetProgress(FW_FILE.size, FW_FILE.size);
          resolve(resp);
        } else {
          reject(new Error((resp && resp.message) || ("HTTP " + xhr.status)));
        }
      };
      xhr.onerror = () => {
        fwLog("XHR network error (connessione interrotta, probabilmente il dispositivo si sta riavviando)", "warn");
        fwSetProgress(FW_FILE.size, FW_FILE.size);
        if (FW_FILE.size > 0) {
          resolve({ok:true, assumed:true});
        } else {
          reject(new Error("Network error"));
        }
      };
      xhr.onabort = () => reject(new Error("Upload annullato"));
      xhr.timeout = 120000;
      xhr.ontimeout = () => reject(new Error("Timeout upload 120s"));
      try { xhr.send(fd); } catch (e) { reject(e); }
    });

    fwLog("✅ Aggiornamento completato. Riavvio dispositivo in corso...", "ok");
    fwLog("Conteggio alla rovescia prima di ricaricare la pagina:", "info");
    let cd = 5;
    const cdTimer = setInterval(() => {
      fwLog(" ... " + cd + "s", "info");
      cd--;
      if (cd <= 0) {
        clearInterval(cdTimer);
        fwLog("Ricarico la pagina...", "info");
        setTimeout(() => location.reload(), 500);
      }
    }, 1000);
  });

  fw$("btnRf").addEventListener("click", async () => {
    const info = await apiGetFwInfo();
    if (info) {
      renderFwInfo(info);
      toast("Info firmware aggiornate","ok");
    } else {
      toast("Errore recupero info","err");
    }
  });

  (async function fwInit() {
    const info = await apiGetFwInfo();
    if (info) renderFwInfo(info);
    setInterval(async () => {
      if (fw$("btn") && fw$("btn").disabled) return;
      const i = await apiGetFwInfo();
      if (i) renderFwInfo(i);
    }, 5000);
  })();

  loadAndApplyAll();
})();
</script>
</body>
</html>
)rawliteral";

// ============================================================================
// Helper C per calcoli lato firmware
// ============================================================================
static float dewPointC(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    const float a = 17.27f, b = 237.7f;
    if (rh < 0.01f) rh = 0.01f;
    if (rh > 100.0f) rh = 100.0f;
    float alpha = a * tc / (b + tc) + logf(rh / 100.0f);
    return (b * alpha) / (a - alpha);
}

static float heatIndexC(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    if (rh < 0.0f) rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;
    if (tc < 26.7f) return tc;
    const float c1 = -42.379f;
    const float c2 =   2.04901523f;
    const float c3 =  10.14333127f;
    const float c4 =  -0.22475541f;
    const float c5 =  -0.00683783f;
    const float c6 =  -0.05481717f;
    const float c7 =   0.00122874f;
    const float c8 =   0.00085282f;
    const float c9 =  -0.00000199f;
    float tf = tc * 1.8f + 32.0f;
    float tfsq = tf * tf;
    float rhsq = rh * rh;
    float hif = c1 + c2*tf + c3*rh + c4*tf*rh + c5*tfsq + c6*rhsq
             + c7*tfsq*rh + c8*tf*rhsq + c9*tfsq*rhsq;
    return (hif - 32.0f) / 1.8f;
}

static float absoluteHumidityGm3(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    if (rh < 0.01f) rh = 0.01f;
    if (rh > 100.0f) rh = 100.0f;
    float es = 6.1121f * expf((18.678f - tc / 234.5f) * (tc / (257.14f + tc)));
    float e = (rh / 100.0f) * es;
    return (216.7f * e) / (tc + 273.15f);
}

static uint8_t sht31_crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0xFFu;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t k = 0; k < 8; k++) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1) ^ 0x31u);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

static bool sht31_send_cmd(uint16_t cmd)
{
    Wire.beginTransmission(SHT31_I2C_ADDR);
    Wire.write((uint8_t)(cmd >> 8));
    Wire.write((uint8_t)(cmd & 0xFFu));
    uint8_t err = Wire.endTransmission(true);
    supervisorWatchdogFeed();
    g_diag_wdtFeedCount++;
    if (err != 0) {
        g_status_bits &= ~STATUS_SHT31_I2C_OK;
        return false;
    }
    g_status_bits |= STATUS_SHT31_I2C_OK;
    return true;
}

static bool sht31_readStatusReg(uint16_t* out)
{
    if (!sht31_send_cmd(SHT31_CMD_READ_STATUS)) return false;
    uint8_t rx = Wire.requestFrom((uint8_t)SHT31_I2C_ADDR, (uint8_t)3);
    supervisorWatchdogFeed();
    g_diag_wdtFeedCount++;
    if (rx != 3) return false;
    uint8_t buf[3];
    buf[0] = (uint8_t)Wire.read();
    buf[1] = (uint8_t)Wire.read();
    buf[2] = (uint8_t)Wire.read();
    uint8_t crcCalc = sht31_crc8(buf, 2);
    if (crcCalc != buf[2]) return false;
    if (out) *out = (uint16_t)((buf[0] << 8) | buf[1]);
    return true;
}

static bool sht31_init(void)
{
    Serial.println(F("--------------------------------------------------------------"));
    Serial.println(F(" SHT31-D I2C: inizializzazione ..."));

    Wire.begin(D2, D1);
    Wire.setClock(SHT31_I2C_FREQ);
    Serial.print(F("  Wire.begin(SDA=D2, SCL=D1) @"));
    Serial.print(SHT31_I2C_FREQ); Serial.println(F(" Hz"));

    Wire.beginTransmission(SHT31_I2C_ADDR);
    uint8_t err = Wire.endTransmission(true);
    supervisorWatchdogFeed();
    g_diag_wdtFeedCount++;
    if (err != 0) {
        Serial.print(F("  ERRORE: I2C NACK @ addr 0x"));
        Serial.print(SHT31_I2C_ADDR, HEX);
        Serial.print(F("  (endTransmission rc=")); Serial.print(err); Serial.println(F(")"));
        Serial.println(F("   -> setup continuerà, polling riproverà ogni secondo."));
        g_status_bits = 0;
        return false;
    }
    Serial.print(F("  I2C ACK OK @ addr 0x")); Serial.println(SHT31_I2C_ADDR, HEX);
    g_status_bits |= STATUS_SHT31_I2C_OK;

    if (!sht31_send_cmd(SHT31_CMD_SOFT_RESET)) {
        Serial.println(F("  WARNING: Soft Reset 0x30A2 fallito (continuo)"));
    } else {
        delay(2);
        supervisorWatchdogFeed();
        g_diag_wdtFeedCount++;
        Serial.println(F("  Soft Reset 0x30A2 → OK"));
    }

    if (!sht31_send_cmd(SHT31_CMD_CLEAR_STATUS)) {
        Serial.println(F("  WARNING: Clear Status Register fallito (continuo)"));
    } else {
        Serial.println(F("  Clear Status Register → OK"));
    }

    uint16_t statusReg = 0;
    if (sht31_readStatusReg(&statusReg)) {
        Serial.print(F("  Status Register = 0x")); Serial.println(statusReg, HEX);
        if (statusReg & SHT31_STATUS_HEATER_BIT) {
            Serial.println(F("  Heater risultava ON da stato precedente -> forza OFF"));
            sht31_send_cmd(SHT31_CMD_HEATER_OFF);
        }
    } else {
        Serial.println(F("  WARNING: Lettura Status Register fallita (continuo)"));
    }

    Serial.println(F("  Prova misura singola ..."));
    if (!sht31_send_cmd(SHT31_CMD_SINGLE_HIGH_NOSTRETCH)) {
        Serial.println(F("  WARNING: Test measure cmd fallito (setup OK ma polling riproverà)"));
    } else {
        unsigned long t0 = millis();
        while ((millis() - t0) < SHT31_TCONV_HIGH_MS) {
            delay(1);
            supervisorWatchdogFeed();
            g_diag_wdtFeedCount++;
        }
        uint8_t rx = Wire.requestFrom((uint8_t)SHT31_I2C_ADDR, (uint8_t)6);
        supervisorWatchdogFeed();
        g_diag_wdtFeedCount++;
        if (rx == 6) {
            uint8_t buf[6];
            for (uint8_t k = 0; k < 6; k++) buf[k] = (uint8_t)Wire.read();
            bool crcT = (sht31_crc8(&buf[0], 2) == buf[2]);
            bool crcH = (sht31_crc8(&buf[3], 2) == buf[5]);
            if (crcT && crcH) {
                uint16_t rawT = (uint16_t)((buf[0] << 8) | buf[1]);
                uint16_t rawH = (uint16_t)((buf[3] << 8) | buf[4]);
                float t = -45.0f + 175.0f * ((float)rawT / 65535.0f);
                float h = 100.0f * ((float)rawH / 65535.0f);
                g_sht31_t = t; g_sht31_rh = h;
                g_sht31_dp = dewPointC(t, h);
                g_sht31_hi = heatIndexC(t, h);
                g_sht31_ah = absoluteHumidityGm3(t, h);
                g_status_bits |= STATUS_SHT31_T_OK | STATUS_SHT31_RH_OK | STATUS_SHT31_CRC_OK;
                Serial.print(F("  Test OK: T=")); Serial.print(t, 3);
                Serial.print(F("°C  RH=")); Serial.print(h, 2);
                Serial.print(F("%  CRC=OK  inizializzato completato.\r\n"));
            } else {
                Serial.print(F("  Test misura: CRC fallito (T_CRC=")); Serial.print(crcT ? "OK":"BAD");
                Serial.print(F(" H_CRC=")); Serial.print(crcH ? "OK":"BAD"); Serial.println(F(")"));
            }
        } else {
            Serial.print(F("  Test misura: requestFrom 6 byte fallito, rx=")); Serial.println(rx);
        }
    }
    Serial.println(F("--------------------------------------------------------------"));
    return true;
}

static const char* sensorStatusList(uint8_t bits)
{
    static char buf[200];
    const char* n1 = g_config.sensor_sht31_name[0] ? g_config.sensor_sht31_name : SENSOR_SHT31_NAME_DEFAULT;
    bool tOk  = (bits & STATUS_SHT31_T_OK) != 0;
    bool rhOk = (bits & STATUS_SHT31_RH_OK) != 0;
    bool crcOk = (bits & STATUS_SHT31_CRC_OK) != 0;
    bool heat = (bits & STATUS_SHT31_HEAT_ON) != 0;
    snprintf(buf, sizeof(buf), "%s [T:%s RH:%s CRC:%s%s]",
        n1,
        tOk ? "OK" : "NC",
        rhOk ? "OK" : "NC",
        crcOk ? "OK" : "ERR",
        heat ? " HEATER" : "");
    return buf;
}

static const char* sensorStatusHexBin(uint8_t bits)
{
    static char buf[32];
    snprintf(buf, sizeof(buf), "0x%02X  b%d%d%d%d%d%d",
        (unsigned)bits,
        (bits>>5)&1,(bits>>4)&1,(bits>>3)&1,(bits>>2)&1,(bits>>1)&1,(bits>>0)&1);
    return buf;
}

// ============================================================================
// SHT31-D Polling state machine (Single-Shot HIGH accuracy No-Stretch 1Hz)
// ============================================================================
typedef enum {
    SHT31_POLL_IDLE = 0,
    SHT31_POLL_WAIT = 1
} Sht31PollState;

static Sht31PollState g_sht31_pollState = SHT31_POLL_IDLE;

static void sht31_poll(void)
{
    unsigned long now = millis();

    switch (g_sht31_pollState) {
    case SHT31_POLL_IDLE: {
        if (now - g_sht31_lastSample_ms < SHT31_SAMPLE_INTERVAL_MS) {
            return;
        }
        // Trigger conversione Single-Shot HIGH No-Stretch
        if (!sht31_send_cmd(SHT31_CMD_SINGLE_HIGH_NOSTRETCH)) {
            // Comando fallito (NACK/bus errato) → azzera bit I2C/CRC/T/RH, NAN dati
            g_status_bits &= ~(STATUS_SHT31_I2C_OK | STATUS_SHT31_CRC_OK | STATUS_SHT31_T_OK | STATUS_SHT31_RH_OK);
            g_sht31_t  = NAN;
            g_sht31_rh = NAN;
            g_sht31_dp = NAN;
            g_sht31_hi = NAN;
            g_sht31_ah = NAN;
            g_sht31_lastSample_ms = now;
            return;
        }
        g_status_bits |= STATUS_SHT31_I2C_OK;
        g_sht31_convPending  = true;
        g_sht31_convStart_ms = now;
        g_sht31_pollState    = SHT31_POLL_WAIT;
        break;
    }
    case SHT31_POLL_WAIT: {
        if (now - g_sht31_convStart_ms < SHT31_CONV_TIME_MS) {
            return;
        }
        g_sht31_convPending = false;
        // Richiedi 6 byte: [Tmsb, Tlsb, Tcrc, Hmsb, Hlsb, Hcrc]
        uint8_t n = Wire.requestFrom((uint8_t)SHT31_I2C_ADDR, (uint8_t)6);
        supervisorWatchdogFeed();
        g_diag_wdtFeedCount++;
        if (n != 6) {
            g_status_bits &= ~(STATUS_SHT31_I2C_OK | STATUS_SHT31_CRC_OK | STATUS_SHT31_T_OK | STATUS_SHT31_RH_OK);
            g_sht31_t  = NAN;
            g_sht31_rh = NAN;
            g_sht31_dp = NAN;
            g_sht31_hi = NAN;
            g_sht31_ah = NAN;
            g_sht31_lastSample_ms = now;
            g_sht31_pollState = SHT31_POLL_IDLE;
            return;
        }
        uint8_t buf[6];
        for (uint8_t i = 0; i < 6; i++) buf[i] = Wire.read();

        // Verifica CRC per ogni word (2 byte + 1 byte CRC)
        bool tCrcOk  = (sht31_crc8(&buf[0], 2) == buf[2]);
        bool rhCrcOk = (sht31_crc8(&buf[3], 2) == buf[5]);
        if (tCrcOk && rhCrcOk) g_status_bits |= STATUS_SHT31_CRC_OK;
        else                   g_status_bits &= ~STATUS_SHT31_CRC_OK;

        // Conversione raw Temperature: T[°C] = -45 + 175 * (rawT / 65535)
        uint16_t rawT = ((uint16_t)buf[0] << 8) | buf[1];
        if (tCrcOk) {
            float t = -45.0f + 175.0f * ((float)rawT / 65535.0f);
            if (t >= SHT31_T_MIN_C && t <= SHT31_T_MAX_C) {
                g_sht31_t = t;
                g_status_bits |= STATUS_SHT31_T_OK;
            } else {
                g_sht31_t = NAN;
                g_status_bits &= ~STATUS_SHT31_T_OK;
            }
        } else {
            g_sht31_t = NAN;
            g_status_bits &= ~STATUS_SHT31_T_OK;
        }

        // Conversione raw Humidity: RH[%] = 100 * (rawH / 65535)
        uint16_t rawH = ((uint16_t)buf[3] << 8) | buf[4];
        if (rhCrcOk) {
            float rh = 100.0f * ((float)rawH / 65535.0f);
            if (rh >= SHT31_RH_MIN_PCT && rh <= SHT31_RH_MAX_PCT) {
                g_sht31_rh = rh;
                g_status_bits |= STATUS_SHT31_RH_OK;
            } else {
                g_sht31_rh = NAN;
                g_status_bits &= ~STATUS_SHT31_RH_OK;
            }
        } else {
            g_sht31_rh = NAN;
            g_status_bits &= ~STATUS_SHT31_RH_OK;
        }

        // Calcolo grandezze derivate (solo se T e RH validi)
        bool validTRH = (g_status_bits & (STATUS_SHT31_T_OK | STATUS_SHT31_RH_OK)) ==
                        (STATUS_SHT31_T_OK | STATUS_SHT31_RH_OK);
        if (validTRH) {
            g_sht31_dp = dewPointC(g_sht31_t, g_sht31_rh);
            g_sht31_hi = heatIndexC(g_sht31_t, g_sht31_rh);
            g_sht31_ah = absoluteHumidityGm3(g_sht31_t, g_sht31_rh);
        } else {
            g_sht31_dp = NAN;
            g_sht31_hi = NAN;
            g_sht31_ah = NAN;
        }

        // Heater auto hysteresis (solo se RH valido)
        if (g_status_bits & STATUS_SHT31_RH_OK) {
            if (g_sht31_rh > SHT31_HEATER_ON_RH_PCT && !g_sht31_heater) {
                if (g_sht31_heater_start_ms == 0UL) {
                    g_sht31_heater_start_ms = now;
                } else if (now - g_sht31_heater_start_ms >= SHT31_HEATER_MIN_ON_MS) {
                    if (sht31_send_cmd(SHT31_CMD_HEATER_ON)) {
                        g_sht31_heater = true;
                        g_status_bits |= STATUS_SHT31_HEAT_ON;
                        g_sht31_heater_start_ms = now;
                    }
                }
            } else if (g_sht31_rh < SHT31_HEATER_OFF_RH_PCT && g_sht31_heater) {
                if (sht31_send_cmd(SHT31_CMD_HEATER_OFF)) {
                    g_sht31_heater = false;
                    g_status_bits &= ~STATUS_SHT31_HEAT_ON;
                    g_sht31_heater_start_ms = 0UL;
                }
            } else if (g_sht31_rh <= SHT31_HEATER_ON_RH_PCT) {
                // Reset conteggio se RH non supera più la soglia di accensione
                if (!g_sht31_heater) g_sht31_heater_start_ms = 0UL;
            }
        }

        g_sht31_lastSample_ms = now;
        g_sht31_pollState = SHT31_POLL_IDLE;
        break;
    }
    }
}

// ============================================================================
// Istanze server e sensori
// ============================================================================
static ESP8266WebServer  g_http(HTTP_PORT);
static WebSocketsServer  g_ws(WS_PORT);

// ============================================================================
// MQTT parametrizzato — tracking avanzato riconnessioni
// ============================================================================
static WiFiClient   g_wifiClient;
static PubSubClient g_mqtt(g_wifiClient);
static bool         g_mqtt_discovery_sent = false;
static unsigned long g_mqtt_lastAttemptMs = 0;
static bool         g_mqtt_config_changed = false;

static bool         g_mqtt_was_connected = false;
static unsigned long g_mqtt_connect_ts_ms = 0;
static unsigned long g_mqtt_disconnect_ts_ms = 0;
static uint16_t     g_mqtt_consec_failures = 0;
static uint8_t      g_mqtt_pub_consec_fail = 0;  // publish fallite consecutive CON connected()=true
static unsigned long g_mqtt_last_online_ms = 0;

// ============================================================================
// Stato globale supervisore MQTT
// ============================================================================
typedef enum {
    SUPV_LVL_IDLE = 0,
    SUPV_LVL_L1   = 1,
    SUPV_LVL_L2   = 2,
    SUPV_LVL_L3   = 3,
    SUPV_LVL_L4   = 4
} SupvLevel;

static SupvLevel     g_supvLevel       = SUPV_LVL_IDLE;
static bool          g_supvTripped     = false;
static unsigned long g_supvTripMs      = 0;
static unsigned long g_supvLastL1Ms    = 0;
static unsigned long g_supvLastL2Ms    = 0;
static unsigned long g_supvLastL3Ms    = 0;

static unsigned long g_supvLastGoodConnMs = 0;
static unsigned long g_supvLastGoodPubMs  = 0;
static uint32_t      g_supvL1Count        = 0;
static uint32_t      g_supvL2Count        = 0;
static uint32_t      g_supvL3Count        = 0;
static uint32_t      g_supvResetCount     = 0;
static bool          g_supvL1InProgress   = false;

// State machine L2/L3 asincrone (nessun delay() bloccante, per non rompere WS/WDT)
typedef enum {
    L2S_IDLE = 0,
    L2S_WAIT_COOLDOWN = 1,
    L2S_DONE = 2
} L2State;
typedef enum {
    L3S_IDLE = 0,
    L3S_WAIT_WIFIOFF = 1,
    L3S_WAIT_POST = 2,
    L3S_DONE = 3
} L3State;
static L2State g_supvL2State = L2S_IDLE;
static L3State g_supvL3State = L3S_IDLE;
static unsigned long g_supvL2StepMs = 0;
static unsigned long g_supvL3StepMs = 0;

#define SUPV_L2_DELAY_MS   250UL
#define SUPV_L3_OFF_MS     1200UL
#define SUPV_L3_POST_MS    100UL

// ============================================================================
// OTA FSM state + variabili globali
// ============================================================================
typedef enum {
    OTA_IDLE = 0,
    OTA_HEADER_CHECKED,
    OTA_WRITING,
    OTA_WRITE_COMPLETE,
    OTA_ABORTED
} OtaState;

static OtaState       g_otaState = OTA_IDLE;
static bool           g_otaBusy = false;
static unsigned long  g_otaLastChunkMs = 0;
static unsigned long  g_otaStartMs = 0;
static size_t         g_otaBytesWritten = 0;
static size_t         g_otaBytesExpected = 0;
static size_t         g_otaTotalSizeDeclared = 0;
static bool           g_otaUploadEndReceived = false;
static uint32_t       g_otaWdtChunkCounter = 0;
static char           g_otaAbortReason[96] = {0};
static uint8_t        g_otaFirst4Bytes[4] = {0};
static bool           g_otaFirst4Valid = false;
static char           g_otaLastError[96] = {0};
static unsigned long  g_otaLastOkMs = 0;
static unsigned long  g_otaLastFailMs = 0;

static const char* supvLvlStr(SupvLevel l);

// ============================================================================
// IMPLEMENTAZIONI POSTICIPATE Crash Logger (dopo dichiarazione globali statici)
// ============================================================================
static void crashHeartbeatSave(uint32_t nowMs)
{
    if (nowMs - g_diag_lastHeartbeatMs < 500u && g_diag_lastHeartbeatMs != 0) return;
    g_diag_lastHeartbeatMs = nowMs;

    uint32_t buf[HEARTBEAT_RTC_WORDS];
    memset(buf, 0, sizeof(buf));
    uint32_t hf = (uint32_t)ESP.getFreeHeap();
    if (hf < g_diag_minHeapEver) g_diag_minHeapEver = hf;
    uint32_t swm = stackWatermarkBytes();
    if (swm < g_diag_stackWmBytes) g_diag_stackWmBytes = swm;

    buf[0] = 0x42484541u;  // magic "HEAB"
    buf[1] = 0xA55A0001u;
    buf[2] = nowMs;
    buf[3] = hf;
    buf[4] = g_diag_minHeapEver;
    buf[5] = (uint32_t)ESP.getHeapFragmentation();
    buf[6] = (uint32_t)ESP.getMaxFreeBlockSize();
    buf[7] = swm;
    buf[8] = g_diag_wdtFeedCount;
    buf[9]  = (g_mqtt.connected() ? 0x00010000u : 0x00000000u) | (uint32_t)(WiFi.status() & 0xFFu);
    buf[10] = ((uint32_t)g_status_bits << 16) | (g_sht31_convPending ? 0x00000001u : 0u);
    buf[11] = (uint32_t)g_mqtt_consec_failures;
    buf[12] = g_supvL1Count + (g_supvL2Count << 8) + ((uint32_t)g_supvL3Count << 16) + ((uint32_t)g_supvResetCount << 24);
    buf[13] = g_diag_bootCount;
    buf[15] = crashCrc32(buf, HEARTBEAT_RTC_WORDS - 1);

    // ---- Cache raw words per capturePanicContext (senza API durante exception!) ----
    g_diag_lastMqttWifiWord = buf[9];
    g_diag_lastSensorWord   = buf[10];

    crashRtcWriteWords(HEARTBEAT_RTC_START, buf, HEARTBEAT_RTC_WORDS);
}

static RstReasonCode decodeEspResetReasonRiOnly(const rst_info* ri)
{
    if (ri == nullptr) return RST_EXCEPTION_PANIC;
    switch (ri->reason)
    {
        case REASON_DEFAULT_RST:      return RST_POWERON;
        case REASON_WDT_RST:          return RST_HW_WATCHDOG;
        case REASON_EXCEPTION_RST:    return RST_EXCEPTION_PANIC;
        case REASON_SOFT_WDT_RST:     return RST_SW_WATCHDOG;
        case REASON_SOFT_RESTART:     return RST_SOFTWARE_RESTART;
        case REASON_DEEP_SLEEP_AWAKE: return RST_DEEPSLEEP_WAKE;
        case REASON_EXT_SYS_RST:      return RST_EXTERNAL_PIN;
        default:                      return RST_EXCEPTION_PANIC;
    }
}

static void capturePanicContext(struct rst_info* ri)
{
    // IMPORTANTE: NESSUNA CHIAMATA A METODI OGGETTI C++ QUI.
    // L'eccezione panic disabilita interrupt e lwIP/oggetti potrebbero essere in stato
    // inconsistente. Qualsiasi deref indiretto (g_mqtt.connected(), WiFi.status(),
    // String() temporanei, malloc) può causare LoadStoreErrorBus RICORSIVO
    // → crash loop infinito, RTC crash context mai scritto, prossimo boot vede
    // "nessun crash valido". Usiamo solo variabili globali raw cached.
    memset(&g_panicCtx, 0, sizeof(g_panicCtx));
    g_panicCtx.magic1       = CRASHLOG_MAGIC_1;
    g_panicCtx.magic2       = CRASHLOG_MAGIC_2;
    g_panicCtx.version      = CRASHLOG_VERSION;
    g_panicCtx.rstReason    = decodeEspResetReasonRiOnly(ri);   // solo switch ri->reason, NO String malloc
    g_panicCtx.uptimeMs     = (volatile uint32_t)millis();
    g_panicCtx.heapFree     = (uint32_t)ESP.getFreeHeap();      // OK: legge contatori UMM
    g_panicCtx.heapMin      = (uint32_t)(volatile uint32_t)g_diag_minHeapEver;
    g_panicCtx.heapFragPct  = (uint32_t)ESP.getHeapFragmentation();
    g_panicCtx.stackWmBytes = (uint32_t)(volatile uint32_t)g_diag_stackWmBytes;
    g_panicCtx.wdtFeedCount = (uint32_t)(volatile uint32_t)g_diag_wdtFeedCount;
    g_panicCtx.mqttConnWifiWord = (uint32_t)(volatile uint32_t)g_diag_lastMqttWifiWord;  // <-- cached, NO API
    g_panicCtx.sensorStateWord  = (uint32_t)(volatile uint32_t)g_diag_lastSensorWord;    // <-- cached, NO API
    g_panicCtx.epochSec     = 0;
    g_panicCtx.crc32        = 0;
    crashSaveContext(&g_panicCtx);
    Serial.flush();
    // ~200ms busy delay at 80/160 MHz per permettere a seriale di spedire
    for (volatile uint32_t i = 0; i < 800000u; i++) { (void)i; }
}

extern "C" void custom_crash_callback(struct rst_info* rstInfo, uint32_t stackStart, uint32_t stackEnd)
{
    (void)stackStart; (void)stackEnd;
    capturePanicContext(rstInfo);
}
extern "C" void custom_pre_sleep_callback(uint8_t wakeup_opt, uint32_t sleep_ms)
{
    (void)wakeup_opt; (void)sleep_ms;
}

static void mqttBuildBaseTopic(char* buf, size_t bufLen)
{
    snprintf(buf, bufLen, "homeassistant/sensor/%s/",
        (g_config.mqtt_device_id[0] ? g_config.mqtt_device_id : MQTT_DEVICE_ID_DEFAULT));
}

static void mqttBuildAvailTopic(char* buf, size_t bufLen)
{
    snprintf(buf, bufLen, "homeassistant/sensor/%s/availability",
        (g_config.mqtt_device_id[0] ? g_config.mqtt_device_id : MQTT_DEVICE_ID_DEFAULT));
}

static size_t mqttBuildTopic(char* buf, size_t bufLen, const char* suffix)
{
    char base[96];
    mqttBuildBaseTopic(base, sizeof(base));
    return snprintf(buf, bufLen, "%s%s", base, suffix);
}

static void mqttPublishScalar(const char* stateSuffix, float v, uint8_t decimals)
{
    if (!isfinite(v)) return;
    char topic[160];
    char value[24];
    mqttBuildTopic(topic, sizeof(topic), stateSuffix);
    size_t tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) {
        memcpy(topic + tl, "/state", 7);
    }
    snprintf(value, sizeof(value), "%.*f", decimals, v);
    bool published = g_mqtt.publish(topic, value, true);
    if (published) {
        g_supvLastGoodPubMs = millis();
        g_mqtt_pub_consec_fail = 0;
    } else {
        if (g_mqtt.connected()) {
            g_mqtt_pub_consec_fail++;
            int mqttState = g_mqtt.state();
            Serial.print(F("MQTT PUBLISH FALLITA (connesso ma fallita, consec_fail="));
            Serial.print(g_mqtt_pub_consec_fail);
            Serial.print(F(") topic="));
            Serial.print(topic);
            Serial.print(F(" state=")); Serial.print(mqttState);
            if (mqttState == MQTT_DISCONNECTED) Serial.print(F(" [probabile BUFFER OVF o TCP congestionato]"));
            Serial.println();
            if (g_mqtt_pub_consec_fail >= 3U) {
                Serial.print(F("  *** 3 PUB FAIL consecutive CON connected()=true → FORZO DISCONNECT per triggerare riconnessione"));
                Serial.print(F(" (state=")); Serial.print(mqttState); Serial.println(F(")"));
                char availTopic[160];
                mqttBuildAvailTopic(availTopic, sizeof(availTopic));
                (void)g_mqtt.publish(availTopic, "offline", true);
                g_mqtt.disconnect();
                g_mqtt_was_connected = false;
                g_mqtt_disconnect_ts_ms = millis();
                g_mqtt_consec_failures = 0;
                g_mqtt_lastAttemptMs = 0;
                g_mqtt_pub_consec_fail = 0;
            }
        }
    }
}

static void mqttPublishDiscoveryOne(
    const char* sensorId,
    const char* name,
    const char* devClass,
    const char* unit,
    const char* icon,
    const char* stateClass,
    const char* entityCategory)
{
    char topic[192];
    char stateTopic[160];
    char uniqueId[128];
    char availTopic[160];
    char devId[80];
    char devName[80];

    safeStrncpy(devId, g_config.mqtt_device_id[0] ? g_config.mqtt_device_id : MQTT_DEVICE_ID_DEFAULT, sizeof(devId));
    safeStrncpy(devName, g_config.device_name[0] ? g_config.device_name : MQTT_DEVICE_NAME_DEFAULT, sizeof(devName));

    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s/config", devId, sensorId);
    mqttBuildTopic(stateTopic, sizeof(stateTopic), sensorId);
    size_t tl = strlen(stateTopic);
    if (tl + 7 < sizeof(stateTopic)) {
        memcpy(stateTopic + tl, "/state", 7);
    }
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", devId, sensorId);
    mqttBuildAvailTopic(availTopic, sizeof(availTopic));

    String swVerStr = ESP.getCoreVersion();
    const char* swVer = swVerStr.c_str();

    static char payload[1280];
    const size_t bufSize = sizeof(payload);
    int written = snprintf(payload, bufSize,
        "{"
        "\"name\":\"%s\","
        "\"unique_id\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\"",
        name, uniqueId, stateTopic, availTopic);

    if (written > 0 && written < (int)bufSize - 1) {
        if (devClass && devClass[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"device_class\":\"%s\"", devClass);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        if (unit && unit[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"unit_of_measurement\":\"%s\"", unit);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        if (stateClass && stateClass[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"state_class\":\"%s\"", stateClass);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        if (icon && icon[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"icon\":\"%s\"", icon);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        if (entityCategory && entityCategory[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"entity_category\":\"%s\"", entityCategory);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        written += snprintf(payload + written, bufSize - written,
            ",\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"manufacturer\":\"%s\","
            "\"model\":\"%s\","
            "\"sw_version\":\"%s\""
            "}"
            "}",
            devId, devName, MQTT_MANUFACTURER, MQTT_MODEL, swVer);
    }

    if (written > 0 && written < (int)bufSize - 1) {
        if (!g_mqtt.publish(topic, (const uint8_t*)payload, (unsigned int)written, true)) {
            Serial.print(F("  *** MQTT DISCOVERY PUBLISH FALLITA sid="));
            Serial.print(sensorId);
            Serial.print(F(" lenPayload=")); Serial.print(written);
            Serial.print(F(" state=")); Serial.println(g_mqtt.state());
        }
    } else {
        Serial.print(F("  *** MQTT DISCOVERY BUFFER OVERFLOW sid="));
        Serial.print(sensorId);
        Serial.print(F(" written=")); Serial.print(written);
        Serial.print(F(" bufSize=")); Serial.println(bufSize);
    }
}

static void mqttPublishAllDiscovery(void)
{
    const char* ns = g_config.sensor_sht31_name[0] ? g_config.sensor_sht31_name : SENSOR_SHT31_NAME_DEFAULT;

    char buf[96];
    snprintf(buf, sizeof(buf), "%s Temperatura", ns);
    mqttPublishDiscoveryOne("sht31_temp_c", buf, "temperature", "°C", "mdi:thermometer", "measurement", NULL);
    snprintf(buf, sizeof(buf), "%s Umidita Relativa", ns);
    mqttPublishDiscoveryOne("sht31_hum_rh", buf, "humidity",    "%",  "mdi:water-percent", "measurement", NULL);
    snprintf(buf, sizeof(buf), "%s Punto Rugiada", ns);
    mqttPublishDiscoveryOne("sht31_dew_c", buf, "temperature", "°C", "mdi:water-thermometer", "measurement", NULL);
    snprintf(buf, sizeof(buf), "%s Indice Calore", ns);
    mqttPublishDiscoveryOne("sht31_heat_index_c", buf, "temperature", "°C", "mdi:sun-thermometer", "measurement", NULL);
    snprintf(buf, sizeof(buf), "%s Umidita Assoluta", ns);
    mqttPublishDiscoveryOne("sht31_abs_hum_gm3", buf, NULL, "g/m³", "mdi:water-opacity", "measurement", NULL);
    snprintf(buf, sizeof(buf), "%s Heater", ns);
    mqttPublishDiscoveryOne("sht31_heater", buf, NULL, NULL, "mdi:radiator-disabled", NULL, NULL);

    mqttPublishDiscoveryOne("sensor_status_bitmask", "Stato Sensori Bitmask", NULL, NULL, "mdi:chip", NULL, NULL);
    mqttPublishDiscoveryOne("sensor_status_hexbin",  "Stato Sensori HEX+BIN", NULL, NULL, "mdi:chip", NULL, NULL);
    mqttPublishDiscoveryOne("sensor_status_list",    "Sensori Collegati",     NULL, NULL, "mdi:clipboard-list", NULL, NULL);

    mqttPublishDiscoveryOne("supv_level",       "Supervisore Livello",    NULL, NULL, "mdi:shield-alert", NULL, "diagnostic");
    mqttPublishDiscoveryOne("supv_l1_count",    "Supervisore L1 Count",   NULL, NULL, "mdi:counter", "total_increasing", "diagnostic");
    mqttPublishDiscoveryOne("supv_l2_count",    "Supervisore L2 Count",   NULL, NULL, "mdi:counter", "total_increasing", "diagnostic");
    mqttPublishDiscoveryOne("supv_l3_count",    "Supervisore L3 Count",   NULL, NULL, "mdi:counter", "total_increasing", "diagnostic");
    mqttPublishDiscoveryOne("supv_reset_count", "Supervisore Reset HW X",  NULL, NULL, "mdi:counter", "total_increasing", "diagnostic");
    mqttPublishDiscoveryOne("supv_trip_ms",     "Supervisore Trip Durata (ms)", "duration", "ms", "mdi:timer-outline", NULL, "diagnostic");
    mqttPublishDiscoveryOne("supv_last_good_conn_ms", "Supervisore Ultima Conn OK (fa)", "duration", "ms", "mdi:timer-sync-outline", NULL, "diagnostic");
    mqttPublishDiscoveryOne("supv_last_good_pub_ms",  "Supervisore Ultima Pub OK (fa)",  "duration", "ms", "mdi:timer-sync-outline", NULL, "diagnostic");

    const char* devId2 = g_config.mqtt_device_id[0] ? g_config.mqtt_device_id : MQTT_DEVICE_ID_DEFAULT;
    snprintf(buf, sizeof(buf), "%s WiFi STA IP (v4)", devId2);
    mqttPublishDiscoveryOne("diag_wifi_sta_ip_v4", buf, NULL, NULL, "mdi:ip-network-outline", NULL, "diagnostic");

    snprintf(buf, sizeof(buf), "%s WiFi RSSI", devId2);
    mqttPublishDiscoveryOne("diag_wifi_rssi_dbm", buf, "signal_strength", "dBm", "mdi:wifi-strength-outline", "measurement", "diagnostic");

    snprintf(buf, sizeof(buf), "%s Heap Frammentazione", devId2);
    mqttPublishDiscoveryOne("diag_heap_frag_pct", buf, NULL, "%", "mdi:memory-arrow-down", "measurement", "diagnostic");
}

static void mqttApplyNewConfig(void)
{
    const char* srv = g_config.mqtt_server[0] ? g_config.mqtt_server : MQTT_BROKER_IP_DEFAULT;
    uint16_t port = (g_config.mqtt_port > 0 && g_config.mqtt_port <= 65535) ? g_config.mqtt_port : MQTT_BROKER_PORT_DEFAULT;
    if (g_mqtt.connected()) {
        char availTopic[160];
        mqttBuildAvailTopic(availTopic, sizeof(availTopic));
        (void)g_mqtt.publish(availTopic, "offline", true);
        g_mqtt.disconnect();
    }
    g_mqtt.setServer(srv, port);
    g_mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
    g_mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
    g_mqtt.setBufferSize(MQTT_CLIENT_BUFFER_SIZE);
    g_mqtt_discovery_sent = false;
    g_mqtt_config_changed = false;
    g_mqtt_was_connected = false;
    g_mqtt_consec_failures = 0;
    g_mqtt_lastAttemptMs = 0;
    Serial.print(F("MQTT configurazione aggiornata -> "));
    Serial.print(srv); Serial.print(':'); Serial.println(port);
    Serial.print(F("  KA=")); Serial.print(MQTT_KEEPALIVE_S);
    Serial.print(F("s sockTO=")); Serial.print(MQTT_SOCKET_TIMEOUT_S);
    Serial.print(F("s buf=")); Serial.print(MQTT_CLIENT_BUFFER_SIZE); Serial.println(F("B"));
}

static bool mqttEnsureConnected(void)
{
    if (g_otaBusy) {
        if (g_mqtt.connected()) {
            char availTopic[160];
            mqttBuildAvailTopic(availTopic, sizeof(availTopic));
            (void)g_mqtt.publish(availTopic, "offline", true);
            g_mqtt.disconnect();
            g_mqtt_was_connected = false;
            g_mqtt_disconnect_ts_ms = millis();
            g_mqtt_consec_failures = 0;
            g_mqtt_lastAttemptMs = 0;
            g_mqtt_pub_consec_fail = 0;
        }
        return false;
    }

    if (g_mqtt_config_changed) {
        mqttApplyNewConfig();
    }

    unsigned long now = millis();

    if (g_mqtt.connected()) {
        // --- Sanity check 1: PubSubClient connected() ma la socket è realmente viva? ---
        bool tcpAlive = g_wifiClient.connected();
        bool zombie = !tcpAlive;

        // --- Sanity check 2 (CATCH-ALL): TCP dice "vivo", ma da piu' di SUPV_PUB_STALL_MS
        // non abbiamo avuto nessuna publish andata a buon fine. Altissimo rischio che
        // la socket sia half-open zombie (FIN/RST perso dal router) e che lwIP stia
        // ancora aspettando i TCP keepalive default (7200s) prima di dichiararla morta.
        if (!zombie && g_supvLastGoodPubMs != 0 && (now - g_supvLastGoodPubMs) >= SUPV_PUB_STALL_MS) {
            Serial.print(F("MQTT SOCKET ZOMBIE RILEVATA (catch-all via PUB stagnante)! "));
            Serial.print(F("delta_lastGoodPubMs=")); Serial.print(now - g_supvLastGoodPubMs);
            Serial.print(F("ms soglia=")); Serial.print(SUPV_PUB_STALL_MS); Serial.print(F("ms "));
            Serial.print(F("tcpAlive=")); Serial.print(tcpAlive ? F("SI") : F("NO"));
            Serial.print(F(" mqttState=")); Serial.println(g_mqtt.state());
            zombie = true;
        }
        // --- Sanity check 3 (CATCH-ALL): publish fallite 3+ consecutive (forza disconnect in mqttPublishScalar)
        // e comunque qui ricontrolliamo: il contatore > 0 significa che le ultime publish hanno fallito
        if (!zombie && g_mqtt_pub_consec_fail >= 3U) {
            Serial.print(F("MQTT: 3+ pub fail consecutive ancora non triggerate disconnessione, FORZO"));
            Serial.print(F(" pub_fail=")); Serial.println(g_mqtt_pub_consec_fail);
            zombie = true;
        }

        if (zombie) {
            Serial.print(F("MQTT SOCKET ZOMBIE → DISCONNESSIONE FORZATA "));
            Serial.print(F("state=")); Serial.print(g_mqtt.state());
            Serial.print(F(" WiFi.status=")); Serial.println(WiFi.status());
            char availTopic[160];
            mqttBuildAvailTopic(availTopic, sizeof(availTopic));
            (void)g_mqtt.publish(availTopic, "offline", true);
            g_mqtt.disconnect();
            g_mqtt_was_connected = false;
            g_mqtt_disconnect_ts_ms = now;
            g_mqtt_consec_failures = 0;
            g_mqtt_lastAttemptMs = 0;
            g_mqtt_pub_consec_fail = 0;
            g_supvLastGoodConnMs = now;
            return false;
        }

        // --- Refresh periodico di availability "online" (QoS1 retain) ---
        if (g_mqtt_last_online_ms == 0 || (now - g_mqtt_last_online_ms) >= MQTT_ONLINE_REFRESH_MS) {
            char availTopic[160];
            mqttBuildAvailTopic(availTopic, sizeof(availTopic));
            if (g_mqtt.publish(availTopic, "online", true)) {
                g_mqtt_last_online_ms = now;
            } else {
                Serial.println(F("MQTT: refresh ONLINE fallito (forse socket zombie?), forza disconnect"));
                g_mqtt.disconnect();
                g_mqtt_was_connected = false;
                g_mqtt_disconnect_ts_ms = now;
                g_mqtt_consec_failures = 0;
                g_mqtt_lastAttemptMs = 0;
                return false;
            }
        }

        g_supvLastGoodConnMs = now;
        if (!g_mqtt_was_connected) {
            // Primo loop in cui siamo connessi dopo una disconnessione
            g_mqtt_was_connected = true;
            g_mqtt_connect_ts_ms = now;
            Serial.print(F("MQTT: STABILIZZATA connessione al loop dopo "));
            Serial.print(now - g_mqtt_disconnect_ts_ms);
            Serial.println(F("ms"));
        }
        return true;
    }

    // ====================================================================
    //                    CONNESSIONE NON ATTIVA
    //        Strategia backoff CONDIZIONALE + RICONNESSIONE IMMEDIATA
    // ====================================================================

    if (WiFi.status() != WL_CONNECTED) {
        g_supvLastGoodConnMs = now;
        return false;
    }

    // Calcolo backoff:
    //   - Se eravamo connessi prima (transizione appena persa): BACKOFF = 0
    //   - Altrimenti (stiamo già riprovando da tempo): backoff exp + jitter
    //   - Dopo >5 fallimenti consecutivi, minimo MAX_BACKOFF/2 per non spammare
    unsigned long backoffMs;
    if (g_mqtt_was_connected) {
        // TRANSIZIONE: disconnessione appena rilevata -> riconnetti SUBITO
        backoffMs = 0;
        Serial.print(F("MQTT: DISCONNESSIONE RILEVATA (eravamo connessi fino a "));
        if (g_mqtt_connect_ts_ms != 0) {
            Serial.print(now - g_mqtt_connect_ts_ms);
            Serial.print(F("ms fa, durata conn "));
            Serial.print(g_mqtt_connect_ts_ms - g_mqtt_disconnect_ts_ms);
        } else {
            Serial.print(g_mqtt_connect_ts_ms);
        }
        Serial.println(F("ms) -> RICONNESSIONE IMMEDIATA (backoff=0)"));
        g_mqtt_was_connected = false;
        g_mqtt_disconnect_ts_ms = now;
    } else {
        // Siamo già in fase di retry: backoff esponenziale + jitter
        uint16_t f = g_mqtt_consec_failures;
        if (f > 8) f = 8;
        unsigned long base = (unsigned long)MQTT_CONNECT_BACKOFF_MIN_MS << (f > 0 ? (f - 1) : 0);
        unsigned long jitter = (unsigned long)(ESP.random() % 128);
        backoffMs = base + jitter;
        if (backoffMs > MQTT_CONNECT_BACKOFF_MAX_MS) backoffMs = MQTT_CONNECT_BACKOFF_MAX_MS;
        if (g_mqtt_consec_failures >= 5 && backoffMs < (MQTT_CONNECT_BACKOFF_MAX_MS / 2)) {
            backoffMs = MQTT_CONNECT_BACKOFF_MAX_MS / 2;
        }
    }

    if (backoffMs > 0 && g_mqtt_lastAttemptMs != 0 && (now - g_mqtt_lastAttemptMs) < backoffMs) {
        return false;
    }
    g_mqtt_lastAttemptMs = now;

    // --- Tentativo CONNECT ---
    char clientId[96];
    const char* devId = g_config.mqtt_device_id[0] ? g_config.mqtt_device_id : MQTT_DEVICE_ID_DEFAULT;
    snprintf(clientId, sizeof(clientId), "%s_%08X", devId, (unsigned)ESP.getChipId());

    const char* user = g_config.mqtt_user[0] ? g_config.mqtt_user : MQTT_USER_DEFAULT;
    const char* pass = g_config.mqtt_password[0] ? g_config.mqtt_password : MQTT_PASSWORD_DEFAULT;
    const char* srv  = g_config.mqtt_server[0] ? g_config.mqtt_server : MQTT_BROKER_IP_DEFAULT;
    uint16_t port   = (g_config.mqtt_port > 0 && g_config.mqtt_port <= 65535) ? g_config.mqtt_port : MQTT_BROKER_PORT_DEFAULT;

    char availTopic[160];
    mqttBuildAvailTopic(availTopic, sizeof(availTopic));

    Serial.print(F("MQTT: tentativo connect #"));
    Serial.print(g_mqtt_consec_failures + 1);
    Serial.print(F(" a ")); Serial.print(srv); Serial.print(':'); Serial.print(port);
    Serial.print(F("  backoff=")); Serial.print(backoffMs); Serial.print(F("ms"));
    Serial.print(F("  fail_conseq=")); Serial.println(g_mqtt_consec_failures);

    bool ok = g_mqtt.connect(
        clientId,
        user,
        pass,
        availTopic,
        1,
        true,
        "offline",
        false);

    if (ok) {
        unsigned long restoreMs = 0;
        if (g_mqtt_disconnect_ts_ms != 0) restoreMs = now - g_mqtt_disconnect_ts_ms;
        g_mqtt_was_connected = true;
        g_mqtt_connect_ts_ms = now;
        g_mqtt_consec_failures = 0;
        g_mqtt_pub_consec_fail = 0;
        g_supvLastGoodConnMs = now;
#if defined(ESP8266)
        // TCP keepAlive va abilitato SOLO DOPO che la connect() ha creato _client interno!
        // Prima di una connessione TCP riuscita, WiFiClient._client==nullptr → keepAlive(1)
        // dereferenziava NULL → LoadStoreErrorBus panic (exccause=28, excvaddr=0).
        // Vedere crash report addr2line epc1=0x40212792 (WiFiClient::keepAlive NULL deref).
        g_wifiClient.keepAlive(TCP_KEEPALIVE_IDLE_SEC,
                               TCP_KEEPALIVE_INTERVAL_SEC,
                               TCP_KEEPALIVE_COUNT);
#endif
        Serial.print(F("MQTT: CONNESSIONE RIUSCITA a "));
        Serial.print(srv); Serial.print(':'); Serial.println(port);
        if (restoreMs != 0) {
            Serial.print(F("  -> Disponibilità ripristinata in "));
            Serial.print(restoreMs); Serial.println(F("ms dal drop"));
        }
        if (g_mqtt.publish(availTopic, "online", true)) {
            g_mqtt_last_online_ms = now;
        }
        if (!g_mqtt_discovery_sent) {
            Serial.print(F("  -> Pubblico MQTT discovery ... "));
            mqttPublishAllDiscovery();
            g_mqtt_discovery_sent = true;
            Serial.println(F("OK"));
        }
        // Forza publish dei 4x statici (reset_reason/boot/min_heap/stack_wm) al
        // primo TIMER B successivo alla connessione riuscita.
        g_throttle_needStaticOnConnect = true;
    } else {
        g_mqtt_consec_failures++;
        int mqttState = g_mqtt.state();
        Serial.print(F("  *** MQTT CONNECT FALLITA rc="));
        Serial.print(mqttState);
        Serial.print(F(" ("));
        switch (mqttState) {
            case MQTT_CONNECTION_TIMEOUT:     Serial.print(F("TIMEOUT")); break;
            case MQTT_CONNECTION_LOST:        Serial.print(F("CONNECTION_LOST")); break;
            case MQTT_CONNECT_FAILED:         Serial.print(F("CONNECT_FAILED")); break;
            case MQTT_DISCONNECTED:           Serial.print(F("DISCONNECTED")); break;
            case MQTT_CONNECT_BAD_PROTOCOL:   Serial.print(F("BAD_PROTOCOL")); break;
            case MQTT_CONNECT_BAD_CLIENT_ID:  Serial.print(F("BAD_CLIENT_ID")); break;
            case MQTT_CONNECT_UNAVAILABLE:    Serial.print(F("UNAVAILABLE")); break;
            case MQTT_CONNECT_BAD_CREDENTIALS:Serial.print(F("BAD_CREDENTIALS")); break;
            case MQTT_CONNECT_UNAUTHORIZED:   Serial.print(F("UNAUTHORIZED")); break;
            default: Serial.print(F("?")); break;
        }
        Serial.print(F(") WiFi.status=")); Serial.print(WiFi.status());
        Serial.print(F("  fail_consec=")); Serial.println(g_mqtt_consec_failures);
    }
    return ok;
}

// ============================================================================
// MQTT THROTTLING: implementazione corpi funzione
// ============================================================================

static void throttleResetAllState(void)
{
    unsigned long now = millis();
    g_throttle_lastTimerA_ms = now;
    g_throttle_lastTimerB_ms = now;
    g_throttle_lastSafety_ms = now;
    g_throttle_lastSupvForce_ms = now;
    g_throttle_lastStaticPublish_ms = 0;
    g_lastPubValid = false;
    memset(&g_lastPub, 0, sizeof(g_lastPub));
    g_throttle_needStaticOnConnect = true;
    Serial.println(F("THROTTLE: stato runtime resettato (zero-reboot apply). Prossimo ciclo = forceAll publish."));
}

static bool throttleExceedsThreshFloat(float cur, float last, float thresh)
{
    const bool curFin  = isfinite(cur);
    const bool lastFin = isfinite(last);
    if (curFin != lastFin) return true;
    if (!curFin) return false;
    if (thresh <= 0.0f) return true;
    return fabsf(cur - last) >= thresh;
}

static bool throttleExceedsThreshAbsInt(int cur, int last, uint8_t thresh)
{
    if (thresh == 0) return true;
    int d = cur - last;
    if (d < 0) d = -d;
    return (uint8_t)d >= thresh;
}

// Restituisce true se ALMENO UN valore (non-statico) supera la propria soglia
// oppure è cambiato (bool/counters). Usata dal dispatcher TIMER B per la
// policy ALL-or-NOTHING: se true -> chiamiamo mqttPublishAllDataSmart().
static bool throttleAnyExceedsThresholds(void)
{
    if (!g_lastPubValid) return true;

    // --- Ambientali float ---
    if (throttleExceedsThreshFloat(g_sht31_t,  g_lastPub.sht31_t,  g_config.throttle_thresh_temp_c))  return true;
    if (throttleExceedsThreshFloat(g_sht31_rh, g_lastPub.sht31_rh, g_config.throttle_thresh_rh_pct))  return true;
    if (throttleExceedsThreshFloat(g_sht31_dp, g_lastPub.sht31_dp, g_config.throttle_thresh_dew_c))   return true;
    if (throttleExceedsThreshFloat(g_sht31_hi, g_lastPub.sht31_hi, g_config.throttle_thresh_hi_c))    return true;
    if (throttleExceedsThreshFloat(g_sht31_ah, g_lastPub.sht31_ah, g_config.throttle_thresh_ah_gm3))  return true;

    // --- Heater + status bits (cambio = publish) ---
    if (g_sht31_heater != g_lastPub.sht31_heater) return true;
    if (g_status_bits != g_lastPub.sensor_status_bits) return true;

    // --- Supervisore level/counters (cambio = publish) ---
    if (g_supvLevel    != g_lastPub.supv_level)     return true;
    if (g_supvL1Count  != g_lastPub.supv_l1_count)  return true;
    if (g_supvL2Count  != g_lastPub.supv_l2_count)  return true;
    if (g_supvL3Count  != g_lastPub.supv_l3_count)  return true;
    if (g_supvResetCount != g_lastPub.supv_reset_count) return true;

    // --- Diagnostica dinamica ---
    if (WiFi.status() == WL_CONNECTED) {
        int8_t rssi = (int8_t)WiFi.RSSI();
        if (throttleExceedsThreshAbsInt((int)rssi, (int)g_lastPub.diag_wifi_rssi_dbm, g_config.throttle_thresh_rssi_dbm)) return true;
        uint32_t curIp = (uint32_t)WiFi.localIP();
        if (curIp != 0u && curIp != g_lastPub.diag_wifi_sta_ip_v4) return true;
    }
    float frag = (float)ESP.getHeapFragmentation();
    if (throttleExceedsThreshFloat(frag, g_lastPub.diag_heap_frag_pct, g_config.throttle_thresh_frag_pct)) return true;

    return false;
}

// ============================================================================
// TIMER A (1s invariato): solo uptime + WDT feeds (cambiano per definizione
// ogni secondo — nessun dead-band applicabile, mantenuti a 1Hz.)
// ============================================================================
static void mqttPublishUptimeWdtOnly(void)
{
    if (g_otaBusy) return;
    char topic[160]; char buf[40];
    unsigned long now = millis();

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(now / 1000UL));
    mqttBuildTopic(topic, sizeof(topic), "diag_uptime_s");
    size_t tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    if (g_mqtt.publish(topic, buf, true)) g_supvLastGoodPubMs = now;

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_diag_wdtFeedCount);
    mqttBuildTopic(topic, sizeof(topic), "diag_wdt_feeds");
    tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    if (g_mqtt.publish(topic, buf, true)) g_supvLastGoodPubMs = now;
}

// ============================================================================
// PUBLISH STATI on-connect (reset_reason, boot_count, min_heap ever,
// stack_watermark). Valori statici/quasi-statici che non cambiano a runtime.
// Pubblicati UNA SOLA VOLTA per ogni riconnessione MQTT riuscita, oppure
// on-change (es: min_heap scende).
// ============================================================================
static void mqttPublishStaticOnConnect(void)
{
    if (g_otaBusy) return;
    char topic[160]; char buf[40];
    unsigned long now = millis();

    snprintf(buf, sizeof(buf), "%u", (unsigned)g_rstReasonCached);
    mqttBuildTopic(topic, sizeof(topic), "diag_reset_reason_id");
    size_t tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    g_mqtt.publish(topic, buf, true);
    g_lastPub.diag_reset_reason_id = g_rstReasonCached;

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_diag_bootCount);
    mqttBuildTopic(topic, sizeof(topic), "diag_boot_count");
    tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    g_mqtt.publish(topic, buf, true);
    g_lastPub.diag_boot_count = g_diag_bootCount;

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_diag_minHeapEver);
    mqttBuildTopic(topic, sizeof(topic), "diag_min_heap_B");
    tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    g_mqtt.publish(topic, buf, true);
    g_lastPub.diag_min_heap_B = g_diag_minHeapEver;

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_diag_stackWmBytes);
    mqttBuildTopic(topic, sizeof(topic), "diag_stack_wm_B");
    tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    g_mqtt.publish(topic, buf, true);
    g_lastPub.diag_stack_wm_B = g_diag_stackWmBytes;

    if (WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        uint32_t ipRaw = (uint32_t)ip;
        snprintf(buf, sizeof(buf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        mqttBuildTopic(topic, sizeof(topic), "diag_wifi_sta_ip_v4");
        tl = strlen(topic);
        if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
        g_mqtt.publish(topic, buf, true);
        g_lastPub.diag_wifi_sta_ip_v4 = ipRaw;
    }

    g_throttle_lastStaticPublish_ms = now;
    g_supvLastGoodPubMs = now;
    Serial.println(F("THROTTLE: publish 5x statici on-connect completato (reset_reason, boot, min_heap, stack_wm, sta_ip)."));
}

// ============================================================================
// PUBLISH SMART (ALL-or-NOTHING policy confermata):
//   - Pubblica TUTTI i 21 topic non-statici (come la vecchia mqttPublishAllData
//     MA SENZA: diag_uptime_s, diag_wdt_feeds [TIMER A]; e SENZA i 4 statici
//     on-connect [mqttPublishStaticOnConnect]).
//   - Alla FINE aggiorna TUTTI i g_lastPub.* corrispondenti ai valori correnti.
//   - Questa funzione NON decide SE pubblicare (decisione nel dispatcher loop).
//     Viene chiamata solo se: (forceAll) || (forceSupv 10s) || (any > thresh).
// ============================================================================
static void mqttPublishAllDataSmart(void)
{
    if (g_otaBusy) return;
    unsigned long now = millis();

    // ---- 1. Ambientali (5 float + heater bool) ----
    if (isfinite(g_sht31_t))  mqttPublishScalar("sht31_temp_c", g_sht31_t, 3);
    if (isfinite(g_sht31_rh)) mqttPublishScalar("sht31_hum_rh", g_sht31_rh, 2);
    if (isfinite(g_sht31_dp)) mqttPublishScalar("sht31_dew_c", g_sht31_dp, 2);
    if (isfinite(g_sht31_hi)) mqttPublishScalar("sht31_heat_index_c", g_sht31_hi, 2);
    if (isfinite(g_sht31_ah)) mqttPublishScalar("sht31_abs_hum_gm3", g_sht31_ah, 3);

    {
        char topic[160]; char value[8];
        mqttBuildTopic(topic, sizeof(topic), "sht31_heater");
        size_t tl = strlen(topic);
        if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
        snprintf(value, sizeof(value), "%u", g_sht31_heater ? 1u : 0u);
        if (g_mqtt.publish(topic, value, true)) g_supvLastGoodPubMs = now;
    }

    // ---- 2. Stato sensore (3x formati ridondanti, come deciso) ----
    char topic[160]; char value[16];
    mqttBuildTopic(topic, sizeof(topic), "sensor_status_bitmask");
    size_t tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    snprintf(value, sizeof(value), "0x%02X", (unsigned)g_status_bits);
    if (g_mqtt.publish(topic, value, true)) g_supvLastGoodPubMs = now;

    mqttBuildTopic(topic, sizeof(topic), "sensor_status_hexbin");
    tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    if (g_mqtt.publish(topic, sensorStatusHexBin(g_status_bits), true)) g_supvLastGoodPubMs = now;

    mqttBuildTopic(topic, sizeof(topic), "sensor_status_list");
    tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    if (g_mqtt.publish(topic, sensorStatusList(g_status_bits), true)) g_supvLastGoodPubMs = now;

    // ---- 3. Supervisore (level + 4 counters sempre; trip/lastgood ogni volta perché ALL-or-NOTHING) ----
    char supvTopic[160]; char supvBuf[40];
    const char* lvlStr = supvLvlStr(g_supvLevel);

    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_level");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, lvlStr, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_supvL1Count);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_l1_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_supvL2Count);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_l2_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_supvL3Count);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_l3_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_supvResetCount);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_reset_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    // supv trip/lastgood: ricalcolati da now come nella funzione originale
    unsigned long tripMs = 0;
    if (g_supvTripped && g_supvTripMs != 0) {
        tripMs = now - g_supvTripMs;
    }
    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)tripMs);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_trip_ms");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    unsigned long connAgoMs = (g_supvLastGoodConnMs == 0) ? 0 : now - g_supvLastGoodConnMs;
    unsigned long pubAgoMs  = (g_supvLastGoodPubMs  == 0) ? 0 : now - g_supvLastGoodPubMs;
    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)connAgoMs);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_last_good_conn_ms");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)pubAgoMs);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_last_good_pub_ms");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    // ---- 4. Diagnostica dinamica (RSSI + IP STA + frag_pct) ----
    if (WiFi.status() == WL_CONNECTED) {
        int8_t rssi = (int8_t)WiFi.RSSI();
        snprintf(supvBuf, sizeof(supvBuf), "%d", (int)rssi);
        mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_wifi_rssi_dbm");
        tl = strlen(supvTopic);
        if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
        g_mqtt.publish(supvTopic, supvBuf, true);
        g_lastPub.diag_wifi_rssi_dbm = rssi;

        IPAddress ip = WiFi.localIP();
        uint32_t ipRaw = (uint32_t)ip;
        if (ipRaw != 0u && ipRaw != g_lastPub.diag_wifi_sta_ip_v4) {
            snprintf(supvBuf, sizeof(supvBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
            mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_wifi_sta_ip_v4");
            tl = strlen(supvTopic);
            if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
            g_mqtt.publish(supvTopic, supvBuf, true);
            g_lastPub.diag_wifi_sta_ip_v4 = ipRaw;
        }
    }

    float fragPct = (float)ESP.getHeapFragmentation();
    snprintf(supvBuf, sizeof(supvBuf), "%u", (unsigned)((uint8_t)fragPct));
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_heap_frag_pct");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    g_mqtt.publish(supvTopic, supvBuf, true);
    g_lastPub.diag_heap_frag_pct = fragPct;

    // ---- 5. Aggiorna LAST-PUBLISHED shadow (tutti i campi) ----
    g_lastPub.sht31_t  = g_sht31_t;
    g_lastPub.sht31_rh = g_sht31_rh;
    g_lastPub.sht31_dp = g_sht31_dp;
    g_lastPub.sht31_hi = g_sht31_hi;
    g_lastPub.sht31_ah = g_sht31_ah;
    g_lastPub.sht31_heater = g_sht31_heater;
    g_lastPub.sensor_status_bits = g_status_bits;
    g_lastPub.supv_level  = g_supvLevel;
    g_lastPub.supv_l1_count = g_supvL1Count;
    g_lastPub.supv_l2_count = g_supvL2Count;
    g_lastPub.supv_l3_count = g_supvL3Count;
    g_lastPub.supv_reset_count = g_supvResetCount;
    // (reset_reason/boot/min_heap/stack_wm = statici, gestiti separatamente in mqttPublishStaticOnConnect)
    // (RSSI + frag = già aggiornati sopra durante publish)

    g_lastPubValid = true;
}

static void mqttPublishAllData(void)
{
    if (isfinite(g_sht31_t))  mqttPublishScalar("sht31_temp_c", g_sht31_t, 3);
    if (isfinite(g_sht31_rh)) mqttPublishScalar("sht31_hum_rh", g_sht31_rh, 2);
    if (isfinite(g_sht31_dp)) mqttPublishScalar("sht31_dew_c", g_sht31_dp, 2);
    if (isfinite(g_sht31_hi)) mqttPublishScalar("sht31_heat_index_c", g_sht31_hi, 2);
    if (isfinite(g_sht31_ah)) mqttPublishScalar("sht31_abs_hum_gm3", g_sht31_ah, 3);

    {
        char topic[160]; char value[8];
        mqttBuildTopic(topic, sizeof(topic), "sht31_heater");
        size_t tl = strlen(topic);
        if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
        snprintf(value, sizeof(value), "%u", g_sht31_heater ? 1u : 0u);
        if (g_mqtt.publish(topic, value, true)) g_supvLastGoodPubMs = millis();
    }

    char topic[160]; char value[16];
    mqttBuildTopic(topic, sizeof(topic), "sensor_status_bitmask");
    size_t tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    snprintf(value, sizeof(value), "0x%02X", (unsigned)g_status_bits);
    if (g_mqtt.publish(topic, value, true)) g_supvLastGoodPubMs = millis();

    mqttBuildTopic(topic, sizeof(topic), "sensor_status_hexbin");
    tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    if (g_mqtt.publish(topic, sensorStatusHexBin(g_status_bits), true)) g_supvLastGoodPubMs = millis();

    mqttBuildTopic(topic, sizeof(topic), "sensor_status_list");
    tl = strlen(topic);
    if (tl + 7 < sizeof(topic)) { memcpy(topic + tl, "/state", 7); }
    if (g_mqtt.publish(topic, sensorStatusList(g_status_bits), true)) g_supvLastGoodPubMs = millis();

    // --- Entità supervisore MQTT ---
    char supvTopic[160]; char supvBuf[40];
    const char* lvlStr = supvLvlStr(g_supvLevel);
    unsigned long now = millis();

    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_level");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, lvlStr, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_supvL1Count);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_l1_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_supvL2Count);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_l2_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_supvL3Count);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_l3_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_supvResetCount);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_reset_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    unsigned long tripMs = 0;
    if (g_supvTripped && g_supvTripMs != 0) {
        tripMs = now - g_supvTripMs;
    }
    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)tripMs);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_trip_ms");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    unsigned long connAgoMs = (g_supvLastGoodConnMs == 0) ? 0 : now - g_supvLastGoodConnMs;
    unsigned long pubAgoMs  = (g_supvLastGoodPubMs  == 0) ? 0 : now - g_supvLastGoodPubMs;
    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)connAgoMs);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_last_good_conn_ms");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)pubAgoMs);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "supv_last_good_pub_ms");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    if (g_mqtt.publish(supvTopic, supvBuf, true)) g_supvLastGoodPubMs = now;

    // ========================================================================
    // TELEMETRIA DIAGNOSTICA CRASH LOGGER (Fase 3)
    // ========================================================================
    snprintf(supvBuf, sizeof(supvBuf), "%u", (unsigned)g_rstReasonCached);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_reset_reason_id");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    g_mqtt.publish(supvTopic, supvBuf, true);

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_diag_bootCount);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_boot_count");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    g_mqtt.publish(supvTopic, supvBuf, true);

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)(now / 1000UL));
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_uptime_s");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    g_mqtt.publish(supvTopic, supvBuf, true);

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_diag_minHeapEver);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_min_heap_B");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    g_mqtt.publish(supvTopic, supvBuf, true);

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_diag_stackWmBytes);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_stack_wm_B");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    g_mqtt.publish(supvTopic, supvBuf, true);

    snprintf(supvBuf, sizeof(supvBuf), "%lu", (unsigned long)g_diag_wdtFeedCount);
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_wdt_feeds");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    g_mqtt.publish(supvTopic, supvBuf, true);

    if (WiFi.status() == WL_CONNECTED) {
        snprintf(supvBuf, sizeof(supvBuf), "%d", (int)WiFi.RSSI());
        mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_wifi_rssi_dbm");
        tl = strlen(supvTopic);
        if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
        g_mqtt.publish(supvTopic, supvBuf, true);
    }

    snprintf(supvBuf, sizeof(supvBuf), "%u", (unsigned)ESP.getHeapFragmentation());
    mqttBuildTopic(supvTopic, sizeof(supvTopic), "diag_heap_frag_pct");
    tl = strlen(supvTopic);
    if (tl + 7 < sizeof(supvTopic)) { memcpy(supvTopic + tl, "/state", 7); }
    g_mqtt.publish(supvTopic, supvBuf, true);
}

// ============================================================================
// Supervisore MQTT 4-livelli + Watchdog Hardware
//   Trigger:
//     (A) WiFi connesso ma MQTT non riconnesso per > 30s
//     (B) MQTT connesso ma nessuna publish andata a buon fine per > 30s
//   Recovery escalation:
//     IDLE : tutto ok
//     L1   : riconnetti soft (forza mqttEnsureConnected bypassando backoff)
//     L2   : disconnect MQTT forzato + riconnetti
//     L3   : power-cycle WiFi: WIFI_OFF -> delay -> WIFI_AP_STA -> riconnetti
//     L4   : se dopo 120s ancora in stato trippato -> ESP.restart()
// ============================================================================
static const char* supvLvlStr(SupvLevel l)
{
    switch (l) {
        case SUPV_LVL_IDLE: return "IDLE";
        case SUPV_LVL_L1:   return "L1-Reconnect";
        case SUPV_LVL_L2:   return "L2-ForceDisc";
        case SUPV_LVL_L3:   return "L3-WifiCycle";
        case SUPV_LVL_L4:   return "L4-HWReset";
        default:            return "?";
    }
}

static void supervisorClearTrip(void)
{
    if (g_supvTripped) {
        Serial.print(F("SUPV: stato ripristinato (era "));
        Serial.print(supvLvlStr(g_supvLevel));
        Serial.println(F(") -> IDLE"));
    }
    g_supvTripped  = false;
    g_supvLevel    = SUPV_LVL_IDLE;
    g_supvTripMs   = 0;
    g_supvL1InProgress = false;
    g_supvL2State = L2S_IDLE;
    g_supvL3State = L3S_IDLE;
    g_supvL2StepMs = 0;
    g_supvL3StepMs = 0;
}

static bool supervisorIsStalled(unsigned long now)
{
    if (WiFi.status() != WL_CONNECTED) {
        // --------------------------------------------------------------
        // WiFi non c'e' -> SOSPENDIAMO il rilevamento stall (return false)
        // MA NON resettiamo g_supvTripMs / g_supvTripped / LastGoodConn/Pub
        // perche' micro-glitch ripetuti (<120s tra loro) avrebbero altrimenti
        // impedito PER SEMPRE l'escalation a L4 (reset hardware).
        // Il timer L4 continua a decorrere in tempo reale.
        // --------------------------------------------------------------
        if (g_mqtt_last_online_ms != 0 && (now - g_mqtt_last_online_ms) >= (2UL * MQTT_ONLINE_REFRESH_MS)) {
            g_mqtt_last_online_ms = now;
        }
        return false;
    }
    // Inizializzazione (0): diamo un margine usando now
    unsigned long connMs = (g_supvLastGoodConnMs == 0) ? now : g_supvLastGoodConnMs;
    unsigned long pubMs  = (g_supvLastGoodPubMs  == 0) ? now : g_supvLastGoodPubMs;
    unsigned long onlineMs = (g_mqtt_last_online_ms == 0) ? now : g_mqtt_last_online_ms;

    bool stallConn = (!g_mqtt.connected()) && (now - connMs) >= SUPV_CONN_STALL_MS;
    bool stallPub  = ( g_mqtt.connected()) && (now - pubMs ) >= SUPV_PUB_STALL_MS;

    // --- Fallback G4: SOCKET ZOMBIE ---
    // Anche se connected() ritorna true, se non siamo riusciti a pubblicare
    // "online" per più di 2x il refresh period, c'è un 99% di probabilità
    // che la socket TCP sia morta silenziosamente.
    bool stallOnline = (g_mqtt.connected()) &&
                       (g_mqtt_last_online_ms != 0) &&
                       ((now - onlineMs) >= (2UL * MQTT_ONLINE_REFRESH_MS));

    if (stallOnline && !stallConn && !stallPub) {
        Serial.print(F("SUPV: TRIGGER FALLBACK ONLINE STALL! (last online refresh > 2x periodo) "));
        Serial.print(F("delta=")); Serial.print(now - onlineMs);
        Serial.print(F("ms soglia=")); Serial.print(2UL * MQTT_ONLINE_REFRESH_MS); Serial.println(F("ms"));
    }

    return stallConn || stallPub || stallOnline;
}

static void supervisorL1_forceReconnect(unsigned long now)
{
    g_supvL1Count++;
    g_supvLastL1Ms = now;
    g_mqtt_lastAttemptMs = 0;
    (void)mqttEnsureConnected();
}

// Avvia L2: disconnect sincrono, delay asincrono via state machine in supervisorPoll
static bool supervisorL2_start(unsigned long now)
{
    g_supvL2Count++;
    g_supvLastL2Ms = now;
    Serial.print(F("SUPV L2: disconnect forzato MQTT (count="));
    Serial.print(g_supvL2Count);
    Serial.println(F(")"));
    if (g_mqtt.connected()) {
        char availTopic[160];
        mqttBuildAvailTopic(availTopic, sizeof(availTopic));
        (void)g_mqtt.publish(availTopic, "offline", true);
        g_mqtt.disconnect();
    }
    g_mqtt_discovery_sent = false;
    g_mqtt_lastAttemptMs = 0;
    g_supvL2StepMs = now;
    g_supvL2State = L2S_WAIT_COOLDOWN;
    return false; // non ancora completo: call poll finché ritorna true
}

// Poll L2 state machine. Ritorna true quando L2 è conclusa.
static bool supervisorL2_poll(unsigned long now)
{
    switch (g_supvL2State) {
        case L2S_IDLE:
            return true;
        case L2S_WAIT_COOLDOWN:
            if ((now - g_supvL2StepMs) >= SUPV_L2_DELAY_MS) {
                (void)mqttEnsureConnected();
                g_supvL2State = L2S_DONE;
                return true;
            }
            return false;
        case L2S_DONE:
            return true;
    }
    return true;
}

// Avvia L3 wifi-cycle asincrono: passo 1 subito, restanti in poll()
static bool supervisorL3_start(unsigned long now)
{
    g_supvL3Count++;
    g_supvLastL3Ms = now;
    Serial.print(F("SUPV L3: power-cycle WiFi (count="));
    Serial.print(g_supvL3Count);
    Serial.println(F(")"));
    if (g_mqtt.connected()) {
        char availTopic[160];
        mqttBuildAvailTopic(availTopic, sizeof(availTopic));
        (void)g_mqtt.publish(availTopic, "offline", true);
        g_mqtt.disconnect();
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    g_supvL3StepMs = now;
    g_supvL3State = L3S_WAIT_WIFIOFF;
    g_mqtt_discovery_sent = false;
    g_mqtt_lastAttemptMs = 0;
    return false;
}

// Poll L3 state machine. Ritorna true quando L3 è conclusa.
static bool supervisorL3_poll(unsigned long now)
{
    switch (g_supvL3State) {
        case L3S_IDLE:
            return true;
        case L3S_WAIT_WIFIOFF:
            if ((now - g_supvL3StepMs) >= SUPV_L3_OFF_MS) {
                WiFi.persistent(false);
                WiFi.setAutoReconnect(true);
                WiFi.mode(WIFI_STA);
                WiFi.begin(g_config.wifi_ssid, g_config.wifi_password);
                WiFi.mode(WIFI_AP_STA);
                WiFi.softAP(g_config.ap_ssid, g_config.ap_password);
                g_supvL3StepMs = now;
                g_supvL3State = L3S_WAIT_POST;
            }
            return false;
        case L3S_WAIT_POST:
            if ((now - g_supvL3StepMs) >= SUPV_L3_POST_MS) {
                g_supvL3State = L3S_DONE;
                return true;
            }
            return false;
        case L3S_DONE:
            return true;
    }
    return true;
}

static void supervisorL4_hardReset(unsigned long now)
{
    g_supvResetCount++;
    Serial.print(F("SUPV L4: RESET HARDWARE ESP (reset count="));
    Serial.print(g_supvResetCount);
    Serial.print(F("), trip durata ~"));
    Serial.print(now - g_supvTripMs);
    Serial.println(F("ms"));
    Serial.flush();
    delay(400);
    ESP.restart();
}

static void supervisorPoll(void)
{
    if (g_otaBusy) return;
    unsigned long now = millis();

    // ================================================================
    // CHECK L4 SEMPRE ATTIVO — PRIMA di qualsiasi altra logica
    // Non importa se WiFi non c'e' o siamo in L2/L3 state machine:
    // dopo SUPV_L4_RESET_MS dal TRIP INIZIALE, RESET HARDWARE.
    // ================================================================
    if (g_supvTripped && (now - g_supvTripMs) >= SUPV_L4_RESET_MS) {
        g_supvLevel = SUPV_LVL_L4;
        supervisorL4_hardReset(now);
        return;
    }

    bool stalled = supervisorIsStalled(now);

    // Pompa state machine L2/L3 anche se non siamo più in trip,
    // per consumare l'ultima istanza avviata.
    if (g_supvL2State != L2S_IDLE) (void)supervisorL2_poll(now);
    if (g_supvL3State != L3S_IDLE) {
        (void)supervisorL3_poll(now);
        if (g_supvL3State != L3S_DONE) {
            // Durante L3 non possiamo permetterci altre azioni sul supervisore
            // perché il WiFi è in transizione. Torna al ciclo prossimo.
            return;
        }
    }

    if (!stalled) {
        if (g_supvTripped) {
            // Check L4 SEMPRE, anche se !stalled (caso WiFi assente/glitch che altrimenti impedisce l'escalation
            if ((now - g_supvTripMs) >= SUPV_L4_RESET_MS) {
                g_supvLevel = SUPV_LVL_L4;
                supervisorL4_hardReset(now);
                return;
            }
            // Clear trip solo se siamo veramente guariti: connessione OK + pubblicazioni OK per almeno 3s
            bool recovered = g_mqtt.connected() &&
                            (now - g_supvLastGoodConnMs) < 3000UL &&
                            (now - g_supvLastGoodPubMs)  < 3000UL;
            if (recovered) {
                Serial.print(F("SUPV: TRIP CLEAR (recuperato, lastGoodConn="));
                Serial.print(now - g_supvLastGoodConnMs);
                Serial.print(F("ms lastGoodPub="));
                Serial.print(now - g_supvLastGoodPubMs);
                Serial.println(F("ms)"));
                supervisorClearTrip();
            }
        }
        return;
    }

    if (!g_supvTripped) {
        g_supvTripped = true;
        g_supvLevel = SUPV_LVL_IDLE;
        g_supvTripMs = now;
        Serial.print(F("SUPV: TRIP! "));
        if (!g_mqtt.connected()) {
            Serial.print(F("MQTT non connesso > "));
            Serial.print(now - g_supvLastGoodConnMs);
        } else {
            Serial.print(F("Publish stalled > "));
            Serial.print(now - g_supvLastGoodPubMs);
        }
        Serial.println(F(" ms"));
    }

    // Check L4 timeout — 120s dal trip
    if ((now - g_supvTripMs) >= SUPV_L4_RESET_MS) {
        g_supvLevel = SUPV_LVL_L4;
        supervisorL4_hardReset(now);
        return;
    }

    // Transizioni livello
    switch (g_supvLevel) {
        case SUPV_LVL_IDLE:
            g_supvLevel = SUPV_LVL_L1;
            Serial.print(F("SUPV: avvio L1 (tentativo "));
            Serial.print(g_supvL1Count + 1);
            Serial.println(F(")"));
            supervisorL1_forceReconnect(now);
            break;

        case SUPV_LVL_L1:
            if ((now - g_supvLastL1Ms) >= SUPV_L1_COOLDOWN_MS) {
                g_supvLevel = SUPV_LVL_L2;
                Serial.print(F("SUPV: escalation L1 -> L2 (tentativo "));
                Serial.print(g_supvL2Count + 1);
                Serial.println(F(")"));
                (void)supervisorL2_start(now);
            } else if ((now - g_supvLastL1Ms) >= 2500) {
                // Refresh L1 ogni 2.5s finché siamo a livello 1
                supervisorL1_forceReconnect(now);
            }
            break;

        case SUPV_LVL_L2:
            if (!supervisorL2_poll(now)) break; // ancora in attesa asincrona
            if ((now - g_supvLastL2Ms) >= SUPV_L2_COOLDOWN_MS) {
                g_supvLevel = SUPV_LVL_L3;
                Serial.print(F("SUPV: escalation L2 -> L3 (tentativo "));
                Serial.print(g_supvL3Count + 1);
                Serial.println(F(")"));
                (void)supervisorL3_start(now);
            }
            break;

        case SUPV_LVL_L3:
            if (!supervisorL3_poll(now)) break; // ancora in attesa asincrona
            if ((now - g_supvLastL3Ms) >= SUPV_L3_COOLDOWN_MS) {
                // Torniamo a L1 per continuare la raffica prima di arrivare a L4
                Serial.print(F("SUPV: loop L3 -> di nuovo L1 dopo cooldown, tempo rimasto a trip: "));
                Serial.print(now - g_supvTripMs);
                Serial.print(F("/"));
                Serial.print(SUPV_L4_RESET_MS);
                Serial.println(F("ms"));
                g_supvLevel = SUPV_LVL_L1;
                supervisorL1_forceReconnect(now);
            }
            break;

        case SUPV_LVL_L4:
            supervisorL4_hardReset(now);
            break;
    }
}

static void supervisorInit(void)
{
    unsigned long now = millis();
    g_supvLastGoodConnMs = now;
    g_supvLastGoodPubMs  = now;
    supervisorClearTrip();

#if defined(ESP8266)
    ESP.wdtEnable(SUPV_WDT_HW_MS);
    Serial.print(F("WDT HW: ARMATO a "));
    Serial.print(SUPV_WDT_HW_MS);
    Serial.println(F(" ms"));
#endif
}

static inline void supervisorWatchdogFeed(void)
{
#if defined(ESP8266)
    ESP.wdtFeed();
#endif
}

// ============================================================================
// Tracking stato RICONNESSIONE WiFi Robusta + Fallback STA polling
// ============================================================================
static bool          g_net_fallback_mode       = false;
static unsigned long g_net_last_fallback_poll_ms = 0;
static uint8_t       g_net_fallback_retry_count = 0;
static unsigned long g_net_boot_sta_last_attempt_ms = 0;
static unsigned long g_net_last_sta_connected_ms = 0;

// ============================================================================
// (Rimossi DS18B20 + DHT22 a Ottobre 2026: sostituiti da singolo SHT31-D I²C)
// Driver SHT31: sht31_init() e sht31_poll() sono implementati sopra, nella
// sezione Helper C / polling state machine.
// ============================================================================

// ============================================================================
// Network
// ============================================================================
// Forward declarations
static void netHardResetWifi(void)
{
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    unsigned long t0 = millis();
    while ((millis() - t0) < WIFI_STA_HARD_RESET_OFF_MS) {
        delay(50);
        supervisorWatchdogFeed();
        g_diag_wdtFeedCount++;
    }
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
}

static bool netTrySTAOnce(uint8_t attemptIndex, bool isBootPhase)
{
    (void)isBootPhase;

    WiFi.mode(WIFI_STA);
    WiFi.begin(g_config.wifi_ssid, g_config.wifi_password);
    g_net_boot_sta_last_attempt_ms = millis();

    Serial.print(F("  STA tentativo #"));
    Serial.print(attemptIndex);
    Serial.print(F(" [timeout="));
    Serial.print(WIFI_STA_PER_TRY_TIMEOUT_MS);
    Serial.print(F("ms]  "));

    unsigned long t0 = millis();
    uint8_t dots = 0;
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_STA_PER_TRY_TIMEOUT_MS) {
        delay(250);
        supervisorWatchdogFeed();
        g_diag_wdtFeedCount++;
        Serial.print('.');
        dots++;
        if (dots >= 30) {
            Serial.println();
            Serial.print(F("    ... "));
            dots = 0;
        }
    }
    Serial.println();

    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
        Serial.print(F("    -> OK  IP="));
        Serial.println(WiFi.localIP());
        WiFi.setAutoReconnect(true);
        return true;
    } else {
        Serial.print(F("    -> FALLITA  WiFi.status=0x"));
        Serial.print((unsigned)st, HEX);
        Serial.print(F(" ("));
        switch (st) {
            case WL_IDLE_STATUS:     Serial.print(F("IDLE")); break;
            case WL_NO_SSID_AVAIL:   Serial.print(F("NO_SSID_AVAIL")); break;
            case WL_SCAN_COMPLETED:  Serial.print(F("SCAN_COMPLETED")); break;
            case WL_CONNECTED:       Serial.print(F("CONNECTED")); break;
            case WL_CONNECT_FAILED:  Serial.print(F("CONNECT_FAILED")); break;
            case WL_CONNECTION_LOST: Serial.print(F("CONNECTION_LOST")); break;
            case WL_DISCONNECTED:    Serial.print(F("DISCONNECTED")); break;
            default:                 Serial.print(F("UNKNOWN")); break;
        }
        Serial.println(F(")"));
        return false;
    }
}

static bool netStartSTAWithRetries(void)
{
    Serial.println();
    Serial.println(F("=============================================================="));
    Serial.println(F(" WIFI: connessione STA ROBUSTA (1+10 tentativi con hard reset)"));
    Serial.print(F("  SSID=\""));
    Serial.print(g_config.wifi_ssid);
    Serial.print(F("\"  per-try-timeout="));
    Serial.print(WIFI_STA_PER_TRY_TIMEOUT_MS);
    Serial.print(F("ms  inter-retry-delay="));
    Serial.print(WIFI_STA_INTER_RETRY_DELAY_MS);
    Serial.println(F("ms"));
    Serial.println(F("=============================================================="));

    const uint8_t totalAttempts = 1u + WIFI_STA_RETRY_COUNT;

    for (uint8_t i = 1; i <= totalAttempts; i++)
    {
        if (i > 1) {
            Serial.print(F("  [inter-retry] hard reset WiFi + pausa "));
            Serial.print(WIFI_STA_INTER_RETRY_DELAY_MS);
            Serial.println(F("ms ..."));
            netHardResetWifi();
            unsigned long t0 = millis();
            while ((millis() - t0) < WIFI_STA_INTER_RETRY_DELAY_MS) {
                delay(50);
                supervisorWatchdogFeed();
                g_diag_wdtFeedCount++;
            }
        }

        bool ok = netTrySTAOnce(i, true);
        if (ok) {
            Serial.print(F("  SUCCESSO al tentativo #"));
            Serial.print(i);
            Serial.print(F(" / "));
            Serial.println(totalAttempts);
            return true;
        }
    }

    Serial.print(F("  FALLIMENTO TOTALE dopo "));
    Serial.print(totalAttempts);
    Serial.println(F(" tentativi consecutivi."));
    Serial.println(F("  -> Passaggio a modalità FALLBACK AP+STA (STA continuerà in background)"));
    return false;
}

static void netFallbackPollRetry(void)
{
    #define NET_STA_GRACE_PERIOD_MS  (90UL * 1000UL)
    const unsigned long NOW = millis();

    // Aggiorniamo istante ultima conferma STA connesso
    if (WiFi.status() == WL_CONNECTED) {
        g_net_last_sta_connected_ms = NOW;
    }

    // ==================================================================
    // CASO A: Siamo già in modalità FALLBACK (AP attivo, retry loop STA)
    // ==================================================================
    if (g_net_fallback_mode) {
        // STA si è riconnesso nel frattempo? Disattiviamo AP e torniamo a STA sola.
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print(F("NET-FALLBACK: STA CONNESSO in background! IP="));
            Serial.println(WiFi.localIP());
            Serial.print(F("  (dopo "));
            Serial.print(g_net_fallback_retry_count);
            Serial.println(F(" retry in fallback)"));
            WiFi.setAutoReconnect(true);
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_STA);
            WiFi.setAutoReconnect(true);
            Serial.println(F("AP disattivato (ora sola modalità Station)"));
            g_net_fallback_mode = false;
            g_net_fallback_retry_count = 0;
            g_net_last_fallback_poll_ms = 0;
            return;
        }

        // Altrimenti, attendi periodo di poll prima del prossimo retry STA
        if (g_net_last_fallback_poll_ms != 0 && (NOW - g_net_last_fallback_poll_ms) < WIFI_FALLBACK_POLL_PERIOD_MS) {
            return;
        }

        g_net_last_fallback_poll_ms = NOW;
        g_net_fallback_retry_count++;

        Serial.print(F("NET-FALLBACK: retry STA #"));
        Serial.print(g_net_fallback_retry_count);
        Serial.print(F(" (dopo "));
        Serial.print(WIFI_FALLBACK_POLL_PERIOD_MS);
        Serial.println(F("ms)"));

        if (WiFi.status() != WL_IDLE_STATUS && WiFi.status() != WL_DISCONNECTED) {
            netHardResetWifi();
        }

        WiFi.mode(WIFI_AP_STA);
        (void)WiFi.softAP(g_config.ap_ssid, g_config.ap_password);

        WiFi.begin(g_config.wifi_ssid, g_config.wifi_password);
        g_net_boot_sta_last_attempt_ms = NOW;

        Serial.print(F("  begin() inviato, WiFi.status=0x"));
        Serial.print((unsigned)WiFi.status(), HEX);
        Serial.println(F(" (verifica al prossimo poll tra 30s)"));
        return;
    }

    // ==================================================================
    // CASO B: NON siamo in fallback. Se STA cade per >90s → attiviamo AP.
    // ==================================================================
    if (WiFi.status() != WL_CONNECTED) {
        if (g_net_last_sta_connected_ms == 0) {
            // Primo avvio senza STA: fissiamo il riferimento temporale per evitare falsi trigger
            g_net_last_sta_connected_ms = NOW;
            return;
        }
        unsigned long delta = NOW - g_net_last_sta_connected_ms;
        if (delta >= NET_STA_GRACE_PERIOD_MS) {
            Serial.print(F("NET: STA SCONNESSO da "));
            Serial.print(delta / 1000UL);
            Serial.println(F("s. Attivazione AP per recupero + retry loop STA..."));
            g_net_fallback_mode = true;
            g_net_fallback_retry_count = 0;
            g_net_last_fallback_poll_ms = NOW;
            WiFi.mode(WIFI_AP_STA);
            WiFi.softAP(g_config.ap_ssid, g_config.ap_password);
            WiFi.begin(g_config.wifi_ssid, g_config.wifi_password);
            g_net_boot_sta_last_attempt_ms = NOW;
            Serial.print(F("AP OK   SSID="));
            Serial.print(g_config.ap_ssid);
            Serial.print(F("  IP="));
            Serial.println(WiFi.softAPIP());
            Serial.println(F("NET-FALLBACK: modalità attivata per perdita STA prolungata"));
        }
    }
}

static void netStartSTAorAP(void)
{
    bool staOk = netStartSTAWithRetries();
    const unsigned long now = millis();

    if (staOk) {
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        Serial.print(F("STA OK  IP="));
        Serial.println(WiFi.localIP());
        Serial.println(F("AP: disattivato (STA connessa regolarmente al boot)"));
        g_net_last_sta_connected_ms = now;
        g_net_fallback_mode = false;
        g_net_fallback_retry_count = 0;
        g_net_last_fallback_poll_ms = 0;
    } else {
        g_net_fallback_mode = true;
        g_net_fallback_retry_count = 0;
        g_net_last_fallback_poll_ms = 0;
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(g_config.ap_ssid, g_config.ap_password);
        Serial.print(F("AP OK   SSID="));
        Serial.print(g_config.ap_ssid);
        Serial.print(F("  IP="));
        Serial.println(WiFi.softAPIP());
        Serial.print(F("NET-FALLBACK: modalità attivata — prossimo tentativo STA tra "));
        Serial.print(WIFI_FALLBACK_POLL_PERIOD_MS / 1000UL);
        Serial.println(F(" secondi (loop)"));
    }
}

// ============================================================================
// JSON escaping helper
// ============================================================================
static size_t jsonEscapeAppend(char* dst, size_t dstLen, const char* src)
{
    if (!dst || dstLen == 0) return 0;
    size_t w = 0;
    dst[0] = '\0';
    if (!src) return 0;
    for (size_t i = 0; src[i] != '\0' && w + 8 < dstLen; i++) {
        uint8_t c = (uint8_t)src[i];
        switch (c) {
            case '"':  dst[w++] = '\\'; dst[w++] = '"'; break;
            case '\\': dst[w++] = '\\'; dst[w++] = '\\'; break;
            case '\n': dst[w++] = '\\'; dst[w++] = 'n'; break;
            case '\r': dst[w++] = '\\'; dst[w++] = 'r'; break;
            case '\t': dst[w++] = '\\'; dst[w++] = 't'; break;
            case '\b': dst[w++] = '\\'; dst[w++] = 'b'; break;
            case '\f': dst[w++] = '\\'; dst[w++] = 'f'; break;
            default:
                if (c < 0x20) {
                    w += snprintf(dst + w, dstLen - w, "\\u%04X", (unsigned)c);
                } else {
                    dst[w++] = (char)c;
                }
        }
    }
    if (w < dstLen) dst[w] = '\0';
    return w;
}

// ============================================================================
// Handler HTTP
// ============================================================================
static void handleRoot(void)
{
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    g_http.send_P(200, "text/html; charset=utf-8", INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

static void handleApiLabels(void)
{
    static char resp[1800];
    static char esc1[280];
    static char esc2[280];

    jsonEscapeAppend(esc1, sizeof(esc1), g_config.device_name);
    jsonEscapeAppend(esc2, sizeof(esc2), g_config.sensor_sht31_name);

    snprintf(resp, sizeof(resp),
        "{\"ok\":true,"
        "\"device_name\":\"%s\","
        "\"sht31\":\"%s\"}",
        esc1, esc2);

    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    g_http.send(200, "application/json; charset=utf-8", resp);
}

static void handleApiConfigGet(void)
{
    static char resp[8192];
    static char escDN[280];
    static char escS1[280];
    static char escMS[420];
    static char escMU[320];
    static char escMP[420];
    static char escDI[420];
    static char escWifiSS[220];
    static char escWifiPW[420];
    static char escApSS[220];
    static char escApPW[420];

    jsonEscapeAppend(escDN, sizeof(escDN), g_config.device_name);
    jsonEscapeAppend(escS1, sizeof(escS1), g_config.sensor_sht31_name);
    jsonEscapeAppend(escMS, sizeof(escMS), g_config.mqtt_server);
    jsonEscapeAppend(escMU, sizeof(escMU), g_config.mqtt_user);
    jsonEscapeAppend(escMP, sizeof(escMP), g_config.mqtt_password);
    jsonEscapeAppend(escDI, sizeof(escDI), g_config.mqtt_device_id);
    jsonEscapeAppend(escWifiSS, sizeof(escWifiSS), g_config.wifi_ssid);
    jsonEscapeAppend(escWifiPW, sizeof(escWifiPW), g_config.wifi_password);
    jsonEscapeAppend(escApSS, sizeof(escApSS), g_config.ap_ssid);
    jsonEscapeAppend(escApPW, sizeof(escApPW), g_config.ap_password);

    snprintf(resp, sizeof(resp),
        "{\"ok\":true,"
        "\"device_name\":\"%s\","
        "\"sensor_sht31_name\":\"%s\","
        "\"mqtt_server\":\"%s\","
        "\"mqtt_port\":%u,"
        "\"mqtt_user\":\"%s\","
        "\"mqtt_password\":\"%s\","
        "\"mqtt_device_id\":\"%s\","
        "\"throttle_timer_data_base_ms\":%lu,"
        "\"throttle_safety_max_interval_ms\":%lu,"
        "\"throttle_supv_force_interval_ms\":%lu,"
        "\"throttle_thresh_temp_c\":%.2f,"
        "\"throttle_thresh_rh_pct\":%.2f,"
        "\"throttle_thresh_dew_c\":%.2f,"
        "\"throttle_thresh_hi_c\":%.2f,"
        "\"throttle_thresh_ah_gm3\":%.3f,"
        "\"throttle_thresh_rssi_dbm\":%u,"
        "\"throttle_thresh_frag_pct\":%.2f,"
        "\"wifi_ssid\":\"%s\","
        "\"wifi_password\":\"%s\","
        "\"ap_ssid\":\"%s\","
        "\"ap_password\":\"%s\"}",
        escDN, escS1,
        escMS, (unsigned)g_config.mqtt_port,
        escMU, escMP, escDI,
        (unsigned long)g_config.throttle_timer_data_base_ms,
        (unsigned long)g_config.throttle_safety_max_interval_ms,
        (unsigned long)g_config.throttle_supv_force_interval_ms,
        (double)g_config.throttle_thresh_temp_c,
        (double)g_config.throttle_thresh_rh_pct,
        (double)g_config.throttle_thresh_dew_c,
        (double)g_config.throttle_thresh_hi_c,
        (double)g_config.throttle_thresh_ah_gm3,
        (unsigned)g_config.throttle_thresh_rssi_dbm,
        (double)g_config.throttle_thresh_frag_pct,
        escWifiSS, escWifiPW, escApSS, escApPW);

    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    g_http.send(200, "application/json; charset=utf-8", resp);
}

static String g_httpBodyAccum;

#define HTTP_BODY_MAX_LEN 4096
static void readHttpBodyInto(String& out);

static bool tryParseUint16(const String& s, uint16_t* out)
{
    if (!out) return false;
    if (s.length() == 0 || s.length() > 5) return false;
    long v = 0;
    for (unsigned i = 0; i < s.length(); i++) {
        char c = s.charAt(i);
        if (c < '0' || c > '9') return false;
        v = v * 10 + (c - '0');
        if (v > 65535) return false;
    }
    *out = (uint16_t)v;
    return true;
}

static bool jsonExtractString(const String& json, const char* key, String& out)
{
    out = "";
    if (!key) return false;
    String search = String("\"") + key + "\"";
    int idx = json.indexOf(search);
    if (idx < 0) return false;
    idx += search.length();
    while (idx < (int)json.length() && (json.charAt(idx) == ' ' || json.charAt(idx) == '\t' || json.charAt(idx) == ':')) idx++;
    if (idx >= (int)json.length()) return false;
    if (json.charAt(idx) != '"') return false;
    idx++;
    String val;
    while (idx < (int)json.length()) {
        char c = json.charAt(idx);
        if (c == '"') {
            out = val;
            return true;
        }
        if (c == '\\' && idx + 1 < (int)json.length()) {
            char n = json.charAt(idx + 1);
            switch (n) {
                case '"':  val += '"'; break;
                case '\\': val += '\\'; break;
                case '/':  val += '/'; break;
                case 'n':  val += '\n'; break;
                case 'r':  val += '\r'; break;
                case 't':  val += '\t'; break;
                case 'b':  val += '\b'; break;
                case 'f':  val += '\f'; break;
                case 'u': {
                    if (idx + 5 < (int)json.length()) {
                        String hex = json.substring(idx + 2, idx + 6);
                        char buf[2] = { (char)(int)strtol(hex.c_str(), NULL, 16), '\0' };
                        if (buf[0] >= 0x20) val += buf[0];
                        idx += 4;
                    }
                    break;
                }
                default: val += n; break;
            }
            idx += 2;
        } else {
            val += c;
            idx++;
        }
    }
    return false;
}

static bool jsonExtractUint16(const String& json, const char* key, uint16_t* out)
{
    String s;
    if (!jsonExtractString(json, key, s)) {
        String search = String("\"") + key + "\"";
        int idx = json.indexOf(search);
        if (idx < 0) return false;
        idx += search.length();
        while (idx < (int)json.length() && (json.charAt(idx) == ' ' || json.charAt(idx) == '\t' || json.charAt(idx) == ':')) idx++;
        String num;
        while (idx < (int)json.length()) {
            char c = json.charAt(idx);
            if (c >= '0' && c <= '9') num += c;
            else break;
            idx++;
        }
        return tryParseUint16(num, out);
    }
    return tryParseUint16(s, out);
}

// --- Nuovi parser per throttling params (uint32, float, uint8) ---
static bool tryParseUint32(const String& s, uint32_t* out)
{
    if (!out) return false;
    if (s.length() == 0 || s.length() > 10) return false;
    uint32_t v = 0;
    for (unsigned i = 0; i < s.length(); i++) {
        char c = s.charAt(i);
        if (c < '0' || c > '9') return false;
        uint32_t prev = v;
        v = v * 10u + (uint32_t)(c - '0');
        if (v < prev) return false; // overflow
    }
    *out = v;
    return true;
}

static bool tryParseFloat(const String& s, float* out)
{
    if (!out) return false;
    if (s.length() == 0 || s.length() > 16) return false;
    const char* p = s.c_str();
    char* endp = NULL;
    float v = strtof(p, &endp);
    if (!isfinite(v)) return false;
    if (endp == NULL || endp == p) return false;
    while (*endp != '\0') {
        char c = *endp;
        if (c != ' ' && c != '\t') return false;
        endp++;
    }
    *out = v;
    return true;
}

static bool tryParseUint8(const String& s, uint8_t* out)
{
    if (!out) return false;
    uint16_t v16 = 0;
    if (!tryParseUint16(s, &v16)) return false;
    if (v16 > 255u) return false;
    *out = (uint8_t)v16;
    return true;
}

static bool jsonExtractUint32(const String& json, const char* key, uint32_t* out)
{
    String s;
    if (!jsonExtractString(json, key, s)) {
        String search = String("\"") + key + "\"";
        int idx = json.indexOf(search);
        if (idx < 0) return false;
        idx += search.length();
        while (idx < (int)json.length() && (json.charAt(idx) == ' ' || json.charAt(idx) == '\t' || json.charAt(idx) == ':')) idx++;
        bool neg = false;
        String num;
        if (idx < (int)json.length() && json.charAt(idx) == '-') { neg = true; idx++; }
        while (idx < (int)json.length()) {
            char c = json.charAt(idx);
            if (c >= '0' && c <= '9') num += c;
            else break;
            idx++;
        }
        if (neg) return false;
        return tryParseUint32(num, out);
    }
    return tryParseUint32(s, out);
}

static bool jsonExtractUint8(const String& json, const char* key, uint8_t* out)
{
    uint16_t v16;
    if (!jsonExtractUint16(json, key, &v16)) return false;
    if (v16 > 255u) return false;
    *out = (uint8_t)v16;
    return true;
}

static bool jsonExtractFloat(const String& json, const char* key, float* out)
{
    String s;
    if (!jsonExtractString(json, key, s)) {
        String search = String("\"") + key + "\"";
        int idx = json.indexOf(search);
        if (idx < 0) return false;
        idx += search.length();
        while (idx < (int)json.length() && (json.charAt(idx) == ' ' || json.charAt(idx) == '\t' || json.charAt(idx) == ':')) idx++;
        String num;
        while (idx < (int)json.length()) {
            char c = json.charAt(idx);
            if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') num += c;
            else break;
            idx++;
        }
        return tryParseFloat(num, out);
    }
    return tryParseFloat(s, out);
}
// --- Fine nuovi parser ---

static bool isValidDevId(const String& s)
{
    if (s.length() == 0 || s.length() >= MAX_LEN_MQTT_DEVID) return false;
    for (unsigned i = 0; i < s.length(); i++) {
        char c = s.charAt(i);
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

static void handleApiConfigPost(void)
{
    Config newCfg = g_config;

    String v;
    if (jsonExtractString(g_httpBodyAccum, "device_name", v)) {
        v.trim();
        if (v.length() == 0) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"Nome scheda vuoto\"}");
            return;
        }
        safeStrncpy(newCfg.device_name, v.c_str(), sizeof(newCfg.device_name));
    }
    if (jsonExtractString(g_httpBodyAccum, "sensor_sht31_name", v)) {
        v.trim();
        safeStrncpy(newCfg.sensor_sht31_name, v.c_str(), sizeof(newCfg.sensor_sht31_name));
    }
    if (jsonExtractString(g_httpBodyAccum, "mqtt_server", v)) {
        v.trim();
        if (v.length() == 0) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"Server MQTT vuoto\"}");
            return;
        }
        safeStrncpy(newCfg.mqtt_server, v.c_str(), sizeof(newCfg.mqtt_server));
    }
    uint16_t portVal;
    if (jsonExtractUint16(g_httpBodyAccum, "mqtt_port", &portVal)) {
        newCfg.mqtt_port = portVal;
    }
    if (jsonExtractString(g_httpBodyAccum, "mqtt_user", v)) {
        safeStrncpy(newCfg.mqtt_user, v.c_str(), sizeof(newCfg.mqtt_user));
    }
    if (jsonExtractString(g_httpBodyAccum, "mqtt_password", v)) {
        safeStrncpy(newCfg.mqtt_password, v.c_str(), sizeof(newCfg.mqtt_password));
    }
    if (jsonExtractString(g_httpBodyAccum, "mqtt_device_id", v)) {
        v.trim();
        if (!isValidDevId(v)) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"Device ID non valido (usa lettere/numeri/_/-)\"}");
            return;
        }
        safeStrncpy(newCfg.mqtt_device_id, v.c_str(), sizeof(newCfg.mqtt_device_id));
    }

    // --- Nuovi in v3: throttling MQTT params ---
    uint32_t u32v;
    float fv;
    uint8_t u8v;
    if (jsonExtractUint32(g_httpBodyAccum, "throttle_timer_data_base_ms", &u32v)) {
        newCfg.throttle_timer_data_base_ms = u32v;
    }
    if (jsonExtractUint32(g_httpBodyAccum, "throttle_safety_max_interval_ms", &u32v)) {
        newCfg.throttle_safety_max_interval_ms = u32v;
    }
    if (jsonExtractUint32(g_httpBodyAccum, "throttle_supv_force_interval_ms", &u32v)) {
        newCfg.throttle_supv_force_interval_ms = u32v;
    }
    if (jsonExtractFloat(g_httpBodyAccum, "throttle_thresh_temp_c", &fv)) {
        newCfg.throttle_thresh_temp_c = fv;
    }
    if (jsonExtractFloat(g_httpBodyAccum, "throttle_thresh_rh_pct", &fv)) {
        newCfg.throttle_thresh_rh_pct = fv;
    }
    if (jsonExtractFloat(g_httpBodyAccum, "throttle_thresh_dew_c", &fv)) {
        newCfg.throttle_thresh_dew_c = fv;
    }
    if (jsonExtractFloat(g_httpBodyAccum, "throttle_thresh_hi_c", &fv)) {
        newCfg.throttle_thresh_hi_c = fv;
    }
    if (jsonExtractFloat(g_httpBodyAccum, "throttle_thresh_ah_gm3", &fv)) {
        newCfg.throttle_thresh_ah_gm3 = fv;
    }
    if (jsonExtractUint8(g_httpBodyAccum, "throttle_thresh_rssi_dbm", &u8v)) {
        newCfg.throttle_thresh_rssi_dbm = u8v;
    }
    if (jsonExtractFloat(g_httpBodyAccum, "throttle_thresh_frag_pct", &fv)) {
        newCfg.throttle_thresh_frag_pct = fv;
    }

    // --- Nuovi in v4: Credenziali WiFi STA e AP ---
    if (jsonExtractString(g_httpBodyAccum, "wifi_ssid", v)) {
        v.trim();
        if (v.length() == 0) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"SSID WiFi STA vuoto\"}");
            return;
        }
        if (v.length() >= MAX_LEN_WIFI_SSID) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"SSID WiFi STA troppo lungo (max 32)\"}");
            return;
        }
        safeStrncpy(newCfg.wifi_ssid, v.c_str(), sizeof(newCfg.wifi_ssid));
    }
    if (jsonExtractString(g_httpBodyAccum, "wifi_password", v)) {
        if (v.length() >= MAX_LEN_WIFI_PASS) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"Password WiFi STA troppo lunga (max 63)\"}");
            return;
        }
        safeStrncpy(newCfg.wifi_password, v.c_str(), sizeof(newCfg.wifi_password));
    }
    if (jsonExtractString(g_httpBodyAccum, "ap_ssid", v)) {
        v.trim();
        if (v.length() == 0) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"SSID Access Point vuoto\"}");
            return;
        }
        if (v.length() >= MAX_LEN_AP_SSID) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"SSID Access Point troppo lungo (max 32)\"}");
            return;
        }
        safeStrncpy(newCfg.ap_ssid, v.c_str(), sizeof(newCfg.ap_ssid));
    }
    if (jsonExtractString(g_httpBodyAccum, "ap_password", v)) {
        if (v.length() > 0 && v.length() < 8) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"Password AP troppo corta (min 8 char oppure lascia vuota per AP aperto)\"}");
            return;
        }
        if (v.length() >= MAX_LEN_AP_PASS) {
            g_http.send(400, "application/json", "{\"ok\":false,\"message\":\"Password AP troppo lunga (max 63)\"}");
            return;
        }
        safeStrncpy(newCfg.ap_password, v.c_str(), sizeof(newCfg.ap_password));
    }

    bool mqttChanged =
        (strcmp(newCfg.mqtt_server, g_config.mqtt_server) != 0) ||
        (newCfg.mqtt_port != g_config.mqtt_port) ||
        (strcmp(newCfg.mqtt_user, g_config.mqtt_user) != 0) ||
        (strcmp(newCfg.mqtt_password, g_config.mqtt_password) != 0) ||
        (strcmp(newCfg.mqtt_device_id, g_config.mqtt_device_id) != 0);

    bool labelsChanged =
        (strcmp(newCfg.device_name, g_config.device_name) != 0) ||
        (strcmp(newCfg.sensor_sht31_name, g_config.sensor_sht31_name) != 0);

    bool throttleChanged =
        (newCfg.throttle_timer_data_base_ms     != g_config.throttle_timer_data_base_ms) ||
        (newCfg.throttle_safety_max_interval_ms != g_config.throttle_safety_max_interval_ms) ||
        (newCfg.throttle_supv_force_interval_ms != g_config.throttle_supv_force_interval_ms) ||
        (newCfg.throttle_thresh_temp_c          != g_config.throttle_thresh_temp_c) ||
        (newCfg.throttle_thresh_rh_pct          != g_config.throttle_thresh_rh_pct) ||
        (newCfg.throttle_thresh_dew_c           != g_config.throttle_thresh_dew_c) ||
        (newCfg.throttle_thresh_hi_c            != g_config.throttle_thresh_hi_c) ||
        (newCfg.throttle_thresh_ah_gm3          != g_config.throttle_thresh_ah_gm3) ||
        (newCfg.throttle_thresh_rssi_dbm        != g_config.throttle_thresh_rssi_dbm) ||
        (newCfg.throttle_thresh_frag_pct        != g_config.throttle_thresh_frag_pct);

    bool wifiChanged =
        (strcmp(newCfg.wifi_ssid, g_config.wifi_ssid) != 0) ||
        (strcmp(newCfg.wifi_password, g_config.wifi_password) != 0) ||
        (strcmp(newCfg.ap_ssid, g_config.ap_ssid) != 0) ||
        (strcmp(newCfg.ap_password, g_config.ap_password) != 0);

    if (!configSaveToEeprom(&newCfg)) {
        g_http.send(500, "application/json", "{\"ok\":false,\"message\":\"Errore scrittura EEPROM\"}");
        return;
    }

    if (mqttChanged) {
        g_mqtt_config_changed = true;
    } else if (labelsChanged) {
        g_mqtt_discovery_sent = false;
    }
    if (throttleChanged) {
        throttleResetAllState();   // Zero-reboot apply: reset timer e last-state
        Serial.println(F("  -> Throttling MQTT cambiato: reset timer e last-state (zero-reboot)"));
    }
    if (wifiChanged) {
        Serial.println(F("  -> Credenziali WiFi cambiate: riavvio necessario per applicarle"));
    }

    Serial.println(F("CONFIG salvata in EEPROM"));
    if (wifiChanged)       Serial.println(F("  -> WiFi config cambiata: RIAVVIARE il dispositivo per applicare"));
    else if (mqttChanged)  Serial.println(F("  -> MQTT config cambiata: riconnessione al prossimo ciclo"));
    else if (labelsChanged) Serial.println(F("  -> Nomi cambiati: re-discovery MQTT alla prossima connessione"));
    else if (throttleChanged) Serial.println(F("  -> Parametri throttling applicati (zero-reboot)"));

    if (wifiChanged) {
        g_http.send(200, "application/json", "{\"ok\":true,\"message\":\"Salvato in EEPROM. Riavviare il dispositivo per applicare le modifiche WiFi.\"}");
    } else {
        g_http.send(200, "application/json", "{\"ok\":true,\"message\":\"Salvato in EEPROM\"}");
    }
}

static void handleApiConfig(void)
{
    if (g_http.method() == HTTP_GET) {
        handleApiConfigGet();
        return;
    }
    if (g_http.method() == HTTP_POST || g_http.method() == HTTP_PUT) {
        readHttpBodyInto(g_httpBodyAccum);
        handleApiConfigPost();
        return;
    }
    g_http.send(405, "application/json", "{\"ok\":false,\"message\":\"Method Not Allowed\"}");
}

static void handleApiRestart(void)
{
    if (g_http.method() != HTTP_POST) {
        g_http.send(405, "application/json", "{\"ok\":false,\"message\":\"POST richiesto\"}");
        return;
    }
    g_http.send(200, "application/json", "{\"ok\":true,\"message\":\"Riavvio...\"}");
    g_http.stop();
    delay(300);
    Serial.println(F("Riavvio su richiesta API..."));
    ESP.restart();
}

// ============================================================================
// API Firmware info (per Tab 🔧 Firmware)
// ============================================================================
static void handleApiFwInfo(void)
{
    if (g_http.method() != HTTP_GET) {
        g_http.send(405, "application/json", "{\"ok\":false,\"message\":\"GET richiesto\"}");
        return;
    }
    char buf[768];
    char chipIdHex[16];
    snprintf(chipIdHex, sizeof(chipIdHex), "0x%08X", (unsigned)ESP.getChipId());
    int n = 0;
    n += snprintf(buf + n, sizeof(buf) - n, "{");
    n += snprintf(buf + n, sizeof(buf) - n, "\"fw_version\":\"%s\",", FW_VERSION_STR);
    n += snprintf(buf + n, sizeof(buf) - n, "\"fw_build\":\"%s\",", FW_BUILD_DATE);
    n += snprintf(buf + n, sizeof(buf) - n, "\"chip_id\":\"%s\",", chipIdHex);
    n += snprintf(buf + n, sizeof(buf) - n, "\"flash_size\":%u,", (unsigned)ESP.getFlashChipSize());
    n += snprintf(buf + n, sizeof(buf) - n, "\"flash_real_size\":%u,", (unsigned)ESP.getFlashChipRealSize());
    n += snprintf(buf + n, sizeof(buf) - n, "\"max_sketch_bytes\":%lu,", (unsigned long)OTA_MAX_SKETCH_BYTES);
    n += snprintf(buf + n, sizeof(buf) - n, "\"free_heap\":%u,", (unsigned)ESP.getFreeHeap());
    n += snprintf(buf + n, sizeof(buf) - n, "\"max_free_block\":%u,", (unsigned)ESP.getMaxFreeBlockSize());
    n += snprintf(buf + n, sizeof(buf) - n, "\"heap_frag_pct\":%u,", (unsigned)ESP.getHeapFragmentation());
    n += snprintf(buf + n, sizeof(buf) - n, "\"sdk_version\":\"%s\",", ESP.getCoreVersion().c_str());
    n += snprintf(buf + n, sizeof(buf) - n, "\"boot_version\":%u,", (unsigned)ESP.getBootVersion());
    n += snprintf(buf + n, sizeof(buf) - n, "\"reset_reason\":\"%s\",", rstReasonStr(g_rstReasonCached));
    n += snprintf(buf + n, sizeof(buf) - n, "\"ota_busy\":%s,", g_otaBusy ? "true" : "false");
    n += snprintf(buf + n, sizeof(buf) - n, "\"ota_state\":%u,", (unsigned)g_otaState);
    n += snprintf(buf + n, sizeof(buf) - n, "\"ota_bytes_written\":%lu,", (unsigned long)g_otaBytesWritten);
    n += snprintf(buf + n, sizeof(buf) - n, "\"ota_bytes_expected\":%lu,", (unsigned long)g_otaBytesExpected);
    n += snprintf(buf + n, sizeof(buf) - n, "\"ota_total_declared\":%lu,", (unsigned long)g_otaTotalSizeDeclared);
    n += snprintf(buf + n, sizeof(buf) - n, "\"ota_last_ok_ms\":%lu,", (unsigned long)g_otaLastOkMs);
    n += snprintf(buf + n, sizeof(buf) - n, "\"ota_last_fail_ms\":%lu,", (unsigned long)g_otaLastFailMs);
    n += snprintf(buf + n, sizeof(buf) - n, "\"ota_last_error\":\"%s\"", g_otaLastError);
    n += snprintf(buf + n, sizeof(buf) - n, "}");
    g_http.send(200, "application/json", buf);
}

// ============================================================================
// OTA: Upload chunk handler + final handler
// ============================================================================
static void otaResetState(void)
{
    g_otaState = OTA_IDLE;
    g_otaBusy = false;
    g_otaBytesWritten = 0;
    g_otaBytesExpected = 0;
    g_otaTotalSizeDeclared = 0;
    g_otaUploadEndReceived = false;
    g_otaWdtChunkCounter = 0;
    g_otaLastChunkMs = 0;
    g_otaStartMs = 0;
    g_otaFirst4Valid = false;
    memset(g_otaFirst4Bytes, 0, sizeof(g_otaFirst4Bytes));
    memset(g_otaAbortReason, 0, sizeof(g_otaAbortReason));
}

static void otaLogAndSetAbort(const char* reason)
{
    strncpy(g_otaAbortReason, reason, sizeof(g_otaAbortReason) - 1);
    g_otaAbortReason[sizeof(g_otaAbortReason) - 1] = '\0';
    strncpy(g_otaLastError, reason, sizeof(g_otaLastError) - 1);
    g_otaLastError[sizeof(g_otaLastError) - 1] = '\0';
    g_otaLastFailMs = millis();
    Serial.print(F("OTA ABORT: ")); Serial.println(reason);
    g_otaState = OTA_ABORTED;
    Update.end(false);
    g_otaBusy = false;
}

static void handleUploadChunk(void)
{
    HTTPUpload& u = g_http.upload();

    if (u.name != "firmware") return;

    const unsigned long now = millis();

    if (u.status == UPLOAD_FILE_START) {
        otaResetState();
        g_otaBusy = true;
        Serial.print(F("OTA START: file=")); Serial.println(u.filename);
        Serial.print(F("OTA: tot_size declared by multipart=")); Serial.println((unsigned long)u.totalSize);
        if (u.totalSize > 0) {
            g_otaTotalSizeDeclared = u.totalSize;
            if (g_otaTotalSizeDeclared > OTA_MAX_SKETCH_BYTES) {
                char tmp[96];
                snprintf(tmp, sizeof(tmp), "Firmware troppo grande: %lu byte (max %lu)",
                         (unsigned long)g_otaTotalSizeDeclared, (unsigned long)OTA_MAX_SKETCH_BYTES);
                otaLogAndSetAbort(tmp);
                return;
            }
        }
        if (g_http.hasHeader("Content-Length")) {
            unsigned long cl = strtoul(g_http.header("Content-Length").c_str(), NULL, 10);
            if (cl == 0 || cl > (4UL * OTA_MAX_SKETCH_BYTES)) {
                Serial.println(F("OTA WARN: Content-Length HTTP assente, zero o irrealistico; multipart non garantisce size. Procedo con running check."));
            }
        }

        // PREPARAZIONE PRE-UPLOAD: disconnetti SEMPRE MQTT/WS e ferma socket TCP MQTT
        // PRIMA che il loop() abbia la possibilita' di ri-connetterli in race.
        // g_otaBusy è già true. Disconnettiamo QUI in modo SINCRONO per
        // massimizzare heap libero disponibile quando arriverà il primo WRITE chunk.
        Serial.print(F("OTA PREP: Heap PRIMA cleanup MQTT/WS/TCP: ")); Serial.print((unsigned)ESP.getFreeHeap()); Serial.println(F(" byte"));
        yield();
        if (g_mqtt.connected()) {
            char availTopic[160];
            mqttBuildAvailTopic(availTopic, sizeof(availTopic));
            (void)g_mqtt.publish(availTopic, "offline", true);
            g_mqtt.disconnect();
            g_mqtt_was_connected = false;
            g_mqtt_disconnect_ts_ms = millis();
            g_mqtt_consec_failures = 0;
            g_mqtt_lastAttemptMs = 0;
            g_mqtt_pub_consec_fail = 0;
        }
        yield();
        g_ws.close();
        yield();
        if (g_wifiClient.connected()) {
            Serial.println(F("OTA PREP: Stop socket TCP MQTT per liberare +1-2KB heap..."));
            g_wifiClient.stop();
        }
        for (uint8_t i = 0u; i < 6u; i++) {
            delay(30);
            yield();
            ESP.wdtFeed();
        }
        Serial.print(F("OTA PREP: Heap DOPO disconnect MQTT/WS/stop TCP MQTT: ")); Serial.print((unsigned)ESP.getFreeHeap()); Serial.println(F(" byte"));
        yield();
        ESP.wdtFeed();
        return;
    }

    if (u.status == UPLOAD_FILE_ABORTED) {
        if (g_otaState != OTA_IDLE && g_otaState != OTA_ABORTED) {
            otaLogAndSetAbort("Upload abortito dal client/server (UPLOAD_FILE_ABORTED)");
        }
        return;
    }

    if (u.status == UPLOAD_FILE_END) {
        g_otaUploadEndReceived = true;
        Serial.print(F("OTA: UPLOAD_FILE_END ricevuto. Bytes ricevuti finora=")); Serial.println((unsigned long)g_otaBytesWritten);
        return;
    }

    if (u.status != UPLOAD_FILE_WRITE) return;

    uint8_t* data = u.buf;
    size_t len = u.currentSize;
    if (len == 0 || data == nullptr) return;

    if (g_otaState == OTA_IDLE) {

        if (len < 4) {
            otaLogAndSetAbort("Primo chunk troppo corto (< 4 byte magic)");
            return;
        }
        memcpy(g_otaFirst4Bytes, data, 4);
        g_otaFirst4Valid = true;

        if (g_otaFirst4Bytes[0] != OTA_ESP_MAGIC_BYTE) {
            char tmp[96];
            snprintf(tmp, sizeof(tmp),
                     "Magic byte 0x%02X non valido (atteso 0x%02X) — non è un firmware ESP8266",
                     g_otaFirst4Bytes[0], (unsigned)OTA_ESP_MAGIC_BYTE);
            otaLogAndSetAbort(tmp);
            return;
        }

        if (ESP.getFreeHeap() < OTA_MIN_FREE_HEAP_BYTES) {
            Serial.print(F("OTA: Heap ancora basso nel primo WRITE (PREP non sufficiente): ")); Serial.print((unsigned)ESP.getFreeHeap()); Serial.println(F(" byte -> retry stop MQTT/WS/TCP"));
            yield();
            if (g_mqtt.connected()) {
                char availTopic[160];
                mqttBuildAvailTopic(availTopic, sizeof(availTopic));
                (void)g_mqtt.publish(availTopic, "offline", true);
                g_mqtt.disconnect();
                g_mqtt_was_connected = false;
                g_mqtt_disconnect_ts_ms = millis();
                g_mqtt_consec_failures = 0;
                g_mqtt_lastAttemptMs = 0;
                g_mqtt_pub_consec_fail = 0;
            }
            yield();
            g_ws.close();
            yield();
            if (g_wifiClient.connected()) {
                Serial.println(F("OTA: Stop socket TCP MQTT PubSubClient per liberare +1-2KB heap..."));
                g_wifiClient.stop();
            }
            for (uint8_t i = 0u; i < 4u; i++) {
                delay(40);
                yield();
                ESP.wdtFeed();
            }
            Serial.print(F("OTA: Heap DOPO retry disconnect MQTT/WS/stop TCP MQTT: ")); Serial.print((unsigned)ESP.getFreeHeap()); Serial.println(F(" byte"));
            yield();
            if (ESP.getFreeHeap() < OTA_MIN_FREE_HEAP_BYTES) {
                char tmp[96];
                snprintf(tmp, sizeof(tmp),
                         "Heap libero insufficiente DOPO retry PREP + WRITE MQTT+WS+stop TCP: %u byte (min %lu)",
                         (unsigned)ESP.getFreeHeap(), (unsigned long)OTA_MIN_FREE_HEAP_BYTES);
                otaLogAndSetAbort(tmp);
                return;
            }
        }

        if (g_otaTotalSizeDeclared > 0) {
            g_otaBytesExpected = g_otaTotalSizeDeclared;
        } else {
            g_otaBytesExpected = OTA_MAX_SKETCH_BYTES;
        }
        g_otaStartMs = now;
        g_otaLastChunkMs = now;
        g_otaBytesWritten = 0;
        g_otaWdtChunkCounter = 0;

        Serial.print(F("OTA BEGIN: estimated_size=")); Serial.print((unsigned long)g_otaBytesExpected);
        if (g_otaTotalSizeDeclared == 0) {
            Serial.print(F("  [nessuna size dichiarata; uso cap max sketch + check running byte scritti]"));
        }
        Serial.print(F("  ESP header: 0x"));
        Serial.print(g_otaFirst4Bytes[0], HEX); Serial.print(" ");
        Serial.print(g_otaFirst4Bytes[1], HEX); Serial.print(" ");
        Serial.print(g_otaFirst4Bytes[2], HEX); Serial.print(" ");
        Serial.println(g_otaFirst4Bytes[3], HEX);

        if (!Update.begin(g_otaBytesExpected, U_FLASH)) {
            String errStr = Update.getErrorString();
            const char* err = errStr.length() > 0 ? errStr.c_str() : nullptr;
            char tmp[96];
            snprintf(tmp, sizeof(tmp), "Update.begin() fallito: %s", err ? err : "sconosciuto");
            otaLogAndSetAbort(tmp);
            return;
        }

        g_otaState = OTA_WRITING;
        g_otaBusy = true;
    }

    if (g_otaState != OTA_WRITING && g_otaState != OTA_HEADER_CHECKED) {
        return;
    }
    if (g_otaState == OTA_HEADER_CHECKED) g_otaState = OTA_WRITING;

    if (g_otaStartMs != 0ul && ((uint32_t)now - g_otaStartMs > OTA_HARD_TIMEOUT_MS)) {
        otaLogAndSetAbort("Timeout hard OTA superato (>120s)");
        return;
    }

    if (g_otaBytesWritten + len > OTA_MAX_SKETCH_BYTES) {
        char tmp[96];
        snprintf(tmp, sizeof(tmp),
                 "Overflow dimensione massima: %lu + %u > %lu byte (firmware piu' grande del previsto)",
                 (unsigned long)g_otaBytesWritten, (unsigned)len, (unsigned long)OTA_MAX_SKETCH_BYTES);
        otaLogAndSetAbort(tmp);
        return;
    }

    size_t written = Update.write(data, len);
    if (written != len) {
        String errStr = Update.getErrorString();
        const char* err = errStr.length() > 0 ? errStr.c_str() : nullptr;
        char tmp[96];
        snprintf(tmp, sizeof(tmp),
                 "Update.write() mismatch: scritti %u di %u byte. %s",
                 (unsigned)written, (unsigned)len, err ? err : "");
        otaLogAndSetAbort(tmp);
        return;
    }

    g_otaBytesWritten += written;
    g_otaLastChunkMs = now;

    g_otaWdtChunkCounter += written;
    while (g_otaWdtChunkCounter >= OTA_WDT_FEED_CHUNK_BYTES) {
        g_otaWdtChunkCounter -= OTA_WDT_FEED_CHUNK_BYTES;
        ESP.wdtFeed();
        g_diag_wdtFeedCount++;
        yield();
    }

    yield();
    ESP.wdtFeed();

    if (((g_otaBytesWritten & 0xFFFFu) == 0u) || (g_otaTotalSizeDeclared > 0 && g_otaBytesWritten >= g_otaTotalSizeDeclared) || (len < 1460u)) {
        Serial.print(F("OTA: scritti ")); Serial.print((unsigned long)g_otaBytesWritten);
        Serial.print(F(" / "));
        if (g_otaTotalSizeDeclared > 0) Serial.println((unsigned long)g_otaTotalSizeDeclared);
        else Serial.println(F("max"));
    }
}

static void handleUploadFinish(void)
{
    if (g_http.method() != HTTP_POST) {
        g_http.send(405, "application/json", "{\"ok\":false,\"message\":\"POST richiesto\"}");
        otaResetState();
        return;
    }

    if (g_otaState == OTA_ABORTED) {
        char resp[256];
        snprintf(resp, sizeof(resp),
                 "{\"ok\":false,\"message\":\"%s\",\"bytes_written\":%lu,\"bytes_expected\":%lu}",
                 g_otaAbortReason[0] ? g_otaAbortReason : "abortito senza motivo specifico",
                 (unsigned long)g_otaBytesWritten,
                 (unsigned long)g_otaBytesExpected);
        g_http.send(400, "application/json", resp);
        otaResetState();
        return;
    }

    if (g_otaState != OTA_WRITING || g_otaBytesWritten == 0) {
        g_http.send(400, "application/json",
                    "{\"ok\":false,\"message\":\"Nessun dato firmware ricevuto\"}");
        otaResetState();
        return;
    }

    if (!g_otaUploadEndReceived) {
        char tmp[96];
        snprintf(tmp, sizeof(tmp),
                 "Upload incompleto: UPLOAD_FILE_END mai ricevuto (%lu byte scritti)",
                 (unsigned long)g_otaBytesWritten);
        otaLogAndSetAbort(tmp);
        char resp[256];
        snprintf(resp, sizeof(resp),
                 "{\"ok\":false,\"message\":\"%s\",\"bytes_written\":%lu,\"bytes_expected\":%lu}",
                 g_otaAbortReason,
                 (unsigned long)g_otaBytesWritten,
                 (unsigned long)g_otaBytesExpected);
        g_http.send(400, "application/json", resp);
        otaResetState();
        return;
    }

    if (g_otaTotalSizeDeclared > 0) {
        if (g_otaBytesWritten != g_otaTotalSizeDeclared) {
            char tmp[96];
            snprintf(tmp, sizeof(tmp),
                     "Byte scritti (%lu) != dichiarati (%lu) — upload incompleto",
                     (unsigned long)g_otaBytesWritten, (unsigned long)g_otaTotalSizeDeclared);
            otaLogAndSetAbort(tmp);
            char resp[256];
            snprintf(resp, sizeof(resp),
                     "{\"ok\":false,\"message\":\"%s\",\"bytes_written\":%lu,\"bytes_expected\":%lu}",
                     g_otaAbortReason,
                     (unsigned long)g_otaBytesWritten,
                     (unsigned long)g_otaTotalSizeDeclared);
            g_http.send(400, "application/json", resp);
            otaResetState();
            return;
        }
    } else {
        if (g_otaBytesWritten < 102400UL) {
            char tmp[96];
            snprintf(tmp, sizeof(tmp),
                     "Firmware troppo piccolo (%lu byte, min 100KB): probabilmente upload troncato",
                     (unsigned long)g_otaBytesWritten);
            otaLogAndSetAbort(tmp);
            char resp[256];
            snprintf(resp, sizeof(resp),
                     "{\"ok\":false,\"message\":\"%s\",\"bytes_written\":%lu,\"bytes_expected\":%lu}",
                     g_otaAbortReason,
                     (unsigned long)g_otaBytesWritten,
                     (unsigned long)g_otaBytesExpected);
            g_http.send(400, "application/json", resp);
            otaResetState();
            return;
        }
    }

    g_otaState = OTA_WRITE_COMPLETE;

    ESP.wdtFeed();
    g_diag_wdtFeedCount++;

    bool endOk = Update.end(true);
    ESP.wdtFeed();
    g_diag_wdtFeedCount++;

    if (!endOk) {
        String errStr = Update.getErrorString();
        const char* err = errStr.length() > 0 ? errStr.c_str() : nullptr;
        char tmp[96];
        snprintf(tmp, sizeof(tmp), "Update.end() fallito: %s", err ? err : "sconosciuto");
        otaLogAndSetAbort(tmp);
        char resp[256];
        snprintf(resp, sizeof(resp),
                 "{\"ok\":false,\"message\":\"%s\",\"bytes_written\":%lu}",
                 g_otaAbortReason, (unsigned long)g_otaBytesWritten);
        g_http.send(500, "application/json", resp);
        otaResetState();
        return;
    }

    memset(g_otaLastError, 0, sizeof(g_otaLastError));
    g_otaLastOkMs = millis();

    const char* md5CStr = "n/a";

    Serial.print(F("OTA OK: md5=")); Serial.println(md5CStr);
    Serial.print(F("OTA OK: ")); Serial.print((unsigned long)g_otaBytesWritten);
    Serial.println(F(" byte scritti con successo. Riavvio in 1500ms..."));

    g_otaBusy = false;

    char resp[512];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"bytes_written\":%lu,\"bytes_expected\":%lu,"
             "\"md5_calc\":\"%s\",\"restart_in_ms\":1500,\"fw_version_new\":\"%s\","
             "\"size_declared\":%s}",
             (unsigned long)g_otaBytesWritten,
             (unsigned long)g_otaBytesExpected,
             md5CStr,
             FW_VERSION_STR,
             (g_otaTotalSizeDeclared > 0) ? "true" : "false");
    g_http.sendHeader("Connection", "close");
    g_http.send(200, "application/json", resp);

    g_http.stop();
    Serial.flush();

    for (uint8_t i = 0; i < 15; i++) {
        delay(100);
        ESP.wdtFeed();
        g_diag_wdtFeedCount++;
    }

    ESP.restart();

    // Non raggiungibile
    otaResetState();
}

static void readHttpBodyInto(String& out)
{
    out = "";
    if (g_http.hasArg("plain") && g_http.arg("plain").length() > 0) {
        out = g_http.arg("plain");
        if (out.length() >= 2) return;
    }
    WiFiClient& c = g_http.client();
    if (!c.connected()) return;
    int avail = c.available();
    if (avail <= 0) return;
    size_t toRead = (size_t)avail;
    if (toRead > HTTP_BODY_MAX_LEN) toRead = HTTP_BODY_MAX_LEN;
    out.reserve(toRead + 16);
    size_t got = 0;
    unsigned long t0 = millis();
    uint16_t loopCnt = 0;
    while (got < toRead && (millis() - t0) < 300) {
        if ((loopCnt++ & 0x1Fu) == 0u) {
            ESP.wdtFeed();
            g_diag_wdtFeedCount++;
        }
        int a = c.available();
        if (a <= 0) { delay(2); continue; }
        uint8_t b;
        if (c.read(&b, 1) == 1) {
            if (b >= 0x20 || b == '\r' || b == '\n' || b == '\t') {
                out += (char)b;
            } else if (b != 0) {
                out += (char)b;
            }
            got++;
        } else {
            break;
        }
    }
}

// ============================================================================
// Output seriale CSV
// ============================================================================
static void serialPrintCsv(void)
{
    static unsigned long lastMs = 0;
    static bool header_printed = false;
    unsigned long now = millis();
    if ((now - lastMs) < SHT31_SAMPLE_INTERVAL_MS) return;
    lastMs = now;

    if (!header_printed) {
        Serial.println(F("ts_ms,SHT31_T_C,SHT31_RH_PCT,SHT31_DEW_C,SHT31_HI_C,SHT31_AH_GM3,status_hex,heater,status_list"));
        header_printed = true;
    }

    char line[256];
    int w = 0;
    w += snprintf(line + w, sizeof(line) - w, "%lu,", (unsigned long)now);
    if (isfinite(g_sht31_t))  w += snprintf(line + w, sizeof(line) - w, "%.3f,", g_sht31_t);
    else                      w += snprintf(line + w, sizeof(line) - w, "NaN,");
    if (isfinite(g_sht31_rh)) w += snprintf(line + w, sizeof(line) - w, "%.2f,", g_sht31_rh);
    else                      w += snprintf(line + w, sizeof(line) - w, "NaN,");
    if (isfinite(g_sht31_dp)) w += snprintf(line + w, sizeof(line) - w, "%.2f,", g_sht31_dp);
    else                      w += snprintf(line + w, sizeof(line) - w, "NaN,");
    if (isfinite(g_sht31_hi)) w += snprintf(line + w, sizeof(line) - w, "%.2f,", g_sht31_hi);
    else                      w += snprintf(line + w, sizeof(line) - w, "NaN,");
    if (isfinite(g_sht31_ah)) w += snprintf(line + w, sizeof(line) - w, "%.3f,", g_sht31_ah);
    else                      w += snprintf(line + w, sizeof(line) - w, "NaN,");
    w += snprintf(line + w, sizeof(line) - w, "0x%02X,%u,%s",
        (unsigned)g_status_bits, g_sht31_heater ? 1u : 0u, sensorStatusList(g_status_bits));
    Serial.println(line);
}

// ============================================================================
// Broadcast WebSocket
// ============================================================================
static void wsBroadcast(void)
{
    static unsigned long lastMs = 0;
    unsigned long now = millis();
    if ((now - lastMs) < SAMPLE_PERIOD_MS) return;
    lastMs = now;

    uint8_t buf[28];
    float vals[7];
    vals[0] = g_sht31_t;
    vals[1] = g_sht31_rh;
    vals[2] = g_sht31_dp;
    vals[3] = g_sht31_hi;
    vals[4] = g_sht31_ah;
    vals[5] = (float)g_status_bits;
    vals[6] = g_sht31_heater ? 1.0f : 0.0f;
    memcpy(buf, vals, sizeof(buf));

    g_ws.broadcastBIN(buf, sizeof(buf));
}

// ============================================================================
// setup / loop
// ============================================================================
void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(50);
    g_rstReasonCached = diagInitAndPrintResetReport();
    g_diag_bootCount++;
    Serial.print(F("DIAG: boot_count=")); Serial.println(g_diag_bootCount);

    Serial.println();
    Serial.println(F("=============================================================="));
    Serial.println(F(" ESP8266 1xSHT31-D I²C @400kHz (D1=SCL D2=SDA)  [EEPROM v2 config]"));
    Serial.println(F(" Grandezze: T, RH, DewPoint, HeatIndex, AbsHumidity + Heater"));
    Serial.println(F(" Output: Seriale CSV + Pagina Web(HTTP:80) + WS(81) + MQTT"));
    Serial.println(F("=============================================================="));

    bool eepromOk = configLoadFromEeprom(&g_config);
    if (!eepromOk) {
        Serial.println(F("EEPROM: nessuna config valida -> usati default + salvataggio iniziale"));
        (void)configSaveToEeprom(&g_config);
    } else {
        Serial.println(F("EEPROM: config caricata"));
    }
    Serial.print(F("  Device name    : ")); Serial.println(g_config.device_name);
    Serial.print(F("  MQTT device ID : ")); Serial.println(g_config.mqtt_device_id);
    Serial.print(F("  MQTT server    : ")); Serial.print(g_config.mqtt_server);
    Serial.print(':'); Serial.println(g_config.mqtt_port);

    Serial.println(F("--- Inizializzazione sensore SHT31-D ---"));
    (void)sht31_init();
    netStartSTAorAP();

    g_http.on("/", HTTP_GET, handleRoot);
    g_http.on("/api/labels", HTTP_GET, handleApiLabels);
    g_http.on("/api/config", HTTP_ANY, handleApiConfig, []() { /* upload body handler: no-op */ });
    g_http.on("/api/restart", HTTP_ANY, handleApiRestart, []() { /* upload body handler: no-op */ });
    g_http.on("/api/fw_info", HTTP_GET, handleApiFwInfo);
    g_http.onFileUpload(handleUploadChunk);
    g_http.on("/update", HTTP_POST, handleUploadFinish);
    g_http.onNotFound([]() {
        if (g_http.uri() == "/api/config" && (g_http.method() == HTTP_POST || g_http.method() == HTTP_PUT)) {
            readHttpBodyInto(g_httpBodyAccum);
            handleApiConfigPost();
            return;
        }
        if (g_http.uri() == "/api/restart" && g_http.method() == HTTP_POST) {
            handleApiRestart();
            return;
        }
        g_http.send(404, "text/plain", "Not Found");
    });
    g_http.begin();

    g_ws.begin();
#ifdef ESP8266
    // Heartbeat WS: ping ogni 15s, pong timeout 6s, chiudi dopo 2 ping senza risposta.
    // Elimina connessioni zombie lasciate da router/NAT che droppano il TCP silenziosamente.
    g_ws.enableHeartbeat(15000, 6000, 2);
#endif

    const char* srv = g_config.mqtt_server[0] ? g_config.mqtt_server : MQTT_BROKER_IP_DEFAULT;
    uint16_t port  = (g_config.mqtt_port > 0 && g_config.mqtt_port <= 65535) ? g_config.mqtt_port : MQTT_BROKER_PORT_DEFAULT;

    g_wifiClient.setNoDelay(true);
    // Nota: g_wifiClient.keepAlive(1) NON viene chiamato qui!
    // WiFiClient::keepAlive() dereferenzia _client interno SENZA null-check.
    // Prima della prima connessione _client=nullptr → LoadStoreErrorBus NULL deref (exccause=28 excvaddr=0).
    // keepAlive() viene invece impostato dentro mqttConnect() SUBITO DOPO g_mqtt.connect() ok
    // quando il ClientContext è stato creato dalla connessione TCP.
    g_mqtt.setServer(srv, port);
    g_mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
    g_mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
    g_mqtt.setBufferSize(MQTT_CLIENT_BUFFER_SIZE);
    Serial.print(F("MQTT client cfg: KA=")); Serial.print(MQTT_KEEPALIVE_S);
    Serial.print(F("s  SockTO=")); Serial.print(MQTT_SOCKET_TIMEOUT_S);
    Serial.print(F("s  Buf=")); Serial.print(MQTT_CLIENT_BUFFER_SIZE);
    Serial.print(F("B  NagleOFF=1  TCP_KA=1  server=")); Serial.print(srv); Serial.print(':'); Serial.println(port);

    supervisorInit();

    Serial.print(F("HTTP su :")); Serial.println(HTTP_PORT);
    Serial.print(F("WS   su :")); Serial.println(WS_PORT);
    Serial.println(F("--- Setup completato ---"));
}

void loop()
{
    unsigned long now = millis();
    g_diag_wdtFeedCount++;
    crashHeartbeatSave((uint32_t)now);

    if (g_otaState != OTA_IDLE && g_otaState != OTA_ABORTED) {
        if ((uint32_t)now - g_otaLastChunkMs >= OTA_SOFT_TIMEOUT_MS) {
            Update.end(false);
            otaLogAndSetAbort("Soft timeout: nessun chunk entro 60s");
        }
    }

    if (diagStackOverflowDetected()) {
        static uint32_t lastStackWarn = 0;
        if ((uint32_t)now - lastStackWarn > 5000u) {
            lastStackWarn = (uint32_t)now;
            Serial.print(F("! DIAG: STACK ALLERTATO - disponibili="));
            Serial.print(stackAvailableBytes());
            Serial.print(F("B  stack_used_wm="));
            Serial.print(g_diag_stackWmBytes);
            Serial.println(F("B"));
        }
    }
    {
        uint32_t freeHeap = (uint32_t)ESP.getFreeHeap();
        uint32_t maxBlock = (uint32_t)ESP.getMaxFreeBlockSize();
        if (freeHeap < 4096u || maxBlock < 2048u) {
            static uint32_t lastHeapWarn = 0;
            if ((uint32_t)now - lastHeapWarn > 5000u) {
                lastHeapWarn = (uint32_t)now;
                Serial.print(F("! DIAG: HEAP CRITICO - free=")); Serial.print(freeHeap);
                Serial.print(F("B  max_block=")); Serial.print(maxBlock);
                Serial.print(F("B  frag=")); Serial.print(ESP.getHeapFragmentation());
                Serial.print(F("%  min_ever=")); Serial.print(g_diag_minHeapEver);
                Serial.println(F("B"));
            }
        }
    }

    supervisorWatchdogFeed();

    g_http.handleClient();
    supervisorWatchdogFeed();

    g_ws.loop();
    g_mqtt.loop();
    supervisorWatchdogFeed();

    netFallbackPollRetry();
    supervisorWatchdogFeed();

    // ----- RILEVAZIONE TRANSIZIONE MQTT CONNESSO -> DISCONNESSO -----
    // Subito dopo g_mqtt.loop() (che consuma gli eventi di disconnect),
    // confronta lo stato attuale con g_mqtt_was_connected per forzare
    // la riconnessione IMMEDIATA nel prossimo mqttEnsureConnected()
    // senza aspettare backoff.
    {
        bool nowConn = g_mqtt.connected();
        if (g_mqtt_was_connected && !nowConn) {
            unsigned long connDur = 0;
            if (g_mqtt_connect_ts_ms != 0) connDur = now - g_mqtt_connect_ts_ms;
            if (!g_otaBusy) {
                Serial.print(F("LOOP: TRANSIZIONE MQTT CONNESSO->DISCONNESSO RILEVATA "));
                Serial.print(F("durata_conn=")); Serial.print(connDur); Serial.print(F("ms"));
                Serial.print(F(" state=")); Serial.print(g_mqtt.state());
                Serial.print(F(" wifi=")); Serial.println(WiFi.status());
            }
            g_mqtt_was_connected = false;
            g_mqtt_disconnect_ts_ms = now;
            g_mqtt_consec_failures = 0;
            g_mqtt_pub_consec_fail = 0;
            g_mqtt_lastAttemptMs = 0;
            if (g_otaBusy) {
                g_mqtt_lastAttemptMs = (unsigned long)0ul - (unsigned long)600000ul;
            }
        } else if (!g_mqtt_was_connected && nowConn) {
            // Transizione DISCONNESSO -> CONNESSO rilevata nel loop
            // (non è già stata settata in mqttEnsureConnected? a volte sì, a volte no per race)
            // Setto lo stato per evitare log ripetuti.
            g_mqtt_was_connected = true;
            if (g_mqtt_connect_ts_ms == 0) g_mqtt_connect_ts_ms = now;
            // Fallback safety: se per race non è stato settato in mqttEnsureConnected,
            // marca la necessità di publish dei 4x statici on-connect.
            g_throttle_needStaticOnConnect = true;
        }
    }

    (void)mqttEnsureConnected();
    supervisorWatchdogFeed();

    sht31_poll();
    supervisorWatchdogFeed();

    serialPrintCsv();
    supervisorWatchdogFeed();

    wsBroadcast();
    supervisorWatchdogFeed();

    // ========================================================================
    // DISPATCHER MQTT A 2 TIMER + THROTTLING ON-CHANGE + SAFETY HEARTBEAT
    // ========================================================================
    // TIMER A (1s INDIFFERIBILE): diag_uptime_s + diag_wdt_feeds
    //   -> Motivo: l'utente ha espresso preferenza per mantenere questi a 1Hz,
    //      e per definizione cambiano OGNI secondo (dead-band impossibile).
    // TIMER B (base 5s + condizionale): tutti gli altri 21 topic non-statici.
    //   -> Trigger: (forceAll safety 60s) OR (forceSupv 10s delta supv)
    //              OR (ALMENO UN valore > soglia, policy ALL-or-NOTHING).
    // SAFETY MAX INTERVAL (60s): forza publish COMPLETO di tutti i 21 topic
    //   -> Motivo: MQTT retained; se non pubblichiamo per troppo tempo HA vede
    //      i valori come stale. Heartbeat forzato anche se TUTTI invariati.
    if (g_mqtt.connected()) {
        // ---- TIMER A (1s invariato): uptime + wdt feeds ----
        if ((now - g_throttle_lastTimerA_ms) >= 1000UL) {
            g_throttle_lastTimerA_ms = now;
            mqttPublishUptimeWdtOnly();
            supervisorWatchdogFeed();
        }

        // ---- Calcolo flag force ALL-or-NOTHING per TIMER B ----
        // Safety: forziamo publish COMPLETO ogni safety_max_interval (60s default)
        // oppure al primo avvio / dopo config-changed (g_lastPubValid == false).
        const bool forceAllNow = (!g_lastPubValid) ||
                                 ((now - g_throttle_lastSafety_ms) >= (unsigned long)g_config.throttle_safety_max_interval_ms);
        // Forza i 3 campi delta supv (trip_ms, last_good_*) ogni 10s default.
        // Nota: siccome policy ALL-or-NOTHING, se forceSupv è true pubblichiamo
        //       TUTTO, non solo i 3 supv delta.
        const bool forceSupvNow = ((now - g_throttle_lastSupvForce_ms) >= (unsigned long)g_config.throttle_supv_force_interval_ms);
        // Base TIMER B tick (5s default) oppure forceAllNow che bypassa il tick:
        // se scade safety max, non aspettiamo il prossimo tick base 5s.
        const bool timerBTick = forceAllNow ||
                                ((now - g_throttle_lastTimerB_ms) >= (unsigned long)g_config.throttle_timer_data_base_ms);

        if (timerBTick) {
            // Eventuali publish statici (4x on-connect) non ancora inviati dopo
            // l'ultima riconnessione MQTT riuscita: invia PRIMA dei dinamici.
            if (g_throttle_needStaticOnConnect) {
                mqttPublishStaticOnConnect();
                g_throttle_needStaticOnConnect = false;
                supervisorWatchdogFeed();
            }

            // Valutazione policy ALL-or-NOTHING: chiamiamo mqttPublishAllDataSmart
            // SOLAMENTE se almeno una condizione è vera:
            //   (a) forceAllNow (safety 60s o primo publish)
            //   (b) forceSupvNow (forza supv delta ogni 10s)
            //   (c) ALMENO UN valore supera la propria soglia
            const bool anyChanged = throttleAnyExceedsThresholds();
            const bool doSmartPublish = forceAllNow || forceSupvNow || anyChanged;

            if (doSmartPublish) {
                mqttPublishAllDataSmart();
                // Aggiorna i timestamp dei timer solo DOPO la publish riuscita:
                //   - forceAllNow → resetta anche il safety counter
                //   - forceSupvNow → resetta il counter supv
                //   - SEMPRE → resetta il timer B base per evitare burst
                g_throttle_lastTimerB_ms = now;
                if (forceAllNow)   g_throttle_lastSafety_ms = now;
                if (forceSupvNow)  g_throttle_lastSupvForce_ms = now;
            } else {
                // Nessun cambiamento, nessuna force: NESSUN PUBLISH → risparmio!
                // Ma resettiamo comunque timer B base così non scatta di nuovo
                // al prossimo millis() e non teniamo bloccato il tick.
                g_throttle_lastTimerB_ms = now;
            }
            supervisorWatchdogFeed();
        }
    }

    supervisorPoll();
    supervisorWatchdogFeed();
}

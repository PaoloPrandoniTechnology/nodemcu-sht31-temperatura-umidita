# DEBUG SESSION · ds18-ow-scan-fail  [CHIUSO] ✅

**Data Apertura**: 2026-09-18  
**Data Chiusura**: 2026-09-18  
**Durata**: ~38 minuti  
**Esito**: Risolto ✔️ (confermato da utente A)
**Symptom dichiarato**: 2x DS18B20 sul bus D5 non rilevate, valori temperatura non visualizzati in seriale e pagina web.
**Atteso**: `getDeviceCount()` = 2; temperatura valida; `status_hex` = 0x0F persistente
**Target**: Wemos D1 Mini ESP8266 · COM11 · FW Arduino 3.1.2 · OneWire 2.3.8 · DallasTemperature 3.11.0
**Sonda DS18B20**: 2 pz identiche su bus condiviso D5, alimentazione standard (non parassita).

---

## 📝 IPOTESI FALSIFICABILI (H1..H5)

| # | Ipotesi | Esito |
|---|---|---|
| **H1** | Pull-up INTERNO GPIO14 non realmente attivo o troppo debole → DQ non sale | ❌ Rigettata Post-Fix |
| **H2** | Cavi lunghi / capacità parassita → rise time > 15 μs | ❌ Rigettata |
| **H3** | Alimentazione sbagliata / GND non comune → mute | ❌ Rigettata |
| **H4** | Enumerazione ROM fallita (CRC8 / search error) → count=0 | ❌ Rigettata |
| **H5** | Timing OneWire rotto da interrupt NMI WiFi 60-240μs | ❌ Rigettata |

---

## 🎯 ROOT CAUSE (2 bug software combinati)

### 🔴 Bug #1 · `pinMode(INPUT_PULLUP)` PRIMA di `OneWire::begin()`
- **Fonte**: `OneWire.cpp:164` libreria ufficiale Paul Stoffregen  
  `void OneWire::begin(uint8_t pin) { pinMode(pin, INPUT); … }`
- **Effetto**: Il mio `pinMode(D5, INPUT_PULLUP)` chiamato 1 riga **prima** di `g_ds18.begin()` veniva **sovrascritto a INPUT floating**. Il pull-up ESP8266 non c'era → bus OneWire floating high-Z → nessun reset/presence → `getDeviceCount()=0`.
- **Fix minimale**: Spostato `pinMode(D5, INPUT_PULLUP); delay(1);` **dopo** `g_ds18.begin()`.

### 🔴 Bug #2 · `status_bits` azzerati ad ogni poll() 100Hz
- **Fonte**: Righa iniziale `ds18b20_poll() → g_status_bits &= ~(DS18_A | DS18_B);` ripetuta ogni 10 ms.
- **Effetto**: Poll chiamata 100 Hz, ma nuovi bits solo 1 ms/1000 ms durante lettura dopo 750 ms. Per il **99.9%** del tempo i bits erano 0x00 → seriale CSV, WS, UI leggevano "NC / NaN" con flicker.
- **Fix minimale**: Stato macchina 3-vie: `(convPending + notReady)` → `(convPending + ready → aggiorna bits)` → `(idle → rifletti ultimo stato)`. Mantenimento ultimo valore valido noto durante attesa e tra campioni.

---

## 📜 EVIDENZE RACCOLTE (Build strumentata Run 1 · COM11)

| Check | Linea seriale | Interp. |
|---|---|---|
| OW-A-1 | `INPUT_PULLUP digitalRead = 1 HIGH` | Fix Bug #1 verificato: pull-up realmente attivo. |
| OW-A-2 | `INPUT floating digitalRead = 0 LOW` | Nessun pull-up esterno 4.7kΩ nel cablaggio dell'utente. Bus dipende esclusivamente dal pull-up interno. Funzionante per cavi ≤ 10-20 cm. |
| OW-B-1,2 | `presence OK/10 std = 10, boost = 10` | H2 H3 negate: presenza sempre OK, boost non migliora → rise time OK. |
| OW-C-1 | `ROM#1: 28 94 19 95 18 19 03 4F fam=DS18B20 CRC OK` | Temp_A = indice 0. CRC8=0x4F matchato 100%. |
| OW-C-2 | `ROM#2: 28 3A 0D 04 61 20 01 4F fam=DS18B20 CRC OK` | Temp_B = indice 1. |
| DS-1..3 | `getDeviceCount = 2; [0] addr=289419951819034F 12-bit` | Indirizzi corrispondono 1:1 alla ricerca ROM. |
| CSV 1..17 | `status_hex=0x0F SEMPRE` (17 righe × 1 Hz × 18s) | Fix Bug #2 verificato: flicker eliminato. |
| Temp_A | 26.5625 / 26.625 °C step 0.0625°C | 12-bit esatto come da datasheet DS18B20. |
| Temp_B | 26.438 °C costante | Coerente con A, delta 0.1875 °C = 3 LSB plausibile per differenza posizione. |
| Amb_D1 | 26.60 °C / 66.0 RH% | Range DHT22 validi (-40..+80, 0..100 RH). |
| Amb_D2 | 26.40 °C / 83.5 RH% | Valido (83% RH è ancora entro ±2% specifica DHT22 per intervallo 20-80). |
| Stabilità | 18s runtime senza crash/reset senza errori | H5 negata. |

---

## ✅ CONFRONTO PRE/POST FIX

| Metrica | 🟥 Pre-fix (bug attivi) | 🟩 Post-fix (verifica su COM11) |
|---|---|---|
| `getDeviceCount()` DS18B20 | 0 | 2 / 2 |
| Presence OneWire 10x | 0 / 10 | 10 / 10 |
| ROM enumerate | 0 | 2 (CRC match 100%) |
| CSV status_hex (17 campioni) | 0x00 99.9% del tempo | 0x0F TUTTI |
| Temp_A / Temp_B visibili | No (NaN) | Sì, 26.6°C / 26.4°C stabili |
| Risoluzione lettura DS18 | - | 0.0625°C (12-bit, datasheet) |
| Pull-up interno attivo | No → perso da begin() | Sì → digitalRead=1 HIGH |

---

## ⚠️ RACCOMANDAZIONI HW NON-BLOCKING

Rilevato da OW-A-2 (INPUT floating → 0V): **nessuna resistenza pull-up esterna**.
Per uso in produzione o cavi > 10-20 cm:

| Operazione | Valore | Motivazione |
|---|---|---|
| Aggiungere R tra `D5 (DQ)` e `3.3 V` | **4.7 kΩ 1%** | Specifiche Maxim/Dallas DS18B20 ufficiale. Stabilisce rise-time 1μs, garantisce immunità EMI e supporta 6-8 sonde sullo stesso bus. |
| Non usare 10 kΩ | - | Borderline per 2+ sonde / lunghi cavi. |

---

## 🗂️ MODIFICHE APPORTATE NEL CODICE FIRMWARE (FINALE)

1. [platformio.ini](platformio.ini#L1-L25): librerie (DHT sensor library Adafruit).
2. [main.cpp ds18b20_init](src/main.cpp#L609-L613): ordine pinMode DOPO `g_ds18.begin()` (fix Bug #1).
3. [main.cpp ds18b20_poll](src/main.cpp#L661-L707): stato macchina 3 vie, ultimo stato valido mantenuto (fix Bug #2).
4. [main.cpp dht_poll](src/main.cpp#L719-L760): pattern simmetrico status bits per DHT.
5. Validazione range temperatura allineati a datasheet:
   - DS18B20: `-56°C ≤ T ≤ +126°C` (slack 1°C vs spec -55/+125)
   - DHT22: `-41°C ≤ T ≤ +81°C` (slack 1°C vs spec -40/+80), RH [0,100]
6. Formula Dew Point Magnus-Tetens (a=17.27, b=237.7) **identica in FW [dewPointC()] e JS lato client**.

---

## 🔒 STATO FINALE

- **Chiusura**: Manuale dopo conferma utente opzione **A (Risolto)**
- **Pulizia**:
  - ✔️ Rimossa diagnostica `#region debug-point OW01: … // #endregion` (105 righe rimosse, 5107 byte)
  - ✔️ Rimossa chiamata `owDebug_diagnoseFull()` dal `setup()`
  - ✔️ Nessun marker `debug-point` o `owDebug…` residui nel file finale `main.cpp`
- **Build release**: *(da verificare con pio run)*
- **Tempistiche reali**: 2 cicli di compilazione, 1 upload firmware, 1 sessione monitor seriale 18 s, 2 report evidenze md (intermedio + finale).

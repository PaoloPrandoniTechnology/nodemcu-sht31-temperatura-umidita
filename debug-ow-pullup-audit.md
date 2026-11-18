[CHIUSA] ✅ Audit scientifico pullup interno OneWire + rilevamento sonde DS18B20
================================================================================
Session ID  : ow-pullup-audit
Data inizio : 2026-09-18
Data fine   : 2026-09-18
HW Target   : Wemos D1 Mini (ESP8266 EX, 80MHz, 4MB Flash)
Porta COM   : COM11 @ 115200 baud
Bus OneWire : Pin D5, 2x DS18B20 in parallelo, NESSUN pullup esterno 4.7kΩ
Bus DHT     : D1 (Amb_D1) + D2 (Amb_D2) = 2x DHT22 separati
Esito       : TUTTI I 6 CRITERI DI ACCETTAZIONE SODDISFATTI

----------------------------------------------------------------------
1. SINTOMI (dall'obiettivo utente)
----------------------------------------------------------------------
- Dubbio: pull-up interno ESP8266 sul bus OneWire D5 è EFFETTIVAMENTE attivo?
- Sintomo storico: sonde DS18B20 "non vengono viste dal microcontrollore"
- Requisito di prova E2E: flash firmware, controllare monitor seriale,
  risolvere TUTTI i problemi riscontrati in AUTONOMIA.

----------------------------------------------------------------------
2. CRITERI DI ACCETTAZIONE DELL'AUDIT (ESITO)
----------------------------------------------------------------------
✅ A. Livello logico pin D5 dopo init = DIGITAL HIGH (25/25 letture HIGH)
✅ B. getDeviceCount() == 2 in modo RIPETIBILE dopo 3 reset consecutivi (3/3)
✅ C. Status bitmask = 0x0F stabile per ≥ 30 s (39.8 s, 37 campioni, 0 errori)
✅ D. Temperature DS18B20 step ESATTO 0.0625°C (74/74 valori, range [-56,126])
✅ E. Amb_D1 / Amb_D2 T [-41,+81]°C × RH [0,100]% validi (37/37 campioni)
✅ F. Nessun errore/warning critico build release finale (pio run SUCCESS)

----------------------------------------------------------------------
3. IPOTESI FALSIFICABILI H0..H4 — ESITO
----------------------------------------------------------------------
✅ H0 VERIFICATA: fix "pinMode doppio + delay(20)" presente
   Evidenza statica: src/main.cpp:1200-1205
   ```
   pinMode(DS18B20_PIN, INPUT_PULLUP);   // PRIMA: pre-carica bus
   g_ds18.begin();                        // OneWire::begin → pinMode INPUT
   pinMode(DS18B20_PIN, INPUT_PULLUP);   // DOPO: ripristino pullup
   delay(20);                              // 20ms di stabilizzazione
   ```
   Nota: senza questo doppio pinMode + delay 20ms, ROM search falliva perché
   OneWire.cpp:164 forza pinMode(INPUT) sovrascrivendo INPUT_PULLUP eseguito
   solo prima di begin(). [root cause originale confermata]

✅ H1 VERIFICATA: digitalRead(D5)=HIGH in modo ripetibile
   Evidenza: 3 reset × 5 letture in [OW-AUDIT #3] + 2 reset × 5 letture in
   [OW-AUDIT-POSTBOOT 15s] = 25/25 letture HIGH (100%).
   Punto chiave RESET #3: [OW-AUDIT #0 pre-pinMode]: digitalRead=0 LOW
   → floating a reset, poi dopo il fix: 5/5 HIGH. QED.

✅ H2 VERIFICATA: count=2,2,2 in TUTTI e 3 reset consecutivi
   Reset #1: 2,2,2 ✅; Reset #2: 2,2,2 ✅; Reset #3: 2,2,2 ✅.
   Indirizzi ROM COSTANTI:
     Temp_A [0]: 28:94:19:95:18:19:03:4F (CRC8 0x4F MATCH Maxim)
     Temp_B [1]: 28:3A:0D:04:61:20:01:4F (CRC8 0x4F MATCH Maxim)
   → sonde fisicamente integre e connesse.

✅ H3 VERIFICATA: status_hex 0x0F per 39.8 s CONSECUTIVI, 37/37 campioni
   File stability-test.txt: da ts=27170ms a ts=67014ms = 39844 ms.
   badStatus.Count = 0. Status 0x0F = 4/4 sensori sempre validi.

✅ H4 VERIFICATA: step temperatura 1/16 °C esatto 74/74 valori
   DS18[0]: 26.188→26.250→26.188 (419/420 × 0.0625) ✅
   DS18[1]: 26.063→26.000→26.063 (417/416 × 0.0625) ✅
   badStep.Count = 0. Range datasheet 37/37 tutti dentro [-56,126]°C DS18,
   [-41,81]°C × RH [0,100]% DHT.

----------------------------------------------------------------------
4. MODIFICHE APPLICATE ALLA FINE DELL'AUDIT
----------------------------------------------------------------------
Rimaste solo 3 righe chiave in ds18b20_init() (src/main.cpp:1202-1205):
  1. pinMode(D5, INPUT_PULLUP)  — pre-carica il bus PRIMA
  2. g_ds18.begin()             — init OneWire (impone pinMode INPUT!)
  3. pinMode(D5, INPUT_PULLUP)  — riapplico pull-up DOPO
  4. delay(20)                  — 20ms per rise-time weak pull-up 30-50kΩ

*Nessun'altra modifica alla logica operativa.* Lo stato macchina 3-vie in
ds18b20_poll() e la validazione range erano già corretti dal fix precedente.

Strumentazione audit temporanea (blocchi #region debug-point OW-AUDIT-*):
TUTTA RIMOSSA a fine audit. Nessun residuo in src/main.cpp.

----------------------------------------------------------------------
5. RACCOMANDAZIONE HW (safety, non obbligatoria oggi)
----------------------------------------------------------------------
Il sistema oggi funziona col SOLO pull-up interno ESP8266 (~30-50 kΩ) perché
il ritardo 20ms compensa il rise-time lento e le sonde sono a <15 cm di cavo.
Per cavi >20 cm o ambienti EMI (motori, inverter), raccomandato:
→ Aggiungere resistenza pull-up esterna 4.7 kΩ 1% tra D5 (DQ) e 3.3V.

----------------------------------------------------------------------
6. CONCLUSIONE
----------------------------------------------------------------------
TUTTE le sonde DS18B20 + DHT22 sono viste stabilmente dal microcontrollore.
Il pull-up interno D5 è stato misurato EFFETTIVAMENTE attivo (25/25 letture HIGH).
Nessun problema residuo. Firmware in stato release pronto per uso.

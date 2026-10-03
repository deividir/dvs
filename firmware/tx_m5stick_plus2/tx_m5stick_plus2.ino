/*
========================================================
 M5STICK TRANSMITTER (M5StickC Plus / Plus2 / StickS3) via M5Unified
 DVS / Phase DIY - ESP-NOW low latency motion packet
 Mesmo protocolo, TDMA e filtro do TX ESP32-C3 + BMI270, com LCD.

 Botoes:
   BtnA (frente)        : alterna Sem Deck -> A -> B
   BtnB (lateral)       : troca de pagina (principal / diagnostico)
   BtnB (segurar ~1s)   : recalibra o gyro (deixe o prato parado)

 Biblioteca necessaria: M5Unified (instala M5GFX junto).
 Placa no Arduino IDE: a do seu modelo (M5StickC Plus / Plus2 / StickS3).
 USE_LONG_RANGE precisa ser IGUAL ao RX e ao outro TX.
========================================================
*/

#include <M5Unified.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <Preferences.h>
#include <math.h>

// ============ CONFIGURACAO ============
#define TX_DECK_ID 1            // Deck B para operar junto com o TX C3 configurado como Deck A
#define ESPNOW_CHANNEL 11       // canal fixo de teste, igual ao RX
#define FORCE_FIXED_CHANNEL 1   // 1 = usa diretamente o canal do RX durante o teste
#define FORCE_DECK_ID 1         // 1 = ignora deck antigo salvo na NVS durante o teste
#define USE_LONG_RANGE 0        // modo normal: menor tempo no ar e melhor para alta taxa de pacotes
#define USE_FIXED_RATE 0        // deixa a taxa PHY adaptativa durante o teste
#define FIXED_RATE WIFI_PHY_RATE_24M
#define TX_POWER_QDBM 52        // unidades de 0,25 dBm (52 = aproximadamente 13 dBm)
#define FORCE_RPM_MULTIPLIER 1  // 1 = ignora o valor salvo na NVS
#define FIXED_RPM_MULTIPLIER 1.0000f

#define GYRO_AXIS 2             // 0=X 1=Y 2=Z (Z = perpendicular a tela, stick deitado no prato)
#define GYRO_SIGN (-1.0f)       // mesmo sinal do TX C3. Se o sentido sair invertido, use +1.0f
#define LCD_ROTATION 1          // 1 = paisagem; use 3 para girar 180 graus
#define LCD_BRIGHTNESS 60       // menor brilho = mais autonomia (bateria de 200 mAh)
#define MPU6886_FAST_ODR 1      // StickC Plus/Plus2: sobe a amostragem do gyro de ~166 Hz para 1 kHz
// ======================================

uint8_t receiverMAC[] = { 0x14, 0xC1, 0x9F, 0x2C, 0xDE, 0x7C };

#define SEND_RATE_HZ 100
#define SEND_INTERVAL_US (1000000UL / SEND_RATE_HZ)
#define TDMA_SLOT_WIDTH_US (SEND_INTERVAL_US / 2)
#define TDMA_SEND_GUARD_US 500
#define RX_ANCHOR_TIMEOUT_MS 5000
#define PAIR_CHANNEL_DWELL_MS 120
#define PAIR_HELLO_INTERVAL_MS 40
#define HANDSHAKE_TIMEOUT_MS 5000
#define SEND_INFLIGHT_WATCHDOG_US 20000

#define PROTOCOL_VERSION 2
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4
#define MSG_CALIB 5
#define MSG_CALIB_ACK 6
#define MSG_SET_PARAM 7
#define MSG_CFG_ACK 8
#define MSG_CFG_GET 9
#define CFG_DECK_ID 1
#define CFG_ALPHA_SLOW 2
#define CFG_ALPHA_FAST 3
#define CFG_FAST_THRESHOLD 4

static const uint8_t pairChannels[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
static const uint8_t pairChannelCount = sizeof(pairChannels) / sizeof(pairChannels[0]);

// Filtro assimetrico (iguais ao TX C3; podem ser mudados pelo RX via FILTER)
float ALPHA_SLOW = 0.50f;
float ALPHA_FAST = 0.70f;
float FAST_THRESHOLD_RPM = 0.15f;
float DEADZONE_RPM = 0.20f;
float rpmMultiplier = 1.0f;     // ajustado via CALIB_A/B do RX e salvo em NVS

// Auto-calibracao do offset do gyro (parado)
#define AUTO_CALIBRATION_STABLE_SAMPLES 400
#define AUTO_CALIBRATION_SAMPLE_DELAY_MS 2
#define AUTO_CALIBRATION_STABLE_DPS_DELTA 2.0f
#define AUTO_CALIBRATION_MIN_COMPARE_SAMPLES 20
#define AUTO_CALIBRATION_MAX_DPS 6.0f
#define AUTO_CALIBRATION_TIMEOUT_MS 4000
#define AUTO_TRIM_WINDOW_MS 2000
#define AUTO_TRIM_STABLE_DELTA_DPS 2.0f
#define AUTO_TRIM_MAX_ABS_DPS 50.0f

typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  int16_t rpmCenti;
  int16_t gyroRaw;  // dps * 10
  uint8_t batteryPct;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

dvs_packet packet;

volatile uint8_t deckId = TX_DECK_ID;   // 0 = sem deck
uint8_t activeChannel = ESPNOW_CHANNEL;
bool hopPending = true;
int16_t hopIndex = 0;
uint32_t lastHopMillis = 0;
uint32_t lastPairHelloMillis = 0;
bool wasReceiverReady = false;

volatile bool receiverReady = false;
volatile uint32_t lastReceiverReplyMillis = 0;

// TDMA: relogio do RX estimado por offset (local - rx) com filtro de minimo
volatile uint32_t rxOffsetUs = 0;
volatile uint32_t rxClockSeenMillis = 0;
volatile bool rxClockSynced = false;
uint32_t tdmaLastSentFrame = 0xFFFFFFFFUL;

float gyroOffset = 0.0f;   // dps
float filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;
volatile uint8_t batteryLevelPct = 100;

// Portal de envio: so 1 pacote por vez no radio (todos os envios passam por radioSend)
volatile bool espNowSendInFlight = false;
volatile uint32_t lastSendUs = 0;
volatile uint32_t espNowSendOk = 0;
volatile uint32_t espNowSendFail = 0;
uint32_t txSentCount = 0;
uint32_t inflightWatchdogTrips = 0;

// ---- estado compartilhado com a UI ----
volatile float uiRpm = 0.0f;
volatile float uiDps = 0.0f;
volatile int8_t rxRssi = -127;
volatile uint16_t statSentPerSec = 0, statOkPerSec = 0, statFailPerSec = 0;
volatile bool calibrating = false;
volatile bool uiReqCycleDeck = false;
volatile bool uiReqRecal = false;
volatile uint8_t uiPage = 0;

// ================= NVS (gravacao adiada, nunca no callback) =================
#define DIRTY_FILTER 1
#define DIRTY_MULT 2
#define DIRTY_DECK 4
#define DIRTY_CHAN 8
volatile uint8_t nvsDirty = 0;
volatile uint32_t nvsDirtyMs = 0;

void markDirty(uint8_t f) { nvsDirty |= f; nvsDirtyMs = millis(); }

void loadSettingsNvs() {
  Preferences prefs;
  if (!prefs.begin("dvs", true)) return;
  float m = prefs.getFloat("rpmMult", -1.0f);
  if (m > 0.5f && m < 2.0f) rpmMultiplier = m;
  uint8_t d = prefs.getUChar("deckId", 0);
  if (d == 1 || d == 2) deckId = d;
  float s = prefs.getFloat("alphaSlow", -1.0f);
  float f = prefs.getFloat("alphaFast", -1.0f);
  float t = prefs.getFloat("alphaThr", -1.0f);
  if (s > 0.05f && s < 0.99f) ALPHA_SLOW = s;
  if (f > 0.05f && f < 0.99f) ALPHA_FAST = f;
  if (t > 0.01f && t < 10.0f) FAST_THRESHOLD_RPM = t;
  int ch = prefs.getInt("chan", 0);
  if (ch >= 1 && ch <= 13) activeChannel = (uint8_t)ch;
  prefs.end();
}

void flushNvs() {
  if (!nvsDirty || millis() - nvsDirtyMs < 1000) return;
  uint8_t flags = nvsDirty;
  nvsDirty = 0;
  Preferences prefs;
  if (!prefs.begin("dvs", false)) return;
  if (flags & DIRTY_FILTER) {
    prefs.putFloat("alphaSlow", ALPHA_SLOW);
    prefs.putFloat("alphaFast", ALPHA_FAST);
    prefs.putFloat("alphaThr", FAST_THRESHOLD_RPM);
  }
  if (flags & DIRTY_MULT) prefs.putFloat("rpmMult", rpmMultiplier);
  if ((flags & DIRTY_DECK) && (deckId == 1 || deckId == 2)) prefs.putUChar("deckId", deckId);
  if (flags & DIRTY_CHAN) prefs.putInt("chan", (int)activeChannel);
  prefs.end();
}

// ================= IMU =================
static inline bool readGyroDps(float *dps) {
  M5.Imu.update();
  const auto &d = M5.Imu.getImuData();
  *dps = (GYRO_AXIS == 0) ? d.gyro.x : (GYRO_AXIS == 1) ? d.gyro.y : d.gyro.z;
  return true;
}

void autoCalibrateGyro() {
  int stableSamples = 0;
  float sum = 0.0f;
  uint32_t startCal = millis();
  while (stableSamples < AUTO_CALIBRATION_STABLE_SAMPLES) {
    float dps = 0.0f;
    readGyroDps(&dps);
    bool reset = false;
    if (fabsf(dps) > AUTO_CALIBRATION_MAX_DPS) {
      reset = true;
    } else if (stableSamples >= AUTO_CALIBRATION_MIN_COMPARE_SAMPLES) {
      float mean = sum / stableSamples;
      if (fabsf(dps - mean) > AUTO_CALIBRATION_STABLE_DPS_DELTA) reset = true;
    }
    if (reset) {
      stableSamples = 0;
      sum = 0.0f;
      if (millis() - startCal > AUTO_CALIBRATION_TIMEOUT_MS) break;
      delay(AUTO_CALIBRATION_SAMPLE_DELAY_MS);
      continue;
    }
    sum += dps;
    stableSamples++;
    delay(AUTO_CALIBRATION_SAMPLE_DELAY_MS);
  }
  if (stableSamples >= AUTO_CALIBRATION_STABLE_SAMPLES) {
    gyroOffset = sum / (float)stableSamples;
  } else {
    gyroOffset = 0.0f;
    Serial.println("Calibracao ignorada (prato em movimento): offset = 0");
  }
  filteredRPM = 0.0f;
}

void recalibrate() {
  calibrating = true;
  autoCalibrateGyro();
  calibrating = false;
}

static inline float dpsToRPM(float dps) {
  return GYRO_SIGN * ((dps - gyroOffset) / 6.0f) * rpmMultiplier;
}

void autoTrimGyroOffset(float dps) {
  static uint32_t windowStart = 0;
  static float wMin = 1e9f, wMax = -1e9f, wSum = 0.0f;
  static uint32_t wCount = 0;
  wMin = fminf(wMin, dps);
  wMax = fmaxf(wMax, dps);
  wSum += dps;
  wCount++;
  if (windowStart == 0) windowStart = millis();
  if (millis() - windowStart >= AUTO_TRIM_WINDOW_MS) {
    if (wCount > 0 && (wMax - wMin) <= AUTO_TRIM_STABLE_DELTA_DPS) {
      float meanDps = wSum / (float)wCount;
      if (fabsf(meanDps) <= AUTO_TRIM_MAX_ABS_DPS) gyroOffset = meanDps;
    }
    wMin = 1e9f; wMax = -1e9f; wSum = 0.0f; wCount = 0;
    windowStart = millis();
  }
}

// ================= RADIO =================
void applyPeerRate() {
#if !USE_LONG_RANGE && USE_FIXED_RATE && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0))
  esp_now_rate_config_t rc = {};
  rc.phymode = WIFI_PHY_MODE_11G;
  rc.rate = FIXED_RATE;
  esp_now_set_peer_rate_config(receiverMAC, &rc);
#endif
}

void addReceiverPeer(uint8_t ch) {
  if (esp_now_is_peer_exist(receiverMAC)) esp_now_del_peer(receiverMAC);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, receiverMAC, 6);
  p.channel = ch;
  p.encrypt = false;
  esp_now_add_peer(&p);
  applyPeerRate();
}

bool radioSend(const dvs_packet &p) {
  if (espNowSendInFlight) return false;
  espNowSendInFlight = true;
  lastSendUs = micros();
  esp_err_t e = esp_now_send(receiverMAC, (const uint8_t *)&p, sizeof(p));
  if (e != ESP_OK) {
    espNowSendInFlight = false;
    espNowSendFail++;
    Serial.printf("RADIO_SEND_ERR type=%u err=%d ch=%u\n",
                  (unsigned)p.msgType, (int)e, (unsigned)activeChannel);
    return false;
  }
  return true;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  espNowSendInFlight = false;
  if (status == ESP_NOW_SEND_SUCCESS) espNowSendOk++; else espNowSendFail++;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("RADIO_SEND_FAIL ch=%u\n", (unsigned)activeChannel);
  }
}

// ---- TDMA: relogio do RX ----
void parseRxClockBeacon(uint32_t rxTs, uint32_t localUs) {
  uint32_t off = localUs - rxTs;   // = (local - rx), inclui latencia (sempre >= 0)
  if (!rxClockSynced) {
    rxOffsetUs = off;
    tdmaLastSentFrame = 0xFFFFFFFFUL;
  } else {
    int32_t d = (int32_t)(off - rxOffsetUs);
    if (d < 0 || d > 50000) rxOffsetUs = off;  // novo minimo, ou RX reiniciou
    else rxOffsetUs += 5;                      // acompanha deriva de cristal
  }
  rxClockSeenMillis = millis();
  rxClockSynced = true;
}
static inline uint32_t estimatedRxMicrosNow() { return micros() - rxOffsetUs; }
static inline uint32_t rxTimeToLocal(uint32_t rxUs) { return rxUs + rxOffsetUs; }
static inline uint32_t tdmaSlotOffsetUs() { return (deckId == 1) ? 0 : TDMA_SLOT_WIDTH_US; }
static inline bool rxClockIsFresh() {
  return rxClockSynced && (millis() - rxClockSeenMillis < RX_ANCHOR_TIMEOUT_MS);
}

// ---- comandos remotos: o callback so enfileira, o loop() aplica ----
typedef struct { uint8_t type; uint8_t param; int16_t value; } msg_item_t;
#define Q_SIZE 8
static msg_item_t cmdQ[Q_SIZE];
static volatile uint8_t cmdHead = 0, cmdTail = 0;
static msg_item_t ackQ[Q_SIZE];
static uint8_t ackHead = 0, ackTail = 0;

void pushAck(uint8_t type, uint8_t param, int16_t value) {
  uint8_t n = (ackHead + 1) % Q_SIZE;
  if (n == ackTail) return;
  ackQ[ackHead] = { type, param, value };
  ackHead = n;
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  uint32_t t = micros();  // captura ANTES de qualquer outro trabalho
  if (len != sizeof(dvs_packet)) return;
  dvs_packet in;
  memcpy(&in, dataPtr, sizeof(in));
  if (in.version != PROTOCOL_VERSION || in.deckId != deckId) return;

  if (in.msgType == MSG_WELCOME || in.msgType == MSG_PING) {
    if (info && info->rx_ctrl) rxRssi = (int8_t)info->rx_ctrl->rssi;
    parseRxClockBeacon(in.timestampMicros, t);
    lastReceiverReplyMillis = millis();
    receiverReady = true;
    Serial.printf("RADIO_RX type=%u deck=%u ch=%u rssi=%d\n",
                  (unsigned)in.msgType, (unsigned)in.deckId,
                  (unsigned)activeChannel, (int)rxRssi);
  } else if (in.msgType == MSG_CALIB || in.msgType == MSG_SET_PARAM || in.msgType == MSG_CFG_GET) {
    uint8_t n = (cmdHead + 1) % Q_SIZE;
    if (n != cmdTail) {
      cmdQ[cmdHead] = { in.msgType, in.batteryPct, in.rpmCenti };
      cmdHead = n;
    }
  }
}

void startPairing() {
  receiverReady = false;
  wasReceiverReady = false;
  rxClockSynced = false;
  hopPending = true;
  lastHopMillis = 0;
  lastPairHelloMillis = 0;
  // Durante o diagnostico, inicia diretamente no canal fixo do RX.
#if FORCE_FIXED_CHANNEL
  activeChannel = ESPNOW_CHANNEL;
  hopIndex = -1;
  return;
#else
  // o primeiro salto cai no ultimo canal conhecido
  int idx = 0;
  for (int i = 0; i < pairChannelCount; i++) if (pairChannels[i] == activeChannel) idx = i;
  hopIndex = (int16_t)(idx - 1);
#endif
}

void setDeck(uint8_t newId) {
  if ((newId != 1 && newId != 2) || newId == deckId) return;
  deckId = newId;
  startPairing();
  markDirty(DIRTY_DECK);
}

void cycleDeck() {
  deckId = (deckId + 1) % 3;
  if (deckId == 0) {
    receiverReady = false;
    wasReceiverReady = false;
    rxClockSynced = false;
  } else {
    startPairing();
    markDirty(DIRTY_DECK);
  }
}

void serviceCommands() {
  while (cmdTail != cmdHead) {
    msg_item_t c = cmdQ[cmdTail];
    cmdTail = (cmdTail + 1) % Q_SIZE;
    if (c.type == MSG_CALIB) {
      float m = (float)c.value / 10000.0f;
      if (m > 0.5f && m < 2.0f) {
        rpmMultiplier = m;
        markDirty(DIRTY_MULT);
        pushAck(MSG_CALIB_ACK, 0, (int16_t)lroundf(rpmMultiplier * 10000.0f));
      }
    } else if (c.type == MSG_SET_PARAM) {
      float f = (float)c.value / 1000.0f;
      bool ok = false;
      switch (c.param) {
        case CFG_DECK_ID: setDeck((uint8_t)c.value); ok = true; break;
        case CFG_ALPHA_SLOW: if (f > 0.05f && f < 0.99f) { ALPHA_SLOW = f; ok = true; } break;
        case CFG_ALPHA_FAST: if (f > 0.05f && f < 0.99f) { ALPHA_FAST = f; ok = true; } break;
        case CFG_FAST_THRESHOLD: if (f > 0.01f && f < 10.0f) { FAST_THRESHOLD_RPM = f; ok = true; } break;
      }
      if (ok) {
        if (c.param != CFG_DECK_ID) markDirty(DIRTY_FILTER);
        pushAck(MSG_CFG_ACK, c.param, c.value);
      }
    } else if (c.type == MSG_CFG_GET) {
      pushAck(MSG_CFG_ACK, CFG_ALPHA_SLOW, (int16_t)lroundf(ALPHA_SLOW * 1000.0f));
      pushAck(MSG_CFG_ACK, CFG_ALPHA_FAST, (int16_t)lroundf(ALPHA_FAST * 1000.0f));
      pushAck(MSG_CFG_ACK, CFG_FAST_THRESHOLD, (int16_t)lroundf(FAST_THRESHOLD_RPM * 1000.0f));
    }
  }
}

void serviceAcks() {
  if (ackTail == ackHead || espNowSendInFlight || !receiverReady) return;
  msg_item_t a = ackQ[ackTail];
  dvs_packet p = {};
  p.msgType = a.type;
  p.version = PROTOCOL_VERSION;
  p.deckId = deckId;
  p.batteryPct = a.param;
  p.rpmCenti = a.value;
  p.timestampMicros = micros();
  if (radioSend(p)) ackTail = (ackTail + 1) % Q_SIZE;
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(TX_POWER_QDBM);
#if USE_LONG_RANGE
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#else
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
#endif
  esp_wifi_set_channel(activeChannel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ESP-NOW");
    M5.Display.fillScreen(TFT_RED);
    M5.Display.drawString("ERRO ESP-NOW", 10, 50);
    while (true) delay(1000);
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  addReceiverPeer(activeChannel);
  Serial.printf("MAC: %s  LR=%d\n", WiFi.macAddress().c_str(), USE_LONG_RANGE);
}

void statsTick(uint32_t nowMs) {
  static uint32_t last = 0, pOk = 0, pFail = 0, pSent = 0;
  static uint8_t n = 0;
  if (nowMs - last < 1000) return;
  last = nowMs;
  uint32_t ok = espNowSendOk, fail = espNowSendFail, sent = txSentCount;
  statOkPerSec = (uint16_t)(ok - pOk);
  statFailPerSec = (uint16_t)(fail - pFail);
  statSentPerSec = (uint16_t)(sent - pSent);
  pOk = ok; pFail = fail; pSent = sent;
  if (++n >= 5) {
    n = 0;
    Serial.printf("TX_SEND_STAT sent/s=%u ok/s=%u fail/s=%u wdTrips=%lu\n",
                  (unsigned)statSentPerSec, (unsigned)statOkPerSec, (unsigned)statFailPerSec,
                  (unsigned long)inflightWatchdogTrips);
  }
}

// ================= UI (task separada no core 0, prioridade baixa) =================
#define HIST_N 116
static float hist[HIST_N];
static uint8_t histPos = 0;
static M5Canvas canvas(&M5.Display);
static bool canvasOk = false;

static const char *deckName() { return deckId == 1 ? "DECK A" : deckId == 2 ? "DECK B" : "SEM DECK"; }

void drawUi() {
  lgfx::LovyanGFX *g = canvasOk ? (lgfx::LovyanGFX *)&canvas : (lgfx::LovyanGFX *)&M5.Display;
  const int W = g->width(), H = g->height();

  float rpm = uiRpm, dps = uiDps;
  bool ready = receiverReady;
  uint8_t bat = batteryLevelPct;
  uint8_t d = deckId;

  hist[histPos] = rpm;
  histPos = (histPos + 1) % HIST_N;

  uint16_t deckCol = (d == 1) ? TFT_CYAN : (d == 2) ? TFT_ORANGE : TFT_DARKGREY;
  g->fillScreen(TFT_BLACK);
  g->fillRect(0, 0, W, 22, deckCol);
  g->setTextColor(TFT_BLACK, deckCol);
  g->setFont(&fonts::Font4);
  g->setTextDatum(middle_left);
  g->drawString(deckName(), 6, 11);
  char b[48];
  snprintf(b, sizeof(b), "%u%%", (unsigned)bat);
  g->setFont(&fonts::Font2);
  g->setTextDatum(middle_right);
  g->drawString(b, W - 6, 11);

  if (calibrating) {
    g->setTextColor(TFT_YELLOW, TFT_BLACK);
    g->setFont(&fonts::Font4);
    g->setTextDatum(middle_center);
    g->drawString("CALIBRANDO...", W / 2, H / 2 - 8);
    g->setFont(&fonts::Font2);
    g->drawString("deixe o prato parado", W / 2, H / 2 + 20);
  } else if (uiPage == 0) {
    // status do link logo abaixo do cabecalho
    g->setFont(&fonts::Font2);
    g->setTextDatum(top_left);
    if (d == 0) {
      g->setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      snprintf(b, sizeof(b), "A: escolher deck");
    } else if (ready) {
      g->setTextColor(TFT_GREEN, TFT_BLACK);
      snprintf(b, sizeof(b), "LINK ch%u  %d dBm", (unsigned)activeChannel, (int)rxRssi);
    } else {
      g->setTextColor(TFT_YELLOW, TFT_BLACK);
      snprintf(b, sizeof(b), "BUSCANDO RX  ch%u", (unsigned)activeChannel);
    }
    g->drawString(b, 6, 25);

    // RPM grande
    snprintf(b, sizeof(b), "%.2f", rpm);
    g->setFont(&fonts::Font7);
    g->setTextColor(ready ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
    g->drawString(b, 6, 42);
    g->setFont(&fonts::Font2);
    g->setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    g->drawString("RPM", W - 36, 70);

    // pitch e gyro
    g->setFont(&fonts::Font4);
    if (fabsf(rpm) > 5.0f) {
      float pitch = (fabsf(rpm) / 33.3333f - 1.0f) * 100.0f;
      g->setTextColor(fabsf(pitch) < 0.05f ? TFT_GREEN : TFT_YELLOW, TFT_BLACK);
      snprintf(b, sizeof(b), "%+.2f%%", pitch);
    } else {
      g->setTextColor(TFT_DARKGREY, TFT_BLACK);
      snprintf(b, sizeof(b), "--");
    }
    g->drawString(b, 6, 96 - 18);
    g->setFont(&fonts::Font2);
    g->setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    g->setTextDatum(top_right);
    snprintf(b, sizeof(b), "%.1f dps", dps);
    g->drawString(b, W - 6, 80);

    // scope com autoescala (span minimo 1 RPM)
    const int sy = 100, sh = H - sy - 2;
    g->drawRect(2, sy, W - 4, sh, TFT_DARKGREY);
    float mn = 1e9f, mx = -1e9f;
    for (int i = 0; i < HIST_N; i++) { mn = fminf(mn, hist[i]); mx = fmaxf(mx, hist[i]); }
    float span = fmaxf(mx - mn, 1.0f);
    float mid = (mx + mn) * 0.5f;
    int px = 0, py = 0;
    for (int i = 0; i < HIST_N; i++) {
      float v = hist[(histPos + i) % HIST_N];
      int x = 4 + (i * (W - 8)) / (HIST_N - 1);
      int y = sy + sh / 2 - (int)((v - mid) / span * (sh - 6));
      if (i > 0) g->drawLine(px, py, x, y, deckCol);
      px = x; py = y;
    }
  } else {
    g->setFont(&fonts::Font2);
    g->setTextDatum(top_left);
    g->setTextColor(TFT_WHITE, TFT_BLACK);
    int y = 26;
    const int lh = 16;
    snprintf(b, sizeof(b), "ch %u  rssi %d dBm  LR %s", (unsigned)activeChannel, (int)rxRssi, USE_LONG_RANGE ? "ON" : "OFF");
    g->drawString(b, 6, y); y += lh;
    snprintf(b, sizeof(b), "env %u/s ok %u/s falha %u/s", (unsigned)statSentPerSec, (unsigned)statOkPerSec, (unsigned)statFailPerSec);
    g->drawString(b, 6, y); y += lh;
    snprintf(b, sizeof(b), "seq %lu  wd %lu", (unsigned long)sequenceNumber, (unsigned long)inflightWatchdogTrips);
    g->drawString(b, 6, y); y += lh;
    snprintf(b, sizeof(b), "gyro off %+.3f dps", gyroOffset);
    g->drawString(b, 6, y); y += lh;
    snprintf(b, sizeof(b), "a %.2f/%.2f thr %.2f", ALPHA_SLOW, ALPHA_FAST, FAST_THRESHOLD_RPM);
    g->drawString(b, 6, y); y += lh;
    snprintf(b, sizeof(b), "mult %.4f  sync %s", rpmMultiplier, rxClockSynced ? "SIM" : "NAO");
    g->drawString(b, 6, y); y += lh;
    snprintf(b, sizeof(b), "heap %luk", (unsigned long)(ESP.getFreeHeap() / 1024));
    g->drawString(b, 6, y);
  }

  if (canvasOk) canvas.pushSprite(0, 0);
}

void uiTask(void *) {
  uint32_t lastBat = 0;
  for (;;) {
    M5.update();
    if (M5.BtnA.wasClicked()) uiReqCycleDeck = true;
    if (M5.BtnB.wasHold()) uiReqRecal = true;
    else if (M5.BtnB.wasClicked()) uiPage = (uiPage + 1) % 2;

    if (millis() - lastBat > 2000) {
      lastBat = millis();
      int32_t lv = M5.Power.getBatteryLevel();
      if (lv >= 0) batteryLevelPct = (uint8_t)constrain((int)lv, 0, 100);
    }
    drawUi();
    vTaskDelay(pdMS_TO_TICKS(33));
  }
}

// ================= SETUP / LOOP =================
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(LCD_ROTATION);
  M5.Display.setBrightness(LCD_BRIGHTNESS);
  M5.Display.fillScreen(TFT_BLACK);

  canvasOk = (canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr);

  if (!M5.Imu.isEnabled()) {
    M5.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5.Display.drawString("IMU nao encontrado", 10, 50);
    while (true) delay(1000);
  }

#if MPU6886_FAST_ODR
  if (M5.Imu.getType() == m5::imu_t::imu_mpu6886) {
    // SMPLRT_DIV (0x19): taxa = 1 kHz / (1 + div). O padrao do M5Unified e ~div 5 (~166 Hz),
    // o que da amostras repetidas/velhas a 200 Hz de envio. Com 0 o dado tem no max 1 ms.
    uint8_t before = M5.In_I2C.readRegister8(0x68, 0x19, 400000);
    M5.In_I2C.writeRegister8(0x68, 0x19, 0x00, 400000);
    uint8_t after = M5.In_I2C.readRegister8(0x68, 0x19, 400000);
    Serial.printf("MPU6886 SMPLRT_DIV: %u -> %u\n", (unsigned)before, (unsigned)after);
  }
#endif

  loadSettingsNvs();
#if FORCE_DECK_ID
  deckId = TX_DECK_ID;
  Serial.printf("DECK_FORCADO=%u\n", (unsigned)deckId);
#endif
#if FORCE_RPM_MULTIPLIER
  rpmMultiplier = FIXED_RPM_MULTIPLIER;
  Serial.printf("RPM_MULTIPLIER_FIXO=%.4f\n", rpmMultiplier);
#endif
  setupEspNow();
  startPairing();
  xTaskCreatePinnedToCore(uiTask, "ui", 6144, NULL, 1, NULL, 0);

  recalibrate();
  randomSeed(micros() ^ (deckId << 16));
  nextSendMicros = micros();
  Serial.printf("RPM_MULTIPLIER_ATIVO=%.4f deck=%u\n", rpmMultiplier, (unsigned)deckId);
}

void loop() {
  uint32_t nowMs = millis();

  if (uiReqCycleDeck) { uiReqCycleDeck = false; cycleDeck(); }
  if (uiReqRecal) { uiReqRecal = false; recalibrate(); }

  serviceCommands();
  flushNvs();
  statsTick(nowMs);

  // Watchdog do portal: se o callback de envio se perder, nao trava o TX.
  if (espNowSendInFlight && (uint32_t)(micros() - lastSendUs) > SEND_INFLIGHT_WATCHDOG_US) {
    espNowSendInFlight = false;
    inflightWatchdogTrips++;
  }

  // Perda do RX: roda ANTES de qualquer return de envio.
  if (deckId != 0 && receiverReady && nowMs - lastReceiverReplyMillis > HANDSHAKE_TIMEOUT_MS) {
    Serial.println("Receptor perdido, reconectando...");
    startPairing();
  }

  if (deckId == 0) {
    uiRpm = 0.0f;
    delay(5);
    return;
  }

  // --- PAREAMENTO: varre canais enviando HELLO (comeca no ultimo canal) ---
  if (!receiverReady) {
    if (hopPending || nowMs - lastHopMillis >= PAIR_CHANNEL_DWELL_MS) {
      hopPending = false;
#if FORCE_FIXED_CHANNEL
      activeChannel = ESPNOW_CHANNEL;
      hopIndex = 0;
#else
      hopIndex = (int16_t)(((int32_t)hopIndex + 1) % (int32_t)pairChannelCount);
      activeChannel = pairChannels[hopIndex];
#endif
      esp_wifi_set_channel(activeChannel, WIFI_SECOND_CHAN_NONE);
      addReceiverPeer(activeChannel);
      lastHopMillis = nowMs;
      lastPairHelloMillis = 0;
      Serial.printf("PAIR: canal %u\n", (unsigned)activeChannel);
    }
    if (nowMs - lastPairHelloMillis >= PAIR_HELLO_INTERVAL_MS) {
      dvs_packet hello = {};
      hello.msgType = MSG_HELLO;
      hello.version = PROTOCOL_VERSION;
      hello.deckId = deckId;
      hello.seq = sequenceNumber;
      hello.timestampMicros = micros();
      if (radioSend(hello)) lastPairHelloMillis = nowMs;
    }
    return;
  }
  if (!wasReceiverReady) {
    wasReceiverReady = true;
    markDirty(DIRTY_CHAN);
    Serial.printf("PAIRED_ON_CH,%u\n", (unsigned)activeChannel);
  }

  serviceAcks();

  uint32_t now = micros();
  if (espNowSendInFlight) return;

  // --- TDMA: dispara dentro do slot deste deck, no relogio do RX ---
  if (rxClockIsFresh()) {
    uint32_t rxNow = estimatedRxMicrosNow();
    uint32_t frameIdx = rxNow / SEND_INTERVAL_US;
    uint32_t slotStartRx = frameIdx * SEND_INTERVAL_US + tdmaSlotOffsetUs();
    uint32_t slotEndRx = slotStartRx + TDMA_SLOT_WIDTH_US;
    bool insideSlot = ((int32_t)(rxNow - slotStartRx) >= 0) &&
                      ((int32_t)(rxNow - (slotEndRx - TDMA_SEND_GUARD_US)) < 0);
    if (insideSlot && tdmaLastSentFrame != frameIdx) {
      nextSendMicros = now;
    } else if ((int32_t)(rxNow - slotStartRx) < 0) {
      nextSendMicros = rxTimeToLocal(slotStartRx);
    } else {
      nextSendMicros = rxTimeToLocal(slotStartRx + SEND_INTERVAL_US);
    }
  } else {
    if ((int32_t)(now - nextSendMicros) < 0) return;
    nextSendMicros += SEND_INTERVAL_US;
    if ((int32_t)(now - nextSendMicros) > (int32_t)SEND_INTERVAL_US) nextSendMicros = now + SEND_INTERVAL_US;
  }
  if ((int32_t)(now - nextSendMicros) < 0) return;

  float dps = 0.0f;
  if (!readGyroDps(&dps)) return;
  autoTrimGyroOffset(dps);

  float rpm = dpsToRPM(dps);
  float delta = rpm - filteredRPM;
  float alpha = (fabsf(delta) > FAST_THRESHOLD_RPM) ? ALPHA_FAST : ALPHA_SLOW;
  filteredRPM += delta * alpha;
  if (fabsf(filteredRPM) < DEADZONE_RPM) filteredRPM = 0.0f;

  uiRpm = filteredRPM;
  uiDps = dps;

  packet.msgType = MSG_DATA;
  packet.version = PROTOCOL_VERSION;
  packet.deckId = deckId;
  packet.rpmCenti = (int16_t)constrain(lroundf(filteredRPM * 100.0f), -32768, 32767);
  packet.gyroRaw = (int16_t)constrain(lroundf(dps * 10.0f), -32768, 32767);
  packet.batteryPct = batteryLevelPct;
  packet.seq = sequenceNumber++;
  packet.timestampMicros = now;

  if (radioSend(packet)) {
    txSentCount++;
    if (rxClockSynced) tdmaLastSentFrame = estimatedRxMicrosNow() / SEND_INTERVAL_US;
  }
}

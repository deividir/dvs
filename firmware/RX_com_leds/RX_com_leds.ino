/*
========================================================
 ESP32-S3 DVS RECEIVER - DUAL CV02 I2S DAC + LEDs
 Saidas: 
 - Deck A (ID 1): LED Azul no Pino 4
 - Deck B (ID 2): LED Azul no Pino 5
 LED do deck: solido = link ok, piscando rapido = sinal fraco/perda, apagado = sem sinal
========================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/i2s.h>
#include <math.h>

// --- PINOS DOS LEDs ---
#define LED_DECK_A 4
#define LED_DECK_B 5
#define LED_CALIB 6

// --- ALERTA DE SINAL (LEDs dos decks) ---
// LED solido = link ok; piscando rapido = sinal fraco (RSSI) ou perda de pacotes
#define RSSI_WARN_THRESHOLD_DBM -75
#define LOST_PACKET_WARN_THRESHOLD 20
#define WARN_BLINK_MS 100
#define WARN_HOLD_MS 2000

#define DAC_A_BCK_PIN   2
#define DAC_A_LRCK_PIN  11
#define DAC_A_DATA_PIN  12

#define DAC_B_BCK_PIN   15
#define DAC_B_LRCK_PIN  14
#define DAC_B_DATA_PIN  13

#define ESPNOW_CHANNEL 11  // fallback: usado apenas se o scan de canais falhar
#define USE_LONG_RANGE 1  // 1 = modo long range (512/256 kbps, OBRIGATORIO ser igual ao TX), 0 = taxa normal
#define PROTOCOL_VERSION 2
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4
#define PING_INTERVAL_MS 500
#define DECK_TIMEOUT_MS 1500
#define RX_BOOT_ID 0x5A

#define DEBUG_SERIAL 1
#define DEBUG_BAUD 115200
#define DEBUG_PRINT_INTERVAL_MS 500

#define SAMPLE_RATE 44100
#define DMA_BUF_LEN 32
#define DMA_BUF_COUNT 4

#define BASE_RPM 33.333f
#define DEADZONE_RPM 0.015f
#define MAX_RPM_RATIO 3.0f
#define RPM_SMOOTHING 0.45f
#define OUTPUT_GAIN 0.70f
#define CALIB_THRESHOLD_RPM 1.0f
#define STOP_DEBOUNCE_MS 100
#define CALIB_STABLE_MS 200
#define POSITION_RESET_MS 15000
#define SIN_COS_TABLE_SIZE 1024
#define WRAP_GAP_MS 400

// Tom de teste de DAC (comando serial TEST_A / TEST_B pelo dashboard).
#define TEST_TONE_FREQ_HZ 1000
#define TEST_TONE_MS 10000
#define TEST_TONE_AMPLITUDE 0.25f

// TESTE DE ISOLAMENTO: 1 = compila APENAS o deck A (uma porta I2S, uma task).
// Usado para separar "interferencia entre as 2 portas I2S" de "deck A com defeito".
// Quando 1, o deck B nao e configurado nem o seu task criado.
#define SOLO_DECK_A 0

// Em modo SOLO, usa os pinos do deck B (13/14/15, que ja sabemos funcionar)
// para o deck A. Se o tom tocar nesses pinos, o problema sao os pinos 10/11/12.
#define SOLO_DECK_A_USA_PINOS_B 0

#define CV02_RESOLUTION 1000
#define CV02_BITS 20
#define CV02_SEED 0x59017UL
#define CV02_TAPS 0x361e4UL
#define CV02_LENGTH 712000UL
#define CV02_START_CYCLE 12000UL
#define CV02_START_PHASE ((uint64_t)CV02_START_CYCLE << 16)
#define CV02_PACKED_BYTES ((CV02_LENGTH + 7) / 8)

typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  int16_t rpmCenti;
  int16_t gyroRaw;
  uint8_t batteryPct;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

typedef struct {
  bool seen;
  int16_t rpmCenti;
  uint32_t lastSeq;
  uint32_t lastSeenMillis;
  uint32_t lastPingMillis;
  uint32_t packetCount;
  uint16_t lostPackets;
  uint8_t batteryPct;
  int8_t rssi;
  uint8_t mac[6];
} deck_state;

typedef struct {
  uint8_t deckId;
  i2s_port_t port;
  int bckPin;
  int lrckPin;
  int dataPin;
  const char *taskName;
  float filteredRpm;
  float calibOffset;
  uint64_t cv02Phase64;
  uint32_t wrapGapEndMillis;
  uint32_t stopCandidateStart;
  uint32_t calibStableStart;
  bool calibrating;
  uint32_t testToneUntil;
  uint16_t testToneFreqHz;
  uint32_t tonePhase;
} audio_deck_state;

deck_state deckStates[2];
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool hasPendingWelcome[2] = {false, false};
uint8_t pendingWelcomeMac[2][6];
uint8_t pendingWelcomeDeck[2] = {0, 0};

uint8_t cv02PackedBits[CV02_PACKED_BYTES];
uint32_t lastDebugPrintMillis = 0;
int16_t sinTable[SIN_COS_TABLE_SIZE];
int16_t cosTable[SIN_COS_TABLE_SIZE];

// Batimento cardiaco dos tasks de audio (1 incremento por buffer escrito com sucesso).
volatile uint32_t audioWriteCount[2] = {0, 0};
volatile esp_err_t lastI2sErr[2] = {ESP_OK, ESP_OK};
// Pico de amostra (magnitude maxima) do ultimo buffer escrito por cada task.
volatile uint32_t peakSample[2] = {0, 0};

// Canal ESP-NOW em uso (definido pelo scan no boot; ESPNOW_CHANNEL e o fallback).
uint8_t activeEspNowChannel = ESPNOW_CHANNEL;

void buildSinCosTables() {
  for (int i = 0; i < SIN_COS_TABLE_SIZE; i++) {
    float angle = (float)i * 6.28318530718f / (float)SIN_COS_TABLE_SIZE;
    sinTable[i] = (int16_t)(sinf(angle) * 32767.0f);
    cosTable[i] = (int16_t)(cosf(angle) * 32767.0f);
  }
}

audio_deck_state audioDecks[2] = {
#if SOLO_DECK_A && SOLO_DECK_A_USA_PINOS_B
  { 1, I2S_NUM_0, DAC_B_BCK_PIN, DAC_B_LRCK_PIN, DAC_B_DATA_PIN, "audioDeckA", 0.0f, 0.0f, CV02_START_PHASE, 0, 0, 0, false, 0, TEST_TONE_FREQ_HZ, 0 },
#else
  { 1, I2S_NUM_0, DAC_A_BCK_PIN, DAC_A_LRCK_PIN, DAC_A_DATA_PIN, "audioDeckA", 0.0f, 0.0f, CV02_START_PHASE, 0, 0, 0, false, 0, TEST_TONE_FREQ_HZ, 0 },
#endif
  { 2, I2S_NUM_1, DAC_B_BCK_PIN, DAC_B_LRCK_PIN, DAC_B_DATA_PIN, "audioDeckB", 0.0f, 0.0f, CV02_START_PHASE, 0, 0, 0, false, 0, TEST_TONE_FREQ_HZ, 0 }
};

// [Funções de suporte: lfsrBit, lfsrForward, setPackedBit, getPackedBit, buildCv02Bits mantidas aqui]
static uint8_t lfsrBit(uint32_t code, uint32_t taps) { uint32_t taken = code & taps; uint8_t parity = 0; while (taken != 0) { parity ^= (uint8_t)(taken & 0x1U); taken >>= 1; } return parity; }
static uint32_t lfsrForward(uint32_t current) { uint8_t bit = lfsrBit(current, CV02_TAPS | 0x1U); return (current >> 1) | ((uint32_t)bit << (CV02_BITS - 1)); }
static void setPackedBit(uint32_t index, uint8_t value) { uint32_t byteIndex = index >> 3; uint8_t bitMask = (uint8_t)(1U << (index & 7)); if (value != 0) cv02PackedBits[byteIndex] |= bitMask; else cv02PackedBits[byteIndex] &= (uint8_t)~bitMask; }
static inline uint8_t getPackedBit(uint32_t index) { uint32_t byteIndex = index >> 3; uint8_t bitMask = (uint8_t)(1U << (index & 7)); return (cv02PackedBits[byteIndex] & bitMask) != 0; }
void buildCv02Bits() { memset(cv02PackedBits, 0, sizeof(cv02PackedBits)); uint32_t code = CV02_SEED; for (uint32_t i = 0; i < CV02_LENGTH; i++) { setPackedBit(i, (uint8_t)(code & 0x1U)); code = lfsrForward(code); } }

bool ensurePeer(const uint8_t *mac) { if (esp_now_is_peer_exist(mac)) return true; esp_now_peer_info_t peerInfo = {}; memcpy(peerInfo.peer_addr, mac, 6); peerInfo.channel = activeEspNowChannel; peerInfo.encrypt = false; return esp_now_add_peer(&peerInfo) == ESP_OK; }
void sendControlMessage(const uint8_t *mac, uint8_t deckId, uint8_t msgType) { if (!ensurePeer(mac)) return; dvs_packet response = {}; response.msgType = msgType; response.version = PROTOCOL_VERSION; response.deckId = deckId; response.gyroRaw = (int16_t)RX_BOOT_ID; response.timestampMicros = micros(); esp_now_send(mac, (uint8_t *)&response, sizeof(response)); }

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) return;
  dvs_packet packet; memcpy(&packet, dataPtr, sizeof(packet));
  if (packet.version != PROTOCOL_VERSION || packet.deckId < 1 || packet.deckId > 2) return;
  const uint8_t *sourceMac = info->src_addr; uint8_t deckIndex = packet.deckId - 1; deck_state *state = &deckStates[deckIndex];
  int8_t rssi = (info->rx_ctrl != NULL) ? (int8_t)info->rx_ctrl->rssi : -127;
  portENTER_CRITICAL(&stateMux); memcpy(state->mac, sourceMac, 6); state->lastSeenMillis = millis(); state->rssi = rssi; state->batteryPct = packet.batteryPct; portEXIT_CRITICAL(&stateMux);
  if (packet.msgType == MSG_HELLO) { portENTER_CRITICAL(&stateMux); memcpy(pendingWelcomeMac[deckIndex], sourceMac, 6); pendingWelcomeDeck[deckIndex] = packet.deckId; hasPendingWelcome[deckIndex] = true; portEXIT_CRITICAL(&stateMux); return; }
  if (packet.msgType != MSG_DATA) return;
  uint16_t lost = 0; portENTER_CRITICAL(&stateMux); if (state->seen) { uint32_t seqDelta = packet.seq - state->lastSeq; if (seqDelta > 0) lost = (uint16_t)min(seqDelta - 1, 65535UL); } state->seen = true; state->rpmCenti = packet.rpmCenti; state->lastSeq = packet.seq; state->packetCount++; state->lostPackets = lost; portEXIT_CRITICAL(&stateMux);
}

// Escaneia as redes 2.4 GHz vizinhas e escolhe o canal com menos APs fortes.
// IMPORTANTE: rodar ANTES de ativar o modo Long Range — com WIFI_PROTOCOL_LR
// puro o radio nao decodifica beacons padrao. Empates favorecem 1/6/11.
void selectCleanChannel() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.println("SCAN: buscando canal 2.4GHz mais limpo...");
  int found = WiFi.scanNetworks();
  if (found <= 0) {
    Serial.println("SCAN: nenhuma rede encontrada, mantendo canal padrao");
    Serial.printf("RF_CHANNEL,%d\n", activeEspNowChannel);
    return;
  }
  float score[14] = {0};
  for (int i = 0; i < found; i++) {
    int ch = WiFi.channel(i);
    int rssi = WiFi.RSSI(i);
    if (ch < 1 || ch > 13) continue;
    // AP forte perto pesa muito mais que AP distante.
    float w = (rssi > -60) ? 4.0f : (rssi > -70) ? 2.0f : (rssi > -80) ? 1.0f : 0.3f;
    score[ch] += w;
  }
  static const uint8_t prefOrder[13] = {1, 6, 11, 2, 7, 12, 3, 8, 13, 4, 9, 5, 10};
  uint8_t best = ESPNOW_CHANNEL;
  float bestScore = 1e9f;
  for (int i = 0; i < 13; i++) {
    uint8_t ch = prefOrder[i];
    if (score[ch] < bestScore) { bestScore = score[ch]; best = ch; }
  }
  activeEspNowChannel = best;
  Serial.printf("RF_CHANNEL,%d\n", activeEspNowChannel);
}

void setupEspNow() { esp_wifi_set_max_tx_power(78);
#if USE_LONG_RANGE
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#else
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
#endif
  WiFi.setSleep(false);
  esp_wifi_set_channel(activeEspNowChannel, WIFI_SECOND_CHAN_NONE);
  esp_now_init(); esp_now_register_recv_cb(OnDataRecv); }
void setupI2S(audio_deck_state *deck, bool useApll) { i2s_config_t config = { .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = SAMPLE_RATE, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, .dma_buf_count = DMA_BUF_COUNT, .dma_buf_len = DMA_BUF_LEN, .use_apll = useApll, .tx_desc_auto_clear = true, .fixed_mclk = 0 }; i2s_pin_config_t pins = { .bck_io_num = deck->bckPin, .ws_io_num = deck->lrckPin, .data_out_num = deck->dataPin, .data_in_num = I2S_PIN_NO_CHANGE }; i2s_driver_install(deck->port, &config, 0, NULL); i2s_set_pin(deck->port, &pins); i2s_zero_dma_buffer(deck->port); }

static inline void renderCv02Sample(audio_deck_state *deck, float rpm, int16_t *leftOut, int16_t *rightOut) {
  if (millis() < deck->wrapGapEndMillis) { *leftOut = 0; *rightOut = 0; return; }
  float ratio = rpm / BASE_RPM; if(fabsf(ratio) < DEADZONE_RPM) ratio = 0.0f; if(ratio > MAX_RPM_RATIO) ratio = MAX_RPM_RATIO; if(ratio < -MAX_RPM_RATIO) ratio = -MAX_RPM_RATIO;
  int32_t phaseStep = (int32_t)(ratio * ((float)CV02_RESOLUTION * 65536.0f / SAMPLE_RATE));
  deck->cv02Phase64 += (int64_t)phaseStep; int64_t cycle = (int64_t)(deck->cv02Phase64 >> 16);
  if (cycle < 0) { deck->cv02Phase64 = (uint64_t)CV02_LENGTH << 16; deck->wrapGapEndMillis = millis() + WRAP_GAP_MS; *leftOut = 0; *rightOut = 0; return; }
  if (cycle >= (int64_t)CV02_LENGTH) { deck->cv02Phase64 = CV02_START_PHASE; deck->wrapGapEndMillis = millis() + WRAP_GAP_MS; *leftOut = 0; *rightOut = 0; return; }
  uint32_t cycleIndex = (uint32_t)cycle;
  uint16_t tableIndex = (deck->cv02Phase64 >> 6) & (SIN_COS_TABLE_SIZE - 1);
  int16_t sine = sinTable[tableIndex]; int16_t cosine = cosTable[tableIndex];
  uint8_t bit = getPackedBit(cycleIndex); float modulation = bit ? 1.0f : 1.0f - ((-(float)cosine / 32767.0f + 1.0f) * 0.25f);
  *leftOut = (int16_t)((-(float)cosine / 32767.0f) * modulation * OUTPUT_GAIN * 32767.0f); *rightOut = (int16_t)(((float)sine / 32767.0f) * modulation * OUTPUT_GAIN * 32767.0f);
}

float readTargetRpm(uint8_t deckId) { uint8_t deckIndex = deckId <= 1 ? 0 : 1; int16_t rpmCenti = 0; uint32_t lastSeenMillis = 0; portENTER_CRITICAL(&stateMux); rpmCenti = deckStates[deckIndex].rpmCenti; lastSeenMillis = deckStates[deckIndex].lastSeenMillis; portEXIT_CRITICAL(&stateMux); if (lastSeenMillis == 0 || millis() - lastSeenMillis > DECK_TIMEOUT_MS) return 0.0f; return (float)rpmCenti / 100.0f; }
void audioTask(void *param) { audio_deck_state *deck = (audio_deck_state *)param; int16_t buffer[DMA_BUF_LEN * 2]; while (true) {
    float targetRpm = readTargetRpm(deck->deckId);
    bool below = (fabsf(targetRpm) < CALIB_THRESHOLD_RPM);
    bool stopped;
    if (below) {
      if (deck->stopCandidateStart == 0) deck->stopCandidateStart = millis();
      stopped = (millis() - deck->stopCandidateStart >= STOP_DEBOUNCE_MS);
    } else {
      deck->stopCandidateStart = 0;
      stopped = false;
    }
    if (stopped) {
      deck->calibrating = true;
      if (deck->calibStableStart == 0) deck->calibStableStart = millis();
      if (millis() - deck->calibStableStart >= POSITION_RESET_MS) { deck->cv02Phase64 = CV02_START_PHASE; }
      deck->filteredRpm = 0.0f;
    } else {
      deck->calibStableStart = 0;
      deck->calibrating = false;
      float delta = targetRpm - deck->filteredRpm;
      float alpha = (fabsf(delta) > 10.0f) ? 0.8f : RPM_SMOOTHING;
      deck->filteredRpm += delta * alpha;
    }
    for (int i = 0; i < DMA_BUF_LEN; i++) { int16_t left, right; if (millis() < deck->testToneUntil) { deck->tonePhase += (uint32_t)((float)deck->testToneFreqHz * 65536.0f / SAMPLE_RATE); int16_t tone = (int16_t)((float)sinTable[(deck->tonePhase >> 6) & (SIN_COS_TABLE_SIZE - 1)] * TEST_TONE_AMPLITUDE); left = tone; right = tone; } else if (stopped) { left = 0; right = 0; } else { renderCv02Sample(deck, deck->filteredRpm, &left, &right); } buffer[i * 2] = left; buffer[i * 2 + 1] = right; }
    size_t written;
    esp_err_t i2sErr = i2s_write(deck->port, buffer, sizeof(buffer), &written, portMAX_DELAY);
    if (i2sErr == ESP_OK) {
      audioWriteCount[deck->deckId - 1]++;
      int16_t peak = 0;
      for (int i = 0; i < DMA_BUF_LEN * 2; i++) { int16_t v = buffer[i]; int16_t av = (v < 0) ? (int16_t)(-v) : v; if (av > peak) peak = av; }
      peakSample[deck->deckId - 1] = (uint32_t)peak;
    } else if (i2sErr != lastI2sErr[deck->deckId - 1]) {
      lastI2sErr[deck->deckId - 1] = i2sErr;
      Serial.printf("I2S_WRITE_ERR deck=%d err=0x%x\n", deck->deckId, (unsigned)i2sErr);
    }
  } }

void serviceEspNowControl() {
  for (uint8_t i = 0; i < 2; i++) { uint8_t welcomeMacCopy[6]; uint8_t welcomeDeckCopy = 0; bool shouldSendWelcome = false; portENTER_CRITICAL(&stateMux); if (hasPendingWelcome[i]) { memcpy(welcomeMacCopy, pendingWelcomeMac[i], 6); welcomeDeckCopy = pendingWelcomeDeck[i]; hasPendingWelcome[i] = false; shouldSendWelcome = true; } portEXIT_CRITICAL(&stateMux); if (shouldSendWelcome) sendControlMessage(welcomeMacCopy, welcomeDeckCopy, MSG_WELCOME); }
  uint32_t nowMillis = millis(); for (uint8_t i = 0; i < 2; i++) { uint8_t macCopy[6]; bool shouldPing = false; portENTER_CRITICAL(&stateMux); deck_state *state = &deckStates[i]; if (state->lastSeenMillis != 0 && nowMillis - state->lastSeenMillis <= DECK_TIMEOUT_MS && nowMillis - state->lastPingMillis >= PING_INTERVAL_MS) { memcpy(macCopy, state->mac, 6); state->lastPingMillis = nowMillis; shouldPing = true; } portEXIT_CRITICAL(&stateMux); if (shouldPing) sendControlMessage(macCopy, i + 1, MSG_PING); }
}

void handleSerialCommand() {
  static String cmdLine = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      cmdLine.trim();
      if (cmdLine.startsWith("TEST_A")) {
        audioDecks[0].testToneFreqHz = TEST_TONE_FREQ_HZ;
        audioDecks[0].testToneUntil = millis() + TEST_TONE_MS;
        Serial.println("TEST_A ON");
      } else if (cmdLine.startsWith("TEST_B")) {
        audioDecks[1].testToneFreqHz = TEST_TONE_FREQ_HZ;
        audioDecks[1].testToneUntil = millis() + TEST_TONE_MS;
        Serial.println("TEST_B ON");
      } else if (cmdLine.startsWith("TEST_OFF")) {
        audioDecks[0].testToneUntil = 0;
        audioDecks[1].testToneUntil = 0;
        Serial.println("TEST_OFF");
      }
      cmdLine = "";
    } else if (c != '\r') {
      cmdLine += c;
    }
  }
}

void updateDeckLed(uint8_t deckIndex, uint8_t pin) {
  static uint32_t warnHoldUntil[2] = {0, 0};
  bool seen = false;
  uint32_t lastSeenMillis = 0;
  int8_t rssi = 0;
  uint16_t lost = 0;
  portENTER_CRITICAL(&stateMux);
  seen = deckStates[deckIndex].seen;
  lastSeenMillis = deckStates[deckIndex].lastSeenMillis;
  rssi = deckStates[deckIndex].rssi;
  lost = deckStates[deckIndex].lostPackets;
  portEXIT_CRITICAL(&stateMux);

  uint32_t now = millis();
  bool alive = (seen && lastSeenMillis != 0 && (now - lastSeenMillis <= DECK_TIMEOUT_MS));
  if (!alive) {
    digitalWrite(pin, LOW);
    return;
  }

  bool warn = (rssi < RSSI_WARN_THRESHOLD_DBM) || (lost > LOST_PACKET_WARN_THRESHOLD);
  if (warn) warnHoldUntil[deckIndex] = now + WARN_HOLD_MS;
  bool blinking = (now < warnHoldUntil[deckIndex]);
  digitalWrite(pin, blinking ? ((now / WARN_BLINK_MS) % 2) : HIGH);
}

void sendTelemetry() {
  uint32_t now = millis();
  if (now - lastDebugPrintMillis < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintMillis = now;
  deck_state local[2];
  portENTER_CRITICAL(&stateMux); memcpy(local, deckStates, sizeof(local)); portEXIT_CRITICAL(&stateMux);
  static uint32_t prevWriteCount[2] = {0, 0};
  uint32_t wrDelta[2];
  for (uint8_t i = 0; i < 2; i++) {
    wrDelta[i] = audioWriteCount[i] - prevWriteCount[i];
    prevWriteCount[i] = audioWriteCount[i];
  }
  Serial.printf("TELEM,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%lu,%lu\n",
    (unsigned long)now,
    (local[0].lastSeenMillis != 0 && now - local[0].lastSeenMillis <= DECK_TIMEOUT_MS) ? 1 : 0,
    (int)local[0].rssi, (int)local[0].batteryPct, (int)local[0].rpmCenti, (int)local[0].lostPackets,
    (local[1].lastSeenMillis != 0 && now - local[1].lastSeenMillis <= DECK_TIMEOUT_MS) ? 1 : 0,
    (int)local[1].rssi, (int)local[1].batteryPct, (int)local[1].rpmCenti, (int)local[1].lostPackets,
    (unsigned long)((audioDecks[0].cv02Phase64 >> 16) / 1000ULL),
    (unsigned long)((audioDecks[1].cv02Phase64 >> 16) / 1000ULL),
    (local[0].seen ? (unsigned long)(now - local[0].lastSeenMillis) : 0UL),
    (local[1].seen ? (unsigned long)(now - local[1].lastSeenMillis) : 0UL),
    (unsigned long)wrDelta[0], (unsigned long)wrDelta[1],
    (audioDecks[0].testToneUntil != 0 && now < audioDecks[0].testToneUntil) ? 1 : 0,
    (audioDecks[1].testToneUntil != 0 && now < audioDecks[1].testToneUntil) ? 1 : 0,
    (int)lastI2sErr[0], (int)lastI2sErr[1],
    (unsigned long)peakSample[0], (unsigned long)peakSample[1]);
}

void setup() {
  pinMode(LED_DECK_A, OUTPUT);
  pinMode(LED_DECK_B, OUTPUT);
  pinMode(LED_CALIB, OUTPUT);
  Serial.begin(DEBUG_BAUD);
  Serial.println("DVS_RX_READY,v2");
  buildSinCosTables();
  buildCv02Bits();
  setupI2S(&audioDecks[0], false);
#if !SOLO_DECK_A
  setupI2S(&audioDecks[1], false);
  // FIX ESP32-S3: instalar a segunda porta I2S corrompe o clock da primeira
  // (porta instalada primeiro saia com ruido). Reinstala a primeira apos a segunda.
  i2s_driver_uninstall(audioDecks[0].port);
  setupI2S(&audioDecks[0], false);
#endif
  selectCleanChannel();
  setupEspNow();
  // Reaplica o clock I2S nas duas portas apos o WiFi iniciar (PLL compartilhado).
  // IMPORTANTE: setar a porta 1 por ultimo corrompe o clock da porta 0 no ESP32-S3
  // com o driver legado. Ordem invertida: porta 0 (deck A) e a ultima a ser setada.
#if SOLO_DECK_A
  i2s_set_clk(audioDecks[0].port, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  i2s_zero_dma_buffer(audioDecks[0].port);
#else
  for (int i = 1; i >= 0; i--) {
    i2s_set_clk(audioDecks[i].port, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
    i2s_zero_dma_buffer(audioDecks[i].port);
  }
#endif
  xTaskCreatePinnedToCore(audioTask, audioDecks[0].taskName, 8192, &audioDecks[0], 3, NULL, 1);
#if !SOLO_DECK_A
  xTaskCreatePinnedToCore(audioTask, audioDecks[1].taskName, 8192, &audioDecks[1], 3, NULL, 1);
#endif
}

void loop() {
  handleSerialCommand();
  serviceEspNowControl();
  sendTelemetry();

  // Controle dos LEDs Azul (solido = ok, piscando = sinal fraco/perda, apagado = sem deck)
  updateDeckLed(0, LED_DECK_A);
  updateDeckLed(1, LED_DECK_B);
  uint32_t now = millis();
  bool anyCalibrating = audioDecks[0].calibrating || audioDecks[1].calibrating;
  digitalWrite(LED_CALIB, anyCalibrating ? ((now / 150) % 2) : LOW);
  
  delay(10);
}

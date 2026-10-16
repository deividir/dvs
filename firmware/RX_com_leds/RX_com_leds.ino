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
#include <driver/ledc.h>
#include <math.h>
#include <Preferences.h>

// --- PINOS DOS LEDs ---
#define LED_DECK_A 4
#define LED_DECK_B 5
#define LED_CALIB 6

// --- ALERTA DE SINAL (LEDs dos decks) ---
// LED solido = link ok; piscando rapido = sinal fraco (RSSI) ou perda de pacotes
#define RSSI_WARN_THRESHOLD_DBM -75
#define LOST_PACKET_WARN_THRESHOLD 50
#define WARN_BLINK_MS 100
#define WARN_HOLD_MS 500

#define DAC_A_BCK_PIN   2
#define DAC_A_LRCK_PIN  11
#define DAC_A_DATA_PIN  12

#define DAC_B_BCK_PIN   15
#define DAC_B_LRCK_PIN  14
#define DAC_B_DATA_PIN  13

#define ESPNOW_CHANNEL 11  // canal fixo de teste
#define FORCE_FIXED_CHANNEL 1  // 1 = ignora scan/NVS durante o teste; 0 = usa a logica original
#define USE_LONG_RANGE 0  // modo normal: menor tempo no ar e melhor para alta taxa de pacotes
// OBS: long range usa WIFI_PROTOCOL_LR (1Mbps, amplia sensibilidade/alcsencia);
// o custo e mais trafego de ar por pacote. Funciona apenas se TODOS os modulos
// (RX + os 2 TX) estiverem com o mesmo valor e reflashados.
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
#define CFG_TX_POWER 5
#define PING_INTERVAL_MS 500
#define DECK_TIMEOUT_MS 2500
#define RX_BOOT_ID 0x5A

#define DEBUG_SERIAL 1
#define DEBUG_BAUD 115200
#define DEBUG_PRINT_INTERVAL_MS 500

#define SAMPLE_RATE 44100
#define DMA_BUF_LEN 32
#define DMA_BUF_COUNT 6
// Gap Maximo de escritas I2S sem underrun obrigatorio do DAC:
// = capacidade total do ring DMA (DMA_BUF_COUNT x DMA_BUF_LEN frames).
#define UNDERFLOW_GAP_US ((uint32_t)((uint32_t)DMA_BUF_COUNT * DMA_BUF_LEN * 1000000UL / SAMPLE_RATE))
#define DMA_TOTAL_FRAMES ((uint32_t)DMA_BUF_COUNT * DMA_BUF_LEN)

#define BASE_RPM 33.333f
#define DEADZONE_RPM 0.015f
#define MAX_RPM_RATIO 3.0f
#define RPM_SMOOTHING 0.6f
#define OUTPUT_GAIN 0.70f
#define CALIB_THRESHOLD_RPM 1.0f
#define STOP_DEBOUNCE_MS 100
#define SIN_COS_TABLE_SIZE 1024

// Fade de inicio (anti-"pio"): apos o toca-discos comecar a girar, o volume do
// timecode sobe de 0 a 100% em START_FADE_MS. Configuravel via comando FADE
// (0 desliga) e persistido em NVS. Custo: 1 millis() + 1 mul/somente no fade.
#define START_FADE_MS_DEFAULT 200
uint16_t gStartFadeMs = START_FADE_MS_DEFAULT;

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
// 0 = sem lead-in: a agulha no ponto inicial le o timecode no ciclo 0,
// fazendo a musica comecar exatamente onde o stylus esta (sem deslocamento).
#define CV02_START_CYCLE 0UL
#define CV02_START_PHASE ((uint64_t)CV02_START_CYCLE << 16)
#define CV02_PACKED_BYTES ((CV02_LENGTH + 7) / 8)

// --- Timecode Rekordbox ---
// Mesmo esquema do CV02 (1 bit por ciclo de portadora de 1000 Hz, L/R em
// quadratura), porem com modulacao de 2 niveis CONSTANTES (bit1 = 1.0,
// bit0 = 0.75) e sequencia pseudo-aleatoria (m-sequence) de grau 20 propria.
// Diferente do CV02, NAO usa tabela de bits em RAM: a sequencia e gerada sob
// demanda por um LFSR (sem custo de memoria). RB_TAPS ja esta na convencao de
// lfsrBit/lfsrForward (bits revertidos em relacao a mascara do matlab/BM).
#define RB_RESOLUTION 1000
#define RB_BITS 20
#define RB_SEED 0x8D02UL
#define RB_TAPS 0x2E89BUL
#define RB_LEVEL0 0.75f
#define RB_LEVEL1 1.0f
#define RB_MASK ((uint32_t)((1UL << RB_BITS) - 1UL))

// --- Timecode Traktor MK2 ---
// Esquema DIFERENTE do Serato/Rekordbox. Reverso-engenheirado do arquivo
// "Timecode Traktor MK2 (Playback).mp3". Estrutura:
//   - Portadora de 3000 Hz, L/R em quadratura (L = -cos, R = +sin), amplitude fixa.
//   - Sinal de dados PAM a 1200 simbolos/s SOMADO (offset) as duas saidas.
//   - Sequencia = m-sequence de grau 22 (LFSR de 22 bits, taps {10,14,15,22}).
//   - Pulso de conformacao medido (raised-cosine com cauda negativa ~1.5T).
// Verificado: demodulacao do arquivo da grau 22 com predicao held-out 100%.
#define TR_RESOLUTION 1200u
#define TR_CARRIER_HZ 3000.0f
#define TR_BITS 22
#define TR_MASK ((uint32_t)((1UL << TR_BITS) - 1UL))
// Estado inicial do LFSR (qualquer estado nao-nulo da m-sequence; define a
// posicao absoluta lida pelo Traktor no inicio).
#define TR_SEED 0x3F049CUL
// Niveis em fracao de fundo de escala (0..1), antes de OUTPUT_GAIN.
#define TR_CARRIER_LEVEL 0.45f
#define TR_OFFSET_LEVEL 0.244f
// Numero de simbolos somados para cada lado na convolucao do pulso.
#define TR_J 3
#define TR_WIN (2 * TR_J + 1)
// Pulso medido (149 amostras, tau de -74 a +74, pico no indice 74 = 10000).
#define TR_PULSE_LEN 149
#define TR_PULSE_CENTER 74
static const int16_t trPulse[TR_PULSE_LEN] = {
  -215, -245, -270, -289, -302, -306, -302, -290, -270, -241, -206, -164,
  -117, -67, -14, 38, 88, 134, 174, 207, 231, 245, 246, 236, 213, 178, 131,
  73, 7, -67, -146, -228, -308, -385, -454, -512, -556, -582, -587, -567,
  -521, -445, -337, -196, -21, 189, 434, 712, 1023, 1366, 1738, 2136, 2557,
  2999, 3456, 3926, 4404, 4886, 5368, 5844, 6312, 6767, 7204, 7621, 8013,
  8377, 8710, 9010, 9273, 9498, 9684, 9827, 9929, 9986, 10000, 9970, 9895,
  9778, 9618, 9416, 9175, 8895, 8579, 8230, 7850, 7443, 7011, 6558, 6089,
  5606, 5115, 4619, 4123, 3631, 3148, 2677, 2223, 1789, 1379, 996, 643, 321,
  33, -221, -439, -622, -770, -884, -966, -1017, -1041, -1039, -1015, -973,
  -915, -846, -769, -688, -605, -524, -448, -378, -318, -268, -230, -204,
  -191, -190, -201, -223, -254, -292, -337, -386, -438, -490, -540, -587,
  -629, -665, -693, -714, -727, -731, -726, -714, -695, -670, -639
};

// Modo do timecode gerado nas saidas de audio: 0 = Serato (CV02),
// 1 = Rekordbox, 2 = Traktor MK2.
// Persistido em NVS (key "tcMode") e alteravel pelo comando serial TIMECODE.
#define TC_MODE_SERATO 0
#define TC_MODE_REKORDBOX 1
#define TC_MODE_TRAKTOR 2
uint8_t timecodeMode = TC_MODE_SERATO;

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
  uint32_t firstSeenMillis;
  uint32_t lastPingMillis;
  uint32_t packetCount;
  uint32_t lostPackets;
  uint16_t lastGap;
  uint32_t winReceived;
  uint32_t winMissed;
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
  uint32_t stopCandidateStart;
  uint32_t calibStableStart;
  bool calibrating;
  uint32_t testToneUntil;
  uint16_t testToneFreqHz;
  uint32_t tonePhase;
  float volume;
  uint32_t fadeStartMillis;
  uint32_t rbState;
  int64_t rbCycle;
  uint32_t trState;
  int64_t trSymCycle;
  int64_t trSymPhase64;
  int64_t trCarPhase64;
  uint8_t trWin[TR_WIN];
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

// Brilho dos LEDs de deck (0-100%, PWM via ledc). Default 100 = comportamento atual.
uint8_t ledBrightness = 100;
// true quando o pino foi configurado com sucesso via ledcAttach (PWM de brilho).
bool ledPwmOk[2] = {false, false};

// Batimento cardiaco dos tasks de audio (1 incremento por buffer escrito com sucesso).
volatile uint32_t audioWriteCount[2] = {0, 0};
volatile esp_err_t lastI2sErr[2] = {ESP_OK, ESP_OK};
// Pico de amostra (magnitude maxima) do ultimo buffer escrito por cada task.
volatile uint32_t peakSample[2] = {0, 0};
// Monitor de underrun do DAC: contagem cumulativa quando o gap entre escritas
// I2S supera a capacidade total do ring DMA, e o maior gap desde o ultimo reporte.
volatile uint32_t underrunCount[2] = {0, 0};
volatile uint32_t underrunMaxGapUs[2] = {0, 0};
// Uso do buffer DMA (% dos frames enfileirados) desde o ultimo reporte:
// somatorio de pct + contagem (para a media) e pct minimo (pior caso / folga).
volatile uint32_t buffSumPct[2] = {0, 0};
volatile uint32_t buffCount[2] = {0, 0};
volatile uint32_t buffMinPct[2] = {100, 100};

// Canal ESP-NOW em uso (definido pelo scan no boot; ESPNOW_CHANNEL e o fallback).
uint8_t activeEspNowChannel = ESPNOW_CHANNEL;
uint8_t txPowerQdbm[2] = {52, 52};

void buildSinCosTables() {
  for (int i = 0; i < SIN_COS_TABLE_SIZE; i++) {
    float angle = (float)i * 6.28318530718f / (float)SIN_COS_TABLE_SIZE;
    sinTable[i] = (int16_t)(sinf(angle) * 32767.0f);
    cosTable[i] = (int16_t)(cosf(angle) * 32767.0f);
  }
}

audio_deck_state audioDecks[2] = {
#if SOLO_DECK_A && SOLO_DECK_A_USA_PINOS_B
  { 1, I2S_NUM_0, DAC_B_BCK_PIN, DAC_B_LRCK_PIN, DAC_B_DATA_PIN, "audioDeckA", 0.0f, 0.0f, CV02_START_PHASE, 0, 0, false, 0, TEST_TONE_FREQ_HZ, 0, 1.0f, 0, RB_SEED, 0, TR_SEED, 0, 0, 0 },
#else
  { 1, I2S_NUM_0, DAC_A_BCK_PIN, DAC_A_LRCK_PIN, DAC_A_DATA_PIN, "audioDeckA", 0.0f, 0.0f, CV02_START_PHASE, 0, 0, false, 0, TEST_TONE_FREQ_HZ, 0, 1.0f, 0, RB_SEED, 0, TR_SEED, 0, 0, 0 },
#endif
  { 2, I2S_NUM_1, DAC_B_BCK_PIN, DAC_B_LRCK_PIN, DAC_B_DATA_PIN, "audioDeckB", 0.0f, 0.0f, CV02_START_PHASE, 0, 0, false, 0, TEST_TONE_FREQ_HZ, 0, 1.0f, 0, RB_SEED, 0, TR_SEED, 0, 0, 0 }
};

// [Funções de suporte: lfsrBit, lfsrForward, setPackedBit, getPackedBit, buildCv02Bits mantidas aqui]
static uint8_t lfsrBit(uint32_t code, uint32_t taps) { uint32_t taken = code & taps; uint8_t parity = 0; while (taken != 0) { parity ^= (uint8_t)(taken & 0x1U); taken >>= 1; } return parity; }
static uint32_t lfsrForward(uint32_t current) { uint8_t bit = lfsrBit(current, CV02_TAPS | 0x1U); return (current >> 1) | ((uint32_t)bit << (CV02_BITS - 1)); }
static void setPackedBit(uint32_t index, uint8_t value) { uint32_t byteIndex = index >> 3; uint8_t bitMask = (uint8_t)(1U << (index & 7)); if (value != 0) cv02PackedBits[byteIndex] |= bitMask; else cv02PackedBits[byteIndex] &= (uint8_t)~bitMask; }
static inline uint8_t getPackedBit(uint32_t index) { uint32_t byteIndex = index >> 3; uint8_t bitMask = (uint8_t)(1U << (index & 7)); return (cv02PackedBits[byteIndex] & bitMask) != 0; }
void buildCv02Bits() { memset(cv02PackedBits, 0, sizeof(cv02PackedBits)); uint32_t code = CV02_SEED; for (uint32_t i = 0; i < CV02_LENGTH; i++) { setPackedBit(i, (uint8_t)(code & 0x1U)); code = lfsrForward(code); } }

// LFSR do rekordbox: mesma estrutura de lfsrForward (shift right + realimenta no
// MSB), porem com os taps do rekordbox. rbReverse devolve o estado anterior
// (permite acompanhar a agulha girando ao contrario sem tabela em RAM).
static uint32_t lfsrForwardT(uint32_t current, uint32_t taps) { uint8_t bit = lfsrBit(current, taps | 0x1U); return (current >> 1) | ((uint32_t)bit << (RB_BITS - 1)); }
static uint32_t lfsrReverseT(uint32_t next, uint32_t taps) { uint32_t x = (next << 1) & RB_MASK; uint8_t b0 = lfsrBit(x, taps | 0x1U) ^ (uint8_t)((next >> (RB_BITS - 1)) & 0x1U); return x | b0; }

// LFSR do Traktor MK2: 22 bits, taps {10,14,15,22}. O bit transmitido de um
// estado S e o MSB (S>>21). trForward avanca um simbolo; trReverse devolve o
// estado anterior (permite acompanhar a agulha girando ao contrario).
static inline uint32_t trForward(uint32_t s) {
  uint32_t fb = ((s >> 9) ^ (s >> 13) ^ (s >> 14) ^ (s >> 21)) & 0x1U;
  return ((s << 1) | fb) & TR_MASK;
}
static inline uint32_t trReverse(uint32_t s) {
  uint32_t msb = (s ^ (s >> 10) ^ (s >> 14) ^ (s >> 15)) & 0x1U;
  return (s >> 1) | (msb << (TR_BITS - 1));
}

// Preenche a janela deslizante com os simbolos trSymCycle-3 .. trSymCycle+3
// (trWin[TR_J] = simbolo corrente). Sem tabela em RAM.
static void trBuildWindow(audio_deck_state *deck) {
  uint32_t t = deck->trState;
  deck->trWin[TR_J] = (uint8_t)((t >> (TR_BITS - 1)) & 0x1U);
  for (int i = TR_J - 1; i >= 0; i--) { t = trReverse(t); deck->trWin[i] = (uint8_t)((t >> (TR_BITS - 1)) & 0x1U); }
  t = deck->trState;
  for (int i = TR_J + 1; i < TR_WIN; i++) { t = trForward(t); deck->trWin[i] = (uint8_t)((t >> (TR_BITS - 1)) & 0x1U); }
}

static void trInitDeck(audio_deck_state *deck) {
  deck->trState = TR_SEED;
  deck->trSymCycle = 0;
  deck->trSymPhase64 = 0;
  deck->trCarPhase64 = 0;
  trBuildWindow(deck);
}

bool ensurePeer(const uint8_t *mac) { if (esp_now_is_peer_exist(mac)) return true; esp_now_peer_info_t peerInfo = {}; memcpy(peerInfo.peer_addr, mac, 6); peerInfo.channel = activeEspNowChannel; peerInfo.encrypt = false; return esp_now_add_peer(&peerInfo) == ESP_OK; }
void sendControlMessage(const uint8_t *mac, uint8_t deckId, uint8_t msgType) { if (!ensurePeer(mac)) return; dvs_packet response = {}; response.msgType = msgType; response.version = PROTOCOL_VERSION; response.deckId = deckId; response.gyroRaw = (int16_t)RX_BOOT_ID; response.timestampMicros = micros(); esp_now_send(mac, (uint8_t *)&response, sizeof(response)); }

// Envia o novo RPM_MULTIPLIER de um deck para o TX correspondente via ESP-NOW.
// O valor vai no campo rpmCenti escalado por 10000 (ex.: 0.9990 -> 9990).
void sendCalibCommand(uint8_t deckId, float multiplier) {
  if (deckId < 1 || deckId > 2) return;
  uint8_t deckIndex = deckId - 1;
  uint8_t macCopy[6];
  portENTER_CRITICAL(&stateMux);
  if (deckStates[deckIndex].lastSeenMillis == 0) { portEXIT_CRITICAL(&stateMux); Serial.println("CALIB_ERR: deck sem sinal"); return; }
  memcpy(macCopy, deckStates[deckIndex].mac, 6);
  portEXIT_CRITICAL(&stateMux);
  if (!ensurePeer(macCopy)) return;
  dvs_packet calib = {};
  calib.msgType = MSG_CALIB;
  calib.version = PROTOCOL_VERSION;
  calib.deckId = deckId;
  calib.rpmCenti = (int16_t)lroundf(multiplier * 10000.0f);
  calib.timestampMicros = micros();
  // ESP-NOW e sem conexao: envia 3x para reduzir perda por colisao/ruido.
  for (int i = 0; i < 3; i++) {
    esp_now_send(macCopy, (uint8_t *)&calib, sizeof(calib));
    delay(20);
  }
  Serial.printf("CALIB_SENT deck=%d M=%.4f\n", deckId, multiplier);
}

// Envia um parametro de configuracao ao TX de um deck via ESP-NOW.
// O id vai no campo batteryPct; o valor (Q1000 p/ floats) em rpmCenti.
void sendParamCommand(uint8_t deckId, uint8_t paramId, int16_t value) {
  if (deckId < 1 || deckId > 2) return;
  uint8_t deckIndex = deckId - 1;
  uint8_t macCopy[6];
  portENTER_CRITICAL(&stateMux);
  if (deckStates[deckIndex].lastSeenMillis == 0) { portEXIT_CRITICAL(&stateMux); Serial.println("CFG_ERR: deck sem sinal"); return; }
  memcpy(macCopy, deckStates[deckIndex].mac, 6);
  portEXIT_CRITICAL(&stateMux);
  if (!ensurePeer(macCopy)) return;
  dvs_packet cfg = {};
  cfg.msgType = MSG_SET_PARAM;
  cfg.version = PROTOCOL_VERSION;
  cfg.deckId = deckId;
  cfg.batteryPct = paramId;
  cfg.rpmCenti = value;
  cfg.timestampMicros = micros();
  // ESP-NOW e sem conexao: envia 3x para reduzir perda por colisao/ruido.
  for (int i = 0; i < 3; i++) {
    esp_now_send(macCopy, (uint8_t *)&cfg, sizeof(cfg));
    delay(20);
  }
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) return;
  dvs_packet packet; memcpy(&packet, dataPtr, sizeof(packet));
  if (packet.version != PROTOCOL_VERSION || packet.deckId < 1 || packet.deckId > 2) return;
  if (packet.msgType == MSG_CFG_ACK) {
    const char *name = "?";
    switch (packet.batteryPct) {
      case CFG_DECK_ID: name = "deckId"; break;
      case CFG_ALPHA_SLOW: name = "alphaSlow"; break;
      case CFG_ALPHA_FAST: name = "alphaFast"; break;
      case CFG_FAST_THRESHOLD: name = "threshold"; break;
      case CFG_TX_POWER: name = "txPowerQdbm"; break;
    }
    if (packet.batteryPct == CFG_DECK_ID) Serial.printf("CFG_ACK,deck=%u,%s=%d\n", packet.deckId, name, packet.rpmCenti);
    else Serial.printf("CFG_ACK,deck=%u,%s=%.3f\n", packet.deckId, name, (float)packet.rpmCenti / 1000.0f);
    return;
  }
  const uint8_t *sourceMac = info->src_addr; uint8_t deckIndex = packet.deckId - 1; deck_state *state = &deckStates[deckIndex];
  int8_t rssi = (info->rx_ctrl != NULL) ? (int8_t)info->rx_ctrl->rssi : -127;
  portENTER_CRITICAL(&stateMux); memcpy(state->mac, sourceMac, 6); state->lastSeenMillis = millis(); state->rssi = rssi; state->batteryPct = packet.batteryPct; portEXIT_CRITICAL(&stateMux);
  if (packet.msgType == MSG_HELLO) {
    Serial.printf("HELLO_RX deck=%u ch=%u rssi=%d\n",
                  (unsigned)packet.deckId, (unsigned)activeEspNowChannel, (int)rssi);
    portENTER_CRITICAL(&stateMux); memcpy(pendingWelcomeMac[deckIndex], sourceMac, 6); pendingWelcomeDeck[deckIndex] = packet.deckId; hasPendingWelcome[deckIndex] = true; portEXIT_CRITICAL(&stateMux); return;
  }
  if (packet.msgType == MSG_CALIB_ACK) { Serial.printf("CALIB_ACK deck=%d M=%.4f\n", packet.deckId, (float)packet.rpmCenti / 10000.0f); return; }
  if (packet.msgType != MSG_DATA) return;
  uint16_t lost = 0; portENTER_CRITICAL(&stateMux); if (state->seen) { uint32_t seqDelta = packet.seq - state->lastSeq; if (seqDelta > 1) lost = (uint16_t)min(seqDelta - 1, 65535UL); } if (!state->seen) state->firstSeenMillis = millis(); state->seen = true; state->rpmCenti = packet.rpmCenti; state->lastSeq = packet.seq; state->packetCount++; state->lostPackets += lost; state->lastGap = lost; state->winReceived++; state->winMissed += lost; portEXIT_CRITICAL(&stateMux);
}

// Escaneia as redes 2.4 GHz vizinhas e escolhe o canal com menos APs fortes.
// Canal 2.4GHz persistido em NVS: escolha manual sobrevive a reboots.
int loadChannelNvs() {
  Preferences prefs;
  prefs.begin("dvs", true);
  int saved = prefs.getInt("chan", 0);
  prefs.end();
  if (saved >= 1 && saved <= 13) return saved;
  return 0;
}
void saveChannelNvs(uint8_t ch) {
  Preferences prefs;
  prefs.begin("dvs", false);
  prefs.putInt("chan", ch);
  prefs.end();
}

// Brilho dos LEDs de deck persistido em NVS: sobrevive a reboots.
int loadLedBrightnessNvs() {
  Preferences prefs;
  prefs.begin("dvs", true);
  int saved = prefs.getInt("ledBrt", -1);
  prefs.end();
  if (saved >= 0 && saved <= 100) return saved;
  return 100;
}
void saveLedBrightnessNvs(uint8_t b) {
  Preferences prefs;
  prefs.begin("dvs", false);
  prefs.putInt("ledBrt", (int)b);
  prefs.end();
}

// Fade de inicio (ms) persistido em NVS: sobrevive a reboots.
uint16_t loadStartFadeNvs() {
  Preferences prefs;
  prefs.begin("dvs", true);
  int saved = prefs.getInt("fadeMs", -1);
  prefs.end();
  if (saved >= 0 && saved <= 1000) return (uint16_t)saved;
  return START_FADE_MS_DEFAULT;
}
void saveStartFadeNvs(uint16_t ms) {
  Preferences prefs;
  prefs.begin("dvs", false);
  prefs.putInt("fadeMs", (int)ms);
  prefs.end();
}

// Modo do timecode (Serato/Rekordbox) persistido em NVS: sobrevive a reboots.
uint8_t loadTimecodeModeNvs() {
  Preferences prefs;
  prefs.begin("dvs", true);
  int saved = prefs.getInt("tcMode", -1);
  prefs.end();
  if (saved == TC_MODE_SERATO || saved == TC_MODE_REKORDBOX || saved == TC_MODE_TRAKTOR) return (uint8_t)saved;
  return TC_MODE_SERATO;
}
void saveTimecodeModeNvs(uint8_t m) {
  Preferences prefs;
  prefs.begin("dvs", false);
  prefs.putInt("tcMode", (int)m);
  prefs.end();
}

// Volume por deck (0.0-1.0) persistido em NVS: sobrevive a reboots.
float loadDeckVolumeNvs(int deckIndex) {
  Preferences prefs;
  prefs.begin("dvs", true);
  float saved = prefs.getFloat(deckIndex == 0 ? "volA" : "volB", -1.0f);
  prefs.end();
  if (saved >= 0.0f && saved <= 1.0f) return saved;
  return 1.0f;
}
void saveDeckVolumeNvs(int deckIndex, float v) {
  Preferences prefs;
  prefs.begin("dvs", false);
  prefs.putFloat(deckIndex == 0 ? "volA" : "volB", v);
  prefs.end();
}

void loadTxPowerNvs() {
  Preferences prefs;
  prefs.begin("dvs", true);
  for (int i = 0; i < 2; i++) {
    int saved = prefs.getInt(i == 0 ? "pwrA" : "pwrB", 52);
    if (saved == 40 || saved == 52 || saved == 64 || saved == 72 || saved == 80) txPowerQdbm[i] = (uint8_t)saved;
  }
  prefs.end();
}

void saveTxPowerNvs(int deckIndex) {
  if (deckIndex < 0 || deckIndex > 1) return;
  Preferences prefs;
  prefs.begin("dvs", false);
  prefs.putInt(deckIndex == 0 ? "pwrA" : "pwrB", (int)txPowerQdbm[deckIndex]);
  prefs.end();
}

// Cache dos filtros aplicados aos decks (por slot), persistido em NVS.
// Fonte de segurança: se o TX perder os valores (NVS do C3), o RX reaplica
// automaticamente quando o deck reconectar (ver serviceEspNowControl).
typedef struct { float slow; float fast; float thr; bool valid; } filter_cache_t;
static filter_cache_t gFilterCache[2];

void loadFilterCacheNvs() {
  Preferences prefs;
  prefs.begin("dvs", true);
  for (int i = 0; i < 2; i++) {
    const char *k = i == 0 ? "filA" : "filB";
    float s = prefs.getFloat((String(k) + "_slow").c_str(), -1.0f);
    float f = prefs.getFloat((String(k) + "_fast").c_str(), -1.0f);
    float t = prefs.getFloat((String(k) + "_thr").c_str(), -1.0f);
    if (s > 0.05f && s < 0.99f && f > 0.05f && f < 0.99f && t > 0.01f && t < 10.0f) {
      gFilterCache[i].slow = s; gFilterCache[i].fast = f; gFilterCache[i].thr = t; gFilterCache[i].valid = true;
    }
  }
  prefs.end();
}

void saveFilterCacheNvs(int i) {
  if (i < 0 || i > 1) return;
  Preferences prefs;
  prefs.begin("dvs", false);
  const char *k = i == 0 ? "filA" : "filB";
  prefs.putFloat((String(k) + "_slow").c_str(), gFilterCache[i].slow);
  prefs.putFloat((String(k) + "_fast").c_str(), gFilterCache[i].fast);
  prefs.putFloat((String(k) + "_thr").c_str(), gFilterCache[i].thr);
  prefs.end();
}

// Aplica o canal no radio e re-posiciona os peers conhecidos (usado pelo CHANNEL/SCAN).
void applyChannelToRadio() {
  esp_wifi_set_channel(activeEspNowChannel, WIFI_SECOND_CHAN_NONE);
  for (uint8_t d = 0; d < 2; d++) {
    uint8_t macCopy[6];
    portENTER_CRITICAL(&stateMux);
    memcpy(macCopy, deckStates[d].mac, 6);
    portEXIT_CRITICAL(&stateMux);
    if (esp_now_is_peer_exist(macCopy)) {
      esp_now_peer_info_t p = {};
      memcpy(p.peer_addr, macCopy, 6);
      p.channel = activeEspNowChannel;
      p.encrypt = false;
      esp_now_mod_peer(&p);
    }
  }
}

// Se ja houver canal salvo em NVS (escolha manual anterior), usa ele direto.
// force=true = re-scannea mesmo assim (comando SCAN) e salva o resultado.
// IMPORTANTE: rodar ANTES de ativar o modo Long Range — com WIFI_PROTOCOL_LR
// puro o radio nao decodifica beacons padrao. Empates favorecem 1/6/11.
void selectCleanChannel(bool force = false) {
#if FORCE_FIXED_CHANNEL
  // Modo de teste: mantem o RX em um canal conhecido para eliminar o scan
  // como variavel do diagnostico. Os TXs ainda podem parear por varredura.
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  activeEspNowChannel = ESPNOW_CHANNEL;
  Serial.printf("RF_CHANNEL,%d (fixo para teste)\n", activeEspNowChannel);
  return;
#endif
  if (!force) {
    int saved = loadChannelNvs();
    if (saved >= 1 && saved <= 13) {
      activeEspNowChannel = (uint8_t)saved;
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      Serial.printf("RF_CHANNEL,%d (salvo em NVS)\n", activeEspNowChannel);
      return;
    }
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.println("SCAN: buscando canal 2.4GHz mais limpo...");
  int found = WiFi.scanNetworks();
  if (found <= 0) {
    Serial.println("SCAN: nenhuma rede encontrada, mantendo canal padrao");
    saveChannelNvs(activeEspNowChannel);
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
    if (force) {
      String ssid = WiFi.SSID(i);
      ssid.replace('|', '_');
      ssid.replace('\n', ' ');
      ssid.replace('\r', ' ');
      int auth = WiFi.encryptionType(i);
      Serial.printf("SCAN_NET|%d|%d|%d|%s\n", ch, rssi, auth, ssid.c_str());
    }
  }
  if (force) Serial.printf("SCAN_DONE,%d\n", found);
  static const uint8_t prefOrder[13] = {12, 13, 1, 6, 11, 2, 7, 3, 8, 4, 9, 5, 10};
  uint8_t best = ESPNOW_CHANNEL;
  float bestScore = 1e9f;
  for (int i = 0; i < 13; i++) {
    uint8_t ch = prefOrder[i];
    if (score[ch] < bestScore) { bestScore = score[ch]; best = ch; }
  }
  activeEspNowChannel = best;
  saveChannelNvs(best);
  Serial.printf("RF_CHANNEL,%d\n", activeEspNowChannel);
}

// Diagnostico de canal (comando SCAN_REPORT): varre as redes 2.4GHz visiveis e
// conta quantas ocupam o MESMO canal em que o ESP-NOW opera. NAO troca o canal,
// nao grava no NVS e nao mexe nos peers — e somente leitura. O radio sai do
// canal durante a varredura (80ms por canal) e depois volta.
void scanDiagnostic() {
#if USE_LONG_RANGE
  Serial.println("SCAN_ERR: varredura indisponivel em Long Range");
  return;
#else
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  uint8_t active = activeEspNowChannel;
  int found = WiFi.scanNetworks(false, false, false, 80);
  if (found < 0) {
    Serial.println("SCAN_ERR: falha na varredura");
    return;
  }
  int same = 0;
  for (int i = 0; i < found; i++) {
    int ch = WiFi.channel(i);
    if (ch != (int)active) continue;
    same++;
    String ssid = WiFi.SSID(i);
    ssid.replace('|', '_');
    ssid.replace('\n', ' ');
    ssid.replace('\r', ' ');
    Serial.printf("SCAN_NET|%d|%d|%d|%s\n", ch, WiFi.RSSI(i), WiFi.encryptionType(i), ssid.c_str());
  }
  Serial.printf("SCAN_REPORT,active=%u,same=%d,total=%d\n", (unsigned)active, same, found);
  WiFi.scanDelete();
  applyChannelToRadio();
#endif
}

void setupEspNow() { esp_wifi_set_max_tx_power(52);
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
  float ratio = rpm / BASE_RPM; if(fabsf(ratio) < DEADZONE_RPM) ratio = 0.0f; if(ratio > MAX_RPM_RATIO) ratio = MAX_RPM_RATIO; if(ratio < -MAX_RPM_RATIO) ratio = -MAX_RPM_RATIO;
  int32_t phaseStep = (int32_t)(ratio * ((float)CV02_RESOLUTION * 65536.0f / SAMPLE_RATE));
  deck->cv02Phase64 += (int64_t)phaseStep; int64_t cycle = (int64_t)deck->cv02Phase64 >> 16;
  int64_t ci = cycle % (int64_t)CV02_LENGTH; if (ci < 0) ci += (int64_t)CV02_LENGTH;
  uint32_t cycleIndex = (uint32_t)ci;
  uint16_t tableIndex = (deck->cv02Phase64 >> 6) & (SIN_COS_TABLE_SIZE - 1);
  int16_t sine = sinTable[tableIndex]; int16_t cosine = cosTable[tableIndex];
  uint8_t bit = getPackedBit(cycleIndex); float modulation = bit ? 1.0f : 1.0f - ((-(float)cosine / 32767.0f + 1.0f) * 0.25f);
  *leftOut = (int16_t)((-(float)cosine / 32767.0f) * modulation * OUTPUT_GAIN * 32767.0f); *rightOut = (int16_t)(((float)sine / 32767.0f) * modulation * OUTPUT_GAIN * 32767.0f);
}

// Timecode Rekordbox: mesma quadratura do CV02 (L = -cos*mod, R = +sin*mod),
// com mod de 2 niveis constantes e bit vindo do LFSR gerado sob demanda.
static inline void renderRekordboxSample(audio_deck_state *deck, float rpm, int16_t *leftOut, int16_t *rightOut) {
  float ratio = rpm / BASE_RPM; if(fabsf(ratio) < DEADZONE_RPM) ratio = 0.0f; if(ratio > MAX_RPM_RATIO) ratio = MAX_RPM_RATIO; if(ratio < -MAX_RPM_RATIO) ratio = -MAX_RPM_RATIO;
  int32_t phaseStep = (int32_t)(ratio * ((float)RB_RESOLUTION * 65536.0f / SAMPLE_RATE));
  deck->cv02Phase64 += (int64_t)phaseStep; int64_t cycle = (int64_t)deck->cv02Phase64 >> 16;
  // Acompanha o ciclo avancando/retrocedendo o LFSR (delta e no maximo ~1 por amostra).
  int64_t delta = cycle - deck->rbCycle;
  if (delta > 0) { int64_t k = (delta > 64) ? 64 : delta; while (k--) deck->rbState = lfsrForwardT(deck->rbState, RB_TAPS); deck->rbCycle = cycle; }
  else if (delta < 0) { int64_t k = (delta < -64) ? -64 : delta; while (k++) deck->rbState = lfsrReverseT(deck->rbState, RB_TAPS); deck->rbCycle = cycle; }
  uint8_t bit = (uint8_t)(deck->rbState & 0x1U);
  uint16_t tableIndex = (deck->cv02Phase64 >> 6) & (SIN_COS_TABLE_SIZE - 1);
  int16_t sine = sinTable[tableIndex]; int16_t cosine = cosTable[tableIndex];
  float modulation = bit ? RB_LEVEL1 : RB_LEVEL0;
  *leftOut = (int16_t)((-(float)cosine / 32767.0f) * modulation * OUTPUT_GAIN * 32767.0f); *rightOut = (int16_t)(((float)sine / 32767.0f) * modulation * OUTPUT_GAIN * 32767.0f);
}

// Timecode Traktor MK2: portadora 3000 Hz em quadratura (L = -cos, R = +sin)
// SOMADA (offset) a um sinal PAM de 1200 simbolos/s (m-sequence de grau 22) com
// o pulso medido. O mesmo offset s(t) entra nas duas saidas.
static inline void renderTraktorSample(audio_deck_state *deck, float rpm, int16_t *leftOut, int16_t *rightOut) {
  float ratio = rpm / BASE_RPM; if(fabsf(ratio) < DEADZONE_RPM) ratio = 0.0f; if(ratio > MAX_RPM_RATIO) ratio = MAX_RPM_RATIO; if(ratio < -MAX_RPM_RATIO) ratio = -MAX_RPM_RATIO;
  // Relogio de simbolo (1200 Hz): avanca/retrocede o LFSR e a janela.
  int32_t symStep = (int32_t)(ratio * ((float)TR_RESOLUTION * 65536.0f / SAMPLE_RATE));
  deck->trSymPhase64 += (int64_t)symStep;
  int64_t cyc = deck->trSymPhase64 >> 16;
  int64_t delta = cyc - deck->trSymCycle;
  if (delta != 0) {
    int64_t k = delta > 8 ? 8 : (delta < -8 ? -8 : delta);
    if (k > 0) { while (k--) deck->trState = trForward(deck->trState); }
    else { while (k++) deck->trState = trReverse(deck->trState); }
    deck->trSymCycle = cyc;
    trBuildWindow(deck);
  }
  // Offset PAM: convolucao da janela de simbolos com o pulso medido.
  uint32_t frac16 = (uint32_t)(deck->trSymPhase64 & 0xFFFF);
  float f = (float)frac16 * (1.0f / 65536.0f);
  float stepSamples = (float)SAMPLE_RATE / (float)TR_RESOLUTION;
  float s = 0.0f;
  for (int m = -TR_J; m <= TR_J; m++) {
    float x = (f - (float)m) * stepSamples + (float)TR_PULSE_CENTER;
    int i0 = (int)floorf(x);
    if (i0 < 0) i0 = 0;
    if (i0 > TR_PULSE_LEN - 2) i0 = TR_PULSE_LEN - 2;
    float fr = x - (float)i0;
    float pv = (float)trPulse[i0] + fr * ((float)trPulse[i0 + 1] - (float)trPulse[i0]);
    if (deck->trWin[TR_J + m] == 0) pv = -pv;
    s += pv;
  }
  s *= (TR_OFFSET_LEVEL / 10000.0f);
  // Portadora de 3000 Hz (L = -cos, R = +sin).
  int32_t carStep = (int32_t)(ratio * ((float)TR_CARRIER_HZ * 65536.0f / SAMPLE_RATE));
  deck->trCarPhase64 += (int64_t)carStep;
  uint16_t tableIndex = (deck->trCarPhase64 >> 6) & (SIN_COS_TABLE_SIZE - 1);
  float sine = (float)sinTable[tableIndex] / 32767.0f;
  float cosine = (float)cosTable[tableIndex] / 32767.0f;
  *leftOut = (int16_t)((-cosine * TR_CARRIER_LEVEL + s) * OUTPUT_GAIN * 32767.0f);
  *rightOut = (int16_t)((sine * TR_CARRIER_LEVEL + s) * OUTPUT_GAIN * 32767.0f);
}

// Despacha para o renderer do modo ativo (Serato/CV02, Rekordbox ou Traktor).
static inline void renderTimecodeSample(audio_deck_state *deck, float rpm, int16_t *leftOut, int16_t *rightOut) {
  if (timecodeMode == TC_MODE_REKORDBOX) renderRekordboxSample(deck, rpm, leftOut, rightOut);
  else if (timecodeMode == TC_MODE_TRAKTOR) renderTraktorSample(deck, rpm, leftOut, rightOut);
  else renderCv02Sample(deck, rpm, leftOut, rightOut);
}

float readTargetRpm(uint8_t deckId) { uint8_t deckIndex = deckId <= 1 ? 0 : 1; int16_t rpmCenti = 0; uint32_t lastSeenMillis = 0; portENTER_CRITICAL(&stateMux); rpmCenti = deckStates[deckIndex].rpmCenti; lastSeenMillis = deckStates[deckIndex].lastSeenMillis; portEXIT_CRITICAL(&stateMux); if (lastSeenMillis == 0 || millis() - lastSeenMillis > DECK_TIMEOUT_MS) return 0.0f; return (float)rpmCenti / 100.0f; }
void audioTask(void *param) { audio_deck_state *deck = (audio_deck_state *)param; int16_t buffer[DMA_BUF_LEN * 2]; uint8_t deckIdx = deck->deckId - 1; uint32_t lastWriteUs = micros(); uint32_t qFrames = DMA_TOTAL_FRAMES; while (true) {
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
      deck->calibStableStart = 0;
      deck->filteredRpm = 0.0f;
    } else {
      if (deck->calibrating && gStartFadeMs > 0) deck->fadeStartMillis = millis();
      deck->calibStableStart = 0;
      deck->calibrating = false;
      float delta = targetRpm - deck->filteredRpm;
      float alpha = (fabsf(delta) > 2.5f) ? 0.8f : RPM_SMOOTHING;
      deck->filteredRpm += delta * alpha;
    }
    for (int i = 0; i < DMA_BUF_LEN; i++) { int16_t left, right; if (millis() < deck->testToneUntil) { deck->tonePhase += (uint32_t)((float)deck->testToneFreqHz * 65536.0f / SAMPLE_RATE); int16_t tone = (int16_t)((float)sinTable[(deck->tonePhase >> 6) & (SIN_COS_TABLE_SIZE - 1)] * TEST_TONE_AMPLITUDE); left = tone; right = tone; } else if (stopped) { left = 0; right = 0; } else { renderTimecodeSample(deck, deck->filteredRpm, &left, &right); } buffer[i * 2] = left; buffer[i * 2 + 1] = right; }
    // Volume do deck + fade de inicio (envelope 0->1 em START_FADE_MS).
    float fadeGain = 1.0f;
    if (deck->fadeStartMillis && gStartFadeMs > 0) {
      uint32_t elapsed = millis() - deck->fadeStartMillis;
      if (elapsed < (uint32_t)gStartFadeMs) fadeGain = (float)elapsed / (float)gStartFadeMs;
      else deck->fadeStartMillis = 0;
    }
    if (deck->volume < 0.999f || fadeGain < 1.0f) {
      float gain = deck->volume * fadeGain;
      for (int v = 0; v < DMA_BUF_LEN * 2; v++) {
        int32_t scaled = (int32_t)((float)buffer[v] * gain);
        if (scaled > 32767) scaled = 32767;
        if (scaled < -32768) scaled = -32768;
        buffer[v] = (int16_t)scaled;
      }
    }
    size_t written;
    esp_err_t i2sErr = i2s_write(deck->port, buffer, sizeof(buffer), &written, portMAX_DELAY);
    {
      uint32_t nowUs = micros();
      uint32_t writeGapUs = nowUs - lastWriteUs;
      lastWriteUs = nowUs;
      if (writeGapUs > UNDERFLOW_GAP_US) underrunCount[deckIdx]++;
      if (writeGapUs > underrunMaxGapUs[deckIdx]) underrunMaxGapUs[deckIdx] = writeGapUs;
      // Estimativa da ocupacao do ring DMA (Lei de Little): o DAC consome
      // frames pelo clock real; o task repoe DMA_BUF_LEN por escrita.
      uint32_t consumedFrames = (uint32_t)((uint64_t)writeGapUs * SAMPLE_RATE / 1000000ULL);
      if (consumedFrames > qFrames) consumedFrames = qFrames;
      qFrames = qFrames - consumedFrames + DMA_BUF_LEN;
      if (qFrames > DMA_TOTAL_FRAMES) qFrames = DMA_TOTAL_FRAMES;
      uint32_t pct = (uint32_t)((uint64_t)qFrames * 100ULL / DMA_TOTAL_FRAMES);
      buffSumPct[deckIdx] += pct;
      buffCount[deckIdx]++;
      if (pct < buffMinPct[deckIdx]) buffMinPct[deckIdx] = pct;
    }
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
  for (uint8_t i = 0; i < 2; i++) { uint8_t welcomeMacCopy[6]; uint8_t welcomeDeckCopy = 0; bool shouldSendWelcome = false; portENTER_CRITICAL(&stateMux); if (hasPendingWelcome[i]) { memcpy(welcomeMacCopy, pendingWelcomeMac[i], 6); welcomeDeckCopy = pendingWelcomeDeck[i]; hasPendingWelcome[i] = false; shouldSendWelcome = true; } portEXIT_CRITICAL(&stateMux); if (shouldSendWelcome) { Serial.printf("WELCOME_TX deck=%u ch=%u\n", (unsigned)welcomeDeckCopy, (unsigned)activeEspNowChannel); sendControlMessage(welcomeMacCopy, welcomeDeckCopy, MSG_WELCOME);
    // Reaplica os filtros persistidos quando o deck reconecta: cobre o caso
    // do TX perder os valores no NVS do C3 (fonte de segurança).
    if (gFilterCache[i].valid) {
      Serial.printf("FILTER_REAPPLY deck=%u slow=%.3f fast=%.3f thr=%.3f\n", welcomeDeckCopy, gFilterCache[i].slow, gFilterCache[i].fast, gFilterCache[i].thr);
      sendParamCommand(welcomeDeckCopy, CFG_ALPHA_SLOW, (int16_t)lroundf(gFilterCache[i].slow * 1000.0f));
      sendParamCommand(welcomeDeckCopy, CFG_ALPHA_FAST, (int16_t)lroundf(gFilterCache[i].fast * 1000.0f));
      sendParamCommand(welcomeDeckCopy, CFG_FAST_THRESHOLD, (int16_t)lroundf(gFilterCache[i].thr * 1000.0f));
    }
    Serial.printf("POWER_REAPPLY deck=%u qdbm=%u\n", (unsigned)welcomeDeckCopy, (unsigned)txPowerQdbm[welcomeDeckCopy - 1]);
    sendParamCommand(welcomeDeckCopy, CFG_TX_POWER, (int16_t)txPowerQdbm[welcomeDeckCopy - 1]);
  } }
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
      } else if (cmdLine.startsWith("UPTIME")) {
        uint32_t upMillis = millis();
        for (uint8_t d = 0; d < 2; d++) {
          if (deckStates[d].firstSeenMillis != 0) {
            uint32_t upSec = (upMillis - deckStates[d].firstSeenMillis) / 1000;
            uint32_t h = upSec / 3600;
            uint32_t m = (upSec % 3600) / 60;
            uint32_t s = upSec % 60;
            if (h > 0) Serial.printf("UPTIME deck%d=%luh%lum%lus\n", d + 1, (unsigned long)h, (unsigned long)m, (unsigned long)s);
            else if (m > 0) Serial.printf("UPTIME deck%d=%lum%lus\n", d + 1, (unsigned long)m, (unsigned long)s);
            else Serial.printf("UPTIME deck%d=%lus\n", d + 1, (unsigned long)s);
          } else {
            Serial.printf("UPTIME deck%d=off\n", d + 1);
          }
        }
      } else if (cmdLine.startsWith("CALIB_A") || cmdLine.startsWith("CALIB_B")) {
        float mult = cmdLine.substring(7).toFloat();
        if (mult > 0.5f && mult < 2.0f) {
          uint8_t deck = cmdLine.startsWith("CALIB_A") ? 1 : 2;
          sendCalibCommand(deck, mult);
        } else {
          Serial.println("CALIB_ERR: valor invalido (use M entre 0.5 e 2.0)");
        }
      } else if (cmdLine.startsWith("CHANNEL")) {
        String arg = cmdLine.substring(7);
        arg.trim();
        if (arg.length() == 0) {
          Serial.printf("RF_CHANNEL,%d\n", activeEspNowChannel);
        } else {
          int ch = arg.toInt();
          if (ch >= 1 && ch <= 13) {
            activeEspNowChannel = (uint8_t)ch;
            applyChannelToRadio();
            saveChannelNvs((uint8_t)ch);
            Serial.printf("RF_CHANNEL,%d\n", activeEspNowChannel);
          } else {
            Serial.println("CHANNEL_ERR: use 1-13");
          }
        }
      } else if (cmdLine.equals("SCAN_REPORT")) {
        scanDiagnostic();
      } else if (cmdLine.equals("SCAN")) {
        selectCleanChannel(true);
        applyChannelToRadio();
      } else if (cmdLine.startsWith("DECK_ID")) {
        int fromSlot = cmdLine.substring(8).toInt();
        int toId = 0;
        int sp = cmdLine.indexOf(' ', 8);
        if (sp >= 8) toId = cmdLine.substring(sp + 1).toInt();
        if (fromSlot < 1 || fromSlot > 2) {
          Serial.println("CFG_ERR: use DECK_ID <1|2> [1|2] (slot -> novo deck; sem argumento inverte)");
        } else {
          if (toId != 1 && toId != 2) toId = (fromSlot == 1) ? 2 : 1;
          sendParamCommand((uint8_t)fromSlot, CFG_DECK_ID, (int16_t)toId);
          Serial.printf("DECK_ID_SENT slot=%d newId=%d\n", fromSlot, toId);
        }
      } else if (cmdLine.startsWith("DECK_STATUS")) {
        for (uint8_t d = 1; d <= 2; d++) {
          uint32_t last = 0;
          portENTER_CRITICAL(&stateMux);
          last = deckStates[d - 1].lastSeenMillis;
          portEXIT_CRITICAL(&stateMux);
          Serial.printf("DECK_STATUS slot=%u online=%u deckId=%u\n", d, (last != 0) ? 1 : 0, d);
        }
      } else if (cmdLine.startsWith("FILTER?")) {
        // Leitura dos filtros vigentes nos TX: encaminha MSG_CFG_GET ao(s) deck(s).
        // CFG_ACK retorna (3 por deck) e ja e impresso no serial para o dashboard.
        int deck = 0;
        bool perDeck = false;
        String arg = cmdLine.substring(7);
        arg.trim();
        if (arg.length() == 1 && (arg[0] == '1' || arg[0] == '2')) {
          perDeck = true; deck = arg[0] - '0';
        }
        uint8_t sent = 0;
        uint8_t startDeck = perDeck ? (uint8_t)deck : 1;
        uint8_t endDeck = perDeck ? (uint8_t)deck : 2;
        for (uint8_t d = startDeck; d <= endDeck; d++) {
          uint8_t macCopy[6];
          portENTER_CRITICAL(&stateMux);
          uint32_t last = deckStates[d - 1].lastSeenMillis;
          memcpy(macCopy, deckStates[d - 1].mac, 6);
          portEXIT_CRITICAL(&stateMux);
          if (last == 0) continue;
          sendControlMessage(macCopy, d, MSG_CFG_GET);
          sent++;
        }
        if (sent == 0) Serial.println("CFG_ERR: nenhum deck com sinal");
        else Serial.printf("FILTER_GET_SENT decks=%u\n", sent);
      } else if (cmdLine.startsWith("FILTER")) {
        int deck = 0; float slow = 0, fast = 0, thr = 0;
        int n = sscanf(cmdLine.c_str(), "FILTER %d %f %f %f", &deck, &slow, &fast, &thr);
        bool perDeck = (n == 4 && (deck == 1 || deck == 2));
        if (!perDeck) {
          n = sscanf(cmdLine.c_str(), "FILTER %f %f %f", &slow, &fast, &thr);
        }
        if (n != (perDeck ? 4 : 3) || slow < 0.05f || slow > 0.99f ||
            fast < 0.05f || fast > 0.99f || thr < 0.01f || thr > 10.0f) {
          Serial.println("CFG_ERR: use FILTER [<deck 1|2>] <alphaSlow> <alphaFast> <threshold>");
        } else {
          uint8_t sent = 0;
          uint8_t startDeck = perDeck ? (uint8_t)deck : 1;
          uint8_t endDeck = perDeck ? (uint8_t)deck : 2;
          for (uint8_t d = startDeck; d <= endDeck; d++) {
            uint8_t macCopy[6];
            portENTER_CRITICAL(&stateMux);
            uint32_t last = deckStates[d - 1].lastSeenMillis;
            memcpy(macCopy, deckStates[d - 1].mac, 6);
            portEXIT_CRITICAL(&stateMux);
            if (last == 0) continue;
            sendParamCommand(d, CFG_ALPHA_SLOW, (int16_t)lroundf(slow * 1000.0f));
            sendParamCommand(d, CFG_ALPHA_FAST, (int16_t)lroundf(fast * 1000.0f));
            sendParamCommand(d, CFG_FAST_THRESHOLD, (int16_t)lroundf(thr * 1000.0f));
            gFilterCache[d - 1].slow = slow;
            gFilterCache[d - 1].fast = fast;
            gFilterCache[d - 1].thr = thr;
            gFilterCache[d - 1].valid = true;
            saveFilterCacheNvs(d - 1);
            sent++;
          }
          if (sent == 0) Serial.println("CFG_ERR: nenhum deck com sinal");
          else Serial.printf("FILTER_SENT decks=%u slow=%.3f fast=%.3f thr=%.3f\n", sent, slow, fast, thr);
        }
      } else if (cmdLine.startsWith("TX_POWER?")) {
        Serial.printf("TX_POWER_OK,%u,%u\n", (unsigned)txPowerQdbm[0], (unsigned)txPowerQdbm[1]);
      } else if (cmdLine.startsWith("TX_POWER")) {
        int deck = 0, power = 0;
        int n = sscanf(cmdLine.c_str(), "TX_POWER %d %d", &deck, &power);
        if (n != 2 || (deck != 1 && deck != 2) ||
            !(power == 40 || power == 52 || power == 64 || power == 72 || power == 80)) {
          Serial.println("TX_POWER_ERR: use TX_POWER <deck 1|2> <40|52|64|72|80>");
        } else {
          uint8_t d = (uint8_t)(deck - 1);
          uint32_t last = 0;
          portENTER_CRITICAL(&stateMux);
          last = deckStates[d].lastSeenMillis;
          portEXIT_CRITICAL(&stateMux);
          if (last == 0) {
            Serial.printf("TX_POWER_ERR: deck %d sem sinal\n", deck);
          } else {
            txPowerQdbm[d] = (uint8_t)power;
            saveTxPowerNvs(d);
            sendParamCommand((uint8_t)deck, CFG_TX_POWER, (int16_t)power);
            Serial.printf("TX_POWER_SENT deck=%d qdbm=%d dbm=%.2f\n", deck, power, power / 4.0f);
          }
        }
      } else if (cmdLine.startsWith("LED_BRIGHT")) {
        String arg = cmdLine.substring(10);
        arg.trim();
        if (arg.length() == 0) {
          Serial.printf("LED_BRIGHT_OK,%d\n", (int)ledBrightness);
        } else {
          int b = arg.toInt();
          if (b >= 0 && b <= 100) {
            ledBrightness = (uint8_t)b;
            saveLedBrightnessNvs(ledBrightness);
            Serial.printf("LED_BRIGHT_OK,%d\n", (int)ledBrightness);
          } else {
            Serial.println("LED_ERR: use 0-100");
          }
        }
      } else if (cmdLine.startsWith("VOLUME")) {
        if (cmdLine.equals("VOLUME")) {
          Serial.printf("VOLUME_OK,%.3f,%.3f\n", audioDecks[0].volume, audioDecks[1].volume);
        } else {
          int deck = 0; float vol = 0;
          int n = sscanf(cmdLine.c_str(), "VOLUME %d %f", &deck, &vol);
          bool valid = false;
          if (n == 2 && (deck == 1 || deck == 2) && vol >= 0.0f && vol <= 1.0f) {
            audioDecks[deck - 1].volume = vol;
            saveDeckVolumeNvs(deck - 1, vol);
            valid = true;
          } else {
            n = sscanf(cmdLine.c_str(), "VOLUME %f", &vol);
            if (n == 1 && vol >= 0.0f && vol <= 1.0f) {
              audioDecks[0].volume = vol;
              audioDecks[1].volume = vol;
              saveDeckVolumeNvs(0, vol);
              saveDeckVolumeNvs(1, vol);
              valid = true;
            }
          }
if (valid) Serial.printf("VOLUME_OK,%.3f,%.3f\n", audioDecks[0].volume, audioDecks[1].volume);
          else Serial.println("VOLUME_ERR: use VOLUME [<deck 1|2>] <0.00-1.00>");
        }
      } else if (cmdLine.startsWith("FADE")) {
        String arg = cmdLine.substring(5);
        arg.trim();
        if (arg.length() == 0) {
          Serial.printf("FADE_OK,%d\n", (int)gStartFadeMs);
        } else {
          int ms = arg.toInt();
          if (ms >= 0 && ms <= 1000) {
            gStartFadeMs = (uint16_t)ms;
            saveStartFadeNvs(gStartFadeMs);
            Serial.printf("FADE_OK,%d\n", (int)gStartFadeMs);
          } else {
            Serial.println("FADE_ERR: use 0-1000 (ms)");
          }
        }
      } else if (cmdLine.startsWith("TIMECODE")) {
        String arg = cmdLine.substring(8);
        arg.trim();
        if (arg.length() == 0) {
          Serial.printf("TIMECODE_OK,%u\n", (unsigned)timecodeMode);
        } else {
          int m = -1;
          if (arg.equalsIgnoreCase("serato") || arg.equalsIgnoreCase("cv02") || arg == "0") m = TC_MODE_SERATO;
          else if (arg.equalsIgnoreCase("rekordbox") || arg.equalsIgnoreCase("rb") || arg == "1") m = TC_MODE_REKORDBOX;
          else if (arg.equalsIgnoreCase("traktor") || arg.equalsIgnoreCase("mk2") || arg == "2") m = TC_MODE_TRAKTOR;
          if (m < 0) {
            Serial.println("TIMECODE_ERR: use TIMECODE serato|rekordbox|traktor");
          } else {
            timecodeMode = (uint8_t)m;
            saveTimecodeModeNvs(timecodeMode);
            // Ressincroniza os LFSRs (rekordbox e traktor) com o ciclo atual de
            // cada deck, evitando um salto gigante ao trocar de modo girando.
            for (uint8_t d = 0; d < 2; d++) {
              audioDecks[d].rbState = RB_SEED;
              audioDecks[d].rbCycle = (int64_t)(audioDecks[d].cv02Phase64 >> 16);
              trInitDeck(&audioDecks[d]);
            }
            Serial.printf("TIMECODE_OK,%u\n", (unsigned)timecodeMode);
          }
        }
      }
      cmdLine = "";
    } else if (c != '\r') {
      cmdLine += c;
    }
  }
}

void updateDeckLed(uint8_t deckIndex, uint8_t ledPin) {
  static uint32_t warnHoldUntil[2] = {0, 0};
  bool seen = false;
  uint32_t lastSeenMillis = 0;
  int8_t rssi = 0;
  uint16_t lost = 0;
  portENTER_CRITICAL(&stateMux);
  seen = deckStates[deckIndex].seen;
  lastSeenMillis = deckStates[deckIndex].lastSeenMillis;
  rssi = deckStates[deckIndex].rssi;
  lost = deckStates[deckIndex].lastGap;
  portEXIT_CRITICAL(&stateMux);

  uint32_t now = millis();
  bool alive = (seen && lastSeenMillis != 0 && (now - lastSeenMillis <= DECK_TIMEOUT_MS));
  if (!alive) {
    // Timeout: reinicia a base de sequencia para nao contabilizar o gap da
    // varredura de reconexao como perda de pacotes (o TX segue enviando
    // durante o scan, mas o RX nao escuta canais de varredura).
    portENTER_CRITICAL(&stateMux);
    if (deckStates[deckIndex].seen) {
      deckStates[deckIndex].seen = false;
      deckStates[deckIndex].lastSeq = 0;
      deckStates[deckIndex].lastGap = 0;
    }
    portEXIT_CRITICAL(&stateMux);
    if (ledPwmOk[deckIndex]) ledcWrite(ledPin, 0); else digitalWrite(ledPin, LOW);
    return;
  }

  bool warn = (rssi < RSSI_WARN_THRESHOLD_DBM) || (lost > LOST_PACKET_WARN_THRESHOLD);
  if (warn) warnHoldUntil[deckIndex] = now + WARN_HOLD_MS;
  bool blinking = (now < warnHoldUntil[deckIndex]);
  uint32_t duty = (uint32_t)((uint32_t)ledBrightness * 255UL / 100UL);
  if (ledPwmOk[deckIndex]) {
    ledcWrite(ledPin, blinking ? (((now / WARN_BLINK_MS) % 2) ? duty : 0) : duty);
  } else {
    digitalWrite(ledPin, blinking ? ((now / WARN_BLINK_MS) % 2) : (duty > 0));
  }
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
  Serial.printf("TELEM,%lu,%d,%d,%d,%d,%lu,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%lu,%lu,%lu\n",
    (unsigned long)now,
    (local[0].lastSeenMillis != 0 && now - local[0].lastSeenMillis <= DECK_TIMEOUT_MS) ? 1 : 0,
    (int)local[0].rssi, (int)local[0].batteryPct, (int)local[0].rpmCenti, (unsigned long)local[0].lostPackets,
    (local[1].lastSeenMillis != 0 && now - local[1].lastSeenMillis <= DECK_TIMEOUT_MS) ? 1 : 0,
    (int)local[1].rssi, (int)local[1].batteryPct, (int)local[1].rpmCenti, (unsigned long)local[1].lostPackets,
    (unsigned long)((audioDecks[0].cv02Phase64 >> 16) / 1000ULL),
    (unsigned long)((audioDecks[1].cv02Phase64 >> 16) / 1000ULL),
    (local[0].seen ? (unsigned long)(now - local[0].lastSeenMillis) : 0UL),
    (local[1].seen ? (unsigned long)(now - local[1].lastSeenMillis) : 0UL),
    (unsigned long)wrDelta[0], (unsigned long)wrDelta[1],
    (audioDecks[0].testToneUntil != 0 && now < audioDecks[0].testToneUntil) ? 1 : 0,
    (audioDecks[1].testToneUntil != 0 && now < audioDecks[1].testToneUntil) ? 1 : 0,
    (int)lastI2sErr[0], (int)lastI2sErr[1],
    (unsigned long)peakSample[0], (unsigned long)peakSample[1]);
  uint32_t avgBuffA = buffCount[0] ? buffSumPct[0] / buffCount[0] : 100;
  uint32_t avgBuffB = buffCount[1] ? buffSumPct[1] / buffCount[1] : 100;
  Serial.printf("AUDIO_STAT,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
    (unsigned long)underrunCount[0], (unsigned long)underrunCount[1],
    (unsigned long)underrunMaxGapUs[0], (unsigned long)underrunMaxGapUs[1],
    (unsigned long)avgBuffA, (unsigned long)buffMinPct[0],
    (unsigned long)avgBuffB, (unsigned long)buffMinPct[1]);
  underrunMaxGapUs[0] = 0;
  underrunMaxGapUs[1] = 0;
  buffSumPct[0] = 0; buffCount[0] = 0; buffMinPct[0] = 100;
  buffSumPct[1] = 0; buffCount[1] = 0; buffMinPct[1] = 100;
  // Perda real em % desta janela de 500ms; 255 = nenhum pacote recebido no periodo.
  uint32_t lossPct[2];
  for (uint8_t i = 0; i < 2; i++) {
    uint32_t total = local[i].winReceived + local[i].winMissed;
    lossPct[i] = total ? (unsigned long)(local[i].winMissed * 100UL / total) : 255;
  }
  Serial.printf("LOSS_PCT,%lu,%lu\n", (unsigned long)lossPct[0], (unsigned long)lossPct[1]);
  portENTER_CRITICAL(&stateMux);
  deckStates[0].winReceived = 0; deckStates[0].winMissed = 0;
  deckStates[1].winReceived = 0; deckStates[1].winMissed = 0;
  portEXIT_CRITICAL(&stateMux);
}

void setup() {
  // LEDs de deck via PWM (ledc) para controle de brilho; LED de calibracao continua GPIO.
  ledBrightness = (uint8_t)loadLedBrightnessNvs();
  audioDecks[0].volume = loadDeckVolumeNvs(0);
  audioDecks[1].volume = loadDeckVolumeNvs(1);
  gStartFadeMs = loadStartFadeNvs();
  timecodeMode = loadTimecodeModeNvs();
  loadTxPowerNvs();
  loadFilterCacheNvs();
  ledPwmOk[0] = ledcAttach(LED_DECK_A, 5000, 8);
  ledPwmOk[1] = ledcAttach(LED_DECK_B, 5000, 8);
  if (!ledPwmOk[0]) pinMode(LED_DECK_A, OUTPUT);
  if (!ledPwmOk[1]) pinMode(LED_DECK_B, OUTPUT);
  pinMode(LED_CALIB, OUTPUT);
  Serial.begin(DEBUG_BAUD);
  Serial.println("DVS_RX_READY,v2");
  Serial.printf("LED_PWM_A=%d LED_PWM_B=%d\n", ledPwmOk[0], ledPwmOk[1]);
  Serial.printf("TIMECODE_OK,%u\n", (unsigned)timecodeMode);
  buildSinCosTables();
  buildCv02Bits();
  // Inicializa o LFSR/janela do Traktor (trState=TR_SEED, fase 0, janela pronta).
  trInitDeck(&audioDecks[0]);
  trInitDeck(&audioDecks[1]);
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

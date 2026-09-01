/*
========================================================
 ESP32-C3 TRANSMITTER + BMI270 (SparkFun lib)
 DVS / Phase DIY - ESP-NOW low latency motion packet
 Adaptado do firmware C3+BMI160 mantendo o mesmo protocolo,
 timing, calibracao automatica, filtro e diagnostico por LED.
========================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// ============ DEFINA O DECK DESTA PLACA ============
// TX_DECK_ID: 1 = Deck A (deck 1), 2 = Deck B (deck 2)
// Para gravar a placa do deck 2, mude para 2 e compile.
#define TX_DECK_ID 2
// ===================================================
#define DEFAULT_DECK_ID TX_DECK_ID
uint8_t deckId = DEFAULT_DECK_ID;

#define ESPNOW_CHANNEL 11  // apenas canal inicial/fallback; o pareamento descobre o canal do RX
#define USE_LONG_RANGE 1  // 1 = modo long range (512/256 kbps, OBRIGATORIO ser igual ao RX), 0 = taxa normal
#define SEND_RATE_HZ 150
#define SEND_INTERVAL_US (1000000UL / SEND_RATE_HZ)
// Pareamento por varredura: o TX pula pelos canais enviando HELLO ate o RX
// (que fica fixo no canal mais limpo escolhido no boot dele) responder WELCOME.
#define PAIR_CHANNEL_DWELL_MS 120  // tempo em cada canal antes de pular pro proximo
#define PAIR_HELLO_INTERVAL_MS 40  // intervalo entre HELLOs dentro do mesmo canal
#define HANDSHAKE_TIMEOUT_MS 3000

#define PROTOCOL_VERSION 2
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4

// LED onboard azul do ESP32-C3 Super Mini (ativo em LOW).
// Feedback de estado: Sem Deck = 3 piscadas, Deck A = 1, Deck B = 2.
#define ONBOARD_LED_PIN 8
#define LED_BLINK_MS 150

// Botao BOOT (GPIO9) alterna o deck: Sem Deck -> A -> B -> Sem Deck.
#define BOOT_BUTTON_PIN 9
#define BUTTON_DEBOUNCE_MS 200

// Pino ADC da bateria (divisor 2:1 10K+10K no pino 3).
// BATT_FULL/EMPTY_MV sao as tensoes no PINO: 4.2V cheio -> 2.10V,
// 3.3V vazio -> 1.65V. Deixe BATT_PIN -1 se nao houver divisor.
#define BATT_PIN 3
// Calibracao individual por deck (variacao de hardware/ADC):
// deck A chega a 100% com ~2042mV no pino; deck B com ~2050mV.
#if TX_DECK_ID == 1
#define BATT_FULL_MV 2040
#else
#define BATT_FULL_MV 2050
#endif
#define BATT_EMPTY_MV 1650
#define BATT_SAMPLE_MS 2000
#define BATT_AVG_SAMPLES 8
#define BATT_LOW_THRESHOLD_PCT 20
#define BATT_LOW_BLINK_MS 300

uint8_t receiverMAC[] = { 0x14, 0xC1, 0x9F, 0x2C, 0xDE, 0x7C };

// Canais varridos durante o pareamento (1..13, Brasil).
// O TX pula por essa lista ate receber WELCOME do RX.
static const uint8_t pairChannels[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
static const uint8_t pairChannelCount = sizeof(pairChannels) / sizeof(pairChannels[0]);
uint8_t activeChannel = ESPNOW_CHANNEL;
int16_t hopIndex = -1;  // -1 = ainda nao comecou a pular
uint32_t lastHopMillis = 0;
uint32_t lastPairHelloMillis = 0;
bool wasReceiverReady = false;

// Filtro assimetrico (fast attack / slow release):
// - ALPHA_SLOW suaviza ruido durante giro estavel.
// - ALPHA_FAST responde quase instantaneamente a paradas
//   bruscas e scratches (mudancas grandes de RPM).
// - FAST_THRESHOLD_RPM define o que conta como "transiente
//   grande" (em RPM). Ajuste conforme o comportamento desejado:
//   valores menores tornam o ataque rapido mais sensivel.
float ALPHA_SLOW = 0.50f;
float ALPHA_FAST = 0.85f;
float FAST_THRESHOLD_RPM = 0.5f;
float DEADZONE_RPM = 0.20f;
// SENTIDO DO GIRO: depende de como o BMI270 esta MONTADO na placa.
// - Se girando PARA FRENTE o painel/Serato mostra para TRAS: multiplique por -1.
// - Se girando PARA FRENTE mostra para frente: deixe em +1.
// (esta placa atual le como o M5Stick: +1)
// Calibracao individual por deck (variacao do scale do giroscopio):
// 1.003 resultou em offset constante +0.4% (0%->+0.4, +8%->+8.4, -8%->-7.6).
// 0.999 anula esse offset: 0%->0.0, +8%->+8.0, -8%->-8.0 no Serato.
// Ajustar apos teste se ainda desviar.
// Calibracao deck 2 (TX_DECK_ID 2)
#if TX_DECK_ID == 2
#define RPM_MULTIPLIER 0.9974f
#else
#define RPM_MULTIPLIER 0.999f
#endif

// Auto-calibracao: o transmissor espera uma janela de gyro
// estavel com o toca-discos parado. Se houver movimento, a
// janela reinicia para nao gravar offset durante o giro.
#define AUTO_CALIBRATION_STABLE_SAMPLES 400
#define AUTO_CALIBRATION_SAMPLE_DELAY_MS 2
#define AUTO_CALIBRATION_STABLE_DPS_DELTA 2.0f
#define AUTO_CALIBRATION_MIN_COMPARE_SAMPLES 20
#define AUTO_CALIBRATION_MAX_DPS 6.0f
#define AUTO_CALIBRATION_TIMEOUT_MS 4000

// Auto-trim continuo: re-ancora o offset enquanto o prato
// estiver parado (espelha o comportamento do M5Stick).
#define AUTO_TRIM_WINDOW_MS 2000
#define AUTO_TRIM_STABLE_DELTA_DPS 2.0f
#define AUTO_TRIM_MAX_ABS_DPS 50.0f

typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  int16_t rpmCenti;
  int16_t gyroRaw;  // dps * 10 (deci-graus/s), NAO e mais o ADC bruto
  uint8_t batteryPct;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

dvs_packet packet;

volatile bool receiverReady = false;
volatile uint32_t lastReceiverReplyMillis = 0;

BMI270 imu;

float gyroOffsetZ = 0.0f; // em dps
float filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;
volatile uint8_t batteryLevelPct = 100;
uint32_t lastBattSampleMillis = 0;

void setOnboardLed(bool on) {
  digitalWrite(ONBOARD_LED_PIN, on ? LOW : HIGH); // ativo em LOW
}

void setupLED() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  setOnboardLed(false);
}

void blinkOnboardLed(uint16_t times) {
  for (uint16_t i = 0; i < times; i++) {
    setOnboardLed(true);
    delay(LED_BLINK_MS);
    setOnboardLed(false);
    delay(LED_BLINK_MS);
  }
}

bool probeBMI270() {
  if (imu.beginI2C(BMI2_I2C_SEC_ADDR) == BMI2_OK) return true;
  if (imu.beginI2C(BMI2_I2C_PRIM_ADDR) == BMI2_OK) return true;
  return false;
}

void setupBMI270() {
  Wire.begin();

  int attempts = 0;
  while (!probeBMI270()) {
    Serial.println("BMI270 nao encontrado!");
    blinkOnboardLed(2);
    attempts++;
    if (attempts > 20) {
      // trava piscando se nao conectar de forma alguma
      while (true) blinkOnboardLed(2);
    }
  }

  Serial.println("BMI270 conectado!");
}

static inline bool readGyroZDps(float *dps) {
  imu.getSensorData();
  *dps = imu.data.gyroZ;
  return true;
}

void autoCalibrateGyroZ() {
  int stableSamples = 0;
  float sum = 0.0f;
  uint32_t startCal = millis();

  while (stableSamples < AUTO_CALIBRATION_STABLE_SAMPLES) {
    float dps = 0.0f;
    readGyroZDps(&dps);

    if (fabsf(dps) > AUTO_CALIBRATION_MAX_DPS) {
      stableSamples = 0;
      sum = 0.0f;
      setOnboardLed(false);
      if (millis() - startCal > AUTO_CALIBRATION_TIMEOUT_MS) break;
      delay(AUTO_CALIBRATION_SAMPLE_DELAY_MS);
      continue;
    }

    if (stableSamples >= AUTO_CALIBRATION_MIN_COMPARE_SAMPLES) {
      float mean = sum / stableSamples;
      if (fabsf(dps - mean) > AUTO_CALIBRATION_STABLE_DPS_DELTA) {
        stableSamples = 0;
        sum = 0.0f;
        setOnboardLed(false);
        if (millis() - startCal > AUTO_CALIBRATION_TIMEOUT_MS) break;
        delay(LED_BLINK_MS);
        continue;
      }
    }

    sum += dps;
    stableSamples++;

    if ((stableSamples % 25) == 0) {
      setOnboardLed(true);
    } else if ((stableSamples % 25) == 12) {
      setOnboardLed(false);
    }

    delay(AUTO_CALIBRATION_SAMPLE_DELAY_MS);
  }

  if (stableSamples >= AUTO_CALIBRATION_STABLE_SAMPLES) {
    gyroOffsetZ = sum / (float)stableSamples;
  } else {
    gyroOffsetZ = 0.0f;
    Serial.println("Calibracao ignorada (prato em movimento): offset = 0");
  }
}

static inline float dpsToRPM(float dps) {
  float corrected = dps - gyroOffsetZ;
  float rpm = -(corrected / 6.0f) * RPM_MULTIPLIER;
  return rpm;
}

// Ajusta gyroOffsetZ continuamente enquanto o prato estiver parado.
// Se o dps ficar praticamente constante por AUTO_TRIM_WINDOW_MS, o prato
// esta parado -> anula o residual (evita o timecode derivando).
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
      if (fabsf(meanDps) <= AUTO_TRIM_MAX_ABS_DPS) {
        gyroOffsetZ = meanDps;
      }
    }
    wMin = 1e9f; wMax = -1e9f; wSum = 0.0f; wCount = 0;
    windowStart = millis();
  }
}

static inline void sampleBattery() {
#if BATT_PIN >= 0
  if (millis() - lastBattSampleMillis < BATT_SAMPLE_MS) return;
  lastBattSampleMillis = millis();
  long sumMv = 0;
  for (int i = 0; i < BATT_AVG_SAMPLES; i++) {
    sumMv += analogReadMilliVolts(BATT_PIN);
    delayMicroseconds(100);
  }
  int mv = (int)(sumMv / BATT_AVG_SAMPLES);
  float pct = (float)(mv - BATT_EMPTY_MV) / (float)(BATT_FULL_MV - BATT_EMPTY_MV) * 100.0f;
  batteryLevelPct = (uint8_t)constrain((int)lroundf(pct), 0, 100);
#else
  batteryLevelPct = 100;
#endif
}

// ---------------- BOTAO BOOT: SELETOR DE DECK ----------------

bool bootButtonPrev = false;
uint32_t lastBootButtonMillis = 0;

void toggleDeck() {
  deckId = (deckId + 1) % 3;

  if (deckId == 0) {
    receiverReady = false;
    setOnboardLed(false);
    blinkOnboardLed(3);
    Serial.println("Deck: Sem Deck");
  } else {
    receiverReady = false;
    lastReceiverReplyMillis = 0;
    hopIndex = -1;  // reinicia varredura de canais
    blinkOnboardLed(deckId == 1 ? 1 : 2);
    setOnboardLed(true);
    Serial.println(deckId == 1 ? "Deck: A" : "Deck: B");
  }
}

void handleBootButton() {
  bool pressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
  if (pressed && !bootButtonPrev && (millis() - lastBootButtonMillis > BUTTON_DEBOUNCE_MS)) {
    lastBootButtonMillis = millis();
    toggleDeck();
  }
  bootButtonPrev = pressed;
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) {
    return;
  }

  dvs_packet incoming;
  memcpy(&incoming, dataPtr, sizeof(incoming));

  if (incoming.version != PROTOCOL_VERSION || incoming.deckId != deckId) {
    return;
  }

  if (incoming.msgType == MSG_WELCOME || incoming.msgType == MSG_PING) {
    receiverReady = true;
    lastReceiverReplyMillis = millis();
    setOnboardLed(true);
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.print("MAC ESP32: ");
  Serial.println(WiFi.macAddress());

  esp_err_t setErr = esp_wifi_set_max_tx_power(80); // potencia maxima de TX (20 dBm no C3)
  int8_t txPower = 0;
  esp_wifi_get_max_tx_power(&txPower); // retorna em unidades de 0,25 dBm
#if USE_LONG_RANGE
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#else
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
#endif
  {
    uint8_t proto = 0;
    esp_wifi_get_protocol(WIFI_IF_STA, &proto);
    Serial.printf("RF: setErr=%d power=%.1fdBm LR=%s\n",
                  (int)setErr,
                  txPower / 4.0f,
                  (proto & WIFI_PROTOCOL_LR) ? "ON" : "OFF");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ESP-NOW");
    while (true) blinkOnboardLed(2);
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = activeChannel;  // placeholder; sera atualizado via esp_now_mod_peer apos pareamento
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erro adicionando peer.");
    while (true) blinkOnboardLed(2);
  }

  Serial.println("Peer adicionado.");
  setOnboardLed(false);
}

void sendControlMessage(uint8_t msgType) {
  dvs_packet control = {};
  control.msgType = msgType;
  control.version = PROTOCOL_VERSION;
  control.deckId = deckId;
  control.seq = sequenceNumber;
  control.timestampMicros = micros();

  esp_now_send(receiverMAC, (uint8_t *)&control, sizeof(control));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("================================");
  Serial.println("BMI270 ESP-NOW TRANSMISSOR (S3)");
  Serial.println("================================");

  setupLED();
#if BATT_PIN >= 0
  analogSetPinAttenuation(BATT_PIN, ADC_11db);
#endif
  setupBMI270();
  setupEspNow();
  autoCalibrateGyroZ();
  setOnboardLed(false);
  blinkOnboardLed(1); // anuncia: Deck A
  nextSendMicros = micros();
}

void loop() {
  handleBootButton();

  // Bateria: amostra periodica + alerta no LED azul (pisca sem parar).
  // Verificado antes do guard do deck para funcionar tambem em Sem Deck.
  sampleBattery();
  if (batteryLevelPct < BATT_LOW_THRESHOLD_PCT) {
    setOnboardLed((millis() / BATT_LOW_BLINK_MS) % 2);
  }

  if (deckId != 0 && !receiverReady) {
    // --- PAREAMENTO POR VARREDURA DE CANAIS ---
    // Pula por 1..13 enviando HELLO em cada canal. Quando o RX responde
    // WELCOME, o TX trava nesse canal. Funciona inclusive na reconexao
    // apos perda de sinal (HANDSHAKE_TIMEOUT_MS abaixo).
    uint32_t nowMs = millis();
    if ((hopIndex < 0) || (nowMs - lastHopMillis >= PAIR_CHANNEL_DWELL_MS)) {
      hopIndex = (int16_t)(((int32_t)hopIndex + 1) % (int32_t)pairChannelCount);
      activeChannel = pairChannels[hopIndex];
      esp_now_del_peer(receiverMAC);
      esp_wifi_set_channel(activeChannel, WIFI_SECOND_CHAN_NONE);
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, receiverMAC, 6);
      peerInfo.channel = activeChannel;
      peerInfo.encrypt = false;
      esp_now_add_peer(&peerInfo);
      lastHopMillis = nowMs;
      lastPairHelloMillis = 0;  // forca HELLO imediato no novo canal
      Serial.printf("PAIR: canal %u\n", activeChannel);
    }
    if (nowMs - lastPairHelloMillis >= PAIR_HELLO_INTERVAL_MS) {
      sendControlMessage(MSG_HELLO);
      lastPairHelloMillis = nowMs;
    }
  } else if (deckId != 0 && receiverReady && !wasReceiverReady) {
    // --- PAREAMENTO CONCLUIDO: fixa o peer no canal correto ---
    esp_now_peer_info_t peerUpdate = {};
    memcpy(peerUpdate.peer_addr, receiverMAC, 6);
    peerUpdate.channel = activeChannel;
    peerUpdate.encrypt = false;
    esp_now_mod_peer(&peerUpdate);
    Serial.printf("PAIRED_ON_CH,%u\n", activeChannel);
  }
  wasReceiverReady = (deckId != 0) && receiverReady;

  uint32_t now = micros();
  if (deckId == 0) return;

  if ((int32_t)(now - nextSendMicros) < 0) {
    return;
  }

  nextSendMicros += SEND_INTERVAL_US;
  if ((int32_t)(now - nextSendMicros) > (int32_t)SEND_INTERVAL_US) {
    nextSendMicros = now + SEND_INTERVAL_US;
  }

  float dps = 0.0f;
  if (!readGyroZDps(&dps)) {
    return;
  }

  autoTrimGyroOffset(dps);

  float rpm = dpsToRPM(dps);

  // Filtro assimetrico: ataque rapido em transientes grandes
  // (parada brusca, scratch), release lento em giro estavel.
  float delta = rpm - filteredRPM;
  float alpha = (fabsf(delta) > FAST_THRESHOLD_RPM) ? ALPHA_FAST : ALPHA_SLOW;
  filteredRPM += delta * alpha;

  // Deadzone aplicada apos o filtro, para nao atrasar o
  // cruzamento do threshold quando o prato para de fato.
  if (fabsf(filteredRPM) < DEADZONE_RPM) {
    filteredRPM = 0.0f;
  }

  packet.msgType = MSG_DATA;
  packet.version = PROTOCOL_VERSION;
  packet.deckId = deckId;
  packet.rpmCenti = (int16_t)constrain(lroundf(filteredRPM * 100.0f), -32768, 32767);
  packet.gyroRaw = (int16_t)constrain(lroundf(dps * 10.0f), -32768, 32767);
  sampleBattery();
  packet.batteryPct = batteryLevelPct;
  packet.seq = sequenceNumber++;
  packet.timestampMicros = now;

  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  if (wasReceiverReady && millis() - lastReceiverReplyMillis > HANDSHAKE_TIMEOUT_MS) {
    receiverReady = false;
    wasReceiverReady = false;
    setOnboardLed(false);
    Serial.println("Receptor perdido, reconectando...");
    nextSendMicros = micros();
    hopIndex = -1;  // reinicia varredura de canais
    lastHopMillis = 0;
  }
}

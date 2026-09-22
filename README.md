# Changelog | Release Notes

---

## English

# 1. Visual warning for weak signal/loss

Deck LEDs blink fast when RSSI < -75 dBm or lost packets > 20

LEDs off when a deck has no signal (timeout), solid when OK

2s hold on the warning so it doesn't blink all the time

# 2. DAC test tone

Serial commands TEST\_A, TEST\_B, TEST\_OFF send a 1kHz tone (10s, 25% amplitude) per deck

Allows testing each DAC individually without moving wires

Corresponding buttons in the dashboard

# 3. SOLO\_DECK\_A isolation mode

SOLO\_DECK\_A / SOLO\_DECK\_A\_USA\_PINOS\_B to compile only deck A (one I2S port)

Allows isolating whether the issue is interference between the two I2S ports or faulty hardware

# 4. Dual I2S clock fix (ESP32-S3)

Reinstallation of I2S port 0 after port 1 to avoid clock corruption in the legacy driver

Reordering: i2s\_set\_clk of port 0 last (port 1 last corrupted port 0's clock)

# 5. Expanded telemetry (24 fields)

Added: time since last packet, audio buffers per 0.5s, sample peak, I2S error, test tone status

audioWriteCount delta computed every 0.5s

# 6. Robust debounced stop

STOP\_DEBOUNCE\_MS 250 — debounce before considering it stopped (avoids false stops from jitter)

CALIB\_THRESHOLD\_RPM reduced from 5.0 to 1.0 (detects stop earlier)

# 7. Adaptive RPM smoothing

RPM\_SMOOTHING from 0.12 to 0.25

High alpha (0.8) for deltas > 10 RPM (fast transient response), low (0.25) for small deltas (stability)

# 8. Parallel reception protocol

hasPendingWelcome\[2], pendingWelcomeMac\[2]\[6], etc. arrays — support for two simultaneous decks in the handshake

# 9. Extended timeout

DECK\_TIMEOUT\_MS from 1000 to 1500ms (tolerance to momentary packet loss)

# 10. Deck selector (A / B / No Deck)

Boot button cycles No Deck → Deck A → Deck B → No Deck

Display shows the current state at the top

Automatic handshake when switching decks

Packets are not sent when deck = 0 (stops transmitting)

# 11. Continuous gyro auto-trim

When the platter is stopped for 2 seconds, recalculates the gyro offset

Eliminates timecode drift caused by residual gyro offset at rest

# 12. Recalibrated filters

ALPHA\_SLOW: 0.15 → 0.35 | ALPHA\_FAST: 0.50 → 0.70

FAST\_THRESHOLD\_RPM: 3.0 → 1.5 | DEADZONE\_RPM: 0.15 → 0.5

RPM\_MULTIPLIER: 1.00 → 1.001 (micro-adjust)

# 13. More robust auto-calibration

AUTO\_CALIBRATION\_STABLE\_SAMPLES: 150 → 400

AUTO\_CALIBRATION\_SAMPLE\_DELAY\_MS: 1 → 2

AUTO\_CALIBRATION\_STABLE\_DPS\_DELTA: 2.5 → 2.0

AUTO\_CALIBRATION\_MIN\_COMPARE\_SAMPLES: 10 → 20

Configurable timeout via AUTO\_CALIBRATION\_TIMEOUT\_MS (4000ms)

# 14. Increased send rate

SEND\_RATE\_HZ: 150 → 300 Hz (latency halved)

# 15. DAC test buttons

"Test DAC A", "Test DAC B", "Stop" — send serial commands via WebSerial API

# 16. Bidirectional WebSerial

writer to send commands to the RX (TEST\_A/TEST\_B/TEST\_OFF)

Error handling on send

---

## English — v2.0 Highlights

## ✨ New Features

- **New "General" menu** in the dashboard (sidebar) putting the most-used settings in one place — LED brightness, DAC volume, and deck swapping for the TX boxes.
- **LED brightness control** (0–100%) via slider in the dashboard, using LEDC PWM (8-bit, 5 kHz), persisted in NVS — survives reboots.
- **Per-DAC volume control** (A and B, independent, 0–100%) persisted in NVS — covers both timecode and test tone.
- **Swap TX boxes' decks over ESP-NOW** without opening the boxes — new "TX Configuration" section with per-slot checkboxes.
- **Checkboxes synced** with each box's current deck (confirmed by the TX over ESP-NOW).
- **ESP32 core 3.3.x compatibility** (migrated the old LEDC API to per-pin `ledcAttach`/`ledcWrite`).
- **Brightness fallback**: if PWM fails, LEDs still work over digital GPIO (with `LED_PWM_A/B` boot diagnostics).
- **Aligned dashboard**: menus (Calibration, Audio Tests, WiFi, and General) and telemetry centered at 1400 px; log hidden on General and Audio Tests menus.
- **Brightness and volume auto-loaded** on Web Serial connect (`LED_BRIGHT` and `VOLUME`).

## 🔧 Fixes

- Fixed compile error (extra brace in the volume block).
- Fixed LEDs staying off after reflashing (cause: `ledcAttach` failing silently).

---

## Português

# 1. Alerta visual de sinal fraco/perda

LEDs dos decks piscam rápido quando RSSI < -75 dBm ou pacotes perdidos > 20

LEDs apagados quando deck sem sinal (timeout), sólidos quando OK

Hold de 2s no aviso para não ficar piscando o tempo todo

# 2. Tom de teste de DAC

Comandos seriais TEST\_A, TEST\_B, TEST\_OFF enviam tom de 1kHz (10s, 25% amplitude) por deck

Permite testar cada DAC individualmente sem mover fios

Botões correspondentes no dashboard

# 3. Modo isolamento SOLO\_DECK\_A

SOLO\_DECK\_A / SOLO\_DECK\_A\_USA\_PINOS\_B para compilar apenas o deck A (uma porta I2S)

Permite isolar se o problema é interferência entre as duas portas I2S ou hardware defeituoso

# 4. Correção do clock I2S duplo (ESP32-S3)

Reinstalação da porta I2S 0 após a porta 1 para corrompimento de clock no driver legado

Reordenação: i2s\_set\_clk da porta 0 por último (a porta 1 por último corrompia o clock da 0)

# 5. Telemetria expandida (24 campos)

Adicionados: tempo desde último pacote, buffers de áudio por 0.5s, pico de amostra, erro I2S, status do tom de teste

Cálculo de delta de audioWriteCount a cada 0.5s

# 6. Parada robusta com debounce

STOP\_DEBOUNCE\_MS 250 — debounce antes de considerar parado (evita falsas paradas por jitter)

CALIB\_THRESHOLD\_RPM reduzido de 5.0 para 1.0 (detecta parada mais cedo)

# 7. Suavização de RPM adaptativa

RPM\_SMOOTHING de 0.12 para 0.25

Alpha alto (0.8) para deltas > 10 RPM (resposta rápida em transientes), baixo (0.25) para deltas pequenos (estabilidade)

# 8. Protocolo de recepção paralelo

Arrays de hasPendingWelcome\[2], pendingWelcomeMac\[2]\[6], etc. — suporte a dois decks simultâneos no handshake

# 9. Timeout estendido

DECK\_TIMEOUT\_MS de 1000 para 1500ms (tolerância a perda momentânea de pacotes)

# 10. Seletor de Deck (A / B / Sem Deck)

Botão boot alterna entre Sem Deck → Deck A → Deck B → Sem Deck

Display mostra o estado atual no topo

Handshake automático ao trocar de deck

Envio de pacotes para = 0 não acontece (para de transmitir)

# 11. Auto-trim contínuo do giroscópio

Quando o prato está parado por 2 segundos, recalcula o offset do giroscópio

Elimina drift do timecode causado por residual do giroscópio parado

# 12. Filtros recalibrados

ALPHA\_SLOW: 0.15 → 0.35 | ALPHA\_FAST: 0.50 → 0.70

FAST\_THRESHOLD\_RPM: 3.0 → 1.5 | DEADZONE\_RPM: 0.15 → 0.5

RPM\_MULTIPLIER: 1.00 → 1.001 (micro-ajuste)

# 13. Calibração automática mais robusta

AUTO\_CALIBRATION\_STABLE\_SAMPLES: 150 → 400

AUTO\_CALIBRATION\_SAMPLE\_DELAY\_MS: 1 → 2

AUTO\_CALIBRATION\_STABLE\_DPS\_DELTA: 2.5 → 2.0

AUTO\_CALIBRATION\_MIN\_COMPARE\_SAMPLES: 10 → 20

Timeout configurável via AUTO\_CALIBRATION\_TIMEOUT\_MS (4000ms)

# 14. Taxa de envio aumentada

SEND\_RATE\_HZ: 150 → 300 Hz (latência reduzida pela metade)

# 15. Botões de teste de DAC

"Testar DAC A", "Testar DAC B", "Parar" — enviam comandos seriais via WebSerial API

# 16. WebSerial bidirecional

writer para enviar comandos ao RX (TEST\_A/TEST\_B/TEST\_OFF)

Tratamento de erro no envio

---

## Português — Destaques da v2.0

## ✨ Novidades

- **Novo menu "Geral"** no dashboard (sidebar) com as configurações mais usadas em um só lugar — brilho dos LEDs, volume dos DACs e troca de deck das caixas.
- **Controle de brilho dos LEDs** (0–100%) via slider no dashboard, com PWM por LEDC (8 bits, 5 kHz) e persistência em NVS — sobrevive a reinicializações.
- **Controle de volume por DAC** (A e B, independentes, 0–100%) com persistência em NVS — cobre timecode e tom de teste.
- **Troca de deck das caixas TX via ESP-NOW** sem abrir as caixas — nova seção "Configuração TX" com checkbox por slot.
- **Checkboxes sincronizados** com o deck atual de cada caixa (confirmado pelo TX via ESP-NOW).
- **Compatibilidade com ESP32 core 3.3.x** (migração da API LEDC antiga para `ledcAttach`/`ledcWrite` por pino).
- **Fallback de brilho**: se o PWM falhar, os LEDs funcionam via GPIO digital (com diagnóstico no boot `LED_PWM_A/B`).
- **Dashboard alinhado**: menus (Calibração, Testes de Áudio, WiFi e Geral) e telemetria centralizados a 1400 px; log oculto nos menus Geral e Testes de Áudio.
- **Brilho e volume lidos automaticamente** ao conectar no Web Serial (`LED_BRIGHT` e `VOLUME`).

## 🔧 Correções

- Erro de compilação corrigido (chave extra no bloco de volume).
- LEDs que ficavam apagados após reflash corrigidos (causa: `ledcAttach` falhando silenciosamente).
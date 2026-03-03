# p4-edge-vision

Firmware ESP32-P4 pour la partie vision et agrégation des événements capteurs.

## Rôle dans la chaîne IoT

1. Lance la détection vision (`app_vision_event_start`).
2. Récupère la température depuis le C6 (format texte `TEMP,<valeur>` sur UART).
3. Emballe les données en trames UART v1 (16 octets + CRC) via `protocol_uart_v1.h`.
4. Envoie ces trames vers le bridge LoRa (`uplink-lorawan`) sur UART.

## Fichiers importants

- `firmware/esp32p4_eye/vision_event/main/main.c`: orchestration principale (vision + UART C6 + émission UART v1).
- `firmware/esp32p4_eye/vision_event/main/protocol_uart_v1.h`: format de trame commun.

## Build rapide (ESP-IDF)

Le projet applicatif principal est dans:
`firmware/esp32p4_eye/vision_event`

```bash
cd firmware/esp32p4_eye/vision_event
. $HOME/esp/esp-idf/export.sh
idf.py set-target esp32p4
idf.py build
idf.py -p <PORT_P4> flash monitor
```

## Câblage utile

- `C6 GPIO16 (TX)` -> `P4 GPIO34 (RX)` (par défaut, selon la config UART HCI/NimBLE).
- `P4 GPIO37 (TX)` -> entrée UART du bridge LoRa STM32 (`PA10 / D2`).
- Masse commune entre cartes.

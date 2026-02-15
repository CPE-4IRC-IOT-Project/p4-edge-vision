# ESP32-P4-EYE - Architecture Événementielle avec Files FreeRTOS

## Vue d'ensemble

Cette implémentation utilise une **architecture événementielle basée sur des files (queues) FreeRTOS**, inspirée du livre *"Developing IoT Projects with ESP32"* (Packt Publishing, 2nd edition).

Le système est divisé en **4 tâches indépendantes** qui communiquent via des files, permettant une séparation claire des responsabilités et une meilleure scalabilité.

## Architecture

```
┌──────────────┐     Queue 1      ┌─────────────────┐     Queue 2      ┌──────────────┐     Queue 3      ┌────────────┐
│              │  camera_frame_   │                 │  detection_      │              │  count_event_   │            │
│ CameraTask   ├─────────────────►│ DetectionTask   ├─────────────────►│ CountingTask ├─────────────────►│ UARTTask   │
│              │    event_t       │                 │   result_event_t*│              │       _t        │            │
│ (Core 0)     │                  │   (Core 1)      │                  │  (Core 0)    │                 │ (Core 0)   │
│ Priority: 5  │                  │   Priority: 4   │                  │ Priority: 3  │                 │Priority: 2 │
└──────────────┘                  └─────────────────┘                  └──────────────┘                 └────────────┘
```

## Tâches

### 1. CameraTask (`camera_task.cpp`)
**Responsabilités:**
- Initialiser la caméra MIPI-CSI via BSP
- Capturer les frames en continu (1920x1080 RGB565)
- Envoyer les frames vers la queue de détection

**Priorité:** 5 (la plus haute)  
**Core:** 0  
**Période:** 200ms (5 FPS configurable)

### 2. DetectionTask (`detection_task.cpp`)
**Responsabilités:**
- Recevoir les frames de la queue caméra
- Redimensionner les images (1920x1080 → 224x224 RGB888)
- Exécuter l'inférence avec le modèle PedestrianDetect (esp-dl)
- Extraire les bounding boxes et scores de confiance
- Envoyer les résultats vers la queue de comptage

**Priorité:** 4  
**Core:** 1 (séparé de la caméra pour parallélisme)  
**Modèle:** PedestrianDetect (esp-dl human face detection)

### 3. CountingTask (`counting_task.cpp`)
**Responsabilités:**
- Recevoir les résultats de détection
- Compter le nombre de personnes détectées
- Détecter les changements significatifs (±1 personne)
- Gérer le heartbeat périodique (120s par défaut)
- Gérer les alarmes (seuil de personnes dépassé)
- Envoyer les événements vers la queue UART

**Priorité:** 3  
**Core:** 0  

**Logique événementielle:**
- **Changement:** Envoi immédiat si `|count_actuel - count_précédent| >= 1`
- **Heartbeat:** Envoi périodique toutes les 2 minutes (configurable)
- **Alarme:** Envoi si `count >= seuil` (10 par défaut)

### 4. UARTTask (`uart_task.cpp`)
**Responsabilités:**
- Recevoir les événements de comptage
- Construire les trames binaires selon `protocol_uart.h`
- Calculer le CRC-16-CCITT
- Envoyer les trames (16 octets) vers le STM32 via UART

**Priorité:** 2 (la plus basse)  
**Core:** 0  
**Protocole:** Voir `protocol_uart.h` et `PROTOCOL_UART_README.md`

## Files (Queues)

| Queue | Taille | Type | Description |
|-------|--------|------|-------------|
| `camera_to_detection` | 2 | `camera_frame_event_t` | Frames capturées (pointeurs) |
| `detection_to_counting` | 5 | `detection_result_event_t*` | Résultats d'inférence (pointeurs) |
| `counting_to_uart` | 10 | `count_event_t` | Événements de comptage |

## Structures de données (`event_types.h`)

### camera_frame_event_t
```cpp
typedef struct {
    uint16_t* frame_data;      // Pointeur vers RGB565 (1920x1080)
    int width, height;
    int64_t timestamp_us;
    uint32_t frame_id;
} camera_frame_event_t;
```

### detection_result_event_t
```cpp
typedef struct {
    uint32_t frame_id;
    int64_t timestamp_us;
    int people_count;
    detected_person_t* persons;     // Tableau dynamique
    int max_persons;
    float avg_confidence;
    int inference_time_ms;
} detection_result_event_t;
```

### count_event_t
```cpp
typedef struct {
    count_event_type_t type;        // HEARTBEAT, CHANGE, ALARM, SYSTEM_STATUS
    uint8_t count;
    uint8_t confidence;
    uint8_t flags;                  // DAYLIGHT, MOTION, ALARM, LOW_BATTERY
    uint32_t timestamp;
    uint8_t node_id;
    uint16_t counter;               // Anti-rejeu
} count_event_t;
```

## Protocole UART

Les événements sont transmis au STM32 sous forme de **trames binaires de 16 octets** :

```
| SOF | LEN | TYPE | NODE_ID | TIMESTAMP | EVENT_ID | COUNT | CONF | FLAGS | CNT | CRC |
  1     1     1       1          4           1         1       1      1       2     2
```

**Types d'événements:**
- `0x10` : `EVENT_PERSON_COUNT` (heartbeat)
- `0x11` : `EVENT_PERSON_COUNT_CHANGE` (changement détecté)
- `0x20` : `EVENT_ALARM` (alarme)
- `0x30` : `EVENT_SYSTEM_STATUS` (statut système)

Voir `protocol_uart.h` et `PROTOCOL_UART_README.md` pour les détails complets.

## Configuration

Tous les paramètres sont configurables dans `main.cpp` :

```cpp
// Caméra
#define CAM_CAPTURE_PERIOD_MS   200     // 5 FPS

// Modèle
#define MODEL_WIDTH             224
#define MODEL_HEIGHT            224

// UART
#define UART_PORT_NUM           UART_NUM_0
#define UART_BAUD_RATE          115200
#define NODE_ID                 0x01

// Détection
#define DETECTION_CONFIDENCE    0.5f
#define HEARTBEAT_PERIOD_S      120
#define ALARM_THRESHOLD         10

// Queues
#define QUEUE_CAMERA_SIZE       2
#define QUEUE_DETECTION_SIZE    5
#define QUEUE_COUNTING_SIZE     10
```

## Compilation

```bash
cd p4-edge-vision
idf.py build
idf.py flash monitor
```

## Avantages de cette architecture

### ✅ Séparation des responsabilités
Chaque tâche a un rôle bien défini et peut être testée/modifiée indépendamment.

### ✅ Scalabilité
Facile d'ajouter de nouvelles tâches (ex: display task, WiFi task, etc.)

### ✅ Performance
- Parallélisme : Caméra sur Core 0, Détection sur Core 1
- Files non-bloquantes : pas de perte de frames si une tâche est lente

### ✅ Robustesse
- Gestion des débordements de queue (logs + drop gracieux)
- Allocation/libération mémoire explicite
- Timeouts sur les opérations bloquantes

### ✅ Maintenance
- Code modulaire et documenté
- Logging détaillé pour le debugging
- Configuration centralisée

## Monitoring

Le système affiche des statistiques toutes les 30 secondes :

```
Queue status: Camera=1/2, Detection=3/5, Counting=2/10
Free heap: 234568 bytes, Free PSRAM: 8123456 bytes
```

## Références

- **Architecture inspirée de:** *Developing IoT Projects with ESP32* (2nd edition, Packt Publishing)  
  GitHub: https://github.com/PacktPublishing/Developing-IoT-Projects-with-ESP32-2nd-edition

- **Modèle de détection:** ESP-DL Human Face Detection  
  GitHub: https://github.com/espressif/esp-dl/tree/master/examples/human_face_detect

- **Protocole UART:** Voir `protocol_uart.h` et `PROTOCOL_UART_README.md`

## Logs typiques

```
I (1234) main: ESP32-P4-EYE - Event-Driven Architecture
I (1235) main: Camera: 1920x1080 @ 200ms
I (1236) main: Model: 224x224
I (1240) camera_task: Camera task started
I (1245) camera_task: BSP camera initialized
I (1250) detection_task: Detection task started
I (1255) detection_task: Loading PedestrianDetect model...
I (2100) detection_task: Model loaded successfully
I (2105) counting_task: Counting task started
I (2110) uart_task: UART task started
I (2115) uart_task: UART0 configured: 115200 baud, 8N1
I (2500) detection_task: Frame 0: detected 2 person(s), conf=0.87, resize=45ms, infer=120ms, total=165ms
I (2505) counting_task: Received detection result: count=2, confidence=87%
I (2510) counting_task: Count changed: 0 -> 2 (diff=2)
I (2515) uart_task: Sent UART frame: type=0x11, count=2, conf=87%, flags=0x01, ctr=0
```

## TODO / Améliorations futures

- [ ] Ajouter une tâche LCD pour afficher les résultats en temps réel
- [ ] Implémenter la détection jour/nuit (capteur luminosité)
- [ ] Ajouter le tracking de personnes (ID persistants)
- [ ] Implémenter la détection de mouvement (FLAG_MOTION)
- [ ] Ajouter WiFi/MQTT pour telemetry
- [ ] Configuration via NVS (paramètres persistants)
- [ ] OTA updates

## Auteur

Architecture implémentée pour le projet IOT CPE Lyon S8 2026.

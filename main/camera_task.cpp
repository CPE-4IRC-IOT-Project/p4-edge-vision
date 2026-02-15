/**
 * @file camera_task.cpp
 * @brief Implémentation de la tâche de capture caméra
 */

#include "camera_task.h"
#include "event_types.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

#include "bsp/esp32_p4_eye.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_video_device.h"
#include "linux/videodev2.h"

static const char* TAG = "camera_task";

// Configuration V4L2
#define NUM_BUFFERS 2

/**
 * @brief Configure le format de capture V4L2
 */
static int cam_set_format(int fd, int width, int height, uint32_t pixelformat)
{
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixelformat;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        ESP_LOGE(TAG, "VIDIOC_S_FMT failed");
        return -1;
    }
    ESP_LOGI(TAG, "Format set: %dx%d", width, height);
    return 0;
}

/**
 * @brief Demande l'allocation de buffers V4L2
 */
static int cam_request_buffers(int fd)
{
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = NUM_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0)
    {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
        return -1;
    }
    return 0;
}

/**
 * @brief Enqueue les buffers V4L2 pour la capture
 */
static int cam_queue_buffers(int fd)
{
    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
        {
            ESP_LOGE(TAG, "VIDIOC_QBUF[%d] failed", i);
            return -1;
        }
    }
    return 0;
}

/**
 * @brief Démarre le streaming V4L2
 */
static int cam_stream_on(int fd)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed");
        return -1;
    }
    return 0;
}

/**
 * @brief Point d'entrée de la tâche caméra
 */
void camera_task(void* pvParameters)
{
    system_config_t* config = (system_config_t*)pvParameters;
    ESP_LOGI(TAG, "Camera task started");

    // Initialiser la caméra via BSP
    bsp_camera_cfg_t cam_cfg = {};
    ESP_ERROR_CHECK(bsp_camera_start(&cam_cfg));
    ESP_LOGI(TAG, "BSP camera initialized");

    // Ouvrir le device V4L2
    int cam_fd = open(BSP_CAMERA_DEVICE, O_RDONLY);
    if (cam_fd < 0)
    {
        ESP_LOGE(TAG, "Cannot open %s", BSP_CAMERA_DEVICE);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Opened %s (fd=%d)", BSP_CAMERA_DEVICE, cam_fd);

    // Déclarer frame_id avant les gotos (problème C++ avec goto)
    uint32_t frame_id = 0;

    // Configurer la capture
    if (cam_set_format(cam_fd, config->camera_width, config->camera_height, 
                       config->camera_pixel_format) < 0)
        goto cleanup;
    if (cam_request_buffers(cam_fd) < 0)
        goto cleanup;
    if (cam_queue_buffers(cam_fd) < 0)
        goto cleanup;
    if (cam_stream_on(cam_fd) < 0)
        goto cleanup;

    ESP_LOGI(TAG, "Camera streaming started");

    // Boucle de capture
    while (true)
    {
        // Attendre une frame
        struct v4l2_buffer vbuf;
        memset(&vbuf, 0, sizeof(vbuf));
        vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(cam_fd, VIDIOC_DQBUF, &vbuf) < 0)
        {
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Créer l'événement de frame
        camera_frame_event_t frame_event = {
            .frame_data = (uint16_t*)vbuf.m.userptr,
            .width = config->camera_width,
            .height = config->camera_height,
            .timestamp_us = esp_timer_get_time(),
            .frame_id = frame_id++
        };

        // Envoyer vers la queue de détection (non-bloquant)
        if (xQueueSend(config->camera_to_detection_queue, &frame_event, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Detection queue full, dropping frame %lu", (unsigned long)frame_event.frame_id);
        }

        // Re-queue le buffer pour la prochaine capture
        if (ioctl(cam_fd, VIDIOC_QBUF, &vbuf) < 0)
        {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed");
        }

        // Attendre avant la prochaine capture
        vTaskDelay(pdMS_TO_TICKS(config->camera_capture_period_ms));
    }

cleanup:
    close(cam_fd);
    ESP_LOGE(TAG, "Camera task exiting");
    vTaskDelete(NULL);
}

/**
 * @brief Créer et démarrer la tâche caméra
 */
TaskHandle_t camera_task_start(system_config_t* config)
{
    TaskHandle_t task_handle = NULL;
    
    BaseType_t ret = xTaskCreatePinnedToCore(
        camera_task,
        "camera_task",
        8192,               // Stack size
        config,             // Parameters
        5,                  // Priority
        &task_handle,
        0                   // Core 0
    );

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create camera task");
        return NULL;
    }

    ESP_LOGI(TAG, "Camera task created");
    return task_handle;
}

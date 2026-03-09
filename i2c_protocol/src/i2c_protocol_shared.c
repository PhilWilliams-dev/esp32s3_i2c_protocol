#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "esp_timer.h"
#include "driver/i2c_types.h"
#include "i2c_protocol_shared.h"


// Asynchronous debug logging system
// Controlled by ASYNC_DEBUG_ENABLED in i2c_protocol_shared.h
#ifndef ASYNC_DEBUG_ENABLED
#define ASYNC_DEBUG_ENABLED 0
#endif

#if ASYNC_DEBUG_ENABLED

#define ASYNC_LOG_QUEUE_SIZE 1000
#define ASYNC_LOG_MAX_MESSAGE 128

typedef struct {
    uint64_t timestamp;
    uint32_t sequence;
    char tag[16];
    char message[ASYNC_LOG_MAX_MESSAGE];
} async_log_entry_t;

static QueueHandle_t async_log_queue = NULL;
static TaskHandle_t async_log_task_handle = NULL;
static uint32_t async_log_sequence = 0;

/// @brief Background task that drains the async log queue and prints entries via ets_printf.
/// @param arg Unused (required by FreeRTOS task signature).
static void async_log_task(void *arg) {
    async_log_entry_t entry;
    
    while (1) {
        if (xQueueReceive(async_log_queue, &entry, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Uses ets_printf (ROM function, lock-free) to avoid spinlock conflicts with ISRs
            uint32_t current_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
            uint32_t entry_time_ms = (uint32_t)(entry.timestamp / 1000);
            ets_printf("I (%lu) %s: [%lu:%lu] %s\n", 
                       current_time_ms,
                       entry.tag, 
                       entry_time_ms,
                       entry.sequence, 
                       entry.message);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/// @brief Creates the log queue and spawns the background print task. Safe to call multiple times.
void async_log_init(void) {
    if (async_log_queue != NULL) {
        return;
    }
    
    async_log_queue = xQueueCreate(ASYNC_LOG_QUEUE_SIZE, sizeof(async_log_entry_t));
    if (async_log_queue == NULL) {
        ESP_LOGE("ASYNC_LOG", "Failed to create async log queue");
        return;
    }
    
    BaseType_t task_created = xTaskCreatePinnedToCore(
        async_log_task,
        "async_log",
        2048,
        NULL,
        0,  // Idle priority to avoid ISR conflicts
        &async_log_task_handle,
        1   // Run on core 1 if dual core
    );
    
    if (task_created != pdTRUE) {
        ESP_LOGE("ASYNC_LOG", "Failed to create async log task");
    } else {
        ESP_LOGI("ASYNC_LOG", "Async logging initialized");
    }
}

/// @brief Queues a formatted log message for background output. Non-blocking; drops if queue is full.
/// @param tag Short identifier shown in the log line.
/// @param format printf-style format string, followed by variadic arguments.
void async_log(const char* tag, const char* format, ...) {
    if (async_log_queue != NULL) {
        async_log_entry_t entry;
        entry.timestamp = esp_timer_get_time();
        entry.sequence = async_log_sequence++;
        strncpy(entry.tag, tag, sizeof(entry.tag) - 1);
        entry.tag[sizeof(entry.tag) - 1] = '\0';
        
        va_list args;
        va_start(args, format);
        vsnprintf(entry.message, sizeof(entry.message), format, args);
        va_end(args);
        
        xQueueSend(async_log_queue, &entry, 0); /* Non-blocking, drops if queue full */
    }
}

/// @brief Queues a hex dump of a byte buffer for background output.
/// @param tag Short identifier shown in the log line.
/// @param prefix Optional text prepended to the hex string (may be NULL).
/// @param data Byte array to dump (may be NULL).
/// @param len Number of bytes to dump.
void async_log_hex(const char* tag, const char* prefix, const uint8_t* data, size_t len) {
    if (async_log_queue == NULL) {
        return;
    }

    if (data == NULL || len == 0) {
        async_log(tag, "%s<empty>", (prefix != NULL) ? prefix : "");
        return;
    }

    char buf[ASYNC_LOG_MAX_MESSAGE];
    size_t pos = 0;

    if (prefix != NULL && prefix[0] != '\0') {
        pos = (size_t)snprintf(buf, sizeof(buf), "%s", prefix);
        if (pos >= sizeof(buf)) {
            buf[sizeof(buf) - 1] = '\0';
            async_log(tag, "%s", buf);
            return;
        }
    }

    for (size_t i = 0; i < len && pos + 2 < sizeof(buf); ++i) {
        int written = snprintf(&buf[pos], sizeof(buf) - pos, "%02X", data[i]);
        if (written <= 0) {
            break;
        }
        pos += (size_t)written;
    }

    async_log(tag, "%s", buf);
}

#else
// No-op stubs when async logging is disabled
void async_log_init(void) {}
void async_log(const char* tag, const char* format, ...) {}
void async_log_hex(const char* tag, const char* prefix, const uint8_t* data, size_t len) {}
#endif


/// @brief Returns the number of chunks needed to send the given byte count.
/// @param bytes Total payload size in bytes.
/// @return Number of PACKET_PAYLOAD_SIZE chunks required (rounded up).
uint16_t bytes_to_chunks(uint32_t bytes){
    return (bytes + PACKET_PAYLOAD_SIZE - 1) / PACKET_PAYLOAD_SIZE;
}


/// @brief Returns the maximum byte capacity for the given chunk count.
/// @param chunks Number of chunks.
/// @return Maximum byte count (chunks * PACKET_PAYLOAD_SIZE).
uint32_t chunks_to_bytes(uint16_t chunks){
    return chunks * PACKET_PAYLOAD_SIZE;
}

/// @brief Checks if a byte array contains a null terminator within the given length.
/// @param data Input byte array to check (may be NULL).
/// @param length Number of bytes to scan.
/// @return true if null-terminated within length, false otherwise.
bool is_null_terminated(const uint8_t* data, size_t length) {
    if (data == NULL || length == 0) {
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        if (data[i] == '\0') {
            return true;
        }
    }
    return false;
}


/// @brief Allocates a new null-terminated string from a byte array.
///        Copies up to the first null or full length, appends a terminator if needed.
///        Caller must free the result.
/// @param data Input byte array (may be NULL).
/// @param length Number of bytes to consider.
/// @return Null-terminated char* on success (must be freed), or NULL on failure.
char* make_null_terminated_str(const uint8_t* data, size_t length) {
    if (data == NULL || length == 0) {
        return NULL;
    }

    size_t effective_len = 0;
    bool needs_terminator = true;
    for (effective_len = 0; effective_len < length; effective_len++) {
        if (data[effective_len] == '\0') {
            needs_terminator = false;
            break;
        }
    }

    size_t alloc_size = effective_len + (needs_terminator ? 1 : 0) + 1;
    char* result = (char*)malloc(alloc_size);
    if (result == NULL) {
        return NULL;
    }

    memcpy(result, data, effective_len);
    result[effective_len] = '\0';

    return result;
}

/// @brief Assembles 2 bytes into a 16-bit integer respecting endianness.
/// @param bytes 2-byte array to convert.
/// @param is_big_endian Endian format selector.
/// @return The reconstructed 16-bit integer.
uint16_t bytes_to_integer(uint8_t *bytes, int is_big_endian) {
    if (is_big_endian) {
        return (bytes[0] << 8) | bytes[1]; // MSB first
    } else {
        return (bytes[1] << 8) | bytes[0]; // LSB first
    }
}

/// @brief Splits a 16-bit integer into 2 bytes respecting endianness.
/// @param value The 16-bit integer to split.
/// @param bytes Output array to store the 2 bytes.
/// @param is_big_endian Endian format selector.
void integer_to_bytes(uint16_t value, uint8_t *bytes, int is_big_endian) {
    if (is_big_endian) {
        bytes[0] = (value >> 8) & 0xFF;  // MSB first
        bytes[1] = value & 0xFF;         // LSB second
    } else {
        bytes[0] = value & 0xFF;         // LSB first
        bytes[1] = (value >> 8) & 0xFF;  // MSB second
    }
}

/// @brief Calculates a CRC-16-CCITT checksum over the given data.
/// @param data Data to checksum.
/// @param len Length of data.
/// @return CRC-16 value.
uint16_t calculate_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    const uint16_t poly = 0x1021;
    for (size_t i = 0; i < len; i++) {
        crc ^= (data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/// @brief Builds a transmit byte array from a packet structure, including CRC.
///        For CMD_START_CHUNKING the length field carries the chunk count, so
///        the payload is zeroed and the size check is skipped.
/// @param packet Source packet.
/// @param buffer Output buffer (must be at least PACKET_TOTAL_SIZE bytes).
void assemble_transmit_buffer(data_packet_t *packet, uint8_t *buffer){
    
    bool ignoresizecheck = false;
    
    // Header: command (1) + reply (1) + length (2, little-endian)
    buffer[0] = packet->command;
    buffer[1] = packet->reply;
    buffer[2] = packet->length & 0xFF;
    buffer[3] = (packet->length >> 8) & 0xFF;

    if(packet->command == CMD_START_CHUNKING) ignoresizecheck = true;

    memset(&buffer[PACKET_HEADER_SIZE], 0, PACKET_PAYLOAD_SIZE);
    
    if(packet->length > 0 && ignoresizecheck == false){
        memcpy(&buffer[PACKET_HEADER_SIZE], packet->data, packet->length);
    }
   
    uint16_t crc = calculate_crc16(buffer, PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE);

    buffer[PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE] = crc & 0xFF;
    buffer[PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE + 1] = (crc >> 8) & 0xFF;

}

/// @brief Decodes a received byte array into a packet structure with CRC validation.
///        For CMD_START_CHUNKING the length field carries the chunk count, so
///        the payload size check is skipped.
/// @param buffer The raw received bytes.
/// @param buffer_length Size of the buffer.
/// @param packet Output packet (zeroed first to prevent stale data).
/// @return ESP_OK on success, or an appropriate error code.
esp_err_t decode_received_packet(const uint8_t *buffer, size_t buffer_length, data_packet_t *packet) {

    if (buffer_length < PACKET_HEADER_SIZE + PACKET_CRC_SIZE || buffer_length > PACKET_TOTAL_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(packet, 0, sizeof(data_packet_t));

    bool ignoresizecheck = false;

    packet->command = buffer[0];
    packet->reply = buffer[1];

    if(packet->command == CMD_START_CHUNKING) ignoresizecheck = true;

    uint8_t len_bytes[2] = {buffer[2], buffer[3]};
    uint16_t extracted_length = bytes_to_integer(len_bytes, 0);
    
    // CMD_OK with zero length is a lightweight acknowledgement - skip CRC
    if(packet->command == CMD_OK && extracted_length == 0) {
        packet->length = 0;
        return ESP_OK;
    }
    
    packet->length = extracted_length;

    if(ignoresizecheck == false){
        if (packet->length > PACKET_PAYLOAD_SIZE) {
            return ESP_ERR_INVALID_STATE;
        }

        if ((buffer_length < PACKET_HEADER_SIZE + packet->length + PACKET_CRC_SIZE)) {
            return ESP_ERR_INVALID_SIZE;
        }

        memcpy(packet->data, &buffer[PACKET_HEADER_SIZE], packet->length);
        
        if(packet->length < PACKET_PAYLOAD_SIZE) {
            memset(packet->data + packet->length, 0, PACKET_PAYLOAD_SIZE - packet->length);
        }
    }

    uint8_t crc_bytes[2] = {buffer[PACKET_TOTAL_SIZE - 2], buffer[PACKET_TOTAL_SIZE - 1]};
    uint16_t received_crc = bytes_to_integer(crc_bytes, 0);
    
    uint16_t calc_crc = calculate_crc16(buffer, PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE);

    if (calc_crc != received_crc) {
        return ESP_ERR_INVALID_CRC;
    }
    packet->crc = received_crc;
    
    return ESP_OK;
}

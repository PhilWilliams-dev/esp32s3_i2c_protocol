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
#include "driver/i2c_slave.h"
#include "driver/i2c_types.h"
#include "i2c_protocol_shared.h"


//Logging Tags
static const char *DECODELOG = "Packet Decode"; 

// Asynchronous debug logging system
// ASYNC_DEBUG_ENABLED is now defined in i2c_protocol_shared.h
// Set it to 0 there to disable async logging and use ESP_LOGI/ESP_LOGE instead
#ifndef ASYNC_DEBUG_ENABLED
#define ASYNC_DEBUG_ENABLED 0  // Default to enabled if not defined in header
#endif

#if ASYNC_DEBUG_ENABLED

#define ASYNC_LOG_QUEUE_SIZE 1000
#define ASYNC_LOG_MAX_MESSAGE 128

typedef struct {
    uint64_t timestamp;        // Microseconds since boot
    uint32_t sequence;         // Sequence number for ordering
    char tag[16];              // Log tag
    char message[ASYNC_LOG_MAX_MESSAGE];
} async_log_entry_t;

static QueueHandle_t async_log_queue = NULL;
static TaskHandle_t async_log_task_handle = NULL;
static uint32_t async_log_sequence = 0;

// Async log task that processes the queue
static void async_log_task(void *arg) {
    async_log_entry_t entry;
    
    while (1) {
        // Try to receive from queue with timeout
        if (xQueueReceive(async_log_queue, &entry, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Log the entry using ets_printf (ROM function, lock-free) to avoid spinlock conflicts with ISRs
            uint32_t current_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
            uint32_t entry_time_ms = (uint32_t)(entry.timestamp / 1000);
            ets_printf("I (%lu) %s: [%lu:%lu] %s\n", 
                       current_time_ms,
                       entry.tag, 
                       entry_time_ms,
                       entry.sequence, 
                       entry.message);
        }
        
        // Small delay to prevent CPU spinning
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Initialize async logging
void async_log_init(void) {
    // Only initialize once (check if already initialized)
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
        0,  // Idle priority - only runs when nothing else is running, avoids ISR conflicts
        &async_log_task_handle,
        1   // Run on core 1 if dual core
    );
    
    if (task_created != pdTRUE) {
        ESP_LOGE("ASYNC_LOG", "Failed to create async log task");
    } else {
        ESP_LOGI("ASYNC_LOG", "Async logging initialized");
    }
}

// Helper function for async logging (public function)
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
        
        xQueueSend(async_log_queue, &entry, 0); /* Non-blocking - drop if queue full */
    }
}

// Helper to log a buffer as hex via the async logger
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
// If disabled, make it a no-op
void async_log_init(void) {}
void async_log(const char* tag, const char* format, ...) {}
void async_log_hex(const char* tag, const char* prefix, const uint8_t* data, size_t len) {}
#endif


/// @brief Takes the number of bytes that will be sent and gives you how many chunks are needed to send that
/// @param bytes 
/// @return number of chunks needed
uint16_t bytes_to_chunks(uint32_t bytes){
    return (bytes + PACKET_PAYLOAD_SIZE - 1) / PACKET_PAYLOAD_SIZE;
}


/// @brief Takes the number of chunks and gives you back the maximun number of bytes needed to hold that data
/// @param chunks 
/// @return bytes
uint32_t chunks_to_bytes(uint16_t chunks){
    return chunks * PACKET_PAYLOAD_SIZE;
}

///@brief Checks if a byte array contains a null terminator ('\0') within the given length.
///        Returns true if already null-terminated (valid C string), false otherwise.
///@param data Input byte array to check. Can be NULL.
///@param length Length of the input data (bounds the check).
///@return true if null-terminated within length, false otherwise (including for NULL/zero-length input).
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


/**
 * @brief Allocates a new null-terminated string from the input byte array.
 *        Copies up to the first null or full length, adds a null terminator if needed.
 *        Caller must free the result.
 * @param data Input byte array (assumed to contain string data). Can be NULL.
 * @param length Length of the input data (required to bound the operation).
 * @return Null-terminated char* on success (must be freed), or NULL on failure/invalid input.
 */
char* make_null_terminated_str(const uint8_t* data, size_t length) {
    if (data == NULL || length == 0) {
        return NULL;
    }

    // Find effective length (up to first null if present)
    size_t effective_len = 0;
    bool needs_terminator = true;
    for (effective_len = 0; effective_len < length; effective_len++) {
        if (data[effective_len] == '\0') {
            needs_terminator = false;
            break;
        }
    }

    // Allocate space (effective_len + 1 for terminator if needed)
    size_t alloc_size = effective_len + (needs_terminator ? 1 : 0) + 1;  // +1 for safety
    char* result = (char*)malloc(alloc_size);
    if (result == NULL) {
        ESP_LOGE("make_null_terminated_str", "Memory allocation failed");
        return NULL;
    }

    memcpy(result, data, effective_len);
    result[effective_len] = '\0';  // Ensure terminated

    return result;
}

/// @brief Assembes 2 bytes in the correct order for big or little endian
/// @param bytes 2 bytes in an array
/// @param is_big_endian  = choos a endian
/// @return returns the integer
uint16_t bytes_to_integer(uint8_t *bytes, int is_big_endian) {
    if (is_big_endian) {
        return (bytes[0] << 8) | bytes[1]; // MSB first
    } else {
        return (bytes[1] << 8) | bytes[0]; // LSB first
    }
}

/// @brief Splits an integer into 2 bytes in the correct order for big or little endian
/// @param value The 16-bit integer to split
/// @param bytes Output array to store the 2 bytes
/// @param is_big_endian Choose endian format
void integer_to_bytes(uint16_t value, uint8_t *bytes, int is_big_endian) {
    if (is_big_endian) {
        bytes[0] = (value >> 8) & 0xFF;  // MSB first
        bytes[1] = value & 0xFF;         // LSB second
    } else {
        bytes[0] = value & 0xFF;         // LSB first
        bytes[1] = (value >> 8) & 0xFF;  // MSB second
    }
}

/// @brief Calculates the CRC for data that will be sent
/// @param data - data to crc
/// @param length Length of data
/// @return crc value
uint16_t calculate_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF; // Initial value for CRC-16-CCITT
    const uint16_t poly = 0x1021; // CRC-16-CCITT polynomial
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

/// @brief Takes the packet structure and builds the byte array that will be sent includes the CRC calc.
/// @param packet 
/// @param buffer 
/// @param ignoreData if true the assembler will assume that the data length is zero.  This can be used to send a number as the length value as is done with the start chunking function.
void assemble_transmit_buffer(data_packet_t *packet, uint8_t *buffer){
    
    bool ignoresizecheck = false;
    
    // Header: command (1 byte) + reply type (1 byte) + length (2 bytes, little-endian)
    buffer[0] = packet->command;
    buffer[1] = packet->reply;
    buffer[2] = packet->length & 0xFF; // Length LSB
    buffer[3] = (packet->length >> 8) & 0xFF; // Length MSB

    if(packet->command == CMD_START_CHUNKING) ignoresizecheck = true; //Ignore the size when we are starting chunking

    memset(&buffer[PACKET_HEADER_SIZE], 0, PACKET_PAYLOAD_SIZE); // Zero payload
    
    if(packet->length > 0 && ignoresizecheck == false){
        // Payload: packet payload bytes
        memcpy(&buffer[PACKET_HEADER_SIZE], packet->data, packet->length);
    }
   
    // CRC-16: calculate over command, length, and data
    uint16_t crc = calculate_crc16(buffer, PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE);

    // Append CRC (little-endian)
    buffer[PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE] = crc & 0xFF; // CRC LSB
    buffer[PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE + 1] = (crc >> 8) & 0xFF; // CRC MSB

    //ESP_LOGW("DEBUG", "Packet Assembled the following:");
    //ESP_LOG_BUFFER_HEX_LEVEL("DEBUG", buffer, PACKET_TOTAL_SIZE, ESP_LOG_WARN);
}

/// @brief Takes in incoming packet of data and returns a structure with the data & commmand all crc checked.
/// @param buffer The bufer to decode
/// @param buffer_length size of the buffer, should match the packet structure or it will be rejected
/// @param packet The output CRC checked packet
/// @param ignoresizecheck When true allows the length field to be longer than the protocol permits, aany data payload will be ignored.  Used for sending a value as the length as is used in the chunk start message.
/// @return esp_err_t
/// @brief Logs packet details including raw hex, command, reply, length, and optional chunking info
/// @param action "Received", "Processed", or "Sent" to indicate the action
/// @param buffer Raw packet buffer
/// @param packet Decoded/encoded packet structure (can be NULL if not decoded/encoded yet)
/// @param chunk_num Current chunk number (0 = not chunking, >0 = chunk number)
/// @param total_chunks Total number of chunks (0 = not chunking, >0 = total chunks)
/// @param is_rx true for incoming packets (use "decoded"), false for outgoing (use "encoded")
void log_packet_details(const char *action, const uint8_t *buffer, const data_packet_t *packet, uint16_t chunk_num, uint16_t total_chunks, bool is_rx) {
    const char *decode_encode = is_rx ? "Decoded" : "Encoded";
    
    // Log raw hex
    char prefix[64];
    snprintf(prefix, sizeof(prefix), "%s: Raw packet: ", action);
    async_log_hex("DEBUG", prefix, buffer, PACKET_TOTAL_SIZE);
    
    if (packet != NULL) {
        // Build log message with packet details
        if (chunk_num > 0 && total_chunks > 0) {
            // Chunking context
            ASYNC_LOG("DEBUG", "%s: %s - cmd=%u, reply=%u, len=%u, chunk=%u/%u", 
                      action, decode_encode, packet->command, packet->reply, packet->length, chunk_num, total_chunks);
        } else if (packet->command == CMD_START_CHUNKING) {
            // Start chunking - length field contains number of chunks
            ASYNC_LOG("DEBUG", "%s: %s - cmd=%u (START_CHUNKING), reply=%u, total_chunks=%u", 
                      action, decode_encode, packet->command, packet->reply, packet->length);
        } else if (packet->command == CMD_END_CHUNKING) {
            // End chunking
            ASYNC_LOG("DEBUG", "%s: %s - cmd=%u (END_CHUNKING), reply=%u, len=%u", 
                      action, decode_encode, packet->command, packet->reply, packet->length);
        } else {
            // Normal packet
            ASYNC_LOG("DEBUG", "%s: %s - cmd=%u, reply=%u, len=%u", 
                      action, decode_encode, packet->command, packet->reply, packet->length);
        }
    } else {
        // Buffer only, extract basic info
        uint8_t cmd = buffer[0];
        uint8_t reply = buffer[1];
        uint16_t len = buffer[2] | (buffer[3] << 8);
        
        if (chunk_num > 0 && total_chunks > 0) {
            ASYNC_LOG("DEBUG", "%s: Raw - cmd=%u, reply=%u, len=%u, chunk=%u/%u", 
                      action, cmd, reply, len, chunk_num, total_chunks);
        } else if (cmd == CMD_START_CHUNKING) {
            ASYNC_LOG("DEBUG", "%s: Raw - cmd=%u (START_CHUNKING), reply=%u, total_chunks=%u", 
                      action, cmd, reply, len);
        } else {
            ASYNC_LOG("DEBUG", "%s: Raw - cmd=%u, reply=%u, len=%u", 
                      action, cmd, reply, len);
        }
    }
}

esp_err_t decode_received_packet(const uint8_t *buffer, size_t buffer_length, data_packet_t *packet) {

    if (buffer_length < PACKET_HEADER_SIZE + PACKET_CRC_SIZE || buffer_length > PACKET_TOTAL_SIZE) {
        return ESP_ERR_INVALID_SIZE; // Invalid size
    }

    // FIX: Defensively clear the entire packet structure first to prevent stale data
    memset(packet, 0, sizeof(data_packet_t));

    bool ignoresizecheck = false;

    // Extract command
    packet->command = buffer[0];
    //Extract Reply Type
    packet->reply = buffer[1];

    if(packet->command == CMD_START_CHUNKING) ignoresizecheck = true; //Ignore the size when we are starting chunking

    // Extract length early to check for CMD_OK with zero length
    uint8_t len_bytes[2] = {buffer[2], buffer[3]};
    uint16_t extracted_length = bytes_to_integer(len_bytes, 0);
    
    // Handle CMD_OK packets with zero length (any reply type)
    if(packet->command == CMD_OK && extracted_length == 0) {
        // Data buffer already cleared by memset above, but be explicit
        packet->length = 0;
        return ESP_OK;
    }
    
    // Extract length (little-endian) for normal packets
    packet->length = extracted_length;

    if(ignoresizecheck == false){
        // Validate length
        if (packet->length > PACKET_PAYLOAD_SIZE) {
            return ESP_ERR_INVALID_STATE; // Length exceeds max payload
        }

        if ((buffer_length < PACKET_HEADER_SIZE + packet->length + PACKET_CRC_SIZE)) {
            return ESP_ERR_INVALID_SIZE; // Buffer size mismatch
        }

        // Extract payload
        memcpy(packet->data, &buffer[PACKET_HEADER_SIZE], packet->length); //Will only load the specified data length into the structure.
        
        // Zero remaining bytes if length < PACKET_PAYLOAD_SIZE
        if(packet->length < PACKET_PAYLOAD_SIZE) {
            memset(packet->data + packet->length, 0, PACKET_PAYLOAD_SIZE - packet->length);
        }
    }

    // Extract received CRC (little-endian)
    uint8_t crc_bytes[2] = {buffer[PACKET_TOTAL_SIZE - 2], buffer[PACKET_TOTAL_SIZE - 1]}; //Grab the last two bytes for the crc check
    uint16_t received_crc = bytes_to_integer(crc_bytes, 0);
    
    // Calculate CRC over header + payload
    uint16_t calc_crc = calculate_crc16(buffer, PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE);

    if (calc_crc != received_crc) {
        ASYNC_LOG(DECODELOG, "Recived CRC was %u, calculated was %u", received_crc, calc_crc);
        //ESP_LOGE(DECODELOG, "Recived CRC was %u, calculated was %u", received_crc, calc_crc);
        return ESP_ERR_INVALID_CRC; // CRC mismatch
    }
    packet->crc = received_crc;
    
    return ESP_OK;
}


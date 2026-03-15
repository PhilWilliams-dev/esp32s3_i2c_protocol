#ifndef I2CSHARED_H
#define I2CSHARED_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c_types.h"

// ── Configuration mapped from Kconfig (idf.py menuconfig) ──

// Hardware
#define SDA_PIN             CONFIG_I2C_PROTOCOL_SDA_PIN
#define SCL_PIN             CONFIG_I2C_PROTOCOL_SCL_PIN
#define I2C_BUS_SPEED       CONFIG_I2C_PROTOCOL_BUS_SPEED
#define I2C_SCL_WAIT_US     CONFIG_I2C_PROTOCOL_SCL_WAIT_US

#if defined(CONFIG_I2C_PROTOCOL_PORT_0)
#define I2C_PORT I2C_NUM_0
#elif defined(CONFIG_I2C_PROTOCOL_PORT_1)
#define I2C_PORT I2C_NUM_1
#endif

#ifdef CONFIG_I2C_PROTOCOL_ADDR_BIT_7
#define I2C_ADDR_BIT I2C_ADDR_BIT_7
#else
#define I2C_ADDR_BIT I2C_ADDR_BIT_10
#endif

// Protocol tuning
#define MAX_PAYLOAD_SIZE        CONFIG_I2C_PROTOCOL_MAX_PAYLOAD_SIZE
#define PACKET_PAYLOAD_SIZE     CONFIG_I2C_PROTOCOL_PACKET_PAYLOAD_SIZE
#define I2C_REQUEST_TIMEOUT     CONFIG_I2C_PROTOCOL_REQUEST_TIMEOUT
#define I2C_COMMS_MANAGER_DELAY CONFIG_I2C_PROTOCOL_COMMS_MANAGER_DELAY

// Protocol-structural constants
#define PACKET_HEADER_SIZE 4   // command (1) + reply (1) + length (2, little-endian)
#define PACKET_CRC_SIZE 2      // 16-bit CRC (little-endian)
#define PACKET_TOTAL_SIZE (PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE + PACKET_CRC_SIZE)

// Task configuration
#define I2C_RX_TASK_STACK_SIZE   CONFIG_I2C_PROTOCOL_RX_TASK_STACK_SIZE
#define I2C_TX_TASK_STACK_SIZE   CONFIG_I2C_PROTOCOL_TX_TASK_STACK_SIZE
#define I2C_PROC_TASK_STACK_SIZE CONFIG_I2C_PROTOCOL_PROC_TASK_STACK_SIZE
#define I2C_COMMS_TASK_PRIORITY  CONFIG_I2C_PROTOCOL_COMMS_TASK_PRIORITY
#define I2C_PROC_TASK_PRIORITY   CONFIG_I2C_PROTOCOL_PROC_TASK_PRIORITY
#define I2C_MAIN_TASK_STACK_SIZE (I2C_RX_TASK_STACK_SIZE + I2C_TX_TASK_STACK_SIZE + I2C_PROC_TASK_STACK_SIZE)

// Command type - extensible by user
typedef uint8_t i2c_command_t;

// Reserved command range - user commands must be below this value
#define CMD_RESERVED_MIN    201

// Reserved commands (used internally by the driver - DO NOT redefine)
#define CMD_START_CHUNKING  250
#define CMD_END_CHUNKING    251
#define CMD_RESTART         252
#define CMD_ERR             254
#define CMD_OK              255

// Helper macro for users to define commands with compile-time validation
#define I2C_USER_COMMAND(name, value) \
    _Static_assert((value) > 0 && (value) < CMD_RESERVED_MIN, \
        "Command " #name " (" #value ") must be between 1 and 200"); \
    static const i2c_command_t name = (value)

typedef enum {
    REPLY_NONE,
    REPLY_OK,
    REPLY_DATA,
    REPLY_CHUNK
    } reply_type_t;
    
typedef enum {
    RX_STATE_IDLE,
    RX_STATE_RECEIVING,
    RX_STATE_PROCESSING,
    RX_STATE_CLEANUP,
    RX_STATE_ERROR
    } rx_state_t;
    
typedef enum {
    TX_STATE_IDLE,
    TX_STATE_WAIT,
    TX_STATE_SENDING,
    TX_STATE_CLEANUP,
    TX_STATE_ERROR
    } tx_state_t;

typedef struct {
    i2c_command_t command;
    reply_type_t reply;
    uint16_t length;
    uint8_t data[PACKET_PAYLOAD_SIZE];
    uint16_t crc;
} data_packet_t;


/// @brief Splits a 16-bit integer into 2 bytes respecting endianness.
/// @param value The 16-bit integer to split.
/// @param bytes Output array (at least 2 bytes).
/// @param is_big_endian Non-zero for big-endian, zero for little-endian.
void integer_to_bytes(uint16_t value, uint8_t *bytes, int is_big_endian);

/// @brief Assembles 2 bytes into a 16-bit integer respecting endianness.
/// @param bytes 2-byte input array.
/// @param is_big_endian Non-zero for big-endian, zero for little-endian.
/// @return The reconstructed 16-bit integer.
uint16_t bytes_to_integer(uint8_t *bytes, int is_big_endian);

/// @brief Decodes a received byte array into a packet structure with CRC validation.
/// @param buffer Raw received bytes (must be PACKET_TOTAL_SIZE for a full packet).
/// @param buffer_length Size of the buffer.
/// @param packet Output packet structure (zeroed first to prevent stale data).
/// @return ESP_OK on success, or ESP_ERR_INVALID_SIZE / ESP_ERR_INVALID_STATE / ESP_ERR_INVALID_CRC.
esp_err_t decode_received_packet(const uint8_t *buffer, size_t buffer_length, data_packet_t *packet);

/// @brief Builds a transmit byte array from a packet structure, including CRC.
/// @param packet Source packet to serialise.
/// @param buffer Output buffer (must be at least PACKET_TOTAL_SIZE bytes).
void assemble_transmit_buffer(data_packet_t *packet, uint8_t *buffer);

/// @brief Returns the number of chunks needed to send the given byte count.
/// @param bytes Total payload size in bytes.
/// @return Number of PACKET_PAYLOAD_SIZE chunks required (rounded up).
uint16_t bytes_to_chunks(uint32_t bytes);

/// @brief Returns the maximum byte capacity for the given chunk count.
/// @param chunks Number of chunks.
/// @return Maximum byte count (chunks * PACKET_PAYLOAD_SIZE).
uint32_t chunks_to_bytes(uint16_t chunks);

/// @brief Checks if a byte array contains a null terminator within the given length.
/// @param data Input byte array (may be NULL).
/// @param length Number of bytes to scan.
/// @return true if a '\0' is found within length, false otherwise.
bool is_null_terminated(const uint8_t* data, size_t length);

/// @brief Allocates a new null-terminated string from a byte array. Caller must free the result.
/// @param data Input byte array (may be NULL).
/// @param length Number of bytes to consider.
/// @return Heap-allocated null-terminated char*, or NULL on failure/invalid input.
char* make_null_terminated_str(const uint8_t* data, size_t length);

#ifdef CONFIG_I2C_PROTOCOL_ASYNC_DEBUG
#define ASYNC_DEBUG_ENABLED 1
#else
#define ASYNC_DEBUG_ENABLED 0
#endif

#if ASYNC_DEBUG_ENABLED

/// @brief Initialises the async logging queue and background task. Safe to call multiple times.
void async_log_init(void);

/// @brief Queues a formatted log message for background output. Non-blocking; drops if queue is full.
/// @param tag Short identifier shown in the log output.
/// @param format printf-style format string.
void async_log(const char* tag, const char* format, ...);

/// @brief Queues a hex dump of a byte buffer for background output.
/// @param tag Short identifier shown in the log output.
/// @param prefix Optional string prepended to the hex data (may be NULL).
/// @param data Byte array to dump (may be NULL).
/// @param len Number of bytes to dump.
void async_log_hex(const char* tag, const char* prefix, const uint8_t* data, size_t len);

#define ASYNC_LOG_INIT()                       async_log_init()
#define ASYNC_LOG(tag, format, ...)            async_log((tag), (format), ##__VA_ARGS__)
#define ASYNC_LOG_HEX(tag, prefix, data, len)  async_log_hex((tag), (prefix), (data), (len))

#else

#define ASYNC_LOG_INIT()                       do {} while(0)
#define ASYNC_LOG(tag, format, ...)            do {} while(0)
#define ASYNC_LOG_HEX(tag, prefix, data, len)  do {} while(0)

#endif



#endif // I2CSHARED_H

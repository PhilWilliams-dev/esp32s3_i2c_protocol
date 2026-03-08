#ifndef I2CSHARED_H
#define I2CSHARED_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c_types.h"


#define MAX_PAYLOAD_SIZE (122880)  // User-defined total payload size in bytes (e.g., 120 KB)
#define PACKET_HEADER_SIZE 4    // command (1) + length (2, little-endian)
#define PACKET_CRC_SIZE 2      // 16-bit CRC (little-endian)
#define PACKET_PAYLOAD_SIZE 40 // Fixed payload size per chunk (adjustable)

//Do not modify these uness you know what you are doing
#define PACKET_TOTAL_SIZE (PACKET_HEADER_SIZE + PACKET_PAYLOAD_SIZE + PACKET_CRC_SIZE) // 46 bytes per chunk
#define I2C_RX_TASK_STACK_SIZE 4096
#define I2C_TX_TASK_STACK_SIZE 4096
#define I2C_PROC_TASK_STACK_SIZE 4096 //+ MAX_PAYLOAD_SIZE
#define I2C_MAIN_TASK_STACK_SIZE (I2C_RX_TASK_STACK_SIZE + I2C_TX_TASK_STACK_SIZE + I2C_PROC_TASK_STACK_SIZE) //Main root task size

//I2C Timing
#define I2C_COMMS_MANAGER_DELAY 10 //task delay for tx and rx tasks, cycle time if you will
#define I2C_REQUEST_TIMEOUT 5000
#define I2C_COMMS_TASK_PRIORITY 5
#define I2C_PROC_TASK_PRIORITY 5

//I2C Config
#define I2C_BUS_SPEED 100000
#define SDA_PIN GPIO_NUM_8
#define SCL_PIN GPIO_NUM_9
#define I2C_PORT I2C_NUM_0
#define I2C_ADDR_BIT I2C_ADDR_BIT_7
#define I2C_SCL_WAIT_US 10000000

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
    I2C_SEND = 1,
    I2C_SEND_OK_BACK = 2,
    I2C_SEND_DATA_OK_BACK = 3,
    I2C_SEND_DATA_DATA_BACK = 4
    } i2c_command_type_t;

typedef enum {
    REPLY_NONE,  // No reply expected
    REPLY_OK,    // Expects CMD_OK
    REPLY_DATA,  // Expects OK with data
    REPLY_CHUNK //Expects to get the data back in chunks
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

    // Data packet structure for I2C communication
typedef struct {
    i2c_command_t command;
    reply_type_t reply;
    uint16_t length;
    uint8_t data[PACKET_PAYLOAD_SIZE];
    uint16_t crc; // 16-bit CRC
} data_packet_t;


void integer_to_bytes(uint16_t value, uint8_t *bytes, int is_big_endian);
uint16_t bytes_to_integer(uint8_t *bytes, int is_big_endian);

esp_err_t decode_received_packet(const uint8_t *buffer, size_t buffer_length, data_packet_t *packet);
void assemble_transmit_buffer(data_packet_t *packet, uint8_t *buffer);


uint16_t bytes_to_chunks(uint32_t bytes);
uint32_t chunks_to_bytes(uint16_t chunks);
bool is_null_terminated(const uint8_t* data, size_t length);
char* make_null_terminated_str(const uint8_t* data, size_t length);

// ASYNC_DEBUG_ENABLED control - set to 0 to disable async logging
#ifndef ASYNC_DEBUG_ENABLED
#define ASYNC_DEBUG_ENABLED 0  // Default to enabled
#endif

#define ASYNC_LOG(tag, format, ...) async_log((tag), (format), ##__VA_ARGS__);
void async_log_init(void);
void async_log(const char* tag, const char* format, ...);
void async_log_hex(const char* tag, const char* prefix, const uint8_t* data, size_t len);
void log_packet_details(const char *action, const uint8_t *buffer, const data_packet_t *packet, uint16_t chunk_num, uint16_t total_chunks, bool is_rx);



#endif // I2CSHARED_H

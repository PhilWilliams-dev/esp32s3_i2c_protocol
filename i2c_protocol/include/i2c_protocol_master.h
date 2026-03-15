#ifndef I2C_MASTER_PROTOCOL_H
#define I2C_MASTER_PROTOCOL_H

#include "sdkconfig.h"

#ifndef CONFIG_I2C_PROTOCOL_MASTER
#error "I2C Master code detected but CONFIG_I2C_PROTOCOL_MASTER is not enabled. " \
       "Please enable master mode in menuconfig: Component config -> i2c_protocol -> I2C Protocol Role -> Master, " \
       "or set CONFIG_I2C_PROTOCOL_MASTER=y in sdkconfig"
#endif

#include "i2c_protocol_shared.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/// @brief Entry-point FreeRTOS task. Initialises the master protocol and spawns
///        the RX, TX, and command-processor sub-tasks.
/// @param arg Pointer to a caller-owned i2c_master_bus_config_t. Must remain valid.
void i2c_master_task(void *arg);

/// @brief Sends a command to a slave and optionally waits for a response.
///        Payloads larger than PACKET_PAYLOAD_SIZE are automatically chunked.
///        For REPLY_DATA and REPLY_CHUNK the caller must free *response_data.
/// @param command      The command byte to send.
/// @param payload      Pointer to the outbound data (may be NULL if payload_length is 0).
/// @param payload_length  Number of bytes in payload.
/// @param slave_id     7-bit I2C address of the target slave.
/// @param reply_type   Expected reply: REPLY_NONE, REPLY_OK, REPLY_DATA, or REPLY_CHUNK.
/// @param response_data   [out] Heap-allocated response buffer (caller frees). May be NULL for REPLY_NONE/OK.
/// @param response_length [out] Number of bytes written to *response_data. May be NULL for REPLY_NONE/OK.
/// @return ESP_OK on success, or an appropriate esp_err_t on failure.
esp_err_t i2c_make_request(i2c_command_t command, uint8_t *payload, uint32_t payload_length, uint8_t slave_id, reply_type_t reply_type, uint8_t **response_data, uint32_t *response_length);


#endif // I2C_MASTER_PROTOCOL_H

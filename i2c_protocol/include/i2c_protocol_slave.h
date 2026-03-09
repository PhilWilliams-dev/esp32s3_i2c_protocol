#ifndef I2C_SLAVE_PROTOCOL_H
#define I2C_SLAVE_PROTOCOL_H

#ifndef CONFIG_I2C_PROTOCOL_SLAVE
#error "I2C Slave code detected but CONFIG_I2C_PROTOCOL_SLAVE is not enabled. " \
       "Please enable slave mode in menuconfig: Component config -> i2c_protocol -> I2C Protocol Role -> Slave, " \
       "or set CONFIG_I2C_PROTOCOL_SLAVE=y in sdkconfig"
#endif

#include <stdint.h>
#include "esp_err.h"
#include "i2c_protocol_shared.h"

#define MAX_COMMANDS 50

typedef struct {
    i2c_command_t command;
    esp_err_t (*process_command)(data_packet_t *packet, data_packet_t *response_packet, uint8_t **chunked_payload, uint16_t *chunked_payload_length);
    } command_registry_item_t;

extern command_registry_item_t command_registry[];

/// @brief Registers a command handler with the slave protocol.
/// @param reg Pointer to a command_registry_item_t describing the command and its handler.
/// @return ESP_OK on success, ESP_ERR_NO_MEM if MAX_COMMANDS has been reached.
esp_err_t i2c_register_command(command_registry_item_t *reg);

/// @brief Entry-point FreeRTOS task. Initialises the slave protocol and spawns
///        the RX, TX, and command-processor sub-tasks. Suspends itself once running.
/// @param arg Pointer to a uint8_t holding the 7-bit I2C slave address. Must remain valid.
void i2c_slave_task(void *arg);

#endif // I2C_SLAVE_PROTOCOL_H

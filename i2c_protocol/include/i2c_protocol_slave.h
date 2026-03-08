#ifndef I2C_SLAVE_PROTOCOL_H
#define I2C_SLAVE_PROTOCOL_H

// Check if slave mode is enabled in configuration
#ifndef CONFIG_I2C_PROTOCOL_SLAVE
#error "I2C Slave code detected but CONFIG_I2C_PROTOCOL_SLAVE is not enabled. " \
       "Please enable slave mode in menuconfig: Component config -> i2c_protocol -> I2C Protocol Role -> Slave, " \
       "or set CONFIG_I2C_PROTOCOL_SLAVE=y in sdkconfig"
#endif

#include <stdint.h>
#include "esp_err.h"
//#include "i2c_protocol_commands.h"
#include "i2c_protocol_shared.h"

#define MAX_COMMANDS 50



typedef struct {
    i2c_command_t command;
    esp_err_t (*process_command)(data_packet_t *packet, data_packet_t *response_packet, uint8_t **chunked_payload, uint16_t *chunked_payload_length); // Processes command, sets response
    } command_registry_item_t;

extern command_registry_item_t slave_registry[];
esp_err_t i2c_register_command(command_registry_item_t *reg);


void i2c_slave_task(void *arg);

// Memory management functions
void print_memory_info(void);
void start_heap_tracing(void);
void stop_heap_tracing(void);
void print_heap_trace_info(void);

#endif // I2C_SLAVE_PROTOCOL_H

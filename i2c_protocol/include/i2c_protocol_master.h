#ifndef I2C_MASTER_PROTOCOL_H
#define I2C_MASTER_PROTOCOL_H

// Check if master mode is enabled in configuration
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


void i2c_master_task(void *arg);

esp_err_t i2c_make_request(i2c_command_t command, uint8_t *payload, uint32_t payload_length, uint8_t slave_id, reply_type_t reply_type, uint8_t **response_data, uint32_t *response_length);


#endif // I2C_MASTER_PROTOCOL_H

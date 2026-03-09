#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/i2c_slave.h"
#include "driver/i2c_types.h"
#include "i2c_protocol_slave.h"
#include "i2c_protocol_shared.h"

static TaskHandle_t rx_task_handle = NULL;
static TaskHandle_t tx_task_handle = NULL;
static TaskHandle_t command_task_handle = NULL;
static i2c_slave_dev_handle_t slave_handle;

// RX storage
static uint8_t raw_rx_buffer[PACKET_TOTAL_SIZE];
static uint32_t raw_rx_buffer_length = 0;
static data_packet_t rx_current_packet;
uint8_t *chunked_data;
uint16_t chunked_data_length = 0;

// TX storage
static uint8_t raw_tx_buffer[PACKET_TOTAL_SIZE];
static data_packet_t tx_current_packet;

// State machines
static tx_state_t tx_state = TX_STATE_IDLE;
static rx_state_t rx_state = RX_STATE_IDLE;

// Flags
static bool tx_ready = false;
static bool rx_complete = false;
static bool chunking = false;

// Command registry
command_registry_item_t command_registry[MAX_COMMANDS];
static uint32_t slave_registry_count = 0;

static SemaphoreHandle_t rx_complete_sem = NULL;
static SemaphoreHandle_t tx_ready_sem = NULL;
static SemaphoreHandle_t rx_wake_sem = NULL;
static SemaphoreHandle_t tx_wake_sem = NULL;



/// @brief Registers a command handler with the slave protocol.
/// @param reg Pointer to a command_registry_item_t describing the command and its handler.
/// @return ESP_OK on success, ESP_ERR_NO_MEM if MAX_COMMANDS has been reached.
esp_err_t i2c_register_command(command_registry_item_t *reg) { 
    if (slave_registry_count >= MAX_COMMANDS) return ESP_ERR_NO_MEM;
    command_registry[slave_registry_count++] = *reg;
    return ESP_OK;
}


// ─── RX State Machine ───────────────────────────────────────────────

/// @brief ISR callback when the master sends data.
///        Copies the incoming bytes into the raw buffer and wakes the RX task.
///        Returns pdTRUE (NACK) if already busy processing a previous packet.
/// @param handle   The slave device handle.
/// @param evt_data Contains the received buffer and its length.
/// @param arg      Unused user context.
/// @return pdFALSE on success, pdTRUE if busy (NACK).
static bool i2c_slave_rx_cb(i2c_slave_dev_handle_t handle, const i2c_slave_rx_done_event_data_t *evt_data, void *arg) {
    if(rx_state == RX_STATE_IDLE){
        memcpy(raw_rx_buffer, evt_data->buffer, evt_data->length);
        raw_rx_buffer_length = evt_data->length;
        rx_complete = false;
        rx_state = RX_STATE_RECEIVING;
        portMEMORY_BARRIER();
        BaseType_t higherPrioWoken = pdFALSE;
        xSemaphoreGiveFromISR(rx_wake_sem, &higherPrioWoken);
        portYIELD_FROM_ISR(higherPrioWoken);
        return pdFALSE;
    }
    else{
        return pdTRUE;
    }
}

/// @brief Decodes the raw buffer into rx_current_packet and validates the CRC.
///        On any decode error, sets TX to error state so the master can retrieve CMD_ERR.
void rx_receiving_state(){
    esp_err_t decode = decode_received_packet(raw_rx_buffer, raw_rx_buffer_length, &rx_current_packet);

    switch(decode){
        case ESP_OK:
            rx_state = RX_STATE_PROCESSING;
            break;
        case ESP_ERR_INVALID_SIZE:
        case ESP_ERR_INVALID_CRC:
        case ESP_ERR_INVALID_STATE:
            rx_state = RX_STATE_CLEANUP;
            tx_state = TX_STATE_ERROR;
            break;
        default:
            esp_system_abort("Unhandled RX packet decode error.");
            rx_state = RX_STATE_CLEANUP;
            break;
        }
    return;
}

/// @brief Handles protocol-level commands (CMD_RESTART, CMD_ERR) directly.
///        All other commands signal the command-processor task via rx_complete_sem.
void rx_processing_state(){
    switch(rx_current_packet.command){
        case(CMD_RESTART):
            esp_restart();
            break;
        case(CMD_ERR):
            tx_state = TX_STATE_CLEANUP;
            rx_state = RX_STATE_CLEANUP;
            break;
        default:
            rx_complete = true;
            __sync_synchronize();
            xSemaphoreGive(rx_complete_sem);
            rx_state = RX_STATE_CLEANUP;
            break;
    }
    return;
}

/// @brief Resets the raw buffer. The rx_current_packet is intentionally left intact
///        so the command-processor task can still read it.
void rx_cleanup_state(){
    memset(raw_rx_buffer, 0, PACKET_TOTAL_SIZE);
    raw_rx_buffer_length = 0;
    rx_state = RX_STATE_IDLE;
    return;
}

/// @brief Dispatches to the appropriate RX state handler based on the current rx_state.
void rx_state_machine(){
    switch(rx_state){
        case RX_STATE_IDLE:
            break;
        case RX_STATE_RECEIVING:
            rx_receiving_state();
            break;
        case RX_STATE_PROCESSING:
            rx_processing_state();
            break;
        case RX_STATE_CLEANUP:
            rx_cleanup_state();
            break;
        default:
            rx_cleanup_state();
    }
}

/// @brief FreeRTOS task that waits on rx_wake_sem and runs the RX state machine until idle.
/// @param arg Unused (required by FreeRTOS task signature).
void i2c_slave_rx_task(void *arg){
    while(1){
        
        xSemaphoreTake(rx_wake_sem, pdMS_TO_TICKS(1000));
        while (rx_state != RX_STATE_IDLE) {
            rx_state_machine();
        }
        
    }
}



// ─── TX State Machine ───────────────────────────────────────────────

/// @brief ISR callback when the master requests data (clock-stretch read).
///        Despite being named "TX", this is master-driven: the master sends data (RX)
///        then requests a reply (TX).  Wakes the TX task to begin sending.
/// @param handle   The slave device handle.
/// @param evt_data Request event data from the driver.
/// @param arg      Unused user context.
/// @return Always pdFALSE.
static bool i2c_slave_tx_cb(i2c_slave_dev_handle_t handle, const i2c_slave_request_event_data_t *evt_data, void *arg) {
    tx_state = TX_STATE_SENDING;
    portMEMORY_BARRIER();
    BaseType_t higherPrioWoken = pdFALSE;
    xSemaphoreGiveFromISR(tx_wake_sem, &higherPrioWoken);
    portYIELD_FROM_ISR(higherPrioWoken);
    return pdFALSE;
}

/// @brief Waits for tx_ready_sem (up to 10 s).  If the semaphore times out the
///        response is overwritten with CMD_ERR.  REPLY_OK sends a minimal 1-byte
///        acknowledgement; all other replies send a full assembled packet.
void tx_sending_state(){
    
    if(xSemaphoreTake(tx_ready_sem, pdMS_TO_TICKS(10000)) == pdTRUE){
        __sync_synchronize();
    }
    else{
        tx_current_packet.command = CMD_ERR;
        tx_current_packet.length = 0;
    }

    uint32_t writeout = 0;

    esp_err_t txerr;
    if(tx_current_packet.reply == REPLY_OK){
        uint8_t okpayload[1];
        okpayload[0] = CMD_OK;
        txerr = i2c_slave_write(slave_handle, okpayload, 1, &writeout, I2C_REQUEST_TIMEOUT);
    }
    else{
        assemble_transmit_buffer(&tx_current_packet, raw_tx_buffer);
        txerr = i2c_slave_write(slave_handle, raw_tx_buffer, PACKET_TOTAL_SIZE, &writeout, I2C_REQUEST_TIMEOUT);
    }
    
    if(txerr != ESP_OK){
        tx_state = TX_STATE_CLEANUP;
    }
    tx_state = TX_STATE_CLEANUP;
    return;

}

/// @brief Zeroes the TX buffers and returns to idle.
void tx_cleanup_state(){
    memset(raw_tx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&tx_current_packet, 0, sizeof(data_packet_t));
    tx_state = TX_STATE_IDLE;
    return;
}

/// @brief Prepares a CMD_ERR response and signals the TX task to send it.
void tx_error_state(){
    tx_current_packet.command = CMD_ERR;
    tx_current_packet.length = 0;
    __sync_synchronize();
    tx_ready = true;
    xSemaphoreGive(tx_ready_sem);
    tx_state = TX_STATE_CLEANUP;
}

/// @brief Dispatches to the appropriate TX state handler based on the current tx_state.
void tx_state_machine(){
    switch(tx_state){
        case TX_STATE_IDLE:
            break;
        case TX_STATE_SENDING:
            tx_sending_state();
            break;
        case TX_STATE_CLEANUP:
            tx_cleanup_state();
            break;
        case TX_STATE_ERROR:
            tx_error_state();
            break;
        default:
            tx_cleanup_state();
    }
}

/// @brief FreeRTOS task that waits on tx_wake_sem and runs the TX state machine until idle.
/// @param arg Unused (required by FreeRTOS task signature).
void i2c_slave_tx_task(void *arg){
    while(1){
        xSemaphoreTake(tx_wake_sem, pdMS_TO_TICKS(1000));
        while (tx_state != TX_STATE_IDLE) {
            tx_state_machine();
        }
    }
}



// ─── Command Processor ──────────────────────────────────────────────

/// @brief FreeRTOS task that processes received commands. Waits for rx_complete_sem
///        then looks up the received command in the registry and invokes the matching handler.
///        Handles both single-packet and chunked inbound data, and dispatches
///        the appropriate reply type (OK / DATA / CHUNK / NONE).
///        The rx_current_packet must be consumed here -- cleanup only clears the raw buffer.
/// @param arg Unused (required by FreeRTOS task signature).
void i2c_slave_execute_command_task(void *arg) {
   

    main_task_loop:
    
    while (1) {

        if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }

        __sync_synchronize();

        if (!rx_complete) {
            // Spurious wake -- semaphore signalled but flag not set
            continue;
        }

        rx_complete = false;
        __sync_synchronize();

            data_packet_t initial_packet;
                

                // ── Inbound chunking ──
                if(rx_current_packet.command == CMD_START_CHUNKING){
                    chunking = true;
                    uint16_t number_of_chunks = rx_current_packet.length;
                    uint16_t current_chunk = 0;

                    chunked_data = malloc((number_of_chunks * PACKET_PAYLOAD_SIZE + 1) * sizeof(uint8_t));
                    
                    if(chunked_data == NULL){
                        tx_current_packet.command = CMD_ERR;
                        tx_current_packet.length = 0;
                        __sync_synchronize();
                        tx_ready = true;
                        xSemaphoreGive(tx_ready_sem);
                        chunked_data_length = 0;
                        chunking = false;
                        rx_complete = false;
                        memset(&initial_packet, 0, sizeof(data_packet_t));
                        goto main_task_loop;
                    }

                    // Acknowledge CMD_START_CHUNKING so the master sends the first data chunk
                    rx_complete = false;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.length = 0;
                    tx_current_packet.reply = REPLY_OK;
                    __sync_synchronize();
                    tx_ready = true;
                    xSemaphoreGive(tx_ready_sem);

                    // Receive chunks until CMD_END_CHUNKING or error
                    while(1){

                        if(xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(500)) == pdTRUE){

                            if (rx_current_packet.command != CMD_END_CHUNKING){
                                current_chunk++;
                                if(current_chunk > number_of_chunks){
                                    // More chunks than expected
                                    tx_current_packet.command = CMD_ERR;
                                    tx_current_packet.length = 0;
                                    __sync_synchronize();
                                    tx_ready = true;
                                    xSemaphoreGive(tx_ready_sem);
                                    if(chunking == true) free(chunked_data);
                                    chunked_data_length = 0;
                                    chunking = false;
                                    rx_complete = false;
                                    memset(&initial_packet, 0, sizeof(data_packet_t));
                                    goto main_task_loop;
                                }
                                
                                // Each data chunk carries the actual command -- capture it
                                initial_packet.command = rx_current_packet.command;

                                memcpy(chunked_data + chunked_data_length, rx_current_packet.data, rx_current_packet.length);
                                chunked_data_length +=rx_current_packet.length;
                                

                                // Acknowledge this chunk
                                tx_current_packet.command = CMD_OK;
                                tx_current_packet.length = 0;
                                tx_current_packet.reply = REPLY_OK;
                                rx_complete = false;
                                __sync_synchronize();
                                tx_ready = true;
                                xSemaphoreGive(tx_ready_sem);

                            }
                            else if(rx_current_packet.command == CMD_END_CHUNKING){
                                if(rx_current_packet.command == CMD_END_CHUNKING && current_chunk != number_of_chunks){
                                    // Ended before all chunks arrived
                                    tx_current_packet.command = CMD_ERR;
                                    tx_current_packet.length = 0;
                                    __sync_synchronize();
                                    tx_ready = true;
                                    xSemaphoreGive(tx_ready_sem);
                                    if(chunking == true) free(chunked_data);
                                    chunked_data_length = 0;
                                    chunking = false;
                                    rx_complete = false;
                                    memset(&initial_packet, 0, sizeof(data_packet_t));
                                    goto main_task_loop;
                                }
                                else{
                                    // CMD_END_CHUNKING carries the reply type for the actual command
                                    initial_packet.reply = rx_current_packet.reply; 
                                    
                                    chunking = false;
                                    current_chunk = 0;
                                    number_of_chunks = 0;
                                    rx_complete = false;
                                    
                                    break;
                                }



                    }
                        } 
                        else{
                            // Timed out waiting for next chunk
                            tx_current_packet.command = CMD_ERR;
                            tx_current_packet.length = 0;
                            __sync_synchronize();
                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);
                            if (chunking == true) free(chunked_data);
                            chunked_data_length = 0;
                            chunking = false;
                            break;
                        }


                        
                    }



           
                }
                else{
                    // ── Single-packet path ──
                    memset(&initial_packet, 0, sizeof(data_packet_t));
                    
                    initial_packet.command = rx_current_packet.command;
                    initial_packet.reply = rx_current_packet.reply;
                    initial_packet.length = rx_current_packet.length;
                    initial_packet.crc = rx_current_packet.crc;
                    
                    memcpy(initial_packet.data, rx_current_packet.data, rx_current_packet.length);
                }

                // ── Look up and execute the registered command handler ──

                for (uint32_t i = 0; i < slave_registry_count; i++) {

                if (command_registry[i].command == initial_packet.command) {
                    
                    if(chunked_data != NULL && chunked_data_length > 0){
                        // Inbound chunked data is ready for the handler
                    }
                    else{
                        chunked_data = NULL;
                        chunked_data_length = 0;
                    }
                    
                    esp_err_t err = command_registry[i].process_command(&initial_packet, &tx_current_packet, &chunked_data, &chunked_data_length);

                    if (err == ESP_OK){

                        if(initial_packet.reply == REPLY_OK){
                            tx_current_packet.command = CMD_OK;
                            tx_current_packet.length = 0;
                            tx_current_packet.reply = REPLY_OK;
                            __sync_synchronize();
                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);
                            
                            if(chunked_data != NULL) {
                                free(chunked_data);
                                chunked_data = NULL;
                            }
                            chunked_data_length = 0;
                        }
                        else if(initial_packet.reply == REPLY_DATA){
                            __sync_synchronize();
                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);
                            
                            if(chunked_data != NULL) {
                                free(chunked_data);
                                chunked_data = NULL;
                            }
                            chunked_data_length = 0;
                        }

                        else if(initial_packet.reply == REPLY_CHUNK){
                            // ── Outbound chunked reply ──

                            uint16_t current_chunk = 0;
                            uint16_t data_offset = 0;
                            uint16_t number_of_chunks = bytes_to_chunks(chunked_data_length);
                            
                            // Tell the master how many chunks to expect
                            tx_current_packet.command = CMD_START_CHUNKING;
                            tx_current_packet.length = number_of_chunks;
                            tx_current_packet.reply = REPLY_NONE;
                            memset(&tx_current_packet.data, 0, sizeof(PACKET_PAYLOAD_SIZE));
                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);

                            if(xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(500))){

                                // Send each data chunk, acknowledged by CMD_OK from master
                                while(current_chunk < number_of_chunks){

                                    current_chunk++;
                                    
                                    tx_current_packet.command = CMD_OK;
                                    tx_current_packet.reply = REPLY_CHUNK;

                                    if(data_offset + PACKET_PAYLOAD_SIZE > chunked_data_length){
                                        tx_current_packet.length = chunked_data_length - data_offset;
                                    }
                                    else
                                    {
                                        tx_current_packet.length = PACKET_PAYLOAD_SIZE;
                                    }

                                    memcpy(tx_current_packet.data, chunked_data + data_offset, tx_current_packet.length);
                                 
                                    data_offset += tx_current_packet.length;
                        
                                    tx_ready = true;
                                    xSemaphoreGive(tx_ready_sem);
                                    
                                    // Wait for acknowledgement from master
                                    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                                        rx_complete = false;
                                        memset(&initial_packet, 0, sizeof(data_packet_t));
                                        if(chunked_data != NULL) free(chunked_data);
                                        chunked_data_length = 0;
                                        rx_state = RX_STATE_CLEANUP;
                                        tx_state = TX_STATE_CLEANUP;
                                        xSemaphoreGive(rx_wake_sem);
                                        xSemaphoreGive(tx_wake_sem);
                                        goto main_task_loop;
                                    }
                                    
                                    if(rx_current_packet.command != CMD_OK){
                                        rx_complete = false;
                                        memset(&initial_packet, 0, sizeof(data_packet_t));
                                        if(chunked_data != NULL) free(chunked_data);
                                        chunked_data_length = 0;
                                        goto main_task_loop;
                                    }

                                }

                                // All chunks sent -- send CMD_END_CHUNKING
                                tx_current_packet.command = CMD_END_CHUNKING;
                                tx_current_packet.length = 0;
                                tx_current_packet.reply = REPLY_NONE;
                                tx_ready = true;
                                xSemaphoreGive(tx_ready_sem);
    
                                // Wait for confirmation
                                xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(500));

                                rx_complete = false;
                                memset(&initial_packet, 0, sizeof(data_packet_t));
                                if(chunked_data != NULL) free(chunked_data);
                                chunked_data_length = 0;
                                rx_state = RX_STATE_CLEANUP;
                                tx_state = TX_STATE_CLEANUP;
                                xSemaphoreGive(rx_wake_sem);
                                xSemaphoreGive(tx_wake_sem);
                                goto main_task_loop;

                            }
                            else
                            {
                                // Timed out waiting for master to acknowledge CMD_START_CHUNKING
                                rx_complete = false;
                                memset(&initial_packet, 0, sizeof(data_packet_t));
                                if(chunked_data != NULL) free(chunked_data);
                                chunked_data_length = 0;
                                rx_state = RX_STATE_CLEANUP;
                                tx_state = TX_STATE_CLEANUP;
                                xSemaphoreGive(rx_wake_sem);
                                xSemaphoreGive(tx_wake_sem);
                                goto main_task_loop;
                            }
                            

                        }
                        else{
                            // REPLY_NONE -- no response needed
                            if(chunked_data != NULL) {
                                free(chunked_data);
                                chunked_data = NULL;
                            }
                            chunked_data_length = 0;
                        }
                    }
                    else {
                        // Command handler returned an error -- send CMD_ERR if a reply is expected
                        if(initial_packet.reply != REPLY_NONE){
                            tx_current_packet.command = CMD_ERR;
                            tx_current_packet.length = 0;
                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);
                            
                        }
                        if(chunked_data != NULL) {
                            free(chunked_data);
                            chunked_data = NULL;
                        }
                        chunked_data_length = 0;
                    }

                    memset(&initial_packet, 0, sizeof(data_packet_t));
                    chunking = false;
                    goto main_task_loop;
                }
            }

             // No registered handler found for this command
             rx_state = RX_STATE_CLEANUP;
             tx_state = TX_STATE_CLEANUP;
             if(chunked_data != NULL) {
                 free(chunked_data);
                 chunked_data = NULL;
             }
             chunked_data_length = 0;
             chunking = false;
             xSemaphoreGive(rx_wake_sem);
             xSemaphoreGive(tx_wake_sem);
             continue;
    }
}


// ─── Task & Init ────────────────────────────────────────────────────

/// @brief Initialises the I2C slave bus, buffers, semaphores, and registers callbacks.
/// @param slaveid 7-bit I2C slave address.
/// @return ESP_OK on success, ESP_FAIL if the slave device or callback registration fails.
esp_err_t i2c_protocol_init(uint8_t slaveid){

     esp_err_t err;

    rx_complete_sem = xSemaphoreCreateBinary();
    if (rx_complete_sem == NULL) {
        vTaskSuspend(NULL);
    }

    tx_ready_sem = xSemaphoreCreateBinary();
    if (tx_ready_sem == NULL) {
        vTaskSuspend(NULL);
    }

    rx_wake_sem = xSemaphoreCreateBinary();
    if (rx_wake_sem == NULL) {
        vTaskSuspend(NULL);
    }

    tx_wake_sem = xSemaphoreCreateBinary();
    if (tx_wake_sem == NULL) {
        vTaskSuspend(NULL);
    }

    async_log_init();

    memset(raw_rx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(raw_tx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&rx_current_packet, 0, sizeof(data_packet_t));
    memset(&tx_current_packet, 0, sizeof(data_packet_t));

    i2c_slave_config_t slave_config = {
        .addr_bit_len = I2C_ADDR_BIT,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .slave_addr = slaveid,
        .send_buf_depth = PACKET_TOTAL_SIZE,
        .receive_buf_depth = PACKET_TOTAL_SIZE,
        .flags.enable_internal_pullup = true
    };

    i2c_slave_event_callbacks_t callbacks = {
        .on_receive = i2c_slave_rx_cb,
        .on_request = i2c_slave_tx_cb,
    };

    err = i2c_new_slave_device(&slave_config, &slave_handle);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    err = i2c_slave_register_event_callbacks(slave_handle, &callbacks, NULL);

    if (err != ESP_OK) {
        i2c_del_slave_device(slave_handle);
        return ESP_FAIL;
    }
    return ESP_OK;
}


/// @brief Entry-point FreeRTOS task: initialises the slave protocol and spawns the
///        RX, TX, and command-processor sub-tasks. Suspends itself once running.
/// @param arg Pointer to a uint8_t holding the 7-bit I2C slave address. Suspends on failure.
void i2c_slave_task(void *arg){
    
    uint8_t *slaveid = (uint8_t *)arg;
    if (!slaveid) {
        vTaskSuspend(NULL);
        return;
    }
    esp_err_t err = i2c_protocol_init(*slaveid);
    if (err != ESP_OK) {
        vTaskSuspend(NULL);
        return;
    }

    BaseType_t task_created;

    task_created = xTaskCreatePinnedToCore(i2c_slave_rx_task, "i2c_rx",  I2C_RX_TASK_STACK_SIZE, NULL, I2C_COMMS_TASK_PRIORITY, &rx_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));

    task_created = xTaskCreatePinnedToCore(i2c_slave_tx_task, "i2c_tx",  I2C_TX_TASK_STACK_SIZE, NULL, I2C_COMMS_TASK_PRIORITY, &tx_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));

    task_created = xTaskCreatePinnedToCore(i2c_slave_execute_command_task, "i2c_PROC",  I2C_PROC_TASK_STACK_SIZE, NULL, I2C_PROC_TASK_PRIORITY, &command_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));

    vTaskSuspend(NULL);
}


#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "i2c_protocol_shared.h"

static TaskHandle_t rx_task_handle = NULL;
static TaskHandle_t tx_task_handle = NULL;
static TaskHandle_t command_task_handle = NULL;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t slave_device;

// RX storage
static uint8_t raw_rx_buffer[PACKET_TOTAL_SIZE];
static data_packet_t rx_current_packet;
// Preserved across cleanup so i2c_make_request() can read REPLY_CHUNK responses
static data_packet_t saved_chunked_response_packet;
static bool saved_chunked_response_valid = false;
// Preserved across cleanup so i2c_make_request() can read REPLY_DATA and chunked payload
static uint8_t saved_packet_data[PACKET_PAYLOAD_SIZE];
static uint32_t saved_packet_length = 0;
static bool saved_packet_data_valid = false;

// TX storage
static uint8_t raw_tx_buffer[PACKET_TOTAL_SIZE];
static data_packet_t tx_current_packet;

// State machines
static tx_state_t tx_state = TX_STATE_IDLE;
static rx_state_t rx_state = RX_STATE_IDLE;

// Flags
static bool tx_ready = false;
static bool chunking = false;
static bool reply_chunking = false;

static bool request_complete = false;
static bool request_error_state = false;
static esp_err_t request_err;

static SemaphoreHandle_t tx_complete_sem = NULL;
static SemaphoreHandle_t rx_complete_sem = NULL;
static SemaphoreHandle_t chunk_reply_read_sem = NULL;



// ─── RX State Machine ───────────────────────────────────────────────

/// @brief Receives a reply from the slave device and decodes it into rx_current_packet.
///        Saves a copy of chunking-related packets and payload data before cleanup can clear them.
void rx_receiving_state(){
    esp_err_t err;

    // REPLY_OK is a single-byte acknowledgement; everything else is a full packet
    if(tx_current_packet.reply == REPLY_OK && !reply_chunking){
        err = i2c_master_receive(slave_device, raw_rx_buffer, 1, I2C_REQUEST_TIMEOUT);
        ASYNC_LOG("MST_RX", "Received 1-byte REPLY_OK");
    }
    else{
        err = i2c_master_receive(slave_device, raw_rx_buffer, PACKET_TOTAL_SIZE, I2C_REQUEST_TIMEOUT);
        ASYNC_LOG("MST_RX", "Received %u bytes", PACKET_TOTAL_SIZE);
        ASYNC_LOG_HEX("MST_RX", "Raw: ", raw_rx_buffer, PACKET_TOTAL_SIZE);
    }

    if(err != ESP_OK){
        ASYNC_LOG("MST_RX", "Receive failed: %d", err);
        request_err = err;
        request_error_state = true;
        request_complete = true;
        rx_state = RX_STATE_ERROR;
        return;
    }

    err = decode_received_packet(raw_rx_buffer, PACKET_TOTAL_SIZE, &rx_current_packet);

    if(err != ESP_OK){
        ASYNC_LOG("MST_RX", "Decode error: %d", err);
        request_err = err;
        request_error_state = true;
        request_complete = true;
        rx_state = RX_STATE_CLEANUP;
        return;
    }

    ASYNC_LOG("MST_RX", "Decoded cmd=%u reply=%u len=%u", rx_current_packet.command, rx_current_packet.reply, rx_current_packet.length);
    ASYNC_LOG_HEX("MST_RX", "Payload: ", rx_current_packet.data, rx_current_packet.length);

    // Save chunking control packets before cleanup clears rx_current_packet
    if(rx_current_packet.command == CMD_START_CHUNKING || rx_current_packet.command == CMD_END_CHUNKING){
        memcpy(&saved_chunked_response_packet, &rx_current_packet, sizeof(data_packet_t));
        saved_chunked_response_valid = true;
    }

    // Save payload data before cleanup clears rx_current_packet
    if(rx_current_packet.length > 0 && rx_current_packet.length <= PACKET_PAYLOAD_SIZE){
        saved_packet_length = rx_current_packet.length;
        memcpy(saved_packet_data, rx_current_packet.data, saved_packet_length);
        saved_packet_data_valid = true;
    }

    rx_state = RX_STATE_PROCESSING;
}

/// @brief Cleans up RX and triggers TX cleanup.
///        When reply_chunking, blocks until the caller reads the data before clearing buffers.
///        saved_packet_data_valid is intentionally NOT cleared here -- i2c_make_request() clears
///        it after consuming the data.
void rx_cleanup_state(){
    if(reply_chunking){
        request_complete = true; 
        xSemaphoreGive(rx_complete_sem);
        tx_state = TX_STATE_CLEANUP;
        xSemaphoreTake(chunk_reply_read_sem, pdTICKS_TO_MS(1000));
    }
    memset(raw_rx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&rx_current_packet, 0, sizeof(data_packet_t));
    request_complete = true; 
    rx_state = RX_STATE_IDLE;
    
    if(!reply_chunking){
        xSemaphoreGive(rx_complete_sem);
        tx_state = TX_STATE_CLEANUP;
    }
}

/// @brief Resets the I2C bus after an error and transitions to cleanup.
void rx_state_error(){
    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_master_get_bus_handle(I2C_PORT, &bus_handle);
    i2c_master_bus_reset(bus_handle);
    request_err = ESP_FAIL;
    request_error_state = true;
    rx_state = RX_STATE_CLEANUP;
}

/// @brief Handles protocol-level responses. CMD_ERR triggers an error; everything else
///        is treated as a successful reply and moves to cleanup.
void rx_processing_state() {
    switch(rx_current_packet.command){
        case(CMD_ERR):
            request_err = ESP_FAIL;
            request_error_state = true;
            request_complete = true;
            rx_state = RX_STATE_CLEANUP;
            return;
        
        default:
            request_error_state = false;
            rx_state = RX_STATE_CLEANUP;
            return;
    }
}

/// @brief Dispatches to the appropriate RX state handler based on the current rx_state.
void rx_state_machine(){
    switch(rx_state){
        case RX_STATE_IDLE:
            break;
        case RX_STATE_PROCESSING:
            rx_processing_state();
            break;
        case RX_STATE_RECEIVING:
            rx_receiving_state();
            break;
        case RX_STATE_CLEANUP:
            rx_cleanup_state();
            break;
        case RX_STATE_ERROR:
            rx_state_error();
            break;
        default:
            rx_state = RX_STATE_CLEANUP;
            break;
    }
}

/// @brief FreeRTOS task that continuously runs the RX state machine.
/// @param arg Unused (required by FreeRTOS task signature).
void i2c_master_rx_task(void *arg){
    while(1){
        rx_state_machine();
        vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY));
    }
}



// ─── TX State Machine ───────────────────────────────────────────────

/// @brief Transmits the assembled packet to the slave.
///        If a reply is expected, kicks the RX state machine into RECEIVING and
///        parks TX in WAIT until RX completes. Otherwise marks the request done.
void tx_sending_state(){
    ASYNC_LOG("MST_TX", "Sending cmd=%u reply=%u len=%u", tx_current_packet.command, tx_current_packet.reply, tx_current_packet.length);
    ASYNC_LOG_HEX("MST_TX", "Raw: ", raw_tx_buffer, PACKET_TOTAL_SIZE);

    esp_err_t err = i2c_master_transmit(slave_device, raw_tx_buffer, PACKET_TOTAL_SIZE, I2C_REQUEST_TIMEOUT);
    
    if(err != ESP_OK){
        ASYNC_LOG("MST_TX", "Transmit failed: %d", err);
        request_err = err;
        request_error_state = true;
        request_complete = true;
        tx_state = TX_STATE_ERROR;
    }

    if(tx_current_packet.reply != REPLY_NONE){
        rx_state = RX_STATE_RECEIVING;
        tx_state = TX_STATE_WAIT;
    }
    else{
        request_error_state = false;
        request_complete = true; 
        tx_state = TX_STATE_CLEANUP;
    }
}

/// @brief Blocks additional transmissions while waiting for an outstanding reply.
///        RX cleanup will move TX to CLEANUP when the reply arrives.
void tx_waiting_state(){
    if(rx_state == RX_STATE_IDLE) tx_state = TX_STATE_CLEANUP;
    if(rx_state == RX_STATE_ERROR) tx_state = TX_STATE_ERROR;
}

/// @brief Zeroes the TX buffers, returns to idle, and signals tx_complete_sem.
void tx_cleanup_state(){
    memset(raw_tx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&tx_current_packet, 0, sizeof(data_packet_t));
    tx_state = TX_STATE_IDLE;
    tx_ready = false;
    xSemaphoreGive(tx_complete_sem);
}

/// @brief Resets the I2C bus after a TX error and transitions to cleanup.
void tx_error_state(){
    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_master_get_bus_handle(I2C_PORT, &bus_handle);
    i2c_master_bus_reset(bus_handle);
    request_err = ESP_FAIL;
    request_error_state = true;
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
        case TX_STATE_WAIT:
            tx_waiting_state();
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

/// @brief FreeRTOS task that continuously runs the TX state machine.
/// @param arg Unused (required by FreeRTOS task signature).
void i2c_master_tx_task(void *arg){

    while(1){
        tx_state_machine();
        vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY));
    }
}



// ─── Chunked Request ────────────────────────────────────────────────

/// @brief Sends a payload that exceeds PACKET_PAYLOAD_SIZE by splitting it into chunks.
///        Protocol: CMD_START_CHUNKING -> N data chunks -> CMD_END_CHUNKING.
///        Each chunk is acknowledged with CMD_OK before the next is sent.
///        For REPLY_CHUNK, the response is left for i2c_make_request() to handle.
/// @param command        The command byte stamped on each data chunk.
/// @param reply_type     Reply type sent with CMD_END_CHUNKING.
/// @param payload        Pointer to the full outbound data buffer.
/// @param payload_length Total number of bytes in payload.
/// @return ESP_OK on success, ESP_ERR_TIMEOUT, or ESP_FAIL.
esp_err_t make_chunked_request(i2c_command_t command, reply_type_t reply_type, uint8_t *payload, uint32_t payload_length){

    chunking = true;
    uint16_t current_chunk = 0;
    uint16_t data_offset = 0;
    uint16_t number_of_chunks = bytes_to_chunks(payload_length);
    ASYNC_LOG("MST_CHUNK", "Start TX chunking: %u chunks, %lu bytes", number_of_chunks, payload_length);
    tx_current_packet.command = CMD_START_CHUNKING;
    tx_current_packet.length = number_of_chunks;
    tx_current_packet.reply = REPLY_OK;
    memset(&tx_current_packet.data, 0, sizeof(PACKET_PAYLOAD_SIZE));

    request_complete = false;
    tx_ready = true;

    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
        chunking = false;
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
        chunking = false;
        return ESP_ERR_TIMEOUT;
    }

    if(request_complete == true && request_error_state == true){
        chunking = false;
        return ESP_FAIL;
    }

    while(current_chunk < number_of_chunks){

            current_chunk++;
            request_complete = false;
            tx_current_packet.command = command;
            tx_current_packet.reply = REPLY_OK;

            // Calculate how much data remains for this chunk
            if(data_offset + PACKET_PAYLOAD_SIZE > payload_length){
                tx_current_packet.length = payload_length - data_offset;
            }
            else{
                tx_current_packet.length = PACKET_PAYLOAD_SIZE;
            }
            memcpy(tx_current_packet.data, payload + data_offset, tx_current_packet.length);
         
            data_offset += tx_current_packet.length;
            ASYNC_LOG("MST_CHUNK", "TX chunk %u/%u (%u bytes)", current_chunk, number_of_chunks, tx_current_packet.length);

            tx_ready = true;

            if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                chunking = false;
                return ESP_ERR_TIMEOUT;
            }

            if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                chunking = false;
                return ESP_ERR_TIMEOUT;
            }

            if (request_error_state == true) {
                chunking = false;
                return ESP_FAIL;
            }
    }

    // All chunks sent -- send the end marker with the final reply type
    ASYNC_LOG("MST_CHUNK", "TX chunking complete");
    tx_current_packet.command = CMD_END_CHUNKING;
    tx_current_packet.length = 0;
    tx_current_packet.reply = reply_type;
    request_complete = false;
    tx_ready = true;
    
    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
        chunking = false;
        saved_chunked_response_valid = false;
        return ESP_ERR_TIMEOUT;
    }

    // Wait for the slave's response to the end marker (unless REPLY_NONE)
    if (tx_current_packet.reply != REPLY_NONE) {
        if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
            chunking = false;
            saved_chunked_response_valid = false;
            return ESP_ERR_TIMEOUT;
        }
    }

    if(request_complete == true && request_error_state == true){
        chunking = false;
        saved_chunked_response_valid = false;
        return ESP_FAIL;
    }

    // For REPLY_CHUNK the response handling is deferred to i2c_make_request()
    if (reply_type == REPLY_CHUNK) {
        chunking = false;
        return ESP_OK;
    }
    
    chunking = false;
    return ESP_OK;
}



// ─── Public Request API ─────────────────────────────────────────────

/// @brief Sends a command to a slave and optionally waits for a response.
///        Payloads larger than PACKET_PAYLOAD_SIZE are automatically chunked.
///        For REPLY_DATA and REPLY_CHUNK the caller must free *response_data.
/// @param command         The command byte to send.
/// @param payload         Pointer to the outbound data (may be NULL if payload_length is 0).
/// @param payload_length  Number of bytes in payload.
/// @param slave_id        7-bit I2C address of the target slave.
/// @param reply_type      Expected reply: REPLY_NONE, REPLY_OK, REPLY_DATA, or REPLY_CHUNK.
/// @param response_data   [out] Heap-allocated response buffer (caller frees). May be NULL for REPLY_NONE/OK.
/// @param response_length [out] Number of bytes written to *response_data. May be NULL for REPLY_NONE/OK.
/// @return ESP_OK on success, or an appropriate esp_err_t on failure.
esp_err_t i2c_make_request(i2c_command_t command, uint8_t *payload, uint32_t payload_length, uint8_t slave_id, reply_type_t reply_type, uint8_t **response_data, uint32_t *response_length){
    ASYNC_LOG("MST_REQ", "Request cmd=%u reply=%u len=%lu slave=0x%02X", command, reply_type, payload_length, slave_id);

    esp_err_t error_check;
    
    i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT,
        .device_address = slave_id,
        .scl_speed_hz = I2C_BUS_SPEED,
        .scl_wait_us = I2C_SCL_WAIT_US
        };

    error_check = i2c_master_bus_add_device(bus_handle, &device, &slave_device);
    if(error_check != ESP_OK){
        return ESP_FAIL;
    }

    if (payload_length > MAX_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_SIZE;
        }

    
    if (reply_type == REPLY_DATA || reply_type == REPLY_CHUNK) {
        if (response_data == NULL) {
            return ESP_ERR_INVALID_ARG;
        }
        if (response_length == NULL) {
            return ESP_ERR_INVALID_ARG;
        }
        *response_data = NULL;
        *response_length = 0;
    } else {
        if (response_length != NULL) {
            *response_length = 0;
        }
    }

    if(reply_type == NULL) reply_type = REPLY_NONE;
    if(reply_type != REPLY_CHUNK) {
        reply_chunking = false;
    }
    // reply_chunking is only set true when we actually receive CMD_START_CHUNKING,
    // not here -- setting it too early blocks RX cleanup during outbound chunking.
    

    // CMD_START_CHUNKING uses the length field to carry the chunk count, so
    // a NULL payload with non-zero length is valid in that case.
    if((payload == NULL && payload_length > 0 && command != CMD_START_CHUNKING) || (payload != NULL && payload_length == 0)){
        return ESP_ERR_INVALID_ARG;
    }
    
    
    // Route to chunked path if payload exceeds a single packet
    if (payload_length > PACKET_PAYLOAD_SIZE) {

        esp_err_t err = make_chunked_request(command, reply_type, payload, payload_length);
        
        if (err != ESP_OK) {
            return err;
        }
        
        // For REPLY_CHUNK, restore the saved CMD_START_CHUNKING packet
        // (cleanup may have cleared rx_current_packet by now)
        if (reply_type == REPLY_CHUNK) {
            if(request_error_state == true){
                reply_chunking = false;
                saved_chunked_response_valid = false;
                return request_err;
            }
            
            if(saved_chunked_response_valid) {
                memcpy(&rx_current_packet, &saved_chunked_response_packet, sizeof(data_packet_t));
                saved_chunked_response_valid = false;
            }
            
            if(rx_current_packet.command != CMD_START_CHUNKING) {
                reply_chunking = false;
                saved_chunked_response_valid = false;
                return ESP_ERR_INVALID_RESPONSE;
            }
            
            reply_chunking = true;
        }
    }
    else{
        tx_current_packet.length = payload_length;
        memcpy(tx_current_packet.data, payload, payload_length); 
        
        tx_current_packet.command = command;
        tx_current_packet.reply = reply_type;
        request_complete = false;
        tx_ready = true;

        if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }

        if (tx_current_packet.reply != REPLY_NONE) {
            if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                return ESP_ERR_TIMEOUT;
            }
        } 
    }
    
    

    if(request_error_state == true){
        return request_err;
    } 
    else{
        // ── Handle REPLY_DATA ──
        if(reply_type == REPLY_DATA){
            if (response_data != NULL && response_length != NULL) {
                if(saved_packet_data_valid){
                    *response_data = (uint8_t*)malloc(saved_packet_length + 1);
                    if (*response_data == NULL) {
                        saved_packet_data_valid = false;
                        return ESP_ERR_NO_MEM;
                    }
                    memcpy(*response_data, saved_packet_data, saved_packet_length);
                    *response_length = saved_packet_length;
                    saved_packet_data_valid = false;
                } else {
                    // Fallback -- should not normally reach here
                    *response_data = (uint8_t*)malloc(rx_current_packet.length + 1);
                    if (*response_data == NULL) {
                        return ESP_ERR_NO_MEM;
                    }
                    memcpy(*response_data, rx_current_packet.data, rx_current_packet.length);
                    *response_length = (uint32_t)rx_current_packet.length;
                }
            }
        }

        // ── Handle REPLY_CHUNK ──
        if(reply_type == REPLY_CHUNK){
            uint16_t number_of_chunks;
            uint8_t *chunked_data;
            uint32_t incoming_data_length;
            
            // Restore saved packet in case cleanup ran before we got here
            if(saved_chunked_response_valid) {
                memcpy(&rx_current_packet, &saved_chunked_response_packet, sizeof(data_packet_t));
                saved_chunked_response_valid = false;
            }

            if(rx_current_packet.command == CMD_START_CHUNKING){
                reply_chunking = true;
                number_of_chunks = rx_current_packet.length;
                ASYNC_LOG("MST_CHUNK", "Start RX chunking: %u chunks expected", number_of_chunks);
                
                // Handle empty (0-chunk) response
                if(number_of_chunks == 0) {
                    *response_data = (uint8_t*)malloc(1);
                    if (*response_data == NULL) {
                        reply_chunking = false;
                        return ESP_ERR_NO_MEM;
                    }
                    (*response_data)[0] = 0;
                    *response_length = 0;
                    
                    xSemaphoreGive(chunk_reply_read_sem);
                    
                    // Acknowledge CMD_START_CHUNKING
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_CHUNK;
                    request_complete = false;
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        free(*response_data);
                        *response_data = NULL;
                        reply_chunking = false;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    // Wait for CMD_END_CHUNKING
                    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        free(*response_data);
                        *response_data = NULL;
                        reply_chunking = false;
                        saved_chunked_response_valid = false;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    if(saved_chunked_response_valid) {
                        memcpy(&rx_current_packet, &saved_chunked_response_packet, sizeof(data_packet_t));
                        saved_chunked_response_valid = false;
                    }
                    
                    if(rx_current_packet.command != CMD_END_CHUNKING) {
                        free(*response_data);
                        *response_data = NULL;
                        reply_chunking = false;
                        return ESP_ERR_INVALID_RESPONSE;
                    }
                    
                    // Confirm CMD_END_CHUNKING
                    xSemaphoreGive(chunk_reply_read_sem);
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_NONE;
                    request_complete = false;
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        free(*response_data);
                        *response_data = NULL;
                        reply_chunking = false;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    reply_chunking = false;
                    return ESP_OK;
                }
                
                // Normal case: allocate buffer for all incoming chunks
                incoming_data_length = chunks_to_bytes(rx_current_packet.length);
                xSemaphoreGive(chunk_reply_read_sem);
                *response_data = (uint8_t*)malloc(incoming_data_length + 1);
                if (*response_data == NULL) {
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_ERR;
                    tx_current_packet.reply = REPLY_NONE;
                    request_complete = false;
                    tx_ready = true;
                    xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(500));
                    reply_chunking = false;
                    return ESP_ERR_NO_MEM;
                }

                // Acknowledge CMD_START_CHUNKING and request first data chunk
                tx_current_packet.length = 0;
                tx_current_packet.command = CMD_OK;
                tx_current_packet.reply = REPLY_CHUNK;
                request_complete = false;
                tx_ready = true;

                if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
                    *response_data = NULL;
                    response_length = 0;
                    reply_chunking = false;
                    return ESP_ERR_TIMEOUT;
                }

                if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
                    *response_data = NULL;
                    response_length = 0;
                    reply_chunking = false;
                    return ESP_ERR_TIMEOUT;
                }


            }
            else{
                *response_data = NULL;
                response_length = 0;
                reply_chunking = false;
                return ESP_ERR_INVALID_RESPONSE;
            }

            uint16_t chunked_data_length = 0;
            uint16_t current_chunk = 1;

            // ── Chunk receive loop ──
            while(true){
                // Check for premature or normal end-of-chunking
                if(rx_current_packet.command == CMD_END_CHUNKING && current_chunk < number_of_chunks){
                    tx_current_packet.command = CMD_ERR;
                    tx_current_packet.length = 0;
                    tx_ready = true;
                    reply_chunking = false;
                    return ESP_ERR_INVALID_RESPONSE;
                } else if(rx_current_packet.command == CMD_END_CHUNKING){
                    // All chunks received -- confirm and return
                    xSemaphoreGive(chunk_reply_read_sem);
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_NONE;
                    request_complete = false;
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        *response_data = NULL;
                        response_length = 0;
                        reply_chunking = false;
                        return ESP_ERR_TIMEOUT;
                    }
                    reply_chunking = false;
                    *response_length = chunked_data_length;
                    return ESP_OK;
                }
            
                // Guard against overflowing the allocated buffer
                if (chunked_data_length + rx_current_packet.length > incoming_data_length) {
                    *response_data = NULL;
                    tx_current_packet.command = CMD_ERR;
                    tx_current_packet.length = 0;
                    tx_ready = true;
                    reply_chunking = false;
                    return ESP_ERR_INVALID_SIZE;
                }
            
                // Use saved data if available (cleanup may have cleared rx_current_packet)
                uint32_t chunk_length = 0;
                uint8_t* chunk_data = NULL;

                if(saved_packet_data_valid && saved_packet_length > 0){
                    chunk_length = saved_packet_length;
                    chunk_data = saved_packet_data;
                    saved_packet_data_valid = false;
                } else {
                    chunk_length = rx_current_packet.length;
                    chunk_data = rx_current_packet.data;
                }

                if(chunk_length > 0){
                    memcpy((*response_data) + chunked_data_length, chunk_data, chunk_length);
                    chunked_data_length += chunk_length;
                    ASYNC_LOG("MST_CHUNK", "RX chunk %u/%u (%lu bytes)", current_chunk, number_of_chunks, chunk_length);
                }
                xSemaphoreGive(chunk_reply_read_sem);
                
                if(current_chunk == number_of_chunks) {
                    // Last data chunk processed -- acknowledge and wait for CMD_END_CHUNKING
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_CHUNK;
                    request_complete = false;
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        *response_data = NULL;
                        reply_chunking = false;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        *response_data = NULL;
                        reply_chunking = false;
                        saved_chunked_response_valid = false;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    if(saved_chunked_response_valid) {
                        memcpy(&rx_current_packet, &saved_chunked_response_packet, sizeof(data_packet_t));
                        saved_chunked_response_valid = false;
                    }
                    
                    if(rx_current_packet.command != CMD_END_CHUNKING) {
                       xSemaphoreGive(chunk_reply_read_sem);
                       tx_current_packet.command = CMD_ERR;
                       tx_current_packet.length = 0;
                       tx_current_packet.reply = REPLY_NONE;
                       request_complete = false;
                       tx_ready = true;
                       xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT));
                       
                       reply_chunking = false;
                        return ESP_ERR_INVALID_RESPONSE;
                    }
                    
                    // Confirm CMD_END_CHUNKING
                    xSemaphoreGive(chunk_reply_read_sem);
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_NONE;
                    request_complete = false;
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        *response_data = NULL;
                        response_length = 0;
                        reply_chunking = false;
                        return ESP_ERR_TIMEOUT;
                    }
                    reply_chunking = false;
                    *response_length = chunked_data_length;
                    return ESP_OK;
                }
                else{
                    // More chunks expected -- acknowledge and wait for the next one
                    current_chunk++;
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_CHUNK;
                    request_complete = false;
                    tx_ready = true;
                
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        *response_data = NULL;
                        reply_chunking = false;
                        return ESP_ERR_TIMEOUT;
                    }
                
                    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        *response_data = NULL;
                        reply_chunking = false;
                        return ESP_ERR_TIMEOUT;
                    }
                }
            }


        }



        // Brief delay on fire-and-forget sends to give the slave time to process
        vTaskDelay(pdMS_TO_TICKS(10));
        ASYNC_LOG("MST_REQ", "Complete cmd=%u result=ESP_OK", command);
        return ESP_OK;
    }
}



// ─── Task & Init ────────────────────────────────────────────────────

/// @brief FreeRTOS task that polls for tx_ready, assembles the packet, and triggers the TX state machine.
/// @param arg Unused (required by FreeRTOS task signature).
void execute_command_task(void *arg) {
    while (1) {
        if (tx_state == TX_STATE_IDLE && tx_ready == true) {
                assemble_transmit_buffer(&tx_current_packet, raw_tx_buffer);
                tx_state = TX_STATE_SENDING;
                tx_ready = false;
        }
        vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY));
    }
}



/// @brief Initialises the I2C master bus, buffers, and synchronisation primitives.
/// @param bus_config Caller-provided bus configuration (pins, speed, etc.).
/// @return ESP_OK on success, ESP_FAIL if bus creation or reset fails.
esp_err_t i2c_master_protocol_init(i2c_master_bus_config_t *bus_config){
    memset(raw_rx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(raw_tx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&rx_current_packet, 0, sizeof(data_packet_t));
    memset(&tx_current_packet, 0, sizeof(data_packet_t));

    esp_err_t err = i2c_new_master_bus(bus_config, &bus_handle);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    err = i2c_master_bus_reset(bus_handle);
    if (err != ESP_OK) {
        i2c_del_master_bus(bus_handle);
        return ESP_FAIL;
    }

    tx_complete_sem = xSemaphoreCreateBinary();
    if (tx_complete_sem == NULL) {
        return ESP_FAIL;
    }

    rx_complete_sem = xSemaphoreCreateBinary();
    if (rx_complete_sem == NULL) {
        return ESP_FAIL;
    }

    chunk_reply_read_sem = xSemaphoreCreateBinary();
    if (chunk_reply_read_sem == NULL) {
        return ESP_FAIL;
    }

    ASYNC_LOG_INIT();

    return ESP_OK;
}

/// @brief Entry-point FreeRTOS task: initialises the master protocol and spawns the
///        RX, TX, and command-processor sub-tasks.
/// @param arg Pointer to a caller-owned i2c_master_bus_config_t. Suspends on failure.
void i2c_master_task(void *arg)
{
    i2c_master_bus_config_t *bus_config = (i2c_master_bus_config_t *)arg;
    if (!bus_config) {
        vTaskSuspend(NULL);
        return;
    }
    esp_err_t err = i2c_master_protocol_init(bus_config);
    if (err != ESP_OK) {
        vTaskSuspend(NULL);
        return;
    }

    BaseType_t task_created;

    task_created = xTaskCreatePinnedToCore(i2c_master_rx_task, "i2c_rx",  I2C_RX_TASK_STACK_SIZE, NULL, I2C_COMMS_TASK_PRIORITY, &rx_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));

    task_created = xTaskCreatePinnedToCore(i2c_master_tx_task, "i2c_tx",  I2C_TX_TASK_STACK_SIZE, NULL, I2C_COMMS_TASK_PRIORITY, &tx_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));

    task_created = xTaskCreatePinnedToCore(execute_command_task, "i2c_proc",  I2C_PROC_TASK_STACK_SIZE, NULL, I2C_PROC_TASK_PRIORITY, &command_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));

    while(1){
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

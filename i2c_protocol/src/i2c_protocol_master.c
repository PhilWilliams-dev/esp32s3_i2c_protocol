
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "i2c_protocol_shared.h"
#include <math.h>

//Logging Tags
static const char *TXLOG = "I2C_TX"; 
static const char *RXLOG = "I2C_RX"; 
static const char *I2CLOG = "I2C_Master"; 
static const char *PROCLOG = "I2C_Processing"; 

//Handles
static TaskHandle_t rx_task_handle = NULL;
static TaskHandle_t tx_task_handle = NULL;
static TaskHandle_t command_task_handle = NULL;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t slave_device;

//rx storage
static uint8_t raw_rx_buffer[PACKET_TOTAL_SIZE];
static data_packet_t rx_current_packet;
// Saved packet for REPLY_CHUNK when cleanup clears rx_current_packet before i2c_make_request() can read it
static data_packet_t saved_chunked_response_packet;
static bool saved_chunked_response_valid = false;
// Saved packet data for REPLY_DATA and chunked responses
static uint8_t saved_packet_data[PACKET_PAYLOAD_SIZE];
static uint32_t saved_packet_length = 0;
static bool saved_packet_data_valid = false;
//tx storage
static uint8_t raw_tx_buffer[PACKET_TOTAL_SIZE];
static uint32_t raw_tx_buffer_length = 0;
static data_packet_t tx_current_packet;

//Start the state machine.
static tx_state_t tx_state = TX_STATE_IDLE;
static rx_state_t rx_state = RX_STATE_IDLE;

//Flags
static bool tx_ready = false;
static bool rx_complete = false;
static bool chunking = false;
static bool reply_chunking = false;
// Chunking state for logging
static uint16_t current_chunk_num = 0;
static uint16_t total_chunks_count = 0;

static bool request_complete = false;
static bool request_error_state = false;
static esp_err_t request_err;

static SemaphoreHandle_t tx_complete_sem = NULL;
static SemaphoreHandle_t rx_complete_sem = NULL;
static SemaphoreHandle_t chunk_reply_read_sem = NULL;





/// @brief Is called by the state machine when a reply is expected.  This will make a call to the remote device and retieve the data in the devices output buffer.  This is then processed into our internal packet structure. 
void rx_receiving_state(){
    ASYNC_LOG("DEBUG", "RX: rx_receiving_state() called, tx_current_packet.reply=%u, rx_state=%u", tx_current_packet.reply, rx_state);
    
    esp_err_t err;

    if(tx_current_packet.reply == REPLY_OK && !reply_chunking){
        ASYNC_LOG("DEBUG", "RX: About to receive 1 byte for CMD_OK");
        err = i2c_master_receive(slave_device, raw_rx_buffer, 1, I2C_REQUEST_TIMEOUT);
        ASYNC_LOG("DEBUG", "RX: CMD_OK Reply");
    }
    else{
        ASYNC_LOG("DEBUG", "RX: About to receive %u bytes for non-CMD_OK reply", PACKET_TOTAL_SIZE);
        err = i2c_master_receive(slave_device, raw_rx_buffer, PACKET_TOTAL_SIZE, I2C_REQUEST_TIMEOUT);
        ASYNC_LOG("DEBUG", "RX: Non CMD_OK Reply");
    }

    

    //Reset everything if it errors
    if(err != ESP_OK){
        ASYNC_LOG("DEBUG", "RX: Failed with %s", esp_err_to_name(err));
        request_err = err;
        request_error_state = true;
        request_complete = true;
        rx_state = RX_STATE_ERROR;
        return;
    }

    //Decode the packet into our internal structure
    err = decode_received_packet(raw_rx_buffer, PACKET_TOTAL_SIZE, &rx_current_packet);
    ASYNC_LOG("DEBUG", "RX: Decoded Packet");
    ASYNC_LOG("DEBUG", "RX: Command is %u", rx_current_packet.command);
    ASYNC_LOG("DEBUG", "RX: Reply is %u", rx_current_packet.reply);
    ASYNC_LOG("DEBUG", "RX: Length is %u", rx_current_packet.length);
    ASYNC_LOG("DEBUG", "RX: Data is:");
    

    //Reset everything if it fails
    if(err != ESP_OK){
        ASYNC_LOG("DEBUG", "RX: Decode Error: %s", esp_err_to_name(err));
        request_err = err;
        request_error_state = true;
        request_complete = true;
        rx_state = RX_STATE_CLEANUP; //Was error but testing something
        return;
    }

    // Log received packet details with chunking context
    log_packet_details("RX: Received", raw_rx_buffer, &rx_current_packet, current_chunk_num, total_chunks_count, true);

    // If we received CMD_START_CHUNKING or CMD_END_CHUNKING, save it before cleanup clears it
    // This is needed for REPLY_CHUNK responses where the packet needs to be read after cleanup
    if(rx_current_packet.command == CMD_START_CHUNKING || rx_current_packet.command == CMD_END_CHUNKING){
        memcpy(&saved_chunked_response_packet, &rx_current_packet, sizeof(data_packet_t));
        saved_chunked_response_valid = true;
        ASYNC_LOG("DEBUG", "RX: Saved %s (cmd=%u, chunks=%u) before cleanup", 
                  rx_current_packet.command == CMD_START_CHUNKING ? "CMD_START_CHUNKING" : "CMD_END_CHUNKING",
                  rx_current_packet.command, rx_current_packet.length);
    }

    // Save packet data and length before cleanup clears rx_current_packet
    // This is needed for REPLY_DATA and chunked responses
    if(rx_current_packet.length > 0 && rx_current_packet.length <= PACKET_PAYLOAD_SIZE){
        saved_packet_length = rx_current_packet.length;
        memcpy(saved_packet_data, rx_current_packet.data, saved_packet_length);
        saved_packet_data_valid = true;
        ASYNC_LOG("DEBUG", "RX: Saved packet data (len=%u) before cleanup", saved_packet_length);
    }

    rx_state = RX_STATE_PROCESSING;
}

//Cleans up the RX side and the TX side as there is no way to be here if the TX side did not ask for a response.
void rx_cleanup_state(){
    //If we are chunking then wait for the rx data to be read before continuing
    if(reply_chunking){
        request_complete = true; 
        xSemaphoreGive(rx_complete_sem);
        ASYNC_LOG("DEBUG", "RX: Paused for data to be read");
        ASYNC_LOG("DEBUG", "RX: About to block on chunk_reply_read_sem, rx_state will be set to IDLE after this");
        tx_state = TX_STATE_CLEANUP;
        xSemaphoreTake(chunk_reply_read_sem, pdTICKS_TO_MS(1000));
        ASYNC_LOG("DEBUG", "RX: Pause Released");
    }
    memset(raw_rx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&rx_current_packet, 0, sizeof(data_packet_t));
    request_complete = true; 
    rx_state = RX_STATE_IDLE;
    ASYNC_LOG("DEBUG", "RX: Cleanup complete, rx_state set to IDLE");
    
    //If chunking we already did this but needed to read the data before it was cleared
    if(!reply_chunking){
        xSemaphoreGive(rx_complete_sem);
        tx_state = TX_STATE_CLEANUP;
    }
    
    // Note: saved_packet_data_valid is NOT cleared here - it's cleared in i2c_make_request()
    // when the data is actually used. This ensures the saved data is available when
    // i2c_make_request() wakes up after taking rx_complete_sem.
    
    ASYNC_LOG("DEBUG", "RX: Cleaned up");
}

/// @brief Triggers a bus reset and then points the state machine to cleanup
void rx_state_error(){
    ASYNC_LOG("DEBUG", "RX: Request ended in error.  Resetting bus");
    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_master_get_bus_handle(I2C_PORT, &bus_handle);
    i2c_master_bus_reset(bus_handle);
    //delay to make sure that other parts notice the error state.
    request_err = ESP_FAIL;
    request_error_state = true;
    rx_state = RX_STATE_CLEANUP;
}

/// @brief Mainly only here for any specific cases that we want to handle outside od the requesting function.  For example the CMD_ERR.  Otherwise just sets the complete flag and shuts down the rx side.
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

/// @brief State machine for the RX Side
void rx_state_machine(){
    switch(rx_state){
        case RX_STATE_IDLE:
            break;
        case RX_STATE_PROCESSING:
            rx_processing_state();
            break;
        case RX_STATE_RECEIVING:
            ASYNC_LOG("DEBUG", "RX: State machine sees RX_STATE_RECEIVING, tx_current_packet.reply=%u", tx_current_packet.reply);
            rx_receiving_state();
            break;
        case RX_STATE_CLEANUP:
            rx_cleanup_state();
            break;
        case RX_STATE_ERROR:
            rx_state_error();
            break;
        default:
            ASYNC_LOG("DEBUG", "RX: Unhandled State %u", rx_state);
            rx_state = RX_STATE_CLEANUP;
            break;
    }
}

/// @brief Freertos task for the RX side of the protocol 
void i2c_master_rx_task(void *arg){
    while(1){
        rx_state_machine();
        vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY));
    }//End of while loop
    ASYNC_LOG("DEBUG", "I2C Comms RX Task ended");
    vTaskSuspend(NULL); //Should nev er end here
}








/// @brief does the actual sending of requests to the remote device and manages the setup for any reply if one is required. 
void tx_sending_state(){
 
    esp_err_t err = i2c_master_transmit(slave_device, raw_tx_buffer, PACKET_TOTAL_SIZE, I2C_REQUEST_TIMEOUT);
    
    
    if(err != ESP_OK){
        //There was an error sending the request, mark the request complete with an error flag and reset
        ASYNC_LOG("DEBUG", "TX: Transmit failed with %s", esp_err_to_name(err));
        request_err = err;
        request_error_state = true;
        request_complete = true;
        tx_state = TX_STATE_ERROR;
    }
    else{
        // Log sent packet details with chunking context
        log_packet_details("TX: Sent", raw_tx_buffer, &tx_current_packet, current_chunk_num, total_chunks_count, false);
    }

    if(tx_current_packet.reply != REPLY_NONE){
        //There is a response expected so we need to kick off the RX side, we will mark the request complete there. If all is successfull then in the RX_Cleanup
        ASYNC_LOG("DEBUG", "TX: Reply Expected, kicking off RX");
        ASYNC_LOG("DEBUG", "TX: Setting rx_state=RX_STATE_RECEIVING, tx_current_packet.reply=%u", tx_current_packet.reply);
        rx_state = RX_STATE_RECEIVING;
        tx_state = TX_STATE_WAIT; //Head to wait to stop any additional TX requests while we wait for the response
        ASYNC_LOG("DEBUG", "TX: rx_state set, tx_state=TX_STATE_WAIT");
    }
    else{
        //No response so mark the request compete
        request_error_state = false;
        request_complete = true; 
        tx_state = TX_STATE_CLEANUP;
        ASYNC_LOG("DEBUG", "TX: No Reply Expected, marking request complete");
    }
}

/// @brief Puts the TX side in a blocked state to make sure that no additional TX requests can be made while a reply is outstanding.  This means that if a reply is indicated that the TX side cleanup is triggered by the RX side when it's complete. 
void tx_waiting_state(){
    if(rx_state == RX_STATE_IDLE) tx_state = TX_STATE_CLEANUP; // Clean up the TX side ones the RX is idle  *** testing we do this in the RX cleanup
    if(rx_state == RX_STATE_ERROR) tx_state = TX_STATE_ERROR; //Do a full clean up if the RX side errors.
}

/// @brief Cleans up the TX side making it ready for next use 
void tx_cleanup_state(){
    reply_type_t tx_reply_type = tx_current_packet.reply;
    memset(raw_tx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&tx_current_packet, 0, sizeof(data_packet_t));
    tx_state = TX_STATE_IDLE;
    tx_ready = false;
    xSemaphoreGive(tx_complete_sem);
    //if(tx_reply_type == REPLY_NONE) xSemaphoreGive(rx_complete_sem); //Done so the rx side gets released when no reply is expected.
    ASYNC_LOG("DEBUG", "TX: Cleaned up");
}

/// @brief Triggers a bus reset and then points the state machine to cleanup 
void tx_error_state(){
    ASYNC_LOG("DEBUG", "TX: Request ended in error.  Resetting bus");
    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_master_get_bus_handle(I2C_PORT, &bus_handle);
    i2c_master_bus_reset(bus_handle);
    //delay to make sure that other parts notice the error state.
    request_err = ESP_FAIL;
    request_error_state = true;
    tx_state = TX_STATE_CLEANUP;
}

///@brief State machine for the TX Side
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
            ASYNC_LOG("DEBUG", "TX: No config for state %u", tx_state);
            tx_cleanup_state();
    }
}

///@brief Freertos task for the TX Side
void i2c_master_tx_task(void *arg){

    while(1){
        tx_state_machine();
        vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY));
    }//End of while loop
    ASYNC_LOG("DEBUG", "I2C Comms TX Task ended");
    vTaskSuspend(NULL); //Should nev er end here

}



esp_err_t make_chunked_request(i2c_command_t command, reply_type_t reply_type, uint8_t *payload, uint32_t payload_length){

    chunking = true;
    uint16_t current_chunk = 0;
    uint16_t data_offset = 0;
    uint16_t number_of_chunks = bytes_to_chunks(payload_length);
    // Set chunking state for logging
    total_chunks_count = number_of_chunks;
    current_chunk_num = 0;
    tx_current_packet.command = CMD_START_CHUNKING;
    tx_current_packet.length = number_of_chunks;
    tx_current_packet.reply = REPLY_OK;
    memset(&tx_current_packet.data, 0, sizeof(PACKET_PAYLOAD_SIZE)); //Make sure the sendng buffer is all zero

    request_complete = false; //set the flag to hold interest of the caller till we are done
    tx_ready = true; //Pull the trigger

    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
        ASYNC_LOG("DEBUG", "Chunking: Timeout waiting for start-chunk completion");
        // Reset chunking state on timeout
        chunking = false;
        current_chunk_num = 0;
        total_chunks_count = 0;
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
        ASYNC_LOG("DEBUG", "Chunking: Timeout waiting for start-chunk TX cleanup");
        // Reset chunking state on timeout
        chunking = false;
        current_chunk_num = 0;
        total_chunks_count = 0;
        return ESP_ERR_TIMEOUT;
    }





    if(request_complete == true && request_error_state == true){
        //There was an error to bit out
        ASYNC_LOG("DEBUG", "Chunking: There was an error returned by the slave in response to the start chunk command");
        // Reset chunking state on error
        chunking = false;
        current_chunk_num = 0;
        total_chunks_count = 0;
        return ESP_FAIL;
    }

    ASYNC_LOG("DEBUG", "Chunking: Start Chunk Sent OK");

    while(current_chunk < number_of_chunks){ //NOTE:  Needs to timeut and reset

            current_chunk++;
            ASYNC_LOG("DEBUG", "Chunking: About to send chunk %u of %u", current_chunk, number_of_chunks);
            //Ready for the next packet
            request_complete = false;
            tx_current_packet.command = command;
            tx_current_packet.reply = REPLY_OK;
            if(data_offset + PACKET_PAYLOAD_SIZE > payload_length){
                //Not a whole packet left
                tx_current_packet.length = payload_length - data_offset;
            }
            else{
                tx_current_packet.length = PACKET_PAYLOAD_SIZE;
            }
            memcpy(tx_current_packet.data, payload + data_offset, tx_current_packet.length);
         
            data_offset += tx_current_packet.length;

            // Update chunk number for logging
            current_chunk_num = current_chunk;
            ASYNC_LOG("DEBUG", "Chunking: Sending chunk %u of %u that was %u bytes payload", current_chunk, number_of_chunks, tx_current_packet.length);
            tx_ready = true;

            // Wait for TX to complete first (this triggers RX state machine)
            if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                ASYNC_LOG("DEBUG", "Chunking: Timeout waiting for chunk %u TX cleanup", current_chunk);
                // Reset chunking state on timeout
                chunking = false;
                current_chunk_num = 0;
                total_chunks_count = 0;
                return ESP_ERR_TIMEOUT;
            }

            // Now wait for RX to complete (slave's OK response)
            if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                ASYNC_LOG("DEBUG", "Chunking: Timeout waiting for chunk %u completion", current_chunk);
                // Reset chunking state on timeout
                chunking = false;
                current_chunk_num = 0;
                total_chunks_count = 0;
                return ESP_ERR_TIMEOUT;
            }

            if (request_error_state == true) {
                ASYNC_LOG("DEBUG", "Chunking: Error during chunk %u transmission", current_chunk);
                // Reset chunking state on error
                chunking = false;
                current_chunk_num = 0;
                total_chunks_count = 0;
                return ESP_FAIL;
            }

        //}

        //vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY)); 
        //}

        //while(tx_state != TX_STATE_IDLE){
        //Wait for the first packet to send
        //vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY));
    }

    tx_current_packet.command = CMD_END_CHUNKING;
    tx_current_packet.length = 0;
    tx_current_packet.reply = reply_type;
    request_complete = false; //set the flag to hold interest of the caller till we are done
    // Reset chunk number for end chunking command (not a data chunk)
    current_chunk_num = 0;
    tx_ready = true; //Pull the trigger
    ASYNC_LOG("DEBUG", "Chunking: Just sent the End Chunk Command");
    
    // Wait for TX to complete first
    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
        ASYNC_LOG("DEBUG", "Chunking: Timeout waiting for end-chunk TX cleanup");
        // Reset chunking state on timeout
        chunking = false;
        current_chunk_num = 0;
        total_chunks_count = 0;
        saved_chunked_response_valid = false;
        return ESP_ERR_TIMEOUT;
    }

        // For all reply types (including REPLY_CHUNK), wait for response
    // REPLY_CHUNK response will be handled by i2c_make_request() after we return
    if (tx_current_packet.reply != REPLY_NONE) {  // If reply expected, wait for RX
        if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
            // Reset chunking state on timeout
            chunking = false;
            current_chunk_num = 0;
            total_chunks_count = 0;
            saved_chunked_response_valid = false;
            return ESP_ERR_TIMEOUT;
        }
    }

    // Check for errors
    if(request_complete == true && request_error_state == true){
        //There was an error to bit out
        ASYNC_LOG("DEBUG", "Chunking: There was an error returned by the slave in response to the end chunk command");
        // Reset chunking state on error
        chunking = false;
        current_chunk_num = 0;
        total_chunks_count = 0;
        saved_chunked_response_valid = false;
        return ESP_FAIL;
    }

     // For REPLY_CHUNK, don't process the response here - let i2c_make_request() handle it
    // For other reply types, we've already received and processed the response
    if (reply_type == REPLY_CHUNK) {
        // Packet was already saved in rx_receiving_state() when CMD_START_CHUNKING was decoded
        // Just verify it was saved correctly
        if(saved_chunked_response_valid && saved_chunked_response_packet.command == CMD_START_CHUNKING){
            ASYNC_LOG("DEBUG", "Chunking: CMD_START_CHUNKING response already saved (cmd=%u, chunks=%u)", saved_chunked_response_packet.command, saved_chunked_response_packet.length);
        } else {
            ASYNC_LOG("DEBUG", "Chunking: WARNING - CMD_START_CHUNKING not saved! saved_valid=%d, cmd=%u", saved_chunked_response_valid, saved_chunked_response_valid ? saved_chunked_response_packet.command : 0);
        }
        chunking = false;
        current_chunk_num = 0;
        total_chunks_count = 0;
        return ESP_OK;
    }
    
    // Reset chunking state on successful completion
    chunking = false;
    current_chunk_num = 0;
    total_chunks_count = 0;
    return ESP_OK;
}




esp_err_t i2c_make_request(i2c_command_t command, uint8_t *payload, uint32_t payload_length, uint8_t slave_id, reply_type_t reply_type, uint8_t **response_data, uint32_t *response_length){

    esp_err_t error_check;
    
    i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT,
        .device_address = slave_id,
        .scl_speed_hz = I2C_BUS_SPEED,
        .scl_wait_us = I2C_SCL_WAIT_US
        };

    //Make sure that the device handle can be created first.
    error_check = i2c_master_bus_add_device(bus_handle, &device, &slave_device);
    if(error_check != ESP_OK){
        ASYNC_LOG("DEBUG", "Failed to create device handle for slave id: %u with the error %s", slave_id, esp_err_to_name(error_check));
        return ESP_FAIL;
    }

    //Is the payload too big?
    if (payload_length > MAX_PAYLOAD_SIZE) {
        ASYNC_LOG("DEBUG", "Payload too large to be handled: %lu vs %d", payload_length, MAX_PAYLOAD_SIZE);
        return ESP_ERR_INVALID_SIZE;
        }

    
    if (reply_type == REPLY_DATA || reply_type == REPLY_CHUNK) {
        if (response_data == NULL) {  // Check the pointer-to-pointer itself
            ASYNC_LOG("DEBUG", "response_data is NULL but required for reply_type %d", reply_type);
            return ESP_ERR_INVALID_ARG;
        }
        if (response_length == NULL) {  // Existing check for length pointer
            ASYNC_LOG("DEBUG", "response_length is NULL but required for reply_type %d", reply_type);
            return ESP_ERR_INVALID_ARG;
        }
        // Optional: Initialize caller's pointer to NULL before allocation (safety)
        *response_data = NULL;
        *response_length = 0;
    } else {
        // For non-data replies, allow NULL and skip allocation later
        if (response_length != NULL) {
            *response_length = 0;  // Set default for caller
        }
    }

    if(reply_type == NULL) reply_type = REPLY_NONE; //If there is a NULL for reply type then replace it wish a REPLY_NONE
    if(reply_type != REPLY_CHUNK) {
        reply_chunking = false; // This ensures it's reset if a non-chunked request follows a chunked one
        // Reset chunking logging state for non-chunked requests
        current_chunk_num = 0;
        total_chunks_count = 0;
    }
    // Don't set reply_chunking = true here - it should only be set when we actually receive CMD_START_CHUNKING
    // Setting it too early causes RX cleanup to block during outbound chunking
    

    if((payload == NULL && payload_length > 0 && command != CMD_START_CHUNKING) || (payload != NULL && payload_length == 0)){ //Bypass for start chunking because we use the length field to tell the remote end whow many chunks there are.
        ASYNC_LOG("DEBUG", "Data parameters are invalid, your payload lengths don't match.  e.g. you have a NULL payload but have specified a non-zero length");
        return ESP_ERR_INVALID_ARG;
    }
    
    
    //This is where the TX chunking choice is made
    if (payload_length > PACKET_PAYLOAD_SIZE) {
        ASYNC_LOG("DEBUG", "Payload too large for a single packet: %lu vs %d  Starting Chunking", payload_length, PACKET_PAYLOAD_SIZE);

        esp_err_t err = make_chunked_request(command, reply_type, payload, payload_length);
        ASYNC_LOG("DEBUG", "Got back from make chunked request");
        
        // Check for errors from chunked request
        if (err != ESP_OK) {
            return err;
        }
        
        // For REPLY_CHUNK, make_chunked_request() already waited for CMD_START_CHUNKING
        // Restore the saved packet (cleanup may have cleared rx_current_packet)
        if (reply_type == REPLY_CHUNK) {
            // Check for errors
            if(request_error_state == true){
                ASYNC_LOG("DEBUG", "PROC: Error state after chunked request");
                reply_chunking = false;
                saved_chunked_response_valid = false;
                current_chunk_num = 0;
                total_chunks_count = 0;
                return request_err;
            }
            
            // Restore the saved packet (cleanup may have cleared rx_current_packet)
            if(saved_chunked_response_valid) {
                memcpy(&rx_current_packet, &saved_chunked_response_packet, sizeof(data_packet_t));
                saved_chunked_response_valid = false;
                ASYNC_LOG("DEBUG", "PROC: Restored saved CMD_START_CHUNKING response (cmd=%u)", rx_current_packet.command);
            }
            
            // Verify we got CMD_START_CHUNKING (saved by make_chunked_request())
            if(rx_current_packet.command != CMD_START_CHUNKING) {
                ASYNC_LOG("DEBUG", "PROC: Expected CMD_START_CHUNKING but got command %u", rx_current_packet.command);
                reply_chunking = false;
                saved_chunked_response_valid = false;
                current_chunk_num = 0;
                total_chunks_count = 0;
                return ESP_ERR_INVALID_RESPONSE;
            }
            
            // Now set reply_chunking = true since we're about to receive chunks
            // This will cause RX cleanup to pause until data is read
            reply_chunking = true;
            ASYNC_LOG("DEBUG", "PROC: Received CMD_START_CHUNKING, setting reply_chunking=true, entering chunked receive logic");
        }
    }
    else{
        //Data parameters are valid
        tx_current_packet.length = payload_length;
        memcpy(tx_current_packet.data, payload, payload_length); 
        
        tx_current_packet.command = command;
        tx_current_packet.reply = reply_type;
        request_complete = false; //set the flag to hold interest of the caller till we are done
        ASYNC_LOG("DEBUG", "About to go the normal 40 byte route");
        tx_ready = true; //Pull the trigger

        if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {  // Wait for TX send
            return ESP_ERR_TIMEOUT;
        }

        if (tx_current_packet.reply != REPLY_NONE) {  // If reply expected, wait for RX
            if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                return ESP_ERR_TIMEOUT;
            }
            ASYNC_LOG("XX", "TEST: Data length is %u", rx_current_packet.length);
        } 
        ASYNC_LOG("XX", "TEST2: Data length is %u", rx_current_packet.length);
    }
    
    

    //Deal with the output of the request if there is any.
    if(request_error_state == true){
        return request_err; //Returning Error
    } 
    else{
        //Deal with standard Data packet coming back
        if(reply_type == REPLY_DATA){
            if (response_data != NULL && response_length != NULL) {
                if(saved_packet_data_valid){
                    // Use saved data (cleanup cleared rx_current_packet)
                    *response_data = (uint8_t*)malloc(saved_packet_length + 1);
                    if (*response_data == NULL) {
                        saved_packet_data_valid = false;
                        return ESP_ERR_NO_MEM;
                    }
                    ASYNC_LOG("DEBUGHUNT", "PROC: Copying saved data to response data");
                    ASYNC_LOG("DEBUGHUNT", "PROC: Saved data length is %u", saved_packet_length);
                    memcpy(*response_data, saved_packet_data, saved_packet_length);
                    *response_length = saved_packet_length;
                    saved_packet_data_valid = false;
                } else {
                    // Fallback (shouldn't happen)
                    *response_data = (uint8_t*)malloc(rx_current_packet.length + 1);
                    if (*response_data == NULL) {
                        return ESP_ERR_NO_MEM;
                    }
                    ASYNC_LOG("DEBUGHUNT", "PROC: Copying data to response data (fallback)");
                    ASYNC_LOG("DEBUGHUNT", "PROC: Data length is %u", rx_current_packet.length);
                    memcpy(*response_data, rx_current_packet.data, rx_current_packet.length);
                    *response_length = (uint32_t)rx_current_packet.length;
                }
            }
        }

        //Deal with a larger response coming back.
        if(reply_type == REPLY_CHUNK){
            ASYNC_LOG("DEBUG", "PROC: We are expecting to get a chunked response");
            uint16_t number_of_chunks;
            
            uint8_t *chunked_data;
            uint32_t incoming_data_length;

            // Check if we already have CMD_START_CHUNKING (from chunked request path)
            // or if we need to wait for it (from single packet path)
            if(rx_current_packet.command == CMD_START_CHUNKING){
                // Set reply_chunking = true since we're about to receive chunks
                // This will cause RX cleanup to pause until data is read
                reply_chunking = true;
                number_of_chunks = rx_current_packet.length;
                
                // Special case: Handle 0 chunks (empty response)
                if(number_of_chunks == 0) {
                    ASYNC_LOG("DEBUG", "PROC: Received START_CHUNKING with 0 chunks (empty response)");
                    // Set chunking state for logging
                    total_chunks_count = 0;
                    current_chunk_num = 0;
                    
                    // Allocate a minimal buffer (1 byte with value 0)
                    *response_data = (uint8_t*)malloc(1);
                    if (*response_data == NULL) {
                        ASYNC_LOG("DEBUG", "PROC: Failed to allocate memory for empty chunked response");
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_NO_MEM;
                    }
                    (*response_data)[0] = 0;  // Initialize to 0
                    *response_length = 0;    // Return 0 length
                    
                    // Wait for CMD_END_CHUNKING to complete the protocol
                    xSemaphoreGive(chunk_reply_read_sem);
                    
                    // Send CMD_OK to acknowledge
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_CHUNK;
                    request_complete = false;
                    current_chunk_num = 0;
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout waiting for CMD_OK to send (0 chunks)");
                        free(*response_data);
                        *response_data = NULL;
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    // Wait for CMD_END_CHUNKING
                    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout waiting for CMD_END_CHUNKING (0 chunks)");
                        free(*response_data);
                        *response_data = NULL;
                        reply_chunking = false;
                        saved_chunked_response_valid = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    // Restore the saved packet (cleanup may have cleared rx_current_packet)
                    if(saved_chunked_response_valid) {
                        memcpy(&rx_current_packet, &saved_chunked_response_packet, sizeof(data_packet_t));
                        saved_chunked_response_valid = false;
                        ASYNC_LOG("DEBUG", "PROC: Restored saved CMD_END_CHUNKING response (cmd=%u) for 0 chunks case", rx_current_packet.command);
                    }
                    
                    // Verify we received CMD_END_CHUNKING
                    if(rx_current_packet.command != CMD_END_CHUNKING) {
                        ASYNC_LOG("DEBUG", "PROC: Expected CMD_END_CHUNKING but received command %u (0 chunks)", rx_current_packet.command);
                        free(*response_data);
                        *response_data = NULL;
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_INVALID_RESPONSE;
                    }
                    
                    // Confirm END_CHUNKING
                    xSemaphoreGive(chunk_reply_read_sem);
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_NONE;
                    request_complete = false;
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout waiting for final CMD_OK to send (0 chunks)");
                        free(*response_data);
                        *response_data = NULL;
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    // Successfully completed 0-chunk response
                    reply_chunking = false;
                    current_chunk_num = 0;
                    total_chunks_count = 0;
                    ASYNC_LOG("DEBUG", "PROC: Successfully handled 0-chunk response");
                    return ESP_OK;
                }
                
                // Normal case: number_of_chunks > 0
                incoming_data_length = chunks_to_bytes(rx_current_packet.length);
                // Set chunking state for logging
                total_chunks_count = number_of_chunks;
                current_chunk_num = 0;
                ASYNC_LOG("DEBUG", "PROC: We received a start chunking answer that told us to expect %u packets", number_of_chunks);
                ASYNC_LOG("DEBUG", "PROC: Allocating %u bytes memory to hold the data", incoming_data_length + 1);
                xSemaphoreGive(chunk_reply_read_sem);
                uint32_t max_required_size = rx_current_packet.length;
                *response_data = (uint8_t*)malloc(incoming_data_length + 1);  // +1 for potential null terminator
                if (*response_data == NULL) {
                    ASYNC_LOG("DEBUG", "PROC: Failed to allocate %u bytes for chunked response", incoming_data_length + 1);
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_ERR;
                    tx_current_packet.reply = REPLY_NONE;
                    request_complete = false;
                    tx_ready = true;
                    // Wait briefly for CMD_ERR to send (optional)
                    xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(500));
                    // Reset chunking state on error
                    reply_chunking = false;
                    current_chunk_num = 0;
                    total_chunks_count = 0;
                    return ESP_ERR_NO_MEM;
                }

                //Send back the OK
                tx_current_packet.length = 0;
                tx_current_packet.command = CMD_OK;
                tx_current_packet.reply = REPLY_CHUNK;
                request_complete = false; //set the flag to hold interest of the caller till we are done
                ASYNC_LOG("DEBUG", "PROC: Asking for First Chunk");
                tx_ready = true; //Pull the trigger

                //wait for the responce 
                if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(500)) != pdTRUE) {  // Wait for TX send
                    ASYNC_LOG("DEBUG", "PROC: Timeout Waiting for first CMD_OK to send");
                    *response_data = NULL;
                    response_length = 0;
                    // Reset chunking state on timeout
                    reply_chunking = false;
                    current_chunk_num = 0;
                    total_chunks_count = 0;
                    return ESP_ERR_TIMEOUT;
                }

                if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(500)) != pdTRUE) { //Wait for the RX to arrive
                    ASYNC_LOG("DEBUG", "PROC: Timeout Waiting for First Chunk to Arrive");
                    *response_data = NULL;
                    response_length = 0;
                    // Reset chunking state on timeout
                    reply_chunking = false;
                    current_chunk_num = 0;
                    total_chunks_count = 0;
                    return ESP_ERR_TIMEOUT;
                }


            }
            else{
                *response_data = NULL;
                response_length = 0;
                // Reset chunking state on invalid response
                reply_chunking = false;
                current_chunk_num = 0;
                total_chunks_count = 0;
                return ESP_ERR_INVALID_RESPONSE; //We asked for chunks and gote something else
            }

            uint16_t chunked_data_length = 0;
            uint16_t current_chunk = 1;
            // Align logging with processing: first chunk is 1
            current_chunk_num = current_chunk;

            while(true){
                // Packet was already logged when received in rx_receiving_state()
                // Check for CMD_END_CHUNKING FIRST, before processing
                if(rx_current_packet.command == CMD_END_CHUNKING && current_chunk < number_of_chunks){
                    // Error: End arrived too early
                    ASYNC_LOG("DEBUG", "PROC: Not enough chunks Arrived!  We have received %u packets of a total of %u", current_chunk, number_of_chunks);
                    tx_current_packet.command = CMD_ERR;
                    tx_current_packet.length = 0;
                    tx_ready = true;
                    // Reset chunking state on error
                    reply_chunking = false;
                    current_chunk_num = 0;
                    total_chunks_count = 0;
                    return ESP_ERR_INVALID_RESPONSE;
                } else if(rx_current_packet.command == CMD_END_CHUNKING){
                    // Normal end - we've received all chunks
                    xSemaphoreGive(chunk_reply_read_sem);
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_NONE;
                    request_complete = false;
                    ASYNC_LOG("DEBUG", "PROC: Confirming the End of Chunking");
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout Waiting for Final CMD_OK to send");
                        *response_data = NULL;
                        response_length = 0;
                        // Reset chunking state on timeout
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                    reply_chunking = false;
                    // Reset chunking logging state on successful completion
                    current_chunk_num = 0;
                    total_chunks_count = 0;
                    ASYNC_LOG("DEBUG", "PROC: the length of the chunked packet is %u", chunked_data_length);
                    *response_length = chunked_data_length;
                    return ESP_OK;
                }
            
                // Validate we haven't exceeded expected data
                if (chunked_data_length + rx_current_packet.length > incoming_data_length) {
                    ASYNC_LOG("DEBUG", "PROC: Slave sent more data than promised during chunk %u of %u", current_chunk, number_of_chunks);
                    *response_data = NULL;
                    tx_current_packet.command = CMD_ERR;
                    tx_current_packet.length = 0;
                    tx_ready = true;
                    // Reset chunking state on error
                    reply_chunking = false;
                    current_chunk_num = 0;
                    total_chunks_count = 0;
                    return ESP_ERR_INVALID_SIZE;
                }
            
                // Process the chunk - use saved data if available (cleanup may have cleared rx_current_packet)
                uint32_t chunk_length = 0;
                uint8_t* chunk_data = NULL;

                if(saved_packet_data_valid && saved_packet_length > 0){
                    chunk_length = saved_packet_length;
                    chunk_data = saved_packet_data;
                    saved_packet_data_valid = false; // Clear after use
                    ASYNC_LOG("DEBUG", "PROC: Using saved chunk data (len=%u)", chunk_length);
                } else {
                    chunk_length = rx_current_packet.length;
                    chunk_data = rx_current_packet.data;
                    ASYNC_LOG("DEBUG", "PROC: Using rx_current_packet data (len=%u)", chunk_length);
                }

                if(chunk_length > 0){
                    memcpy((*response_data) + chunked_data_length, chunk_data, chunk_length);
                    chunked_data_length += chunk_length;
                    ASYNC_LOG("DEBUG", "PROC: Data length is %u", chunk_length);
                    ASYNC_LOG("DEBUG", "PROC: Processed chunk %u of %u, chunked_data_length now = %u", current_chunk, number_of_chunks, chunked_data_length);
                } else {
                    ASYNC_LOG("DEBUG", "PROC: WARNING - chunk_length is 0 for chunk %u", current_chunk);
                }
                xSemaphoreGive(chunk_reply_read_sem);
                ASYNC_LOG("DEBUG", "PROC: Processed chunk %u of %u", current_chunk, number_of_chunks);
                
                // Check if this was the last chunk BEFORE incrementing
                if(current_chunk == number_of_chunks) {
                    // We've processed all chunks, now send CMD_OK and wait for CMD_END_CHUNKING
                    ASYNC_LOG("DEBUG", "PROC: Processed last chunk (%u), sending CMD_OK and waiting for CMD_END_CHUNKING", number_of_chunks);
                    
                    // Send CMD_OK to acknowledge the last chunk
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_CHUNK;
                    request_complete = false;
                    current_chunk_num = current_chunk; // Update for logging
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout Waiting for CMD_OK to send (after last chunk)");
                        *response_data = NULL;
                        // Reset chunking state on timeout
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    // Now wait for CMD_END_CHUNKING
                    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout Waiting for CMD_END_CHUNKING after last chunk");
                        *response_data = NULL;
                        // Reset chunking state on timeout
                        reply_chunking = false;
                        saved_chunked_response_valid = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                    
                    // Restore the saved packet (cleanup may have cleared rx_current_packet)
                    if(saved_chunked_response_valid) {
                        memcpy(&rx_current_packet, &saved_chunked_response_packet, sizeof(data_packet_t));
                        saved_chunked_response_valid = false;
                        ASYNC_LOG("DEBUG", "PROC: Restored saved CMD_END_CHUNKING response (cmd=%u) after last chunk", rx_current_packet.command);
                    }
                    
                    // Check that we received CMD_END_CHUNKING
                    if(rx_current_packet.command != CMD_END_CHUNKING) {
                        ASYNC_LOG("DEBUG", "PROC: Expected CMD_END_CHUNKING but received command %u", rx_current_packet.command);
                       // Release the semaphore so RX cleanup can proceed
                       xSemaphoreGive(chunk_reply_read_sem);
                        
                       // Send CMD_ERR to notify slave
                       tx_current_packet.command = CMD_ERR;
                       tx_current_packet.length = 0;
                       tx_current_packet.reply = REPLY_NONE;
                       request_complete = false;
                       tx_ready = true;
                       
                       // Wait for TX to complete
                       if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                           ASYNC_LOG("DEBUG", "PROC: Timeout waiting for CMD_ERR to send");
                       }
                       
                       // Reset chunking flag and logging state
                       reply_chunking = false;
                       current_chunk_num = 0;
                       total_chunks_count = 0;
                        return ESP_ERR_INVALID_RESPONSE;
                    }
                    
                    // We received CMD_END_CHUNKING, confirm it
                    xSemaphoreGive(chunk_reply_read_sem);
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_NONE;
                    request_complete = false;
                    ASYNC_LOG("DEBUG", "PROC: Confirming the End of Chunking");
                    tx_ready = true;
                    
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout Waiting for Final CMD_OK to send");
                        *response_data = NULL;
                        response_length = 0;
                        // Reset chunking state on timeout
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                    reply_chunking = false;
                    // Reset chunking logging state on successful completion
                    current_chunk_num = 0;
                    total_chunks_count = 0;
                    ASYNC_LOG("DEBUG", "PROC: About to return, chunked_data_length = %u, current_chunk = %u, number_of_chunks = %u", chunked_data_length, current_chunk, number_of_chunks);
                    ASYNC_LOG("DEBUG", "PROC: the length of the chunked packet is %u", chunked_data_length);
                    *response_length = chunked_data_length;
                    return ESP_OK;
                }
                else{
                    // Not the last chunk, increment and ask for next
                    current_chunk++;
                    current_chunk_num = current_chunk; // Update for next chunk logging
                    // Not the last chunk, ask for next chunk
                    tx_current_packet.length = 0;
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.reply = REPLY_CHUNK;
                    request_complete = false;
                    ASYNC_LOG("DEBUG", "PROC: Asking for chunk %u of %u", current_chunk, number_of_chunks);
                    tx_ready = true;
                
                    if (xSemaphoreTake(tx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout Waiting for CMD_OK to send");
                        *response_data = NULL;
                        // Reset chunking state on timeout
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                
                    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                        ASYNC_LOG("DEBUG", "PROC: Timeout Waiting for Next Chunk to Arrive");
                        *response_data = NULL;
                        // Reset chunking state on timeout
                        reply_chunking = false;
                        current_chunk_num = 0;
                        total_chunks_count = 0;
                        return ESP_ERR_TIMEOUT;
                    }
                }
            }


        }//End of REPLY_CHUNK if statement




        vTaskDelay(pdMS_TO_TICKS(10)); //Makes sure that on super fast blind sends there is a bit of a delay for the slave to get it's house in order.  This value was tested for to be the most reliable.
        return ESP_OK;
    }

    

    //Clean up
    //i2c_master_bus_rm_device(slave_device);
}




/// @brief Waits for the tx_ready flag and then copies the tx_current_packet into the raw_tx_buffer before triggering the TX state machine to send it
void execute_command_task(void *arg) {
    while (1) {
        if (tx_state == TX_STATE_IDLE && tx_ready == true) {
                raw_tx_buffer_length = tx_current_packet.length;
                assemble_transmit_buffer(&tx_current_packet, raw_tx_buffer);
                tx_state = TX_STATE_SENDING;
                tx_ready = false; //reset the trigger
        }
        vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY));
    }//Main task loop end
}





/// @brief Is called by the main task and sets up the master bus and other structures. 
esp_err_t i2c_master_protocol_init(i2c_master_bus_config_t *bus_config){
    //Clear the  buffers
    memset(raw_rx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(raw_tx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&rx_current_packet, 0, sizeof(data_packet_t));
    memset(&tx_current_packet, 0, sizeof(data_packet_t));
    //memset(&current, 0, sizeof(i2c_request_t));
    //current.command = -1;

    esp_err_t err = i2c_new_master_bus(bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(I2CLOG, "Master Bus Create Failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    err = i2c_master_bus_reset(bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(I2CLOG, "Master Bus Reset Failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(bus_handle);
        return ESP_FAIL;
    }
    ESP_LOGI(I2CLOG, "Master Bus Created");

    tx_complete_sem = xSemaphoreCreateBinary();
    if (tx_complete_sem == NULL) {
        ESP_LOGE(PROCLOG, "Failed to create TX semaphore, terminating tasks");
        return ESP_FAIL;
    }

    rx_complete_sem = xSemaphoreCreateBinary();
    if (rx_complete_sem == NULL) {
        ESP_LOGE(PROCLOG, "Failed to create RX semaphore, terminating tasks");
        return ESP_FAIL;
    }

    chunk_reply_read_sem = xSemaphoreCreateBinary();
    if (chunk_reply_read_sem == NULL) {
        ESP_LOGE(PROCLOG, "Failed to create RX semaphore, terminating tasks");
        return ESP_FAIL;
    }

    // Initialize async logging
    async_log_init();

    return ESP_OK;
}

/// @brief Freertos task that Manages the other parts of the protocol.
void i2c_master_task(void *arg)
{
    esp_log_level_set(TXLOG, ESP_LOG_WARN);  // Set to DEBUG level
    esp_log_level_set(RXLOG, ESP_LOG_WARN);  // Set to DEBUG level
    esp_log_level_set(I2CLOG, ESP_LOG_WARN);  // Set to DEBUG level
    esp_log_level_set(PROCLOG, ESP_LOG_WARN);

    i2c_master_bus_config_t *bus_config = (i2c_master_bus_config_t *)arg;
    if (!bus_config) {
        ESP_LOGE(I2CLOG, "Invalid bus configuration");
        vTaskSuspend(NULL);
        return;
    }
    esp_err_t err = i2c_master_protocol_init(bus_config);
    if (err != ESP_OK) {
        ESP_LOGE(I2CLOG, "Protocol initialization failed");
        vTaskSuspend(NULL);
        return;
    }

    BaseType_t task_created;

    task_created = xTaskCreatePinnedToCore(i2c_master_rx_task, "i2c_rx",  I2C_RX_TASK_STACK_SIZE, NULL, I2C_COMMS_TASK_PRIORITY, &rx_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));
    ESP_LOGI(I2CLOG, "I2C RX Manager Started");

    task_created = xTaskCreatePinnedToCore(i2c_master_tx_task, "i2c_tx",  I2C_TX_TASK_STACK_SIZE, NULL, I2C_COMMS_TASK_PRIORITY, &tx_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));
    ESP_LOGI(I2CLOG, "I2C TX Manager Started");

    task_created = xTaskCreatePinnedToCore(execute_command_task, "i2c_proc",  I2C_PROC_TASK_STACK_SIZE, NULL, I2C_PROC_TASK_PRIORITY, &command_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));
    ESP_LOGI(I2CLOG, "I2C Command Manager Started");

    while(1){

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    
}//end main

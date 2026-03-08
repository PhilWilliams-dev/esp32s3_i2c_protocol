#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_slave.h"
#include "driver/i2c_types.h"
#include <freertos/projdefs.h>
#include "i2c_protocol_slave.h"
#include "i2c_protocol_shared.h"
#include "esp_heap_caps.h"
// #include "esp_heap_trace.h"  // Commented out due to API compatibility issues

//Logging Tags
static const char *TXLOG = "I2C_TX"; 
static const char *RXLOG = "I2C_RX"; 
static const char *I2CLOG = "I2C_Slave"; 
static const char *PROCLOG = "I2C_Processing"; 

//Handles
static TaskHandle_t rx_task_handle = NULL;
static TaskHandle_t tx_task_handle = NULL;
static TaskHandle_t command_task_handle = NULL;
static i2c_slave_dev_handle_t slave_handle;

//rx storage
static uint8_t raw_rx_buffer[PACKET_TOTAL_SIZE];
static uint32_t raw_rx_buffer_length = 0;
static data_packet_t rx_current_packet;
uint8_t *chunked_data;
uint16_t chunked_data_length = 0;
// Chunking state for logging
static uint16_t current_chunk_num = 0;
static uint16_t total_chunks_count = 0;

//tx storage
static uint8_t raw_tx_buffer[PACKET_TOTAL_SIZE];
static data_packet_t tx_current_packet;

//Start the state machine.
static tx_state_t tx_state = TX_STATE_IDLE;
static rx_state_t rx_state = RX_STATE_IDLE;

//Flags
static bool tx_ready = false;
static bool rx_complete = false;
static bool chunking = false;


//Command Registry
command_registry_item_t command_registry[MAX_COMMANDS];
static uint32_t slave_registry_count = 0;

static SemaphoreHandle_t rx_complete_sem = NULL;
static SemaphoreHandle_t tx_ready_sem = NULL;
static SemaphoreHandle_t rx_wake_sem = NULL;
static SemaphoreHandle_t tx_wake_sem = NULL;




esp_err_t i2c_register_command(command_registry_item_t *reg) { 
    if (slave_registry_count >= MAX_COMMANDS) return ESP_ERR_NO_MEM;
    command_registry[slave_registry_count++] = *reg;
    return ESP_OK;
}

/// @brief Callback for when a master makes a request to send data to us.
static bool i2c_slave_rx_cb(i2c_slave_dev_handle_t handle, const i2c_slave_rx_done_event_data_t *evt_data, void *arg) {
    //ESP_EARLY_LOGD(RXLOG, "rx Callback triggered by master request");
    if(rx_state == RX_STATE_IDLE){
        ASYNC_LOG("DEBUG", "ISR_RX: Received %lu bytes, rx_state=IDLE", (unsigned long)evt_data->length);
        memcpy(raw_rx_buffer, evt_data->buffer, evt_data->length);
        raw_rx_buffer_length = evt_data->length;
        rx_complete = false; //This holds the processor at bay untill we set this true later.  Makes sure that the packet is recived before we look at it
        rx_state = RX_STATE_RECEIVING;
        portMEMORY_BARRIER();
        BaseType_t higherPrioWoken = pdFALSE;
        xSemaphoreGiveFromISR(rx_wake_sem, &higherPrioWoken);
        ASYNC_LOG("DEBUG", "ISR_RX: Gave rx_wake_sem, higherPrioWoken=%d", higherPrioWoken);
        portYIELD_FROM_ISR(higherPrioWoken);
        return pdFALSE;
    }
    else{
        ASYNC_LOG("DEBUG", "RX: Request speed too fast, sending NACK");
        return pdTRUE;
    }
}

/// @brief Completes the initial receipt of the packet, loads it into a new data object and checks the crc
/// @return 
void rx_receiving_state(){
    esp_err_t decode = decode_received_packet(raw_rx_buffer, raw_rx_buffer_length, &rx_current_packet);

    
    switch(decode){
        case ESP_OK:
            // Log received packet - decoded
            log_packet_details("Received", raw_rx_buffer, &rx_current_packet, current_chunk_num, total_chunks_count, true);
            rx_state = RX_STATE_PROCESSING;
            break;
        case ESP_ERR_INVALID_SIZE:
            ASYNC_LOG("DEBUG", "RX: Inbound packet is not the correct size, clearing buffer");
            rx_state = RX_STATE_CLEANUP;
            tx_state = TX_STATE_ERROR; //Loads in a CMD_ERR for the master to collect if it's asking.
            break;
        case ESP_ERR_INVALID_CRC:
            ASYNC_LOG("DEBUG", "RX: Packet payload failed CRC, clearing buffer");
            rx_state = RX_STATE_CLEANUP;
            tx_state = TX_STATE_ERROR; //Loads in a CMD_ERR for the master to collect if it's asking.
            break;
        case ESP_ERR_INVALID_STATE: // Length exceeds max payload or other issues
            ASYNC_LOG("DEBUG", "RX: Packet payload incorrect, clearing buffer");
            rx_state = RX_STATE_CLEANUP;
            tx_state = TX_STATE_ERROR; //Loads in a CMD_ERR for the master to collect if it's asking.
            break;
        default:
            ASYNC_LOG("DEBUG", "RX: Packet error: %s", esp_err_to_name(decode));
            esp_system_abort("Unhandled RX packet decode error.");
            rx_state = RX_STATE_CLEANUP;
            break;
        }
    return;
}

/// @brief Handles some low level commands but basicly hands off the ready signal to the processing task.
/// @return 
void rx_processing_state(){
    //Here we will manage some base level commands that we know need to further processing for example the reboot command.
    switch(rx_current_packet.command){
        case(CMD_RESTART):
            esp_restart();
            break;
        case(CMD_ERR):
            tx_state = TX_STATE_CLEANUP;
            rx_state = RX_STATE_CLEANUP;
            break;
        default:
            rx_complete = true; //This is the trigger for the command task
            ASYNC_LOG("DEBUG", "RX: rx_complete=true, giving rx_complete_sem, rx_state=%d, cmd=%u", rx_state, (unsigned int)rx_current_packet.command);
            __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
            xSemaphoreGive(rx_complete_sem);
            ASYNC_LOG("DEBUG", "RX: rx_complete_sem given");
            rx_state = RX_STATE_CLEANUP;
            break;
    }
return;
}

/// @brief Cleans up the RX side and makes it ready to start again
/// @return 
void rx_cleanup_state(){
    memset(raw_rx_buffer, 0, PACKET_TOTAL_SIZE);
    raw_rx_buffer_length = 0;
    //The rx_current_packet should be cleaned up by the command process not here.  Otherwise the data is gone by the time you want it.
    rx_state = RX_STATE_IDLE;
    return;
}

/// @brief The state machine that makes the whole RX side function
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

/// @brief Freertos task for the RX side of this protocol
void i2c_slave_rx_task(void *arg){
    while(1){
        
        xSemaphoreTake(rx_wake_sem, pdMS_TO_TICKS(1000));
        while (rx_state != RX_STATE_IDLE) {
            rx_state_machine();
        }
        
    }//End of while loop
    ASYNC_LOG("DEBUG", "I2C Comms RX Task ended");
    assert(false); //Should never end here
}






/// @brief Callback that is triggered when the I2C master makes a request.  Dispite the name of TX the protocol is master driven, e.g. it sends data (RX here) and then requests data back (TX here).  This call back will directly enter a wait cycle. A WAIT COMMAND will be sent causing the master to repeat the request. As soon as the tx_ready flag is set the data will be sent managed in the waiting state function
static bool i2c_slave_tx_cb(i2c_slave_dev_handle_t handle, const i2c_slave_request_event_data_t *evt_data, void *arg) {
    tx_state = TX_STATE_SENDING;
    portMEMORY_BARRIER();
    BaseType_t higherPrioWoken = pdFALSE;
    xSemaphoreGiveFromISR(tx_wake_sem, &higherPrioWoken);
    portYIELD_FROM_ISR(higherPrioWoken);
    return pdFALSE;
}

/// @brief Responsible for the actual 'sending' of the outbound packet.   
void tx_sending_state(){
    
    if(xSemaphoreTake(tx_ready_sem, pdMS_TO_TICKS(10000)) == pdTRUE){ //Will hold for 10 seconds
      //Stuff that only happens if we are ok
      __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
    }
    else{
        //Overwrite the current packet with an error to send back instead
        tx_current_packet.command = CMD_ERR;
        tx_current_packet.length = 0;
    }

    uint32_t writeout = 0;

    esp_err_t txerr;
    if(tx_current_packet.reply == REPLY_OK){
        //Special Case to make OK as short os possible
        uint8_t okpayload[1];
        okpayload[0] = CMD_OK;
        // Async debug: initial/simple OK response byte
        async_log_hex("DEBUG", "TX OK byte: ", okpayload, 1);
        txerr = i2c_slave_write(slave_handle, okpayload, 1, &writeout, I2C_REQUEST_TIMEOUT);
    }
    else{
        //Build and send the outbound packet.
        assemble_transmit_buffer(&tx_current_packet, raw_tx_buffer);
        
        // Log encoded and sent packet
        log_packet_details("Sent", raw_tx_buffer, &tx_current_packet, current_chunk_num, total_chunks_count, false);
        
        txerr = i2c_slave_write(slave_handle, raw_tx_buffer, PACKET_TOTAL_SIZE, &writeout, I2C_REQUEST_TIMEOUT);
    }
    
    
    
    if(txerr != ESP_OK){
        ASYNC_LOG("DEBUG", "TX: Error sending packet: %s.  Resetting buffers.", esp_err_to_name(txerr));
        tx_state = TX_STATE_CLEANUP;
    }
    tx_state = TX_STATE_CLEANUP;
    return;

}

/// @brief Cleans up the TX side and makes it ready to start again
void tx_cleanup_state(){
    memset(raw_tx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&tx_current_packet, 0, sizeof(data_packet_t));
    tx_state = TX_STATE_IDLE;
    return;
}

void tx_error_state(){
    ASYNC_LOG("DEBUG", "TX: Sending CMD_ERR");
    tx_current_packet.command = CMD_ERR;
    tx_current_packet.length = 0;
    __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
    tx_ready = true;
    xSemaphoreGive(tx_ready_sem);
    tx_state = TX_STATE_CLEANUP;
}

/// @brief State mahcine that makes the TX side work
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
            ASYNC_LOG("DEBUG", "TX: No config for state %u", tx_state);
            tx_cleanup_state();
    }
}

/// @brief Freertos task that runs the TX side of the protocol 
void i2c_slave_tx_task(void *arg){
    while(1){
        xSemaphoreTake(tx_wake_sem, pdMS_TO_TICKS(1000));
        while (tx_state != TX_STATE_IDLE) {
            tx_state_machine();
        }
    }//End of while loop
    ASYNC_LOG("DEBUG", "I2C Comms TX Task ended");
    assert(false); //Should nev er end here
}






/// @brief Freertos task that does the actual processing of the data that arrives and provdes the data that will be sent back to the master if reqeusted.  The rx_current_packet must be cleared here when the command processor is finished with the data.
void i2c_slave_execute_command_task(void *arg) {
   

    //Main task loop for the command processor
    main_task_loop:
    
    while (1) {

        //ASYNC_LOG("DEBUG", "CMD: Waiting for rx_complete_sem, rx_complete=%d", rx_complete);
        if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            // Just a timeout – no new packet (normal when idle)
            continue;
        }

        ASYNC_LOG("DEBUG", "CMD: Got rx_complete_sem, rx_complete=%d", rx_complete);
        __sync_synchronize();

        if (!rx_complete) {
            // Semaphore signalled but flag not set: treat as stale/spurious, ignore cleanly
            ASYNC_LOG("DEBUG", "CMD: rx_complete_sem signalled but rx_complete==0, ignoring");
            continue;
        }

        // We have a valid packet - clear the flag IMMEDIATELY to prevent races
        rx_complete = false;  // <-- ADD THIS HERE, at the start of processing
        __sync_synchronize();
        ASYNC_LOG("DEBUG", "CMD: Starting packet processing, cmd=%u", (unsigned int)rx_current_packet.command);
        
        // Log that we're processing this packet
        log_packet_details("Processed", raw_rx_buffer, &rx_current_packet, current_chunk_num, total_chunks_count, true);

        //if (rx_complete == true) { //The RX side has recived something and parced it into a valid packet.  Note that commands like Restart that are embedded are handled in the rx_processing state machine code.
            //ESP_LOGW("DEBUG","LOG 1");
            //used to Keep an internal copy of the rx_packet that started the processing task so it's not lost while waiting or chunking.
            data_packet_t initial_packet;
                

                //Start of Chunking
                if(rx_current_packet.command == CMD_START_CHUNKING){ //We are chunking baby
                    //ESP_LOGW("DEBUG","LOG 2");
                    ASYNC_LOG("DEBUG", "CMD: CMD_START_CHUNKING detected, chunks=%u", (unsigned int)rx_current_packet.length);
                    chunking = true; //Set the protocol into chunking mode
                    uint16_t number_of_chunks = rx_current_packet.length;
                    uint16_t current_chunk = 0;
                    // Set chunking state for logging
                    total_chunks_count = number_of_chunks;
                    current_chunk_num = 0;
                    //uint16_t chunking_offset = 0;

                    chunked_data = malloc((number_of_chunks * PACKET_PAYLOAD_SIZE + 1) * sizeof(uint8_t)); //Allocates the memory to strore the data we are about to be sent.  The +1 is to allow for the use of a null terminator in commands if it is needed.
                    
                    if(chunked_data == NULL){
                        //Allocation of memory failed
                        ASYNC_LOG("DEBUG", "CMD: Failed to allocate memory for chunked data");
                        
                        tx_current_packet.command = CMD_ERR;
                        tx_current_packet.length = 0;
                        __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
                        tx_ready = true;
                        xSemaphoreGive(tx_ready_sem);
                        chunked_data_length = 0;
                        chunking = false;
                        rx_complete = false;
                        memset(&initial_packet, 0, sizeof(data_packet_t));
                        goto main_task_loop;
                    }

                    rx_complete = false; //Se we can get the next packet when it's sent
                    ASYNC_LOG("DEBUG", "CMD: Clearing rx_complete_sem (chunking start), chunking=%d, chunks=%u", chunking, (unsigned int)number_of_chunks);
                    //xSemaphoreTake(rx_complete_sem, 0); //Clear the semaphore
                    ASYNC_LOG("DEBUG", "CMD: Semaphore cleared, sending initial OK");
                    //Send OK back so the master will send the next packet
                    tx_current_packet.command = CMD_OK;
                    tx_current_packet.length = 0;
                    tx_current_packet.reply = REPLY_OK;
                    __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
                    tx_ready = true;
                    xSemaphoreGive(tx_ready_sem);

                    while(1){

                        ASYNC_LOG("DEBUG", "CMD: Waiting for chunk, current_chunk=%u, number_of_chunks=%u", (unsigned int)current_chunk, (unsigned int)number_of_chunks);
                        if(xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(500)) == pdTRUE){
                            ASYNC_LOG("DEBUG", "CMD: Got chunk, command=%u, chunking=%d, current_chunk=%u", (unsigned int)rx_current_packet.command, chunking, (unsigned int)current_chunk);
                            //Keep going till we get and end, we will check the logic in here to send a err back if we get sent more chunks than expected.
                            if (rx_current_packet.command != CMD_END_CHUNKING){ //End chunking is belt and braces in case the end chunk arrives between the two statements.
                                current_chunk++;
                                // Update chunk number for logging
                                current_chunk_num = current_chunk;
                                //ESP_LOGW("DEBUG","LOG 3");
                                if(current_chunk > number_of_chunks){
                                    //Master sent too many chunks
                                    ASYNC_LOG("DEBUG", "CMD: Too many chunks sent!");
                                    tx_current_packet.command = CMD_ERR;
                                    tx_current_packet.length = 0;
                                    __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
                                    tx_ready = true;
                                    xSemaphoreGive(tx_ready_sem);
                                    if(chunking == true) free(chunked_data);
                                    chunked_data_length = 0;
                                    chunking = false;
                                    rx_complete = false;
                                    memset(&initial_packet, 0, sizeof(data_packet_t));
                                    goto main_task_loop;
                                }
                                
                                initial_packet.command = rx_current_packet.command; //Set the actual command that we are planning to execute.

                                memcpy(chunked_data + chunked_data_length, rx_current_packet.data, rx_current_packet.length);
                                chunked_data_length +=rx_current_packet.length;
                                






                                //Send OK back so the master will send the next packet
                                tx_current_packet.command = CMD_OK;
                                tx_current_packet.length = 0;
                                tx_current_packet.reply = REPLY_OK;
                                rx_complete = false; //Se we can get the next packet when it's sent
                                __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
                                tx_ready = true;
                                xSemaphoreGive(tx_ready_sem);

                            }//End of rx_compete if
                            else if(rx_current_packet.command == CMD_END_CHUNKING){
                        //ESP_LOGW("DEBUG","LOG 4");
                        //ESP_LOGI("DEBUG","We are ending Chunking.  rx_complete: %u command: %u", rx_complete, rx_current_packet.command );
                        //we got an end chunking command
                        if(rx_current_packet.command == CMD_END_CHUNKING && current_chunk != number_of_chunks){
                            //We ended before all the chunks arrived.  Kill the chunking.
                            ASYNC_LOG("DEBUG", "CMD: Expected the end chunk command! number of chunks is: %u", current_chunk);
                            tx_current_packet.command = CMD_ERR;
                            tx_current_packet.length = 0;
                            __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
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
                            //ESP_LOGW("DEBUG","LOG 5");
                            //We are correctly ending the chunking process
                            //above we have set the .command to the command that is sent with each chunk and collected the data.  The final end_chunking packet carries the reply_type that the actual command wants.
                            initial_packet.reply = rx_current_packet.reply; 
                            ASYNC_LOG("DEBUG", "CMD: CMD_END_CHUNKING received, command=%u, reply=%d, chunks=%u, data_length=%u", 
                                     (unsigned int)initial_packet.command, initial_packet.reply, 
                                     (unsigned int)current_chunk, (unsigned int)chunked_data_length);
                            
                            // DO NOT free chunked_data here - pass it to the command processor
                            // The processor will use it, and framework will clean up after processing
                            // (except for REPLY_CHUNK where it's cleaned up after transmission)
                            
                            chunking = false;
                            current_chunk = 0;
                            number_of_chunks = 0;
                            
                            // Ensure rx_complete is cleared
                            rx_complete = false;
                            
                            break;
                        }



                    }
                        } 
                        else{
                            ASYNC_LOG("DEBUG", "CMD: TIMEOUT waiting for chunk, current_chunk=%u, number_of_chunks=%u", (unsigned int)current_chunk, (unsigned int)number_of_chunks);
                            tx_current_packet.command = CMD_ERR;
                            tx_current_packet.length = 0;
                            __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);
                            if (chunking == true) free(chunked_data);
                            chunked_data_length = 0;
                            chunking = false;
                            break;
                        }



    
                            //ESP_LOGW("DEBUG","LOG 6");
                            
    
    
                        
                    //vTaskDelay(pdMS_TO_TICKS(I2C_COMMS_MANAGER_DELAY));
                    }//End of while not end of chunking loop
                    ASYNC_LOG("DEBUG", "CMD: Chunking loop ended, chunks received=%u", (unsigned int)current_chunk);
                    //xSemaphoreTake(rx_complete_sem, 0); //Clear the semaphore









           
                }
                else{
                    // Zero initial_packet first to ensure clean state
                    memset(&initial_packet, 0, sizeof(data_packet_t));
                    
                    initial_packet.command = rx_current_packet.command;
                    initial_packet.reply = rx_current_packet.reply;
                    initial_packet.length = rx_current_packet.length;
                    initial_packet.crc = rx_current_packet.crc;
                    
                    memcpy(initial_packet.data, rx_current_packet.data, rx_current_packet.length);
                }

                //A normal packet would end up here first.  The chunked packed should have been handled by now.  For chunked the processosor needs to address the correct storage.

                for (uint32_t i = 0; i < slave_registry_count; i++) {

                if (command_registry[i].command == initial_packet.command) {
                    
                    // Check if we have chunked data from inbound chunking
                    // Note: chunking flag may be false if we just finished receiving chunks,
                    // so check for actual data presence instead
                    if(chunked_data != NULL && chunked_data_length > 0){
                        // Chunked data ready for processing - keep it for the processor
                    }
                    else{
                        // Not chunked receive - clear chunked_data so command processor knows it can allocate
                        chunked_data = NULL;
                        chunked_data_length = 0;
                    }
                    
                    //Run the processor
                    esp_err_t err = command_registry[i].process_command(&initial_packet, &tx_current_packet, &chunked_data, &chunked_data_length);
                    if (err == ESP_OK){
                        //Processor ran OK
                        if(initial_packet.reply == REPLY_OK){
                            //ONly an OK expected so we just set that up
                            
                            tx_current_packet.command = CMD_OK;
                            tx_current_packet.length = 0;
                            tx_current_packet.reply = REPLY_OK;
                            __sync_synchronize(); //Memory Barrier to ensure the data is written before the semaphore is given.
                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);
                            
                            // Clean up chunked_data after processing (not needed for REPLY_OK)
                            if(chunked_data != NULL) {
                                free(chunked_data);
                                chunked_data = NULL;
                            }
                            chunked_data_length = 0;
                        }
                        else if(initial_packet.reply == REPLY_DATA){
                            //Data expected so we just send whatever the command processor setup
                            
                            // Memory barrier to ensure writes are complete before signaling TX task
                            __sync_synchronize();

                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);
                            
                            // Clean up chunked_data after processing (not needed for REPLY_DATA)
                            if(chunked_data != NULL) {
                                free(chunked_data);
                                chunked_data = NULL;
                            }
                            chunked_data_length = 0;
                        }

                        else if(initial_packet.reply == REPLY_CHUNK){
                            //The command is expecting to send back a chunked reply

                            uint16_t current_chunk = 0;
                            uint16_t data_offset = 0;
                            uint16_t number_of_chunks = bytes_to_chunks(chunked_data_length);
                            
                        
                            tx_current_packet.command = CMD_START_CHUNKING;
                            tx_current_packet.length = number_of_chunks;
                            tx_current_packet.reply = REPLY_NONE;
                            memset(&tx_current_packet.data, 0, sizeof(PACKET_PAYLOAD_SIZE)); //Make sure the sendng buffer is all zero
                            // Set chunking state for logging
                            total_chunks_count = number_of_chunks;
                            current_chunk_num = 0;
                            tx_ready = true; //Pull the trigger
                            xSemaphoreGive(tx_ready_sem);

                           

                            if(xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(500))){
                                if(rx_current_packet.command != CMD_OK){
                                    ASYNC_LOG("DEBUG", "CMD: master did not reply OK");
                                }

                                //Send each chunk
                                while(current_chunk < number_of_chunks){

                                    current_chunk++;
                                    //Ready for the next packet
                                    
                                    //Respond OK to the request
                                    tx_current_packet.command = CMD_OK;
                                    tx_current_packet.reply = REPLY_CHUNK;

                                    //Build the next chunks data length
                                    if(data_offset + PACKET_PAYLOAD_SIZE > chunked_data_length){
                                        //Not a whole packet left
                                        tx_current_packet.length = chunked_data_length - data_offset;
                                    }
                                    else
                                    {
                                        tx_current_packet.length = PACKET_PAYLOAD_SIZE;
                                    }



                                    memcpy(tx_current_packet.data, chunked_data + data_offset, tx_current_packet.length);
                                 
                                    data_offset += tx_current_packet.length;
                        
                                    // Update chunk number for logging
                                    current_chunk_num = current_chunk;
                                    tx_ready = true;
                                    xSemaphoreGive(tx_ready_sem);
                                    
                                    //Wait to recived the next packet
                                    if (xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(I2C_REQUEST_TIMEOUT)) != pdTRUE) {
                                        ASYNC_LOG("DEBUG", "CMD: Timeout waiting for chunk %u completion", current_chunk);
                                        rx_complete = false;
                                        memset(&initial_packet, 0, sizeof(data_packet_t));
                                        if(chunked_data != NULL) free(chunked_data);
                                        chunked_data_length = 0;
                                        rx_state = RX_STATE_CLEANUP;
                                        tx_state = TX_STATE_CLEANUP;
                                        xSemaphoreGive(rx_wake_sem);  // Add this to immediately wake RX task for cleanup
                                        xSemaphoreGive(tx_wake_sem);
                                        goto main_task_loop;
                                    }
                                    
                                    //Error out of the process if the master does not say OK
                                    if(rx_current_packet.command != CMD_OK){
                                        ASYNC_LOG("DEBUG", "CMD: master did not reply OK");
                                        rx_complete = false;
                                        memset(&initial_packet, 0, sizeof(data_packet_t));
                                        if(chunked_data != NULL) free(chunked_data);
                                        chunked_data_length = 0;
                                        goto main_task_loop;
                                    }

                                }

                                //Send the end chunking add the true size of the data to the chunk endas the master has no way of knowing.
                                tx_current_packet.command = CMD_END_CHUNKING;
                                tx_current_packet.length = 0;
                                tx_current_packet.reply = REPLY_NONE;
                                current_chunk_num = 0; // Reset for END_CHUNKING
                                tx_ready = true; //Pull the trigger
                                xSemaphoreGive(tx_ready_sem);
    
                               
                                //Wait for the end chunk to be confirmed
                                if(xSemaphoreTake(rx_complete_sem, pdMS_TO_TICKS(500))){
                                    if(rx_current_packet.command != CMD_OK){
                                        ASYNC_LOG("DEBUG", "CMD: master did not reply OK");
                                    }
                                }

                                //Clean up 
                                rx_complete = false;
                                memset(&initial_packet, 0, sizeof(data_packet_t));
                                if(chunked_data != NULL) free(chunked_data);
                                chunked_data_length = 0;
                                rx_state = RX_STATE_CLEANUP;
                                tx_state = TX_STATE_CLEANUP;
                                xSemaphoreGive(rx_wake_sem);  // Add this to immediately wake RX task for cleanup
                                xSemaphoreGive(tx_wake_sem);
                                goto main_task_loop;

                            }//End of the semaphore that is triggered after the start chunking command is ack'd
                            else
                            {
                                //Timeout waiting for first chunk
                                rx_complete = false;
                                memset(&initial_packet, 0, sizeof(data_packet_t));
                                if(chunked_data != NULL) free(chunked_data);
                                chunked_data_length = 0;
                                rx_state = RX_STATE_CLEANUP;
                                tx_state = TX_STATE_CLEANUP;
                                xSemaphoreGive(rx_wake_sem);  // Add this to immediately wake RX task for cleanup
                                xSemaphoreGive(tx_wake_sem);
                                goto main_task_loop;
                            }
                            
                            
                            









                        }//End of the REPLY_CHUNK section
                        else{
                            //If we are here then there is no reply needed (REPLY_NONE).
                            // Clean up chunked_data after processing
                            if(chunked_data != NULL) {
                                free(chunked_data);
                                chunked_data = NULL;
                            }
                            chunked_data_length = 0;
                        }
                    }
                    else {
                        //The command processor failed so we need to generate and error and send one if a reply is expected.
                        ASYNC_LOG("DEBUG", "CMD: Failed to process command %u because %s", initial_packet.command, esp_err_to_name(err));
                        if(initial_packet.reply != REPLY_NONE){
                            tx_current_packet.command = CMD_ERR;
                            tx_current_packet.length = 0;
                            tx_ready = true;
                            xSemaphoreGive(tx_ready_sem);
                            
                        }
                        // Clean up chunked_data after processor failure
                        if(chunked_data != NULL) {
                            free(chunked_data);
                            chunked_data = NULL;
                        }
                        chunked_data_length = 0;
                    }
                    //Clean up 
                    //rx_complete = false;
                    ASYNC_LOG("DEBUG", "CMD: Cleanup complete, rx_complete=false, chunking=%d, going to main_task_loop", chunking);
                    memset(&initial_packet, 0, sizeof(data_packet_t));
                    //memset(&rx_current_packet, 0, sizeof(data_packet_t)); //Clear the current packet so we don't have old data in it.
                    // Note: chunked_data cleanup is handled per-case above (REPLY_OK, REPLY_DATA, REPLY_NONE, or processor failure)
                    // For REPLY_CHUNK, cleanup happens after transmission completes
                    chunking = false;
                    // Ensure semaphore is cleared before waiting for next request
                    ASYNC_LOG("DEBUG", "CMD: Clearing semaphore before main_task_loop");
                    //xSemaphoreTake(rx_complete_sem, 0); // Clear any pending semaphore
                    ASYNC_LOG("DEBUG", "CMD: Semaphore cleared, entering main_task_loop");
                    goto main_task_loop;
                }
            }//End of for loop that looks for a command processor

             //We did not find a command to process this request.
             ASYNC_LOG("DEBUG", "CMD: No handler for command %u", (unsigned int)initial_packet.command);
             ASYNC_LOG("DEBUG", "CMD: No handler for command %u, cleaning up", (unsigned int)initial_packet.command);
             rx_state = RX_STATE_CLEANUP;
             tx_state = TX_STATE_CLEANUP;
             //rx_complete = false;
             // Clean up chunked_data if it exists (no processor to handle it)
             if(chunked_data != NULL) {
                 free(chunked_data);
                 chunked_data = NULL;
             }
             chunked_data_length = 0;
             chunking = false;
             //memset(&rx_current_packet, 0, sizeof(data_packet_t));
             ASYNC_LOG("DEBUG", "CMD: No handler cleanup done, clearing semaphore");
             //xSemaphoreTake(rx_complete_sem, 0);
             ASYNC_LOG("DEBUG", "CMD: No handler - entering main_task_loop");
             xSemaphoreGive(rx_wake_sem);  // Add this to immediately wake RX task for cleanup
             xSemaphoreGive(tx_wake_sem);
             continue;

        //}//is there a request loop
    }
}


/// @brief Is called by the main task and setups up the master bus, loads the command handers etc. 
esp_err_t i2c_protocol_init(uint8_t slaveid){

     //Load in the command handlers and die of it failes.
     esp_err_t err;

    rx_complete_sem = xSemaphoreCreateBinary();
    if (rx_complete_sem == NULL) {
        ESP_LOGE(PROCLOG, "Failed to create RX semaphore, terminating tasks");
        vTaskSuspend(NULL);
    }

    tx_ready_sem = xSemaphoreCreateBinary();
    if (tx_ready_sem == NULL) {
        ESP_LOGE(PROCLOG, "Failed to create TX semaphore, terminating tasks");
        vTaskSuspend(NULL);
    }

    rx_wake_sem = xSemaphoreCreateBinary();
    if (rx_wake_sem == NULL) {
        ESP_LOGE(PROCLOG, "Failed to create RX wake semaphore, terminating tasks");
        vTaskSuspend(NULL);
    }

    tx_wake_sem = xSemaphoreCreateBinary();
    if (tx_wake_sem == NULL) {
        ESP_LOGE(PROCLOG, "Failed to create TX wake semaphore, terminating tasks");
        vTaskSuspend(NULL);
    }

    // Initialize async logging
    async_log_init();

    //Clear the  buffers
    memset(raw_rx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(raw_tx_buffer, 0, PACKET_TOTAL_SIZE);
    memset(&rx_current_packet, 0, sizeof(data_packet_t));
    memset(&tx_current_packet, 0, sizeof(data_packet_t));

    //Setup the i2c slave
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
        ESP_LOGE(I2CLOG, "Slave Create Failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    //tx_current_packet_mutex = xSemaphoreCreateMutex();
    //if (tx_current_packet_mutex == NULL) {
    //ESP_LOGE(I2CLOG, "Failed to create tx_mutex");
    //return ESP_FAIL;
    //}

    err = i2c_slave_register_event_callbacks(slave_handle, &callbacks, NULL);

    if (err != ESP_OK) {
        ESP_LOGE(I2CLOG, "Callbacks Create Failed: %s", esp_err_to_name(err));
        i2c_del_slave_device(slave_handle);
        return ESP_FAIL;
    }
    return ESP_OK;
}


/// @brief Freertos task that Manages the other parts of the protocol.  This is the task that is exposed to callers 
void i2c_slave_task(void *arg){
    esp_log_level_set(TXLOG, ESP_LOG_WARN);  
    esp_log_level_set(RXLOG, ESP_LOG_WARN);  
    esp_log_level_set(I2CLOG, ESP_LOG_WARN);  
    esp_log_level_set(PROCLOG, ESP_LOG_WARN);

    
    uint8_t *slaveid = (uint8_t *)arg;
    if (!slaveid) {
        ESP_LOGE(I2CLOG, "Invalid slave ID");
        vTaskSuspend(NULL);
        return;
    }
    esp_err_t err = i2c_protocol_init(*slaveid);
    if (err != ESP_OK) {
        ESP_LOGE(I2CLOG, "I2C Slave Protocol initialization failed, ending tasks");
        vTaskSuspend(NULL);
        return;
    }

    BaseType_t task_created;

    //This needs to me moved to a calculated value along with the main task when the code is all finished
    #define I2C_COMMS_STACK_SIZE 16000
    #define I2C_COMMS_TASK_PRIORITY 5
    
    task_created = xTaskCreatePinnedToCore(i2c_slave_rx_task, "i2c_rx",  I2C_RX_TASK_STACK_SIZE, NULL, I2C_COMMS_TASK_PRIORITY, &rx_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));
    ESP_LOGI(I2CLOG, "I2C RX Manager Started");

    task_created = xTaskCreatePinnedToCore(i2c_slave_tx_task, "i2c_tx",  I2C_TX_TASK_STACK_SIZE, NULL, I2C_COMMS_TASK_PRIORITY, &tx_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));
    ESP_LOGI(I2CLOG, "I2C TX Manager Started");

    task_created = xTaskCreatePinnedToCore(i2c_slave_execute_command_task, "i2c_PROC",  I2C_PROC_TASK_STACK_SIZE, NULL, I2C_PROC_TASK_PRIORITY, &command_task_handle, 0);
    assert(task_created == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));
    ESP_LOGI(I2CLOG, "I2C Command Manager Started");




    //print_memory_info();
 


    vTaskSuspend(NULL);

//while(1){
    //ESP_LOGW("TXREADY", "%u", tx_ready);
    //ESP_LOGW("RX Complete", "%u", rx_complete);
    //ESP_LOGW("TXSTATE", "%u", tx_state);
    //ESP_LOGW("RXSTATE", "%u", rx_state);
    //ESP_LOGW("CHUNKING", "%u", chunking);
    //ESP_LOGW("DEBUG", "Cycle Count is: %u", debug_counter);
  //  vTaskDelay(pdMS_TO_TICKS(1000));
  //  }
}

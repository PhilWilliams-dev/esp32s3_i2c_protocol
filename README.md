# I2C Protocol Driver

An ESP-IDF component that implements a structured, CRC-protected I2C communication protocol with automatic chunking for large payloads. Supports both master and slave roles (one per build) and handles all framing, error detection, and flow control internally.

**Key features:**

- Fixed-size packets with CRC16 integrity checking
- Automatic chunking of payloads up to 120 KB (configurable, limited by your mavailable memory)
- Command/handler registry pattern for slaves
- Simple request/response API for masters
- All configuration via `idf.py menuconfig`

---

## 1. Adding the Component to Your Project

Copy the `i2c_protocol/` directory into your project's `components/` folder:

```
my_project/
  components/
    i2c_protocol/
      include/
        i2c_protocol_shared.h
        i2c_protocol_slave.h
        i2c_protocol_master.h
      src/
        i2c_protocol_shared.c
        i2c_protocol_slave.c
        i2c_protocol_master.c
      Kconfig
      CMakeLists.txt
  main/
    main.c
    my_commands.h
    my_commands.c
    CMakeLists.txt
```

In your `main/CMakeLists.txt`, add `i2c_protocol` as a dependency:

```cmake
idf_component_register(
    SRCS "main.c" "my_commands.c"
    REQUIRES i2c_protocol
    INCLUDE_DIRS "."
)
```

---

## 2. Configuration

Run `idf.py menuconfig` and navigate to **Component config > I2C Protocol**.

### Role

| Setting | Default | Description |
|---|---|---|
| Protocol role | Slave | Master or Slave (one per build) |

### Hardware

## Required
| Setting | Default | Description |
|---|---|---|
| SDA GPIO pin | 8 | GPIO number for the SDA line |
| SCL GPIO pin | 9 | GPIO number for the SCL line |
| I2C port | I2C_NUM_0 | I2C peripheral (0 or 1) |

## Optional
| Setting | Default | Description |
|---|---|---|
| Bus speed | 100000 Hz | I2C clock frequency |
| 7-bit addressing | Yes | Uncheck for 10-bit addressing |
| SCL wait timeout | 10000000 us | Clock-stretching timeout |

### Optional Protocol Tuning

| Setting | Default | Description |
|---|---|---|
| Payload size per chunk | 40 bytes | Data bytes per I2C packet (1--256) |
| Max total payload | 122880 bytes | Largest single transfer (chunked automatically) |
| Request timeout | 5000 ms | Timeout for I2C transactions |
| Task loop delay | 10 ms | Cycle time for TX/RX state machines |

### Task Configuration

| Setting | Default | Description |
|---|---|---|
| RX / TX / Proc stack size | 4096 bytes each | FreeRTOS stack for internal tasks |
| Comms / Proc priority | 5 each | FreeRTOS task priorities |

### Debug

| Setting | Default | Description |
|---|---|---|
| Async debug logging | Off | Enable ISR-safe background logging |

---

## 3. Slave Implementation

### 3.1 Define Your Commands

Create a commands header (e.g. `my_commands.h`):

```c
#ifndef MY_COMMANDS_H
#define MY_COMMANDS_H

#include "i2c_protocol_shared.h"

// Command IDs must be 1--200.
// IDs 201+ are reserved by the driver (CMD_RESERVED_MIN).
#define CMD_LED_TOGGLE   1
#define CMD_GET_SENSOR   2
#define CMD_SEND_CONFIG  3

// Handler prototypes
esp_err_t handle_led_toggle(
    data_packet_t *packet,
    data_packet_t *response_packet,
    uint8_t **chunked_payload,
    uint16_t *chunked_payload_length
);

esp_err_t handle_get_sensor(
    data_packet_t *packet,
    data_packet_t *response_packet,
    uint8_t **chunked_payload,
    uint16_t *chunked_payload_length
);

#endif
```

### 3.2 Command Handler Signature

Every command handler has the same signature:

```c
esp_err_t handler(
    data_packet_t *packet,           // Incoming packet (command, data, length)
    data_packet_t *response_packet,  // Fill this to send a response
    uint8_t **chunked_payload,       // For receiving/sending large payloads
    uint16_t *chunked_payload_length // Length of chunked data
);
```

**Parameters:**

- `packet` -- the received request. Read `packet->command`, `packet->reply` (the reply type the master expects), `packet->data`, and `packet->length`.
- `response_packet` -- populate this to send data back. Set `response_packet->command = CMD_OK`, fill `response_packet->data`, and set `response_packet->length`.
- `chunked_payload` / `chunked_payload_length` -- for payloads that arrived via chunking (larger than `PACKET_PAYLOAD_SIZE`), the reassembled data is here. For sending chunked responses, allocate into `*chunked_payload` and set the length.

Return `ESP_OK` on success, or an `esp_err_t` error code on failure.

### 3.3 Handler Examples

**Fire-and-forget (REPLY_NONE)** -- master sends data, expects no response:

```c
esp_err_t handle_led_toggle(data_packet_t *packet, data_packet_t *response_packet,
                            uint8_t **chunked_payload, uint16_t *chunked_payload_length) {
    uint8_t led_state = packet->data[0];
    gpio_set_level(LED_PIN, led_state);
    return ESP_OK;
}
```

**Data response (REPLY_DATA)** -- master expects up to `PACKET_PAYLOAD_SIZE` bytes back:

```c
esp_err_t handle_get_sensor(data_packet_t *packet, data_packet_t *response_packet,
                            uint8_t **chunked_payload, uint16_t *chunked_payload_length) {
    uint16_t reading = read_adc();

    response_packet->command = CMD_OK;
    response_packet->length = 2;
    integer_to_bytes(reading, response_packet->data, 0);  // little-endian

    return ESP_OK;
}
```

**Chunked response (REPLY_CHUNK)** -- master expects a large payload:

```c
esp_err_t handle_get_log(data_packet_t *packet, data_packet_t *response_packet,
                         uint8_t **chunked_payload, uint16_t *chunked_payload_length) {
    const char *log_data = get_stored_log();
    size_t len = strlen(log_data);

    if (*chunked_payload != NULL) free(*chunked_payload);

    *chunked_payload = malloc(len);
    if (*chunked_payload == NULL) return ESP_ERR_NO_MEM;

    memcpy(*chunked_payload, log_data, len);
    *chunked_payload_length = (uint16_t)len;

    return ESP_OK;
}
```

**Receiving chunked data** -- master sent a large payload:

```c
esp_err_t handle_firmware_block(data_packet_t *packet, data_packet_t *response_packet,
                                uint8_t **chunked_payload, uint16_t *chunked_payload_length) {
    // The driver has already reassembled the chunks into chunked_payload
    process_firmware(*chunked_payload, *chunked_payload_length);
    return ESP_OK;
}
```

### 3.4 Starting the Slave

```c
#include "i2c_protocol_shared.h"
#include "i2c_protocol_slave.h"
#include "my_commands.h"

void app_main() {
    uint8_t slave_address = 0x0B;  // 7-bit I2C address

    // 1. Start the driver task
    TaskHandle_t slave_handle;
    xTaskCreatePinnedToCore(
        i2c_slave_task, "i2c_slave",
        I2C_RX_TASK_STACK_SIZE + I2C_TX_TASK_STACK_SIZE + I2C_PROC_TASK_STACK_SIZE,
        &slave_address, I2C_COMMS_TASK_PRIORITY, &slave_handle, 0
    );
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));  // Wait for init

    // 2. Register command handlers (IDs must be 1--200)
    i2c_register_command(&(command_registry_item_t){
        .command = CMD_LED_TOGGLE,
        .process_command = handle_led_toggle
    });
    i2c_register_command(&(command_registry_item_t){
        .command = CMD_GET_SENSOR,
        .process_command = handle_get_sensor
    });

    // 3. Application loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

---

## 4. Master Implementation

### 4.1 Starting the Master

```c
#include "i2c_protocol_shared.h"
#include "i2c_protocol_master.h"

void app_main() {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 7,
    };

    TaskHandle_t master_handle;
    xTaskCreatePinnedToCore(
        i2c_master_task, "i2c_master",
        I2C_RX_TASK_STACK_SIZE + I2C_TX_TASK_STACK_SIZE + I2C_PROC_TASK_STACK_SIZE,
        &bus_config, I2C_COMMS_TASK_PRIORITY, &master_handle, 0
    );
    ulTaskNotifyTake(false, pdMS_TO_TICKS(100));
}
```

### 4.2 Sending Requests

All master communication uses `i2c_make_request()`:

```c
esp_err_t i2c_make_request(
    i2c_command_t command,        // Command byte (1--200)
    uint8_t *payload,             // Data to send (NULL if none)
    uint32_t payload_length,      // Byte count
    uint8_t slave_id,             // 7-bit slave address
    reply_type_t reply_type,      // Expected reply type
    uint8_t **response_data,      // [out] heap-allocated response (caller frees)
    uint32_t *response_length     // [out] response byte count
);
```

**Reply types:**

| Type | Use When | response_data |
|---|---|---|
| `REPLY_NONE` | Fire-and-forget; no response expected | NULL |
| `REPLY_OK` | Wait for acknowledgement only, no data back | NULL |
| `REPLY_DATA` | Expect up to `PACKET_PAYLOAD_SIZE` bytes back | Caller must `free()` |
| `REPLY_CHUNK` | Expect a large (chunked) response | Caller must `free()` |

### 4.3 Request Examples

**Fire-and-forget:**

```c
uint8_t led_on = 1;
i2c_make_request(CMD_LED_TOGGLE, &led_on, 1, 0x0B, REPLY_NONE, NULL, NULL);
```

**Request with data response:**

```c
uint8_t *response = NULL;
uint32_t response_len = 0;

esp_err_t err = i2c_make_request(
    CMD_GET_SENSOR, NULL, 0, 0x0B,
    REPLY_DATA, &response, &response_len
);

if (err == ESP_OK && response != NULL) {
    uint16_t reading = bytes_to_integer(response, 0);
    printf("Sensor: %u\n", reading);
    free(response);
}
```

**Send large payload (auto-chunked):**

```c
uint8_t config_blob[2048];
// ... fill config_blob ...

i2c_make_request(
    CMD_SEND_CONFIG, config_blob, sizeof(config_blob), 0x0B,
    REPLY_OK, NULL, NULL
);
```

**Request large response (chunked):**

```c
uint8_t *log_data = NULL;
uint32_t log_len = 0;

esp_err_t err = i2c_make_request(
    CMD_GET_LOG, NULL, 0, 0x0B,
    REPLY_CHUNK, &log_data, &log_len
);

if (err == ESP_OK && log_data != NULL) {
    printf("Got %lu bytes of log data\n", log_len);
    free(log_data);
}
```

---

## 5. Reserved Command IDs

User commands must use IDs **1--200**. IDs 201 and above are reserved by the driver (`CMD_RESERVED_MIN`):

| ID | Name | Purpose |
|---|---|---|
| 250 | `CMD_START_CHUNKING` | Begins a chunked transfer |
| 251 | `CMD_END_CHUNKING` | Ends a chunked transfer |
| 252 | `CMD_RESTART` | Requests retransmission |
| 254 | `CMD_ERR` | Error response |
| 255 | `CMD_OK` | Success / acknowledgement |

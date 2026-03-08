set(srcs "src/i2c_protocol_shared.c")  # Add shared sources if any

if(CONFIG_I2C_PROTOCOL_MASTER)
    list(APPEND srcs "src/i2c_protocol_master.c")
elseif(CONFIG_I2C_PROTOCOL_SLAVE)
    list(APPEND srcs "src/i2c_protocol_slave.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_timer
    REQUIRES "driver"
)
